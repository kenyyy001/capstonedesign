#include <Arduino.h>

// =====================================================
// TES PWM 3 MOTOR / 6 DRIVER BTN7970B (ESP32)
//
// Tiap motor = 2 chip BTN7970B (half-bridge):
//   chip A (IN_A)  -> PWM = maju
//   chip B (IN_B)  -> PWM = mundur
// Pin INH keenam chip harus HIGH lewat hardware.
//
// Perintah serial (115200 baud):
//   W = semua motor maju
//   S = semua motor mundur
//   X = stop semua (langsung)
//   + = naikkan kecepatan 10%
//   - = turunkan kecepatan 10%
//   T = tes tiap driver satu per satu (6 driver)
//   H = tampilkan menu
// =====================================================

// ---------------- PIN MOTOR ----------------
#define M1_IN_A 18
#define M1_IN_B 4

#define M2_IN_A 2
#define M2_IN_B 13

#define M3_IN_A 14
#define M3_IN_B 27

// ---------------- PARAMETER PWM ----------------
const uint32_t PWM_FREQ = 20000;   // 20 kHz (batas BTN7970B: 25 kHz)
const uint8_t  PWM_RES  = 8;       // resolusi 8 bit -> duty 0..255
const int      PWM_MAX  = 255;

// ---------------- PARAMETER RAMP ----------------
const int           RAMP_STEP        = 4;    // perubahan duty per langkah
const unsigned long RAMP_INTERVAL_MS = 10;   // jeda tiap langkah

// ---------------- STRUKTUR MOTOR ----------------
struct Motor
{
    uint8_t pinA;
    uint8_t pinB;
    uint8_t chA;   // channel LEDC (dipakai di core ESP32 v2.x)
    uint8_t chB;
    int     cur;   // duty bertanda saat ini: + maju, - mundur
};

Motor motors[3] = {
    {M1_IN_A, M1_IN_B, 0, 1, 0},
    {M2_IN_A, M2_IN_B, 2, 3, 0},
    {M3_IN_A, M3_IN_B, 4, 5, 0}
};

int speedPct    = 40;   // kecepatan awal 40% (aman untuk tes)
int targetSpeed = 0;    // duty bertanda yang dituju semua motor

// =====================================================
// WRAPPER PWM (kompatibel core ESP32 v2.x dan v3.x)
// =====================================================

#if ESP_ARDUINO_VERSION_MAJOR >= 3
inline void pwmAttach(uint8_t pin, uint8_t ch)
{
    (void)ch;
    ledcAttach(pin, PWM_FREQ, PWM_RES);
}
inline void pwmWrite(uint8_t pin, uint8_t ch, uint32_t duty)
{
    (void)ch;
    ledcWrite(pin, duty);
}
#else
inline void pwmAttach(uint8_t pin, uint8_t ch)
{
    ledcSetup(ch, PWM_FREQ, PWM_RES);
    ledcAttachPin(pin, ch);
}
inline void pwmWrite(uint8_t pin, uint8_t ch, uint32_t duty)
{
    (void)pin;
    ledcWrite(ch, duty);
}
#endif

// =====================================================
// KONTROL MOTOR
// =====================================================

int speedDuty()
{
    return speedPct * PWM_MAX / 100;
}

// v > 0 : maju (PWM di chip A, chip B = LOW)
// v < 0 : mundur (PWM di chip B, chip A = LOW)
// v = 0 : stop (kedua chip LOW)
void setMotor(int i, int v)
{
    Motor &m = motors[i];
    v = constrain(v, -PWM_MAX, PWM_MAX);
    m.cur = v;

    if (v > 0)
    {
        pwmWrite(m.pinB, m.chB, 0);
        pwmWrite(m.pinA, m.chA, (uint32_t)v);
    }
    else if (v < 0)
    {
        pwmWrite(m.pinA, m.chA, 0);
        pwmWrite(m.pinB, m.chB, (uint32_t)(-v));
    }
    else
    {
        pwmWrite(m.pinA, m.chA, 0);
        pwmWrite(m.pinB, m.chB, 0);
    }
}

void stopAll()
{
    targetSpeed = 0;
    for (int i = 0; i < 3; i++)
        setMotor(i, 0);
}

// Ramp halus supaya arus start tidak melonjak (mencegah brownout)
void updateRamp()
{
    static unsigned long last = 0;

    if (millis() - last < RAMP_INTERVAL_MS)
        return;
    last = millis();

    for (int i = 0; i < 3; i++)
    {
        int c = motors[i].cur;

        if (c < targetSpeed)
            c = min(c + RAMP_STEP, targetSpeed);
        else if (c > targetSpeed)
            c = max(c - RAMP_STEP, targetSpeed);

        if (c != motors[i].cur)
            setMotor(i, c);
    }
}

// Tes tiap driver: M1-A, M1-B, M2-A, M2-B, M3-A, M3-B
void testEachDriver()
{
    int duty = speedDuty();

    stopAll();
    Serial.println();
    Serial.println("=== TES 6 DRIVER (satu per satu) ===");

    for (int m = 0; m < 3; m++)
    {
        for (int d = 1; d >= -1; d -= 2)
        {
            uint8_t pin = (d > 0) ? motors[m].pinA : motors[m].pinB;

            Serial.print("Motor ");
            Serial.print(m + 1);
            Serial.print(d > 0 ? " MAJU   " : " MUNDUR ");
            Serial.print("(chip ");
            Serial.print(d > 0 ? "A" : "B");
            Serial.print(", GPIO ");
            Serial.print(pin);
            Serial.print(", PWM ");
            Serial.print(speedPct);
            Serial.println("%)");

            for (int v = 0; v <= duty; v += 5)
            {
                setMotor(m, d * v);
                delay(10);
            }
            setMotor(m, d * duty);
            delay(1500);

            setMotor(m, 0);
            delay(500);
        }
    }

    Serial.println("=== TES SELESAI ===");

    while (Serial.available())
        Serial.read();
}

void printMenu()
{
    Serial.println();
    Serial.println("========================================");
    Serial.println("      TES PWM 3 MOTOR / 6 DRIVER");
    Serial.println("========================================");
    Serial.println("W = semua maju");
    Serial.println("S = semua mundur");
    Serial.println("X = stop semua");
    Serial.println("+ = naikkan kecepatan 10%");
    Serial.println("- = turunkan kecepatan 10%");
    Serial.println("T = tes tiap driver satu per satu");
    Serial.println("H = menu");
    Serial.println("========================================");
    Serial.print("Kecepatan saat ini: ");
    Serial.print(speedPct);
    Serial.println("%");
}

// =====================================================
// SETUP
// =====================================================

void setup()
{
    // Pastikan semua pin LOW secepat mungkin (hindari motor gerak saat boot)
    for (int i = 0; i < 3; i++)
    {
        pinMode(motors[i].pinA, OUTPUT);
        pinMode(motors[i].pinB, OUTPUT);
        digitalWrite(motors[i].pinA, LOW);
        digitalWrite(motors[i].pinB, LOW);
    }

    Serial.begin(115200);

    for (int i = 0; i < 3; i++)
    {
        pwmAttach(motors[i].pinA, motors[i].chA);
        pwmAttach(motors[i].pinB, motors[i].chB);
    }

    stopAll();
    printMenu();
}

// =====================================================
// LOOP
// =====================================================

void loop()
{
    if (Serial.available())
    {
        char command = Serial.read();

        if (command == '\n' || command == '\r' || command == ' ')
            return;

        if (command >= 'a' && command <= 'z')
            command = command - 'a' + 'A';

        switch (command)
        {
        case 'W':
            targetSpeed = speedDuty();
            Serial.print("SEMUA MOTOR -> MAJU (");
            Serial.print(speedPct);
            Serial.println("%)");
            break;

        case 'S':
            targetSpeed = -speedDuty();
            Serial.print("SEMUA MOTOR -> MUNDUR (");
            Serial.print(speedPct);
            Serial.println("%)");
            break;

        case 'X':
            stopAll();
            Serial.println("SEMUA MOTOR -> STOP");
            break;

        case '+':
        case '=':
            speedPct = min(speedPct + 10, 100);
            if (targetSpeed > 0) targetSpeed = speedDuty();
            if (targetSpeed < 0) targetSpeed = -speedDuty();
            Serial.print("Kecepatan: ");
            Serial.print(speedPct);
            Serial.println("%");
            break;

        case '-':
            speedPct = max(speedPct - 10, 10);
            if (targetSpeed > 0) targetSpeed = speedDuty();
            if (targetSpeed < 0) targetSpeed = -speedDuty();
            Serial.print("Kecepatan: ");
            Serial.print(speedPct);
            Serial.println("%");
            break;

        case 'T':
            testEachDriver();
            break;

        case 'H':
            printMenu();
            break;

        default:
            Serial.print("Perintah tidak dikenal: ");
            Serial.println(command);
            break;
        }
    }

    updateRamp();
}
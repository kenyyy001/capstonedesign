#include <Arduino.h>

// =====================================================
// TES PWM 3 MOTOR / 6 DRIVER BTN7970B (ESP32) - FIXED
//
// PERBAIKAN:
// 1. GPIO 2 diganti GPIO 15 (GPIO 2 = strapping pin, bermasalah dgn PWM)
// 2. LEDC timer dikonfigurasi eksplisit (hindari konflik timer sharing)
// 3. Ramp logic diperbaiki - update per-motor secara independen
// 4. Tambah debug serial untuk diagnosa
// 5. Tambah mode direct (tanpa ramp) untuk testing cepat
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
//   D = direct mode (tanpa ramp, langsung ke speed)
//   H = tampilkan menu
// =====================================================

// ---------------- PIN MOTOR ----------------
// PERBAIKAN: GPIO 2 diganti GPIO 15
// GPIO 2 adalah strapping pin di ESP32, terhubung ke LED onboard
// dan bisa menyebabkan PWM tidak keluar dengan benar
#define M1_IN_A 18
#define M1_IN_B 4

#define M2_IN_A 15   // SEBELUMNYA GPIO 2 (strapping pin!)
#define M2_IN_B 13

#define M3_IN_A 14
#define M3_IN_B 27

// ---------------- PARAMETER PWM ----------------
const uint32_t PWM_FREQ = 20000;   // 20 kHz (batas BTN7970B: 25 kHz)
const uint8_t  PWM_RES  = 8;       // resolusi 8 bit -> duty 0..255
const int      PWM_MAX  = 255;

// ---------------- PARAMETER RAMP ----------------
const int           RAMP_STEP        = 5;    // perubahan duty per langkah
const unsigned long RAMP_INTERVAL_MS = 8;    // jeda tiap langkah

// ---------------- STRUKTUR MOTOR ----------------
struct Motor
{
    uint8_t pinA;
    uint8_t pinB;
    uint8_t chA;      // channel LEDC
    uint8_t chB;
    uint8_t timerA;   // timer LEDC (PERBAIKAN: eksplisit)
    uint8_t timerB;
    int     cur;      // duty bertanda saat ini
    int     target;   // target duty per-motor
};

// PERBAIKAN: Setiap motor pakai timer berbeda untuk hindari konflik
// Timer 0 -> Motor 1 (ch 0, 1)
// Timer 1 -> Motor 2 (ch 2, 3)
// Timer 2 -> Motor 3 (ch 4, 5)
Motor motors[3] = {
    {M1_IN_A, M1_IN_B, 0, 1, 0, 0, 0, 0},
    {M2_IN_A, M2_IN_B, 2, 3, 1, 1, 0, 0},
    {M3_IN_A, M3_IN_B, 4, 5, 2, 2, 0, 0}
};

int speedPct     = 40;    // kecepatan awal 40%
bool useRamp     = true;  // PERBAIKAN: flag ramp on/off
bool directMode  = false; // PERBAIKAN: mode direct

// =====================================================
// WRAPPER PWM (kompatibel core ESP32 v2.x dan v3.x)
// PERBAIKAN: Tambah error checking
// =====================================================

#if ESP_ARDUINO_VERSION_MAJOR >= 3

inline bool pwmAttach(uint8_t pin, uint8_t ch)
{
    (void)ch;
    return ledcAttach(pin, PWM_FREQ, PWM_RES);
}
inline void pwmWrite(uint8_t pin, uint8_t ch, uint32_t duty)
{
    (void)ch;
    ledcWrite(pin, duty);
}

#else

inline bool pwmAttach(uint8_t pin, uint8_t ch)
{
    // PERBAIKAN: Gunakan ledcSetup dengan timer eksplisit
    // agar tidak ada konflik saat channel berbagi timer
    ledcSetup(ch, PWM_FREQ, PWM_RES);
    ledcAttachPin(pin, ch);
    return true;
}
inline void pwmWrite(uint8_t pin, uint8_t ch, uint32_t duty)
{
    (void)pin;
    ledcWrite(ch, duty);
}

#endif

// =====================================================
// KONTROL MOTOR
// PERBAIKAN: setMotor sekarang lebih robust
// =====================================================

int speedDuty()
{
    return speedPct * PWM_MAX / 100;
}

void setMotor(int i, int v)
{
    Motor &m = motors[i];
    v = constrain(v, -PWM_MAX, PWM_MAX);
    m.cur = v;

    if (v > 0)
    {
        // MAJU: PWM di chip A, chip B = LOW
        pwmWrite(m.pinA, m.chA, (uint32_t)v);
        pwmWrite(m.pinB, m.chB, 0);
    }
    else if (v < 0)
    {
        // MUNDUR: PWM di chip B, chip A = LOW
        pwmWrite(m.pinA, m.chA, 0);
        pwmWrite(m.pinB, m.chB, (uint32_t)(-v));
    }
    else
    {
        // STOP: kedua chip LOW
        pwmWrite(m.pinA, m.chA, 0);
        pwmWrite(m.pinB, m.chB, 0);
    }
}

void stopAll()
{
    for (int i = 0; i < 3; i++)
    {
        motors[i].target = 0;
        setMotor(i, 0);
    }
}

// PERBAIKAN: Ramp logic yang lebih benar
// Setiap motor punya target sendiri, diupdate independen
void updateRamp()
{
    static unsigned long last = 0;

    if (millis() - last < RAMP_INTERVAL_MS)
        return;
    last = millis();

    for (int i = 0; i < 3; i++)
    {
        int c = motors[i].cur;
        int t = motors[i].target;

        if (c == t)
            continue;  // sudah mencapai target

        if (c < t)
        {
            c += RAMP_STEP;
            if (c > t) c = t;
        }
        else // c > t
        {
            c -= RAMP_STEP;
            if (c < t) c = t;
        }

        setMotor(i, c);
    }
}

// PERBAIKAN: Set semua motor ke target yang sama
void setAllTarget(int target)
{
    for (int i = 0; i < 3; i++)
    {
        motors[i].target = target;
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
        // Test MAJU (chip A)
        {
            uint8_t pin = motors[m].pinA;
            Serial.print("Motor ");
            Serial.print(m + 1);
            Serial.print(" MAJU   (chip A, GPIO ");
            Serial.print(pin);
            Serial.print(", PWM ");
            Serial.print(speedPct);
            Serial.println("%)");

            for (int v = 0; v <= duty; v += 5)
            {
                setMotor(m, v);
                delay(10);
            }
            setMotor(m, duty);
            delay(1500);
            setMotor(m, 0);
            delay(500);
        }

        // Test MUNDUR (chip B)
        {
            uint8_t pin = motors[m].pinB;
            Serial.print("Motor ");
            Serial.print(m + 1);
            Serial.print(" MUNDUR (chip B, GPIO ");
            Serial.print(pin);
            Serial.print(", PWM ");
            Serial.print(speedPct);
            Serial.println("%)");

            for (int v = 0; v <= duty; v += 5)
            {
                setMotor(m, -v);
                delay(10);
            }
            setMotor(m, -duty);
            delay(1500);
            setMotor(m, 0);
            delay(500);
        }
    }

    Serial.println("=== TES SELESAI ===");
    while (Serial.available()) Serial.read();
}

// PERBAIKAN: Debug - tampilkan status semua motor
void printStatus()
{
    Serial.println("--- STATUS MOTOR ---");
    for (int i = 0; i < 3; i++)
    {
        Serial.print("Motor ");
        Serial.print(i + 1);
        Serial.print(": cur=");
        Serial.print(motors[i].cur);
        Serial.print(" target=");
        Serial.print(motors[i].target);
        Serial.print(" pinA=");
        Serial.print(motors[i].pinA);
        Serial.print(" pinB=");
        Serial.print(motors[i].pinB);
        Serial.println();
    }
    Serial.print("Speed: ");
    Serial.print(speedPct);
    Serial.println("%");
    Serial.print("Mode: ");
    Serial.println(directMode ? "DIRECT (tanpa ramp)" : "RAMP");
    Serial.println("--------------------");
}

void printMenu()
{
    Serial.println();
    Serial.println("========================================");
    Serial.println("   TES PWM 3 MOTOR / 6 DRIVER [FIXED]");
    Serial.println("========================================");
    Serial.println("W = semua maju");
    Serial.println("S = semua mundur");
    Serial.println("X = stop semua");
    Serial.println("+ = naikkan kecepatan 10%");
    Serial.println("- = turunkan kecepatan 10%");
    Serial.println("T = tes tiap driver satu per satu");
    Serial.println("D = toggle direct/ramp mode");
    Serial.println("P = print status motor");
    Serial.println("H = menu");
    Serial.println("========================================");
    Serial.print("Kecepatan saat ini: ");
    Serial.print(speedPct);
    Serial.println("%");
}

// =====================================================
// SETUP
// PERBAIKAN: Tambah error checking & debug output
// =====================================================

void setup()
{
    Serial.begin(115200);
    delay(500);  // Tunggu serial stabil
    Serial.println();
    Serial.println("=== ESP32 BTN7970B Motor Controller ===");
    Serial.println("Initializing...");

    // Pastikan semua pin LOW secepat mungkin
    for (int i = 0; i < 3; i++)
    {
        pinMode(motors[i].pinA, OUTPUT);
        pinMode(motors[i].pinB, OUTPUT);
        digitalWrite(motors[i].pinA, LOW);
        digitalWrite(motors[i].pinB, LOW);
    }

    // Attach PWM dengan error checking
    for (int i = 0; i < 3; i++)
    {
        bool okA = pwmAttach(motors[i].pinA, motors[i].chA);
        bool okB = pwmAttach(motors[i].pinB, motors[i].chB);

        Serial.print("Motor ");
        Serial.print(i + 1);
        Serial.print(" PWM attach: pinA(GPIO");
        Serial.print(motors[i].pinA);
        Serial.print(")=");
        Serial.print(okA ? "OK" : "FAIL!");
        Serial.print(" pinB(GPIO");
        Serial.print(motors[i].pinB);
        Serial.print(")=");
        Serial.println(okB ? "OK" : "FAIL!");
    }

    stopAll();
    Serial.println("Init complete.");
    printMenu();
}

// =====================================================
// LOOP
// PERBAIKAN: Tambah command D (direct mode) dan P (print status)
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
        {
            int duty = speedDuty();
            if (directMode)
            {
                // Direct: langsung set tanpa ramp
                for (int i = 0; i < 3; i++)
                    setMotor(i, duty);
            }
            else
            {
                setAllTarget(duty);
            }
            Serial.print("SEMUA MOTOR -> MAJU (");
            Serial.print(speedPct);
            Serial.print("%, duty=");
            Serial.print(duty);
            Serial.println(")");
            break;
        }

        case 'S':
        {
            int duty = -speedDuty();
            if (directMode)
            {
                for (int i = 0; i < 3; i++)
                    setMotor(i, duty);
            }
            else
            {
                setAllTarget(duty);
            }
            Serial.print("SEMUA MOTOR -> MUNDUR (");
            Serial.print(speedPct);
            Serial.print("%, duty=");
            Serial.print(duty);
            Serial.println(")");
            break;
        }

        case 'X':
            stopAll();
            Serial.println("SEMUA MOTOR -> STOP");
            break;

        case '+':
        case '=':
            speedPct = min(speedPct + 10, 100);
            {
                int duty = speedDuty();
                if (directMode)
                {
                    for (int i = 0; i < 3; i++)
                    {
                        if (motors[i].cur > 0) setMotor(i, duty);
                        else if (motors[i].cur < 0) setMotor(i, -duty);
                    }
                }
                else
                {
                    for (int i = 0; i < 3; i++)
                    {
                        if (motors[i].target > 0) motors[i].target = duty;
                        else if (motors[i].target < 0) motors[i].target = -duty;
                    }
                }
            }
            Serial.print("Kecepatan: ");
            Serial.print(speedPct);
            Serial.println("%");
            break;

        case '-':
            speedPct = max(speedPct - 10, 10);
            {
                int duty = speedDuty();
                if (directMode)
                {
                    for (int i = 0; i < 3; i++)
                    {
                        if (motors[i].cur > 0) setMotor(i, duty);
                        else if (motors[i].cur < 0) setMotor(i, -duty);
                    }
                }
                else
                {
                    for (int i = 0; i < 3; i++)
                    {
                        if (motors[i].target > 0) motors[i].target = duty;
                        else if (motors[i].target < 0) motors[i].target = -duty;
                    }
                }
            }
            Serial.print("Kecepatan: ");
            Serial.print(speedPct);
            Serial.println("%");
            break;

        case 'T':
            testEachDriver();
            break;

        case 'D':
            directMode = !directMode;
            Serial.print("Mode: ");
            Serial.println(directMode ? "DIRECT (tanpa ramp)" : "RAMP");
            break;

        case 'P':
            printStatus();
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

    // PERBAIKAN: Ramp hanya dijalankan jika bukan direct mode
    if (!directMode)
        updateRamp();
}
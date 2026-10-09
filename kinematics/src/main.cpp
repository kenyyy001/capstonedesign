#include <Arduino.h>

// =====================================================
// PIN MOTOR (driver H-bridge: IN_A / IN_B)
// =====================================================

// MOTOR 1
#define M1_IN_A 18
#define M1_IN_B 4

// MOTOR 2
#define M2_IN_A 2
#define M2_IN_B 13

// MOTOR 3
#define M3_IN_A 14
#define M3_IN_B 27

// =====================================================
// PIN ENCODER
// GPIO 34, 35, 36, 39 = input-only, tanpa pull-up internal
// Sinyal encoder harus 3,3 V (jangan 5 V langsung)
// =====================================================

// MOTOR 1
#define ENC1_A 36
#define ENC1_B 39

// MOTOR 2
#define ENC2_A 34
#define ENC2_B 35

// MOTOR 3
#define ENC3_A 32
#define ENC3_B 33

// =====================================================
// ENCODER
// =====================================================

volatile long enc1 = 0;
volatile long enc2 = 0;
volatile long enc3 = 0;

// Decoding pada setiap perubahan kanal A (x2).
// Tanda (+/-) hanya relatif: yang penting nilainya berubah
// dan arahnya konsisten saat motor maju / mundur.
void IRAM_ATTR isr1()
{
    enc1 += (digitalRead(ENC1_A) != digitalRead(ENC1_B)) ? 1 : -1;
}

void IRAM_ATTR isr2()
{
    enc2 += (digitalRead(ENC2_A) != digitalRead(ENC2_B)) ? 1 : -1;
}

void IRAM_ATTR isr3()
{
    enc3 += (digitalRead(ENC3_A) != digitalRead(ENC3_B)) ? 1 : -1;
}

void resetEncoders()
{
    noInterrupts();
    enc1 = 0;
    enc2 = 0;
    enc3 = 0;
    interrupts();
}

// =====================================================
// MOTOR 1
// =====================================================

void motor1Forward()
{
    digitalWrite(M1_IN_A, HIGH);
    digitalWrite(M1_IN_B, LOW);
}

void motor1Backward()
{
    digitalWrite(M1_IN_A, LOW);
    digitalWrite(M1_IN_B, HIGH);
}

void motor1Stop()
{
    digitalWrite(M1_IN_A, LOW);
    digitalWrite(M1_IN_B, LOW);
}

// =====================================================
// MOTOR 2
// =====================================================

void motor2Forward()
{
    digitalWrite(M2_IN_A, HIGH);
    digitalWrite(M2_IN_B, LOW);
}

void motor2Backward()
{
    digitalWrite(M2_IN_A, LOW);
    digitalWrite(M2_IN_B, HIGH);
}

void motor2Stop()
{
    digitalWrite(M2_IN_A, LOW);
    digitalWrite(M2_IN_B, LOW);
}

// =====================================================
// MOTOR 3
// =====================================================

void motor3Forward()
{
    digitalWrite(M3_IN_A, HIGH);
    digitalWrite(M3_IN_B, LOW);
}

void motor3Backward()
{
    digitalWrite(M3_IN_A, LOW);
    digitalWrite(M3_IN_B, HIGH);
}

void motor3Stop()
{
    digitalWrite(M3_IN_A, LOW);
    digitalWrite(M3_IN_B, LOW);
}

// =====================================================
// SEMUA MOTOR
// =====================================================

void allForward()
{
    motor1Forward();
    motor2Forward();
    motor3Forward();
}

void allBackward()
{
    motor1Backward();
    motor2Backward();
    motor3Backward();
}

void allStop()
{
    motor1Stop();
    motor2Stop();
    motor3Stop();
}

// =====================================================
// KONTROL BERDASARKAN MOTOR TERPILIH
// selectedMotor: 0 = semua motor, 1/2/3 = motor individual
// =====================================================

int selectedMotor = 0;

void motorForward(int m)
{
    if (m == 0) allForward();
    else if (m == 1) motor1Forward();
    else if (m == 2) motor2Forward();
    else if (m == 3) motor3Forward();
}

void motorBackward(int m)
{
    if (m == 0) allBackward();
    else if (m == 1) motor1Backward();
    else if (m == 2) motor2Backward();
    else if (m == 3) motor3Backward();
}

void motorStop(int m)
{
    if (m == 0) allStop();
    else if (m == 1) motor1Stop();
    else if (m == 2) motor2Stop();
    else if (m == 3) motor3Stop();
}

void printTarget()
{
    if (selectedMotor == 0)
        Serial.print("SEMUA MOTOR");
    else
    {
        Serial.print("Motor ");
        Serial.print(selectedMotor);
    }
}

void printMenu()
{
    Serial.println();
    Serial.println("========================================");
    Serial.println("   TEST 3 MOTOR OMNI + ENCODER");
    Serial.println("========================================");
    Serial.println("0 = pilih SEMUA motor");
    Serial.println("1 = pilih Motor 1");
    Serial.println("2 = pilih Motor 2");
    Serial.println("3 = pilih Motor 3");
    Serial.println();
    Serial.println("W = maju (motor terpilih)");
    Serial.println("S = mundur (motor terpilih)");
    Serial.println("X = stop (motor terpilih)");
    Serial.println("Z = STOP SEMUA motor (darurat)");
    Serial.println("E = reset hitungan encoder");
    Serial.println("H = tampilkan menu ini");
    Serial.println("========================================");
    Serial.print("Target saat ini: ");
    printTarget();
    Serial.println();
}

// =====================================================
// SETUP
// =====================================================

void setup()
{
    Serial.begin(115200);

    // Motor
    pinMode(M1_IN_A, OUTPUT);
    pinMode(M1_IN_B, OUTPUT);
    pinMode(M2_IN_A, OUTPUT);
    pinMode(M2_IN_B, OUTPUT);
    pinMode(M3_IN_A, OUTPUT);
    pinMode(M3_IN_B, OUTPUT);

    // Kondisi awal
    allStop();

    // Encoder
    // GPIO 34-39 tidak punya pull-up internal: pakai pull-up eksternal
    // (mis. 10k ke 3,3 V) jika encoder bertipe open-collector.
    pinMode(ENC1_A, INPUT);
    pinMode(ENC1_B, INPUT);
    pinMode(ENC2_A, INPUT);
    pinMode(ENC2_B, INPUT);
    pinMode(ENC3_A, INPUT_PULLUP);
    pinMode(ENC3_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENC1_A), isr1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC2_A), isr2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC3_A), isr3, CHANGE);

    printMenu();
}

// =====================================================
// LOOP
// =====================================================

void loop()
{
    // ---------------------------------------------
    // PERINTAH SERIAL
    // ---------------------------------------------
    if (Serial.available())
    {
        char command = Serial.read();

        // Abaikan Enter / newline / spasi
        if (command == '\n' || command == '\r' || command == ' ')
            return;

        if (command >= 'a' && command <= 'z')
            command = command - 'a' + 'A';

        switch (command)
        {
        case '0':
        case '1':
        case '2':
        case '3':
            // Hentikan semua dulu agar tes mulai dari kondisi diam
            allStop();
            selectedMotor = command - '0';
            Serial.print("Target: ");
            printTarget();
            Serial.println("  (semua motor di-STOP)");
            break;

        case 'W':
            motorForward(selectedMotor);
            printTarget();
            Serial.println(" -> MAJU");
            break;

        case 'S':
            motorBackward(selectedMotor);
            printTarget();
            Serial.println(" -> MUNDUR");
            break;

        case 'X':
            motorStop(selectedMotor);
            printTarget();
            Serial.println(" -> STOP");
            break;

        case 'Z':
            allStop();
            Serial.println("SEMUA MOTOR -> STOP (darurat)");
            break;

        case 'E':
            resetEncoders();
            Serial.println("Encoder di-reset ke 0");
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

    // ---------------------------------------------
    // CETAK NILAI ENCODER SETIAP 500 ms
    // ---------------------------------------------
    static unsigned long lastPrint = 0;

    if (millis() - lastPrint >= 500)
    {
        lastPrint = millis();

        long e1, e2, e3;

        noInterrupts();
        e1 = enc1;
        e2 = enc2;
        e3 = enc3;
        interrupts();

        Serial.printf("ENC1=%ld  ENC2=%ld  ENC3=%ld\n", e1, e2, e3);
    }
}
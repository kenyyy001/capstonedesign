#include <Arduino.h>

// ===================== PIN =====================
// Motor 1 = D23 & D19 | Motor 2 = D18 & D4 | Motor 3 = D2 & D13
const uint8_t IN_A[3] = {23, 18, 13};
const uint8_t IN_B[3] = {19,  4, 2};
const int N = 3;

// Motor yang sedang dipilih: -1 = semua, 0..2 = motor 1..3
int selected = -1;

// ===================== DRIVER =====================
void motorForward(int i)  { digitalWrite(IN_A[i], HIGH); digitalWrite(IN_B[i], LOW);  }
void motorBackward(int i) { digitalWrite(IN_A[i], LOW);  digitalWrite(IN_B[i], HIGH); }
void motorStop(int i)     { digitalWrite(IN_A[i], LOW);  digitalWrite(IN_B[i], LOW);  }

void stopAll() { for (int i = 0; i < N; i++) motorStop(i); }

// Jeda singkat saat balik arah supaya arus tidak melonjak
void run(bool forward)
{
    for (int i = 0; i < N; i++) {
        if (selected == -1 || selected == i) motorStop(i);
    }
    delay(100);
    for (int i = 0; i < N; i++) {
        if (selected == -1 || selected == i) {
            if (forward) motorForward(i); else motorBackward(i);
        }
    }
}

void printTarget()
{
    if (selected == -1) Serial.println("Target: SEMUA MOTOR");
    else Serial.printf("Target: MOTOR %d\n", selected + 1);
}

// ===================== SETUP =====================
void setup()
{
    Serial.begin(115200);

    for (int i = 0; i < N; i++) {
        pinMode(IN_A[i], OUTPUT);
        pinMode(IN_B[i], OUTPUT);
    }
    stopAll();

    Serial.println();
    Serial.println("=== TEST 3 MOTOR OMNI ===");
    Serial.println("w = maju | s = mundur | x = stop semua");
    Serial.println("1 / 2 / 3 = pilih motor | a = semua motor");
    printTarget();
}

// ===================== LOOP =====================
void loop()
{
    if (!Serial.available()) return;

    char c = tolower(Serial.read());

    switch (c) {
        case 'w': run(true);  Serial.println("MAJU");   break;
        case 's': run(false); Serial.println("MUNDUR"); break;
        case 'x': stopAll();  Serial.println("STOP");   break;
        case 'a': stopAll(); selected = -1; printTarget(); break;
        case '1': case '2': case '3':
            stopAll(); selected = c - '1'; printTarget(); break;
        default: break;   // abaikan \r \n
    }
}
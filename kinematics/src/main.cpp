#include <Arduino.h>

// Tombol 1..6 -> GPIO
const uint8_t PINS[6] = {23, 19, 18, 4, 2, 13};
const int NP = 6;

void allLow() { for (int i = 0; i < NP; i++) digitalWrite(PINS[i], LOW); }

void setup()
{
    Serial.begin(115200);
    for (int i = 0; i < NP; i++) pinMode(PINS[i], OUTPUT);
    allLow();
    Serial.println("\n1=GPIO23 2=GPIO19 3=GPIO18 4=GPIO4 5=GPIO2 6=GPIO13");
    Serial.println("Tekan 1-6 = tahan satu GPIO HIGH, 0 = semua LOW");
}

void loop()
{
    if (!Serial.available()) return;
    char c = Serial.read();

    if (c == '0') {
        allLow();
        Serial.println("Semua LOW");
    } else if (c >= '1' && c <= '6') {
        allLow();
        int i = c - '1';
        digitalWrite(PINS[i], HIGH);
        Serial.printf("GPIO %d = HIGH\n", PINS[i]);
    }
}
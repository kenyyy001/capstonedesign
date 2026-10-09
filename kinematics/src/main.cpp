#include <Arduino.h>

// =====================================================
// MOTOR 1
// =====================================================

#define M1_IN_A 18
#define M1_IN_B 4

// =====================================================
// MOTOR 2
// =====================================================

#define M2_IN_A 2
#define M2_IN_B 13

// =====================================================
// MOTOR 3
// =====================================================

#define M3_IN_A 14
#define M3_IN_B 27


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
// SETUP
// =====================================================

void setup()
{
    Serial.begin(115200);

    // Motor 1
    pinMode(M1_IN_A, OUTPUT);
    pinMode(M1_IN_B, OUTPUT);

    // Motor 2
    pinMode(M2_IN_A, OUTPUT);
    pinMode(M2_IN_B, OUTPUT);

    // Motor 3
    pinMode(M3_IN_A, OUTPUT);
    pinMode(M3_IN_B, OUTPUT);

    // Kondisi awal
    allStop();

    Serial.println();
    Serial.println("========================================");
    Serial.println("        TEST 3 MOTOR OMNI");
    Serial.println("========================================");

    Serial.println();
    Serial.println("SEMUA MOTOR:");
    Serial.println("W = MAJU");
    Serial.println("S = MUNDUR");
    Serial.println("X = STOP");

    Serial.println();
    Serial.println("MOTOR INDIVIDUAL:");
    Serial.println("1 = pilih Motor 1");
    Serial.println("2 = pilih Motor 2");
    Serial.println("3 = pilih Motor 3");

    Serial.println();
    Serial.println("Setelah memilih motor:");
    Serial.println("W = maju");
    Serial.println("S = mundur");
    Serial.println("X = stop");

    Serial.println("========================================");
}


// =====================================================
// LOOP
// =====================================================

void loop()
{
    if (Serial.available())
    {
        char command = Serial.read();

        // =============================================
        // SEMUA MOTOR
        // =============================================

        if (command == 'w' || command == 'W')
        {
            allForward();

            Serial.println("SEMUA MOTOR -> MAJU");
        }

        else if (command == 's' || command == 'S')
        {
            allBackward();

            Serial.println("SEMUA MOTOR -> MUNDUR");
        }

        else if (command == 'x' || command == 'X')
        {
            allStop();

            Serial.println("SEMUA MOTOR -> STOP");
        }

        // =============================================
        // MOTOR 1
        // =============================================

        else if (command == '1')
        {
            Serial.println("Motor 1 dipilih");
            Serial.println("W = maju | S = mundur | X = stop");
            
            while (Serial.available())
                Serial.read();

            while (true)
            {
                if (Serial.available())
                {
                    char cmd = Serial.read();

                    if (cmd == 'w' || cmd == 'W')
                    {
                        motor1Forward();
                        Serial.println("Motor 1 -> MAJU");
                    }

                    else if (cmd == 's' || cmd == 'S')
                    {
                        motor1Backward();
                        Serial.println("Motor 1 -> MUNDUR");
                    }

                    else if (cmd == 'x' || cmd == 'X')
                    {
                        motor1Stop();
                        Serial.println("Motor 1 -> STOP");
                        break;
                    }
                }
            }
        }

        // =============================================
        // MOTOR 2
        // =============================================

        else if (command == '2')
        {
            Serial.println("Motor 2 dipilih");
            Serial.println("W = maju | S = mundur | X = stop");

            while (Serial.available())
                Serial.read();

            while (true)
            {
                if (Serial.available())
                {
                    char cmd = Serial.read();

                    if (cmd == 'w' || cmd == 'W')
                    {
                        motor2Forward();
                        Serial.println("Motor 2 -> MAJU");
                    }

                    else if (cmd == 's' || cmd == 'S')
                    {
                        motor2Backward();
                        Serial.println("Motor 2 -> MUNDUR");
                    }

                    else if (cmd == 'x' || cmd == 'X')
                    {
                        motor2Stop();
                        Serial.println("Motor 2 -> STOP");
                        break;
                    }
                }
            }
        }

        // =============================================
        // MOTOR 3
        // =============================================

        else if (command == '3')
        {
            Serial.println("Motor 3 dipilih");
            Serial.println("W = maju | S = mundur | X = stop");

            while (Serial.available())
                Serial.read();

            while (true)
            {
                if (Serial.available())
                {
                    char cmd = Serial.read();

                    if (cmd == 'w' || cmd == 'W')
                    {
                        motor3Forward();
                        Serial.println("Motor 3 -> MAJU");
                    }

                    else if (cmd == 's' || cmd == 'S')
                    {
                        motor3Backward();
                        Serial.println("Motor 3 -> MUNDUR");
                    }

                    else if (cmd == 'x' || cmd == 'X')
                    {
                        motor3Stop();
                        Serial.println("Motor 3 -> STOP");
                        break;
                    }
                }
            }
        }
    }
}
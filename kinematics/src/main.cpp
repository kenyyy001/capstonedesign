```cpp
#include <Arduino.h>

// =====================================================
// PIN MOTOR
// =====================================================

// MOTOR 1
#define M1_IN_A 23
#define M1_IN_B 19

// MOTOR 2
#define M2_IN_A 18
#define M2_IN_B 4

// MOTOR 3
#define M3_IN_A 2
#define M3_IN_B 13


// =====================================================
// PIN ENCODER
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
// PWM
// =====================================================

#define PWM_FREQ        5000
#define PWM_RESOLUTION  8

#define M1_CH_A 0
#define M1_CH_B 1

#define M2_CH_A 2
#define M2_CH_B 3

#define M3_CH_A 4
#define M3_CH_B 5


// =====================================================
// ENCODER DAN RODA
// =====================================================

const int PPR = 1640;

const float WHEEL_RADIUS_CM = 2.9;

const float WHEEL_CIRCUMFERENCE =
    2.0 * PI * WHEEL_RADIUS_CM;

const float CM_PER_PULSE =
    WHEEL_CIRCUMFERENCE / PPR;


// =====================================================
// INVERSE KINEMATICS
// MATRKS DARI PROGRAM DOSEN
// =====================================================

double matrix_kecepatan[9] =
{
    -0.3333,  0.5774, 0.0317,
    -0.3333, -0.5774, 0.0317,
     0.6667,  0.0000, 0.0317
};


// =====================================================
// INPUT ROBOT
// =====================================================

double linear_x  = 0.0;
double linear_y  = 0.0;
double angular_z = 0.0;


// =====================================================
// HASIL INVERSE KINEMATICS
// =====================================================

double V1 = 0.0;
double V2 = 0.0;
double V3 = 0.0;


// =====================================================
// PWM DASAR
//
// Ganti sesuai hasil kalibrasi motormu.
// =====================================================

const int BASE_PWM_M1 = 55;
const int BASE_PWM_M2 = 50;
const int BASE_PWM_M3 = 60;


// =====================================================
// KECEPATAN MAKSIMUM TARGET
// =====================================================

const float MAX_WHEEL_SPEED = 10.0;


// =====================================================
// ENCODER COUNTER
// =====================================================

volatile long encoder_count1 = 0;
volatile long encoder_count2 = 0;
volatile long encoder_count3 = 0;


// =====================================================
// KECEPATAN AKTUAL
// =====================================================

float speedM1 = 0.0;
float speedM2 = 0.0;
float speedM3 = 0.0;


// =====================================================
// TARGET KECEPATAN
// =====================================================

float targetM1 = 0.0;
float targetM2 = 0.0;
float targetM3 = 0.0;


// =====================================================
// PID
// =====================================================

struct PIDController
{
    float Kp;
    float Ki;
    float Kd;

    float integral;
    float lastError;

    float output;
};


PIDController pid1 =
{
    2.5,
    0.2,
    0.02,

    0.0,
    0.0,
    0.0
};


PIDController pid2 =
{
    2.5,
    0.2,
    0.02,

    0.0,
    0.0,
    0.0
};


PIDController pid3 =
{
    2.5,
    0.2,
    0.02,

    0.0,
    0.0,
    0.0
};


// =====================================================
// MAKSIMUM KOREKSI PID
// =====================================================

const float PID_LIMIT = 20.0;


// =====================================================
// STATUS ROBOT
// =====================================================

bool robotMoving = false;


// =====================================================
// ENCODER INTERRUPT
// =====================================================

void IRAM_ATTR encoderISR1()
{
    int A = digitalRead(ENC1_A);
    int B = digitalRead(ENC1_B);

    if ((A == HIGH) != (B == LOW))
        encoder_count1--;
    else
        encoder_count1++;
}


void IRAM_ATTR encoderISR2()
{
    int A = digitalRead(ENC2_A);
    int B = digitalRead(ENC2_B);

    if ((A == HIGH) != (B == LOW))
        encoder_count2--;
    else
        encoder_count2++;
}


void IRAM_ATTR encoderISR3()
{
    int A = digitalRead(ENC3_A);
    int B = digitalRead(ENC3_B);

    if ((A == HIGH) != (B == LOW))
        encoder_count3--;
    else
        encoder_count3++;
}


// =====================================================
// RESET PID
// =====================================================

void resetPID(PIDController &pid)
{
    pid.integral = 0.0;
    pid.lastError = 0.0;
    pid.output = 0.0;
}


void resetAllPID()
{
    resetPID(pid1);
    resetPID(pid2);
    resetPID(pid3);
}


// =====================================================
// MOTOR PWM
// =====================================================

void setMotorPWM(
    int channelA,
    int channelB,
    float pwm)
{
    float value = abs(pwm);

    if (value > 255)
        value = 255;


    if (pwm > 0)
    {
        ledcWrite(channelA, (int)value);
        ledcWrite(channelB, 0);
    }

    else if (pwm < 0)
    {
        ledcWrite(channelA, 0);
        ledcWrite(channelB, (int)value);
    }

    else
    {
        ledcWrite(channelA, 0);
        ledcWrite(channelB, 0);
    }
}


// =====================================================
// MOTOR 1
// =====================================================

void motor1(float pwm)
{
    setMotorPWM(M1_CH_A, M1_CH_B, pwm);
}


// =====================================================
// MOTOR 2
// =====================================================

void motor2(float pwm)
{
    setMotorPWM(M2_CH_A, M2_CH_B, pwm);
}


// =====================================================
// MOTOR 3
// =====================================================

void motor3(float pwm)
{
    setMotorPWM(M3_CH_A, M3_CH_B, pwm);
}


// =====================================================
// STOP
// =====================================================

void stopRobot()
{
    motor1(0);
    motor2(0);
    motor3(0);

    targetM1 = 0;
    targetM2 = 0;
    targetM3 = 0;

    resetAllPID();

    robotMoving = false;
}


// =====================================================
// INVERSE KINEMATICS
// =====================================================

void inverseKinematics()
{
    V3 =
        matrix_kecepatan[0] * linear_x +
        matrix_kecepatan[1] * linear_y +
        matrix_kecepatan[2] * angular_z;


    V2 =
        matrix_kecepatan[3] * linear_x +
        matrix_kecepatan[4] * linear_y +
        matrix_kecepatan[5] * angular_z;


    V1 =
        matrix_kecepatan[6] * linear_x +
        matrix_kecepatan[7] * linear_y +
        matrix_kecepatan[8] * angular_z;
}


// =====================================================
// HITUNG TARGET KECEPATAN RODA
//
// Motor 1 <- V3
// Motor 2 <- V2
// Motor 3 <- V1
//
// Untuk rotasi murni:
//
// V1 = 0.0317 * angular_z
// V2 = 0.0317 * angular_z
// V3 = 0.0317 * angular_z
//
// Jadi ketiga roda mempunyai besar kecepatan
// yang sama saat rotasi.
// =====================================================

void calculateTarget()
{
    float maxV =
        max(
            max(
                (float)abs(V1),
                (float)abs(V2)
            ),
            (float)abs(V3)
        );


    if (maxV < 0.001)
    {
        targetM1 = 0;
        targetM2 = 0;
        targetM3 = 0;

        return;
    }


    float scale =
        MAX_WHEEL_SPEED / maxV;


    // Pemetaan fisik
    targetM1 = abs(V3) * scale;
    targetM2 = abs(V2) * scale;
    targetM3 = abs(V1) * scale;
}


// =====================================================
// BACA KECEPATAN ENCODER
// =====================================================

void updateEncoderSpeed(float dt)
{
    long c1;
    long c2;
    long c3;


    noInterrupts();

    c1 = encoder_count1;
    c2 = encoder_count2;
    c3 = encoder_count3;

    encoder_count1 = 0;
    encoder_count2 = 0;
    encoder_count3 = 0;

    interrupts();


    speedM1 =
        abs(c1) *
        CM_PER_PULSE /
        dt;


    speedM2 =
        abs(c2) *
        CM_PER_PULSE /
        dt;


    speedM3 =
        abs(c3) *
        CM_PER_PULSE /
        dt;
}


// =====================================================
// PID
// =====================================================

float updatePID(
    PIDController &pid,
    float target,
    float actual,
    float dt)
{
    float error =
        target - actual;


    // Integral
    pid.integral +=
        error * dt;


    // Anti wind-up
    if (pid.integral > 30)
        pid.integral = 30;

    if (pid.integral < -30)
        pid.integral = -30;


    // Derivative
    float derivative =
        (error - pid.lastError) / dt;


    // PID
    pid.output =
        (pid.Kp * error) +
        (pid.Ki * pid.integral) +
        (pid.Kd * derivative);


    // Batasi koreksi
    if (pid.output > PID_LIMIT)
        pid.output = PID_LIMIT;

    if (pid.output < -PID_LIMIT)
        pid.output = -PID_LIMIT;


    pid.lastError = error;


    return pid.output;
}


// =====================================================
// DRIVE ROBOT
// =====================================================

void driveRobot(float dt)
{
    // =================================================
    // PWM DASAR
    // =================================================

    float pwmM1 = BASE_PWM_M1;
    float pwmM2 = BASE_PWM_M2;
    float pwmM3 = BASE_PWM_M3;


    // =================================================
    // PID
    // =================================================

    float correctionM1 =
        updatePID(
            pid1,
            targetM1,
            speedM1,
            dt
        );


    float correctionM2 =
        updatePID(
            pid2,
            targetM2,
            speedM2,
            dt
        );


    float correctionM3 =
        updatePID(
            pid3,
            targetM3,
            speedM3,
            dt
        );


    // =================================================
    // KOREKSI PID
    // =================================================

    pwmM1 += correctionM1;
    pwmM2 += correctionM2;
    pwmM3 += correctionM3;


    // =================================================
    // BATAS PWM
    // =================================================

    if (pwmM1 < 0)
        pwmM1 = 0;

    if (pwmM2 < 0)
        pwmM2 = 0;

    if (pwmM3 < 0)
        pwmM3 = 0;


    if (pwmM1 > 255)
        pwmM1 = 255;

    if (pwmM2 > 255)
        pwmM2 = 255;

    if (pwmM3 > 255)
        pwmM3 = 255;


    // =================================================
    // ARAH MOTOR
    // =================================================

    // Motor 1 <- V3
    if (V3 < 0)
        motor1(-pwmM1);

    else if (V3 > 0)
        motor1(pwmM1);

    else
        motor1(0);


    // Motor 2 <- V2
    if (V2 < 0)
        motor2(-pwmM2);

    else if (V2 > 0)
        motor2(pwmM2);

    else
        motor2(0);


    // Motor 3 <- V1
    if (V1 < 0)
        motor3(-pwmM3);

    else if (V1 > 0)
        motor3(pwmM3);

    else
        motor3(0);


    // =================================================
    // MONITORING
    // =================================================

    static unsigned long lastPrint = 0;

    if (millis() - lastPrint >= 500)
    {
        lastPrint = millis();

        Serial.println();
        Serial.println("========== DATA ==========");

        Serial.print("X = ");
        Serial.println(linear_x, 3);

        Serial.print("Y = ");
        Serial.println(linear_y, 3);

        Serial.print("Omega = ");
        Serial.println(angular_z, 3);

        Serial.println();

        Serial.print("V1 = ");
        Serial.println(V1, 3);

        Serial.print("V2 = ");
        Serial.println(V2, 3);

        Serial.print("V3 = ");
        Serial.println(V3, 3);

        Serial.println();

        Serial.print("Target M1 = ");
        Serial.println(targetM1);

        Serial.print("Target M2 = ");
        Serial.println(targetM2);

        Serial.print("Target M3 = ");
        Serial.println(targetM3);

        Serial.println();

        Serial.print("Speed M1 = ");
        Serial.print(speedM1);
        Serial.println(" cm/s");

        Serial.print("Speed M2 = ");
        Serial.print(speedM2);
        Serial.println(" cm/s");

        Serial.print("Speed M3 = ");
        Serial.print(speedM3);
        Serial.println(" cm/s");

        Serial.println();

        Serial.print("PID M1 = ");
        Serial.println(correctionM1);

        Serial.print("PID M2 = ");
        Serial.println(correctionM2);

        Serial.print("PID M3 = ");
        Serial.println(correctionM3);

        Serial.println();

        Serial.print("PWM M1 = ");
        Serial.println(pwmM1);

        Serial.print("PWM M2 = ");
        Serial.println(pwmM2);

        Serial.print("PWM M3 = ");
        Serial.println(pwmM3);

        Serial.println("==========================");
    }
}


// =====================================================
// SETUP MOTOR
// =====================================================

void setupMotor()
{
    // MOTOR 1
    ledcSetup(M1_CH_A, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(M1_CH_B, PWM_FREQ, PWM_RESOLUTION);

    ledcAttachPin(M1_IN_A, M1_CH_A);
    ledcAttachPin(M1_IN_B, M1_CH_B);


    // MOTOR 2
    ledcSetup(M2_CH_A, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(M2_CH_B, PWM_FREQ, PWM_RESOLUTION);

    ledcAttachPin(M2_IN_A, M2_CH_A);
    ledcAttachPin(M2_IN_B, M2_CH_B);


    // MOTOR 3
    ledcSetup(M3_CH_A, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(M3_CH_B, PWM_FREQ, PWM_RESOLUTION);

    ledcAttachPin(M3_IN_A, M3_CH_A);
    ledcAttachPin(M3_IN_B, M3_CH_B);


    stopRobot();
}


// =====================================================
// SETUP
// =====================================================

void setup()
{
    Serial.begin(115200);


    // Encoder
    pinMode(ENC1_A, INPUT);
    pinMode(ENC1_B, INPUT);

    pinMode(ENC2_A, INPUT);
    pinMode(ENC2_B, INPUT);

    pinMode(ENC3_A, INPUT);
    pinMode(ENC3_B, INPUT);


    // Interrupt
    attachInterrupt(
        digitalPinToInterrupt(ENC1_A),
        encoderISR1,
        CHANGE
    );

    attachInterrupt(
        digitalPinToInterrupt(ENC2_A),
        encoderISR2,
        CHANGE
    );

    attachInterrupt(
        digitalPinToInterrupt(ENC3_A),
        encoderISR3,
        CHANGE
    );


    setupMotor();


    Serial.println();
    Serial.println("========================================");
    Serial.println(" OMNI ROBOT");
    Serial.println(" IK + ENCODER + PID + ANGULAR MOTION");
    Serial.println("========================================");
    Serial.println("W = MAJU");
    Serial.println("S = MUNDUR");
    Serial.println("A = PUTAR KIRI");
    Serial.println("D = PUTAR KANAN");
    Serial.println("X = STOP");
    Serial.println("========================================");
}


// =====================================================
// LOOP
// =====================================================

void loop()
{
    static unsigned long lastPIDTime = 0;

    unsigned long now = millis();


    // =================================================
    // UPDATE PID SETIAP 50 ms
    // =================================================

    if (now - lastPIDTime >= 50)
    {
        float dt =
            (now - lastPIDTime) / 1000.0;

        lastPIDTime = now;


        // Baca encoder
        updateEncoderSpeed(dt);


        // Jalankan PID
        if (robotMoving)
        {
            driveRobot(dt);
        }
    }


    // =================================================
    // SERIAL COMMAND
    // =================================================

    if (Serial.available() > 0)
    {
        char command = Serial.read();


        // =============================================
        // MAJU
        // =============================================

        if (command == 'w' ||
            command == 'W')
        {
            linear_x = 0.5;
            linear_y = 0.0;
            angular_z = 0.0;


            resetAllPID();


            inverseKinematics();
            calculateTarget();


            robotMoving = true;


            Serial.println();
            Serial.println(">>> MAJU");
        }


        // =============================================
        // MUNDUR
        // =============================================

        else if (command == 's' ||
                 command == 'S')
        {
            linear_x = -0.5;
            linear_y = 0.0;
            angular_z = 0.0;


            resetAllPID();


            inverseKinematics();
            calculateTarget();


            robotMoving = true;


            Serial.println();
            Serial.println(">>> MUNDUR");
        }


        // =============================================
        // PUTAR KIRI
        // =============================================
        //
        // CCW / angular_z positif
        // =============================================

        else if (command == 'a' ||
                 command == 'A')
        {
            linear_x = 0.0;
            linear_y = 0.0;
            angular_z = 0.5;


            resetAllPID();


            inverseKinematics();
            calculateTarget();


            robotMoving = true;


            Serial.println();
            Serial.println(">>> PUTAR KIRI");
        }


        // =============================================
        // PUTAR KANAN
        // =============================================
        //
        // CW / angular_z negatif
        // =============================================

        else if (command == 'd' ||
                 command == 'D')
        {
            linear_x = 0.0;
            linear_y = 0.0;
            angular_z = -0.5;


            resetAllPID();


            inverseKinematics();
            calculateTarget();


            robotMoving = true;


            Serial.println();
            Serial.println(">>> PUTAR KANAN");
        }


        // =============================================
        // STOP
        // =============================================

        else if (command == 'x' ||
                 command == 'X')
        {
            stopRobot();

            Serial.println();
            Serial.println(">>> STOP");
        }
    }
}
```

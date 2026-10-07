#include <Arduino.h>

// =====================================================
// PIN MOTOR
// =====================================================

// MOTOR 1
#define M1_IN_A 27
#define M1_IN_B 14

// MOTOR 2
#define M2_IN_A 18
#define M2_IN_B 4

// MOTOR 3
#define M3_IN_A 2
#define M3_IN_B 13


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
// INVERSE KINEMATICS
// Matriks dari program dosen
//
// V3 = -0.3333*x + 0.5774*y + 0.0317*omega
// V2 = -0.3333*x - 0.5774*y + 0.0317*omega
// V1 =  0.6667*x + 0*y       + 0.0317*omega
// =====================================================

double matrix_kecepatan[9] =
{
    -0.3333,  0.5774, 0.0317,
    -0.3333, -0.5774, 0.0317,
     0.6667,  0.0000, 0.0317
};


// =====================================================
// INPUT KECEPATAN ROBOT
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
// KONSTANTA PWM
// =====================================================

const double SPEED_LIMIT = 0.5;
const int PWM_MAX = 100;


// =====================================================
// MOTOR 1
// speed positif:
//     IN A = PWM
//     IN B = 0
//
// speed negatif:
//     IN A = 0
//     IN B = PWM
// =====================================================

void motor1(double speed)
{
    int pwm = (int)abs(speed);

    if (pwm > 255)
        pwm = 255;

    if (speed > 0)
    {
        ledcWrite(M1_CH_A, pwm);
        ledcWrite(M1_CH_B, 0);
    }
    else if (speed < 0)
    {
        ledcWrite(M1_CH_A, 0);
        ledcWrite(M1_CH_B, pwm);
    }
    else
    {
        ledcWrite(M1_CH_A, 0);
        ledcWrite(M1_CH_B, 0);
    }
}


// =====================================================
// MOTOR 2
// =====================================================

void motor2(double speed)
{
    int pwm = (int)abs(speed);

    if (pwm > 255)
        pwm = 255;

    if (speed > 0)
    {
        ledcWrite(M2_CH_A, pwm);
        ledcWrite(M2_CH_B, 0);
    }
    else if (speed < 0)
    {
        ledcWrite(M2_CH_A, 0);
        ledcWrite(M2_CH_B, pwm);
    }
    else
    {
        ledcWrite(M2_CH_A, 0);
        ledcWrite(M2_CH_B, 0);
    }
}


// =====================================================
// MOTOR 3
// =====================================================

void motor3(double speed)
{
    int pwm = (int)abs(speed);

    if (pwm > 255)
        pwm = 255;

    if (speed > 0)
    {
        ledcWrite(M3_CH_A, pwm);
        ledcWrite(M3_CH_B, 0);
    }
    else if (speed < 0)
    {
        ledcWrite(M3_CH_A, 0);
        ledcWrite(M3_CH_B, pwm);
    }
    else
    {
        ledcWrite(M3_CH_A, 0);
        ledcWrite(M3_CH_B, 0);
    }
}


// =====================================================
// STOP
// =====================================================

void stopRobot()
{
    motor1(0);
    motor2(0);
    motor3(0);
}


// =====================================================
// INVERSE KINEMATICS
// =====================================================

void inverseKinematics()
{
    V3 = matrix_kecepatan[0] * linear_x
       + matrix_kecepatan[1] * linear_y
       + matrix_kecepatan[2] * angular_z;

    V2 = matrix_kecepatan[3] * linear_x
       + matrix_kecepatan[4] * linear_y
       + matrix_kecepatan[5] * angular_z;

    V1 = matrix_kecepatan[6] * linear_x
       + matrix_kecepatan[7] * linear_y
       + matrix_kecepatan[8] * angular_z;
}


// =====================================================
// KONVERSI KE PWM
// =====================================================

int toPWM(double velocity)
{
    double pwm;

    pwm = (abs(velocity) / SPEED_LIMIT) * PWM_MAX;

    if (pwm > 255)
        pwm = 255;

    return (int)pwm;
}


// =====================================================
// GERAKKAN ROBOT
//
// PENTING:
// Hasil inverse kinematics dipetakan ke motor fisik:
//
// Motor 1 <- V3
// Motor 2 <- V2
// Motor 3 <- V1
//
// Pemetaan ini disesuaikan dengan konfigurasi fisik
// robotmu dan arah motor yang sudah diuji.
// =====================================================

void driveRobot()
{
    // ---------------------------------------------
    // Motor 1 menggunakan V3
    // ---------------------------------------------

    int pwmM1 = toPWM(V3);

    if (V3 > 0)
        motor1(pwmM1);
    else if (V3 < 0)
        motor1(-pwmM1);
    else
        motor1(0);


    // ---------------------------------------------
    // Motor 2 menggunakan V2
    // ---------------------------------------------

    int pwmM2 = toPWM(V2);

    if (V2 > 0)
        motor2(pwmM2);
    else if (V2 < 0)
        motor2(-pwmM2);
    else
        motor2(0);


    // ---------------------------------------------
    // Motor 3 menggunakan V1
    // ---------------------------------------------

    int pwmM3 = toPWM(V1);

    if (V1 > 0)
        motor3(pwmM3);
    else if (V1 < 0)
        motor3(-pwmM3);
    else
        motor3(0);
}


// =====================================================
// TAMPILKAN DATA
// =====================================================

void printData()
{
    Serial.println();
    Serial.println("========== DATA ==========");

    Serial.print("linear_x  : ");
    Serial.println(linear_x);

    Serial.print("linear_y  : ");
    Serial.println(linear_y);

    Serial.print("angular_z : ");
    Serial.println(angular_z);

    Serial.print("V1 : ");
    Serial.println(V1);

    Serial.print("V2 : ");
    Serial.println(V2);

    Serial.print("V3 : ");
    Serial.println(V3);

    Serial.print("Motor 1 PWM : ");
    Serial.println(toPWM(V3));

    Serial.print("Motor 2 PWM : ");
    Serial.println(toPWM(V2));

    Serial.print("Motor 3 PWM : ");
    Serial.println(toPWM(V1));

    Serial.println("==========================");
}


// =====================================================
// SETUP PWM
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

    setupMotor();

    Serial.println();
    Serial.println("=================================");
    Serial.println(" OMNI ROBOT - INVERSE KINEMATICS");
    Serial.println("=================================");
    Serial.println("W = MAJU");
    Serial.println("S = MUNDUR");
    Serial.println("X = STOP");
    Serial.println("=================================");
}


// =====================================================
// LOOP
// =====================================================

void loop()
{
    if (Serial.available() > 0)
    {
        char command = Serial.read();


        // =============================================
        // MAJU
        // =============================================

        if (command == 'w' || command == 'W')
        {
            linear_x  = 0.5;
            linear_y  = 0.0;
            angular_z = 0.0;

            inverseKinematics();

            driveRobot();

            printData();

            Serial.println(">>> MAJU");
        }


        // =============================================
        // MUNDUR
        // =============================================

        else if (command == 's' || command == 'S')
        {
            linear_x  = -0.5;
            linear_y  = 0.0;
            angular_z = 0.0;

            inverseKinematics();

            driveRobot();

            printData();

            Serial.println(">>> MUNDUR");
        }


        // =============================================
        // STOP
        // =============================================

        else if (command == 'x' || command == 'X')
        {
            linear_x  = 0.0;
            linear_y  = 0.0;
            angular_z = 0.0;

            V1 = 0.0;
            V2 = 0.0;
            V3 = 0.0;

            stopRobot();

            Serial.println(">>> STOP");
        }
    }
}
#include <Arduino.h>
#include <math.h>

// =====================================================
// 1. PIN MOTOR ESP32
// =====================================================

// MOTOR 1
#define M1_IN_A 23
#define M1_IN_B 19

// MOTOR 2
#define M2_IN_A 18
#define M2_IN_B 4

// MOTOR 3
#define M3_IN_A 13
#define M3_IN_B 2


// =====================================================
// 2. PIN ENCODER
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
// 3. KONFIGURASI PWM
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
// 4. PARAMETER ENCODER DAN RODA
// =====================================================

// Sesuaikan jika hasil kalibrasi PPR berbeda.
const int PPR = 1640;

const float WHEEL_RADIUS_CM = 2.9f;

const float WHEEL_CIRCUMFERENCE =
    2.0f * PI * WHEEL_RADIUS_CM;

const float CM_PER_PULSE =
    WHEEL_CIRCUMFERENCE / PPR;


// =====================================================
// 5. INVERSE KINEMATICS DARI PROGRAM DOSEN
// =====================================================
//
// V3 = -0.3333*x + 0.5774*y + 0.0317*omega
// V2 = -0.3333*x - 0.5774*y + 0.0317*omega
// V1 =  0.6667*x             + 0.0317*omega
//
// Pemetaan fisik yang digunakan dalam program ini:
//
// Motor 1 <- V1
// Motor 2 <- V2
// Motor 3 <- V3
//
// =====================================================

const double matrix_kecepatan[9] =
{
    -0.3333,  0.5774, 0.0317,
    -0.3333, -0.5774, 0.0317,
     0.6667,  0.0000, 0.0317
};


// =====================================================
// 6. INPUT GERAK ROBOT
// =====================================================

double linear_x  = 0.0;
double linear_y  = 0.0;
double angular_z = 0.0;


// =====================================================
// 7. HASIL INVERSE KINEMATICS
// =====================================================

double V1 = 0.0;
double V2 = 0.0;
double V3 = 0.0;


// =====================================================
// 8. PENGATURAN KECEPATAN
// =====================================================

// Kecepatan target roda tercepat.
// Nilai ini mengikuti skala target 5 cm/s pada
// Serial Monitor yang kamu kirim.

const float MAX_WHEEL_SPEED = 5.0f;

// PWM dasar maksimum untuk target kecepatan tersebut.
const int PWM_AT_MAX_SPEED = 80;


// =====================================================
// 9. PWM MINIMUM TIAP MOTOR DAN INPUT DRIVER
// =====================================================
//
// Nilai awal mengikuti konfigurasi sebelumnya.
// Sesuaikan dengan hasil pengujian fisik.
//
// Positif = IN_A mendapatkan PWM
// Negatif = IN_B mendapatkan PWM
// =====================================================

// MOTOR 1
const int PWM_MIN_M1_INA = 75;
const int PWM_MIN_M1_INB = 55;

// MOTOR 2
const int PWM_MIN_M2_INA = 50;
const int PWM_MIN_M2_INB = 50;

// MOTOR 3
const int PWM_MIN_M3_INA = 60;
const int PWM_MIN_M3_INB = 60;


// =====================================================
// 10. STATUS PID
// =====================================================
//
// FALSE untuk sementara agar kita bisa memastikan
// arah gerakan benar tanpa koreksi berdasarkan
// feedback encoder yang belum valid.
//
// Setelah ketiga encoder terbaca benar, ubah menjadi
// TRUE untuk mengaktifkan koreksi PID.
// =====================================================

const bool ENABLE_ENCODER_PID = true;


// =====================================================
// 11. DATA ENCODER
// =====================================================

volatile long encoder_count1 = 0;
volatile long encoder_count2 = 0;
volatile long encoder_count3 = 0;


// =====================================================
// 12. KECEPATAN AKTUAL RODA
// =====================================================

float speedM1 = 0.0f;
float speedM2 = 0.0f;
float speedM3 = 0.0f;


// =====================================================
// 13. TARGET KECEPATAN RODA
// =====================================================

float targetM1 = 0.0f;
float targetM2 = 0.0f;
float targetM3 = 0.0f;


// =====================================================
// 14. PARAMETER PID
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


// Nilai awal. Belum merupakan hasil tuning final.
PIDController pid1 =
{
    1.5f, 0.1f, 0.0f,
    0.0f, 0.0f, 0.0f
};

PIDController pid2 =
{
    1.5f, 0.1f, 0.0f,
    0.0f, 0.0f, 0.0f
};

PIDController pid3 =
{
    1.5f, 0.1f, 0.0f,
    0.0f, 0.0f, 0.0f
};

const float PID_LIMIT = 15.0f;


// =====================================================
// 15. STATUS ROBOT
// =====================================================

bool robotMoving = false;

// ----- TAMBAHAN: kendali dari ROS2 -----
float speedScale = 1.0f;                      // 0.0 - 1.0, skala kecepatan dari ROS2
bool remoteMode = false;                      // true selama perintah V aktif
unsigned long lastRemoteCmd = 0;
const unsigned long REMOTE_TIMEOUT_MS = 500;  // tidak ada perintah V selama ini = STOP


// =====================================================
// 16. ENCODER INTERRUPT
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
// 17. RESET PID
// =====================================================

void resetPID(PIDController &pid)
{
    pid.integral = 0.0f;
    pid.lastError = 0.0f;
    pid.output = 0.0f;
}


void resetAllPID()
{
    resetPID(pid1);
    resetPID(pid2);
    resetPID(pid3);
}


// =====================================================
// 18. FUNGSI ARAH
// =====================================================

int signOf(double value)
{
    if (value > 0.0001)
        return 1;

    if (value < -0.0001)
        return -1;

    return 0;
}


// =====================================================
// 19. KENDALI PWM MOTOR
// =====================================================
//
// PWM positif:
// IN_A = PWM
// IN_B = 0
//
// PWM negatif:
// IN_A = 0
// IN_B = PWM
//
// PWM nol:
// Kedua input = 0
// =====================================================

void setMotorPWM(
    int channelA,
    int channelB,
    float signedPWM)
{
    int pwm = (int)fabsf(signedPWM);

    if (pwm > 255)
        pwm = 255;

    if (signedPWM > 0.0f)
    {
        ledcWrite(channelA, pwm);
        ledcWrite(channelB, 0);
    }
    else if (signedPWM < 0.0f)
    {
        ledcWrite(channelA, 0);
        ledcWrite(channelB, pwm);
    }
    else
    {
        ledcWrite(channelA, 0);
        ledcWrite(channelB, 0);
    }
}


// =====================================================
// 20. MOTOR 1
// =====================================================

void motor1(float pwm)
{
    setMotorPWM(M1_CH_A, M1_CH_B, pwm);
}


// =====================================================
// 21. MOTOR 2
// =====================================================

void motor2(float pwm)
{
    setMotorPWM(M2_CH_A, M2_CH_B, pwm);
}


// =====================================================
// 22. MOTOR 3
// =====================================================

void motor3(float pwm)
{
    setMotorPWM(M3_CH_A, M3_CH_B, pwm);
}


// =====================================================
// 23. STOP ROBOT
// =====================================================

void stopRobot()
{
    motor1(0);
    motor2(0);
    motor3(0);

    linear_x = 0.0;
    linear_y = 0.0;
    angular_z = 0.0;

    V1 = 0.0;
    V2 = 0.0;
    V3 = 0.0;

    targetM1 = 0.0f;
    targetM2 = 0.0f;
    targetM3 = 0.0f;

    resetAllPID();

    speedScale = 1.0f;
    remoteMode = false;
    robotMoving = false;
}


// =====================================================
// 24. INVERSE KINEMATICS
// =====================================================

void inverseKinematics()
{
    V3 =
        matrix_kecepatan[0] * linear_x
      + matrix_kecepatan[1] * linear_y
      + matrix_kecepatan[2] * angular_z;

    V2 =
        matrix_kecepatan[3] * linear_x
      + matrix_kecepatan[4] * linear_y
      + matrix_kecepatan[5] * angular_z;

    V1 =
        matrix_kecepatan[6] * linear_x
      + matrix_kecepatan[7] * linear_y
      + matrix_kecepatan[8] * angular_z;
}


// =====================================================
// 25. HITUNG TARGET KECEPATAN
// =====================================================
//
// PENTING:
//
// Motor 1 <- V1
// Motor 2 <- V2
// Motor 3 <- V3
//
// Target dinormalisasi sehingga roda dengan nilai
// absolut V terbesar memiliki target MAX_WHEEL_SPEED.
// =====================================================

void calculateTarget()
{
    float absV1 = fabsf((float)V1);
    float absV2 = fabsf((float)V2);
    float absV3 = fabsf((float)V3);

    float maxV = absV1;

    if (absV2 > maxV)
        maxV = absV2;

    if (absV3 > maxV)
        maxV = absV3;

    if (maxV < 0.0001f)
    {
        targetM1 = 0.0f;
        targetM2 = 0.0f;
        targetM3 = 0.0f;
        return;
    }

    float scale = MAX_WHEEL_SPEED * speedScale / maxV;

    targetM1 = absV1 * scale;
    targetM2 = absV2 * scale;
    targetM3 = absV3 * scale;
}


// =====================================================
// 26. BACA KECEPATAN ENCODER
// =====================================================

void updateEncoderSpeed(float dt)
{
    if (dt <= 0.001f)
        return;

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
        fabsf((float)c1) * CM_PER_PULSE / dt;

    speedM2 =
        fabsf((float)c2) * CM_PER_PULSE / dt;

    speedM3 =
        fabsf((float)c3) * CM_PER_PULSE / dt;
}


// =====================================================
// 27. PEMILIH PWM MINIMUM
// =====================================================

int getMinimumPWM(int motorNumber, int direction)
{
    // direction positif menggunakan IN_A
    // direction negatif menggunakan IN_B

    if (motorNumber == 1)
    {
        return (direction > 0)
            ? PWM_MIN_M1_INA
            : PWM_MIN_M1_INB;
    }

    if (motorNumber == 2)
    {
        return (direction > 0)
            ? PWM_MIN_M2_INA
            : PWM_MIN_M2_INB;
    }

    return (direction > 0)
        ? PWM_MIN_M3_INA
        : PWM_MIN_M3_INB;
}


// =====================================================
// 28. KONVERSI TARGET KE PWM DASAR
// =====================================================

float targetToPWM(float target, int minimumPWM)
{
    if (target <= 0.01f)
        return 0.0f;

    float ratio = target / MAX_WHEEL_SPEED;

    if (ratio > 1.0f)
        ratio = 1.0f;

    if (ratio < 0.0f)
        ratio = 0.0f;

    float pwm =
        minimumPWM
      + ratio * (PWM_AT_MAX_SPEED - minimumPWM);

    if (pwm > 255.0f)
        pwm = 255.0f;

    return pwm;
}


// =====================================================
// 29. PID
// =====================================================

float updatePID(
    PIDController &pid,
    float target,
    float actual,
    float dt)
{
    if (target <= 0.01f || dt <= 0.001f)
    {
        resetPID(pid);
        return 0.0f;
    }

    float error = target - actual;

    pid.integral += error * dt;

    // Anti-windup
    if (pid.integral > 30.0f)
        pid.integral = 30.0f;

    if (pid.integral < -30.0f)
        pid.integral = -30.0f;

    float derivative =
        (error - pid.lastError) / dt;

    pid.output =
        pid.Kp * error
      + pid.Ki * pid.integral
      + pid.Kd * derivative;

    if (pid.output > PID_LIMIT)
        pid.output = PID_LIMIT;

    if (pid.output < -PID_LIMIT)
        pid.output = -PID_LIMIT;

    pid.lastError = error;

    return pid.output;
}


// =====================================================
// 30. KENDALI SEMUA MOTOR
// =====================================================

void driveRobot(float dt)
{
    // -------------------------------------------------
    // Tentukan arah dari hasil inverse kinematics.
    // Tidak ada pembalikan polaritas tambahan.
    // -------------------------------------------------

    int dir1 = signOf(V1);
    int dir2 = signOf(V2);
    int dir3 = signOf(V3);


    // -------------------------------------------------
    // Target nol berarti motor wajib berhenti.
    // -------------------------------------------------

    if (targetM1 <= 0.01f || dir1 == 0)
    {
        motor1(0);
        resetPID(pid1);
    }

    if (targetM2 <= 0.01f || dir2 == 0)
    {
        motor2(0);
        resetPID(pid2);
    }

    if (targetM3 <= 0.01f || dir3 == 0)
    {
        motor3(0);
        resetPID(pid3);
    }


    // -------------------------------------------------
    // Tentukan PWM minimum berdasarkan arah driver.
    // -------------------------------------------------

    int minPWM1 = getMinimumPWM(1, dir1);
    int minPWM2 = getMinimumPWM(2, dir2);
    int minPWM3 = getMinimumPWM(3, dir3);


    // -------------------------------------------------
    // PWM dasar dari target kecepatan.
    // -------------------------------------------------

    float pwmBase1 =
        targetToPWM(targetM1, minPWM1);

    float pwmBase2 =
        targetToPWM(targetM2, minPWM2);

    float pwmBase3 =
        targetToPWM(targetM3, minPWM3);


    // -------------------------------------------------
    // PID hanya dipakai jika diaktifkan.
    // Untuk pengujian arah, default-nya OFF.
    // -------------------------------------------------

    float correction1 = 0.0f;
    float correction2 = 0.0f;
    float correction3 = 0.0f;

    if (ENABLE_ENCODER_PID)
    {
        correction1 =
            updatePID(pid1, targetM1, speedM1, dt);

        correction2 =
            updatePID(pid2, targetM2, speedM2, dt);

        correction3 =
            updatePID(pid3, targetM3, speedM3, dt);
    }


    // -------------------------------------------------
    // PWM akhir.
    // -------------------------------------------------

    float pwm1 = pwmBase1 + correction1;
    float pwm2 = pwmBase2 + correction2;
    float pwm3 = pwmBase3 + correction3;


    // Motor aktif dipertahankan di atas PWM minimum.
    if (targetM1 > 0.01f && pwm1 < minPWM1)
        pwm1 = minPWM1;

    if (targetM2 > 0.01f && pwm2 < minPWM2)
        pwm2 = minPWM2;

    if (targetM3 > 0.01f && pwm3 < minPWM3)
        pwm3 = minPWM3;


    // Batasi PWM
    pwm1 = constrain(pwm1, 0.0f, 255.0f);
    pwm2 = constrain(pwm2, 0.0f, 255.0f);
    pwm3 = constrain(pwm3, 0.0f, 255.0f);


    // -------------------------------------------------
    // ARAH DAN PWM MOTOR
    //
    // Motor 1 mengikuti V1
    // Motor 2 mengikuti V2
    // Motor 3 mengikuti V3
    // -------------------------------------------------

    if (targetM1 > 0.01f)
        motor1(dir1 * pwm1);

    if (targetM2 > 0.01f)
        motor2(-dir2 * pwm2);

    if (targetM3 > 0.01f)
        motor3(dir3 * pwm3);


    // -------------------------------------------------
    // SERIAL MONITOR
    // -------------------------------------------------

    static unsigned long lastPrint = 0;

    if (millis() - lastPrint >= 500)
    {
        lastPrint = millis();

        Serial.println();
        Serial.println("========== ROBOT MONITOR ==========");

        Serial.print("linear_x = ");
        Serial.println(linear_x, 3);

        Serial.print("linear_y = ");
        Serial.println(linear_y, 3);

        Serial.print("angular_z = ");
        Serial.println(angular_z, 3);

        Serial.println();

        Serial.print("V1 = ");
        Serial.println(V1, 4);

        Serial.print("V2 = ");
        Serial.println(V2, 4);

        Serial.print("V3 = ");
        Serial.println(V3, 4);

        Serial.println();

        Serial.print("Target M1 = ");
        Serial.print(targetM1, 2);
        Serial.println(" cm/s");

        Serial.print("Target M2 = ");
        Serial.print(targetM2, 2);
        Serial.println(" cm/s");

        Serial.print("Target M3 = ");
        Serial.print(targetM3, 2);
        Serial.println(" cm/s");

        Serial.println();

        Serial.print("Speed M1 = ");
        Serial.print(speedM1, 2);
        Serial.println(" cm/s");

        Serial.print("Speed M2 = ");
        Serial.print(speedM2, 2);
        Serial.println(" cm/s");

        Serial.print("Speed M3 = ");
        Serial.print(speedM3, 2);
        Serial.println(" cm/s");

        Serial.println();

        Serial.print("PWM M1 = ");
        Serial.println(pwm1, 1);

        Serial.print("PWM M2 = ");
        Serial.println(pwm2, 1);

        Serial.print("PWM M3 = ");
        Serial.println(pwm3, 1);

        Serial.println("===================================");
    }
}


// =====================================================
// 31. SETUP PWM
// =====================================================

void setupMotor()
{
    // Motor 1
    ledcSetup(M1_CH_A, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(M1_CH_B, PWM_FREQ, PWM_RESOLUTION);

    ledcAttachPin(M1_IN_A, M1_CH_A);
    ledcAttachPin(M1_IN_B, M1_CH_B);

    // Motor 2
    ledcSetup(M2_CH_A, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(M2_CH_B, PWM_FREQ, PWM_RESOLUTION);

    ledcAttachPin(M2_IN_A, M2_CH_A);
    ledcAttachPin(M2_IN_B, M2_CH_B);

    // Motor 3
    ledcSetup(M3_CH_A, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(M3_CH_B, PWM_FREQ, PWM_RESOLUTION);

    ledcAttachPin(M3_IN_A, M3_CH_A);
    ledcAttachPin(M3_IN_B, M3_CH_B);

    motor1(0);
    motor2(0);
    motor3(0);
}


// =====================================================
// 32. SETUP
// =====================================================

// ===================== PIN =====================
// Motor 1 = D23 & D19 | Motor 2 = D18 & D4 | Motor 3 = D2 & D13
const uint8_t IN_A[3] = {27, 18, 2};
const uint8_t IN_B[3] = {14,  4, 13};
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
    Serial.setTimeout(20);

    // GPIO34–GPIO39 tidak memiliki pull-up internal.
    // Pastikan rangkaian encoder menyediakan level
    // logika dan pull-up eksternal yang sesuai.

    pinMode(ENC1_A, INPUT);
    pinMode(ENC1_B, INPUT);

    pinMode(ENC2_A, INPUT);
    pinMode(ENC2_B, INPUT);

    pinMode(ENC3_A, INPUT);
    pinMode(ENC3_B, INPUT);

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

Serial.println("======================================");
Serial.println(" OMNI ROBOT 3 RODA");
Serial.println(" HOLONOMIC + INVERSE KINEMATICS + PID");
Serial.println("======================================");
Serial.println("W = MAJU (+Y)");
Serial.println("S = MUNDUR (-Y)");
Serial.println("Q = TRANSLASI X POSITIF (+X)");
Serial.println("E = TRANSLASI X NEGATIF (-X)");
Serial.println("R = DIAGONAL MAJU + X POSITIF");
Serial.println("T = DIAGONAL MAJU + X NEGATIF");
Serial.println("F = DIAGONAL MUNDUR + X POSITIF");
Serial.println("G = DIAGONAL MUNDUR + X NEGATIF");
Serial.println("A = PUTAR KIRI");
Serial.println("D = PUTAR KANAN");
Serial.println("X = STOP");
Serial.println("======================================");

    Serial.print("Encoder PID: ");
    Serial.println(
        ENABLE_ENCODER_PID ? "AKTIF" : "NONAKTIF"
    );
}


// =====================================================
// 33. LOOP
// =====================================================

void loop()
{
    static unsigned long lastPIDTime = 0;

    unsigned long now = millis();

    // Sampling encoder dan PID setiap 50 ms.
    if (now - lastPIDTime >= 50)
    {
        float dt =
            (now - lastPIDTime) / 1000.0f;

        lastPIDTime = now;

        updateEncoderSpeed(dt);

        if (robotMoving)
        {
            driveRobot(dt);
        }
    }

    // -------------------------------------------------
    // PERINTAH SERIAL
    // -------------------------------------------------
    // WATCHDOG: kalau perintah dari ROS2 berhenti datang, robot berhenti sendiri
    if (remoteMode && (millis() - lastRemoteCmd > REMOTE_TIMEOUT_MS))
    {
        stopRobot();
        Serial.println(">>> WATCHDOG: perintah ROS2 hilang, STOP");
    }

    if (Serial.available() > 0)
    {
        char command = Serial.read();

        // =============================================
        // MAJU
        //
        // Menggunakan sumbu Y karena konfigurasi yang
        // diinginkan adalah Motor 1 berhenti ketika
        // robot bergerak maju lurus.
        // =============================================

        if (command == 'w' || command == 'W')
        {
            linear_x = 0.0;
            linear_y = 0.5;
            angular_z = 0.0;

            resetAllPID();

            inverseKinematics();
            calculateTarget();

            robotMoving = true;

            Serial.println();
            Serial.println(">>> MAJU");

            Serial.print("V1 = ");
            Serial.println(V1, 4);

            Serial.print("V2 = ");
            Serial.println(V2, 4);

            Serial.print("V3 = ");
            Serial.println(V3, 4);
        }


        // =============================================
        // MUNDUR
        // =============================================

        else if (command == 's' || command == 'S')
        {
            linear_x = 0.0;
            linear_y = -0.5;
            angular_z = 0.0;

            resetAllPID();

            inverseKinematics();
            calculateTarget();

            robotMoving = true;

            Serial.println();
            Serial.println(">>> MUNDUR");

            Serial.print("V1 = ");
            Serial.println(V1, 4);

            Serial.print("V2 = ");
            Serial.println(V2, 4);

            Serial.print("V3 = ");
            Serial.println(V3, 4);
        }

// =============================================
// TRANSLASI SUMBU X POSITIF
// =============================================

else if (command == 'q' || command == 'Q')
{
    linear_x = 0.5;
    linear_y = 0.0;
    angular_z = 0.0;

    resetAllPID();

    inverseKinematics();
    calculateTarget();

    robotMoving = true;

    Serial.println();
    Serial.println(">>> TRANSLASI X POSITIF");

    Serial.print("V1 = ");
    Serial.println(V1, 4);

    Serial.print("V2 = ");
    Serial.println(V2, 4);

    Serial.print("V3 = ");
    Serial.println(V3, 4);
}


// =============================================
// TRANSLASI SUMBU X NEGATIF
// =============================================

else if (command == 'e' || command == 'E')
{
    linear_x = -0.5;
    linear_y = 0.0;
    angular_z = 0.0;

    resetAllPID();

    inverseKinematics();
    calculateTarget();

    robotMoving = true;

    Serial.println();
    Serial.println(">>> TRANSLASI X NEGATIF");

    Serial.print("V1 = ");
    Serial.println(V1, 4);

    Serial.print("V2 = ");
    Serial.println(V2, 4);

    Serial.print("V3 = ");
    Serial.println(V3, 4);
}


// =============================================
// GERAK HOLONOMIC DIAGONAL
// Komponen X dan Y aktif bersamaan, tanpa rotasi.
// Masing-masing komponen = 0.5 / sqrt(2) = 0.3536
// agar besar kecepatan translasi nominal tetap 0.5.
// =============================================

// R = MAJU + X POSITIF
else if (command == 'r' || command == 'R')
{
    linear_x = 0.3536;
    linear_y = 0.3536;
    angular_z = 0.0;

    resetAllPID();
    inverseKinematics();
    calculateTarget();
    robotMoving = true;

    Serial.println();
    Serial.println(">>> DIAGONAL: MAJU + X POSITIF");
    Serial.print("V1 = "); Serial.println(V1, 4);
    Serial.print("V2 = "); Serial.println(V2, 4);
    Serial.print("V3 = "); Serial.println(V3, 4);
}

// T = MAJU + X NEGATIF
else if (command == 't' || command == 'T')
{
    linear_x = -0.3536;
    linear_y = 0.3536;
    angular_z = 0.0;

    resetAllPID();
    inverseKinematics();
    calculateTarget();
    robotMoving = true;

    Serial.println();
    Serial.println(">>> DIAGONAL: MAJU + X NEGATIF");
    Serial.print("V1 = "); Serial.println(V1, 4);
    Serial.print("V2 = "); Serial.println(V2, 4);
    Serial.print("V3 = "); Serial.println(V3, 4);
}

// F = MUNDUR + X POSITIF
else if (command == 'f' || command == 'F')
{
    linear_x = 0.3536;
    linear_y = -0.3536;
    angular_z = 0.0;

    resetAllPID();
    inverseKinematics();
    calculateTarget();
    robotMoving = true;

    Serial.println();
    Serial.println(">>> DIAGONAL: MUNDUR + X POSITIF");
    Serial.print("V1 = "); Serial.println(V1, 4);
    Serial.print("V2 = "); Serial.println(V2, 4);
    Serial.print("V3 = "); Serial.println(V3, 4);
}

// G = MUNDUR + X NEGATIF
else if (command == 'g' || command == 'G')
{
    linear_x = -0.3536;
    linear_y = -0.3536;
    angular_z = 0.0;

    resetAllPID();
    inverseKinematics();
    calculateTarget();
    robotMoving = true;

    Serial.println();
    Serial.println(">>> DIAGONAL: MUNDUR + X NEGATIF");
    Serial.print("V1 = "); Serial.println(V1, 4);
    Serial.print("V2 = "); Serial.println(V2, 4);
    Serial.print("V3 = "); Serial.println(V3, 4);
}
        // =============================================
        // PUTAR KIRI
        // =============================================

        else if (command == 'a' || command == 'A')
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

            Serial.print("V1 = ");
            Serial.println(V1, 4);

            Serial.print("V2 = ");
            Serial.println(V2, 4);

            Serial.print("V3 = ");
            Serial.println(V3, 4);
        }


        // =============================================
        // PUTAR KANAN
        // =============================================

        else if (command == 'd' || command == 'D')
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

            Serial.print("V1 = ");
            Serial.println(V1, 4);

            Serial.print("V2 = ");
            Serial.println(V2, 4);

            Serial.print("V3 = ");
            Serial.println(V3, 4);
        }


        // =============================================
        // STOP
        // =============================================
        // V vx vy w s  -> perintah dari ROS2 (serial bridge)
        else if (command == 'v' || command == 'V')
        {
            float vx = Serial.parseFloat();
            float vy = Serial.parseFloat();
            float w  = Serial.parseFloat();
            float s  = Serial.parseFloat();

            bool allZero = (fabsf(vx) < 0.0001f &&
                            fabsf(vy) < 0.0001f &&
                            fabsf(w)  < 0.0001f);

            if (allZero || s < 0.01f)
            {
                stopRobot();
            }
            else
            {
                if (!robotMoving)
                    resetAllPID();

                linear_x  = vx;
                linear_y  = vy;
                angular_z = w   ;
                speedScale = constrain(s, 0.0f, 1.0f);

                inverseKinematics();
                calculateTarget();

                robotMoving = true;
                remoteMode = true;
                lastRemoteCmd = millis();
            }
        }

        else if (command == 'x' || command == 'X')
        {
            stopRobot();

            Serial.println();
            Serial.println(">>> STOP");
        }
    }
}
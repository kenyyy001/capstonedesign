#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <Adafruit_BNO055.h>


#define DEBUG_MONITOR 0
const bool ENABLE_ENCODER_PID = true;

const float VX_SIGN = +1.0f;
const float VY_SIGN = -1-1.0f;
const float W_SIGN = +1.0f;
const float HEADING_SIGN = +1.0f;

const uint8_t IN_A[3] = {23, 18, 13};
const uint8_t IN_B[3] = {19, 4, 2};
const int MOTOR_POL[3] = {+1, +1, -1};

#define ENC1_A 36
#define ENC1_B 39
#define ENC2_A 34
#define ENC2_B 35
#define ENC3_A 32
#define ENC3_B 33

#define PWM_FREQ 5000
#define PWM_RESOLUTION 8

if ESP_ARDUINO_VERSION_MAJOR >= 3
    #define PWM_ATTACH(pin, ch) ledcAttach(pin, PWM_FREQ, PWM_RESOLUTION)
    #define PWM_WRITE(pin, ch, value) ledcWrite(pin, duty)

else 
    #define PWM_ATTACH(pin, ch) do { ledcSetup(ch, PWM_FREQ, PWM_RESOLUTION); ledcAttachPin(pin, ch); } while(0)
    #define PWM_WRITE(pin, ch, duty) ledcWrite(ch, duty)
endif


const int PPR = 1640;
const float WHEEL_RADIUS_CM = 2.9f;
const float CM_PER_PULSE = (2.0f * PI * WHEEL_RADIUS_CM) / PPR;

volatile long encCount[3] = {0, 0, 0};
float speedFilt[3] = {0, 0, 0};
const float SPEED_ALPHA = 0.4f;

void IRAM_ATTR encoderISR1() {
    long c = encCount[0];
    encCount[0] = ((digitalRead(ENC1_A) == HIGH) != (digitalRead(ENC1_B) == LOW)) ? c-1 : c + 1;
    
}

void IRAM_ATTR encoderISR2(){
    long c = encCount[1];
    encCount[1] = ((digitalRead(ENC2_A) == HIGH) != (digitalRead(ENC2_B) == LOW)) ? c - 1 : c + 1;

}

void IRAM_ATTR encoderISR3() {
    long c = encCount[2];
    encCount[2] = ((digitalRead(ENC3_A) == HIGH) != (digitalRead(ENC3_B) == LOW)) ? c - 1 : c + 1;
}

void updateEncoderSpeed(float dt) {
    long c[3];
    noInterrupts();
    for (int i = 0; i < 3; i++) { c[i] = encCount[i]; encCount[i] = 0;}
    interrupts();
    for (int i = 0; i < 3; i++) {
        float raw = fabsf((float)c[i]) * CM_PER_PULSE / dt;
        speedFilt[i] += SPEED_ALPHA * (raw - speedFilt[i]);
    }
}

const float matrix_kecepatan[9] = {
    -0.333f, 0.5774f, 0.0317f,
    -0.333f, -05774f, 0.0317f,
    0.6667f, 0.0000f, 0.0317f

};

const float MAX_WHEEL_SPEED = 5.0f;
const int PWM_AT_MAX_SPEED = 80;

const float LIN_CMD_FULL = 0.35f;
const float GAIN_LIN = MAX_WHEEL_SPEED / (0.5774f * LIN_CMD_FULL);

const float ROT_CMD_FULL = 0.6f;
const float ROT_WHEEL_SPEED_AT_FULL = 4.0f;
const float GAIN_ROT = ROT_WHEEL_SPEED_AT_FULL / (0.0317f * ROT_CMD_FULL);


void inverseKinematics(float vx, float vy, float w, float scale, float out[3]) {
    const float* m = matrix_kecepatan;
    float v3 = GAIN_LIN * (m[0] * vx + m[1] * vy) + GAIN_ROT * (m[2] * w);
    float v2 = GAIN_LIN * (m[3] * vx + m[4] * vy) + GAIN_ROT * (m[5] * w);
    float v1 = GAIN_LIN * (m[6] * vx + m[7] * vy) + GAIN_ROT * (m[8] * w);
    out[0] = v1; out[1] = v2; out[2] = v3;

    float limit = MAX_WHEEL_SPEED * scale;
    float peak = fmaxf(fabsf(v1), fmaxf(fabsf(v2), fabsf(v3)));
    if (peak > limit && peak > 0.0001f) {
        float k = limit / peak;
        for (int i = 0; i < 3; i++) out[i] *= k;
    }
}


const int PWM_MIN_INA[3] = {75, 50, 60};
const int PWM_MIN_INB[3] = {55, 50, 60};

const float WHEEL_ACCEL = 12.0f;
const float WHEEL_DECEL = 15.0f;
const float V_START = 0.6f;
const float V_STOP  = 0.3f;

float goalWheel[3] = {0, 0, 0};
float rampWheel[3] = {0, 0, 0};
bool wheelOn[3]    = {false, false, false};

struct PID { float Kp, Ki, Kd, integral, lastError; };
PID pidWheel[3] = { {1.5f, 00.1f, 0.0f, 0, 0}, {1.5f, 0.1f, 0.0f, 0, 0}, {1.5f, 0.1f, 0.0f, 0, 0}}
const float PID_LIMIT = 15.0f;

void resetPID(PID &p) { p.integral = 0; p.lastError = 0; }

float updatePID(PID &p, float target, float actual, float dt) {
    float err = target - actual;
    p.integral = constrain(p.integral + err * dt, -30.0f, 30.0f);
    float d = (err - p.lastError) / dt;
    p.;astError = err;
    return constrain(p.Kp * err + p.Ki * p.integral + p.Kd * d, -PID_LIMIT, PID_LIMIT);
}

float slew(float cur, float target, float dt) {
    bool away = fabsf(target) > fabsf(cur) && target * cur >= 0.0f;
    float lim = (away ? WHEEL_ACCEL : WHEEL_DECEL) * dt;
    return cur + constrain(target - cur, -lim, lim);
}

void setMotorPWM(int i, float signedPWM) {
    int pwm = constrain((int)fabsf(signedPWM), 0, 255);
    if (signedPWM > 0.0f) {
        PWM_WRITE(IN_B[i], 2 * i + 1, 0);
        PWM_WRITE(IN_A[i], 2 * i, pwm);
    }
    else if (signedPWM < 0.0f) {
        PWM_WRITE(IN_A[i], 2 * i, 0);
        PWM_WRITE(IN_B[i], 2 * i + 1, pwm);

    }
    else {
        PWM_WRITE(IN_A[i], 2 * i, 0);
        PWM_WRITE(IN_B[i], 2 * i 1, 0);
    }
}

void updateWheel(int i, float dt) {
    float target = rampWheel[i];
    float mag = fabsf(target);

    if (!wheelOn[i] && mag > V_START) wheelOn[i] = true;
    else if (wheelOn[i] && mag < V_STOP) wheelOn[i] = false;

    if(!wheelOn[i]) {
        setMotorPWM(i, 0);
        resetPID(pidWheel[i]);
        return;
    }
     int dir = (target > 0.0f) ? 1 : -1;
     int elec = dir * MOTOR_POL[i];
     int minPWM = (elec > 0) ? PWM_MIN_INA[i] : PWM_MIN_INB[i];

     float ratio - constrain(mag / MAX_WHEEL_SPEED, 0.0f, 1.0f);
     float pwm = minPWM + ratio * (PWM_AT_MAX_SPEED - minPWM);
     if (ENABLE_ENCODER_PID) pwm += updatePID(pidWheel[i], mag, speedFilt[i], dt);
     pwm = constrain(pwm, (float)minPWM, 255.0f);

     setMotorPWM(i, elec, * pwm);
}

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
bool  bnoReady = false;
float currentYaw = 0.0f, headingTarget = 0.0f;
bool  headingLocked = false;
PID   headingPID = {2.0f, 0.0f, 0.5f, 0, 0};
const float HEADING_PID_LIMIT = 0.3f;

float angleDiffDeg(float target, float current) {
    float d = target - current;
    while (d > 180.0f)  d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}

void updateYaw() {
    if (!bnoReady) return;
    sensors_event_t ev;
    bno.getEvent(&ev, Adafruit_BNO055::VECTOR_EULER);
    currentYaw = ev.orientation.x;
}

float updateHeadingPID(float errDeg, float dt) {
    headingPID.integral = constrain(headingPID.integral + errDeg * dt, -50.0f, 50.0f);
    float d = (errDeg - headingPID.lastError) / dt;
    headingPID.lastError = errDeg;
    float out = (headingPID.Kp * errDeg + headingPID.Ki * headingPID.integral + headingPID.Kd * d) * 0.01f;
    return constrain(out, -HEADING_PID_LIMIT, HEADING_PID_LIMIT);
}

float cmdX = 0, cmdY = 0, cmdW = 0, cmdScale = 1.0f;   
bool  remoteMode = false;
unsigned long lastRemoteCmd = 0;
const unsigned long REMOTE_TIMEOUT_MS = 500;

const float KEY_LIN = 0.35f;
const float KEY_ROT = 0.5f;

void stopRobot() {                       
    cmdX = cmdY = cmdW = 0.0f;
    cmdScale = 1.0f;
    for (int i = 0; i < 3; i++) {
        goalWheel[i] = 0; rampWheel[i] = 0; wheelOn[i] = false;
        resetPID(pidWheel[i]);
        setMotorPWM(i, 0);
    }
    headingLocked = false;
    remoteMode = false;
}

void setCmd(float x, float y, float w) {
    cmdX = x; cmdY = y; cmdW = w; cmdScale = 1.0f;
    remoteMode = false;
}

void keyboardCommand(char c) {
    const float L = KEY_LIN, D = KEY_LIN * 0.7071f, R = KEY_ROT;
    switch (c) {
        case 'w': setCmd(0,  L, 0); break;
        case 's': setCmd(0, -L, 0); break;
        case 'q': setCmd( L, 0, 0); break;
        case 'e': setCmd(-L, 0, 0); break;
        case 'r': setCmd( D,  D, 0); break;
        case 't': setCmd(-D,  D, 0); break;
        case 'f': setCmd( D, -D, 0); break;
        case 'g': setCmd(-D, -D, 0); break;
        case 'a': setCmd(0, 0,  R); break;
        case 'd': setCmd(0, 0, -R); break;
        case 'x': stopRobot(); break;
        default: return;
    }
    Serial.printf(">>> %c\n", c);
}

void handleV(float vx, float vy, float w, float s) {
    s = constrain(s, 0.0f, 1.0f);
    bool zero = (fabsf(vx) < 1e-4f && fabsf(vy) < 1e-4f && fabsf(w) < 1e-4f) || s < 0.01f;
    if (zero) {
        cmdX = cmdY = cmdW = 0.0f;
        cmdScale = 1.0f;
    } else {
        cmdX = VX_SIGN * vx;
        cmdY = VY_SIGN * vy;
        cmdW = W_SIGN  * w;
        cmdScale = s;
    }
    remoteMode = true;
    lastRemoteCmd = millis();
}

char rxBuf[64];
uint8_t rxLen = 0;
unsigned long rxLast = 0;

void flushMessage() {
    if (rxLen == 0) return;
    rxBuf[rxLen] = 0;
    float v[4];
    int n = 0;
    char *p = rxBuf + 1;                 
    while (n < 4) {
        while (*p == ' ' || *p == ',' || *p == '\t') p++;
        if (!*p) break;
        char *end;
        float val = strtof(p, &end);
        if (end == p) break;
        v[n++] = val;
        p = end;
    }
    if (n == 4) handleV(v[0], v[1], v[2], v[3]);
    rxLen = 0;
}

void readSerial() {
    while (Serial.available()) {
        char ch = Serial.read();
        rxLast = millis();
        if (ch == '\n' || ch == '\r') { flushMessage(); continue; }

        if (ch == 'v' || ch == 'V') {            
            flushMessage();
            rxBuf[rxLen++] = 'v';
        } else if (rxLen > 0) {                  
            if (rxLen < sizeof(rxBuf) - 1) rxBuf[rxLen++] = ch;
        } else if (isalpha((unsigned char)ch)) {
            keyboardCommand(tolower(ch));
        }
    }
    if (rxLen > 0 && millis() - rxLast > 10) flushMessage();
}

void controlStep(float dt) {
    float w = cmdW;
    bool translating = (fabsf(cmdX) + fabsf(cmdY)) > 0.02f;
    if (bnoReady && translating && fabsf(cmdW) < 0.01f) {
        if (!headingLocked) {
            headingTarget = currentYaw;
            headingLocked = true;
            resetPID(headingPID);
        }
        w = HEADING_SIGN * updateHeadingPID(angleDiffDeg(headingTarget, currentYaw), dt);
    } else {
        headingLocked = false;
    }

    inverseKinematics(cmdX, cmdY, w, cmdScale, goalWheel);

    for (int i = 0; i < 3; i++) {
        rampWheel[i] = slew(rampWheel[i], goalWheel[i], dt);
        updateWheel(i, dt);
    }
}

void setup() {
    Serial.begin(115200);

    Wire.begin(21, 22);
    if (bno.begin()) {
        bno.setExtCrystalUse(true);
        bnoReady = true;
    }

    pinMode(ENC1_A, INPUT); pinMode(ENC1_B, INPUT);
    pinMode(ENC2_A, INPUT); pinMode(ENC2_B, INPUT);
    pinMode(ENC3_A, INPUT); pinMode(ENC3_B, INPUT);
    attachInterrupt(digitalPinToInterrupt(ENC1_A), encoderISR1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC2_A), encoderISR2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC3_A), encoderISR3, CHANGE);

    for (int i = 0; i < 3; i++) {
        PWM_ATTACH(IN_A[i], 2 * i);
        PWM_ATTACH(IN_B[i], 2 * i + 1);
    }
    stopRobot();

    Serial.println(bnoReady ? "BNO055 OK" : "BNO055 TIDAK terdeteksi (SDA=21 SCL=22)");
    Serial.println("Siap. Keyboard: w s q e r t f g a d x | ROS2: V vx vy w s");
}

void loop() {
    readSerial();

    unsigned long now = millis();
    static unsigned long lastTick = 0;
    if (now - lastTick >= 50) {
        float dt = constrain((now - lastTick) / 1000.0f, 0.001f, 0.2f);
        lastTick = now;
        updateEncoderSpeed(dt);
        updateYaw();
        controlStep(dt);

if DEBUG_MONITOR
        static unsigned long lastPrint = 0;
        if (now - lastPrint >= 250) {
            lastPrint = now;
            Serial.printf("cmd %.2f %.2f %.2f | goal %.1f %.1f %.1f | spd %.1f %.1f %.1f | yaw %.0f\n",
                          cmdX, cmdY, cmdW, goalWheel[0], goalWheel[1], goalWheel[2],
                          speedFilt[0], speedFilt[1], speedFilt[2], currentYaw);
        }
endif

    }

    if (remoteMode && (now - lastRemoteCmd > REMOTE_TIMEOUT_MS)) {
        stopRobot();
    }
}
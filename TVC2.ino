#include <Arduino.h>
#include <math.h>
#include <HardwareSerial.h>
#include "States.h"
#include "IMU.h"
#include "Baro.h"
#include "ServoController.h"
#include "Pid.h"
#include "Orientation.h"

// ===== FLIGHT STATE =====
float gimbalAngleX = 0;
float gimbalAngleY = 0;
Quaternion q = {1,0,0,0};

// ===== PID CONTROLLERS =====
PID pidPitch;
PID pidRoll;

float pitch;
float roll;
float prevPitch = 0;
float prevRoll = 0;

// ===== LOGGING =====
SdFat SD;
File logFile;
float pressureReadings[100];
bool pressureReadingsFull = false;
int pressureReadingsCounter = 0;

// ===== ALTITUDE DETECTION (BAROMETER) =====
float lastPressureReading = NAN;

// ===== LED =====
constexpr int LED_PIN = PB0;
namespace {
    unsigned long ledPrevMicros = 0;
    bool ledState = false;
}

Vector3 gyroFiltered = {0.0 ,0.0 ,0.0};
Vector3 cachedGyro = {0.0, 0.0, 0.0};
Vector3 cachedAccel = {00, 0.0, 0.0};
float cachedPressure = 0.0f;

// ===== FLIGHT STATE MACHINE =====
enum FlightState {
    PRELAUNCH,
    POWERED,
    APOGEE,
    DESCENT,
    LANDED
};

FlightState flightState;
int servoX = PB8;
int servoY = PB9;
unsigned long prevMicros = 0;
HardwareSerial Serial2(USART2);

// ===== LED CONTROL FUNCTIONS =====
void ledOn() {
    digitalWrite(LED_PIN, HIGH);
    ledState = true;
}

void ledOff() {
    digitalWrite(LED_PIN, LOW);
    ledState = false;
}

void blinkLed(const char* speed) {
    unsigned long interval;

    if (strcmp(speed, "fast") == 0) {
        interval = 200000;
    } else if (strcmp(speed, "slow") == 0) {
        interval = 800000;
    } else {
        interval = 500000;
    }

    unsigned long now = micros();
    if (now - ledPrevMicros >= interval) {
        ledPrevMicros = now;
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState ? HIGH : LOW);
    }
}

// ===== STATE TRANSITION =====
void transitionToState(FlightState newState) {
    flightState = newState;
    Serial2.print("State -> ");

    switch (newState) {
        case PRELAUNCH: Serial2.println("PRELAUNCH"); break;
        case POWERED:   Serial2.println("POWERED");   break;
        case APOGEE:    Serial2.println("APOGEE");    break;
        case DESCENT:   Serial2.println("DESCENT");   break;
        case LANDED:    Serial2.println("LANDED");    break;
    }
}

void setup() {
    Serial2.begin(115200);

    pidRoll  = createPID(KP, KI, KD);
    pidPitch = createPID(KP, KI, KD);

    initIMU();
    if (!initBaro()) {
        Serial2.println("Baro init failed!");
        while (1) { delay(1000); }
    }

    initServos(servoX, servoY);
    pinMode(LED_PIN, OUTPUT);

    prevMicros = micros();

    if (!SD.begin(PA4, SD_SCK_MHZ(4))) {
        Serial2.println("SD init failed!");
        while (1);
    }

    SD.remove("flight.csv");
    logFile = SD.open("flight.csv", FILE_WRITE);
    if (logFile) {
        logFile.println("Micros,State,q.w,q.x,q.y,q.z,Yaw,Pitch,Roll,GyroX,GyroY,GyroZ,AccX,AccY,AccZ,Pressure,GimbalX,GimbalY");
        logFile.close();
    }

    transitionToState(PRELAUNCH);
}

void loop() {
    unsigned long currentMicros = micros();
    float dt = (currentMicros - prevMicros) / 1e6f;
    if (dt <= 0.0f || dt > 0.1f) {
        dt = 0.001f;
    }
    prevMicros = currentMicros;
     
    cachedPressure = getPressure();

    if (flightState == PRELAUNCH || flightState == POWERED || flightState == DESCENT) {
        cachedAccel = getAccData();
        cachedGyro = getGyroData();
    }

    switch (flightState) {
        case PRELAUNCH:
            setTiltRadians(0, 0);
            blinkLed("slow");
            if (checkPrelaunchToPowered(cachedAccel)) transitionToState(POWERED);
            break;

        case POWERED:
            ledOn();
            poweredAscentState(currentMicros, dt, cachedGyro, cachedAccel, cachedPressure);
            if (checkPoweredToApogee(cachedPressure)) transitionToState(APOGEE);
            break;

        case APOGEE:
            chutesOut();
            transitionToState(DESCENT);
            break;

        case DESCENT:
            blinkLed("fast");
            descentState(currentMicros, dt, cachedGyro, cachedAccel, cachedPressure);
            break;
    }
}
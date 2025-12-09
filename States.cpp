#include "States.h" 
#include "IMU.h"
#include "Baro.h"
#include "Pid.h"
#include "ServoController.h"
#include "Orientation.h"
#include <math.h>
#include <HardwareSerial.h>

// Extern globals from quat_tvc.ino
extern HardwareSerial Serial2;

// Helper to print telemetry data (common for all states)
void printTelemetry(float pitch, float roll, float gimbalAngleX, float gimbalAngleY) {
    Serial2.print("Pitch(deg): "); Serial2.print(degrees(pitch), 2);
    Serial2.print("  Roll(deg): "); Serial2.print(degrees(roll), 2);
    Serial2.print("  gX(deg): "); Serial2.print(degrees(gimbalAngleX), 2);
    Serial2.print("  gY(deg): "); Serial2.println(degrees(gimbalAngleY), 2);
}

// Unified logging function
void logFlightData(unsigned long currentMicros, const char* stateName, 
                   Quaternion q_heading, float yaw, float pitch, float roll,
                   Vector3 gyro, Vector3 acc, float pressure,
                   float gimbalX, float gimbalY) {
    static unsigned long prevLogMicros = 0;
    
    if (currentMicros - prevLogMicros < LOG_INTERVAL_US) return;
    prevLogMicros = currentMicros;

    logFile = SD.open("flight.csv", FILE_WRITE);
    if (!logFile) return;

    logFile.print(currentMicros); logFile.print(",");
    logFile.print(stateName); logFile.print(",");
    logFile.print(q_heading.w, 6); logFile.print(",");
    logFile.print(q_heading.x, 6); logFile.print(",");
    logFile.print(q_heading.y, 6); logFile.print(",");
    logFile.print(q_heading.z, 6); logFile.print(",");
    logFile.print(degrees(yaw), 4); logFile.print(",");
    logFile.print(degrees(pitch), 4); logFile.print(",");
    logFile.print(degrees(roll), 4); logFile.print(",");
    logFile.print(gyro.x, 4); logFile.print(",");
    logFile.print(gyro.y, 4); logFile.print(",");
    logFile.print(gyro.z, 4); logFile.print(",");
    logFile.print(acc.x, 4); logFile.print(",");
    logFile.print(acc.y, 4); logFile.print(",");
    logFile.print(acc.z, 4); logFile.print(",");
    logFile.print(pressure, 2); logFile.print(",");
    logFile.print(gimbalX, 4); logFile.print(",");
    logFile.println(gimbalY, 4);
    logFile.close();
}

// ---------------------------
// --- Powered Ascent ---
// ---------------------------
void poweredAscentState(unsigned long currentMicros, float dt, Vector3 cachedGyro, Vector3 cachedAccel, float cachedPressure) {

    gyroFiltered = lowPass(cachedGyro, gyroFiltered, ALPHA);
    integrateGyro(q, gyroFiltered, dt);
    getPitchRoll(q, pitch, roll);

    prevPitch = pitch;
    prevRoll  = roll;

    gimbalAngleX = limitAngle(updatePID(pidPitch, pitch, dt), radians(GIMBAL_LIMIT_DEG));
    gimbalAngleY = limitAngle(updatePID(pidRoll, roll, dt), radians(GIMBAL_LIMIT_DEG));
    setTiltRadians(gimbalAngleX, gimbalAngleY);

    // printTelemetry(pitch, roll, gimbalAngleX, gimbalAngleY);
    logFlightData(currentMicros, "Ascent", q, 0, degrees(pitch), degrees(roll), cachedGyro, cachedAccel, cachedPressure, gimbalAngleX, gimbalAngleY);
}

// ---------------------------
// --- Descent ---
// ---------------------------§
void descentState(unsigned long currentMicros, float dt, Vector3 cachedGyro, Vector3 cachedAccel, float cachedPressure) {

    gyroFiltered = lowPass(cachedGyro, gyroFiltered, ALPHA);
    integrateGyro(q, gyroFiltered, dt);
    getPitchRoll(q, pitch, roll);

    prevPitch = pitch;
    prevRoll  = roll;

    // printTelemetry(pitch, roll, 0, 0);
    logFlightData(currentMicros, "Descent", q, 0, pitch, roll, cachedGyro, cachedAccel, cachedPressure, 0, 0);
}


/*** Transition checks ***/
bool checkPrelaunchToPowered(Vector3 cachedAccel) {
    const float THRESH_G = 1.5f;
    const unsigned long HOLD_US = 50000UL;
    static unsigned long accelAboveSince = 0;

    Vector3 a = cachedAccel;
    float mag = sqrtf(a.x * a.x + a.y * a.y + a.z * a.z);

    if (mag > THRESH_G) {
        if (accelAboveSince == 0) {
            accelAboveSince = micros();
        } else {
            if (micros() - accelAboveSince >= HOLD_US) {
                accelAboveSince = 0;
                return true;
            }
        }
    } else {
        accelAboveSince = 0;
    }

    return false;
}
/***  
Uses a circular array to determine the apogee.
Stores air pressure values 10 ms apart until full.
Then starts checking from the beginning if the current
value is greater (altitude lower) than 1 s ago.
***/
bool checkPoweredToApogee(float cachedPressure) {
    static unsigned long lastSampleTime = 0;
    unsigned long now = millis();

    if (now - lastSampleTime >= 10) {

        lastSampleTime = now;

        // Fill the array of pressure readings
        if (!pressureReadingsFull) {
            pressureReadings[pressureReadingsCounter] = cachedPressure;
            pressureReadingsCounter += 1;
            if (pressureReadingsCounter >= 100) {
                pressureReadingsFull = true;
            } 
            return false;
        }

        // The array is used circularly
        if (pressureReadingsCounter > 99) {
            pressureReadingsCounter = 0;
        }

        if (cachedPressure > (pressureReadings[pressureReadingsCounter]+0.1)) {
            return true;
        } else {
            pressureReadings[pressureReadingsCounter] = cachedPressure;
            pressureReadingsCounter += 1;
        }
    }
    return false;
}

bool checkApogeeToDescent() {
    return true;
}

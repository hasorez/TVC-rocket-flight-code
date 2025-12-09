// GlobalDeclarations.h
#ifndef GLOBALS_H
#define GLOBAlS_H

#include <SdFat.h>
#include "Orientation.h"
#include "Pid.h"

constexpr float KP = 0.4;
constexpr float KI = 0.1;
constexpr float KD = 0.15;
constexpr unsigned long LOG_INTERVAL_US = 10000;
constexpr float GIMBAL_LIMIT_DEG = 4.0f;
constexpr float ALPHA = 0.5f;

extern Quaternion q;
extern float gimbalAngleX, gimbalAngleY;
extern Vector3 gyroFiltered;
extern PID pidPitch;
extern PID pidRoll;
extern float pitch;
extern float roll;
extern float prevPitch;
extern float prevRoll;
extern SdFat SD;
extern File logFile;
// extern unsigned long prevLogMicros;
extern float lastPressureReading;

extern float pressureReadings[100];
extern bool pressureReadingsFull;
extern int pressureReadingsCounter;

// LED
void blinkLed(const char* speed);
void ledOn();
void ledOff();

#endif
#ifndef BARO_H
#define BARO_H

bool initBaro();
void calibrateGround();
float getPressure();
float getTemperature();
float getAltitude();

#endif  // IMU_INTERFACE_H
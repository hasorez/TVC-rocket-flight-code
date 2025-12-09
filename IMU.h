#ifndef IMU_H
#define IMU_H

#include "Orientation.h"

// === Function Declarations ===
void initIMU();
Vector3 getGyroData();
Vector3 getAccData();

#endif  // IMU_INTERFACE_H

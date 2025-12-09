// servo.cpp
#include "ServoController.h"
#include <Arduino.h>

static Servo servoX;
static Servo servoY;
static Servo servoZ;

void initServos(int pinX, int pinY) {
  servoX.attach(pinX);
  servoY.attach(pinY);
  servoZ.attach(PA10);
  servoZ.write(0);
}
// w=pivot to attachments
// r=servo horn wheel radius
void setTiltRadians(float tiltX_rad, float tiltY_rad, float r, float w, float R_tube, float offsetX_deg, float offsetY_deg) {
  // 1) Compute cable‐length change for X‐servo (rotation about Y):
    // ΔxA = R_tube*(cos(tiltX_rad) - 1) + w*sin(tiltX_rad)
  float dxA = R_tube * (cos(tiltX_rad) - 1.0) + w * sin(tiltX_rad);

  // 2) Compute cable‐length change for Y‐servo (rotation about X):
    //  ΔyA = R_tube*(cos(tiltY_rad) - 1) - w*sin(tiltY_rad)
  float dyA = R_tube * (cos(tiltY_rad) - 1.0) - w * sin(tiltY_rad);

  // 3) Convert each Δ into a servo‐wheel angle (phi) via r*sin(phi) = Δ
  float vx = dxA / r;
  float vy = dyA / r;

  // Clamp to [–1..+1] so asin( ) is valid
  vx = constrain(vx, -1.0, +1.0);
  vy = constrain(vy, -1.0, +1.0);

  float phi_x = asin(vx);  // radians
  float phi_y = asin(vy);  // radians

  // 4) Convert phi → “servo degrees” (with 90° = horn straight‐in)
  float servoX_deg = 90.0 + (phi_x * 180.0 / PI) + offsetX_deg;
  float servoY_deg = 90.0 + (phi_y * 180.0 / PI) + offsetY_deg;

  // 5) Constrain to [0…180]
  servoX_deg = constrain(servoX_deg, 0.0, 180.0);
  servoY_deg = constrain(servoY_deg, 0.0, 180.0);

  servoX.write((int)round(servoX_deg));
  servoY.write((int)round(servoY_deg));
}

void chutesOut() {
  servoZ.write(90);
}

float limitAngle(float angle, float limit)  { 
    if (angle > limit) return limit; 
    if (angle < -limit) return -limit; 
    return angle; 
}
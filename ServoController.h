#ifndef SERVO_H
#define SERVO_H

#include <Servo.h>

void initServos(int pinX, int pinY);
// w=pivot to attachments
// r=servo horn wheel radius
void setTiltRadians(float tiltX_rad, float tiltY_rad, float r=13.0, float w=75.0, float R_tube=20.0, float offsetX_deg=0, float offsetY_deg=0);
void chutesOut();
float limitAngle(float angle, float limit);
#endif

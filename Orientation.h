#ifndef ORIENTATION_H
#define ORIENTATION_H

#include <Arduino.h>
#include <math.h>

// ---------------------------
// --- Data structures ---
// ---------------------------
struct Vector3 { float x, y, z; };
struct Quaternion { float w, x, y, z; };

// ---------------------------
// --- Global orientation ---
// ---------------------------
extern Quaternion q;        // current orientation
extern Vector3 gyroFiltered; // optional low-pass filtered gyro

// ---------------------------
// --- Quaternion helpers ---
// ---------------------------
Quaternion quatMul(const Quaternion& q1, const Quaternion& q2);
Quaternion normalizeQuat(const Quaternion& q);

// ---------------------------
// --- Gyro integration ---
// ---------------------------
void integrateGyro(Quaternion &q, Vector3 gyro, float dt);

// ---------------------------
// --- Pitch/Roll extraction ---
// ---------------------------
void getPitchRoll(Quaternion &q, float &pitch, float &roll);

// ---------------------------
// --- Optional gyro LPF ---
// ---------------------------
Vector3 lowPass(Vector3 input, Vector3 prev, float alpha);

// ---------------------------
// --- Update helper ---
// ---------------------------
void update(float dt, Vector3 gyroRaw, float &pitch, float &roll, float alpha);

#endif

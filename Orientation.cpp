#include <Arduino.h>
#include <math.h>

// ---------------------------
// --- Data structures ---
// ---------------------------
struct Vector3 { float x, y, z; };
struct Quaternion { float w, x, y, z; };

Quaternion quatMul(const Quaternion& q1, const Quaternion& q2) {
    return {
        q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z,
        q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y,
        q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x,
        q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w
    };
}

Quaternion normalizeQuat(const Quaternion& q) {
    float mag = sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    if (mag == 0.0f) return {1,0,0,0};
    return {q.w/mag, q.x/mag, q.y/mag, q.z/mag};
}

void integrateGyro(Quaternion &q, Vector3 gyro, float dt) {
    // gyro in rad/s
    Quaternion omega = {0, gyro.x, gyro.y, gyro.z};
    Quaternion q_dot = quatMul(q, omega);
    q_dot.w *= 0.5f; 
    q_dot.x *= 0.5f; 
    q_dot.y *= 0.5f; 
    q_dot.z *= 0.5f;

    q.w += q_dot.w*dt;
    q.x += q_dot.x*dt;
    q.y += q_dot.y*dt;
    q.z += q_dot.z*dt;

    q = normalizeQuat(q);
}

void getPitchRoll(Quaternion &q, float &pitch, float &roll) {
    pitch = asin(-2.0f*(q.x*q.z - q.w*q.y));  // radians
    roll  = atan2(2.0f*(q.y*q.z + q.w*q.x), 1 - 2*(q.x*q.x + q.y*q.y));
}

Vector3 lowPass(Vector3 input, Vector3 prev, float alpha) {
    prev.x = alpha*input.x + (1-alpha)*prev.x;
    prev.y = alpha*input.y + (1-alpha)*prev.y;
    prev.z = alpha*input.z + (1-alpha)*prev.z;
    return prev;
}
#include "PID.h"
#include <algorithm>  // for std::clamp

PID createPID(float kp, float ki, float kd) {
  return PID(kp, ki, kd);
}

float updatePID(PID& pid, float error, float dt) {
  if (dt <= 0.0f) return 0.0f; // avoid division by zero

  // --- Integrator with anti-windup ---
  pid.integral += error * dt;
  pid.integral = std::clamp(pid.integral, pid.integrator_min, pid.integrator_max);

  // --- Derivative with low-pass filter ---
  // raw derivative
  float raw_derivative = (error - pid.prev_error) / dt;

  // filtered derivative using 1st-order low-pass
  float alpha = pid.tau / (pid.tau + dt);
  float derivative = alpha * pid.prev_derivative + (1.0f - alpha) * raw_derivative;

  pid.prev_derivative = derivative;
  pid.prev_error = error;

  // --- PID output ---
  float output = pid.kp * error + pid.ki * pid.integral + pid.kd * derivative;
  return output;
}

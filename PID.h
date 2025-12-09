#ifndef PID_H
#define PID_H

struct PID {
  float kp, ki, kd;
  float integral;
  float prev_error;
  float prev_derivative;
  float tau;              // derivative filter time constant
  float integrator_min;
  float integrator_max;

  PID()
      : kp(0), ki(0), kd(0),
        integral(0), prev_error(0), prev_derivative(0),
        tau(0.1f),                
        integrator_min(-0.3f),      // default anti-windup limits
        integrator_max(0.3f) {}

  PID(float p, float i, float d)
      : kp(p), ki(i), kd(d),
        integral(0), prev_error(0), prev_derivative(0),
        tau(0.1f),
        integrator_min(-0.3f), integrator_max(0.3f) {}
};

PID createPID(float kp, float ki, float kd);
float updatePID(PID& pid, float error, float dt);

#endif

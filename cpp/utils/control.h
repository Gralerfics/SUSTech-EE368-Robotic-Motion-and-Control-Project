#ifndef CONTROL_H
#define CONTROL_H

#include <chrono>

#include "compute.h"

template <typename T>
class PIDController {
private:
    float kp;
    float ki;
    float kd;
    
    time_t last_time;
    T integral_error, last_error;

public:
    PIDController(float kp, float ki, float kd): kp(kp), ki(ki), kd(kd) {}
    
    ~PIDController() {}

    void Init();
    T Step(T error);
    T Step(T error, T error_dot);
};

#endif
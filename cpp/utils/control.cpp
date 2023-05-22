#include "control.h"

template <typename T>
void PIDController<T>::Init() {
    last_time = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
    integral_error = T::Zero();
    last_error = T::Zero();
}

template <typename T>
T PIDController<T>::Step(T error) {
    time_t current_time = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
    time_t dt_us = current_time - last_time;
    float dt = dt_us / 1000000.0;
    last_time = current_time;

    T P = kp * error;

    integral_error += error * dt;
    T I = ki * integral_error;

    T D = kd * (error - last_error) / dt;
    last_error = error;

    return P + I + D;
}

template <typename T>
T PIDController<T>::Step(T error, T error_dot) {
    time_t current_time = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
    time_t dt_us = current_time - last_time;
    float dt = dt_us / 1000000.0;
    last_time = current_time;

    T P = kp * error;

    integral_error += error * dt;
    T I = ki * integral_error;

    T D = kd * error_dot;

    return P + I + D;
}

template class PIDController<Vector3f>;
template class PIDController<Vector2f>;

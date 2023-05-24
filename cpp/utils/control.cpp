#include "control.h"

template <typename T>
void PIDController<T>::Init() {
    // last_time = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
    integral_error = T::Zero();
    last_error = T::Zero();
}

template <typename T>
T PIDController<T>::Step(float dt, T error) {
    T P = kp * error;

    integral_error += error * dt;
    T I = ki * integral_error;

    T D = kd * (error - last_error) / dt;
    last_error = error;

    return P + I + D;
}

template <typename T>
T PIDController<T>::Step(float dt, T error, T error_dot) {
    T P = kp * error;

    integral_error += error * dt;
    T I = ki * integral_error;

    T D = kd * error_dot;

    return P + I + D;
}

template class PIDController<Vector3f>;
template class PIDController<Vector2f>;

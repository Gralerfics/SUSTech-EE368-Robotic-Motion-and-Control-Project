#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "compute.h"

class Trajectory {
protected:
    float period, t;

public:
    Trajectory(float period, float t): period(period), t(t) {}
    ~Trajectory() {}

    void SetTime(float t) { this->t = t; }
    float GetTime() { return t; }
    void SetPeriod(float period) { this->period = period; }
    float GetPeriod() { return period; }

    virtual Vector3f GetPosition() = 0;
    virtual Vector3f GetVelocity() = 0;
    
    virtual void Update(float dt) { t += dt; }
};

#endif
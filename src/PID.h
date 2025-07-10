#ifndef PID_H
#define PID_H

#include <Arduino.h>
#include "config.h"
#include "sensors.h"
#include <cmath>

struct PID {
    float Kp;
    float Ki;
    float Kd;
};

struct PIDErrors {
	float currentError;
	float previousError;
	float integral;
	unsigned long lastTime;
};

extern PID ROLL_PID;
extern PID PITCH_PID;
extern PID YAW_PID;

extern PIDErrors rollErrors;
extern PIDErrors pitchErrors;
extern PIDErrors yawErrors;

float calculatePID(const PID& pidCoeffs, PIDErrors& errors, float setpoint, float currentState);
void updateMotorsFromPID(float rollOutput, float pitchOutput, float yawOutput, uint8_t throttle);

#endif
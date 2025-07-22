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

// attitude PIDs
extern PID ROLL_ATTITUDE_PID;
extern PID PITCH_ATTITUDE_PID;
extern PID YAW_ATTITUDE_PID;

// rate PIDs
extern PID ROLL_RATE_PID;
extern PID PITCH_RATE_PID;
extern PID YAW_RATE_PID;

// error for attitude controller
extern PIDErrors rollAttitudeErrors;
extern PIDErrors pitchAttitudeErrors;
extern PIDErrors yawAttitudeErrors;

// error for rate controller
extern PIDErrors rollRateErrors;
extern PIDErrors pitchRateErrors;
extern PIDErrors yawRateErrors;

extern float MAX_ROLL_RATE;
extern float MAX_PITCH_RATE;
extern float MAX_YAW_RATE;

float calculatePID(const PID& pidCoeffs, PIDErrors& errors, float setpoint, float currentState);
void updateMotorsFromPID(float rollOutput, float pitchOutput, float yawOutput, uint32_t throttle);
void initializePID();
void resetPID();
void PIDControl(float desiredRoll, float desiredPitch, float desiredYaw, uint32_t throttle);

#endif
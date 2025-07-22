#include <Arduino.h>
#include "config.h"
#include "sensors.h"
#include "motors.h"
#include "PID.h"
#include <cmath>

////////////////////////////////////////////////////////////////////////////////////////////////////
// PID FUNCTIONS
////////////////////////////////////////////////////////////////////////////////////////////////////

// PID Coefficients (change for tuning)
PID ROLL_PID	{ 0.0, 0.0, 0.0 };
PID PITCH_PID	{ 0.0, 0.0, 0.0 };
PID YAW_PID		{ 20, 0.0, 0.0 };

// stores error states of an iteration so the program can bring the state closer to setpoints
PIDErrors rollErrors 	{ 0, 0, 0, 0 };
PIDErrors pitchErrors 	{ 0, 0, 0, 0 };
PIDErrors yawErrors 	{ 0, 0, 0, 0 };

constexpr float INTEGRAL_MAX = 20.0;
constexpr float INTEGRAL_MIN = -20.0;

void resetPID() {
    rollErrors.integral = 0;
    rollErrors.previousError = 0;
    pitchErrors.integral = 0;
    pitchErrors.previousError = 0;
    yawErrors.integral = 0;
    yawErrors.previousError = 0;
}

float calculatePID(const PID& pidCoeffs, PIDErrors& errors, float setpoint, float currentState) {
	unsigned long currentTime = micros();

	// convert time from ms to seconds
	float deltaTime = (currentTime - errors.lastTime) / 1000000.0;
	
	// calculate current error
	errors.currentError = setpoint - currentState;

	if ((errors.currentError > 0) != (errors.previousError > 0))
    	errors.integral = 0;

	// calcualte integral
	errors.integral += errors.currentError * deltaTime;

	// prevents integral from getting too large
	errors.integral = constrain(errors.integral, INTEGRAL_MIN, INTEGRAL_MAX);

	// calculate derivative
	float derivative = 0;
	if (deltaTime > 0) {
		derivative = (errors.currentError - errors.previousError) / deltaTime;
	}

	// calculate PID output
	float output = 	(pidCoeffs.Kp * errors.currentError) + 
					(pidCoeffs.Ki * errors.integral) +
					(pidCoeffs.Kd * derivative);

	// update for next iteration
	errors.previousError = errors.currentError;
	errors.lastTime = currentTime;

	return output;
}

void updateMotorsFromPID(float rollOutput, float pitchOutput, float yawOutput, uint32_t throttle) {
	float speedFL = throttle + pitchOutput + rollOutput + yawOutput;
	float speedFR = throttle + pitchOutput - rollOutput - yawOutput;
	float speedRL = throttle - pitchOutput + rollOutput + yawOutput;
	float speedRR = throttle - pitchOutput - rollOutput - yawOutput;

	motorSpeed[MOTOR_FL] = constrain(speedFL, 0, 1023);
	motorSpeed[MOTOR_FR] = constrain(speedFR, 0, 1023);
	motorSpeed[MOTOR_RL] = constrain(speedRL, 0, 1023);
	motorSpeed[MOTOR_RR] = constrain(speedRR, 0, 1023);
}

void initializePID() {
    unsigned long currentTime = micros();
    rollErrors.lastTime = currentTime;
    pitchErrors.lastTime = currentTime;
    yawErrors.lastTime = currentTime;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// END PID FUNCTIONS
////////////////////////////////////////////////////////////////////////////////////////////////////

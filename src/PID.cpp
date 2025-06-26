#include <Arduino.h>
#include "config.h"
#include "sensors.h"
#include "motors.h"
#include "PID.h"
#include <cmath>

////////////////////////////////////////////////////////////////////////////////////////////////////
// PID FUNCTIONS
////////////////////////////////////////////////////////////////////////////////////////////////////

// setpoints (orientation conditions the controller wants to aim for) ((no change in ang velocity))
float SETPOINT_ROLL 	= 0.0;
float SETPOINT_PITCH 	= 0.0;
float SETPOINT_YAW 		= 0.0;

// PID Coefficients (change for tuning)
PID ROLL_PID 	{ 0.6, 0.1, 0.05 };
PID PITCH_PID { 0.6, 0.1, 0.05 };
PID YAW_PID	{ 1.0, 0.5, 0.02 };

// stores error states of an iteration so the program can bring the state closer to setpoints
PIDErrors rollErrors 	{ 0, 0, 0, 0 };
PIDErrors pitchErrors 	{ 0, 0, 0, 0 };
PIDErrors yawErrors 	{ 0, 0, 0, 0 };

constexpr float INTEGRAL_MAX = 10;
constexpr float INTEGRAL_MIN = -10;

float calculatePID(const PID& pidCoeffs, PIDErrors& errors, float setpoint, float currentState) {
	unsigned long currentTime = millis();

	// convert time from ms to seconds
	float deltaTime = (currentTime - errors.lastTime) / 1000.0;
	
	// calculate current error
	errors.currentError = setpoint - currentState;

	// calcualte integral
	errors.integral += errors.currentError * deltaTime;

	// prevents integral from getting too large
	errors.integral = constrain(errors.integral, INTEGRAL_MIN, INTEGRAL_MAX);

	// calculate derivative
	float dt = 0;
	if (deltaTime > 0) {
		dt = (errors.currentError - errors.previousError) / deltaTime;
	}

	// calculate PID output
	float output = 	(pidCoeffs.Kp * errors.currentError) + 
					(pidCoeffs.Ki * errors.integral) +
					(pidCoeffs.Kd * dt);

	// update for next iteration
	errors.previousError = errors.currentError;
	errors.lastTime = currentTime;

	return output;
}

void updateMotorsFromPID(float rollOutput, float pitchOutput, float yawOutput, uint8_t throttle) {
	int16_t rollTrim 	= 50 * rollOutput;
	int16_t pitchTrim 	= 50 * pitchOutput;
	int16_t yawTrim 	= 50 * yawOutput;

	int16_t speedFL = throttle + pitchTrim + rollTrim - yawTrim;
	int16_t speedFR = throttle + pitchTrim - rollTrim + yawTrim;
	int16_t speedRL = throttle - pitchTrim + rollTrim + yawTrim;
	int16_t speedRR = throttle - pitchTrim - rollTrim - yawTrim;

	motorSpeed[MOTOR_FL] = constrain(speedFL, 0, 255);
	motorSpeed[MOTOR_FR] = constrain(speedFR, 0, 255);
	motorSpeed[MOTOR_RL] = constrain(speedRL, 0, 255);
	motorSpeed[MOTOR_RR] = constrain(speedRR, 0, 255);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// END PID FUNCTIONS
////////////////////////////////////////////////////////////////////////////////////////////////////

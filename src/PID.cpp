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
PID ROLL_PID	{ 0.6, 0.0, 0.0000 };
PID PITCH_PID	{ 0.6, 0.0, 0.0000 };
PID YAW_PID		{ 0.6, 0.0, 0.0000 };

// stores error states of an iteration so the program can bring the state closer to setpoints
PIDErrors rollErrors 	{ 0, 0, 0, 0 };
PIDErrors pitchErrors 	{ 0, 0, 0, 0 };
PIDErrors yawErrors 	{ 0, 0, 0, 0 };

constexpr float INTEGRAL_MAX = 1.0;
constexpr float INTEGRAL_MIN = -1.0;
constexpr float MOTOR_TRIM_SCALE = 50.0f;

void resetPID() {
    rollErrors.integral = 0;
    rollErrors.previousError = 0;
    pitchErrors.integral = 0;
    pitchErrors.previousError = 0;
    yawErrors.integral = 0;
    yawErrors.previousError = 0;
}

float calculatePID(const PID& pidCoeffs, PIDErrors& errors, float setpoint, float currentState) {
	unsigned long currentTime = millis();

	// convert time from ms to seconds
	float deltaTime = (currentTime - errors.lastTime) / 1000.0;
	
	// calculate current error
	errors.currentError = setpoint - currentState;

	if ((errors.currentError > -0.1 && errors.previousError < 0.1) || 
		(errors.currentError < 0.1 && errors.previousError > -0.1)) {
        errors.integral = 0;
    }

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

void updateMotorsFromPID(float rollOutput, float pitchOutput, float yawOutput, uint8_t throttle) {
	int16_t rollTrim 	= MOTOR_TRIM_SCALE * rollOutput;
	int16_t pitchTrim 	= MOTOR_TRIM_SCALE * pitchOutput;
	int16_t yawTrim 	= MOTOR_TRIM_SCALE * yawOutput;

	int16_t speedFL = throttle + pitchTrim + rollTrim - yawTrim;
	int16_t speedFR = throttle + pitchTrim - rollTrim + yawTrim;
	int16_t speedRL = throttle - pitchTrim + rollTrim + yawTrim;
	int16_t speedRR = throttle - pitchTrim - rollTrim - yawTrim;

	motorSpeed[MOTOR_FL] = constrain(speedFL, 0, 255);
	motorSpeed[MOTOR_FR] = constrain(speedFR, 0, 255);
	motorSpeed[MOTOR_RL] = constrain(speedRL, 0, 255);
	motorSpeed[MOTOR_RR] = constrain(speedRR, 0, 255);
}

void initializePID() {
    unsigned long currentTime = millis();
    rollErrors.lastTime = currentTime;
    pitchErrors.lastTime = currentTime;
    yawErrors.lastTime = currentTime;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// END PID FUNCTIONS
////////////////////////////////////////////////////////////////////////////////////////////////////

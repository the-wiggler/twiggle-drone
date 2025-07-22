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
PID ROLL_ATTITUDE_PID	{ 20.0, 0.0, 0.0 };
PID PITCH_ATTITUDE_PID	{ 20.0, 0.0, 0.0 };
PID YAW_ATTITUDE_PID	{ 10.0, 0.0, 0.0 };

PID ROLL_RATE_PID		{ 10.0, 0.0, 0.0 };
PID PITCH_RATE_PID		{ 10.0, 0.0, 0.0 };
PID YAW_RATE_PID		{ 05.0, 0.0, 0.0 };

// stores error states of an iteration so the program can bring the state closer to setpoints
PIDErrors rollAttitudeErrors	{ 0, 0, 0, 0 };
PIDErrors pitchAttitudeErrors	{ 0, 0, 0, 0 };
PIDErrors yawAttitudeErrors		{ 0, 0, 0, 0 };

PIDErrors rollRateErrors		{ 0, 0, 0, 0 };
PIDErrors pitchRateErrors		{ 0, 0, 0, 0 };
PIDErrors yawRateErrors			{ 0, 0, 0, 0 };

float MAX_ROLL_RATE 	= 0.3;
float MAX_PITCH_RATE 	= 0.3;
float MAX_YAW_RATE 		= 0.15;

constexpr float ATTITUDE_INTEGRAL_MAX = 0.5;
constexpr float ATTITUDE_INTEGRAL_MIN = -0.5;
constexpr float RATE_INTEGRAL_MAX = 20.0;
constexpr float RATE_INTEGRAL_MIN = -20.0;

void resetPID() {

}

float calculatePID(const PID& pidCoeffs, PIDErrors& errors, float setpoint, float currentState) {
	unsigned long currentTime = micros();
	// convert time from ms to seconds
	float deltaTime = (currentTime - errors.lastTime) / 1000000.0;
	
	// calculate current error
	errors.currentError = setpoint - currentState;

	if ((errors.currentError > 0) != (errors.previousError > 0)) errors.integral = 0;

	// calcualte integral
	errors.integral += errors.currentError * deltaTime;

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

void PIDControl(float desiredRoll, float desiredPitch, float desiredYaw, uint32_t throttle) {
	// attitude control (reads the angle and turns it into a desired turn rate)
	float desiredRollRate 	= calculatePID(ROLL_ATTITUDE_PID, rollAttitudeErrors, 
										 	desiredRoll, orientations.roll);
	float desiredPitchRate 	= calculatePID(PITCH_ATTITUDE_PID, pitchAttitudeErrors, 
										  	desiredPitch, orientations.pitch);
	float desiredYawRate 	= calculatePID(YAW_ATTITUDE_PID, yawAttitudeErrors, 
											desiredYaw, orientations.yaw);
	
	// constrains rates to the max limits allowed by the controller
	desiredRollRate 	= constrain(desiredRollRate, -MAX_ROLL_RATE, MAX_ROLL_RATE);
	desiredPitchRate 	= constrain(desiredPitchRate, -MAX_PITCH_RATE, MAX_PITCH_RATE);
	desiredYawRate 		= constrain(desiredYawRate, -MAX_YAW_RATE, MAX_YAW_RATE);

	// rate control (reads the desited turn rate ans transforms it into appropriate motor output)
	float rollOutput 	= calculatePID(ROLL_RATE_PID, rollRateErrors, 
										desiredRollRate, angularV.roll);
	float pitchOutput 	= calculatePID(PITCH_RATE_PID, pitchRateErrors, 
										desiredPitchRate, angularV.pitch);
	float yawOutput 	= calculatePID(YAW_RATE_PID, yawRateErrors, 
										desiredYawRate, angularV.yaw);
	
	updateMotorsFromPID(rollOutput, pitchOutput, yawOutput, throttle);
}

void initializePID() {
    unsigned long currentTime = micros();
    rollAttitudeErrors.lastTime = currentTime;
    pitchAttitudeErrors.lastTime = currentTime;
    yawAttitudeErrors.lastTime = currentTime;
	rollRateErrors.lastTime = currentTime;
    pitchRateErrors.lastTime = currentTime;
    yawRateErrors.lastTime = currentTime;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// END PID FUNCTIONS
////////////////////////////////////////////////////////////////////////////////////////////////////

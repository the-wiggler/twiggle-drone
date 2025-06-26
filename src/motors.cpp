#include "config.h"
#include "motors.h"

////////////////////////////////////////////////////////////////////////////////////////////////////
// MOTOR FUNCTIONS
////////////////////////////////////////////////////////////////////////////////////////////////////

uint8_t motorSpeed[4] = { 0, 0, 0, 0 };	// an array that holds the motor speed values to be enacted
										// on cycle update

// function to update the speeds of all motors by referencing values from motorSpeed
void updateMotorSpeed() {
	ledcWrite(MOTOR_FL, motorSpeed[MOTOR_FL]);
	ledcWrite(MOTOR_FR, motorSpeed[MOTOR_FR]);
	ledcWrite(MOTOR_RL, motorSpeed[MOTOR_RL]);
	ledcWrite(MOTOR_RR, motorSpeed[MOTOR_RR]);
}

// function that changes the speeds of all motors
void setAllMotorSpeed(uint8_t speed) {
	motorSpeed[MOTOR_FL] = speed;
	motorSpeed[MOTOR_FR] = speed;
	motorSpeed[MOTOR_RL] = speed;
	motorSpeed[MOTOR_RR] = speed;
}

// function that stops all motor movement if there's an emergency
void emergencyStop() {
	motorSpeed[MOTOR_FL] = 0;
	motorSpeed[MOTOR_FR] = 0;
	motorSpeed[MOTOR_RL] = 0;
	motorSpeed[MOTOR_RR] = 0;
	updateMotorSpeed();
	SYSTEM_FAILURE = true;
}

// function that runs each motor individually in a sequence to test them
void testMotors() {
	motorSpeed[MOTOR_FL] = 150;
	updateMotorSpeed();
	delay(2000);
	motorSpeed[MOTOR_FL] = 0;
	updateMotorSpeed();

	motorSpeed[MOTOR_FR] = 150;
	updateMotorSpeed();
	delay(2000);
	motorSpeed[MOTOR_FR] = 0;
	updateMotorSpeed();

	motorSpeed[MOTOR_RL] = 150;
	updateMotorSpeed();
	delay(2000);
	motorSpeed[MOTOR_RL] = 0;
	updateMotorSpeed();

	motorSpeed[MOTOR_RR] = 150;
	updateMotorSpeed();
	delay(2000);
	motorSpeed[MOTOR_RR] = 0;
	updateMotorSpeed();
}
////////////////////////////////////////////////////////////////////////////////////////////////////
// END MOTOR FUNCTIONS
////////////////////////////////////////////////////////////////////////////////////////////////////

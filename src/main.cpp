#include <Arduino.h>

constexpr uint8_t MOTOR_FL_PIN 		= 2; 	// pin for motor front left
constexpr uint8_t MOTOR_FR_PIN 		= 5; 	// pin for motor front right
constexpr uint8_t MOTOR_RL_PIN 		= 3; 	// pin for motor rear left
constexpr uint8_t MOTOR_RR_PIN 		= 4; 	// pin for motor rear right

constexpr uint8_t MOTOR_FL 			= 0; 	// PWM channel for motor front left
constexpr uint8_t MOTOR_FR 			= 1; 	// PWM channel for motor front right
constexpr uint8_t MOTOR_RL 			= 2;	// PWM channel for motor rear left
constexpr uint8_t MOTOR_RR 			= 3; 	// PWM channel for motor rear right

constexpr uint32_t PWM_FREQUENCY 	= 1000;	// PWM update frequency
constexpr uint8_t PWM_RESOLUTION 	= 8;	// PWM resolution (bits)

////////////////////////////////////////////////////////////////////////////////////////////////////
// SETUP
////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
	Serial.begin(115200);

	/////// MOTOR INITIALIZATION ///////////////////////////////////////////////////////////////////
	// front left motor
	ledcSetup(MOTOR_FL, PWM_FREQUENCY, PWM_RESOLUTION);
	ledcAttachPin(MOTOR_FL_PIN, MOTOR_FL);

	//front right motor
	ledcSetup(MOTOR_FR, PWM_FREQUENCY, PWM_RESOLUTION);
	ledcAttachPin(MOTOR_FR_PIN, MOTOR_FR);

	// rear left motor
	ledcSetup(MOTOR_RL, PWM_FREQUENCY, PWM_RESOLUTION);
	ledcAttachPin(MOTOR_RL_PIN, MOTOR_RL);

	// rear right motor
	ledcSetup(MOTOR_RR, PWM_FREQUENCY, PWM_RESOLUTION);
	ledcAttachPin(MOTOR_RR_PIN, MOTOR_RR);

	// set all motor speeds to 0 at startup
	ledcWrite(MOTOR_FL, 0);
	ledcWrite(MOTOR_FR, 0);
	ledcWrite(MOTOR_RL, 0);
	ledcWrite(MOTOR_RR, 0);

	Serial.println("Motors initialized.");
	////////////////////////////////////////////////////////////////////////////////////////////////

	// wait 3 seconds
	delay(3000);
}
////////////////////////////////////////////////////////////////////////////////////////////////////
// END SETUP
////////////////////////////////////////////////////////////////////////////////////////////////////

uint8_t motorSpeed[4] = { 0, 0, 0, 0 };	// an array that holds the motor speed values to be enacted
										// on cycle update

// function to change the speeds of all motors by referencing values from motorSpeed
void updateMotorSpeed() {
	ledcWrite(MOTOR_FL, motorSpeed[MOTOR_FL]);
	ledcWrite(MOTOR_FR, motorSpeed[MOTOR_FR]);
	ledcWrite(MOTOR_RL, motorSpeed[MOTOR_RL]);
	ledcWrite(MOTOR_RR, motorSpeed[MOTOR_RR]);
}

// function to change a motors speed that updateMotorSpeed references on cycle update
void setMotorSpeed(uint8_t motor, uint8_t speed) {
	if (motor > 3) return;
	if (speed > 255) speed = 255;	// helps if speed goes over the limit due to bad programming
	motorSpeed[motor] = speed;
}

// function that stops all motor movement if there's an emergency
void emergencyStop() {
	motorSpeed[MOTOR_FL, 0];
	motorSpeed[MOTOR_FR, 0];
	motorSpeed[MOTOR_RL, 0];
	motorSpeed[MOTOR_RR, 0];
	updateMotorSpeed();
}


////////////////////////////////////////////////////////////////////////////////////////////////////
// LOOP
////////////////////////////////////////////////////////////////////////////////////////////////////
bool SYSTEM_FAILURE = false;	// this is set to true if the system catches a fatal error that
								// requires all operations to be halted by (kinda) stopping the loop
void loop() {
	// check if a failure has been detected
	if (SYSTEM_FAILURE) return;

	// start cycle
	setMotorSpeed(MOTOR_FL, 100);

	updateMotorSpeed();
	// end cycle
}
////////////////////////////////////////////////////////////////////////////////////////////////////
// END LOOP
////////////////////////////////////////////////////////////////////////////////////////////////////

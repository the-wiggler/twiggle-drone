#include <Arduino.h>

constexpr int MOTOR_FL_PIN  = 2; 	// pin for motor front left
constexpr int MOTOR_FR_PIN  = 5; 	// pin for motor front right
constexpr int MOTOR_RL_PIN  = 3; 	// pin for motor rear left
constexpr int MOTOR_RR_PIN  = 4; 	// pin for motor rear right

constexpr int MOTOR_FL = 0; 		// PWM channel for motor front left
constexpr int MOTOR_FR = 1; 		// PWM channel for motor front right
constexpr int MOTOR_RL = 2;			// PWM channel for motor front right
constexpr int MOTOR_RR = 3; 		// PWM channel for motor front right

const int PWM_FREQUENCY = 1000;		// PWM update frequency
const int PWM_RESOLUTION = 8;		// PWM resolution (bits)

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

	////////////////////////////////////////////////////////////////////////////////////////////////

	// wait 5 seconds
	delay(5000);
}
////////////////////////////////////////////////////////////////////////////////////////////////////
// END SETUP
////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////
// LOOP
////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
	for (int dutyCycle = 0; dutyCycle <= 255; dutyCycle += 5) {
		ledcWrite(MOTOR_RL, dutyCycle);
		delay(200);
	}

	ledcWrite(MOTOR_RL, 0);

	delay(10000);
}
////////////////////////////////////////////////////////////////////////////////////////////////////
// END LOOP
////////////////////////////////////////////////////////////////////////////////////////////////////

#include <Arduino.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <cmath>

Adafruit_ICM20948 icm; 						// creates ICM20948 sensor as 'icm' object

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

bool SYSTEM_FAILURE = false;				// this is set to true if the system catches a fatal error that
											// requires all operations to be halted by (kinda) stopping the loop

////////////////////////////////////////////////////////////////////////////////////////////////////
// SETUP
////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
	Serial.begin(115200);
	delay(3000); // small delay to allow serial monitor to set up

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

	/////// SENSOR INITIALIZATION //////////////////////////////////////////////////////////////////
	for (int i = 0; i < 11; i++) {
		if (icm.begin_I2C()) {
			Serial.println("ICM20948 found!");
			break;
		}
		Serial.println("Failed to connect to ICM20948. Retrying...");
		delay(1000);
	}
	// for some reason it sometimes doesnt connect on the first try...idk why???

	icm.setAccelRateDivisor(0); 	// maximum accelerometer rate
	icm.setGyroRateDivisor(0); 		// maximum gyroscope rate

	Serial.println("Sensors initialized.");
	////////////////////////////////////////////////////////////////////////////////////////////////

	// wait 3 seconds
	Serial.println("Startup...");
	delay(1000);
}
////////////////////////////////////////////////////////////////////////////////////////////////////
// END SETUP
////////////////////////////////////////////////////////////////////////////////////////////////////


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

////////////////////////////////////////////////////////////////////////////////////////////////////
// SENSOR FUNCTIONS
////////////////////////////////////////////////////////////////////////////////////////////////////
sensors_event_t accel, gyro, temp, mag;	// start sensor collection objects

float accelData[3] 		= { 0, 0, 0 }; 	// array that holds acceleration data from the ICM20948
float angularV[3]  		= { 0, 0, 0 }; 	// array that holds angular velocity data.
float orientations[2] 	= { 0, 0    };  // array that holds angular orientation values (pitch & yaw)
										// Array order:
										// 0: Roll
										// 1: Pitch
										// 2: Yaw

// IMPORTANT: for the Adafruit ICM20948 X is read as pitch, Y as roll, and Z as yaw

// function to place sensor data in their respective arrays. Run this to update data
void readICM() {
	icm.getEvent(&accel, &gyro, &temp, &mag);

	accelData[0] = accel.acceleration.y;
	accelData[1] = accel.acceleration.x;
	accelData[2] = accel.acceleration.z;

	angularV[0] = gyro.gyro.y;
	angularV[1] = gyro.gyro.z;
	angularV[2] = gyro.gyro.z;
}

// function to calculate the orientation angles (x,y,z) of the drone during a cycle
void calculateOrientation() {
	orientations[0] = atan(accelData[1] / (sqrt((accelData[0] * accelData[0]) + (accelData[2] * accelData[2]))));

	orientations[1] = atan(-accelData[0] / (sqrt((accelData[1] * accelData[1]) + (accelData[2] * accelData[2]))));
}
////////////////////////////////////////////////////////////////////////////////////////////////////
// END SENSOR FUNCTIONS
////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////
// PID FUNCTIONS
////////////////////////////////////////////////////////////////////////////////////////////////////
// setpoints (orientation conditions the controller wants to aim for) ((no change in ang velocity))
constexpr float SETPOINT_ROLL 	= 0.0;
constexpr float SETPOINT_PITCH 	= 0.0;
constexpr float SETPOINT_YAW 	= 0.0;

struct PID {
    float Kp;
    float Ki;
    float Kd;
};

// PID Coefficients (change for tuning)
constexpr PID ROLL_PID 	{ 0.6, 0.1, 0.1 };
constexpr PID PITCH_PID { 0.6, 0.1, 0.1 };
constexpr PID YAW_PID	{ 1.0, 0.5, 0.1 };

struct PIDErrors {
	float currentError;
	float previousError;
	float integral;
	unsigned long lastTime;
};

// stores error states of an iteration so the program can bring the state closer to setpoints
PIDErrors rollErrors 	{ 0, 0, 0, 0 };
PIDErrors pitchErrors 	{ 0, 0, 0, 0 };
PIDErrors yawErrors 	{ 0, 0, 0, 0 };

float calculatePID(const PID& pidCoeffs, PIDErrors& errors, float setpoint, float currentState) {
	unsigned long currentTime = millis();
	float deltaTime = (currentTime - errors.lastTime) / 1000;	// convert to seconds
	
	// calculate current error
	errors.currentError = setpoint - currentState;

	// calcualte integral
	errors.integral += errors.currentError * deltaTime;

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

////////////////////////////////////////////////////////////////////////////////////////////////////
// MONITORING FUNCTIONS
////////////////////////////////////////////////////////////////////////////////////////////////////
void monitorMotorSpeeds() {
	Serial.print("\rFL: "); Serial.print(motorSpeed[MOTOR_FL]); Serial.print(" | FR: ");
	Serial.print(motorSpeed[MOTOR_FR]); Serial.print(" | RL: "); Serial.print(motorSpeed[MOTOR_RL]);
	Serial.print(" | RR: "); Serial.print(motorSpeed[MOTOR_RR]); Serial.print("     ");
}
void monitorRollPitchPID(float rollOutput, float pitchOutput) {
	Serial.print("\rRoll: "); Serial.print(orientations[0]); Serial.print(" - Roll PID: "); 
	Serial.print(rollOutput); Serial.print(" | Pitch: "); Serial.print(orientations[1]); 
	Serial.print(" - Pitch PID: "); 
	Serial.print(pitchOutput); Serial.print("         ");
}
////////////////////////////////////////////////////////////////////////////////////////////////////
// END MONITORING FUNCTIONS
////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////
// LOOP
////////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t throttle = 200;

void loop() {
	// check if a failure has been detected
	// if (millis() > 15000) emergencyStop(); // timer to run the program until a cutoff time
	if (SYSTEM_FAILURE) return;

	// get sensor info
	readICM();

	calculateOrientation();

	// perform PID calculation
	float rollOutput = calculatePID(ROLL_PID, rollErrors, SETPOINT_ROLL, orientations[0]);
	float pitchOutput = calculatePID(PITCH_PID, pitchErrors, SETPOINT_PITCH, orientations[1]);
	float yawOutput = 0;

	updateMotorsFromPID(rollOutput, pitchOutput, yawOutput, throttle);

	updateMotorSpeed();

	monitorMotorSpeeds();
	// monitorRollPitchPID(rollOutput, pitchOutput);
}
////////////////////////////////////////////////////////////////////////////////////////////////////
// END LOOP
////////////////////////////////////////////////////////////////////////////////////////////////////

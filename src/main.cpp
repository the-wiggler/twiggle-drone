#include <Arduino.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <cmath>

#include "config.h"
#include "motors.h"
#include "sensors.h"
#include "PID.h"
#include "debug.h"
#include "communication.h"

SemaphoreHandle_t sensorMutex;

////////////////////////////////////////////////////////////////////////////////////////////////////
// SETUP
////////////////////////////////////////////////////////////////////////////////////////////////////
void core0Process(void *parameter);

void setup() {
	Serial.begin(115200);
	delay(3000); // small delay to allow serial monitor to set up

	sensorMutex = xSemaphoreCreateMutex();

	wifiSetup();

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

	/////// CORE TASK SETUP ////////////////////////////////////////////////////////////////////////
	xTaskCreatePinnedToCore(
		core0Process,
		"core0Process",
		20000,
		NULL,
		2,
		NULL,
		0
	);
	////////////////////////////////////////////////////////////////////////////////////////////////
	
	// wait 3 seconds
	Serial.println("Startup...");
	delay(1000);
}
////////////////////////////////////////////////////////////////////////////////////////////////////
// END SETUP
////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////
// CORE 1 LOOP
////////////////////////////////////////////////////////////////////////////////////////////////////
bool SYSTEM_FAILURE = false;
uint8_t throttle = 200;
constexpr unsigned long LOOP_INTERVAL = 5000; // 5ms is a 200 Hz delay for the PID system

void loop() {
	unsigned long  startTime = micros();

	// check if a failure has been detected
	// if (millis() > 15000) emergencyStop(); // timer to run the program until a cutoff time
	if (SYSTEM_FAILURE) return;

	float rollOutput, pitchOutput;

	// calculate PID outputs if the sensor data is safe to access (not being used)
	if (xSemaphoreTake(sensorMutex, 10 / portTICK_PERIOD_MS)) {
		rollOutput = calculatePID(ROLL_PID, rollErrors, SETPOINT_ROLL, orientations[0]);
		pitchOutput = calculatePID(PITCH_PID, pitchErrors, SETPOINT_PITCH, orientations[1]);
		xSemaphoreGive(sensorMutex);
	}
	else return; // in case core 0 was using our data :(

	// placeholder since im lazy and havent implemented yaw stuff yet
	float yawOutput = 0;

	// changes the state of the motor speed arrays to PID corrected values
	updateMotorsFromPID(rollOutput, pitchOutput, yawOutput, throttle);

	// sends the new motor speeds to the PWM hardware
	updateMotorSpeed();

	// monitorRollPitchPID(rollOutput, pitchOutput);
	// monitorMotorSpeeds();

	// maintains the 200 Hz clock speed
	unsigned long elapsedTime = micros() - startTime;
	if (elapsedTime < LOOP_INTERVAL) delayMicroseconds(LOOP_INTERVAL - elapsedTime); 
	// apparently delayMicroseconds is a thing!
}
////////////////////////////////////////////////////////////////////////////////////////////////////
// END CORE 1
////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////
// CORE 0 LOOP
////////////////////////////////////////////////////////////////////////////////////////////////////
void core0Process(void *parameter) {
	while(true) {
		if (!SYSTEM_FAILURE) {
			if (xSemaphoreTake(sensorMutex, portMAX_DELAY)) {
				readICM();
				calculateOrientation();
				xSemaphoreGive(sensorMutex);
			}
			recieveUDPCommand();
		}
		vTaskDelay(10 / portTICK_PERIOD_MS);
	}
}
////////////////////////////////////////////////////////////////////////////////////////////////////
// END CORE 0
////////////////////////////////////////////////////////////////////////////////////////////////////
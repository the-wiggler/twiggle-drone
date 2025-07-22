#include "sensors.h"
#include <Arduino.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
////////////////////////////////////////////////////////////////////////////////////////////////////
// SENSOR FUNCTIONS
////////////////////////////////////////////////////////////////////////////////////////////////////
Adafruit_ICM20948 icm; 					// creates ICM20948 sensor as 'icm' object
sensors_event_t accel, gyro, temp, mag;

// IMPORTANT: for the Adafruit ICM20948 X is read as pitch, Y as roll, and Z as yaw
sensorData accelData 	= { 0, 0, 0 }; 	// array that holds acceleration data from the ICM20948
sensorData angularV  	= { 0, 0, 0 }; 	// array that holds angular velocity data.
sensorData orientations = { 0, 0, 0 };  // array that holds angular orientation values (pitch & yaw)
										// Array order:
										// 0: Roll
										// 1: Pitch
										// 2: Yaw

// timing value for the complementary filter integral element
unsigned long lastSensorTime = 0;

void initializeSensors() {
	for (int i = 0; i < 5; i++) {
		if (icm.begin_I2C()) {
			Serial.println("ICM20948 found!");
			break;
		}
		Serial.println("Failed to connect to ICM20948. Retrying...");
		delay(1000);
	}
	if (!icm.begin_I2C()) {
		Serial.println("ERROR: Failed to connect to ICM20948");
	}
	// for some reason it sometimes doesnt connect on the first try...idk why???

	icm.setAccelRateDivisor(0); 	// maximum accelerometer rate
	icm.setGyroRateDivisor(0); 		// maximum gyroscope rate

	lastSensorTime = micros();

	Serial.println("Sensors initialized.");
}

// function to place sensor data in their respective arrays. Run this to update data
void readICM() {
	icm.getEvent(&accel, &gyro, &temp, &mag);

	// save accelerometer data to global acceleration variables
	accelData.roll 	= accel.acceleration.y;
	accelData.pitch = accel.acceleration.x;
	accelData.yaw 	= accel.acceleration.z;
 
	// save gyro data to global acceleration variables
	angularV.roll 	= gyro.gyro.y;
	angularV.pitch	= gyro.gyro.x;
	angularV.yaw 	= gyro.gyro.z;
}

// function to calculate the orientation angles (x,y,z) of the drone during a cycle
void calculateOrientation() {
	unsigned long currentTime = micros();
	float deltaTime = (currentTime - lastSensorTime) / 1000000.0f; // 1000000 converts to seconds
	lastSensorTime = currentTime;

	float absoluteRoll = atan2(accelData.pitch, sqrt(accelData.roll * accelData.roll + accelData.yaw * accelData.yaw));
	// magical angle calculations!
	float absolutePitch = atan2(-accelData.roll, sqrt(accelData.pitch * accelData.pitch + accelData.yaw * accelData.yaw));

	// complementary filter
	orientations.roll 	= 0.96f * (orientations.roll + angularV.roll * deltaTime) +
						  0.04f * absoluteRoll;
	orientations.pitch 	= 0.96f * (orientations.pitch + angularV.pitch * deltaTime) +
						  0.04f * absolutePitch;
	
	orientations.yaw += angularV.yaw * deltaTime;
}
////////////////////////////////////////////////////////////////////////////////////////////////////
// END SENSOR FUNCTIONS
////////////////////////////////////////////////////////////////////////////////////////////////////

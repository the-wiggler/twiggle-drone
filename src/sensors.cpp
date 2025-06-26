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


float accelData[3] 		= { 0, 0, 0 }; 	// array that holds acceleration data from the ICM20948
float angularV[3]  		= { 0, 0, 0 }; 	// array that holds angular velocity data.
float orientations[3] 	= { 0, 0, 0 };  // array that holds angular orientation values (pitch & yaw)
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
	angularV[1] = gyro.gyro.x;
	angularV[2] = gyro.gyro.z;
}

// function to calculate the orientation angles (x,y,z) of the drone during a cycle
void calculateOrientation() {
	orientations[0] = atan(accelData[1] / (sqrt((accelData[0] * accelData[0]) + (accelData[2] * accelData[2]))));
	// magical angle calculations!
	orientations[1] = atan(-accelData[0] / (sqrt((accelData[1] * accelData[1]) + (accelData[2] * accelData[2]))));
}
////////////////////////////////////////////////////////////////////////////////////////////////////
// END SENSOR FUNCTIONS
////////////////////////////////////////////////////////////////////////////////////////////////////

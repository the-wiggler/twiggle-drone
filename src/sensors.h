#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

struct sensorData {
	float roll;
	float pitch;
	float yaw;
};

extern Adafruit_ICM20948 icm;
extern sensors_event_t accel, gyro, temp, mag;

extern sensorData accelData;
extern sensorData angularV;
extern sensorData orientations;

extern unsigned long lastSensorTime;

void initializeSensors();
void readICM();
void calculateOrientation();

#endif
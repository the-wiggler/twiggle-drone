#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

extern Adafruit_ICM20948 icm;
extern sensors_event_t accel, gyro, temp, mag;

extern float accelData[3];
extern float angularV[3];
extern float orientations[3];

void initializeSensors();
void readICM();
void calculateOrientation();

#endif
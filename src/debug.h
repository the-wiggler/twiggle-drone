#ifndef DEBUG_H
#define DEBUG_H

#include <Arduino.h>
#include "config.h"
#include "sensors.h"
#include "motors.h"

inline void monitorMotorSpeeds() {
	Serial.print("\rFL: "); Serial.print(motorSpeed[MOTOR_FL]); Serial.print(" | FR: ");
	Serial.print(motorSpeed[MOTOR_FR]); Serial.print(" | RL: "); Serial.print(motorSpeed[MOTOR_RL]);
	Serial.print(" | RR: "); Serial.print(motorSpeed[MOTOR_RR]); Serial.print("     ");
}

inline void monitorRollPitchPID(float rollOutput, float pitchOutput) {
	static unsigned long lastUpdateTime = 0; // Tracks the last time the Serial output was updated
	unsigned long currentTime = millis();

	// Check if 500 ms have passed since the last update
	if (currentTime - lastUpdateTime >= 500) {
		Serial.print("\rRoll: "); Serial.print(orientations[0]); Serial.print(" - Roll PID: "); 
		Serial.print(rollOutput); Serial.print(" | Pitch: "); Serial.print(orientations[1]); 
		Serial.print(" - Pitch PID: "); 
		Serial.print(pitchOutput); Serial.print("         ");

		lastUpdateTime = currentTime;
	}
}

inline void logData(float rollPID, float pitchPID) {
	Serial.print(orientations[0]); //roll
	Serial.print(",");
	Serial.print(rollPID); //roll pid
	Serial.print(",");
	Serial.print(orientations[1]); //pitch
	Serial.print(",");
	Serial.print(pitchPID); //pitch pid
	Serial.print(",");
	Serial.print(motorSpeed[MOTOR_FL]);
	Serial.print(",");
	Serial.print(motorSpeed[MOTOR_FR]);
	Serial.print(",");
	Serial.print(motorSpeed[MOTOR_RL]);
	Serial.print(",");
	Serial.println(motorSpeed[MOTOR_RR]);
}

#endif
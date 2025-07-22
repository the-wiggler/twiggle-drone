#ifndef DEBUG_H
#define DEBUG_H

#include <Arduino.h>
#include "config.h"
#include "sensors.h"
#include "motors.h"
#include "PID.h"
#include "communication.h"

inline void monitorMotorSpeeds() {
	Serial.print("\rFL: "); Serial.print(motorSpeed[MOTOR_FL]); Serial.print(" | FR: ");
	Serial.print(motorSpeed[MOTOR_FR]); Serial.print(" | RL: "); Serial.print(motorSpeed[MOTOR_RL]);
	Serial.print(" | RR: "); Serial.print(motorSpeed[MOTOR_RR]); Serial.print("     ");
}

inline void monitorRollPitchPID(float rollOutput, float pitchOutput, float yawOutput) {
	static unsigned long lastUpdateTime = 0; // Tracks the last time the Serial output was updated
	unsigned long currentTime = millis();

	// Check if 500 ms have passed since the last update
	if (currentTime - lastUpdateTime >= 500) {
		Serial.print("\rRoll: "); Serial.print(orientations.roll); Serial.print(" - Roll PID: "); 
		Serial.print(rollOutput); Serial.print(" | Pitch: "); Serial.print(orientations.pitch); 
		Serial.print(" - Pitch PID: "); Serial.print(pitchOutput); Serial.print(" | Yaw: ");
		Serial.print(orientations.yaw); Serial.print(" - Yaw PID: "); Serial.print(yawOutput);
		Serial.print("         ");

		lastUpdateTime = currentTime;
	}
}

inline void logData(float rollOutput, float pitchOutput) {
	Serial.print(orientations.roll); //roll
	Serial.print(",");
	Serial.print(rollOutput); //roll pid
	Serial.print(",");
	Serial.print(orientations.pitch); //pitch
	Serial.print(",");
	Serial.print(pitchOutput); //pitch pid
	Serial.print(",");
	Serial.print(motorSpeed[MOTOR_FL]);
	Serial.print(",");
	Serial.print(motorSpeed[MOTOR_FR]);
	Serial.print(",");
	Serial.print(motorSpeed[MOTOR_RL]);
	Serial.print(",");
	Serial.println(motorSpeed[MOTOR_RR]);
	Serial.print(ROLL_PID.Kp); //roll pid
	Serial.print(",");
	Serial.print(ROLL_PID.Ki); //roll pid
	Serial.print(",");
	Serial.print(ROLL_PID.Kd); //roll pid
}

inline void receivePidDataPacket(SemaphoreHandle_t pidPacketMutex, udpPacket pid_packet) {
	if (xSemaphoreTake(pidPacketMutex, 0)) {
		ROLL_PID.Kp = pid_packet.roll;
		ROLL_PID.Ki = pid_packet.pitch;
		ROLL_PID.Kd = pid_packet.yaw;
		PITCH_PID.Kp = pid_packet.roll;
		PITCH_PID.Ki = pid_packet.pitch;
		PITCH_PID.Kd = pid_packet.yaw;
		xSemaphoreGive(pidPacketMutex);
	}
}

#endif
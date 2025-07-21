#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

constexpr uint8_t MOTOR_FL_PIN 		= 2; 	// pin for motor front left
constexpr uint8_t MOTOR_FR_PIN 		= 5; 	// pin for motor front right
constexpr uint8_t MOTOR_RL_PIN 		= 3; 	// pin for motor rear left
constexpr uint8_t MOTOR_RR_PIN 		= 4; 	// pin for motor rear right

constexpr uint8_t MOTOR_FL 			= 0; 	// PWM channel for motor front left
constexpr uint8_t MOTOR_FR 			= 1; 	// PWM channel for motor front right
constexpr uint8_t MOTOR_RL 			= 2;	// PWM channel for motor rear left
constexpr uint8_t MOTOR_RR 			= 3; 	// PWM channel for motor rear right

constexpr uint32_t PWM_FREQUENCY 	= 78000;	// PWM update frequency
constexpr uint8_t  PWM_RESOLUTION 	= 10;	// PWM resolution (bits)

extern bool SYSTEM_FAILURE;

#endif
#ifndef MOTORS_H
#define MOTORS_H

#include "config.h"

extern uint32_t motorSpeed[4];

void initializeMotors();
void updateMotorSpeed();
void setAllMotorSpeed(uint32_t speed);
void emergencyStop();
void testMotors();

#endif
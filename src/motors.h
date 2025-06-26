#ifndef MOTORS_H
#define MOTORS_H

#include "config.h"

extern uint8_t motorSpeed[4];

void updateMotorSpeed();
void setAllMotorSpeed(uint8_t speed);
void emergencyStop();
void testMotors();

#endif
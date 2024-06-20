#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

// Motor control pins
#define PWM 25
#define IN2 26
#define IN1 27

// Function prototypes
void setMotor(int dir, int pwmVal);

#endif
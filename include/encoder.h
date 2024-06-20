#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

//Encoderr pins
#define ENCODER_1A 34
#define ENCODER_1B 32

// Global variables 
extern volatile int posi;

// Function prototypes
void encoder1_isr();

#endif
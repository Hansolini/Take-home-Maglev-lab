#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <Wire.h>
#include <Arduino.h>
#include <TCA9548.h>
#include "definitions.h"
// #include "TLx493D_inc.hpp"
#include <Tlv493d.h>

// using namespace ifx::tlx493d;

// === Function Prototypes ===

// Solenoid-related functions
float getSolenoidCurrent(uint16_t pin);
void setSolenoidInput(int pwm, int pin1, int pin2);

// Initialization functions
void initializeSerial();
void initializeSensor();
void initializeSolenoids();
void clearI2CBus();
bool sensorInitializedCorrectly();

#endif // FUNCTIONS_H

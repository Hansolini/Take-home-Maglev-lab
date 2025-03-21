#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <Wire.h>
#include <Arduino.h>
#include <TCA9548.h>
#include "definitions.h"
// #include "TLx493D_inc.hpp"
#include <Tlv493d.h>
#include "cobs.h"

// using namespace ifx::tlx493d;

// Solenoid-related functions
float getSolenoidCurrent(uint16_t pin);
void setSolenoidInput(int pwm, int pin1, int pin2);
void resetSolenoids();  // ATTENTION: serial response is directly handled inside this method (try to separate purposes)

// Initialization functions
void initializeSerial();
void initializeSensor();
void initializeSolenoids();
void clearI2CBus();
bool sensorInitializedCorrectly();

// Serial communication
void sendSensorValues();
void sendStatus();
void sendError(uint8_t);
void setSolenoidCurrents(uint8_t*);  // ATTENTION: same "problem" of resetSolenoids()
void sendCurrentValues();
void sendUntilNull(uint8_t*);

#endif // FUNCTIONS_H

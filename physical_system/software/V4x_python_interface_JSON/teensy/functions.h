#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <Wire.h>
#include <Arduino.h>
#include <TCA9548.h>
#include <ArduinoJson.h>
#include "definitions.h"
// #include "TLx493D_inc.hpp"
#include <Tlv493d.h>

// using namespace ifx::tlx493d;

// Solenoid-related functions
float getSolenoidCurrent(uint16_t pin);
void setSolenoidInput(int pwm, int pin1, int pin2);
void resetSolenoids();

// Initialization functions
void initializeSerial();
void initializeSensor();
void initializeSolenoids();

// Serial communication
void sendSensorValues();
void sendStatus();
void sendError(int, const char*);
void setSolenoidCurrents(JsonArray);
void sendCurrentValues();

#endif // FUNCTIONS_H

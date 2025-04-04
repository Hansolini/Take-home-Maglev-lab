#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <Wire.h>
#include <Arduino.h>
#include <TCA9548.h>
#include "definitions.h"
// #include "TLx493D_inc.hpp"
#include <Tlv493d.h>

// using namespace ifx::tlx493d;

// === External Variable Declarations ===
extern float feedthroughSlopeX[NUM_SENSORS][2];
extern float feedthroughSlopeY[NUM_SENSORS][2];
extern float feedthroughSlopeZX[NUM_SENSORS][2];
extern float feedthroughSlopeZY[NUM_SENSORS][2];
extern float meanMagField[NUM_SENSORS][3];
extern float rawMagField[NUM_SENSORS][3];
extern float rawMagFieldDetrended[NUM_SENSORS][3];

// === Function Prototypes ===

// Solenoid-related functions
float getSolenoidCurrent(uint16_t pin);
void setSolenoidInput(int pwm, int pin1, int pin2);
void applyControlSignals(float pwmInputX, float pwmInputY);

// Initialization functions
void initializeSerial();
void initializeSensors();
void initializeSolenoids();
void clearI2CBus();
bool sensorInitializedCorrectly(int sensorIndex);

// Sensor management functions
void readAllSensors();
void processSensorData(float currentXPos, float currentXNeg, float currentYPos, float currentYNeg);
void checkSensorFreeze(float lastMagField[][3], int* freezeCounter);
void resetSensorAndI2C();

// Calibration functions
void calibrateSensors();
void calibrateDirectFeedthrough();

// Logging functions
void logSystemState(unsigned long currentTime, float currentXPos, float currentXNeg, float currentYPos, float currentYNeg);

#endif // FUNCTIONS_H

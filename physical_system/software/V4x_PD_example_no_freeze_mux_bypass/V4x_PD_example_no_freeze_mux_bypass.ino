/*******************************************************************************
 * Maggy V4.x - Magnetic Levitation System - PD Controller Implementation
 * 
 * This sketch implements a digital control system for magnetic levitation using
 * multiple hall effect sensors (TLV493D) and solenoid actuators. The system:
 * 
 * - Uses multiple hall effect sensors to detect magnet position
 * - Applies a PD (Proportional-Derivative) control algorithm
 * - Compensates for feedthrough effects from solenoid actuation
 * - Includes automatic sensor calibration and fault recovery
 * 
 * ADJUSTABLE PARAMETERS:
 * 
 * Control Parameters:
 * - Kp: Proportional gain. Increase for stiffer control, reduce for softer.
 * - Kd: Derivative gain. Increase to reduce oscillation, but too high 
 *       can cause instability.
 * 
 * Filtering Parameters:
 * - ALPHA: Magnetic field reading filter constant (0-1).
 *          Lower values = more filtering, higher = faster response.
 * - DALPHA: Derivative filter constant (0-1).
 *           Lower values = smoother derivative, higher = faster response.
 * 
 * Timing Parameters:
 * - sensorFrequency: How often sensors are read (Hz)
 * - controlFrequency: How often control updates are applied (Hz)
 * 
 * Hardware Configuration:
 * - SENSOR_CHANNELS array: Maps sensors to multiplexer channels
 * - PRIMARY_SENSOR_INDEX (in definitions.h): Index of primary sensor used for control
 * 
 * TUNING GUIDELINES:
 * 
 * If the system exhibits oscillations:
 * - Increase Kd or decrease Kp
 * - Decrease ALPHA and DALPHA for more filtering
 * 
 * If the system responds sluggishly:
 * - Increase Kp or decrease Kd
 * - Increase ALPHA and DALPHA for less filtering
 * 
 * KNOWN ISSUES:
 * 
 * Sensor Dropouts:
 * - The hall effect sensors occasionally experience communication failures
 *   when the solenoids are actuated, especially during rapid changes in current.
 * - This is likely due to electromagnetic interference or power supply fluctuations
 *   from the solenoids affecting the I2C communication with the sensors.
 * - It also seems to be something with the multiplexer
 * - The system includes automatic detection and recovery from these dropouts.
 * - To minimize dropouts:
 *   - Reduce the I2C speed in the initializeSensors function
 *   - Consider reducing the control frequency
 *   - Might need updates to the circuit (capacitors, isolation, ferrite rings)
 *******************************************************************************/

// Main sketch file for magnetic levitation PD controller
#include "definitions.h"    
#include "functions.h"

// Control parameters
constexpr float Kp = 200;
constexpr float Kd = 0.9;
constexpr float ALPHA = 0.2;
constexpr float DALPHA = 0.2;

// Sensor objects - one for each physical sensor
TLx493D_A1B6 Sensors[NUM_SENSORS] = {TLx493D_A1B6(Wire, TLx493D_IIC_ADDR_A0_e)};

// Timing parameters
constexpr float sensorFrequency = 5000.0;
constexpr int sensorInterval = round(1e6 / sensorFrequency);
constexpr float controlFrequency = 5000.0;
constexpr int controlInterval = round(1e6 / controlFrequency);

// Timing variables
unsigned long prevSensorTime = 0;
unsigned long prevControlTime = 0;
float realSamplingFreq = 0;
int controlLoopCounter = 0;

// Control variables
float magFieldX = 0, magFieldY = 0, magFieldZ = 0;
float prevMagFieldX = 0, prevMagFieldY = 0, prevMagFieldZ = 0;
float dMagFieldX = 0, dMagFieldY = 0;
float currentXPos = 0, currentXNeg = 0, currentYPos = 0, currentYNeg = 0;
float pwmInputX = 0, pwmInputY = 0;
float prevPwmInputX = 0, prevPwmInputY = 0;

// Variables for sensor freeze detection
float lastMagField[NUM_SENSORS][3] = {{0}}; // Store last readings for all sensors
int freezeCounter = 0;

void setup(){
  Serial.begin(115200);
  delay(50);

  initializeSensors();
  delay(50);

  initializeSolenoids();
  delay(50);

  calibrateSensors();
  delay(50);

  calibrateDirectFeedthrough();
  delay(50);
}

void loop(){
  unsigned long currentTime = micros();

  // Sensor reading and processing loop
  if(currentTime - prevSensorTime >= (unsigned long)sensorInterval){
    unsigned long dt = currentTime - prevSensorTime;
    if(dt > 0) realSamplingFreq = 1e6 / (float)dt;

    // Reading currents first might be more robust  
    currentXPos = getSolenoidCurrent(CURRENT_X_POS);
    currentXNeg = getSolenoidCurrent(CURRENT_X_NEG);
    currentYPos = getSolenoidCurrent(CURRENT_Y_POS);
    currentYNeg = getSolenoidCurrent(CURRENT_Y_NEG);

    // Read all sensors and process data
    readAllSensors();
    processSensorData(currentXPos, currentXNeg, currentYPos, currentYNeg);

    // Use the primary sensor for control
    magFieldX = ALPHA * rawMagFieldDetrended[PRIMARY_SENSOR_INDEX][0] + (1.0 - ALPHA) * prevMagFieldX;
    magFieldY = ALPHA * rawMagFieldDetrended[PRIMARY_SENSOR_INDEX][1] + (1.0 - ALPHA) * prevMagFieldY;
    magFieldZ = rawMagFieldDetrended[PRIMARY_SENSOR_INDEX][2];

    if(dt > 0){
      dMagFieldX = DALPHA * ((magFieldX - prevMagFieldX) / (float)dt * 1e6) + (1.0 - DALPHA) * dMagFieldX;
      dMagFieldY = DALPHA * ((magFieldY - prevMagFieldY) / (float)dt * 1e6) + (1.0 - DALPHA) * dMagFieldY;
    }

    prevMagFieldX = magFieldX;
    prevMagFieldY = magFieldY;
    prevMagFieldZ = magFieldZ;
    prevSensorTime = currentTime;

    checkSensorFreeze(lastMagField, &freezeCounter);
  }

  // Control loop
  if(currentTime - prevControlTime >= (unsigned long)controlInterval){
    if(fabs(magFieldZ) > 5){
      // Calculate and apply control signals directly
      pwmInputX = constrain(-Kp * magFieldX - Kd * dMagFieldX, -150, 150);
      pwmInputY = constrain(-Kp * magFieldY - Kd * dMagFieldY, -150, 150);
    } else {
      // When magnet is too far, set control signals to zero
      pwmInputX = 0;
      pwmInputY = 0;
    }

    applyControlSignals(pwmInputX, pwmInputY);

    prevPwmInputX = pwmInputX;
    prevPwmInputY = pwmInputY;

    controlLoopCounter++;
    prevControlTime = currentTime;

    if(controlLoopCounter % 10 == 0){
      logSystemState(currentTime, currentXPos, currentXNeg, currentYPos, currentYNeg);
    }
  }
} 
#include "definitions.h"
#include "functions.h"

// === Sensor Calibration for Direct Feedthrough ===
constexpr float DIRECT_FEEDTHROUGH_SLOPE_X_POSITIVE = 0.28;
constexpr float DIRECT_FEEDTHROUGH_SLOPE_X_NEGATIVE = 0.28;
constexpr float DIRECT_FEEDTHROUGH_SLOPE_Y_POSITIVE = 0.28;
constexpr float DIRECT_FEEDTHROUGH_SLOPE_Y_NEGATIVE = 0.28;

// === PD Controller Gains ===
constexpr float Kp = 200.0;  // Proportional gain
constexpr float Kd = 0.7;    // Derivative gain

// === Filter Constants ===
constexpr float ALPHA = 0.15;   // Filter for magnetic field
constexpr float DALPHA = 0.03;  // Filter for derivatives

// === Sensor and Multiplexer Initialization ===
// Example sensor initialization (choose your sensor class as needed)
Tle493d_a2b6 Sensor = Tle493d_a2b6(Tle493d::FASTMODE, Tle493d::TLE493D_A1);
TCA9548 mux_sensors(0x70);

// === Timing Constants ===
// Sensor sampling (for filtering) at 5 kHz:
constexpr float sensorFrequency = 5000.0;  // Hz
constexpr int sensorInterval = round(1e6 / sensorFrequency);  // microseconds

// Control update frequency at 1000 Hz:
constexpr float controlFrequency = 1000.0;  // Hz
constexpr int controlInterval = round(1e6 / controlFrequency);  // microseconds

// === Timing Variables ===
unsigned long previousSensorTime = 0;  // for sensor & filtering updates
unsigned long previousControlTime = 0; // for control (actuation) updates

// Global variable to store the actual sensor sampling frequency (in Hz)
float realSamplingFrequency = 0;

// Optionally, a counter for logging control updates
int controlLoopCounter = 0;

// === System State Variables ===
float magFieldX = 0, magFieldY = 0, magFieldZ = 0;
float rawMagFieldX = 0, rawMagFieldY = 0, rawMagFieldZ = 0;
// For filtering, store the previous filtered values:
float prevMagFieldX = 0, prevMagFieldY = 0, prevMagFieldZ = 0;
float dMagFieldX = 0, dMagFieldY = 0, dMagFieldZ = 0;
float currentXPos = 0, currentXNeg = 0, currentYPos = 0, currentYNeg = 0;
float pwmControlInputX = 0, pwmControlInputY = 0;

// === Mean Calibration Offsets ===
float meanMagFieldX = 0, meanMagFieldY = 0, meanMagFieldZ = 0;

void setup() {
    initializeSerial();
    Serial.println("Serial initialized.");

    initializeSensor();
    Serial.println("Sensor initialized.");

    initializeSolenoids();
    Serial.println("Solenoids initialized.");

    // Compute mean calibration offsets
    calibrateSensor();
}

void loop() {
    unsigned long currentTime = micros();

    // --- Sensor Update Loop (5 kHz) ---
    if (currentTime - previousSensorTime >= (unsigned long)sensorInterval) {
        // Compute the actual sensor sampling frequency:
        // (Be sure to capture the time difference before updating the timer)
        unsigned long dt = currentTime - previousSensorTime;
        if (dt > 0) {
            realSamplingFrequency = 1e6 / (float)dt;  // Convert microseconds to Hz
        }

        // Update sensor data and filter the measurements
        Sensor.updateData();
        rawMagFieldX = Sensor.getX();
        rawMagFieldY = Sensor.getY();
        rawMagFieldZ = Sensor.getZ();

        // Read solenoid currents for direct feedthrough compensation
        currentXPos = getSolenoidCurrent(CURRENT_X_POS);
        currentXNeg = getSolenoidCurrent(CURRENT_X_NEG);
        currentYPos = getSolenoidCurrent(CURRENT_Y_POS);
        currentYNeg = getSolenoidCurrent(CURRENT_Y_NEG);

        // Adjust magnetic field values for direct feedthrough
        rawMagFieldX += -meanMagFieldX + DIRECT_FEEDTHROUGH_SLOPE_X_POSITIVE * currentXPos - DIRECT_FEEDTHROUGH_SLOPE_X_NEGATIVE * currentXNeg;
        rawMagFieldY += -meanMagFieldY + DIRECT_FEEDTHROUGH_SLOPE_Y_POSITIVE * currentYPos - DIRECT_FEEDTHROUGH_SLOPE_Y_NEGATIVE * currentYNeg;
        rawMagFieldZ += -meanMagFieldZ;

        // === Filtering ===
        // Apply a first-order (exponential) low-pass filter to the sensor data:
        magFieldX = ALPHA * rawMagFieldX + (1.0 - ALPHA) * prevMagFieldX;
        magFieldY = ALPHA * rawMagFieldY + (1.0 - ALPHA) * prevMagFieldY;
        magFieldZ = ALPHA * rawMagFieldZ + (1.0 - ALPHA) * prevMagFieldZ;

        // Calculate derivatives. The term (currentTime - previousSensorTime) is in microseconds,
        // so multiplying by 1e6 converts it to per-second units.
        dMagFieldX = DALPHA * ((magFieldX - prevMagFieldX) / (float)dt * 1e6)
                      + (1.0 - DALPHA) * dMagFieldX;
        dMagFieldY = DALPHA * ((magFieldY - prevMagFieldY) / (float)dt * 1e6)
                      + (1.0 - DALPHA) * dMagFieldY;

        // Save the current filtered values for the next derivative calculation
        prevMagFieldX = magFieldX;
        prevMagFieldY = magFieldY;
        prevMagFieldZ = magFieldZ;

        // Update the sensor timer
        previousSensorTime = currentTime;
    }

    // --- Control Update Loop (1000 Hz) ---
    if (currentTime - previousControlTime >= (unsigned long)controlInterval) {
        // PD Control Logic using the most recent filtered sensor values.
        if (abs(magFieldZ) > 2) {  // Only apply control if the z-field exceeds a threshold.
            pwmControlInputX = constrain(Kp * -magFieldX + Kd * -dMagFieldX, -150, 150);
            pwmControlInputY = constrain(Kp * -magFieldY + Kd * -dMagFieldY, -150, 150);
        } else {
            pwmControlInputX = 0;
            pwmControlInputY = 0;
        }

        // Apply control signals (update PWM outputs or motor driver inputs)
        applyControlSignals();

        // Log data every 10 control updates (optional)
        if (controlLoopCounter % 10 == 0) {
            logSystemState();
        }
        controlLoopCounter++;

        // Update the control timer
        previousControlTime = currentTime;
    }
}

// === Helper Functions ===

// Apply control signals to the motors/actuators
void applyControlSignals() {
    // X-direction control
    setSolenoidInput(pwmControlInputX, MD2_IN1, MD2_IN2);
    setSolenoidInput(-pwmControlInputX, MD3_IN1, MD3_IN2);

    // Y-direction control
    setSolenoidInput(pwmControlInputY, MD4_IN1, MD4_IN2);
    setSolenoidInput(-pwmControlInputY, MD1_IN1, MD1_IN2);
}

// Calibrate sensor by computing mean offsets
void calibrateSensor() {
    Serial.println("Calibrating sensor...");
    for (int i = 0; i < 1000; i++) {
        Sensor.updateData();
        float tempX = Sensor.getX();
        float tempY = Sensor.getY();
        float tempZ = Sensor.getZ();

        meanMagFieldX += tempX / 1000.0;
        meanMagFieldY += tempY / 1000.0;
        meanMagFieldZ += tempZ / 1000.0;
        delay(1);  // Short delay for stable readings
    }
    Serial.println("Calibration complete.");
}

// Log the system's state to the Serial Monitor
void logSystemState() {
    Serial.print("realSamplingFrequency:");
    Serial.print(realSamplingFrequency, 2);
    Serial.print(",currentXPos:");
    Serial.print(currentXPos, 2);
    Serial.print(",MagFieldX:");
    Serial.print(magFieldX, 2);
    Serial.print(",MagFieldY:");
    Serial.print(magFieldY, 2);
    Serial.print(",MagFieldZ:");
    Serial.print(magFieldZ, 2);
    Serial.print(",RawMagFieldX:");
    Serial.print(rawMagFieldX, 2);
    Serial.print(",RawMagFieldY:");
    Serial.println(rawMagFieldY, 2);
}

#include "definitions.h"
#include "functions.h"

// === Sensor Calibration for Direct Feedthrough ===
constexpr float DIRECT_FEEDTHROUGH_SLOPE_X_POSITIVE = 0;
constexpr float DIRECT_FEEDTHROUGH_SLOPE_X_NEGATIVE = 0;
constexpr float DIRECT_FEEDTHROUGH_SLOPE_Y_POSITIVE = 0;
constexpr float DIRECT_FEEDTHROUGH_SLOPE_Y_NEGATIVE = 0;

// === PD Controller Gains ===
constexpr float Kp = 300.0;  // Proportional gain
constexpr float Kd = 0.5;   // Derivative gain

// === Filter Constants ===
constexpr float ALPHA = 0.18;    // Filter for magnetic field
constexpr float DALPHA = 0.3;  // Filter for derivatives

// === Sensor and Multiplexer Initialization ===
// TLx493D_A1B6 Sensor(Wire, TLx493D_IIC_ADDR_A0_e);
Tlv493d Sensor = Tlv493d();
TCA9548 mux_sensors(0x70);

// === Timing Variables ===
constexpr float samplingFrequency = 1300.0;  // Hz
constexpr int samplingInterval = round(1e6 / samplingFrequency);  // Microseconds
unsigned long previousTime = 0;
unsigned long currentTime = 0;
int loopCounter = 0;
float realSamplingFrequency = 0;

// === System State Variables ===
float magFieldX = 0, magFieldY = 0, magFieldZ = 0;
float rawMagFieldX = 0, rawMagFieldY = 0, rawMagFieldZ = 0;
float prevRawMagFieldX = 0, prevRawMagFieldY = 0, prevRawMagFieldZ = 0;
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
    // Run control loop at each sampling interval
    if (micros() - previousTime >= (unsigned long)samplingInterval) {
        currentTime = micros();
        realSamplingFrequency = 1e6/(currentTime - previousTime);
        
        // === Read Magnetic Field ===
        Sensor.updateData();
        rawMagFieldX = Sensor.getX();
        rawMagFieldY = Sensor.getY();
        rawMagFieldZ = Sensor.getZ();

        // === Read Solenoid Currents ===
        currentXPos = getSolenoidCurrent(CURRENT_X_POS);
        currentXNeg = getSolenoidCurrent(CURRENT_X_NEG);
        currentYPos = getSolenoidCurrent(CURRENT_Y_POS);
        currentYNeg = getSolenoidCurrent(CURRENT_Y_NEG);
        
        // Adjust magnetic field values for direct feedthrough
        rawMagFieldX += -meanMagFieldX + DIRECT_FEEDTHROUGH_SLOPE_X_POSITIVE*currentXPos - DIRECT_FEEDTHROUGH_SLOPE_X_NEGATIVE*currentXNeg;
        rawMagFieldY += -meanMagFieldY + DIRECT_FEEDTHROUGH_SLOPE_Y_POSITIVE*currentYPos - DIRECT_FEEDTHROUGH_SLOPE_Y_NEGATIVE*currentYNeg;
        rawMagFieldZ += -meanMagFieldZ;
        
        // Subtract calibration offsets
        magFieldX = ALPHA * rawMagFieldX + (1.0 - ALPHA) * prevMagFieldX;
        magFieldY = ALPHA * rawMagFieldY + (1.0 - ALPHA) * prevMagFieldY;
        magFieldZ = ALPHA * rawMagFieldZ + (1.0 - ALPHA) * prevMagFieldZ;

        // === Calculate Derivatives ===
        dMagFieldX = DALPHA * ((magFieldX - prevMagFieldX) / (currentTime - previousTime) * 1e6) + (1.0 - DALPHA) * dMagFieldX;
        dMagFieldY = DALPHA * ((magFieldY - prevMagFieldY) / (currentTime - previousTime) * 1e6) + (1.0 - DALPHA) * dMagFieldY;

        // === Update Previous State ===
        prevMagFieldX = magFieldX;
        prevMagFieldY = magFieldY;
        prevMagFieldZ = magFieldZ;

        // === PD Control Logic ===
        if (abs(magFieldZ) > 2) {  // Check if bz exceeds threshold
            pwmControlInputX = constrain(Kp * -magFieldX + Kd * -dMagFieldX, -150, 150);
            pwmControlInputY = constrain(Kp * -magFieldY + Kd * -dMagFieldY, -150, 150);

            // pwmControlInputX = 0;
            // pwmControlInputY = 0;
        } else {
            pwmControlInputX = 0;
            pwmControlInputY = 0;
        }

        // Apply control signals
        applyControlSignals();

        // Log data every 100 loops
        if (loopCounter % 10 == 0) {
            logSystemState();
        }

        // Update timing
        loopCounter++;
        previousTime = currentTime;


    }
}

// === Helper Functions ===

// Apply control signals to the motors
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
        // float tempX, tempY, tempZ;
        // Sensor.getMagneticField(&tempX, &tempY, &tempZ);
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

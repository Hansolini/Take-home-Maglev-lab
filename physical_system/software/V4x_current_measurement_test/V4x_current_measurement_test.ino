#include "definitions.h"
#include "functions.h"

// ############# TO BE MODIFIED ###########
// Sensor calibration for direct feedthrough
const double DIRECT_FEEDTHROUGH_SLOPE_X_POSITIVE = 0;
const double DIRECT_FEEDTHROUGH_SLOPE_X_NEGATIVE = 0;
const double DIRECT_FEEDTHROUGH_SLOPE_Y_POSITIVE = 0;
const double DIRECT_FEEDTHROUGH_SLOPE_Y_NEGATIVE = 0;
// ########################################

// === Sampling Configuration ===
constexpr double samplingFrequency = 3000;  // Hz
constexpr int samplingInterval = round(1e6 / samplingFrequency);  // Microseconds

// === Sensor and Multiplexer Initialization ===
TLx493D_A1B6 Sensor(Wire, TLx493D_IIC_ADDR_A0_e);
TCA9548 mux_sensors(0x70);

// === Timing Variables ===
unsigned long previousTime = 0;
unsigned long currentTime = 0;
int loopCounter = 0;

// === System State Variables ===
double magFieldX = 0, magFieldY = 0, magFieldZ = 0;  // Magnetic field components (mT)
double pwmControlInputX = 0, pwmControlInputY = 0;  // Control inputs

void setup() {
    initializeSerial();
    Serial.println("Serial initialized.");
    
    initializeSensor();
    Serial.println("Sensor initialized.");
    
    initializeSolenoids();
    Serial.println("Solenoids initialized.");
}

void loop() {
    // Execute control loop at each sample interval
    if (micros() - previousTime >= (unsigned long)samplingInterval) {
        currentTime = micros();

        // === Read Magnetic Field from Sensor ===
        Sensor.getMagneticField(&magFieldX, &magFieldY, &magFieldZ);

        // === Read Solenoid Currents ===
        float currentXPos = getSolenoidCurrent(CURRENT_X_POS);
        float currentXNeg = getSolenoidCurrent(CURRENT_X_NEG);
        float currentYPos = getSolenoidCurrent(CURRENT_Y_POS);
        float currentYNeg = getSolenoidCurrent(CURRENT_Y_NEG);

        // Adjust magnetic field values for direct feedthrough
        magFieldX += DIRECT_FEEDTHROUGH_SLOPE_X_POSITIVE*currentXPos - DIRECT_FEEDTHROUGH_SLOPE_X_NEGATIVE*currentXNeg;
        magFieldY += DIRECT_FEEDTHROUGH_SLOPE_Y_POSITIVE*currentYPos - DIRECT_FEEDTHROUGH_SLOPE_Y_NEGATIVE*currentYNeg;

        // === Generate Control Signals ===
        updateControlInputs();

        // === Apply Control Signals to Motors ===
        applyControlSignals();

        // === Log System State ===
        if (loopCounter % 10 == 0) {
            logSystemState(currentXPos, currentXNeg, currentYPos, currentYNeg);
        }

        // Update timing
        loopCounter++;
        previousTime = currentTime;
    }
}

// === Helper Functions ===

// Generate square wave for testing control input
void updateControlInputs() {
    // Define the step size for the triangular wave
    const int stepSize = 2;

    // Increment or decrement the control input
    static bool increasing = true;  // Direction of the wave
    if (increasing) {
        pwmControlInputX += stepSize;
        if (pwmControlInputX >= 150) {
            increasing = false;  // Switch direction at the peak
        }
    } else {
        pwmControlInputX -= stepSize;
        if (pwmControlInputX <= -150) {
            increasing = true;  // Switch direction at the trough
        }
    }

    // Keep Y control input constant
    pwmControlInputY = 0;  // Or add a similar wave if desired

    // Constrain values to valid PWM range (not strictly necessary here, but safe)
    pwmControlInputX = constrain(pwmControlInputX, -150, 150);
    pwmControlInputY = constrain(pwmControlInputY, -150, 150);
}


// Apply control signals to the motors
void applyControlSignals() {
    // X-direction control
    setSolenoidInput(-pwmControlInputX, MD3_IN1, MD3_IN2);
    setSolenoidInput(pwmControlInputX, MD2_IN1, MD2_IN2);

    // Y-direction control
    setSolenoidInput(-pwmControlInputY, MD1_IN1, MD1_IN2);
    setSolenoidInput(pwmControlInputY, MD4_IN1, MD4_IN2);
}

// Log the system's state to the Serial Monitor
void logSystemState(float currentXPos, float currentXNeg, float currentYPos, float currentYNeg) {
    Serial.print("MagFieldX:");
    Serial.print(magFieldX, 2);  // Two decimal places
    Serial.print(",MagFieldY:");
    Serial.print(magFieldY, 2);
    Serial.print(",MagFieldZ:");
    Serial.print(magFieldZ, 2);
    Serial.print(",CurrentX+:");
    Serial.print(currentXPos, 2);
    Serial.print(",CurrentX-:");
    Serial.print(currentXNeg, 2);
    Serial.print(",CurrentY+:");
    Serial.print(currentYPos, 2);
    Serial.print(",CurrentY-:");
    Serial.print(currentYNeg, 2);
    Serial.print(",ControlX:");
    Serial.print(pwmControlInputX / 100, 2);
    Serial.print(",ControlY: ");
    Serial.println(pwmControlInputY / 100, 2);
}

#include "functions.h"

extern TLx493D_A1B6 Sensors[NUM_SENSORS];
extern TCA9548 mux_sensors;
 
// Feedthrough slopes
float feedthroughSlopeX[NUM_SENSORS][2];
float feedthroughSlopeY[NUM_SENSORS][2];
float feedthroughSlopeZX[NUM_SENSORS][2];
float feedthroughSlopeZY[NUM_SENSORS][2];

// Sensor data arrays
float rawMagField[NUM_SENSORS][3] = {{0}}; // [sensorIndex][axis] where axis: 0=X, 1=Y, 2=Z
float rawMagFieldDetrended[NUM_SENSORS][3] = {{0}}; // Detrended measurements
float meanMagField[NUM_SENSORS][3] = {{0}}; // Mean (baseline) values for each sensor

// Solenoid-related functions
float getSolenoidCurrent(uint16_t pin) {
  uint16_t data = analogRead(pin);
  float voltage = (data*3.3)/1023.0;     // ADC to voltage
  float voltage_diff = voltage - 1.65;       // Centered around 1.65V (no current)
  float current = voltage_diff/(100.0*0.015); // Gain = 100 (INA214), Rshunt = 0.015Ω
  return current;
}

void setSolenoidInput(int pwm, int pin1, int pin2) {
  if (pwm > 0) {
    analogWrite(pin1, 255 - abs(pwm));
    analogWrite(pin2, 255);
  } else if (pwm < 0) {
    analogWrite(pin1, 255);
    analogWrite(pin2, 255 - abs(pwm));
  } else {
    analogWrite(pin1, 0);
    analogWrite(pin2, 0);
  }
}

void applyControlSignals(float pwmInputX, float pwmInputY) {
  setSolenoidInput(pwmInputX, MD2_IN1, MD2_IN2);
  setSolenoidInput(-pwmInputX, MD3_IN1, MD3_IN2);
  setSolenoidInput(pwmInputY, MD4_IN1, MD4_IN2);
  setSolenoidInput(-pwmInputY, MD1_IN1, MD1_IN2);
}

// Setup-related functions
void initializeSerial() {
  Serial.begin(115200);
}

// Robust initialization with bus clearing and retries
void initializeSensors() {
  // Clear the I2C bus first for reliability
  clearI2CBus();
  delay(50);
  
  // Initialize I2C communication
  Wire.begin();
  Wire.setClock(400000);
  delay(50);
  
  // Reset mux and wait briefly
  mux_sensors.reset();
  delay(10);

  // Enable all sensor channels on the mux
  for (int i = 0; i < NUM_SENSORS; i++) {
    mux_sensors.enableChannel(SENSOR_CHANNELS[i]);
  }

  // Initialize each sensor on its respective channel
  for (int i = 0; i < NUM_SENSORS; i++) {
    int channel = SENSOR_CHANNELS[i];
    
    // Select the sensor channel
    mux_sensors.selectChannel(channel);
    delay(20);
    Serial.print("Initializing sensor on channel ");
    Serial.println(channel);

    // Attempt sensor initialization with retries
    const int maxRetries = 5;
    bool initialized = false;
    for (int retry = 0; retry < maxRetries && !initialized; ++retry)
    {
        initialized =  Sensors[i].begin() && Sensors[i].isFunctional();
        // Sensors[i].setPowerMode(TLx493D_FAST_MODE_e); // Fast mode will be more noisy, but can sample faster. Requires 1Mhz wire speed. This is not supported by the MUX.
        // Sensors[i].setSensitivity(TLx493D_FULL_RANGE_e);

        if (!initialized)
        {
            Serial.printf("Retrying sensor on channel %d... attempt %d\n",
                          channel, retry + 1);
            delay(20);
        }
    }
    if (!initialized) {
      Serial.print("Sensor on channel ");
      Serial.print(channel);
      Serial.println(" failed to initialize.");
    } else {
      Serial.print("Sensor on channel ");
      Serial.print(channel);
      Serial.println(" initialized successfully.");
    }
  }
  
  // Reinitialize I2C after sensor setup to ensure stable communication
  Wire.begin();
  Wire.setClock(400000);
  delay(50);
}

void clearI2CBus() {
  pinMode(SDA, OUTPUT);
  pinMode(SCL, OUTPUT);

  digitalWrite(SDA, HIGH);
  digitalWrite(SCL, HIGH);
  delay(10);

  for (int i = 0; i < 9; i++) {
    digitalWrite(SCL, LOW);
    delayMicroseconds(10);
    digitalWrite(SCL, HIGH);
    delayMicroseconds(10);
  }

  // Generate STOP condition
  digitalWrite(SDA, LOW);
  delayMicroseconds(10);
  digitalWrite(SCL, HIGH);
  delayMicroseconds(10);
  digitalWrite(SDA, HIGH);
  delayMicroseconds(10);
}


void initializeSolenoids() {
  // Defining bit-size on read/write operations
  analogWriteResolution(8);
  analogReadResolution(10);

  // Set pin modes
  pinMode(MD1_IN1, OUTPUT);
  pinMode(MD1_IN2, OUTPUT);
  pinMode(MD2_IN1, OUTPUT);
  pinMode(MD2_IN2, OUTPUT);
  pinMode(MD3_IN1, OUTPUT);
  pinMode(MD3_IN2, OUTPUT);
  pinMode(MD4_IN1, OUTPUT);
  pinMode(MD4_IN2, OUTPUT);

  pinMode(CURRENT_Y_POS, INPUT);
  pinMode(CURRENT_X_NEG, INPUT);
  pinMode(CURRENT_X_POS, INPUT);
  pinMode(CURRENT_Y_NEG, INPUT);

  // Defining PWM frequency - using standard 31250 Hz
  analogWriteFrequency(MD1_IN1, 31250);
  analogWriteFrequency(MD1_IN2, 31250);
  analogWriteFrequency(MD2_IN1, 31250);
  analogWriteFrequency(MD2_IN2, 31250);
  analogWriteFrequency(MD3_IN1, 31250);
  analogWriteFrequency(MD3_IN2, 31250);
  analogWriteFrequency(MD4_IN1, 31250);
  analogWriteFrequency(MD4_IN2, 31250);

  // Setting initial state to 0
  digitalWrite(MD1_IN1, LOW);
  digitalWrite(MD1_IN2, LOW);
  digitalWrite(MD2_IN1, LOW);
  digitalWrite(MD2_IN2, LOW);
  digitalWrite(MD3_IN1, LOW);
  digitalWrite(MD3_IN2, LOW);
  digitalWrite(MD4_IN1, LOW);
  digitalWrite(MD4_IN2, LOW);
}

// Sensor management functions
void readAllSensors()
{
    for (int i = 0; i < NUM_SENSORS; ++i)
    {
        // 1) point the TCA9548A at the current sensor
        // mux_sensors.selectChannel(SENSOR_CHANNELS[i]);
        delayMicroseconds(50);          // let the bus lines settle

        // 2) read one complete XYZ conversion
        double x, y, z;
        if (Sensors[i].getMagneticField(&x, &y, &z))
        {
            rawMagField[i][0] = static_cast<float>(x);
            rawMagField[i][1] = static_cast<float>(y);
            rawMagField[i][2] = static_cast<float>(z);
        }
    }
}

void processSensorData(float currentXPos, float currentXNeg, float currentYPos, float currentYNeg) {
  // Process all sensor data
  for (int i = 0; i < NUM_SENSORS; i++) {
    // Apply feedthrough compensation for each sensor
    rawMagFieldDetrended[i][0] = rawMagField[i][0] - meanMagField[i][0] + 
                                 feedthroughSlopeX[i][0] * currentXPos + 
                                 feedthroughSlopeX[i][1] * currentXNeg;
    
    rawMagFieldDetrended[i][1] = rawMagField[i][1] - meanMagField[i][1] + 
                                 feedthroughSlopeY[i][0] * currentYPos + 
                                 feedthroughSlopeY[i][1] * currentYNeg;
    
    rawMagFieldDetrended[i][2] = rawMagField[i][2] - meanMagField[i][2] + 
                                 feedthroughSlopeZX[i][0] * currentXPos + 
                                 feedthroughSlopeZX[i][1] * currentXNeg + 
                                 feedthroughSlopeZY[i][0] * currentYPos + 
                                 feedthroughSlopeZY[i][1] * currentYNeg;
  }
}

void checkSensorFreeze(float lastMagField[][3], int* freezeCounter) {
  const int freezeLimit = 20;
  bool anyFrozen = false;
  
  // Check all sensors for freezes
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (rawMagField[i][0] == lastMagField[i][0] && 
        rawMagField[i][1] == lastMagField[i][1] && 
        rawMagField[i][2] == lastMagField[i][2]) {
      anyFrozen = true;
    }
    
    // Update last values for each sensor
    lastMagField[i][0] = rawMagField[i][0];
    lastMagField[i][1] = rawMagField[i][1];
    lastMagField[i][2] = rawMagField[i][2];
  }
  
  // Increment or reset freeze counter
  if (anyFrozen) {
    (*freezeCounter)++;
  } else {
    *freezeCounter = 0;
  }

  // Reset if freeze detected for too long
  if(*freezeCounter >= freezeLimit){
    Serial.println("Sensor freeze detected! Resetting...");
    resetSensorAndI2C();
    *freezeCounter = 0;
    delay(1000);
  }
}

void resetSensorAndI2C() {
  Serial.println("Resetting I2C bus and sensor...");
  
  // Call improved initializeSensors which already handles I2C reset
  initializeSensors();
  
  Serial.println("I2C and sensor reset complete");
}

void logSystemState(unsigned long currentTime, float currentXPos, float currentXNeg, float currentYPos, float currentYNeg) {
  // Convert currentTime from microseconds to seconds as a float
  float timeSec = currentTime / 1000000.0;
  
  // Log time and solenoid currents first
  Serial.print("time:");
  Serial.print(timeSec, 6);
  Serial.print(",Ix_plus:");
  Serial.print(currentXPos, 6);
  Serial.print(",Ix_minus:");
  Serial.print(currentXNeg, 6);
  Serial.print(",Iy_plus:");
  Serial.print(currentYPos, 6);
  Serial.print(",Iy_minus:");
  Serial.print(currentYNeg, 6);
  
  // Dynamically log data from all active sensors
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(",bx");
    Serial.print(i);
    Serial.print(":");
    Serial.print(rawMagFieldDetrended[i][0], 6);
    
    Serial.print(",by");
    Serial.print(i);
    Serial.print(":");
    Serial.print(rawMagFieldDetrended[i][1], 6);
    
    Serial.print(",bz");
    Serial.print(i);
    Serial.print(":");
    
    // Add newline only after the last value
    if (i == NUM_SENSORS - 1) {
      Serial.println(rawMagFieldDetrended[i][2], 6);
    } else {
      Serial.print(rawMagFieldDetrended[i][2], 6);
    }
  }
}

void calibrateSensors()
{
    Serial.println("Calibrating sensors...");

    constexpr int SAMPLES = 1000;

    /*  clear the running sums */
    for (int s = 0; s < NUM_SENSORS; ++s) {
        meanMagField[s][0] = meanMagField[s][1] = meanMagField[s][2] = 0.0f;
    }

    /*  collect SAMPLES readings from every sensor */
    for (int n = 0; n < SAMPLES; ++n) {
        for (int s = 0; s < NUM_SENSORS; ++s) {
            mux_sensors.selectChannel(SENSOR_CHANNELS[s]);
            delayMicroseconds(10);          // bus-settle after mux switch

            double x, y, z;
            if (Sensors[s].getMagneticField(&x, &y, &z))
            {
                meanMagField[s][0] += static_cast<float>(x);
                meanMagField[s][1] += static_cast<float>(y);
                meanMagField[s][2] += static_cast<float>(z);
            }
            /* else: bad frame – ignore, keep previous total */

            delay(1);                       // ~1 kSps per sensor, like original
        }
    }

    /*  convert accumulated sums into arithmetic means */
    for (int s = 0; s < NUM_SENSORS; ++s) {
        meanMagField[s][0] /= SAMPLES;
        meanMagField[s][1] /= SAMPLES;
        meanMagField[s][2] /= SAMPLES;
    }

    Serial.println("Sensor calibration complete.");

    /*  dump the new offsets for reference */
    for (int s = 0; s < NUM_SENSORS; ++s) {
        Serial.print("Sensor "); Serial.print(s); Serial.print(" mean XYZ = ");
        Serial.print(meanMagField[s][0]); Serial.print(", ");
        Serial.print(meanMagField[s][1]); Serial.print(", ");
        Serial.println(meanMagField[s][2]);
    }
}


void calibrateDirectFeedthrough()
{
    Serial.println("Calibrating feedthrough…");

    /*--- user-tunable parameters ---*/
    constexpr int  segTime      = 100;      // ms of data per current level
    constexpr int  settleDelay  = 50;       // ms after changing PWM
    constexpr int  numLevels    = 3;
    const     float currentLevels[numLevels] = { 50.0f, 100.0f, 150.0f };

    /*--- solenoid pin map ---*/
    struct SolenoidConfig {
        uint8_t in1;
        uint8_t in2;
        int     currentChannel;
        bool    isPositive;
    };

    const SolenoidConfig solenoidConfigs[4] = {
        { MD2_IN1, MD2_IN2, CURRENT_X_POS, true  }, // X+
        { MD3_IN1, MD3_IN2, CURRENT_X_NEG, false }, // X-
        { MD4_IN1, MD4_IN2, CURRENT_Y_POS, true  }, // Y+
        { MD1_IN1, MD1_IN2, CURRENT_Y_NEG, false }  // Y-
    };

    /*--- accumulators ---*/
    float sumSensor[NUM_SENSORS][3][4] = {0}; // axis 0=X 1=Y 2=Z
    float sumCurrent[4]                 = {0};
    int   counts[4]                     = {0};

    /*--------------------------------------------------
     *  sweep through each solenoid direction
     *-------------------------------------------------*/
    for (int sol = 0; sol < 4; ++sol)
    {
        Serial.print(F("Calibrating feedthrough for solenoid "));
        Serial.println(sol);

        for (int lvl = 0; lvl < numLevels; ++lvl)
        {
            const float pwm = currentLevels[lvl];
            setSolenoidInput(pwm,
                             solenoidConfigs[sol].in1,
                             solenoidConfigs[sol].in2);
            delay(settleDelay);

            const unsigned long t0 = millis();
            while (millis() - t0 < segTime)
            {
                /* measure coil current */
                const float I = getSolenoidCurrent(solenoidConfigs[sol].currentChannel);
                sumCurrent[sol] += I;

                /* measure all magnetometers */
                for (int s = 0; s < NUM_SENSORS; ++s)
                {
                    mux_sensors.selectChannel(SENSOR_CHANNELS[s]);
                    delayMicroseconds(50);   // bus settle

                    double x, y, z;
                    if (Sensors[s].getMagneticField(&x, &y, &z))
                    {
                        sumSensor[s][0][sol] += static_cast<float>(x);
                        sumSensor[s][1][sol] += static_cast<float>(y);
                        sumSensor[s][2][sol] += static_cast<float>(z);
                    }
                }

                ++counts[sol];
                delay(10);
            }

            /*  turn coil off before next PWM level  */
            setSolenoidInput(0,
                             solenoidConfigs[sol].in1,
                             solenoidConfigs[sol].in2);
            delay(50);
        }
    }

    /*--------------------------------------------------
     *  convert sums ⇒ slopes
     *-------------------------------------------------*/
    for (int s = 0; s < NUM_SENSORS; ++s)
    {
        for (int sol = 0; sol < 4; ++sol)
        {
            if (counts[sol] == 0) continue;
            const float Iavg = sumCurrent[sol] / counts[sol];

            /* X axis – affected only by solenoids 0 and 1 */
            if (sol < 2)
            {
                const float Sx = sumSensor[s][0][sol] / counts[sol];
                const float k  = (Sx - meanMagField[s][0]) / Iavg;
                feedthroughSlopeX[s][ solenoidConfigs[sol].isPositive ? 0 : 1 ] = -k;
            }

            /* Y axis – affected only by solenoids 2 and 3 */
            if (sol >= 2)
            {
                const float Sy = sumSensor[s][1][sol] / counts[sol];
                const float k  = (Sy - meanMagField[s][1]) / Iavg;
                feedthroughSlopeY[s][ solenoidConfigs[sol].isPositive ? 0 : 1 ] = -k;
            }

            /* Z axis – affected by all solenoids */
            const float Sz = sumSensor[s][2][sol] / counts[sol];
            const float kZ = (Sz - meanMagField[s][2]) / Iavg;

            if (sol < 2)  // X coils’ impact on Z
                feedthroughSlopeZX[s][ solenoidConfigs[sol].isPositive ? 0 : 1 ] = -kZ;
            else          // Y coils’ impact on Z
                feedthroughSlopeZY[s][ solenoidConfigs[sol].isPositive ? 0 : 1 ] = -kZ;
        }
    }

    Serial.println(F("Feedthrough calibration complete."));
}


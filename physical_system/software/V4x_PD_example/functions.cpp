#include "functions.h"

extern Tlv493d Sensors[];
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
    for (int retry = 0; retry < maxRetries && !initialized; retry++) {
      Sensors[i].begin();
      delay(20);
      Sensors[i].setAccessMode(Sensors[i].MASTERCONTROLLEDMODE);
      Sensors[i].disableTemp();
      delay(20);

      initialized = sensorInitializedCorrectly(i);
      if (!initialized) {
        Serial.print("Retrying sensor initialization on channel ");
        Serial.print(channel);
        Serial.print("... attempt ");
        Serial.println(retry + 1);
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

bool sensorInitializedCorrectly(int sensorIndex){
  Sensors[sensorIndex].updateData();
  float checkX = Sensors[sensorIndex].getX();
  float checkY = Sensors[sensorIndex].getY();
  float checkZ = Sensors[sensorIndex].getZ();

  return (abs(checkX) > 0.00001 || abs(checkY) > 0.00001 || abs(checkZ) > 0.00001);
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
void readAllSensors() {
  // Read all sensors - each with its own sensor object
  for (int i = 0; i < NUM_SENSORS; i++) {
    mux_sensors.enableChannel(SENSOR_CHANNELS[i]); // RE-enabling the channel each time to prevent sensor from freezing (?)
    mux_sensors.selectChannel(SENSOR_CHANNELS[i]);
    delayMicroseconds(10);

    Sensors[i].updateData();
    
    rawMagField[i][0] = Sensors[i].getX();
    rawMagField[i][1] = Sensors[i].getY();
    rawMagField[i][2] = Sensors[i].getZ();
    delayMicroseconds(10);
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

void calibrateSensors(){
  Serial.println("Calibrating sensors...");
  for(int i = 0; i < 1000; i++){
    for (int j = 0; j < NUM_SENSORS; j++) {
      mux_sensors.selectChannel(SENSOR_CHANNELS[j]);
      delayMicroseconds(10);
      Sensors[j].updateData();
      float tempX = Sensors[j].getX();
      float tempY = Sensors[j].getY();
      float tempZ = Sensors[j].getZ();
      meanMagField[j][0] += tempX / 1000.0;
      meanMagField[j][1] += tempY / 1000.0;
      meanMagField[j][2] += tempZ / 1000.0;
      delay(1);
    }
  }
  Serial.println("Sensor calibration complete.");

  for (int j = 0; j < NUM_SENSORS; j++) {
    Serial.println(meanMagField[j][0]);
    Serial.println(meanMagField[j][1]);
    Serial.println(meanMagField[j][2]);
  }
}

void calibrateDirectFeedthrough(){
  Serial.println("Calibrating feedthrough...");
  
  // Calibration parameters
  const int segTime = 100;
  const int settleDelay = 50;
  const int numLevels = 3;
  float currentLevels[numLevels] = {50.0, 100.0, 150.0};
  
  // Define solenoid pin configurations for calibration
  struct SolenoidConfig {
    uint8_t in1;
    uint8_t in2;
    int currentChannel;
    bool isPositive;
  };
  
  SolenoidConfig solenoidConfigs[4] = {
    {MD2_IN1, MD2_IN2, CURRENT_X_POS, true},  // X Positive
    {MD3_IN1, MD3_IN2, CURRENT_X_NEG, false}, // X Negative
    {MD4_IN1, MD4_IN2, CURRENT_Y_POS, true},  // Y Positive
    {MD1_IN1, MD1_IN2, CURRENT_Y_NEG, false}  // Y Negative
  };
  
  // Arrays to store summed measurements for slope calculation
  float sumSensor[NUM_SENSORS][3][4]; // [sensorIndex][axis][solenoidIndex]
  float sumCurrent[4];                // [solenoidIndex]
  int counts[4] = {0};                // [solenoidIndex]
  
  // For each solenoid direction 
  for (int solenoidIdx = 0; solenoidIdx < 4; solenoidIdx++) {
    Serial.print("Calibrating feedthrough for solenoid ");
    Serial.println(solenoidIdx);
    
    // Reset summed values
    for (int i = 0; i < NUM_SENSORS; i++) {
      for (int j = 0; j < 3; j++) {
        sumSensor[i][j][solenoidIdx] = 0;
      }
    }
    sumCurrent[solenoidIdx] = 0;
    counts[solenoidIdx] = 0;
    
    // For each current level
    for (int level = 0; level < numLevels; level++) {
      float appliedCurrent = currentLevels[level];
      
      // Apply the current to this solenoid
      setSolenoidInput(appliedCurrent, solenoidConfigs[solenoidIdx].in1, solenoidConfigs[solenoidIdx].in2);
      delay(settleDelay);
      
      // Measure for segTime duration
      unsigned long startTime = millis();
      while (millis() - startTime < segTime) {
        // Measure solenoid current
        float currentMeasured = getSolenoidCurrent(solenoidConfigs[solenoidIdx].currentChannel);
        sumCurrent[solenoidIdx] += currentMeasured;
        
        // Read all sensors for this current measurement
        for (int sensorIdx = 0; sensorIdx < NUM_SENSORS; sensorIdx++) {
          mux_sensors.selectChannel(SENSOR_CHANNELS[sensorIdx]);
          delayMicroseconds(50);
          Sensors[sensorIdx].updateData();
          
          // Store measurements for X, Y, Z axes
          sumSensor[sensorIdx][0][solenoidIdx] += Sensors[sensorIdx].getX();
          sumSensor[sensorIdx][1][solenoidIdx] += Sensors[sensorIdx].getY();
          sumSensor[sensorIdx][2][solenoidIdx] += Sensors[sensorIdx].getZ();
        }
        
        counts[solenoidIdx]++;
        delay(10);
      }
      
      // Turn off the solenoid current after measurement
      setSolenoidInput(0, solenoidConfigs[solenoidIdx].in1, solenoidConfigs[solenoidIdx].in2);
      delay(50);
    }
  }
  
  // Calculate slopes for each sensor and axis
  for (int sensorIdx = 0; sensorIdx < NUM_SENSORS; sensorIdx++) {
    for (int solenoidIdx = 0; solenoidIdx < 4; solenoidIdx++) {
      if (counts[solenoidIdx] > 0) {
        float avgCurrent = sumCurrent[solenoidIdx] / counts[solenoidIdx];
        
        // X axis - Only affected by X solenoids (0 and 1)
        if (solenoidIdx < 2) {
          float avgSensor = sumSensor[sensorIdx][0][solenoidIdx] / counts[solenoidIdx];
          float slope = (avgSensor - meanMagField[sensorIdx][0]) / avgCurrent;
          
          if (solenoidConfigs[solenoidIdx].isPositive) {
            feedthroughSlopeX[sensorIdx][0] = -slope; // X positive
          } else {
            feedthroughSlopeX[sensorIdx][1] = -slope; // X negative
          }
        }
        
        // Y axis - Only affected by Y solenoids (2 and 3)
        if (solenoidIdx >= 2) {
          float avgSensor = sumSensor[sensorIdx][1][solenoidIdx] / counts[solenoidIdx];
          float slope = (avgSensor - meanMagField[sensorIdx][1]) / avgCurrent;
          
          if (solenoidConfigs[solenoidIdx].isPositive) {
            feedthroughSlopeY[sensorIdx][0] = -slope; // Y positive
          } else {
            feedthroughSlopeY[sensorIdx][1] = -slope; // Y negative
          }
        }
        
        // Z axis - Affected by all solenoids
        float avgSensor = sumSensor[sensorIdx][2][solenoidIdx] / counts[solenoidIdx];
        float slope = (avgSensor - meanMagField[sensorIdx][2]) / avgCurrent;
        
        if (solenoidIdx < 2) { // X solenoids effect on Z
          if (solenoidConfigs[solenoidIdx].isPositive) {
            feedthroughSlopeZX[sensorIdx][0] = -slope; // X positive effect on Z
          } else {
            feedthroughSlopeZX[sensorIdx][1] = -slope; // X negative effect on Z
          }
        } else { // Y solenoids effect on Z
          if (solenoidConfigs[solenoidIdx].isPositive) {
            feedthroughSlopeZY[sensorIdx][0] = -slope; // Y positive effect on Z
          } else {
            feedthroughSlopeZY[sensorIdx][1] = -slope; // Y negative effect on Z
          }
        }
      }
    }
  }
  
  Serial.println("Feedthrough calibration complete.");
  
  // Print results for debugging
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(" (Channel ");
    Serial.print(SENSOR_CHANNELS[i]);
    Serial.println("):");
    
    Serial.print("  X: Pos=");
    Serial.print(feedthroughSlopeX[i][0]);
    Serial.print(", Neg=");
    Serial.println(feedthroughSlopeX[i][1]);
    
    Serial.print("  Y: Pos=");
    Serial.print(feedthroughSlopeY[i][0]);
    Serial.print(", Neg=");
    Serial.println(feedthroughSlopeY[i][1]);
    
    Serial.print("  ZX: Pos=");
    Serial.print(feedthroughSlopeZX[i][0]);
    Serial.print(", Neg=");
    Serial.println(feedthroughSlopeZX[i][1]);
    
    Serial.print("  ZY: Pos=");
    Serial.print(feedthroughSlopeZY[i][0]);
    Serial.print(", Neg=");
    Serial.println(feedthroughSlopeZY[i][1]);
  }
}
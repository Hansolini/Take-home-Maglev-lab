// Problems
// - Still freezes once in a while, and reset is not fast enough to fix during levitation
// - 


#include "definitions.h"
#include "functions.h"

// Feedthrough slopes for X and Y (default values, tuned automatically)
float feedthroughSlopeXPos = 0.5;
float feedthroughSlopeXNeg = 0.5;
float feedthroughSlopeYPos = 0.5;
float feedthroughSlopeYNeg = 0.5;

constexpr float Kp = 140;
constexpr float Kd = 0.4;
constexpr float ALPHA = 0.20;
constexpr float DALPHA = 0.05;
constexpr float PWMALPHA = 0.3;

Tlv493d Sensor = Tlv493d();
TCA9548 mux_sensors(0x70);

constexpr float sensorFrequency = 5000.0;
constexpr int sensorInterval = round(1e6 / sensorFrequency);
constexpr float controlFrequency = 5000.0;
constexpr int controlInterval = round(1e6 / controlFrequency);

unsigned long prevSensorTime = 0;
unsigned long prevControlTime = 0;
float realSamplingFreq = 0;
int controlLoopCounter = 0;

float magFieldX = 0, magFieldY = 0, magFieldZ = 0;
float rawMagFieldX = 0, rawMagFieldY = 0, rawMagFieldZ = 0;
float rawMagFieldXDetrended = 0, rawMagFieldYDetrended = 0, rawMagFieldZDetrended = 0;
float prevMagFieldX = 0, prevMagFieldY = 0, prevMagFieldZ = 0;
float dMagFieldX = 0, dMagFieldY = 0;
float currentXPos = 0, currentXNeg = 0, currentYPos = 0, currentYNeg = 0;
float pwmInputX = 0, pwmInputY = 0;
float prevPwmInputX = 0, prevPwmInputY = 0;

float meanMagFieldX = 0, meanMagFieldY = 0, meanMagFieldZ = 0;

// Variables for sensor freeze detection
float lastMagFieldX = 0, lastMagFieldY = 0, lastMagFieldZ = 0;
int freezeCounter = 0;
const int freezeLimit = 20;

void setup(){
  Serial.begin(115200);
  delay(100);

  clearI2CBus();
  delay(50);

  Wire.begin();
  Wire.setClock(1000000);
  delay(50);

  initializeSensor();
  delay(50);

  Wire.begin();
  Wire.setClock(1000000);
  delay(50);

  initializeSolenoids();
  calibrateSensor();
  calibrateDirectFeedthrough();
}

void loop(){
  unsigned long currentTime = micros();

  if(currentTime - prevSensorTime >= (unsigned long)sensorInterval){
    unsigned long dt = currentTime - prevSensorTime;
    if(dt > 0) realSamplingFreq = 1e6 / (float)dt;

    // Reading currents first might be more robust  
    currentXPos = getSolenoidCurrent(CURRENT_X_POS);
    currentXNeg = getSolenoidCurrent(CURRENT_X_NEG);
    currentYPos = getSolenoidCurrent(CURRENT_Y_POS);
    currentYNeg = getSolenoidCurrent(CURRENT_Y_NEG);

    Sensor.updateData();
    rawMagFieldX = Sensor.getX();
    rawMagFieldY = Sensor.getY();
    rawMagFieldZ = Sensor.getZ();

    rawMagFieldXDetrended = rawMagFieldX - meanMagFieldX + feedthroughSlopeXPos * currentXPos - feedthroughSlopeXNeg * currentXNeg;
    rawMagFieldYDetrended = rawMagFieldY - meanMagFieldY + feedthroughSlopeYPos * currentYPos - feedthroughSlopeYNeg * currentYNeg;
    rawMagFieldZDetrended = rawMagFieldZ - meanMagFieldZ;

    magFieldX = ALPHA * rawMagFieldXDetrended + (1.0 - ALPHA) * prevMagFieldX;
    magFieldY = ALPHA * rawMagFieldYDetrended + (1.0 - ALPHA) * prevMagFieldY;
    magFieldZ = rawMagFieldZDetrended;

    if(dt > 0){
      dMagFieldX = DALPHA * ((magFieldX - prevMagFieldX) / (float)dt * 1e6) + (1.0 - DALPHA) * dMagFieldX;
      dMagFieldY = DALPHA * ((magFieldY - prevMagFieldY) / (float)dt * 1e6) + (1.0 - DALPHA) * dMagFieldY;
    }

    prevMagFieldX = magFieldX;
    prevMagFieldY = magFieldY;
    prevMagFieldZ = magFieldZ;
    prevSensorTime = currentTime;

    checkSensorFreeze();
  }

  if(currentTime - prevControlTime >= (unsigned long)controlInterval){
    if(fabs(magFieldZ) > 18){
      pwmInputX = constrain(-Kp * magFieldX - Kd * dMagFieldX, -150, 150);
      pwmInputY = constrain(-Kp * magFieldY - Kd * dMagFieldY, -150, 150);
 
      pwmInputX = PWMALPHA*pwmInputX + (1.0 - PWMALPHA)*prevPwmInputX;
      pwmInputY = PWMALPHA*pwmInputY + (1.0 - PWMALPHA)*prevPwmInputY;

    } else {
      pwmInputX = 0;
      pwmInputY = 0;
    }

    applyControlSignals();

    prevPwmInputX = pwmInputX;
    prevPwmInputY = pwmInputY;

    controlLoopCounter++;
    prevControlTime = currentTime;

    if(controlLoopCounter % 100 == 0){
      logSystemState(currentTime);
    }
  }
}

void checkSensorFreeze(){
  if(rawMagFieldX == lastMagFieldX && rawMagFieldY == lastMagFieldY && rawMagFieldZ == lastMagFieldZ){
    freezeCounter++;
  } else {
    freezeCounter = 0;
  }

  if(freezeCounter >= freezeLimit){
    Serial.println("Sensor freeze detected! Resetting...");
    resetSensorAndI2C();
    freezeCounter = 0;
  }

  lastMagFieldX = rawMagFieldX;
  lastMagFieldY = rawMagFieldY;
  lastMagFieldZ = rawMagFieldZ;
}




void resetSensorAndI2C(){
  Serial.println("Performing sensor & bus reset...");

  mux_sensors.reset();
  delay(10);
  clearI2CBus();
  delay(10);

  Wire.begin();
  Wire.setClock(1000000);
  delay(20);

  mux_sensors.selectChannel(7);
  delay(10);

  Sensor.begin();
  delay(20);
  Sensor.setAccessMode(Sensor.MASTERCONTROLLEDMODE);
  Sensor.disableTemp();
  delay(20);

  // Re-initialize Wire explicitly after sensor initialization
  Wire.begin();  
  Wire.setClock(1000000);
  delay(20);

  Serial.println("Sensor reset completed.");
}














void logSystemState(unsigned long currentTime) {
  Serial.print("realSamplingFreq:");
  Serial.print(realSamplingFreq, 2);
  Serial.print(",currentXPos:");
  Serial.print(currentXPos, 2);
  Serial.print(",currentXNeg:");
  Serial.print(currentXNeg, 2);
  Serial.print(",currentYPos:");
  Serial.print(currentYPos, 2);
  Serial.print(",currentYNeg:");
  Serial.print(currentYNeg, 2);
  Serial.print(",MagFieldX:");
  Serial.print(magFieldX, 2);
  Serial.print(",MagFieldY:");
  Serial.print(magFieldY, 2);
  Serial.print(",rawMagFieldX:");
  Serial.print(rawMagFieldX, 2);
  Serial.print(",rawMagFieldY:");
  Serial.print(rawMagFieldY, 2);
  Serial.print(",rawMagFieldZ:");
  Serial.println(rawMagFieldZ, 2);
}


void applyControlSignals(){
  setSolenoidInput(pwmInputX, MD2_IN1, MD2_IN2);
  setSolenoidInput(-pwmInputX, MD3_IN1, MD3_IN2);
  setSolenoidInput(pwmInputY, MD4_IN1, MD4_IN2);
  setSolenoidInput(-pwmInputY, MD1_IN1, MD1_IN2);
}

void calibrateSensor(){
  Serial.println("Calibrating sensor...");
  for(int i = 0; i < 1000; i++){
    Sensor.updateData();
    float tempX = Sensor.getX();
    float tempY = Sensor.getY();
    float tempZ = Sensor.getZ();
    meanMagFieldX += tempX / 1000.0;
    meanMagFieldY += tempY / 1000.0;
    meanMagFieldZ += tempZ / 1000.0;
    delay(1);
  }
  Serial.println("Sensor calibration complete.");
}

void calibrateDirectFeedthrough(){
  Serial.println("Calibrating feedthrough...");
  feedthroughSlopeXPos = calibrateChannel(MD2_IN1, MD2_IN2, CURRENT_X_POS, meanMagFieldX, true, 'X');
  feedthroughSlopeXNeg = calibrateChannel(MD3_IN1, MD3_IN2, CURRENT_X_NEG, meanMagFieldX, false, 'X');
  feedthroughSlopeYPos = calibrateChannel(MD4_IN1, MD4_IN2, CURRENT_Y_POS, meanMagFieldY, true, 'Y');
  feedthroughSlopeYNeg = calibrateChannel(MD1_IN1, MD1_IN2, CURRENT_Y_NEG, meanMagFieldY, false, 'Y');
  
  Serial.println("Feedthrough calibration complete.");
}

float calibrateChannel(uint8_t in1, uint8_t in2, int currentChannel, float meanValue, bool invertSlope, char axis){
  const int segTime = 100;
  const int settleDelay = 50;
  const int numLevels = 3;
  float currentLevels[numLevels] = {50.0, 100.0, 150.0};
  float totalSlope = 0;
  
  for(int i = 0; i < numLevels; i++){
    float appliedCurrent = currentLevels[i];
    setSolenoidInput(appliedCurrent, in1, in2);
    delay(settleDelay);
    float sumSensor = 0, sumCurrent = 0;
    int count = 0;
    unsigned long startTime = millis();
    while(millis() - startTime < segTime){
      float currentMeasured = getSolenoidCurrent(currentChannel);
      
      Sensor.updateData();
      float sensorVal = (axis == 'X' ? Sensor.getX() : (axis == 'Y' ? Sensor.getY() : Sensor.getZ()));
      sumSensor += sensorVal;
      sumCurrent += currentMeasured;
      count++;
      delay(10);
    }
    setSolenoidInput(0, in1, in2);
    float avgSensor = sumSensor / count;
    float avgCurrent = sumCurrent / count;
    float measuredSlope = (avgSensor - meanValue) / avgCurrent;
    if(invertSlope) measuredSlope = -measuredSlope;
    totalSlope += measuredSlope;
    delay(50);
  }
  return totalSlope / numLevels;
}

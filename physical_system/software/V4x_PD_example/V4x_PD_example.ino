#include "definitions.h"
#include "functions.h"

float feedthroughSlopeXPos = 0.5;
float feedthroughSlopeXNeg = 0.5;
float feedthroughSlopeYPos = 0.5;
float feedthroughSlopeYNeg = 0.5;
float feedthroughSlopeZ = 0.5;

constexpr float Kp = 120.0;
constexpr float Kd = 0.5;

constexpr float ALPHA = 0.10;
constexpr float DALPHA = 0.02;

Tlv493d Sensor = Tlv493d();
TCA9548 mux_sensors(0x70);

constexpr float sensorFrequency = 5000.0;
constexpr int sensorInterval = round(1e6/sensorFrequency);

constexpr float controlFrequency = 1000.0;
constexpr int controlInterval = round(1e6/controlFrequency);

unsigned long prevSensorTime = 0;
unsigned long prevControlTime = 0;
float realSamplingFreq = 0;
int controlLoopCounter = 0;

float magFieldX = 0, magFieldY = 0, magFieldZ = 0;
float rawMagFieldX = 0, rawMagFieldY = 0, rawMagFieldZ = 0;
float prevMagFieldX = 0, prevMagFieldY = 0, prevMagFieldZ = 0;
float dMagFieldX = 0, dMagFieldY = 0, dMagFieldZ = 0;
float currentXPos = 0, currentXNeg = 0, currentYPos = 0, currentYNeg = 0;
float pwmInputX = 0, pwmInputY = 0;

float meanMagFieldX = 0, meanMagFieldY = 0, meanMagFieldZ = 0;

void setup(){
  initializeSerial();
  Serial.println("Serial initialized.");
  initializeSensor();
  Serial.println("Sensor initialized.");
  initializeSolenoids();
  Serial.println("Solenoids initialized.");
  calibrateSensor();
  calibrateDirectFeedthrough();
}

void loop(){
  unsigned long currentTime = micros();
  if(currentTime - prevSensorTime >= (unsigned long)sensorInterval){
    unsigned long dt = currentTime - prevSensorTime;
    if(dt > 0) realSamplingFreq = 1e6/(float)dt;
    
    Sensor.updateData();
    rawMagFieldX = Sensor.getX();
    rawMagFieldY = Sensor.getY();
    rawMagFieldZ = Sensor.getZ();

    currentXPos = getSolenoidCurrent(CURRENT_X_POS);
    currentXNeg = getSolenoidCurrent(CURRENT_X_NEG);
    currentYPos = getSolenoidCurrent(CURRENT_Y_POS);
    currentYNeg = getSolenoidCurrent(CURRENT_Y_NEG);

    rawMagFieldX += -meanMagFieldX + feedthroughSlopeXPos*currentXPos - feedthroughSlopeXNeg*currentXNeg;
    rawMagFieldY += -meanMagFieldY + feedthroughSlopeYPos*currentYPos - feedthroughSlopeYNeg*currentYNeg;
    rawMagFieldZ += -meanMagFieldZ - feedthroughSlopeZ*(currentXPos+currentXNeg+currentYPos+currentYNeg);

    magFieldX = ALPHA*rawMagFieldX + (1.0-ALPHA)*prevMagFieldX;
    magFieldY = ALPHA*rawMagFieldY + (1.0-ALPHA)*prevMagFieldY;
    magFieldZ = ALPHA*rawMagFieldZ + (1.0-ALPHA)*prevMagFieldZ;

    dMagFieldX = DALPHA*((magFieldX-prevMagFieldX)/(float)dt*1e6) + (1.0-DALPHA)*dMagFieldX;
    dMagFieldY = DALPHA*((magFieldY-prevMagFieldY)/(float)dt*1e6) + (1.0-DALPHA)*dMagFieldY;

    prevMagFieldX = magFieldX;
    prevMagFieldY = magFieldY;
    prevMagFieldZ = magFieldZ;
    prevSensorTime = currentTime;
  }

  if(currentTime - prevControlTime >= (unsigned long)controlInterval){
    if(abs(magFieldZ) > 2){
      pwmInputX = constrain(Kp*-magFieldX + Kd*-dMagFieldX, -150, 150);
      pwmInputY = constrain(Kp*-magFieldY + Kd*-dMagFieldY, -150, 150);
    } else {
      pwmInputX = 0;
      pwmInputY = 0;
    }
    applyControlSignals();
    if(controlLoopCounter % 10 == 0) logSystemState();
    controlLoopCounter++;
    prevControlTime = currentTime;
  }
}

void applyControlSignals(){
  setSolenoidInput(pwmInputX,MD2_IN1,MD2_IN2);
  setSolenoidInput(-pwmInputX,MD3_IN1,MD3_IN2);
  setSolenoidInput(pwmInputY,MD4_IN1,MD4_IN2);
  setSolenoidInput(-pwmInputY,MD1_IN1,MD1_IN2);
}

void calibrateSensor(){
  Serial.println("Calibrating sensor...");
  for(int i=0;i<1000;i++){
    Sensor.updateData();
    float tempX = Sensor.getX();
    float tempY = Sensor.getY();
    float tempZ = Sensor.getZ();
    meanMagFieldX += tempX/1000.0;
    meanMagFieldY += tempY/1000.0;
    meanMagFieldZ += tempZ/1000.0;
    delay(1);
  }
  Serial.println("Sensor calibration complete.");
}

void calibrateDirectFeedthrough(){
  Serial.println("Calibrating feedthrough...");
  feedthroughSlopeXPos = calibrateChannel(MD2_IN1,MD2_IN2,CURRENT_X_POS,meanMagFieldX,true,'X');
  feedthroughSlopeXNeg = calibrateChannel(MD3_IN1,MD3_IN2,CURRENT_X_NEG,meanMagFieldX,false,'X');
  feedthroughSlopeYPos = calibrateChannel(MD4_IN1,MD4_IN2,CURRENT_Y_POS,meanMagFieldY,true,'Y');
  feedthroughSlopeYNeg = calibrateChannel(MD1_IN1,MD1_IN2,CURRENT_Y_NEG,meanMagFieldY,false,'Y');
  float slopeZ1 = calibrateChannel(MD2_IN1,MD2_IN2,CURRENT_X_POS,meanMagFieldZ,false,'Z');
  float slopeZ2 = calibrateChannel(MD3_IN1,MD3_IN2,CURRENT_X_NEG,meanMagFieldZ,false,'Z');
  float slopeZ3 = calibrateChannel(MD4_IN1,MD4_IN2,CURRENT_Y_POS,meanMagFieldZ,false,'Z');
  float slopeZ4 = calibrateChannel(MD1_IN1,MD1_IN2,CURRENT_Y_NEG,meanMagFieldZ,false,'Z');
  feedthroughSlopeZ = (slopeZ1+slopeZ2+slopeZ3+slopeZ4)/4;
  Serial.println("Feedthrough calibration complete.");
}

float calibrateChannel(uint8_t in1, uint8_t in2, int currentChannel, float meanValue, bool invertSlope, char axis){
  const int segTime = 150;
  const int settleDelay = 50;
  const int numLevels = 3;
  float currentLevels[numLevels] = {50.0,100.0,150.0};
  float totalSlope = 0;
  
  for(int i=0;i<numLevels;i++){
    float appliedCurrent = currentLevels[i];
    setSolenoidInput(appliedCurrent,in1,in2);
    delay(settleDelay);
    float sumSensor = 0, sumCurrent = 0;
    int count = 0;
    unsigned long startTime = millis();
    while(millis()-startTime < segTime){
      Sensor.updateData();
      float sensorVal = (axis=='X' ? Sensor.getX() : (axis=='Y' ? Sensor.getY() : Sensor.getZ()));
      float currentMeasured = getSolenoidCurrent(currentChannel);
      sumSensor += sensorVal;
      sumCurrent += currentMeasured;
      count++;
      delay(10);
    }
    setSolenoidInput(0,in1,in2);
    float avgSensor = sumSensor/count;
    float avgCurrent = sumCurrent/count;
    float measuredSlope = (avgSensor-meanValue)/avgCurrent;
    if(invertSlope) measuredSlope = -measuredSlope;
    totalSlope += measuredSlope;
    delay(50);
  }
  return totalSlope/numLevels;
}

void logSystemState(){
  Serial.print("realSamplingFreq:");
  Serial.print(realSamplingFreq,2);
  Serial.print(", currentYPos:");
  Serial.print(currentYPos,2);
  Serial.print(", MagFieldX:");
  Serial.print(magFieldX,2);
  Serial.print(", MagFieldY:");
  Serial.print(magFieldY,2);
  Serial.print(", MagFieldZ:");
  Serial.print(magFieldZ,2);
  Serial.print(", RawMagFieldX:");
  Serial.print(rawMagFieldX,2);
  Serial.print(", RawMagFieldY:");
  Serial.println(rawMagFieldY,2);
}

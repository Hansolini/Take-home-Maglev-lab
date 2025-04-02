
#include "functions.h"

/*
void setup() {
    Serial.begin(115200);
    while (!Serial) ;  // Wait for Serial to open
    delay(1000);       // Give the OS some time to recognize the USB device
}

void loop() {
    Serial.write("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa");
}



*/


// === Functions prototypes ===
void processCommand(String);

// === Sensor and Multiplexer Initialization ===
// Example sensor initialization (choose your sensor class as needed)
// TLx493D_A1B6 Sensor(Wire, TLx493D_IIC_ADDR_A0_e);
Tlv493d Sensor = Tlv493d();
TCA9548 mux_sensors(0x70);

// Feedthrough slopes for X and Y (default values, tuned automatically)
float feedthroughSlopeXPos = 0.5;
float feedthroughSlopeXNeg = 0.5;
float feedthroughSlopeYPos = 0.5;
float feedthroughSlopeYNeg = 0.5;

// filters values for sampled sensors values
constexpr float ALPHA = 0.20;
constexpr float DALPHA = 0.05;

// Timing variables for sensors values sampling and for freezing
unsigned long previousTime = 0;
unsigned long currentTime;
int interval = round(1e6 / 5000); // 5kHz

// Variables for sensors sampling
float magFieldX = 0, magFieldY = 0, magFieldZ = 0;
float dMagFieldX = 0, dMagFieldY = 0, dMagFieldZ = 0;
float rawMagFieldX = 0, rawMagFieldY = 0, rawMagFieldZ = 0;
float rawMagFieldXDetrended = 0, rawMagFieldYDetrended = 0, rawMagFieldZDetrended = 0;
float prevMagFieldX = 0, prevMagFieldY = 0, prevMagFieldZ = 0;
float currentXPos = 0, currentXNeg = 0, currentYPos = 0, currentYNeg = 0;
float meanMagFieldX = 0, meanMagFieldY = 0, meanMagFieldZ = 0;

// Variables for sensor freeze detection
float lastMagFieldX = 0, lastMagFieldY = 0, lastMagFieldZ = 0;
int freezeCounter = 0;
const int freezeLimit = 20;


/*
 * SETUP
 */
void setup() {

  pinMode(LED_BUILTIN, OUTPUT);
  
  initializeSerial();
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


/*
 * LOOP
 */
void loop() {

  // keep the builtin led on when serial connection is up
  (Serial) ? digitalWrite(LED_BUILTIN, HIGH) : digitalWrite(LED_BUILTIN, LOW);

  currentTime = micros();

  // sample and filter sensors values and check if magnet attached on the board to reset solenoids
  if(currentTime - previousTime >= (unsigned long)interval) {
    unsigned long dt = currentTime - previousTime;
    sensors_sampling(dt);
    checkSensorFreeze();
    previousTime = currentTime;
  }

  if (Serial.available()) {
    uint8_t buffer[64];
    int length = Serial.readBytesUntil(0x00, buffer, sizeof(buffer));
    if(length > 0)  processCommand(buffer, length);
  }
}

// process incoming command
void processCommand(const uint8_t *raw_command, int length) {

  uint8_t command[64];
  cobsDecode(raw_command, length, command);

  if (command[0] == 0x01) {
    sendSensorValues();
  } else if (command[0] == 0x02) {
    sendCurrentValues();
  } else if (command[0] == 0x03) {
    uint8_t *currents_values = command + 1;
    setSolenoidCurrents(currents_values);
  } else if (command[0] == 0x04) {
    resetSolenoids();
  } else if (command[0] == 0x05) {
    sendStatus();
  } else if (command[0] == 0xaa) {
    Serial.write(0xee);
  } else {
    sendError(103);
  }
}

void calibrateSensor(){
  // Serial.println("Calibrating sensor...");
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
  // Serial.println("Sensor calibration complete.");
}

void calibrateDirectFeedthrough(){
  // Serial.println("Calibrating feedthrough...");
  feedthroughSlopeXPos = calibrateChannel(MD2_IN1, MD2_IN2, CURRENT_X_POS, meanMagFieldX, true, 'X');
  feedthroughSlopeXNeg = calibrateChannel(MD3_IN1, MD3_IN2, CURRENT_X_NEG, meanMagFieldX, false, 'X');
  feedthroughSlopeYPos = calibrateChannel(MD4_IN1, MD4_IN2, CURRENT_Y_POS, meanMagFieldY, true, 'Y');
  feedthroughSlopeYNeg = calibrateChannel(MD1_IN1, MD1_IN2, CURRENT_Y_NEG, meanMagFieldY, false, 'Y');
  
  // Serial.println("Feedthrough calibration complete.");
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

void checkSensorFreeze(){

  Sensor.updateData();

  if(Sensor.getX() == lastMagFieldX && Sensor.getY() == lastMagFieldY && Sensor.getZ() == lastMagFieldZ){
    freezeCounter++;
  } else {
    freezeCounter = 0;
  }

  if(freezeCounter >= freezeLimit){
    // Serial.println("Sensor freeze detected! Resetting...");
    resetSensorAndI2C();
    freezeCounter = 0;
  }

  Sensor.updateData();

  lastMagFieldX = Sensor.getX();
  lastMagFieldY = Sensor.getY();
  lastMagFieldZ = Sensor.getZ();
}

void resetSensorAndI2C(){
  // Serial.println("Performing sensor & bus reset...");

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

  // Serial.println("Sensor reset completed.");
}


void sensors_sampling(unsigned long dt) {

  // if(dt > 0) realSamplingFreq = 1e6 / (float)dt;

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

}






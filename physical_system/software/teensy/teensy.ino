
#include "functions.h"


// === Functions prototypes ===
void processCommand(String);

// === Sensor and Multiplexer Initialization ===
// Example sensor initialization (choose your sensor class as needed)
// TLx493D_A1B6 Sensor(Wire, TLx493D_IIC_ADDR_A0_e);
Tlv493d Sensor = Tlv493d();
TCA9548 mux_sensors(0x70);

// timing variables for freeze detection
unsigned long previousTime = 0;
unsigned long currentTime;
int interval = round(1e6 / 5000); // 5kHz

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

}


/*
 * LOOP
 */
void loop() {

  currentTime = micros();

  if(currentTime - previousTime >= (unsigned long)interval) {
    checkSensorFreeze();
    previousTime = currentTime;
  }

  // keep the builtin led on when serial connection is up
  (Serial) ? digitalWrite(LED_BUILTIN, HIGH) : digitalWrite(LED_BUILTIN, LOW);

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
  } else {
    sendError(103);
  }
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







#include "functions.h"

// extern TLx493D_A1B6 Sensor;
extern Tlv493d Sensor;
extern TCA9548 mux_sensors;

// Solenoid-related functions
float getSolenoidCurrent(uint16_t pin) {
  uint16_t data = analogRead(pin);
  float voltage = (data * 3.3) / 1023;
  float voltage_diff = voltage - 1.65;
  float current = (voltage_diff / 0.5);  // GAIN = 50, Rshunt = 10mOhms
  return current;
}

// applies PWM to control the solenoids  (This sets solenoids using slow decay mode: https://forum.allaboutcircuits.com/threads/strange-full-h-bridge-problem.161122/)
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

// reset solenoids to default values (pwm: 0)
void resetSolenoids() {
  // X-direction control
  setSolenoidInput(0, MD2_IN1, MD2_IN2);
  setSolenoidInput(0, MD3_IN1, MD3_IN2);

  // Y-direction control
  setSolenoidInput(0, MD4_IN1, MD4_IN2);
  setSolenoidInput(0, MD1_IN1, MD1_IN2);

  StaticJsonDocument<64> doc;
  doc["response"] = "ACK_RESET_CURRENTS";
  doc["status"] = "OK";
  serializeJson(doc, Serial);
  Serial.println();
}

// Setup-related functions
void initializeSerial() {
  Serial.begin(BAUDRATE);
}

void initializeSensor() {
  // Initialize sensor
  // Sensor.begin();
  // if (!Sensor.isFunctional()) {
  //   Serial.println("Sensor is not functional!");
  // }
  // Sensor.softwareReset();
  // Sensor.setDefaultConfig();
  // Sensor.setMeasurement(TLx493D_BxByBz_e);
  // Sensor.setPowerMode(TLx493D_MASTER_CONTROLLED_MODE_e);

  Sensor.begin();
  Sensor.setAccessMode(Sensor.MASTERCONTROLLEDMODE);
  Sensor.disableTemp();

  // Setting I2C clock speed
  Wire.begin();
  Wire.setClock(I2C_CLOCK_SPEED);  // 1 MHz I2C speed
  delay(10);

  // Initialize sensor MUX
  mux_sensors.setResetPin(RESET_I2C);
  mux_sensors.reset();
  if (!mux_sensors.begin()) {
    Serial.println("COULD NOT CONNECT");
  }
  mux_sensors.enableChannel(MUX_CHANNEL);
  mux_sensors.selectChannel(MUX_CHANNEL);
  delay(20);
  
  // Reconfirm I2C speed
  Wire.setClock(I2C_CLOCK_SPEED);
  delay(10);
 }

void initializeSolenoids() {
  // Defining bit-size on read/write operations
  analogWriteResolution(PWM_BIT_SIZE_W);
  analogReadResolution(PWM_BIT_SIZE_R);

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

  // Defining PWM frequency
  analogWriteFrequency(MD1_IN1, PWM_FREQUENCY);
  analogWriteFrequency(MD1_IN2, PWM_FREQUENCY);
  analogWriteFrequency(MD2_IN1, PWM_FREQUENCY);
  analogWriteFrequency(MD2_IN2, PWM_FREQUENCY);
  analogWriteFrequency(MD3_IN1, PWM_FREQUENCY);
  analogWriteFrequency(MD3_IN2, PWM_FREQUENCY);
  analogWriteFrequency(MD4_IN1, PWM_FREQUENCY);
  analogWriteFrequency(MD4_IN2, PWM_FREQUENCY);

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


// send sensor values as JSON
void sendSensorValues() {
  StaticJsonDocument<128> doc;
  doc["response"] = "SENSOR_VALUES";

  JsonArray values = doc.createNestedArray("values");
  JsonObject sensor = values.createNestedObject();

  Sensor.updateData(); // Update sensor reading

  sensor["x"] = Sensor.getX();
  sensor["y"] = Sensor.getY();
  sensor["z"] = Sensor.getZ();

  serializeJson(doc, Serial);
  Serial.println();
}

// send system status
void sendStatus() {
  StaticJsonDocument<128> doc;
  doc["response"] = "SYSTEM_STATUS";
  doc["solenoids"] = 4;
  doc["sensors"] = 9;
  doc["uptime"] = millis() / 1000;

  serializeJson(doc, Serial);
  Serial.println();
}

// send an error response
void sendError(int code, const char* message) {
  StaticJsonDocument<128> doc;
  doc["response"] = "ERROR";
  doc["code"] = code;
  doc["message"] = message;

  serializeJson(doc, Serial);
  Serial.println();
}

// set solenoid current values
void setSolenoidCurrents(JsonArray solenoids) {
  for (JsonObject solenoid : solenoids) {
    int pin1 = solenoid["pin1"];
    int pin2 = solenoid["pin2"];
    int current = solenoid["current"];
    
    /* handle differently (maybe I can reuse IDs and define somewhere tuples associated to those)
    if (id < 0 || id > NUM_MOTOR_PINS - 1) {
      sendError(104, "Invalid solenoid ID");
      return;
    }
    */
    
    setSolenoidInput(current, pin1, pin2);
  }

  StaticJsonDocument<64> doc;
  doc["response"] = "ACK_SET_CURRENTS";
  doc["status"] = "OK";
  serializeJson(doc, Serial);
  Serial.println();
}

// read solenoids currents and send them through serial
void sendCurrentValues() {
  StaticJsonDocument<256> doc;
  doc["response"] = "CURRENT_VALUES";

  JsonArray values = doc.createNestedArray("values");
  JsonObject solenoid = values.createNestedObject();

  solenoid["X_POS"] = getSolenoidCurrent(CURRENT_X_POS);
  solenoid["X_NEG"] = getSolenoidCurrent(CURRENT_X_NEG);
  solenoid["Y_POS"] = getSolenoidCurrent(CURRENT_Y_POS);
  solenoid["Y_NEG"] = getSolenoidCurrent(CURRENT_Y_NEG);

  serializeJson(doc, Serial);
  Serial.println();
}




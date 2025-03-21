
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

// send sensor values
void sendSensorValues() {
  
  uint8_t data[13] = {0x00};

  data[0] = 0x01;

  Sensor.updateData(); // Update sensor reading

  *(float*)(data+1) = Sensor.getX();
  *(float*)(data+5) = Sensor.getY();
  *(float*)(data+9) = Sensor.getZ();

  uint8_t enc_data[sizeof(data) + 2] = {0x00};
  cobsEncode(data, sizeof(data), enc_data);

  /*uint8_t data[] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  float a = 1.0;
  float b = 2.0;
  float c = 3.0;
  *(float*)(data+1) = a;
  *(float*)(data+5) = b;
  *(float*)(data+9) = c;
  uint8_t enc_data[sizeof(data) + 2] = {0x00};
  cobsEncode(data, sizeof(data), enc_data);*/

  sendUntilNull(enc_data);
}

// reset solenoids to default values (pwm: 0)
void resetSolenoids() {
  // X-direction control
  setSolenoidInput(0, MD2_IN1, MD2_IN2);
  setSolenoidInput(0, MD3_IN1, MD3_IN2);

  // Y-direction control
  setSolenoidInput(0, MD4_IN1, MD4_IN2);
  setSolenoidInput(0, MD1_IN1, MD1_IN2);


  uint8_t data[1] = {0x00};

  data[0] = 0x04;

  uint8_t enc_data[sizeof(data) + 2] = {0x00};

  cobsEncode(data, sizeof(data), enc_data);

  sendUntilNull(enc_data);
}

// Setup-related functions
void initializeSerial() {
  Serial.begin(BAUDRATE);
}

// Robust initialization with bus clearing and retries
void initializeSensor() {
  // Explicitly reset mux and select correct channel
  mux_sensors.reset();
  delay(10);
  mux_sensors.enableChannel(7);
  mux_sensors.selectChannel(7);
  delay(20);

  // Explicitly initialize sensor with retries
  const int maxRetries = 5;
  bool initialized = false;
  for(int retry = 0; retry < maxRetries && !initialized; retry++){
    Sensor.begin();
    delay(20);
    Sensor.setAccessMode(Sensor.MASTERCONTROLLEDMODE);
    Sensor.disableTemp();
    delay(20);

    initialized = sensorInitializedCorrectly();
    if (!initialized) {
      // Serial.print("Retrying sensor initialization... attempt ");
      // Serial.println(retry + 1);
    }
  }

  // NOW explicitly override Wire bus again to guarantee high-speed communication
  Wire.begin();
  Wire.setClock(1000000);
  delay(20);

  if(!initialized){
    // Serial.println("Sensor initialization FAILED after retries!");
  } else {
    // Serial.println("Sensor initialization SUCCESSFUL.");
  }
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

bool sensorInitializedCorrectly(){
  Sensor.updateData();
  float checkX = Sensor.getX();
  float checkY = Sensor.getY();
  float checkZ = Sensor.getZ();

  return (abs(checkX) > 0.00001 || abs(checkY) > 0.00001 || abs(checkZ) > 0.00001);
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

// send system status (maybe other information useful to send (?) )
void sendStatus() {

  uint8_t data[5] = {0x00};

  data[0] = 0x05;

  data[1] = 0x04;
  data[2] = 0x09;
  *(uint16_t*)(data+3) = millis() / 1000;

  uint8_t enc_data[sizeof(data) + 2] = {0x00};  // 2 is the maximum COBS overhead
  cobsEncode(data, sizeof(data), enc_data);

  sendUntilNull(enc_data);
}

// send an error response
void sendError(uint8_t code) {

  uint8_t data[2] = {0x00};

  data[0] = 0x00;
  data[1] = code;

  uint8_t enc_data[sizeof(data) + 2] = {0x00};  // 2 is the maximum COBS overhead
  cobsEncode(data, sizeof(data), enc_data);

  sendUntilNull(enc_data);

}

// set solenoid current values
void setSolenoidCurrents(uint8_t *currents_values) {

  uint8_t current_X_POS = currents_values[0];
  uint8_t current_Y_NEG = currents_values[1];
  uint8_t current_X_NEG = currents_values[2];
  uint8_t current_Y_POS = currents_values[3];

  int x_p = current_X_POS - current_X_NEG;
  int x_n = - x_p;
  int y_p = current_Y_POS - current_Y_NEG;
  int y_n = - y_p;

  setSolenoidInput(x_p, 2, 3);
  setSolenoidInput(y_n, 4, 5);
  setSolenoidInput(x_n, 6, 7);
  setSolenoidInput(y_p, 8, 9);

  /*uint8_t data[] = {0x03, 0x00, 0x00, 0x00, 0x00};
  uint8_t a = currents_values[0];
  uint8_t b = currents_values[1];
  uint8_t c = currents_values[2];
  uint8_t d = currents_values[3];
  data[1] = a;
  data[2] = b;
  data[3] = c;
  data[4] = d;
  uint8_t enc_data[sizeof(data) + 2] = {0x00};
  cobsEncode(data, sizeof(data), enc_data);*/

  uint8_t data[1] = {0x00};

  data[0] = 0x03;

  uint8_t enc_data[sizeof(data) + 2] = {0x00};

  cobsEncode(data, sizeof(data), enc_data);

  sendUntilNull(enc_data);

}

// read solenoids currents and send them through serial
void sendCurrentValues() {

  uint8_t data[17] = {0x00};

  data[0] = 0x02;

  *(float*)(data+1) = getSolenoidCurrent(CURRENT_X_POS);
  *(float*)(data+5) = getSolenoidCurrent(CURRENT_Y_NEG);
  *(float*)(data+9) = getSolenoidCurrent(CURRENT_X_NEG);
  *(float*)(data+13) = getSolenoidCurrent(CURRENT_Y_POS);

  uint8_t enc_data[sizeof(data) + 2] = {0x00};  // 2 is the maximum COBS overhead
  cobsEncode(data, sizeof(data), enc_data);

  /*uint8_t data[] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  float a = 1.0;
  float b = 2.0;
  float c = 3.0;
  float d = 4.0;
  *(float*)(data+1) = a;
  *(float*)(data+5) = b;
  *(float*)(data+9) = c;
  *(float*)(data+13) = d;
  uint8_t enc_data[sizeof(data) + 2] = {0x00};
  cobsEncode(data, sizeof(data), enc_data);*/

  sendUntilNull(enc_data);

}

// customized function for sending Serial data until NULL byte (for dealing with COBS encoding)
void sendUntilNull(uint8_t *buffer) {
    while (*buffer != 0x00) {
        Serial.write(*buffer);
        buffer++;
    }
    Serial.write(0x00);  // Send the null terminator
}



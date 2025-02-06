#include "functions.h"

// extern TLx493D_A1B6 Sensor;
// extern Tlv493d Sensor;
extern Tle493d_a2b6 Sensor;
extern TCA9548 mux_sensors;

// Solenoid-related functions
float getSolenoidCurrent(uint16_t pin) {
  uint16_t data = analogRead(pin);
  float voltage = (data * 3.3) / 1023;
  float voltage_diff = voltage - 1.65;
  float current = (voltage_diff / 0.5);  // GAIN = 50, Rshunt = 10mOhms
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

// Setup-related functions
void initializeSerial() {
  Serial.begin(115200);
}

// void initializeSensor() {
//   // Setting I2C clock speed
//   Wire.begin();
//   Wire.setClock(1000000);  // 1 MHz I2C speed
//   delay(10);

//   // Initialize sensor MUX
//   mux_sensors.setResetPin(RESET_I2C);
//   mux_sensors.reset();
//   if (!mux_sensors.begin()) {
//     Serial.println("COULD NOT CONNECT");
//   }
//   mux_sensors.enableChannel(7);
//   mux_sensors.selectChannel(7);
//   delay(20);
  
//   // Reconfirm I2C speed
//   Wire.setClock(1000000);
//   delay(10);

//   // Initialize sensor
//   // Sensor.begin();
//   // if (!Sensor.isFunctional()) {
//   //   Serial.println("Sensor is not functional!");
//   // }
//   // Sensor.softwareReset();
//   // Sensor.setDefaultConfig();
//   // Sensor.setMeasurement(TLx493D_BxByBz_e);
//   // Sensor.setPowerMode(TLx493D_MASTER_CONTROLLED_MODE_e);

//   Sensor.begin();
//   // Sensor.setAccessMode(Sensor.MASTERCONTROLLEDMODE);
//   // Sensor.disableTemp();
//  }

void initializeSensor() {

    Sensor.begin();
    Sensor.setAccessMode(Sensor.FASTMODE); // IMPORTANT! THE SENSOR HAS TO BE INITIALIZED BEFORE WIRE (or some of the other things below), FOR SOME REASON! ELSE, THE SAMPLING FREQUENCY WILL BE CAPPED!


    Wire.begin();
    Wire.setClock(1000000);  // Set I²C speed once
    delay(10);

    mux_sensors.setResetPin(RESET_I2C);
    mux_sensors.reset();
    if (!mux_sensors.begin()) {
        Serial.println("COULD NOT CONNECT");
    }
    mux_sensors.enableChannel(7);  // Enable once during setup
    delay(10);

    // Sensor.disableTemp();
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

  // Defining PWM frequency
  analogWriteFrequency(MD1_IN1, 32258);
  analogWriteFrequency(MD1_IN2, 32258);
  analogWriteFrequency(MD2_IN1, 32258);
  analogWriteFrequency(MD2_IN2, 32258);
  analogWriteFrequency(MD3_IN1, 32258);
  analogWriteFrequency(MD3_IN2, 32258);
  analogWriteFrequency(MD4_IN1, 32258);
  analogWriteFrequency(MD4_IN2, 32258);

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
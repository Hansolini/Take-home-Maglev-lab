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
      Serial.print("Retrying sensor initialization... attempt ");
      Serial.println(retry + 1);
    }
  }


  // NOW explicitly override Wire bus again to guarantee high-speed communication
  Wire.begin();
  Wire.setClock(1000000);
  delay(20);

  if(!initialized){
    Serial.println("Sensor initialization FAILED after retries!");
  } else {
    Serial.println("Sensor initialization SUCCESSFUL.");
  }
}

bool sensorInitializedCorrectly(){
  Sensor.updateData();
  float checkX = Sensor.getX();
  float checkY = Sensor.getY();
  float checkZ = Sensor.getZ();

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

  // Defining PWM frequency
  analogWriteFrequency(MD1_IN1, 52258);
  analogWriteFrequency(MD1_IN2, 52258);
  analogWriteFrequency(MD2_IN1, 52258);
  analogWriteFrequency(MD2_IN2, 52258);
  analogWriteFrequency(MD3_IN1, 52258);
  analogWriteFrequency(MD3_IN2, 52258);
  analogWriteFrequency(MD4_IN1, 52258);
  analogWriteFrequency(MD4_IN2, 52258);

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
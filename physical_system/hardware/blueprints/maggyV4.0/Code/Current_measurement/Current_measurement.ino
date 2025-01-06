#include <Tlv493d.h>
#include <TCA9548.h>
#include <Wire.h>

// Sampling frequency and timer initialization
const double f_s = 5000;  // Hz sampling rate
double prev_time = 1;
double current_time = 2;
const int T = round(1e6 / f_s);  // Timer interval in microseconds

double bx = 0, by = 0, bz = 0;
int direction = 1;

// Sensor initialization
Tlv493d Sensor = Tlv493d();

// Multiplexor I2C initialization
TCA9548 mux_sensors(0x70);
uint8_t channels = 7;

// Motor driver pins setup
#define MD1_IN1 4
#define MD1_IN2 5

#define MD2_IN1 2
#define MD2_IN2 3

#define MD3_IN1 6
#define MD3_IN2 7

#define MD4_IN1 8
#define MD4_IN2 9

#define CURRENT_Y_POS 20
#define CURRENT_X_NEG 21
#define CURRENT_X_POS 22
#define CURRENT_Y_NEG 23

#define RESET_I2C 26

// Counters for initialization and loop iteration tracking
int initCounter = 0;
int loopCounter = 0;

// Control variables
double ux = 0, uy = 0;    // Control inputs

void setup() {
  Serial.begin(115200);
  Serial.println("Initialization...");

  // Set I2C communication frequency to 1MHz for sensor
  Wire.begin();
  Wire.setClock(1000000);

  // Reset Multiplexor, to avoid issue durint the launch of the board
  mux_sensors.setResetPin(RESET_I2C);
  mux_sensors.reset();
  if (mux_sensors.begin() == false)
  {
    Serial.println("COULD NOT CONNECT");
  }
  mux_sensors.enableChannel(7);
  mux_sensors.selectChannel(7);

  // Sensors initialisation
  Sensor.begin();
  Sensor.setAccessMode(Sensor.FASTMODE);
  Sensor.disableTemp();

  Wire.begin();
  Wire.setClock(1000000);

  // Set up motor drivers for PWM control with 8-bit resolution
  analogWriteResolution(8);
  analogReadResolution(10);

  // Set motor driver pins as output
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

  // Set PWM frequency for motor drivers to avoid audible noise
  analogWriteFrequency(MD1_IN1, 32258);
  analogWriteFrequency(MD1_IN2, 32258);
  analogWriteFrequency(MD2_IN1, 32258);
  analogWriteFrequency(MD2_IN2, 32258);
  analogWriteFrequency(MD3_IN1, 32258);
  analogWriteFrequency(MD3_IN2, 32258);
  analogWriteFrequency(MD4_IN1, 32258);
  analogWriteFrequency(MD4_IN2, 32258);

  // Start with motors off
  digitalWrite(MD1_IN1, LOW);
  digitalWrite(MD1_IN2, LOW);
  digitalWrite(MD2_IN1, LOW);
  digitalWrite(MD2_IN2, LOW);
  digitalWrite(MD3_IN1, LOW);
  digitalWrite(MD3_IN2, LOW);
  digitalWrite(MD4_IN1, LOW);
  digitalWrite(MD4_IN2, LOW);

}

// Maps control input 'u' to PWM signal
int map_from_u_to_pwm(int u) {
  return u;
  double a = (165.0 - 135.0) / 255.0;  // Slope
  double b = 135.0;                    // Offset

  if (u > 0) {
    return round(a * u + b);
  } else {
    return round(a * u - b);
  }
}
float convert_current(uint16_t a){
  // ADC conversion
  float volt_in = (a * 3.3)/1023;
  // Bidirectionnal, VREF = 1.65V
  float volt_diff = volt_in-1.65;
  // Calcul current, GAIN = 50 and Rshunt 10mOhms
  float cur = volt_diff/(50*0.010)*1000;
  return volt_in;
}
// Changes motor inputs based on control signals ux and uy
void change_input(int ux, int uy) {

  int ux_pwm = map_from_u_to_pwm(ux);
  int uy_pwm = map_from_u_to_pwm(uy);

  // Set PWM values for x-direction control
  if (ux > 0) {
    analogWrite(MD3_IN1, ux_pwm);
    analogWrite(MD3_IN2, 0);
    analogWrite(MD2_IN1, 0);
    analogWrite(MD2_IN2, ux_pwm);
  } else {
    analogWrite(MD3_IN1, 0);
    analogWrite(MD3_IN2, -ux_pwm);
    analogWrite(MD2_IN1, -ux_pwm);
    analogWrite(MD2_IN2, 0);
  }

  // Set PWM values for y-direction control
  if (uy > 0) {
    analogWrite(MD1_IN1, uy_pwm);
    analogWrite(MD1_IN2, 0);
    analogWrite(MD4_IN1, 0);
    analogWrite(MD4_IN2, uy_pwm);
  } else {
    analogWrite(MD1_IN1, 0);
    analogWrite(MD1_IN2, -uy_pwm);
    analogWrite(MD4_IN1, -uy_pwm);
    analogWrite(MD4_IN2, 0);
  }
}

void loop() {

  // Run control loop based on sampling time T
  if (micros() - prev_time >= T) {
    current_time = micros();
    uint16_t current_x_pos = analogRead(CURRENT_X_POS);
    uint16_t current_x_neg = analogRead(CURRENT_X_NEG);
    uint16_t current_y_pos = analogRead(CURRENT_Y_POS);
    uint16_t current_y_neg = analogRead(CURRENT_Y_NEG);

    float cur_x_pos = convert_current(current_x_pos);
    // Update sensor data
    // PCA9548A(Wire, 7);  // Select sensor channel
    // mux_sensors.selectChannel(7);
    // Sensor.updateData();
    // bx = Sensor.getX();
    // by = Sensor.getY();
    // bz = Sensor.getZ();

    if (ux >= 180) {
      direction = -1;  // Change de direction pour décrémenter
    } 
    else if (ux <= -180) {
      direction = 1;   // Change de direction pour incrémenter
    }

    // ux += direction;
    ux = 0;
    uy = 0;

    // Apply control signals to motors
    change_input(ux, uy);

    // Log data every 100 loops
    if (loopCounter % 100 == 0) {
      Serial.print("Bx:");
      Serial.print(bx);
      Serial.print(',');
      // Serial.print("By:");
      // Serial.print(by);
      // Serial.print(',');
      // Serial.print("Bz:");
      // Serial.print(bz);
      // Serial.print(',');
      Serial.print("X+:");
      Serial.print(current_x_pos);
      Serial.print(',');
      // Serial.print("X-:");
      // Serial.print(current_x_neg);
      // Serial.print(',');
      // Serial.print("Y+:");
      // Serial.print(current_y_pos);
      // Serial.print(',');
      // Serial.print("Y-:");
      // Serial.print(current_y_neg);
      // Serial.print(',');
      Serial.print("ux:");
      Serial.print(ux);
      Serial.print(',');
      Serial.print("uy:");
      Serial.println(uy);
    }
    loopCounter++;

    // Update timer
    prev_time = current_time;
  }
}

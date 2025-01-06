#include <Tlv493d.h>
#include <TCA9548.h>
#include <Wire.h>

// ############# TO BE MODIFIED ###########
// Sensor calibration for direct feedthrough
const double DIRECT_FEEDTHROUGH_SLOPE_X_POSITIVE = 0.7;
const double DIRECT_FEEDTHROUGH_SLOPE_X_NEGATIVE = -0.7;
const double DIRECT_FEEDTHROUGH_SLOPE_Y_POSITIVE = 0.7;
const double DIRECT_FEEDTHROUGH_SLOPE_Y_NEGATIVE = -0.7;

// Control gains for proportional (Kp) and derivative (Kd) control
double Kp = 350;
double Kd = 2.4;
// ########################################

// IIR filter constants for filtering sensor readings and derivatives
#define ALPHA 0.15
#define DALPHA 0.02

// Sampling frequency and timer initialization
const double f_s = 5000;  // Hz sampling rate
double prev_time = 1;
double current_time = 2;
const int T = round(1e6 / f_s);  // Timer interval in microseconds


// Sensor initialization
Tlv493d Sensor = Tlv493d();

// Multiplexor I2C initialization
TCA9548 mux_sensors(0x70);
uint8_t channels = 7;

// Motor driver pins setup
//// Motor driver 1
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

// Magnetic field measurements
double rawBx = 0, rawBy = 0, rawBz = 0;
double bx = 0, by = 0, bz = 0;
double bx_prev = 0, by_prev = 0, bz_prev = 0;
double dbx = 0, dby = 0, dbz = 0;
double dbx_prev = 0, dby_prev = 0, dbz_prev = 0;

// Mean values for calibration offset
float meanBx = 0, meanBy = 0, meanBz = 0;

// Control variables
double ux = 0, uy = 0;    // Control inputs
double ex = 0, ey = 0;    // Position error
double dex = 0, dey = 0;  // Derivative of position error


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


  // mux_sensors.enableChannel(7);
  // pinMode(RESET_I2C, OUTPUT);
  // digitalWrite(RESET_I2C, LOW);  //  page 3 HIGH is normal operation
  // delay(1);
  // digitalWrite(RESET_I2C, HIGH);  //  page 3 HIGH is normal operation

  // Compute initial sensor mean for calibration
  while (initCounter < 1000) {
    delay(1);  // Short delay between sensor readings
    // PCA9548A(Wire, 7);  // Select sensor channel
    mux_sensors.selectChannel(7);
    Sensor.updateData();
    meanBx += Sensor.getX() / 1000;
    meanBy += Sensor.getY() / 1000;
    meanBz += Sensor.getZ() / 1000;
    initCounter++;
  }
}

// Maps control input 'u' to PWM signal
int map_from_u_to_pwm(int u) {
  double a = (165.0 - 135.0) / 255.0;  // Slope
  double b = 135.0;                    // Offset

  if (u > 0) {
    return round(a * u + b);
  } else {
    return round(a * u - b);
  }
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

    float current_x_pos = analogRead(CURRENT_X_POS);
    float current_x_neg = analogRead(CURRENT_X_NEG);
    float current_y_pos = analogRead(CURRENT_Y_POS);
    float current_y_neg = analogRead(CURRENT_Y_NEG);

    // Update sensor data
    // PCA9548A(Wire, 7);  // Select sensor channel
    mux_sensors.selectChannel(7);
    Sensor.updateData();
    rawBx = Sensor.getX() - meanBx - DIRECT_FEEDTHROUGH_SLOPE_X_POSITIVE / 255.0 * ux + DIRECT_FEEDTHROUGH_SLOPE_X_NEGATIVE / 255.0 * ux +0.20;
    rawBy = Sensor.getY() - meanBy - DIRECT_FEEDTHROUGH_SLOPE_Y_POSITIVE / 255.0 * uy + DIRECT_FEEDTHROUGH_SLOPE_Y_NEGATIVE / 255.0 * uy+0.6;
    rawBz = Sensor.getZ() - meanBz;

    // Filtered magnetic field values
    bx = ALPHA * rawBx + (1.0 - ALPHA) * bx_prev;
    by = ALPHA * rawBy + (1.0 - ALPHA) * by_prev;
    bz = ALPHA * rawBz + (1.0 - ALPHA) * bz_prev;

    // Calculate filtered derivatives of magnetic field
    dbx = DALPHA * ((bx - bx_prev) / ((current_time - prev_time + 1) / 1000000.0)) + (1.0 - DALPHA) * dbx_prev;
    dby = DALPHA * ((by - by_prev) / ((current_time - prev_time + 1) / 1000000.0)) + (1.0 - DALPHA) * dby_prev;

    // Update previous values
    bx_prev = bx;
    by_prev = by;
    bz_prev = bz;
    dbx_prev = dbx;
    dby_prev = dby;

    // Control law to determine error and control signal if bz exceeds threshold
    if (abs(bz) > 3) {
      ex = -bx;
      ey = -by;
      dex = -dbx;
      dey = -dby;

      // Calculate control signals with constraints
      ux = constrain(Kp * ex + Kd * dex, -255, 255);
      uy = constrain(Kp * ey + Kd * dey, -255, 255);

      // Apply control signals to motors
      change_input(ux, uy);
    } else {
      ux = 0;
      uy = 0;
      change_input(ux, uy);  // Set motors off if bz threshold is not met
    }

    // Log data every 100 loops
    if (loopCounter % 100 == 0) {
      // Serial.print("Bx:");
      // Serial.print(bx);
      // Serial.print(',');
      // Serial.print("By:");
      // Serial.print(by);
      // Serial.print(',');
      // Serial.print("Bz:");
      // Serial.print(bz);
      // Serial.print(',');
      Serial.print("X+:");
      Serial.print(current_x_pos);
      Serial.print(',');
      Serial.print("X-:");
      Serial.print(current_x_neg);
      Serial.print(',');
      Serial.print("Y+:");
      Serial.print(current_y_pos);
      Serial.print(',');
      Serial.print("Y-:");
      Serial.print(current_y_neg);
      Serial.print(',');
      // Serial.print("ux:");
      // Serial.print(ux);
      // Serial.print(',');
      Serial.print("uy:");
      Serial.println(uy);
    }
    loopCounter++;

    // Update timer
    prev_time = current_time;
  }
}

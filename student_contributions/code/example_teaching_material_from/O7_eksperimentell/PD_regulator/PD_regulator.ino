#include <Tle493d_a2b6.h>
#include <Wire.h>

// ############# TO BE MODIFIED ###########
// Sensor calibration for direct feedthrough
const double DIRECT_FEEDTHROUGH_SLOPE_X_POSITIVE = 0.68;
const double DIRECT_FEEDTHROUGH_SLOPE_X_NEGATIVE = -0.74;
const double DIRECT_FEEDTHROUGH_SLOPE_Y_POSITIVE = 0.75;
const double DIRECT_FEEDTHROUGH_SLOPE_Y_NEGATIVE = -0.70;

// Control gains for proportional (Kp) and derivative (Kd) control
double Kp = 330;
double Kd = 0.3;
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
Tle493d_a2b6 Sensor = Tle493d_a2b6(Tle493d::FASTMODE, Tle493d::TLE493D_A0);

// Motor driver pins setup
//// Motor driver 1
#define MD1_IN1 3
#define MD1_IN2 2

//// Motor driver 2
#define MD2_IN1 5
#define MD2_IN2 4

//// Motor driver 3
#define MD3_IN1 7
#define MD3_IN2 6

//// Motor driver 4
#define MD4_IN1 9
#define MD4_IN2 8

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

  // Initialize sensor
  Sensor.begin();

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

  // Set PWM frequency for motor drivers to avoid audible noise
  int pwm_frequency = 292968;
  analogWriteFrequency(MD1_IN1, pwm_frequency);
  analogWriteFrequency(MD1_IN2, pwm_frequency);
  analogWriteFrequency(MD2_IN1, pwm_frequency);
  analogWriteFrequency(MD2_IN2, pwm_frequency);
  analogWriteFrequency(MD3_IN1, pwm_frequency);
  analogWriteFrequency(MD3_IN2, pwm_frequency);
  analogWriteFrequency(MD4_IN1, pwm_frequency);
  analogWriteFrequency(MD4_IN2, pwm_frequency);

  // Start with motors off
  digitalWrite(MD1_IN1, LOW);
  digitalWrite(MD1_IN2, LOW);
  digitalWrite(MD2_IN1, LOW);
  digitalWrite(MD2_IN2, LOW);
  digitalWrite(MD3_IN1, LOW);
  digitalWrite(MD3_IN2, LOW);
  digitalWrite(MD4_IN1, LOW);
  digitalWrite(MD4_IN2, LOW);

  // Set I2C communication frequency to 1MHz for sensor
  Wire.begin();
  Wire.setClock(1000000);

  // Compute initial sensor mean for calibration
  while (initCounter < 2000) {
    delay(1);  // Short delay between sensor readings
    Sensor.updateData();
    meanBx += Sensor.getX() / 2000;
    meanBy += Sensor.getY() / 2000;
    meanBz += Sensor.getZ() / 2000;
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

    // Update sensor data
    Sensor.updateData();
    rawBx = Sensor.getX() - meanBx - DIRECT_FEEDTHROUGH_SLOPE_X_POSITIVE / 255.0 * ux + DIRECT_FEEDTHROUGH_SLOPE_X_NEGATIVE / 255.0 * ux;
    rawBy = Sensor.getY() - meanBy - DIRECT_FEEDTHROUGH_SLOPE_Y_POSITIVE / 255.0 * uy + DIRECT_FEEDTHROUGH_SLOPE_Y_NEGATIVE / 255.0 * uy;
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
      ux = constrain(Kp * ex + Kd * dex, -256, 256);
      uy = constrain(Kp * ey + Kd * dey, -256, 256);

      // Apply control signals to motors
      change_input(ux, uy);
    } else {
      ux = 0;
      uy = 0;
      change_input(ux, uy);  // Set motors off if bz threshold is not met
    }

    // Log data every 100 loops
    if (loopCounter % 100 == 0) {
      Serial.print("Bx:");
      Serial.print(bx);
      Serial.print(',');
      Serial.print("By:");
      Serial.print(by);
      Serial.print(',');
      Serial.print("Bz:");
      Serial.print(bz);
      Serial.print(',');
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

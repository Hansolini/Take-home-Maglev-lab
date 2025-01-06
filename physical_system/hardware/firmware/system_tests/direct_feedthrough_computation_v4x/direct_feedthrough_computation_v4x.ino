#include "TLx493D_inc.hpp"
#include <TCA9548.h>
#include <Wire.h>

// Sample timer
const double f_s = 5000; // Hz
const int T = round(1e6 / f_s);
unsigned long prev_time = 0;

// Use xensiv library namespace
using namespace ifx::tlx493d;

// Sensor initialization (use the same variant as in the first code)
TLx493D_A1B6 Sensor(Wire, TLx493D_IIC_ADDR_A0_e);

// Multiplexer initialization
TCA9548 mux_sensors(0x70);
#define RESET_I2C 26 // Define your reset pin number

// Motor driver pins setup
//// Motor driver 1
#define MD1_IN1 4
#define MD1_IN2 5

//// Motor driver 2
#define MD2_IN1 2
#define MD2_IN2 3

//// Motor driver 3
#define MD3_IN1 6
#define MD3_IN2 7

//// Motor driver 4
#define MD4_IN1 8
#define MD4_IN2 9

// Counters
int initCounter = 0;
unsigned long loopCounter = 0;

// Measurements
double bx = 0, by = 0, bz = 0;

// Mean of measurements
double meanBx = 0, meanBy = 0, meanBz = 0;

// Control variables for each solenoid
int u1 = 0, u2 = 0, u3 = 0, u4 = 0;

// Data collection for linear regression
const int data_size = 1000;
double u_data[data_size];
double B_data[data_size];
int data_index = 0;
int current_solenoid = 1;

void setup() {
  // Initialize serial
  Serial.begin(115200);
  Serial.println("Initialization...");

  // Set I2C frequency to 1MHz
  Wire.begin();
  Wire.setClock(1000000);

  // Reset Multiplexer to avoid issues during launch
  mux_sensors.setResetPin(RESET_I2C);
  mux_sensors.reset();
  if (mux_sensors.begin() == false) {
    Serial.println("COULD NOT CONNECT TO MULTIPLEXER");
  }

  // Enable and select the appropriate channel (same as first code)
  mux_sensors.enableChannel(7);
  mux_sensors.selectChannel(7);

  // Initialize sensor
  Sensor.begin();

  // Initialize motor drivers and set PWM resolution/frequency as in the first code
  analogWriteResolution(8);

  pinMode(MD1_IN1, OUTPUT);
  pinMode(MD1_IN2, OUTPUT);
  pinMode(MD2_IN1, OUTPUT);
  pinMode(MD2_IN2, OUTPUT);
  pinMode(MD3_IN1, OUTPUT);
  pinMode(MD3_IN2, OUTPUT);
  pinMode(MD4_IN1, OUTPUT);
  pinMode(MD4_IN2, OUTPUT);

  analogWriteFrequency(MD1_IN1, 32258);
  analogWriteFrequency(MD1_IN2, 32258);
  analogWriteFrequency(MD2_IN1, 32258);
  analogWriteFrequency(MD2_IN2, 32258);
  analogWriteFrequency(MD3_IN1, 32258);
  analogWriteFrequency(MD3_IN2, 32258);
  analogWriteFrequency(MD4_IN1, 32258);
  analogWriteFrequency(MD4_IN2, 32258);

  // Start with all solenoids off
  digitalWrite(MD1_IN1, LOW);
  digitalWrite(MD1_IN2, LOW);
  digitalWrite(MD2_IN1, LOW);
  digitalWrite(MD2_IN2, LOW);
  digitalWrite(MD3_IN1, LOW);
  digitalWrite(MD3_IN2, LOW);
  digitalWrite(MD4_IN1, LOW);
  digitalWrite(MD4_IN2, LOW);

  // Compute initial sensor mean for calibration (similar approach as first code)
  while (initCounter < 1000) {
    delay(1);  // Short delay between sensor readings
    double tempX = 0, tempY = 0, tempZ = 0;
    Sensor.getMagneticField(&tempX, &tempY, &tempZ);
    meanBx += tempX / 1000;
    meanBy += tempY / 1000;
    meanBz += tempZ / 1000;
    initCounter++;
  }
}

void linear_regression(double x[], double y[], int n, double &slope, double &intercept) {
  double sum_x = 0, sum_y = 0, sum_xy = 0, sum_xx = 0;
  for (int i = 0; i < n; i++) {
    sum_x += x[i];
    sum_y += y[i];
    sum_xy += x[i] * y[i];
    sum_xx += x[i] * x[i];
  }
  double denominator = n * sum_xx - sum_x * sum_x;
  if (denominator != 0) {
    slope = (n * sum_xy - sum_x * sum_y) / denominator;
    intercept = (sum_y - slope * sum_x) / n;
  } else {
    slope = 0;
    intercept = 0;
  }
}

int map_from_u_to_pwm(int u) {
  double a = (165.0 - 135.0)/255.0;
  double b = 135.0;

  if (u > 0) {
    return round(a*u + b); 
  } else {
    return round(a*u - b);
  }
}

void control_solenoid(int solenoid_num, int u) {
  int IN1, IN2;
  switch (solenoid_num) {
    case 1:
      IN1 = MD1_IN1;
      IN2 = MD1_IN2;
      break;
    case 2:
      IN1 = MD2_IN1;
      IN2 = MD2_IN2;
      break;
    case 3:
      IN1 = MD3_IN1;
      IN2 = MD3_IN2;
      break;
    case 4:
      IN1 = MD4_IN1;
      IN2 = MD4_IN2;
      break;
    default:
      return;
  }

  int pwm = map_from_u_to_pwm(u);
  if (u > 0) {
    analogWrite(IN1, 0);
    analogWrite(IN2, abs(pwm));
  } else {
    analogWrite(IN1, abs(pwm));
    analogWrite(IN2, 0);
  }
}

void loop() {
  unsigned long current_time = micros();
  if (current_time - prev_time >= T) {
    prev_time = current_time;

    // Get measurement (in mT), subtract mean offsets
    double tempX = 0, tempY = 0, tempZ = 0;
    if (!Sensor.getMagneticField(&tempX, &tempY, &tempZ)) {
      // If reading fails, print error
      Serial.println("ERROR : readRegisters failed !");
    }
    double rawBx = tempX - meanBx; 
    double rawBy = tempY - meanBy;
    double rawBz = tempZ - meanBz;

    bx = rawBx; 
    by = rawBy; 
    bz = rawBz;

    // Vary u from -255 to 255
    int u = loopCounter % 512;
    if (u <= 256) {
      u = -256 + u; // Increase from -255 to 0
    } else {
      u = u - 256; // Increase from 0 to 255
    }

    // Control current solenoid
    if (current_solenoid == 1) { // x positive
      control_solenoid(2, u);

      control_solenoid(1, 0);
      control_solenoid(3, 0);
      control_solenoid(4, 0);
    } else if (current_solenoid == 2) { // x negative
      control_solenoid(3, u);

      control_solenoid(1, 0);
      control_solenoid(2, 0);
      control_solenoid(4, 0);
    } else if (current_solenoid == 3) { // y positive
      control_solenoid(4, u);

      control_solenoid(1, 0);
      control_solenoid(2, 0);
      control_solenoid(3, 0);
    } else if (current_solenoid == 4) { // y negative
      control_solenoid(1, u);

      control_solenoid(2, 0);
      control_solenoid(3, 0);
      control_solenoid(4, 0);
    }

    // Record data for linear regression
    if (data_index < data_size) {
      u_data[data_index] = u;

      // Choose the magnetic field component affected by the current solenoid
      double b = 0;
      if (current_solenoid == 1 || current_solenoid == 2) {
        b = bx;
      } else {
        b = by;
      }

      B_data[data_index] = b;
      data_index++;

    } else {
      double slope, intercept;
      linear_regression(u_data, B_data, data_size, slope, intercept);

      Serial.print("Compensation Factor for Solenoid ");
      if (current_solenoid == 1) {
        Serial.print("x positive");
      } else if (current_solenoid == 2) {
        Serial.print("x negative");
      } else if (current_solenoid == 3) {
        Serial.print("y positive");
      } else if (current_solenoid == 4) {
        Serial.print("y negative");
      }

      Serial.print(": ");
      Serial.println(slope * 255);

      // Reset data index and move to next solenoid
      data_index = 0;
      current_solenoid++;
      if (current_solenoid > 4) {
        current_solenoid = 1; // Start over
        Serial.println("#############################################");
      }
    }

    loopCounter++;
  }
}

#include <Tle493d_a2b6.h>
#include <Wire.h>

// Sample timer
const double f_s = 5000; // Hz
const int T = round(1e6 / f_s);
unsigned long prev_time = 0;

// Sensors
Tle493d_a2b6 Sensor = Tle493d_a2b6(Tle493d::FASTMODE, Tle493d::TLE493D_A0);

// Motor driver pins
#define MD1_IN1 3
#define MD1_IN2 2
#define MD2_IN1 5
#define MD2_IN2 4
#define MD3_IN1 7
#define MD3_IN2 6
#define MD4_IN1 9
#define MD4_IN2 8

// Counters
int initCounter = 0;
unsigned long loopCounter = 0;

// Measurements
double bx = 0, by = 0, bz = 0;

// Mean of measurements
float meanBx = 0, meanBy = 0, meanBz = 0;

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

  // Initialize sensor
  Sensor.begin();

  // Initialize motor drivers
  analogWriteResolution(8);

  //// Set pin mode
  pinMode(MD1_IN1, OUTPUT);
  pinMode(MD1_IN2, OUTPUT);

  pinMode(MD2_IN1, OUTPUT);
  pinMode(MD2_IN2, OUTPUT);
  
  pinMode(MD3_IN1, OUTPUT);
  pinMode(MD3_IN2, OUTPUT);
  
  pinMode(MD4_IN1, OUTPUT);
  pinMode(MD4_IN2, OUTPUT);

  // Set PWM frequency to avoid audible tones
  analogWriteFrequency(MD1_IN1, 32258);
  analogWriteFrequency(MD1_IN2, 32258);

  analogWriteFrequency(MD2_IN1, 32258);
  analogWriteFrequency(MD2_IN2, 32258);

  analogWriteFrequency(MD3_IN1, 32258);
  analogWriteFrequency(MD3_IN2, 32258);

  analogWriteFrequency(MD4_IN1, 32258);
  analogWriteFrequency(MD4_IN2, 32258);

  //// Start with motor drivers off
  digitalWrite(MD1_IN1, LOW);
  digitalWrite(MD1_IN2, LOW);
  
  digitalWrite(MD2_IN1, LOW);
  digitalWrite(MD2_IN2, LOW);
  
  digitalWrite(MD3_IN1, LOW);
  digitalWrite(MD3_IN2, LOW);

  digitalWrite(MD4_IN1, LOW);
  digitalWrite(MD4_IN2, LOW);

  // Set I2C frequency to 1MHz
  Wire.begin();
  Wire.setClock(1000000); 

  // Compute sensor mean
  while (initCounter < 1000) {
    delay(1);
    Sensor.updateData();
    meanBx += Sensor.getX() / 1000.0;
    meanBy += Sensor.getY() / 1000.0;
    meanBz += Sensor.getZ() / 1000.0;
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

int map_from_u_to_pwm(int u){
  double a = (165.0 - 135.0)/255.0;
  double b = 135.0;

  if(u > 0){
    return round(a*u + b); 
  }
  else{
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
  if (u > 0){
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

    // Get measurement
    Sensor.updateData();
    bx = Sensor.getX() - meanBx;
    by = Sensor.getY() - meanBy;
    bz = Sensor.getZ() - meanBz;

    // Vary u from -255 to 255
    int u = loopCounter % 512;
    if (u <= 256) {
      u = -256 + u; // Increase from -255 to 0
    } else {
      u = u - 256; // Increase from 0 to 255
    }

    // Control current solenoid
    if (current_solenoid == 1){ // x positive
      control_solenoid(2, u);

      control_solenoid(1, 0);
      control_solenoid(3, 0);
      control_solenoid(4, 0);

    } else if (current_solenoid == 2){ // x negative
      control_solenoid(3, u);

      control_solenoid(1, 0);
      control_solenoid(2, 0);
      control_solenoid(4, 0);

    } else if (current_solenoid == 3){ // y positive
      control_solenoid(4, u);

      control_solenoid(1, 0);
      control_solenoid(2, 0);
      control_solenoid(3, 0);
    } else if (current_solenoid == 4){ // y negative
      control_solenoid(1, u);

      control_solenoid(2, 0);
      control_solenoid(3, 0);
      control_solenoid(4, 0);
    }

    // Record data
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

      if (current_solenoid == 1){
        Serial.print("x positive");
      } else if (current_solenoid == 2){
        Serial.print("x negative");
      } else if (current_solenoid == 3){
        Serial.print("y positive");
      } else if (current_solenoid == 4){
        Serial.print("y negative");
      }

      Serial.print(": ");
      Serial.println(slope*255);

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

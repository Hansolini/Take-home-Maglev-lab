#include <Tle493d_a2b6.h>
#include <Wire.h>

// Sample timer
const double f_s = 5000; // Hz
unsigned long prev_time = 0;
unsigned long start_time = 0;
unsigned long current_time = 0;
const int T = round(1e6 / f_s);

// Sensors
Tle493d_a2b6 Sensor = Tle493d_a2b6(Tle493d::FASTMODE, Tle493d::TLE493D_A0);

// Four motor drivers - IN1 and IN2 are PWM inputs. Polarity of the output is set based on which one of them is active.
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

// Counters
int initCounter = 0;
int loopCounter = 0;

// Measurements
double rawBx = 0, rawBy = 0, rawBz = 0;
double bx = 0, by = 0, bz = 0;
double dbx = 0, dby = 0, dbz = 0;

// Mean of measurements
float meanBx = 0, meanBy = 0, meanBz = 0;

// PID
double Kp = 400, Kd = 1.5; // Increased Kp and fine-tuned Kd
double ux = 0, uy = 0;
double ex = 0, ey = 0;
double dex = 0, dey = 0;

// Initialize RLS filter parameters
const int RLS_ORDER = 2;
double lambda = 0.96;
double P_bx[2][2] = {{1, 0.0}, {0.0, 1}};
double P_by[2][2] = {{1, 0.0}, {0.0, 1}};
double P_bz[2][2] = {{1, 0.0}, {0.0, 1}};
double theta_bx[2] = {0, 0};
double theta_by[2] = {0, 0};
double theta_bz[2] = {0, 0};

void setup() {
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

  // We adjust the I2C clock speed to reach maximum sample time (5.2kHz). This HAS to be done after initializing the sensors!
  Wire.begin();
  Wire.setClock(1000000); // Set I2C frequency to 1MHz

  // Compute sensor mean
  while(initCounter < 1000) {
    delay(1);
    Sensor.updateData();
    meanBx += Sensor.getX()/1000;
    meanBy += Sensor.getY()/1000;
    meanBz += Sensor.getZ()/1000;
    
    initCounter++;
  }

  // Set the start time
  start_time = micros();
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

void change_input(int ux, int uy){
  int ux_pwm = map_from_u_to_pwm(ux);
  int uy_pwm = map_from_u_to_pwm(uy);

  if(ux > 0){
    analogWrite(MD3_IN1, ux_pwm);
    analogWrite(MD3_IN2, 0);

    analogWrite(MD2_IN1, 0);
    analogWrite(MD2_IN2, ux_pwm);
  }
  else{
    analogWrite(MD3_IN1, 0);
    analogWrite(MD3_IN2, -ux_pwm);

    analogWrite(MD2_IN1, -ux_pwm);
    analogWrite(MD2_IN2, 0);
  }

  if(uy > 0){
    analogWrite(MD1_IN1, uy_pwm);
    analogWrite(MD1_IN2, 0);
    
    analogWrite(MD4_IN1, 0);
    analogWrite(MD4_IN2, uy_pwm);
  }
  else{
    analogWrite(MD1_IN1, 0);
    analogWrite(MD1_IN2, -uy_pwm);
    
    analogWrite(MD4_IN1, -uy_pwm);
    analogWrite(MD4_IN2, 0);
  }
}

void rls_filter(double raw_value, double& estimated_value, double& estimated_derivative, double* theta, double P[2][2], double time) {
  double x[2] = {1, time}; // Time since start
  double K[2];
  double P_x[2] = {0, 0};
  double denom = 0;
  
  // Calculate P*x
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      P_x[i] += P[i][j] * x[j];
    }
    denom += P_x[i] * x[i];
  }
  denom = lambda + denom;

  // Calculate K
  for (int i = 0; i < 2; i++) {
    K[i] = P_x[i] / denom;
  }

  // Calculate error
  double y_hat = theta[0] + theta[1] * x[1];
  double e = raw_value - y_hat;

  // Update theta
  for (int i = 0; i < 2; i++) {
    theta[i] += K[i] * e;
  }

  // Update P
  double K_xP[2][2] = {0};
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      for (int k = 0; k < 2; k++) {
        K_xP[i][j] += K[i] * x[k] * P[k][j];
      }
    }
  }
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      P[i][j] = (P[i][j] - K_xP[i][j]) / lambda;
    }
  }

  // Output the estimated value and its derivative
  estimated_value = theta[0] + theta[1] * x[1];
  estimated_derivative = theta[1];
}

void loop() {
  if(micros() - prev_time >= T){
    current_time = micros();
    double elapsed_time = (current_time - start_time) / 1e6; // Time since start in seconds

    // Get measurement      
    Sensor.updateData();
    rawBx = Sensor.getX() - meanBx - 1.43/255*ux; // Compensate for bias by permanent and electromagnets
    rawBy = Sensor.getY() - meanBy - 1.43/255*uy;
    rawBz = Sensor.getZ() - meanBz;

    // Apply RLS filter
    rls_filter(rawBx, bx, dbx, theta_bx, P_bx, elapsed_time);
    rls_filter(rawBy, by, dby, theta_by, P_by, elapsed_time);
    rls_filter(rawBz, bz, dbz, theta_bz, P_bz, elapsed_time);

    if(abs(bz) > 3){
      ex = -bx;
      ey = -by;

      dex = -dbx;
      dey = -dby;

      ux = constrain(Kp*ex + Kd*dex, -255, 255);
      uy = constrain(Kp*ey + Kd*dex, -255, 255);
      change_input(ux, uy);
    }
    else{
        ux = 0;
        uy = 0;
        change_input(ux, uy);
    }

    if(loopCounter % 100 == 0){
      Serial.print(bx);
      Serial.print(',');
      Serial.print(rawBx);
      Serial.print(',');
      Serial.print(bz);
      Serial.print(',');
      Serial.print(ux);
      Serial.print(',');
      Serial.println(uy);
    }
    loopCounter++;

    prev_time = current_time;
  }
}

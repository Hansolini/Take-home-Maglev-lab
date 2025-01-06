#include <Tle493d_a2b6.h>
#include <Wire.h>
#include <algorithm> // Include for std::sort

// Sample timer
const double f_s = 5000; // Hz
double prev_time = 1;
double current_time = 2;
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
double bx_prev = 0, by_prev = 0, bz_prev = 0;
double dbx = 0, dby = 0, dbz = 0;
double dbx_prev = 0, dby_prev = 0, dbz_prev = 0;

// Mean of measurements
float meanBx = 0, meanBy = 0, meanBz = 0;

// PID
double Kp = 450, Kd = 1.5; // Increased Kp and fine-tuned Kd
double ux = 0, uy = 0;
double ex = 0, ey = 0;
double dex = 0, dey = 0;

// Initialize filter
static const int RM_WINDOW_SIZE = 21; // Adjust window size as needed
static double bx_window[RM_WINDOW_SIZE] = {0};
static double by_window[RM_WINDOW_SIZE] = {0};
static double bz_window[RM_WINDOW_SIZE] = {0};

void setup() {
  Serial.begin(115200);
  // while(!Serial);
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

// Repeated Median filter using linear regression
void repeated_median_filter(double* data, int windowSize, double& smoothed_value, double& derivative_value) {
  int halfWindow = windowSize / 2;
  double slopes[windowSize * windowSize];
  int idx = 0;
  
  for (int i = 0; i < windowSize; i++) {
    for (int j = 0; j < windowSize; j++) {
      if (i != j) {
        slopes[idx++] = (data[j] - data[i]) / (j - i);
      }
    }
  }

  std::sort(slopes, slopes + idx);
  double median_slope = slopes[idx / 2];
  
  double intercepts[windowSize];
  for (int i = 0; i < windowSize; i++) {
    intercepts[i] = data[i] - median_slope * i;
  }
  
  std::sort(intercepts, intercepts + windowSize);
  double median_intercept = intercepts[windowSize / 2];
  
  smoothed_value = median_intercept + median_slope * (windowSize - 1);
  derivative_value = median_slope * f_s;
}

void loop() {
  if(micros() - prev_time >= T){
    current_time = micros();
    // Get measurement      
    Sensor.updateData();
    rawBx = Sensor.getX() - meanBx - 1.43/255*ux; // Compensate for bias by permanent and electromagnets
    rawBy = Sensor.getY() - meanBy - 1.43/255*uy;
    rawBz = Sensor.getZ() - meanBz;

    // Update Repeated Median window
    for (int i = 0; i < RM_WINDOW_SIZE - 1; i++) {
      bx_window[i] = bx_window[i + 1];
      by_window[i] = by_window[i + 1];
      bz_window[i] = bz_window[i + 1];
    }
    bx_window[RM_WINDOW_SIZE - 1] = rawBx;
    by_window[RM_WINDOW_SIZE - 1] = rawBy;
    bz_window[RM_WINDOW_SIZE - 1] = rawBz;

    // Repeated Median filter for smoothing and derivative
    repeated_median_filter(bx_window, RM_WINDOW_SIZE, bx, dbx);
    repeated_median_filter(by_window, RM_WINDOW_SIZE, by, dby);
    repeated_median_filter(bz_window, RM_WINDOW_SIZE, bz, dbz);

    bx_prev = bx;
    by_prev = by;
    bz_prev = bz;
    
    dbx_prev = dbx;
    dby_prev = dby;

    if(abs(bz) > 3){
      ex = -bx;
      ey = -by;

      dex = -dbx;
      dey = -dby;

      ux = constrain(Kp*ex + Kd*dex, -255, 255);
      uy = constrain(Kp*ey + Kd*dey, -255, 255);
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
      Serial.print(dbx);
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

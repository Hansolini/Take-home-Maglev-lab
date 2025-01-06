#include <Tle493d_a2b6.h>
#include <Wire.h>

// Sample timer
const double f_s = 5000; // Hz
const int T = round(1e6 / f_s);
double prev_time = 1;
double current_time = 2;

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

// Mean of measurements
float meanBx = 0, meanBy = 0, meanBz = 0;

// PID
double Kp = 550, Kd = 3.5;
double ux = 0, uy = 0;

// Data collection for linear regression
const int data_size = 1000;
double ux_data[data_size];
double uy_data[data_size];
double rawBx_data[data_size];
double rawBy_data[data_size];
int data_index = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("Initialization...");

  // Initialize sensor
  Sensor.begin();

  // Initialize motor drivers
  analogWriteResolution(8);

  // Set pin mode
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

  // Start with motor drivers off
  digitalWrite(MD1_IN1, LOW);
  digitalWrite(MD1_IN2, LOW);
  
  digitalWrite(MD2_IN1, LOW);
  digitalWrite(MD2_IN2, LOW);
  
  digitalWrite(MD3_IN1, LOW);
  digitalWrite(MD3_IN2, LOW);

  digitalWrite(MD4_IN1, LOW);
  digitalWrite(MD4_IN2, LOW);

  // Adjust the I2C clock speed to reach maximum sample time (5.2kHz)
  Wire.begin();
  Wire.setClock(1000000); // Set I2C frequency to 1MHz

  // Compute sensor mean
  while (initCounter < 1000) {
    delay(1);
    Sensor.updateData();
    meanBx += Sensor.getX() / 1000;
    meanBy += Sensor.getY() / 1000;
    meanBz += Sensor.getZ() / 1000;
    
    initCounter++;
  }
}

int solenoid_index = 0;

void linear_regression(double x[], double y[], int n, double &slope, double &intercept) {
  double sum_x = 0, sum_y = 0, sum_xy = 0, sum_xx = 0;
  for (int i = 0; i < n; i++) {
    sum_x += x[i];
    sum_y += y[i];
    sum_xy += x[i] * y[i];
    sum_xx += x[i] * x[i];
  }
  slope = (n * sum_xy - sum_x * sum_y) / (n * sum_xx - sum_x * sum_x);
  intercept = (sum_y - slope * sum_x) / n;
}

void loop() {
  if (micros() - prev_time >= T) {
    current_time = micros();
    // Get measurement      
    Sensor.updateData();
    rawBx = Sensor.getX() - meanBx;
    rawBy = Sensor.getY() - meanBy;
    rawBz = Sensor.getZ() - meanBz;

    // Linearly increase/decrease ux and uy within the range
    ux = (loopCounter % 512) - 255; // Ranges from -255 to 255
    uy = (loopCounter % 512) - 255; // Ranges from -255 to 255

    if (ux < 0) {
      analogWrite(MD3_IN1, 0);
      analogWrite(MD3_IN2, abs(ux));

      analogWrite(MD2_IN1, abs(ux));
      analogWrite(MD2_IN2, 0);
    } else {
      analogWrite(MD3_IN1, abs(ux));
      analogWrite(MD3_IN2, 0);

      analogWrite(MD2_IN1, 0);
      analogWrite(MD2_IN2, abs(ux));
    }

    if (uy < 0) {
      analogWrite(MD1_IN1, 0);
      analogWrite(MD1_IN2, abs(uy));
        
      analogWrite(MD4_IN1, abs(uy));
      analogWrite(MD4_IN2, 0);
    } else {
      analogWrite(MD1_IN1, abs(uy));
      analogWrite(MD1_IN2, 0);
        
      analogWrite(MD4_IN1, 0);
      analogWrite(MD4_IN2, abs(uy));
    }

    if (data_index < data_size) {
      ux_data[data_index] = ux;
      uy_data[data_index] = uy;
      rawBx_data[data_index] = rawBx;
      rawBy_data[data_index] = rawBy;
      data_index++;
    }

    if (loopCounter % 300 == 0) {
      Serial.print(rawBx);
      Serial.print(',');
      Serial.print(rawBy);
      Serial.print(',');
      Serial.print(rawBz);
      Serial.print(',');
      Serial.print(ux / 255 * 0.05);
      Serial.print(',');
      Serial.println(uy / 255 * 0.05);
    }
    loopCounter++;

    prev_time = current_time;
  }

  // Perform linear regression once data collection is complete
  if (data_index >= data_size) {
    double slope_ux_bx, intercept_ux_bx, slope_uy_by, intercept_uy_by;
    linear_regression(ux_data, rawBx_data, data_size, slope_ux_bx, intercept_ux_bx);
    linear_regression(uy_data, rawBy_data, data_size, slope_uy_by, intercept_uy_by);

    Serial.print("Compensation Factor for UX: ");
    Serial.println(slope_ux_bx * 255);
    Serial.print("Compensation Factor for UY: ");
    Serial.println(slope_uy_by * 255);

    // Reset data index to stop further processing
    data_index = -1;
  }
}


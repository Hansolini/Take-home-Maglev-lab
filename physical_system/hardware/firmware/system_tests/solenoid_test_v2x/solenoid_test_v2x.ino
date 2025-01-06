#include <Tle493d_a2b6.h>
#include <MovingAveragePlus.h>

#include <Filters.h>
#include <Filters/Butterworth.hpp>
#include <AH/Timing/MillisMicrosTimer.hpp>

#include <Wire.h>

// IIR filter constants
#define ALPHA 0.06
#define DALPHA 0.06

// Sample timer
const double f_s = 5000; // Hz
Timer<micros> timer = std::round(1e6 / f_s);
double prev_time = 1;
double prev_time_switch = 1;
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
double bx = 0, by = 0, bz = 0;
double bx_prev = 0, by_prev = 0, bz_prev = 0;
double dbx = 0, dby = 0, dbz = 0;
double dbx_prev = 0, dby_prev = 0, dbz_prev = 0;

// Mean of measurements
float meanBx = 0, meanBy = 0, meanBz = 0;

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
  Wire.setClock(1000000); // Set I2C frequency to 400kHz

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

int solenoid_index = 0;

void loop() {
  if(timer){
    current_time = micros();
      // Get measurement      
      Sensor.updateData();
      rawBx = Sensor.getX() - meanBx - 1.85/255*ux; // Compensate for bias by permanent and electromagnets
      rawBy = Sensor.getY() - meanBy - 1.85/255*uy;
      rawBz = Sensor.getZ() - meanBz;

      bx = ALPHA * rawBx + (1.0 - ALPHA) * bx_prev;
      by = ALPHA * rawBy + (1.0 - ALPHA) * by_prev;
      bz = ALPHA * rawBz + (1.0 - ALPHA) * bz_prev;

      dbx = DALPHA * ((bx - bx_prev)/((current_time - prev_time + 10)/1000000.0)) + (1.0 - DALPHA) * dbx_prev;
      dby = DALPHA * ((by - by_prev)/((current_time - prev_time + 10)/1000000.0)) + (1.0 - DALPHA) * dby_prev;

      bx_prev = bx;
      by_prev = by;
      bz_prev = bz;
      
      dbx_prev = dbx;
      dby_prev = dby;

      if(current_time - prev_time_switch > 2000000){        
        solenoid_index = solenoid_index + 1;
        if(solenoid_index == 13){
          solenoid_index = 0;
        }

        prev_time_switch = current_time;
      }

  if (solenoid_index == 0){
        analogWrite(MD1_IN1, 0);
        analogWrite(MD1_IN2, 0);
  
        analogWrite(MD2_IN1, 0);
        analogWrite(MD2_IN2, 0);
  
        analogWrite(MD3_IN1, 0);
        analogWrite(MD3_IN2, 0);
  
        analogWrite(MD4_IN1, 0);
        analogWrite(MD4_IN2, 0);
      
      } else if (solenoid_index == 1){ // M1
        analogWrite(MD1_IN1, 255);
        analogWrite(MD1_IN2, 0);
      } else if (solenoid_index == 2){
        analogWrite(MD1_IN1, 0);
        analogWrite(MD1_IN2, 255);
      } else if (solenoid_index == 3){
        analogWrite(MD1_IN1, 0);
        analogWrite(MD1_IN2, 0);
  
      } else if (solenoid_index == 4){ // M2
        analogWrite(MD2_IN1, 255);
        analogWrite(MD2_IN2, 0);
      } else if (solenoid_index == 5){
        analogWrite(MD2_IN1, 0);
        analogWrite(MD2_IN2, 255);
      } else if (solenoid_index == 6){
        analogWrite(MD2_IN1, 0);
        analogWrite(MD2_IN2, 0);
    
  
      } else if (solenoid_index == 7){ // M3
        analogWrite(MD3_IN1, 255);
        analogWrite(MD3_IN2, 0);
      } else if (solenoid_index == 8){
        analogWrite(MD3_IN1, 0);
        analogWrite(MD3_IN2, 255);
      } else if (solenoid_index == 9){
        analogWrite(MD3_IN1, 0);
        analogWrite(MD3_IN2, 0);
    
  
      } else if (solenoid_index == 10){ // M4
        analogWrite(MD4_IN1, 255);
        analogWrite(MD4_IN2, 0);
      } else if (solenoid_index == 11){
        analogWrite(MD4_IN1, 0);
        analogWrite(MD4_IN2, 255);
      } else if (solenoid_index == 12){
        analogWrite(MD4_IN1, 0);
        analogWrite(MD4_IN2, 0);
      }

      if( loopCounter % 300 == 0){
        Serial.print(bx);
        Serial.print(',');
        Serial.print(by);
        Serial.print(',');
        Serial.print(bz);
        Serial.print(',');
        Serial.println(solenoid_index);
      }
    loopCounter++;

    prev_time = current_time;
  }
}


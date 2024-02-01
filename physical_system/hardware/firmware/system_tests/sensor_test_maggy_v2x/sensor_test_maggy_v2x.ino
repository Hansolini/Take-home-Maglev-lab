#include <Tle493d_a2b6.h>
#include <Wire.h>

// Sample timer
const double f_s = 5000; // Hz
double prev_time = 1;
double current_time = 2;
const int T = round(1e6 / f_s);

// Sensors
Tle493d_a2b6 Sensor = Tle493d_a2b6(Tle493d::FASTMODE, Tle493d::TLE493D_A0);

// Counters
int initCounter = 0;
int loopCounter = 0;

// Measurements
double rawBx = 0, rawBy = 0, rawBz = 0;
double bx = 0, by = 0, bz = 0;

void setup() {
  Serial.begin(115200);
  // while(!Serial);
  Serial.println("Initialization...");

  // Initialize sensor
  Sensor.begin();

  // We adjust the I2C clock speed to reach maximum sample time (5.2kHz). This HAS to be done after initializing the sensors!
  Wire.begin();
  Wire.setClock(1000000); // Set I2C frequency to 400kHz
}

void loop() {
  if(micros() - prev_time >= T){
    current_time = micros();
      // Get measurement      
      Sensor.updateData();
      bx = Sensor.getX();
      by = Sensor.getY();
      bz = Sensor.getZ();

      if( loopCounter % 10 == 0){
        Serial.print(bx);
        Serial.print(',');
        Serial.print(by);
        Serial.print(',');
        Serial.println(bz);
      }
    loopCounter++;

    prev_time = current_time;
  }
}

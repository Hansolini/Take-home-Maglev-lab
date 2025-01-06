#include <Tle493d_a2b6.h>
#include <Wire.h>

// Sample timer
const int f_s = 50; // Hz -> Not necessary with higher rates when testing sensors
unsigned long prev_time = 0;
const int T = round(1e6 / f_s);

// Sensors
Tle493d_a2b6 Sensor = Tle493d_a2b6(Tle493d::FASTMODE, Tle493d::TLE493D_A0);

// Measurements
double bx = 0, by = 0, bz = 0;

void setup() {
  // Initialize serial
  Serial.begin(115200);
  Serial.println("Initialization...");

  // Initialize sensor
  Sensor.begin();

  // Set I2C frequency to 1MHz
  Wire.begin();
  Wire.setClock(1000000);

  // Initialize previous time variables
  prev_time = micros();
}

void loop() {
  unsigned long current_time = micros();

  // Check if sampling interval has passed
  if (current_time - prev_time >= T) {
    prev_time = micros(); // Reset timer for next interval

    // Update sensor measurements    
    Sensor.updateData();
    bx = Sensor.getX();
    by = Sensor.getY();
    bz = Sensor.getZ();

    // Print magnetic field data for each axis
    Serial.print("Bx:");
    Serial.print(bx);
    Serial.print(',');
    Serial.print("By:");
    Serial.print(by);
    Serial.print(',');
    Serial.print("Bz:");
    Serial.println(bz);
  }
}

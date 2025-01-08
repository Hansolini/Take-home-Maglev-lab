#include <Tle493d_a2b6.h>
#include <Wire.h>

// ############# TO BE MODIFIED ###########
const double DIRECT_FEEDTHROUGH_SLOPE_X_POSITIVE = 0;
const double DIRECT_FEEDTHROUGH_SLOPE_X_NEGATIVE = 0;
const double DIRECT_FEEDTHROUGH_SLOPE_Y_POSITIVE = 0;
const double DIRECT_FEEDTHROUGH_SLOPE_Y_NEGATIVE = 0;

const bool REMOVE_MEAN = false;
const bool REMOVE_DIRECT_FEEDTHROUGH = false;
// ########################################

// Sample timer
const int f_s = 50; // Sampling frequency in Hz
const unsigned long T_sample = round(1e6 / f_s); // Sampling interval in microseconds
const unsigned long T_switch = 1e6; // Switching interval in microseconds (1 Hz)
unsigned long prev_time = 0;
unsigned long prev_time_switch = 0;

// Counters
int initCounter = 0;
int solenoid_index = 0;

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

// Inputs
int uxp = 0, uxn = 0, uyp = 0, uyn = 0;

// Measurements
double bx = 0, by = 0, bz = 0;

// Mean of measurements
float meanBx = 0, meanBy = 0, meanBz = 0;

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

  // Initialize previous time variables
  prev_time = micros();
  prev_time_switch = micros();


  if (REMOVE_MEAN){
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

  // Check if sampling interval has passed
  if (current_time - prev_time >= T_sample) {
    prev_time += T_sample; // Reset timer for next interval

    // Update sensor measurements    
    Sensor.updateData();
    bx = Sensor.getX();
    by = Sensor.getY();
    bz = Sensor.getZ();

    if (REMOVE_MEAN) {
      bx = bx - meanBx;
      by = by - meanBy;
      bz = bz - meanBz;
    }

    if (REMOVE_DIRECT_FEEDTHROUGH) {
      bx = bx - DIRECT_FEEDTHROUGH_SLOPE_X_POSITIVE/255.0*uxp - DIRECT_FEEDTHROUGH_SLOPE_X_NEGATIVE/255.0*uxn;
      by = by - DIRECT_FEEDTHROUGH_SLOPE_Y_POSITIVE/255.0*uyp - DIRECT_FEEDTHROUGH_SLOPE_Y_NEGATIVE/255.0*uyn;
    }

    // Print magnetic field data
    Serial.print("Bx:");
    Serial.print(bx);
    Serial.print(',');
    Serial.print("By:");
    Serial.print(by);
    Serial.print(',');
    Serial.print("Bz:");
    Serial.print(bz);
    Serial.print(',');

    // Print expected measurement data from activating one solenoid each iteration
    Serial.print("Expected_Bx:");    
    if (solenoid_index == 1 || solenoid_index == 5){
      Serial.print(1);
    } else if (solenoid_index == 2 || solenoid_index == 4){
      Serial.print(-1);
    } else {
      Serial.print(0);
    }
    Serial.print(',');

    Serial.print("Expected_By:");    
    if (solenoid_index == 7 || solenoid_index == 11){
      Serial.println(1);
    } else if (solenoid_index == 8 || solenoid_index == 10){
      Serial.println(-1);
    } else {
      Serial.println(0);
    }
  }

  // Check if switching interval has passed
  if (current_time - prev_time_switch >= T_switch) {
    prev_time_switch += T_switch; // Reset timer for next interval

    solenoid_index += 1;
    if(solenoid_index >= 13){
      solenoid_index = 0;
    }

    // Control solenoids based on solenoid_index
    if (solenoid_index == 0){
      uxp = 0;
      uxn = 0;
      uyp = 0;
      uyn = 0;

    } else if (solenoid_index == 10){ // M1 - In_y
      uyn = 127;
    } else if (solenoid_index == 11){
      uyn = -127;
    } else if (solenoid_index == 12){
      uyn = 0;
  
    } else if (solenoid_index == 1){ // M2 - Ip_x
      uxp = 127;
    } else if (solenoid_index == 2){
      uxp = -127;
    } else if (solenoid_index == 3){
      uxp = 0;
  
    } else if (solenoid_index == 4){ // M3 - In_x
      uxn = 127;
    } else if (solenoid_index == 5){
      uxn = -127;
    } else if (solenoid_index == 6){
      uxn = 0;
  
    } else if (solenoid_index == 7){ // M4 - Ip_y
      uyp = 127;
    } else if (solenoid_index == 8){
      uyp = -127;
    } else if (solenoid_index == 9){
      uyp = 0;
    }

    control_solenoid(2, uxp);
    control_solenoid(3, uxn);
    control_solenoid(4, uyp);
    control_solenoid(1, uyn);
  }
}

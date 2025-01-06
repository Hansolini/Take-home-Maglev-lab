#include <Tle493d_a2b6.h>
#include <Wire.h>

const byte maxBytes = 250; // Maximum bytes to receive
byte receivedBytes[maxBytes];
boolean newData = false;
static byte ndx = 0;
boolean receiving = false;
char startMarker = '<';
char endMarker = '>';
byte rc;

Tle493d_a2b6 Sensor = Tle493d_a2b6(Tle493d::FASTMODE, Tle493d::TLE493D_A0);

// Motor driver definitions
#define MD1_IN1 3
#define MD1_IN2 2
#define MD2_IN1 5
#define MD2_IN2 4
#define MD3_IN1 7
#define MD3_IN2 6
#define MD4_IN1 9
#define MD4_IN2 8

// Buffer for storing sensor data and timestamps
#define BUFFER_SIZE 1 // Number of samples to accumulate before sending
float buffer[BUFFER_SIZE][4]; // Buffer to store sensor data (3 floats + 1 timestamp per sample)
int bufferIndex = 0; // Index to keep track of buffer position

// Timing variables
unsigned long lastSampleTime = 0;
unsigned long sampleInterval = 100; // Sampling interval in microseconds (5kHz sampling rate)

// Storing input to remove direct feedthrough
int16_t ux = 0;
int16_t uy = 0;

void setup() {
  // Initialize Serial (USB)
  Serial.begin(9600); // Dummy baud rate, USB doesn't use it but initialization is required

  // Initialize sensor
  Sensor.begin();

  // Modify I2C clock speed (HAS TO HAPPEN AFTER SENSOR INITIALIZATION!)
  Wire.begin();
  Wire.setClock(3400000);

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

  // Wait 2 seconds before starting
  delay(2000);
}

int16_t map_from_u_to_pwm(int16_t u) {
  if (u > 0) {
    double a = (165.0 - 135.0) / 255.0;
    double b = 135.0;
    return round(a * u + b);
  } else if (u < 0) {
    double a = (165.0 - 135.0) / 255.0;
    double b = 134.5;
    return round(a * u - b);
  }
  return 0;
}

void change_input(int16_t ux, int16_t uy) {
  int16_t ux_pwm = map_from_u_to_pwm(ux);
  int16_t uy_pwm = map_from_u_to_pwm(uy);

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
  unsigned long currentTime = micros();

  // Sample data at 5kHz
  if (currentTime - lastSampleTime >= sampleInterval) {
    lastSampleTime = currentTime;

    // Read sensor data and store in buffer
    Sensor.updateData();
    buffer[bufferIndex][0] = Sensor.getX();
    buffer[bufferIndex][1] = Sensor.getY();
    buffer[bufferIndex][2] = Sensor.getZ();
    buffer[bufferIndex][3] = (float)currentTime; // Store the timestamp

    // Increment buffer index
    bufferIndex++;

    // Check if buffer is full
    if (bufferIndex >= BUFFER_SIZE) {
      bufferIndex = 0; // Reset buffer index

      // Prepare data to send
      byte data[BUFFER_SIZE * 4 * sizeof(float)];
      memcpy(data, buffer, sizeof(buffer));

      // Send data  
      Serial.write(data, sizeof(data)); // Send using USB
      Serial.send_now();
    }

    // Handle motor control and receiving data
      // ---------Receive data----------
    while (Serial.available() > 0 && !newData) {
        rc = Serial.read();

        if (receiving) {
            if (rc != endMarker) {
                if (ndx < maxBytes) {
                    receivedBytes[ndx] = rc;
                    ndx++;
                }
            } else {
                receiving = false;
                // Only set newData to true if the expected number of bytes is received
                if (ndx == sizeof(int16_t) * 2) {
                    newData = true;
                } else {
                    // Handle the incomplete data case
                    newData = false;
                    Serial.flush();  // Flush the serial buffer to remove incomplete data
                }
            }
        } else if (rc == startMarker) {
            receiving = true;
            ndx = 0; // Reset index for new message
        }
    }

    // ---------Update solenoid current----------
    if (newData) {
        // Ensure we have received exactly the expected number of bytes
        if (ndx == sizeof(int16_t) * 2) {
            int16_t receivedInts[2];
            memcpy(&receivedInts[0], &receivedBytes[0], sizeof(int16_t));
            memcpy(&receivedInts[1], &receivedBytes[sizeof(int16_t)], sizeof(int16_t));
            
            ux = receivedInts[0];
            uy = receivedInts[1];
            
            // Use the received values as needed
            change_input(ux, uy);
        }

        newData = false;  // Reset for the next data packet
    }

  }
}

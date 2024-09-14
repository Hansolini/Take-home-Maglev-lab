#include <Tle493d_a2b6.h>
#include <Wire.h>

// ========================================================
//                      FUNCTION PROTOTYPES
// ========================================================
bool receiveSerialData(int16_t* receivedData);
void readSensorData(float* sensorValues);
void setSolenoidCurrents(const int16_t* receivedData);
void updateDataBuffer(const float* sensorValues, const int16_t* currentSolenoidCurrents);
void sendDataBuffer();
void analogWriteMotor(int pwm, int pin1, int pin2);
void resetSolenoidCurrents();

// ========================================================
//                      TIMING VARIABLES
// ========================================================
unsigned long lastSampleTime = 0;                // Tracks the last sample time
unsigned long sampleInterval = 200;              // Sampling interval in microseconds (5kHz sampling rate)
unsigned long lastDataReceivedTime = 0;          // Tracks the last time data was received

// ========================================================
//                    SENSOR CONFIGURATION
// ========================================================
Tle493d_a2b6 sensors[] = {
    Tle493d_a2b6(Tle493d::FASTMODE, Tle493d::TLE493D_A0),
    // Tle493d_a2b6(Tle493d::FASTMODE, Tle493d::TLE493D_A1),
    // Tle493d_a2b6(Tle493d::FASTMODE, Tle493d::TLE493D_A2)
};
constexpr int NUM_SENSORS = sizeof(sensors) / sizeof(sensors[0]);
constexpr int PWM_FREQUENCY = 20000;//32226.56;             // Frequency for PWM to avoid audible tones
constexpr int PWM_BIT_SIZE = 12;
constexpr int PWM_MAX = pow(2,PWM_BIT_SIZE);

// ========================================================
//                    SOLENOID CONFIGURATION
// ========================================================

// y-
constexpr int MD1_IN1 = 3;
constexpr int MD1_IN2 = 2;

// x+
constexpr int MD2_IN1 = 5;
constexpr int MD2_IN2 = 4;

// x-
constexpr int MD3_IN1 = 7;
constexpr int MD3_IN2 = 6;

// y+
constexpr int MD4_IN1 = 9;
constexpr int MD4_IN2 = 8;

constexpr int MOTOR_PINS[][2] = {
    {MD2_IN1, MD2_IN2}, // x+
    {MD3_IN1, MD3_IN2}, // x-
    {MD4_IN1, MD4_IN2}, // y+
    {MD1_IN1, MD1_IN2}, // y-
};
constexpr int NUM_MOTOR_PINS = sizeof(MOTOR_PINS) / sizeof(MOTOR_PINS[0]);

// ========================================================
//                  DATA RECEPTION SETUP
// ========================================================
const byte MAX_RECEIVE_BYTES = 2 * NUM_MOTOR_PINS; // Max bytes to receive, matches int16_t values count for solenoids
byte serialReceiveBuffer[MAX_RECEIVE_BYTES];       // Buffer to hold incoming serial data
static byte receiveBufferIndex = 0;                // Index for filling serialReceiveBuffer

// Serial communication state flags
bool isReceivingData = false;                      // Indicates if currently receiving data
bool isBufferReadyToSend = false;                  // Indicates when the data buffer is ready to send
bool solenoidsResetDueToTimeout = false;           // Indicates if solenoids have been reset due to timeout

// Serial communication markers
const char PACKET_START_MARKER = '<';              // Start marker for incoming data packets
const char PACKET_END_MARKER = '>';                // End marker for incoming data packets

byte incomingByte;                                 // Holds each incoming byte from Serial.read()
int16_t solenoidCurrents[NUM_MOTOR_PINS] = {0};    // Array to store received int16_t values for motor pins

// ========================================================
//                  DATA TRANSMISSION SETUP
// ========================================================

// Define byte sizes associated with specific data types
constexpr int BYTES_PER_TIMESTAMP = sizeof(unsigned long);         // 4 bytes for timestamp
constexpr int BYTES_PER_SENSOR_MEASUREMENT = sizeof(float);        // 4 bytes for each sensor measurement (float)
constexpr int BYTES_PER_SOLENOID_CURRENT = sizeof(int16_t);        // 2 bytes for each solenoid current (int16_t)

// Calculate the total bytes per sample
constexpr int BUFFER_SIZE = 5;                                     // Number of samples to accumulate before sending
constexpr int SAMPLES_PER_SENSOR = 3;                              // Three values per sensor (X, Y, Z)
constexpr int TOTAL_BYTES_PER_SAMPLE = BYTES_PER_TIMESTAMP + 
                                       (SAMPLES_PER_SENSOR * NUM_SENSORS * BYTES_PER_SENSOR_MEASUREMENT) + 
                                       (NUM_MOTOR_PINS * BYTES_PER_SOLENOID_CURRENT);

// Define the send buffer using the corrected size
byte serialSendBuffer[BUFFER_SIZE][TOTAL_BYTES_PER_SAMPLE];        // Buffer to store data for sending
int bufferIndex = 0;                                               // Index to keep track of buffer position

// ========================================================
//                       SETUP FUNCTION
// ========================================================
void setup() {
    Serial.begin(9600); // Initialize Serial (USB)

    // Initialize sensors
    for (int i = 0; i < NUM_SENSORS; i++) {
        sensors[i].begin();
    }

    // Modify I2C clock speed (IMPORTANT: HAS TO HAPPEN AFTER SENSOR INITIALIZATION!)
    Wire.begin();
    Wire.setClock(3400000);

    // Initialize motor drivers
    analogWriteResolution(PWM_BIT_SIZE);

    for (int i = 0; i < NUM_MOTOR_PINS; i++) {
        pinMode(MOTOR_PINS[i][0], OUTPUT);
        pinMode(MOTOR_PINS[i][1], OUTPUT);

        digitalWrite(MOTOR_PINS[i][0], LOW); // Start with motor drivers off
        digitalWrite(MOTOR_PINS[i][1], LOW);

        analogWriteFrequency(MOTOR_PINS[i][0], PWM_FREQUENCY); // Set PWM frequency
        analogWriteFrequency(MOTOR_PINS[i][1], PWM_FREQUENCY);
    }

    delay(2000); // Wait before starting main loop
}

// ========================================================
//                        MAIN LOOP
// ========================================================
void loop() {
    unsigned long currentTime = micros();

    // Check if it's time to sample data (5kHz)
    if (currentTime - lastSampleTime >= sampleInterval) {
        lastSampleTime = currentTime;

        int16_t receivedData[NUM_MOTOR_PINS];  // Array for received current values
        if (receiveSerialData(receivedData)) { // Receive data from Serial
            lastDataReceivedTime = millis();   // Update last data received time
            setSolenoidCurrents(receivedData); // Update solenoid currents first
            solenoidsResetDueToTimeout = false; // Reset timeout flag after receiving new data
        } else if (millis() - lastDataReceivedTime > 1000 && !solenoidsResetDueToTimeout) { // Check if more than 1 second has passed
            resetSolenoidCurrents();           // Reset solenoid currents to zero
            solenoidsResetDueToTimeout = true; // Set timeout flag to prevent repeated resets
        }

        float sensorValues[SAMPLES_PER_SENSOR * NUM_SENSORS]; // Array for sensor data
        readSensorData(sensorValues); // Read sensor values immediately after updating currents

        // Update the buffer with sensor values and the current state of the solenoids
        updateDataBuffer(sensorValues, solenoidCurrents); // Pass solenoidCurrents for accurate state capture

        if (isBufferReadyToSend) { // Check if the buffer is full
            sendDataBuffer(); // Send data if buffer is ready
        }
    }
}

// ========================================================
//                    FUNCTION DEFINITIONS
// ========================================================

// Receives serial data for solenoid currents
bool receiveSerialData(int16_t* receivedData) {
    while (Serial.available() > 0) {
        incomingByte = Serial.read();  // Read the next byte from serial

        if (isReceivingData) {  // If receiving a data packet
            if (incomingByte != PACKET_END_MARKER) {  // Not the end marker
                if (receiveBufferIndex < MAX_RECEIVE_BYTES) {  // Ensure space in the buffer
                    serialReceiveBuffer[receiveBufferIndex++] = incomingByte;  // Store the byte
                } else {
                    isReceivingData = false; // Stop if buffer overflow
                    receiveBufferIndex = 0; // Reset index
                }
            } else {  // End marker received
                isReceivingData = false;
                // Process data if correct size
                if (receiveBufferIndex == sizeof(int16_t) * NUM_MOTOR_PINS) {
                    memcpy(receivedData, serialReceiveBuffer, sizeof(int16_t) * NUM_MOTOR_PINS); // Copy data
                    return true; // Data ready
                }
                receiveBufferIndex = 0; // Reset index
            }
        } else if (incomingByte == PACKET_START_MARKER) {  // Start marker received
            isReceivingData = true;  // Start new packet
            receiveBufferIndex = 0; // Reset index
        }
    }
    return false; // No new data
}

// Reads sensor data from TLE493D sensors
void readSensorData(float* sensorValues) {
    int idx = 0; // Index for sensor values array
    for (int i = 0; i < NUM_SENSORS; i++) {
        sensors[i].updateData(); // Update sensor readings
        sensorValues[idx++] = sensors[i].getX(); // Store X-axis data
        sensorValues[idx++] = sensors[i].getY(); // Store Y-axis data
        sensorValues[idx++] = sensors[i].getZ(); // Store Z-axis data
    }
}

// Sets solenoid currents based on received data
void setSolenoidCurrents(const int16_t* receivedData) {
    for (int i = 0; i < NUM_MOTOR_PINS; i++) {
        solenoidCurrents[i] = receivedData[i];  // Store received current values
        analogWriteMotor(solenoidCurrents[i], MOTOR_PINS[i][0], MOTOR_PINS[i][1]); // Apply PWM
    }
}

// Resets solenoid currents to zero
void resetSolenoidCurrents() {
    int16_t zeroCurrents[NUM_MOTOR_PINS] = {0}; // Array of zero values
    setSolenoidCurrents(zeroCurrents);          // Set solenoid currents to zero
}

// Updates the data buffer with the latest sensor readings and solenoid currents
void updateDataBuffer(const float* sensorValues, const int16_t* currentSolenoidCurrents) {
    int bufIdx = 0; // Start index for storing data in serialSendBuffer

    // Store timestamp as bytes
    unsigned long timestamp = micros();
    memcpy(&serialSendBuffer[bufferIndex][bufIdx], &timestamp, BYTES_PER_TIMESTAMP);
    bufIdx += BYTES_PER_TIMESTAMP;

    // Store sensor measurements as bytes
    for (int i = 0; i < SAMPLES_PER_SENSOR * NUM_SENSORS; i++) {
        memcpy(&serialSendBuffer[bufferIndex][bufIdx], &sensorValues[i], BYTES_PER_SENSOR_MEASUREMENT);
        bufIdx += BYTES_PER_SENSOR_MEASUREMENT;
    }

    // Store solenoid currents as bytes
    for (int i = 0; i < NUM_MOTOR_PINS; i++) {
        memcpy(&serialSendBuffer[bufferIndex][bufIdx], &currentSolenoidCurrents[i], BYTES_PER_SOLENOID_CURRENT);
        bufIdx += BYTES_PER_SOLENOID_CURRENT;
    }

    bufferIndex++; // Increment buffer index

    if (bufferIndex >= BUFFER_SIZE) { // Check if buffer is full
        bufferIndex = 0; // Reset buffer index
        isBufferReadyToSend = true; // Set flag indicating buffer is full
    }
}

// Sends the buffered data over USB
void sendDataBuffer() {
    if (isBufferReadyToSend) {
        // Correct calculation of the total buffer size in bytes
        size_t bufferSizeInBytes = BUFFER_SIZE * TOTAL_BYTES_PER_SAMPLE;

        // Cast the 2D array to a 1D byte pointer and pass it to Serial.write
        Serial.write(reinterpret_cast<const uint8_t*>(serialSendBuffer), bufferSizeInBytes); // Send the buffer over USB
        Serial.send_now(); // Ensure the data is sent immediately

        isBufferReadyToSend = false; // Reset the flag
    }
}

// Applies PWM to control the solenoids  (This sets solenoids using slow decay mode: https://forum.allaboutcircuits.com/threads/strange-full-h-bridge-problem.161122/)
void analogWriteMotor(int pwm, int pin1, int pin2) {
    if (pwm > 0) {
        analogWrite(pin1, PWM_MAX-pwm);
        analogWrite(pin2, PWM_MAX);
    } else if (pwm < 0) {
        analogWrite(pin1, PWM_MAX);
        analogWrite(pin2, PWM_MAX+pwm);
    } else {
        analogWrite(pin1, 0);
        analogWrite(pin2, 0);
    }
}

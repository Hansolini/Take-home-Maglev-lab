#ifndef DEFINITIONS_H
#define DEFINITIONS_H

// === Constants and Macros ===
// I2C reset pin
#define RESET_I2C 26
#define SDA_PIN 18
#define SCL_PIN 19

// Motor driver pins
#define MD1_IN1 4
#define MD1_IN2 5
#define MD2_IN1 2
#define MD2_IN2 3
#define MD3_IN1 6
#define MD3_IN2 7
#define MD4_IN1 8
#define MD4_IN2 9

// Current sensor pins
#define CURRENT_Y_POS 20
#define CURRENT_X_NEG 21
#define CURRENT_X_POS 22
#define CURRENT_Y_NEG 23

// Sensor configuration
#define NUM_SENSORS 3
extern const int SENSOR_CHANNELS[NUM_SENSORS];
#define PRIMARY_SENSOR_INDEX 0

#endif // DEFINITIONS_H

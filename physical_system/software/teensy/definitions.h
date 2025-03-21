#ifndef DEFINITIONS_H
#define DEFINITIONS_H

// === constants ===

// serial communication baud rate
#define BAUDRATE 115200

// pwm values for solenoids
#define PWM_BIT_SIZE_R 10     // read bit size resolution
#define PWM_BIT_SIZE_W 8      // write bit size resolution
#define PWM_FREQUENCY 52258

// I2C
#define I2C_CLOCK_SPEED 1e6

// sensors MUX
#define MUX_CHANNEL 7


// === pins ===

// I2C reset pin
#define RESET_I2C 26
#define SDA_PIN 18
#define SCL_PIN 19

// Motor driver pins
// y-
#define MD1_IN1 4
#define MD1_IN2 5
// x+
#define MD2_IN1 2
#define MD2_IN2 3
// x-
#define MD3_IN1 6
#define MD3_IN2 7
// y+
#define MD4_IN1 8
#define MD4_IN2 9

// Current sensor pins
#define CURRENT_Y_POS 20
#define CURRENT_X_NEG 21
#define CURRENT_X_POS 22
#define CURRENT_Y_NEG 23

#endif // DEFINITIONS_H

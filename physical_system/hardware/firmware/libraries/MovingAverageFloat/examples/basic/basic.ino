#include "MovingAverageFloat.h"  // https://github.com/pilotak/MovingAverageFloat

// Buffer will be 16 samples long, it will take 16 * sizeof(float) = 64 bytes of RAM
MovingAverageFloat <16> filter;

void setup() {
    Serial.begin(9600);
    Serial.print("result: ");
    Serial.println(filter.add(1.5)); // insert new number and get result
    Serial.print("result: ");
    Serial.println(filter.add(2.5)); // insert new number and get result
    Serial.print("result: ");
    Serial.println(filter.add(2.4)); // insert new number and get result
    Serial.print("result: ");
    Serial.println(filter.get()); // get last result, without adding a newone
}

void loop() {

}
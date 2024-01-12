# Moving average filter for float numbers
[![Build status](https://github.com/pilotak/MovingAverageFloat/workflows/build/badge.svg)](https://github.com/pilotak/MovingAverageFloat/actions)
[![Framework Badge Arduino](https://img.shields.io/badge/framework-arduino-00979C.svg)](https://arduino.cc)
[![Framework Badge mbed](https://img.shields.io/badge/framework-mbed-008fbe.svg)](https://os.mbed.com/)

## Arduino example
Please see `examples` folder

## Mbed example
```cpp
#include "mbed.h"
#include "MovingAverageFloat.h"

// Buffer will be 16 samples long, it will take 16 * sizeof(float) = 64 bytes of RAM
MovingAverageFloat <16> filter;

int main() {
    printf("result: %.2f\n", filter.add(1.5)); // insert new number and get result
    printf("result: %.2f\n", filter.add(2.5)); // insert new number and get result
    printf("result: %.2f\n", filter.add(2.4)); // insert new number and get result
    printf("result: %.2f\n", filter.get()); // get last result, without adding a newone

    return 0;
}
```

### Output
> result: 1.50
> 
> result: 1.56
> 
> result: 1.62
> 
> result: 1.62

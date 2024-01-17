/**
 * Arduino Savitzky Golay Library - Version 1.0.1
 * by James Deromedi <jmderomedi@gmail.com>
 *
 * This Library is licensed under the MIT License
*/

#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include <SavLayFilter.h>

/*(Constructor)
* The parameters allow a link between the users input data, so they do not have to specify which
* data needs to be filtered. An object must be called for each different filter and windowsize
*/
SavLayFilter::SavLayFilter(double* input, int tableNumber, int windowNumber){
  myInput = input;                           //Creates the actual link
  myWindowSize = windowNumber;               //Saves the window size

/*Sets all other private variables to zero*/
  _fillArrayCount = 0;                       
  _arrayPointer = 0;
  _sum = 0.0;
  _smoothedValue = 0.0;

/*Meant to link and save the convolation table to be used*/
  switch (tableNumber) {                     
    case 0:
      _convoluteTable = _quadCubicSmooth;    
      break;  
      case 1:
      _convoluteTable =_quadFirstDerive;
      break;
      case 2:
      _convoluteTable = _quarticQuinticSmooth;
      break;
      case 3:
      _convoluteTable = _cubicQuarticFirstDerive;
      break;
      case 4:
      _convoluteTable = _quinticFirstDerive;
      break;
      case 5:
      _convoluteTable = _quadCubicSecDerive;
      break;
      case 6:
      _convoluteTable = _quarticQuinticSecDerive;
      break;
      case 7:
      _convoluteTable = _cubicQuarticThirdDerive;
      break;
      case 8:
      _convoluteTable = _quinticThirdDerive;
      break;
      case 9:
      _convoluteTable = _quarticQuinticFourDerive;
      break;
      case 10:
      _convoluteTable = _quinticFifthDerive;
      break;
    default:                                //If anything else is given, automatically cubic smoothing
    _convoluteTable = _quadCubicSmooth;
  } //End Switch
}

/*_calculating()
*   A private function that does the actual math. It returns the smoothed value from the windowsize given by the user
*/
// Modified version of _calculating() for forward Savitzky-Golay
float SavLayFilter::_calculating(double inputArray[]) {
  _sum = 0.0;
  // In a forward Savitzky-Golay filter, the window starts at the current point
  // and includes the next (myWindowSize - 1) points.
  for (int i = 0; i < myWindowSize; i++) {
    _sum += (inputArray[i]) * (_convoluteTable[_arrayPointer][i + 1]);
  }
  _sum = (_sum / *_normalizationFactor);
  return _sum;
}

/** Compute()
* What the user calls to filter their data. This function should and be called right after the data
* that is to be smoothed is collected. This allows accurate filtering of the new data.
*/
// Modified version of Compute() for forward Savitzky-Golay
float SavLayFilter::Compute() {
  // Fill the array only up to myWindowSize
  if (_fillArrayCount < myWindowSize) {
    _toBeSmoothedArray[_fillArrayCount] = *myInput;
    _fillArrayCount++;
    _smoothedValue = 0.0;  // Or another placeholder value
    return _smoothedValue;
  } else {
    // Call the calculation function on the window size set by the user
    switch (myWindowSize) {
      case 5:
        _arrayPointer = 0;
        _normalizationFactor = _convoluteTable[_arrayPointer];
        _smoothedValue = SavLayFilter::_calculating(_toBeSmoothedArray);
        break;
      case 7:
        _arrayPointer = 1;
        _normalizationFactor = _convoluteTable[_arrayPointer];
        _smoothedValue = SavLayFilter::_calculating(_toBeSmoothedArray);
        break;
      case 9:
        _arrayPointer = 2;
        _normalizationFactor = _convoluteTable[_arrayPointer];
        _smoothedValue = SavLayFilter::_calculating(_toBeSmoothedArray);
        break;
      case 11:
        _arrayPointer = 3;
        _normalizationFactor = _convoluteTable[_arrayPointer];
        _smoothedValue = SavLayFilter::_calculating(_toBeSmoothedArray);
        break;
      case 13:
        _arrayPointer = 4;
        _normalizationFactor = _convoluteTable[_arrayPointer];
        _smoothedValue = SavLayFilter::_calculating(_toBeSmoothedArray);
        break;
      case 15:
        _arrayPointer = 5;
        _normalizationFactor = _convoluteTable[_arrayPointer];
        _smoothedValue = SavLayFilter::_calculating(_toBeSmoothedArray);
        break;
      case 17:
        _arrayPointer = 6;
        _normalizationFactor = _convoluteTable[_arrayPointer];
        _smoothedValue = SavLayFilter::_calculating(_toBeSmoothedArray);
        break;
      case 19:
        _arrayPointer = 7;
        _normalizationFactor = _convoluteTable[_arrayPointer];
        _smoothedValue = SavLayFilter::_calculating(_toBeSmoothedArray);
        break;
      case 21:
        _arrayPointer = 8;
        _normalizationFactor = _convoluteTable[_arrayPointer];
        _smoothedValue = SavLayFilter::_calculating(_toBeSmoothedArray);
        break;
      case 23:
        _arrayPointer = 9;
        _normalizationFactor = _convoluteTable[_arrayPointer];
        _smoothedValue = SavLayFilter::_calculating(_toBeSmoothedArray);
        break;
      case 25:
        _arrayPointer = 10;
        _normalizationFactor = _convoluteTable[_arrayPointer];
        _smoothedValue = SavLayFilter::_calculating(_toBeSmoothedArray);
        break;
      default:
        _smoothedValue = -9999999;
    }

    return _smoothedValue;
  }
}

void SavLayFilter::UpdateData(double newInput) {
  // Move the window forward by removing the first number and adding the new input
  for (int j = 1; j < myWindowSize; j++) {
    _toBeSmoothedArray[j - 1] = _toBeSmoothedArray[j];
  }
  _toBeSmoothedArray[myWindowSize - 1] = newInput;
}



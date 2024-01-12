/*
MIT License

Copyright (c) 2018 Pavel Slama

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef MOVINGAVERAGEFLOAT_H
#define MOVINGAVERAGEFLOAT_H

#if defined(__MBED__)
#include "mbed.h"
#endif

template <uint8_t N>
class MovingAverageFloat {
 public:
  MovingAverageFloat();
  virtual ~MovingAverageFloat(void);

  float add(float value);
  float get();
  double get_sum();
  void fill(float value);
  void reset();

 private:
  bool _first;
  uint8_t _next;
  double _sum;

  float _buffer[N];
  float _result;
};

template <uint8_t N>
MovingAverageFloat<N>::MovingAverageFloat():
  _first(true),
  _next(0),
  _sum(0) {
  _result = 0;
}

template <uint8_t N>
MovingAverageFloat<N>::~MovingAverageFloat(void) {
}

template <uint8_t N>
float MovingAverageFloat<N>::get() {
  return _result;
}
template <uint8_t N>
double MovingAverageFloat<N>::get_sum() {
  return _sum;
}

template <uint8_t N>
float MovingAverageFloat<N>::add(float value) {
  // fill buffer when using first
  if (_first) {
    _first = false;
    fill(value);

  } else {
    _sum = _sum - _buffer[_next] + value;
    _buffer[_next] = value;
    _next = (_next + 1) & (N - 1);
  }

  _result = _sum / static_cast<float>(N);

  return _result;
}

template <uint8_t N>
void MovingAverageFloat<N>::fill(float value) {
  for (uint16_t i = 0; i < N; i++) {
    _buffer[i] = value;
  }

  _sum = value * static_cast<float>(N);
  _next = 0;
}

template <uint8_t N>
void MovingAverageFloat<N>::reset() {
  _first = true;
}

#endif

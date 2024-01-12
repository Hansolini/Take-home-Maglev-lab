/*
  This sketch shows to smoothen and differentiate a variable in real time.
  NOTE: see the example "alphaBeta-integral" for more details on each function.
*/

#include <AlphaBeta.h>
AlphaBeta filter;

/* input function */
int sensor() {
  return analogRead(A0);
}

void setup() {
  Serial.begin(9600);
  
  // 1. initial state 
  filter.setState( sensor() );  
    
  // 2. initialize time step
  AlphaBeta::updateTimer();
}

void loop() {
  // 3. refresh step
  AlphaBeta::updateTimer();
  
  // 4. update filter
  constexpr float GAIN = 0.8;           
  filter.update( sensor(), GAIN );

  // display output
  Serial.print( filter.smooth() );
  Serial.print( " " );
  Serial.print( filter.deriv() );
  Serial.println();
}

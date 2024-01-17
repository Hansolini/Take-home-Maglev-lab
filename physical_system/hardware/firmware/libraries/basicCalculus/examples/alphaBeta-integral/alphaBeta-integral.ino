/*
  This sketch shows to smoothen, integrate and differentiate a variable in real time
*/

#include <AlphaBetaInt.h>
AlphaBetaInt filter;

/* function with negative values - allows integral to be decreased or increased */
int sensor() {
  return analogRead(A0) - 512;
}

void setup() {
  Serial.begin(9600);
  
  // 1. set initial state of functions
  const float FUNC  = sensor();             // initial value of function
  const float DERIV = 0;                    // initial value of derivative
  const float INTEG = 10;                   // initial value of integral
  
  /* setter functions */
  filter.setState( FUNC, DERIV, INTEG );    // Each variable is optional, defaults to zero
  
  // filter.setInteg( 1 );                  // The value of integral can be set seperately
  
  // 2. initialize time step
  AlphaBeta::updateTimer();                 // All instances of AlphaBeta or AlphaBetaInt use the same timer
}

void loop() {
  // 3. refresh time step
  AlphaBeta::updateTimer();                 // Update the time step for all alpha-beta filters!
  
  // 4. update filter with input
  constexpr float GAIN = 0.8;               // Smoother delay. Must be a value between 0 and 1.
                                            // A value of 1 means there is almost no smoothing.
  float input = sensor();
  filter.update( input, GAIN );

  // display approximate calculus functions
  Serial.print( filter.smooth() );
  Serial.print( " " );
  Serial.print( filter.deriv() );
  Serial.print( " " );
  Serial.print( filter.integ() );
  Serial.println();
}

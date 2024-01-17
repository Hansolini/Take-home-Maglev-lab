#include "AlphaBeta.h"

//---------------------- Timer ----------------------- 

static float AlphaBeta::dt = 0;
static uint32_t AlphaBeta::last_time = 0;

static void AlphaBeta::updateTimer() {  
  uint32_t curr_time = micros();
  dt = float(curr_time - last_time)*1e-6;
  last_time = curr_time;
}

static float AlphaBeta::timeStep() {
  return dt;
}

//---- Alpha-beta filter: smoother and derivative ----

void AlphaBeta::update(float xi, const float GAIN) {  
  // coefficients
  const float ALPHA = GAIN < 0 ? 0 : 
                      GAIN > 1 ? 1 : GAIN; 
  const float BETA = ALPHA * ALPHA * 0.25; 

  // error
  float dx = xi - x;
    
  // smoother
  x += dx_dt*dt + ALPHA*dx;  
    
  // derivative
  dx_dt += BETA * dx/dt; 
}

//--------------------- Setters ----------------------- 

void AlphaBeta::setState(float _x, float _dx_dt) {
  x = _x;
  dx_dt = _dx_dt; 
}

//--------------------- Getters ----------------------- 

float AlphaBeta::smooth() {		// smoothed input value
  return x;
}

float AlphaBeta::deriv() {		// derivative of input
  return dx_dt;
}

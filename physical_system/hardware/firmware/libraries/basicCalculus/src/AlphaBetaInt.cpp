#include "AlphaBetaInt.h"

//--------- smoother, derivative and integral ---------

void AlphaBetaInt::update(float xi, const float ALPHA) {  
  AlphaBeta::update(xi, ALPHA);
  
  // alternative trapezoidal rule
  float dt = AlphaBeta::timeStep();
  float x = smooth();		    	// current x
  float dx = deriv() * dt;			// predict future x
  sxdt += (x + 0.5*dx) * dt;		// triangular area
}

//--------------------- Setters ----------------------- 

void AlphaBetaInt::setInteg(float s) {
  sxdt = s;
}

void AlphaBetaInt::setState(float _x, float _dx_dt, float _sxdt) {
  AlphaBeta::setState(_x, _dx_dt);
  sxdt = _sxdt; 
} 

//--------------------- Getters ----------------------- 

float AlphaBetaInt::integ() {		// integral of input
  return sxdt;
}

#include <Arduino.h>

#ifndef AlphaBeta_h
#define AlphaBeta_h

class AlphaBeta {  
  private:
    static float dt;
    static uint32_t last_time;
    
    float x=0, dx_dt=0;
 
 public: 
 	// timer
 	static void updateTimer();
 	static float timeStep();
 	
 	// setter
 	void setState(float=0, float=0);		// Default to zero state
 	void update(float, const float);	
 	
 	// getter
 	float smooth();
 	float deriv();
}; 

#endif

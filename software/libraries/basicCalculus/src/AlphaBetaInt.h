#include "AlphaBeta.h"

#ifndef AlphaBetaInt_h
#define AlphaBetaInt_h

/* AlphaBeta filter with added integral */

class AlphaBetaInt : public AlphaBeta {  
  private:    
    float sxdt=0;
 
 public: 
 	// setter
 	void setInteg(float);
 	void setState(float=0, float=0, float=0); 	// Default to zero state
 	void update(float, const float);			// Default to GAIN=1
	
	// getter
	float integ();
}; 

#endif

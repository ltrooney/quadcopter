#ifndef PIDController_h
#define PIDController_h

#include "Arduino.h"

class PIDController {
public:
	PIDController();
	PIDController(float, float, float);

	float get_p_gain();
	float get_i_gain();
	float get_d_gain();
    
    float get_p_term();
    float get_i_term();
    float get_d_term();
    
    void set_max_output(float);
	void set_i_threshold(float);		// maximum value before I term resets
	float calculate_output(float, float);
private:
	const float p_gain, i_gain, d_gain;
	float p_output, i_output, d_output;
	float error, error_prior;
	float i_thresh;
    float max_output;
};

#endif

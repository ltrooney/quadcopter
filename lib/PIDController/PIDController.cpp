#include "PIDController.h"

PIDController::PIDController() :
	p_gain(0),
    i_gain(0),
    d_gain(0),
    max_output(300) {}

PIDController::PIDController(float p, float i, float d) :
	p_gain(p),
    i_gain(i),
    d_gain(d),
    max_output(300) {}

void PIDController::set_i_threshold(float thresh) {
    i_thresh = thresh;
}

void PIDController::set_max_output(float max) {
    max_output = max;
}

float PIDController::get_error() {
    return error;
}

float PIDController::get_p_term() {
    return p_output;
}

float PIDController::get_i_term() {
    return i_output;
}

float PIDController::get_d_term() {
    return d_output;
}

float PIDController::calculate_output(float actual, float desired) {
	error = actual - desired;
    
    // calculate p, i, and d terms
    p_output = p_gain * error;
    d_output = d_gain * (error - error_prior);
    if(actual > i_thresh || actual < -i_thresh)
        i_output = 0;
    else
        i_output += i_gain * error;
    error_prior = error;
    
    // combine output
    float output = p_output + i_output + d_output;
    
    return output > max_output ? max_output : output;
}


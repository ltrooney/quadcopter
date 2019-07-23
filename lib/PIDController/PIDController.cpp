#include "PIDController.h"

PIDController::PIDController() : PIDController(0,0,0)    {}

PIDController::PIDController(float p, float i, float d) :
	p_gain(p),
    i_gain(i),
    d_gain(d),
    max_output(300),
    i_thresh(100) {}

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

float PIDController::calculate_output(float err) {
    error = err;
    // calculate p, i, and d terms
    p_output = p_gain * error;
    i_output = disable_i ? 0 : i_output + i_gain * error;
    d_output = first_pass ? 0 : d_gain * (error - error_prior);
    
    first_pass = false;
    
    // anti-windup; prevent I term from getting too big
    if(i_output > i_thresh) {
        i_output = i_thresh;
    } else if(i_output < -i_thresh) {
        i_output = -i_thresh;
    }
    
    error_prior = error;
    
    // combine output
    float output = p_output + d_output;
    output += disable_i ? 0 : i_output;
    
    if(output > max_output) {
        output = max_output;
    } else if(output < -max_output) {
        output = -max_output;
    }
    
    return output;
}

float PIDController::calculate_output(float actual, float desired) {
    calculate_output(actual - desired);
}

void PIDController::disable_i_term(bool val) {
    disable_i = val;
}

void PIDController::reset() {
    p_output = i_output = d_output = 0;
    error_prior = 0;
    first_pass = true;
}

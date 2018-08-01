//
//  LowPassFilter.cpp
//  
//
//  Created by Luke Rooney on 6/8/18.
//

#include "LowPassFilter.h"

LowPassFilter::LowPassFilter(float alpha):
    alpha(alpha) {}

float LowPassFilter::output(float input) {
    float output = alpha*input + (1-alpha)*prior_output;
    prior_output = output;
    return output;
}

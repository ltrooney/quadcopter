//
//  LowPassFilter.hpp
//  
//
//  Created by Luke Rooney on 6/8/18.
//

#ifndef LowPassFilter_hpp
#define LowPassFilter_hpp

#include <Arduino.h>

class LowPassFilter {
public:
    LowPassFilter(float);
    float output(float);
private:
    float alpha, prior_output;
};

#endif /* LowPassFilter_hpp */

#ifndef CYCLEMETER_ROTATION_CALCULATOR_H
#define CYCLEMETER_ROTATION_CALCULATOR_H

#include "average_calculator.h"

class rotation_calculator {

public:
    float rotations_per_unit;

private:
    float max_time_difference = 5000;

    bool first_reading_change;
    bool last_reading;
    float last_time;

    float time_difference;

    average_calculator average_calculator();

public:
    rotation_calculator();
    bool on_reading(bool reading, float time);

private:
    void reset();

};


#endif //CYCLEMETER_ROTATION_CALCULATOR_H

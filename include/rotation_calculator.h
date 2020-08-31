#ifndef CYCLEMETER_ROTATION_CALCULATOR_H
#define CYCLEMETER_ROTATION_CALCULATOR_H

#include "average_calculator.h"

const float MILLISECONDS_IN_SECONDS = 1000;
const int MIN_CHANGES = 3;
const int MAX_TIME_DIFFERENCE = 10000;

class rotation_calculator {

public:
    float rotations_per_second;

private:
    int changes_count;

    bool first_reading_change;
    bool last_reading;
    float last_change_time;

    float time_difference;

    average_calculator rotation_average;

public:
    rotation_calculator(int average_min_readings, int average_max_readings);
    bool on_reading(bool reading, float time);

private:
    void reset();

};


#endif //CYCLEMETER_ROTATION_CALCULATOR_H

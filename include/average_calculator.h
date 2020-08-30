#ifndef CYCLEMETER_AVERAGE_CALCULATOR_H
#define CYCLEMETER_AVERAGE_CALCULATOR_H

const int MIN_READINGS = 3;
const int MAX_READINGS = 10;

class average_calculator {

public:
    float average;

private:
    int readings[MAX_READINGS];

    int next_reading_index;
    float total;
    float number_of_readings;
    float last_time;

public:
    average_calculator();
    bool on_reading(float reading, float time);

private:
    void reset();

};


#endif //CYCLEMETER_AVERAGE_CALCULATOR_H

#ifndef CYCLEMETER_AVERAGE_CALCULATOR_H
#define CYCLEMETER_AVERAGE_CALCULATOR_H

const int MAX_READINGS = 10;

class average_calculator {

public:
    float average;

private:
    float readings[MAX_READINGS];
    int min_readings;
    int max_readings;

    int next_reading_index;
    float total;
    float number_of_readings;
    float last_time;

public:
    average_calculator(int min_readings, int max_readings);
    bool on_reading(float reading, float time);
    void reset();

private:

};


#endif //CYCLEMETER_AVERAGE_CALCULATOR_H

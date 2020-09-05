#ifndef CYCLEMETER_AVERAGE_CALCULATOR_H
#define CYCLEMETER_AVERAGE_CALCULATOR_H

const int MAX_READINGS = 10;

class average_calculator {

public:
    float average;

private:
    float readings[MAX_READINGS];
    int aggregate_readings_count;
    int min_readings;
    int max_readings;

    int aggregate_count;
    float aggregate_total;
    int next_reading_index;
    float total;
    long number_of_readings;
    long last_time;

public:
    average_calculator(int aggregate_readings_count, int min_readings, int max_readings);
    bool on_reading(float reading, long time);
    void reset();

private:

};


#endif //CYCLEMETER_AVERAGE_CALCULATOR_H

#ifndef CYCLEMETER_AVERAGE_CALCULATOR_H
#define CYCLEMETER_AVERAGE_CALCULATOR_H

class average_calculator {

private:
    const int max_readings = 10;
    int readings[10];

    int next_reading_index;
    float total;
    float last_time;

public:
    average_calculator();
    void on_reading(float reading, float time);

private:
    void reset();

};


#endif //CYCLEMETER_AVERAGE_CALCULATOR_H

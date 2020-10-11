#ifndef CYCLE_METER_AVERAGE_CALCULATOR_H
#define CYCLE_METER_AVERAGE_CALCULATOR_H

#include <stdbool.h>
#include <stdint.h>

#define MAX_READINGS 10

typedef struct
{
    float average;

    // TODO: these are read only?
    uint16_t _aggregate_readings_count;
    uint16_t _min_readings;
    uint16_t _max_readings;

    float _readings[MAX_READINGS];

    uint16_t _aggregate_count;
    float _aggregate_total;
    uint16_t _next_reading_index;
    float _total;
    uint16_t _number_of_readings;

} average_calculator_t;


void average_calculator_init(average_calculator_t * average_calculator);
bool average_calculator_on_reading(average_calculator_t * average_calculator, float reading);
void average_calculator_reset(average_calculator_t * average_calculator);

#endif
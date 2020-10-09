#ifndef AVERAGE_CALCULATOR_H
#define AVERAGE_CALCULATOR_H

#include <stdio.h>
#include <math.h>

#define MAX_READINGS 10

typedef struct
{
    float average;

    float readings[MAX_READINGS];
    uint16_t aggregate_readings_count;
    uint16_t min_readings;
    uint16_t max_readings;

    uint16_t aggregate_count;
    float aggregate_total;
    uint16_t next_reading_index;
    float total;
    uint16_t number_of_readings;

} average_calculator_t;

// TODO: need to validate struct data, some should be private?

bool on_reading(average_calculator_t * average_calculator, float reading) {
    
    if (average_calculator->aggregate_readings_count != 1) {
        average_calculator->aggregate_total += reading;
        average_calculator->aggregate_count++;

        if (average_calculator->aggregate_count == average_calculator->aggregate_readings_count) {
            reading = average_calculator->aggregate_total / average_calculator->aggregate_count;
            average_calculator->aggregate_total = 0;
            average_calculator->aggregate_count = 0;
        } else {
            return false;
        }
    }
    

    average_calculator->total = average_calculator->total - average_calculator->readings[average_calculator->next_reading_index];
    average_calculator->readings[average_calculator->next_reading_index] = reading;
    average_calculator->total += reading;

    average_calculator->next_reading_index++;
    if (average_calculator->next_reading_index >= average_calculator->max_readings)
    {
        average_calculator->next_reading_index = 0;
    }

    if (average_calculator->number_of_readings < average_calculator->max_readings) {
        average_calculator->number_of_readings++;
    }

    if (average_calculator->number_of_readings < average_calculator->min_readings) {
        return false;
    }

    average_calculator->average = average_calculator->total / average_calculator->number_of_readings;

    return true;
}

void reset(average_calculator_t * average_calculator) {
    average_calculator->aggregate_total = 0;
    average_calculator->aggregate_count = 0;
    average_calculator->total = 0;

    for (average_calculator->next_reading_index = 0; average_calculator->next_reading_index < MAX_READINGS; average_calculator->next_reading_index++) {
        average_calculator->readings[average_calculator->next_reading_index] = 0;
    }

    average_calculator->next_reading_index = 0;
}

#endif //AVERAGE_CALCULATOR_H
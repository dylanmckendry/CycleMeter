#ifndef CYCLE_METER_ROTATION_CALCULATOR_H
#define CYCLE_METER_ROTATION_CALCULATOR_H

#include <stdbool.h>
#include <stdint.h>

#include "average_calculator.h"

#define MIN_CHANGES 3

typedef struct
{
    float rotations_per_second;

    uint32_t changes_count;

    bool first_reading_change;
    bool last_reading;
    uint32_t last_change_time;

    uint32_t time_difference;

    average_calculator_t rotation_average;

} rotation_calculator_t;

bool rotation_calculator_init(rotation_calculator_t * rotation_calculator);
bool rotation_calculator_on_reading(rotation_calculator_t * rotation_calculator, bool reading, long time);
void rotation_calculator_reset(rotation_calculator_t * rotation_calculator);

#endif
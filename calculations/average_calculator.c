#include "average_calculator.h"

// TODO: need to validate struct data, some should be private?

void average_calculator_init(average_calculator_t * average_calculator)
{

}

bool average_calculator_on_reading(average_calculator_t * average_calculator, float reading)
{
    if (average_calculator->_aggregate_readings_count != 1)
    {
        average_calculator->_aggregate_total += reading;
        average_calculator->_aggregate_count++;

        if (average_calculator->_aggregate_count == average_calculator->_aggregate_readings_count)
        {
            reading = average_calculator->_aggregate_total / average_calculator->_aggregate_count;
            average_calculator->_aggregate_total = 0;
            average_calculator->_aggregate_count = 0;
        }
        else
        {
            return false;
        }
    }
    

    average_calculator->_total = average_calculator->_total - average_calculator->_readings[average_calculator->_next_reading_index];
    average_calculator->_readings[average_calculator->_next_reading_index] = reading;
    average_calculator->_total += reading;

    average_calculator->_next_reading_index++;
    if (average_calculator->_next_reading_index >= average_calculator->_max_readings)
    {
        average_calculator->_next_reading_index = 0;
    }

    if (average_calculator->_number_of_readings < average_calculator->_max_readings)
    {
        average_calculator->_number_of_readings++;
    }

    if (average_calculator->_number_of_readings < average_calculator->_min_readings)
    {
        return false;
    }

    average_calculator->average = average_calculator->_total / average_calculator->_number_of_readings;

    return true;
}

void average_calculator_reset(average_calculator_t * average_calculator)
{
    average_calculator->average = 0;
    average_calculator->_aggregate_total = 0;
    average_calculator->_aggregate_count = 0;
    average_calculator->_total = 0;

    for (average_calculator->_next_reading_index = 0; average_calculator->_next_reading_index < MAX_READINGS; average_calculator->_next_reading_index++)
    {
        average_calculator->_readings[average_calculator->_next_reading_index] = 0;
    }

    average_calculator->_next_reading_index = 0;
}
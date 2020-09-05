#include "average_calculator.h"

average_calculator::average_calculator(int aggregate_readings_count, int min_readings, int max_readings) :
    aggregate_readings_count(aggregate_readings_count),
    min_readings(min_readings),
    max_readings(max_readings) {
    if (max_readings > MAX_READINGS) {
        // crash
    }
    reset();
}

bool average_calculator::on_reading(float reading, long time) {
    
    if (aggregate_readings_count != 1) {
        aggregate_total += reading;
        aggregate_count++;

        if (aggregate_count == aggregate_readings_count) {
            reading = aggregate_total / aggregate_count;
            aggregate_total = 0;
            aggregate_count = 0;
        } else {
            return false;
        }
    }
    

    total = total - readings[next_reading_index];
    readings[next_reading_index] = reading;
    total += reading;

    next_reading_index++;
    if (next_reading_index >= max_readings)
    {
        next_reading_index = 0;
    }

    if (number_of_readings < max_readings) {
        number_of_readings++;
    }

    if (number_of_readings < min_readings) {
        return false;
    }

    average = total / number_of_readings;

    return true;
}

void average_calculator::reset() {
    aggregate_total = 0;
    aggregate_count = 0;
    total = 0;
    last_time = 0;

    for (next_reading_index = 0; next_reading_index < MAX_READINGS; next_reading_index++) {
        readings[next_reading_index] = 0;
    }

    next_reading_index = 0;
}
#include "average_calculator.h"

average_calculator::average_calculator() {
    reset();
}

bool average_calculator::on_reading(float reading, float time) {
    total = total - readings[next_reading_index];
    readings[next_reading_index] = reading;
    total += reading;

    next_reading_index++;
    if (next_reading_index >= MAX_READINGS)
    {
        next_reading_index = 0;
    }

    if (number_of_readings < MAX_READINGS) {
        number_of_readings++;
    }

    if (number_of_readings < MIN_READINGS) {
        return false;
    }

    average = total / number_of_readings;

    return true;
}

void average_calculator::reset() {
    total = 0;
    last_time = 0;

    for (next_reading_index = 0; next_reading_index < MAX_READINGS; next_reading_index++) {
        readings[next_reading_index] = 0;
    }

    next_reading_index = 0;
}
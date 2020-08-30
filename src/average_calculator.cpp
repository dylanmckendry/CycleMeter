#include "average_calculator.h"

average_calculator::average_calculator() {
    reset();
}

void average_calculator::on_reading(float reading, float time) {
    total = total - readings[next_reading_index];
    readings[next_reading_index] = reading;
    total += reading;

    next_reading_index++;
    if (next_reading_index >= max_readings)
    {
        next_reading_index = 0;
    }

    //float average = total / numReadings;
}

void average_calculator::reset() {
    total = 0;
    last_time = 0;

    for (next_reading_index = 0; next_reading_index < max_readings; next_reading_index++) {
        readings[next_reading_index] = 0;
    }

    next_reading_index = 0;
}
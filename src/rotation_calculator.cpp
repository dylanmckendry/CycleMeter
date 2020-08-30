#include "rotation_calculator.h"

rotation_calculator::rotation_calculator() {
    reset();
}

bool rotation_calculator::on_reading(bool reading, float time) {
    if (last_reading == reading)
    {
        return false;
    }

    // after reset
    if (last_time == 0)
    {
        // first real reading
        // TODO: reset average
        last_reading = reading;
        last_time = time;
        return false;
    }

    time_difference = time - last_time;

    // been too long, reset
    if (time_difference > max_time_difference) {
        reset();
        return false;
    }

    // this means it was true before and therefore the last true was a full circuit
    if (reading == false)
    {
        // add to average
        last_reading = reading;
        last_time = time;

        // ignore the first change
        if (first_reading_change) {
            first_reading_change = false;
            return false;
        }

        // add to average;
    }

    // if we have enough averages return true
    rotations_per_unit = time_difference;

    return true;
}

void rotation_calculator::reset() {
    first_reading_change = true;
    last_reading = false;
    last_time = 0;
}


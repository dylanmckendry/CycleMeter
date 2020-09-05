#include "rotation_calculator.h"

rotation_calculator::rotation_calculator(int average_min_readings, int average_max_readings) :
    rotation_average(1, average_min_readings, average_min_readings) {
    reset();
}

bool rotation_calculator::on_reading(bool reading, long time) {
    
    // it hasn't changed
    if (last_reading == reading) {
        return false;
    }

    last_reading = reading;

    // a change is true -> false
    if (reading == false) {
        if (last_change_time == 0) {
            last_change_time = time;
        } else {
            time_difference = time - last_change_time;
            last_change_time = time;
        }
    }

    // been too long, reset
    if (time_difference > MAX_TIME_DIFFERENCE) {
        reset();
        return false;
    }

    // after reset
    // first real reading
    // TODO: reset average
    if (changes_count < MIN_CHANGES) {
        changes_count++;
        return false;
    }

    if (reading == true) {
        return false;
    }

    // this means it was true before and therefore the last true was a full circuit
    // if we have enough averages return true
    if (!rotation_average.on_reading(MICROSECONDS_IN_SECONDS / time_difference, time)) {
        return false;
    }

    rotations_per_second = rotation_average.average;
    return true;
}

void rotation_calculator::reset() {
    last_change_time = 0;
    changes_count = 0;
    time_difference = 0;
    last_reading = false;
    rotation_average.reset();
}


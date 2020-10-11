#ifndef CYCLE_METER_CALCULATIONS_H
#define CYCLE_METER_CALCULATIONS_H

#include <math.h>

#include "calculations_constants.h"

float degrees_to_radians(float degrees)
{
    return degrees * (M_PI / 180);
}

float radians_to_degrees(float radians)
{
    return radians / (M_PI / 180);
}

float calculate_slope_slope_degrees(float slope_degrees)
{
    return tan(degrees_to_radians(slope_degrees));
}

float calculate_slope_ratio(float slope_degrees)
{
    return tan(degrees_to_radians(slope_degrees));
}

float calculate_cadence(float crank_rotations_second)
{
    return crank_rotations_second * 60;
}

float calculate_ground_velocity(float wheel_rotations_second, float wheel_radius)
{
    return wheel_rotations_second * wheel_radius * 2 * M_PI;
}

float calculate_vertical_velocity(float slope_degrees, float ground_velocity)
{
    return sin(degrees_to_radians(slope_degrees)) * ground_velocity;
}

// TODO: should this be abs or just go to 0
float calculate_dynamic_pressure(float stagnation_pressure, float static_pressure, float sensor_pressure_correction)
{
    return abs((stagnation_pressure - static_pressure) + sensor_pressure_correction);
}

float calculate_air_density(float static_pressure, float air_temperature)
{
    return static_pressure / (DRY_AIR_CONSTANT * (air_temperature + CELSIUS_TO_KELVIN));
}

float calculate_air_drag(float air_density, float air_velocity)
{
    return .5f * COEF_FRICTION * air_density * pow(air_velocity, 2) * FRONTAL_AREA;
}

// power
float calculate_air_velocity(float dynamic_pressure, float air_density)
{
    return sqrt((2 * dynamic_pressure) / air_density);
}

float calculate_gravity_power(float total_weight, float vertical_velocity)
{
    return total_weight * vertical_velocity;
}

float calculate_inertia_power(float total_weight, float acceleration, float ground_velocity)
{
    return total_weight * acceleration * ground_velocity;
}

float calculate_air_drag_power(float air_drag, float ground_velocity)
{
    return air_drag * ground_velocity;
}

float calculate_tire_resistance_power(float total_weight, float slope_ratio, float ground_velocity)
{
    return total_weight * sqrt((1 - pow(slope_ratio, 2))) * TIRE_RESISTANCE_CRR * ground_velocity;
}

#endif
#ifndef CYCLE_METER_ROTATION_SENSOR_H
#define CYCLE_METER_ROTATION_SENSOR_H

#include <stdint.h>

#include "nrf_drv_gpiote.h"

typedef struct
{
    nrf_drv_gpiote_pin_t pin;

} speed_sensor_t;

typedef struct
{
    nrf_drv_gpiote_pin_t pin;

} cadence_sensor_t;

void rotation_sensor_init(speed_sensor_t * speed_sensor, cadence_sensor_t * cadence_sensor);

#endif
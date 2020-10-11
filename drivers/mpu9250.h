#ifndef CYCLE_METER_MPU9250_H
#define CYCLE_METER_MPU9250_H

#include <stdint.h>

#include "nrf_drv_twi.h"

typedef struct
{
    uint8_t address;

    uint8_t raw_int_status;

    float gyro_res;
    float accel_res;

    uint8_t raw_accelerometer[6];
    int16_t accelerometer[3];
    float processed_accelerometer[3];

    uint8_t raw_gyroscope[6];
    int16_t gyroscope[3];
    float processed_gyroscope[3];
} mpu9250_t;

void mpu9250_self_test(nrf_drv_twi_t const * twi_driver, mpu9250_t * mpu9250);
void mpu9250_init(nrf_drv_twi_t const * twi_driver, mpu9250_t * mpu9250);
void mpu9250_clear_int_status(nrf_drv_twi_t const * twi_driver, mpu9250_t * mpu9250);
void mpu9250_read_accelerometer(nrf_drv_twi_t const * twi_driver, mpu9250_t * mpu9250);
void mpu9250_read_gyroscope(nrf_drv_twi_t const * twi_driver, mpu9250_t * mpu9250);

#endif //MPU9250_H
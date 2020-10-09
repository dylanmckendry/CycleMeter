#ifndef TWI_COMMON_H
#define TWI_COMMON_H

#include <stdint.h>

#include "nrf_drv_twi.h"

void twi_init(nrf_drv_twi_t * twi_driver, uint32_t scl_pin, uint32_t sda_pin, nrf_drv_twi_evt_handler_t event_handler);
void write_register(nrf_drv_twi_t const * twi_driver, uint8_t device_addresss, uint8_t register_address, uint8_t register_data);
void read_register(nrf_drv_twi_t const * twi_driver, uint8_t device_addresss, uint8_t register_address, uint8_t * return_data, uint8_t return_data_count);

#endif //TWI_COMMON_H
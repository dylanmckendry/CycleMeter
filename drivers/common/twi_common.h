#ifndef TWI_COMMON_H
#define TWI_COMMON_H

#include "app_error.h"
#include "nrf_drv_twi.h"

ret_code_t err_code;

void write_register(nrf_drv_twi_t const * twi_driver, uint8_t device_addresss, uint8_t register_address, uint8_t register_data)
{
    uint8_t reg[2] = {register_address, register_data};
    err_code = nrf_drv_twi_tx(twi_driver, device_addresss, reg, sizeof(reg), false);
    APP_ERROR_CHECK(err_code);
}

void read_register(nrf_drv_twi_t const * twi_driver, uint8_t device_addresss, uint8_t register_address, uint8_t * return_data, uint8_t return_data_count)
{
    err_code = nrf_drv_twi_tx(twi_driver, device_addresss, &register_address, 1, true);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_twi_rx(twi_driver, device_addresss, return_data, return_data_count);
    APP_ERROR_CHECK(err_code);
}


#endif //TWI_COMMON_H
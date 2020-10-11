#include "twi_common.h"

#include "app_error.h"

ret_code_t err_code;

void twi_init(nrf_drv_twi_t * twi_driver, uint32_t scl_pin, uint32_t sda_pin, nrf_drv_twi_evt_handler_t event_handler)
{
    const nrf_drv_twi_config_t twi_config = {
       .scl = scl_pin,
       .sda = sda_pin,
       .frequency = NRF_DRV_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init = false
    };

    err_code = nrf_drv_twi_init(twi_driver, &twi_config, event_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(twi_driver);
}

void write_register(nrf_drv_twi_t const * twi_driver, uint8_t device_addresss, uint8_t register_address, uint8_t register_data)
{
    uint8_t reg[2] = { register_address, register_data };
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
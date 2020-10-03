#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "drivers/mpu9250/mpu9250.h"

#define ARDUINO_SCL_PIN             20    // SCL signal pin
#define ARDUINO_SDA_PIN             21    // SDA signal pin

/* TWI instance ID. */
#if TWI0_ENABLED
#define TWI_INSTANCE_ID     0
#elif TWI1_ENABLED
#define TWI_INSTANCE_ID     1
#endif


/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

static uint8_t m_sample;

/**
 * @brief TWI initialization.
 */
void twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_config = {
       .scl                = ARDUINO_SCL_PIN,
       .sda                = ARDUINO_SDA_PIN,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}

#define MPU9250_ADDR 0x68
#define MPU9250_RA_PWR_MGMT_1 0x6B
#define MPU9250_PWR1_DEVICE_RESET_BIT 7

/**
 * @brief Function for main application entry.
 */
int main(void)
{
    ret_code_t err_code;
    uint8_t address;
    uint8_t sample_data;
    bool detected_device = false;

    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("MPU9250 started.");
    NRF_LOG_FLUSH();
    twi_init();

    uint8_t reg[2] = {MPU9250_RA_PWR_MGMT_1, MPU9250_PWR1_DEVICE_RESET_BIT};
    err_code = nrf_drv_twi_tx(&m_twi, MPU9250_ADDR, reg, sizeof(reg), false);
    APP_ERROR_CHECK(err_code);

    //err_code = nrf_drv_twi_rx(&m_twi, MPU9250_ADDR, &m_sample, sizeof(m_sample));
    //APP_ERROR_CHECK(err_code);

    //NRF_LOG_INFO(m_sample);

    while (true)
    {
        /* Empty loop. */
    }
}
#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "drivers/mpu9250/mpu9250.h"
#include "drivers/common/twi_common.h"

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

    write_register(&m_twi, MPU9250_ADDR, PWR_MGMNT_1, CLOCK_SEL_PLL);
    //write(PWR_MGMNT_1, PWR_RESET);

    //err_code = nrf_drv_twi_rx(&m_twi, MPU9250_ADDR, &m_sample, sizeof(m_sample));
    //APP_ERROR_CHECK(err_code);

    //NRF_LOG_INFO(m_sample);

    NRF_LOG_INFO("MPU9250 ended.");
    NRF_LOG_FLUSH();

    nrf_delay_ms(500);

    //read();

    //while (true)
    //{
    //    /* Empty loop. */
    //}
}
#include <stdio.h>
#include "nrf.h"
#include "bsp.h"
#include "hardfault.h"
#include "app_error.h"
#include "app_timer.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_gpiote.h"
#include "nrf_gpio.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ant.h"
#include "nrf_drv_gpiote.h"
#include "ant_key_manager.h"
#include "ant_bpwr.h"
#include "ant_bpwr_simulator.h"
#include "ant_state_indicator.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define WHEEL_SENSOR_PIN 20
#define CRANK_SENSOR_PIN 21

void wheel_sensor_event_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    if (action == NRF_GPIOTE_POLARITY_LOTOHI && pin == WHEEL_SENSOR_PIN)
    {
        char string_on_stack[] = "stack";
        NRF_LOG_INFO("%s",string_on_stack);
    }    
}

static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

static void utils_setup(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

static void gpio_init()
{
    nrfx_err_t err_code;

    if(!nrf_drv_gpiote_is_init())
    {
        err_code = nrf_drv_gpiote_init();
        APP_ERROR_CHECK(err_code);
    }

    nrf_drv_gpiote_in_config_t config = NRFX_GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
    //config.pull = NRF_GPIO_PIN_PULLUP;
    nrf_drv_gpiote_in_init(WHEEL_SENSOR_PIN, &config, wheel_sensor_event_handler);
    nrf_drv_gpiote_in_event_enable(WHEEL_SENSOR_PIN, true);
}

/**@brief Function for application main entry, does not return.
 */
int main(void)
{
    log_init();
    gpio_init();

    while (true) { }

    //for(;;) __WFE();
}

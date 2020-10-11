#include "rotation_sensor.h"

#include "hardfault.h"
#include "app_error.h"
#include "app_timer.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_gpiote.h"
#include "nrf_gpio.h"
#include "nrf_sdh.h"
#include "nrf_drv_gpiote.h"

speed_sensor_t * _speed_sensor;
cadence_sensor_t * _cadence_sensor;

void rotation_sensor_event_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);

void speed_sensor_init(speed_sensor_t * speed_sensor, cadence_sensor_t * cadence_sensor)
{
    _speed_sensor = speed_sensor;
    _cadence_sensor = cadence_sensor;

    nrf_drv_gpiote_in_config_t wheel_sensor_config = NRFX_GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
    nrf_drv_gpiote_in_init(speed_sensor->pin, &wheel_sensor_config, rotation_sensor_event_handler);
    nrf_drv_gpiote_in_event_enable(speed_sensor->pin, true);

    nrf_drv_gpiote_in_config_t crank_sensor_config = NRFX_GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
    nrf_drv_gpiote_in_init(cadence_sensor->pin, &crank_sensor_config, rotation_sensor_event_handler);
    nrf_drv_gpiote_in_event_enable(cadence_sensor->pin, true);
    //config.pull = NRF_GPIO_PIN_PULLUP;
}

void rotation_sensor_event_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    if (action == NRF_GPIOTE_POLARITY_LOTOHI)
    {
        if (pin = _speed_sensor->pin)
        {

        }
        else if (pin = _cadence_sensor->pin)
        {

        }
    }
}
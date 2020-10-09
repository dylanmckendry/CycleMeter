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

void ant_bpwr_evt_handler(ant_bpwr_profile_t * p_profile, ant_bpwr_evt_t event);
void ant_bpwr_calib_handler(ant_bpwr_profile_t * p_profile, ant_bpwr_page1_data_t * p_page1);

BPWR_SENS_CHANNEL_CONFIG_DEF(m_ant_bpwr,
                             BPWR_CHANNEL_NUM,
                             CHAN_ID_TRANS_TYPE,
                             CHAN_ID_DEV_NUM,
                             ANTPLUS_NETWORK_NUM);
BPWR_SENS_PROFILE_CONFIG_DEF(m_ant_bpwr,
                             (ant_bpwr_torque_t)(SENSOR_TYPE),
                             ant_bpwr_calib_handler,
                             ant_bpwr_evt_handler);

static ant_bpwr_profile_t m_ant_bpwr;


NRF_SDH_ANT_OBSERVER(m_ant_observer, ANT_BPWR_ANT_OBSERVER_PRIO,
                     ant_bpwr_sens_evt_handler, &m_ant_bpwr);

void ant_bpwr_evt_handler(ant_bpwr_profile_t * p_profile, ant_bpwr_evt_t event)
{
    nrf_pwr_mgmt_feed();

    switch (event)
    {
        case ANT_BPWR_PAGE_1_UPDATED:
            /* fall through */
        case ANT_BPWR_PAGE_16_UPDATED:
            /* fall through */
        case ANT_BPWR_PAGE_17_UPDATED:
            /* fall through */
        case ANT_BPWR_PAGE_18_UPDATED:
            /* fall through */
        case ANT_BPWR_PAGE_80_UPDATED:
            /* fall through */
        case ANT_BPWR_PAGE_81_UPDATED:
            p_profile->BPWR_PROFILE_instantaneous_power = 10;
            //ant_bpwr_simulator_one_iteration(&m_ant_bpwr_simulator, event);
            break;

        default:
            break;
    }
}

void ant_bpwr_calib_handler(ant_bpwr_profile_t * p_profile, ant_bpwr_page1_data_t * p_page1)
{
    switch (p_page1->calibration_id)
    {
        case ANT_BPWR_CALIB_ID_MANUAL:
            m_ant_bpwr.BPWR_PROFILE_calibration_id     = ANT_BPWR_CALIB_ID_MANUAL_SUCCESS;
            m_ant_bpwr.BPWR_PROFILE_general_calib_data = CALIBRATION_DATA;
            break;

        case ANT_BPWR_CALIB_ID_AUTO:
            m_ant_bpwr.BPWR_PROFILE_calibration_id     = ANT_BPWR_CALIB_ID_MANUAL_SUCCESS;
            m_ant_bpwr.BPWR_PROFILE_auto_zero_status   = p_page1->auto_zero_status;
            m_ant_bpwr.BPWR_PROFILE_general_calib_data = CALIBRATION_DATA;
            break;

        case ANT_BPWR_CALIB_ID_CUSTOM_REQ:
            m_ant_bpwr.BPWR_PROFILE_calibration_id = ANT_BPWR_CALIB_ID_CUSTOM_REQ_SUCCESS;
            memcpy(m_ant_bpwr.BPWR_PROFILE_custom_calib_data, p_page1->data.custom_calib,
                   sizeof (m_ant_bpwr.BPWR_PROFILE_custom_calib_data));
            break;

        case ANT_BPWR_CALIB_ID_CUSTOM_UPDATE:
            m_ant_bpwr.BPWR_PROFILE_calibration_id = ANT_BPWR_CALIB_ID_CUSTOM_UPDATE_SUCCESS;
            memcpy(m_ant_bpwr.BPWR_PROFILE_custom_calib_data, p_page1->data.custom_calib,
                   sizeof (m_ant_bpwr.BPWR_PROFILE_custom_calib_data));
            break;

        default:
            break;
    }
}

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

    err_code = ant_state_indicator_init(m_ant_bpwr.channel_number, BPWR_SENS_CHANNEL_TYPE);
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

static void softdevice_setup(void)
{
    ret_code_t err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    ASSERT(nrf_sdh_is_enabled());

    err_code = nrf_sdh_ant_enable();
    APP_ERROR_CHECK(err_code);

    err_code = ant_plus_key_set(ANTPLUS_NETWORK_NUM);
    APP_ERROR_CHECK(err_code);
}

static void profile_setup(void)
{
    ret_code_t err_code;

    err_code = ant_bpwr_sens_init(&m_ant_bpwr,
                                  BPWR_SENS_CHANNEL_CONFIG(m_ant_bpwr),
                                  BPWR_SENS_PROFILE_CONFIG(m_ant_bpwr));
    APP_ERROR_CHECK(err_code);

    // fill manufacturer's common data page.
    m_ant_bpwr.page_80 = ANT_COMMON_page80(BPWR_HW_REVISION,
                                           BPWR_MANUFACTURER_ID,
                                           BPWR_MODEL_NUMBER);
    // fill product's common data page.
    m_ant_bpwr.page_81 = ANT_COMMON_page81(BPWR_SW_REVISION_MAJOR,
                                           BPWR_SW_REVISION_MINOR,
                                           BPWR_SERIAL_NUMBER);

    m_ant_bpwr.BPWR_PROFILE_auto_zero_status = ANT_BPWR_AUTO_ZERO_OFF;

    err_code = ant_bpwr_sens_open(&m_ant_bpwr);
    APP_ERROR_CHECK(err_code);

    err_code = ant_state_indicator_channel_opened();
    APP_ERROR_CHECK(err_code);
}



/**@brief Function for application main entry, does not return.
 */
int main(void)
{
    log_init();
    gpio_init();
    utils_setup();
    softdevice_setup();
    profile_setup();

    NRF_LOG_INFO("ANT+ Bicycle Power TX example started.");

    for (;;)
    {
        NRF_LOG_FLUSH();
        nrf_pwr_mgmt_run();
    }
}

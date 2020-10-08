#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "app_timer.h"
#include "nrf_gpiote.h"
#include "nrf_gpio.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ant.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "drivers/mpu9250/mpu9250.h"
#include "drivers/common/twi_common.h"

#define ARDUINO_SCL_PIN             20    // SCL signal pin
#define ARDUINO_SDA_PIN             21    // SDA signal pin

#define MPU9250_ADDR 0x68
#define MPU9250_INT_PIN 3

#define TIMER_TIMEOUT APP_TIMER_TICKS(2000)

//static app_timer_id_t my_timer_id; 
uint32_t timer_start;
uint32_t timer_end;
uint32_t timer_difference;

APP_TIMER_DEF(timer_id);

/* TWI instance ID. */
#if TWI0_ENABLED
#define TWI_INSTANCE_ID     0
#elif TWI1_ENABLED
#define TWI_INSTANCE_ID     1
#endif


/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

static uint8_t m_sample;


void mpu9250_int_event_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    if (action == NRF_GPIOTE_POLARITY_LOTOHI && pin == MPU9250_INT_PIN)
    {
        mpu9250_read_accelerometer(&m_twi, &mpu9250);
        mpu9250_read_gyroscope(&m_twi, &mpu9250);
        mpu9250_clear_int_status(&m_twi, &mpu9250); // TODO: could clear on any data read?
    }
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
    nrf_drv_gpiote_in_init(MPU9250_INT_PIN, &config, mpu9250_int_event_handler);
    nrf_drv_gpiote_in_event_enable(MPU9250_INT_PIN, true);
}

void twi_init(void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_config = {
       .scl                = ARDUINO_SCL_PIN,
       .sda                = ARDUINO_SDA_PIN,
       .frequency          = NRF_DRV_TWI_FREQ_100K, // TODO: 400K
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}

static void softdevice_setup(void)
{
    ret_code_t err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    ASSERT(nrf_sdh_is_enabled());

    //err_code = nrf_sdh_ant_enable();
    //APP_ERROR_CHECK(err_code);

    //err_code = ant_plus_key_set(ANTPLUS_NETWORK_NUM);
    //APP_ERROR_CHECK(err_code);
}

static void timer_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    timer_end = app_timer_cnt_get();
    timer_difference = app_timer_cnt_diff_compute(timer_end, timer_start);
    NRF_LOG_INFO("MPU9250 timer start %d.", timer_start);
    NRF_LOG_INFO("MPU9250 timer end %d.", timer_end);
    NRF_LOG_INFO("MPU9250 timer difference %d.", timer_difference * 0.0305175);
    timer_start = app_timer_cnt_get();
    //SEGGER_RTT_printf(0, "timer timed out");
}

static void timers_init(void)
{
    uint32_t err_code;

    // Initialize timer module.
    app_timer_init();

    // Create timers.
    err_code = app_timer_create(&timer_id, APP_TIMER_MODE_REPEATED, timer_handler);

    APP_ERROR_CHECK(err_code);
}

static void timers_start(void)
{
    uint32_t err_code;

    // Start application timers.
    err_code = app_timer_start(timer_id, TIMER_TIMEOUT, NULL);
    APP_ERROR_CHECK(err_code);
}

// TODO: need to look at RTC and power example
int main(void)
{
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("MPU9250 started.");
    NRF_LOG_FLUSH();

    mpu9250 mpu9250;
    mpu9250.address = MPU9250_ADDR;

    int16_t temperature;

    //twi_init();
    //gpio_init();
    timers_init();
    softdevice_setup();
    //mpu9250_init(&m_twi, &mpu9250);

    
    timers_start();
    
    timer_start = app_timer_cnt_get();
    //nrf_delay_ms(1000);
    //timer_end = app_timer_cnt_get();
    //timer_difference = app_timer_cnt_diff_compute(timer_end, timer_start);
    //NRF_LOG_INFO("MPU9250 timer start %d.", timer_start);
    //NRF_LOG_INFO("MPU9250 timer end %d.", timer_end);
    //NRF_LOG_INFO("MPU9250 timer difference %d.", timer_difference);
    //app_timer_stop(timer_id);


    //uint8_t int_status;
    //read_register(&m_twi, mpu9250.address, MPU9250_INT_STATUS, &int_status, 1);
    //NRF_LOG_INFO("MPU9250 int status %d.", int_status);
    //NRF_LOG_FLUSH();
    //nrf_delay_ms(1000);

    //read_register(&m_twi, mpu9250.address, MPU9250_INT_STATUS, &int_status, 1);
    //NRF_LOG_INFO("MPU9250 int status %d.", int_status);
    //NRF_LOG_FLUSH();
    //nrf_delay_ms(1000);
    //temperature = mpu9250_read_temperature(&m_twi, &mpu9250);
    //mpu9250_read_accelerometer(&m_twi, &mpu9250, mpu9250_accel);
    //mpu9250_read_gyroscope(&m_twi, &mpu9250, mpu9250_gyro);

    //NRF_LOG_INFO("MPU9250 temperature %d.", temperature);
    //NRF_LOG_INFO("MPU9250 accelerometer x %d.", mpu9250_accel[0]);
    //NRF_LOG_INFO("MPU9250 accelerometer y %d.", mpu9250_accel[1]);
    //NRF_LOG_INFO("MPU9250 accelerometer z %d.", mpu9250_accel[2]);
    //NRF_LOG_INFO("MPU9250 gyroscope x %d.", mpu9250_gyro[0]);
    //NRF_LOG_INFO("MPU9250 gyroscope y %d.", mpu9250_gyro[1]);
    //NRF_LOG_INFO("MPU9250 gyroscope z %d.", mpu9250_gyro[2]);
    NRF_LOG_INFO("MPU9250 ended.");
    NRF_LOG_FLUSH();

    //while (true)
    //{
    //    nrf_delay_ms(100);
    //    NRF_LOG_INFO("MPU9250 timer %d.", timer_end);
    //    NRF_LOG_FLUSH();
    //}

    //while (true) { }

    for(;;) 
    {
        NRF_LOG_FLUSH();
        __WFE();
    }
} 
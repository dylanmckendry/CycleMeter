#include <stdio.h>
#include <math.h>

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

#include "madgwick.h"
#include "average_calculator.h"
#include "mpu9250.h"
#include "twi_common.h"

#define ARDUINO_SCL_PIN             20    // SCL signal pin
#define ARDUINO_SDA_PIN             21    // SDA signal pin

#define MPU9250_ADDR 0x68
#define MPU9250_INT_PIN 3

#define TIMER_TIMEOUT APP_TIMER_TICKS(2000)

mpu9250_t mpu9250;
uint32_t loop_start;
uint32_t imu_update_last;
uint32_t to_euler_angles_last;

average_calculator_t average_calculator

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

float degrees_to_radians(float degrees) {
    return degrees * (M_PI / 180.0f);
}

float radians_to_degrees(float radians) {
    return radians / (M_PI / 180.0f);
}

void to_euler_angles(float q0, float q1, float q2, float q3)
{
    float roll, pitch, yaw;

    // roll (x-axis rotation)
    float sinr_cosp = 2 * (q0 * q1 + q2 * q3);
    float cosr_cosp = 1 - 2 * (q1 * q1 + q2 * q2);
    roll = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    float sinp = 2 * (q0 * q2 - q3 * q1);
    if (abs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = asin(sinp);

    // yaw (z-axis rotation)
    float siny_cosp = 2 * (q0 * q3 + q1 * q2);
    float cosy_cosp = 1 - 2 * (q2 * q2 + q3 * q3);
    yaw = atan2(siny_cosp, cosy_cosp);

    on_reading(&average_calculator, pitch);

    //if (log)
    //{
        //NRF_LOG_INFO("Roll " NRF_LOG_FLOAT_MARKER ".", NRF_LOG_FLOAT(roll));
        //NRF_LOG_INFO("Pitch " NRF_LOG_FLOAT_MARKER ".", NRF_LOG_FLOAT(pitch));
        //NRF_LOG_INFO("Yaw " NRF_LOG_FLOAT_MARKER ".", NRF_LOG_FLOAT(yaw));
        //NRF_LOG_INFO(NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(radians_to_degrees(roll)));
        //NRF_LOG_INFO(NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(radians_to_degrees(pitch)));
    //}
}

// TODO: is this low to high or?
void mpu9250_int_event_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    if (action == NRF_GPIOTE_POLARITY_LOTOHI && pin == MPU9250_INT_PIN)
    {
        mpu9250_read_accelerometer(&m_twi, &mpu9250);
        mpu9250_read_gyroscope(&m_twi, &mpu9250);
        mpu9250_clear_int_status(&m_twi, &mpu9250); // TODO: could clear on any data read?
        
        mpu9250.processed_accelerometer[0] = -(float)mpu9250.accelerometer[0] * mpu9250.accel_res;
        mpu9250.processed_accelerometer[1] = (float)mpu9250.accelerometer[1] * mpu9250.accel_res;
        mpu9250.processed_accelerometer[2] = (float)mpu9250.accelerometer[2] * mpu9250.accel_res;

        mpu9250.processed_gyroscope[0] = (float)mpu9250.gyroscope[0] * mpu9250.gyro_res * M_PI / 180.0f;
        mpu9250.processed_gyroscope[1] = -(float)mpu9250.gyroscope[1] * mpu9250.gyro_res * M_PI / 180.0f;
        mpu9250.processed_gyroscope[2] = -(float)mpu9250.gyroscope[2] * mpu9250.gyro_res * M_PI / 180.0f;
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

    mpu9250.address = MPU9250_ADDR;
    average_calculator.aggregate_readings_count = 250;
    average_calculator.min_readings = 5;
    average_calculator.max_readings = 10;

    // TODO: can we do blocking then unblocking
    twi_init(&m_twi, ARDUINO_SCL_PIN, ARDUINO_SDA_PIN, NULL);
    gpio_init();
    timers_init();
    softdevice_setup();
    mpu9250_init(&m_twi, &mpu9250);
    
    loop_start = app_timer_cnt_get();
    imu_update_last = app_timer_cnt_get() + 8192;
    to_euler_angles_last = app_timer_cnt_get() + 8192;

    mpu9250_clear_int_status(&m_twi, &mpu9250);

    //mpu9250_read_accelerometer(&m_twi, &mpu9250);
    //mpu9250_read_gyroscope(&m_twi, &mpu9250);
    //NRF_LOG_INFO("MPU9250 accelerometer x %d.", mpu9250.accelerometer[0]);
    //NRF_LOG_INFO("MPU9250 accelerometer y %d.", mpu9250.accelerometer[1]);
    //NRF_LOG_INFO("MPU9250 accelerometer z %d.", mpu9250.accelerometer[2]);
    //NRF_LOG_INFO("MPU9250 gyroscope x %d.", mpu9250.gyroscope[0]);
    //NRF_LOG_INFO("MPU9250 gyroscope y %d.", mpu9250.gyroscope[1]);
    //NRF_LOG_INFO("MPU9250 gyroscope z %d.", mpu9250.gyroscope[2]);

    //while (true)
    //{
    //    nrf_delay_ms(100);
    //    NRF_LOG_INFO("MPU9250 timer %d.", timer_end);
    //    NRF_LOG_FLUSH();
    //}

    //while (true) { }
    uint16_t count;
    uint32_t difference;
    uint32_t min_difference = 1000;
    uint32_t max_difference;

    for(;;) 
    {
        loop_start = app_timer_cnt_get();

        // TODO: set delta freq from timer
        difference = app_timer_cnt_diff_compute(loop_start, imu_update_last);
        if (difference > 32) // TODO: why is that 1000 Hz? it was the 100hz in i2c
        {
            count++;

            if (difference < min_difference)
            {
                min_difference = difference;
            }
            if (difference > max_difference)
            {
                max_difference = difference;
            }

            MadgwickAHRSupdateIMU(
              mpu9250.processed_accelerometer[0],
              mpu9250.processed_accelerometer[1],
              mpu9250.processed_accelerometer[2],
              mpu9250.processed_gyroscope[0],
              mpu9250.processed_gyroscope[1],
              mpu9250.processed_gyroscope[2]);
            
            to_euler_angles(q0, q1, q2, q3);

            imu_update_last = loop_start;
        }
        
        difference = app_timer_cnt_diff_compute(loop_start, to_euler_angles_last);
        if (difference > 8192)
        {
            NRF_LOG_INFO(NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(radians_to_degrees(average_calculator.average)));
            //to_euler_angles(q0, q1, q2, q3, true);
            //NRF_LOG_INFO("IMU update count: %d.", count);
            //NRF_LOG_INFO("IMU update count: %d, IMU min difference %d, IMU max difference %d.", count, min_difference, max_difference);
            NRF_LOG_FLUSH();
            count = 0;
            min_difference = 1000;
            max_difference = 0;
            to_euler_angles_last = loop_start;
        }

        //else
        //{
        //    NRF_LOG_INFO("Skip");
        //}
        //NRF_LOG_INFO("MPU9250 timer difference " NRF_LOG_FLOAT_MARKER ".", NRF_LOG_FLOAT(timer_difference));
        //
        
        
        

        
        
        // waits for events which stops the loop
        //__WFE();
    }
} 
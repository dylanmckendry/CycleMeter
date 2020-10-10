#include "mpu9250.h"
#include "mpu9250_registers.h"

#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"

#include "drivers/common/twi_common.h"

uint8_t MPU9250Gscale = GFS_2000DPS;
uint8_t MPU9250Ascale = AFS_16G;
uint8_t MPU9250Mscale = MFS_16BITS; // Choose either 14-bit or 16-bit magnetometer resolution
uint8_t MPU9250Mmode = 0x06;        // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
float MPU9250aRes, MPU9250gRes, MPU9250mRes;             // scale resolutions per LSB for the sensors


void mpu9250_set_gyro_res(mpu9250_t * mpu9250)
{
    switch (MPU9250Gscale)
    {
            // Possible gyro scales (and their register bit settings) are:
            // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11). 
            // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
        case GFS_250DPS:
              mpu9250->gyro_res = 250.0/32768.0;
              break;
        case GFS_500DPS:
              mpu9250->gyro_res = 500.0/32768.0;
              break;
        case GFS_1000DPS:
              mpu9250->gyro_res = 1000.0/32768.0;
              break;
        case GFS_2000DPS:
              mpu9250->gyro_res = 2000.0/32768.0;
              break;
    }

    return MPU9250gRes;
}

void mpu9250_set_accel_res(mpu9250_t * mpu9250)
{
    switch (MPU9250Ascale)
    {
            // Possible accelerometer scales (and their register bit settings) are:
            // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11). 
            // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
        case AFS_2G:
              mpu9250->accel_res = 2.0/32768.0;
              break;
        case AFS_4G:
              mpu9250->accel_res = 4.0/32768.0;
              break;
        case AFS_8G:
              mpu9250->accel_res = 8.0/32768.0;
              break;
        case AFS_16G:
              mpu9250->accel_res = 16.0/32768.0;
              break;
    }
}

void mpu9250_self_test(nrf_drv_twi_t const * twi_driver, mpu9250_t * mpu9250)
{

}


void mpu9250_init(nrf_drv_twi_t const * twi_driver, mpu9250_t * mpu9250)
{
    // HACK: move
    mpu9250_set_accel_res(mpu9250);
    mpu9250_set_gyro_res(mpu9250);

    write_register(twi_driver, mpu9250->address, MPU9250_PWR_MGMNT_1, 0x00);
    nrf_delay_ms(100);

    write_register(twi_driver, mpu9250->address, MPU9250_PWR_MGMNT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
    nrf_delay_ms(200); 
  
    // Configure Gyro and Thermometer
    // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively; 
    // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
    // be higher than 1 / 0.0059 = 170 Hz
    // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
    // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
    write_register(twi_driver, mpu9250->address, MPU9250_CONFIG, 0x03);  

    // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
    write_register(twi_driver, mpu9250->address, MPU9250_SMPDIV, 0x04);  // Use a 200 Hz rate; a rate consistent with the filter update rate 
                                        // determined inset in CONFIG above
    uint8_t c;

    // Set gyroscope full scale range
    // Range selects FS_SEL and GFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
    read_register(twi_driver, mpu9250->address, MPU9250_GYRO_CONFIG, &c, 1);
    //  writeRegister(GYRO_CONFIG, c & ~0xE0); // Clear self-test bits [7:5] 
    write_register(twi_driver, mpu9250->address, MPU9250_GYRO_CONFIG, c & ~0x03); // Clear Fchoice bits [1:0] 
    write_register(twi_driver, mpu9250->address, MPU9250_GYRO_CONFIG, c & ~0x18); // Clear GFS bits [4:3]
    write_register(twi_driver, mpu9250->address, MPU9250_GYRO_CONFIG, c | MPU9250Gscale << 3); // Set full scale range for the gyro
    // writeRegister(GYRO_CONFIG, c | 0x00); // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
  
    // Set accelerometer full-scale range configuration
    read_register(twi_driver, mpu9250->address, MPU9250_ACCEL_CONFIG, &c, 1);
    //  writeRegister(ACCEL_CONFIG, c & ~0xE0); // Clear self-test bits [7:5] 
    write_register(twi_driver, mpu9250->address, MPU9250_ACCEL_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
    write_register(twi_driver, mpu9250->address, MPU9250_ACCEL_CONFIG, c | MPU9250Ascale << 3); // Set full scale range for the accelerometer 

    // Set accelerometer sample rate configuration
    // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
    // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
    read_register(twi_driver, mpu9250->address, MPU9250_ACCEL_CONFIG2, &c, 1);
    write_register(twi_driver, mpu9250->address, MPU9250_ACCEL_CONFIG2, c & ~0x0F); // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])  
    write_register(twi_driver, mpu9250->address, MPU9250_ACCEL_CONFIG2, c | 0x03); // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz

    // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates, 
    // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

    // Configure Interrupts and Bypass Enable
    // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
    // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips 
    // can join the I2C bus and all can be controlled by the Arduino as master
    write_register(twi_driver, mpu9250->address, MPU9250_INT_PIN_CFG, 0x22);    
    write_register(twi_driver, mpu9250->address, MPU9250_INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
    nrf_delay_ms(100);
}


void mpu9250_clear_int_status(nrf_drv_twi_t const * twi_driver, mpu9250_t * mpu9250)
{
    read_register(twi_driver, mpu9250->address, MPU9250_INT_STATUS, &mpu9250->raw_int_status, 1);
}


void mpu9250_read_accelerometer(nrf_drv_twi_t const * twi_driver, mpu9250_t * mpu9250)
{
    // TODO: can read all of these at once and just ignore temp
    read_register(twi_driver, mpu9250->address, MPU9250_ACCEL_OUT, &mpu9250->raw_accelerometer[0], 6);  // Read the six raw data registers into data array
    mpu9250->accelerometer[0] = ((int16_t)mpu9250->raw_accelerometer[0] << 8) | mpu9250->raw_accelerometer[1] ;  // Turn the MSB and LSB into a signed 16-bit value
    mpu9250->accelerometer[1] = ((int16_t)mpu9250->raw_accelerometer[2] << 8) | mpu9250->raw_accelerometer[3] ;  
    mpu9250->accelerometer[2] = ((int16_t)mpu9250->raw_accelerometer[4] << 8) | mpu9250->raw_accelerometer[5] ; 
}


void mpu9250_read_gyroscope(nrf_drv_twi_t const * twi_driver, mpu9250_t * mpu9250)
{
    read_register(twi_driver, mpu9250->address, MPU9250_GYRO_OUT, &mpu9250->raw_gyroscope[0], 6);  // Read the six raw data registers sequentially into data array
    mpu9250->gyroscope[0] = ((int16_t)mpu9250->raw_gyroscope[0] << 8) | mpu9250->raw_gyroscope[1] ;  // Turn the MSB and LSB into a signed 16-bit value
    mpu9250->gyroscope[1] = ((int16_t)mpu9250->raw_gyroscope[2] << 8) | mpu9250->raw_gyroscope[3] ;  
    mpu9250->gyroscope[2] = ((int16_t)mpu9250->raw_gyroscope[4] << 8) | mpu9250->raw_gyroscope[5] ;
}
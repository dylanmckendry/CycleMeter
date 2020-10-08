#ifndef MPU9250_H
#define MPU9250_H

#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"

#include "drivers/common/twi_common.h"
// TODO: need to add in self test
// MPU9250 registers
const uint8_t MPU9250_ACCEL_OUT = 0x3B;
const uint8_t MPU9250_GYRO_OUT = 0x43;
const uint8_t MPU9250_TEMP_OUT = 0x41;
const uint8_t MPU9250_EXT_SENS_DATA_00 = 0x49;
const uint8_t MPU9250_ACCEL_CONFIG = 0x1C;
const uint8_t MPU9250_ACCEL_FS_SEL_2G = 0x00;
const uint8_t MPU9250_ACCEL_FS_SEL_4G = 0x08;
const uint8_t MPU9250_ACCEL_FS_SEL_8G = 0x10;
const uint8_t MPU9250_ACCEL_FS_SEL_16G = 0x18;
const uint8_t MPU9250_GYRO_CONFIG = 0x1B;
const uint8_t MPU9250_GYRO_FS_SEL_250DPS = 0x00;
const uint8_t MPU9250_GYRO_FS_SEL_500DPS = 0x08;
const uint8_t MPU9250_GYRO_FS_SEL_1000DPS = 0x10;
const uint8_t MPU9250_GYRO_FS_SEL_2000DPS = 0x18;
const uint8_t MPU9250_ACCEL_CONFIG2 = 0x1D;
const uint8_t MPU9250_ACCEL_DLPF_184 = 0x01;
const uint8_t MPU9250_ACCEL_DLPF_92 = 0x02;
const uint8_t MPU9250_ACCEL_DLPF_41 = 0x03;
const uint8_t MPU9250_ACCEL_DLPF_20 = 0x04;
const uint8_t MPU9250_ACCEL_DLPF_10 = 0x05;
const uint8_t MPU9250_ACCEL_DLPF_5 = 0x06;
const uint8_t MPU9250_CONFIG = 0x1A;
const uint8_t MPU9250_GYRO_DLPF_184 = 0x01;
const uint8_t MPU9250_GYRO_DLPF_92 = 0x02;
const uint8_t MPU9250_GYRO_DLPF_41 = 0x03;
const uint8_t MPU9250_GYRO_DLPF_20 = 0x04;
const uint8_t MPU9250_GYRO_DLPF_10 = 0x05;
const uint8_t MPU9250_GYRO_DLPF_5 = 0x06;
const uint8_t MPU9250_SMPDIV = 0x19;
const uint8_t MPU9250_INT_PIN_CFG = 0x37;
const uint8_t MPU9250_INT_ENABLE = 0x38;
const uint8_t MPU9250_INT_DISABLE = 0x00;
const uint8_t MPU9250_INT_PULSE_50US = 0x00;
const uint8_t MPU9250_INT_WOM_EN = 0x40;
const uint8_t MPU9250_INT_RAW_RDY_EN = 0x01;
const uint8_t MPU9250_INT_STATUS = 0x3A;
const uint8_t MPU9250_PWR_MGMNT_1 = 0x6B;
const uint8_t MPU9250_PWR_CYCLE = 0x20;
const uint8_t MPU9250_PWR_RESET = 0x80;
const uint8_t MPU9250_CLOCK_SEL_PLL = 0x01;
const uint8_t MPU9250_PWR_MGMNT_2 = 0x6C;
const uint8_t MPU9250_SEN_ENABLE = 0x00;
const uint8_t MPU9250_DIS_GYRO = 0x07;
const uint8_t MPU9250_USER_CTRL = 0x6A;
const uint8_t MPU9250_I2C_MST_EN = 0x20;
const uint8_t MPU9250_I2C_MST_CLK = 0x0D;
const uint8_t MPU9250_I2C_MST_CTRL = 0x24;
const uint8_t MPU9250_I2C_SLV0_ADDR = 0x25;
const uint8_t MPU9250_I2C_SLV0_REG = 0x26;
const uint8_t MPU9250_I2C_SLV0_DO = 0x63;
const uint8_t MPU9250_I2C_SLV0_CTRL = 0x27;
const uint8_t MPU9250_I2C_SLV0_EN = 0x80;
const uint8_t MPU9250_I2C_READ_FLAG = 0x80;
const uint8_t MPU9250_MOT_DETECT_CTRL = 0x69;
const uint8_t MPU9250_ACCEL_INTEL_EN = 0x80;
const uint8_t MPU9250_ACCEL_INTEL_MODE = 0x40;
const uint8_t MPU9250_LP_ACCEL_ODR = 0x1E;
const uint8_t MPU9250_WOM_THR = 0x1F;
const uint8_t MPU9250_WHO_AM_I = 0x75;
const uint8_t MPU9250_FIFO_EN = 0x23;
const uint8_t MPU9250_FIFO_TEMP = 0x80;
const uint8_t MPU9250_FIFO_GYRO = 0x70;
const uint8_t MPU9250_FIFO_ACCEL = 0x08;
const uint8_t MPU9250_FIFO_MAG = 0x01;
const uint8_t MPU9250_FIFO_COUNT = 0x72;
const uint8_t MPU9250_FIFO_READ = 0x74;

// AK8963 registers
const uint8_t AK8963_HXL = 0x03; 
const uint8_t AK8963_CNTL1 = 0x0A;
const uint8_t AK8963_PWR_DOWN = 0x00;
const uint8_t AK8963_CNT_MEAS1 = 0x12;
const uint8_t AK8963_CNT_MEAS2 = 0x16;
const uint8_t AK8963_FUSE_ROM = 0x0F;
const uint8_t AK8963_CNTL2 = 0x0B;
const uint8_t AK8963_RESET = 0x01;
const uint8_t AK8963_ASA = 0x10;
const uint8_t AK8963_WHO_AM_I = 0x00;

enum MPU9250Ascale {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};

enum MPU9250Gscale {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};

enum MPU9250Mscale {
  MFS_14BITS = 0, // 0.6 mG per LSB
  MFS_16BITS      // 0.15 mG per LSB
};

uint8_t MPU9250Gscale = GFS_500DPS;
uint8_t MPU9250Ascale = AFS_4G;
uint8_t MPU9250Mscale = MFS_16BITS; // Choose either 14-bit or 16-bit magnetometer resolution
uint8_t MPU9250Mmode = 0x06;        // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
float MPU9250aRes, MPU9250gRes, MPU9250mRes;             // scale resolutions per LSB for the sensors

typedef struct
{
    uint8_t address;

    uint8_t raw_int_status;

    uint8_t raw_accelerometer[6];
    int16_t accelerometer[3];
    float processed_accelerometer[3];

    uint8_t raw_gyroscope[6];
    int16_t gyroscope[3];
    float processed_gyroscope[3];
} mpu9250_t;

void MPU9250getGres() {
  switch (MPU9250Gscale)
  {
 	// Possible gyro scales (and their register bit settings) are:
	// 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11). 
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case GFS_250DPS:
          MPU9250gRes = 250.0/32768.0;
          break;
    case GFS_500DPS:
          MPU9250gRes = 500.0/32768.0;
          break;
    case GFS_1000DPS:
          MPU9250gRes = 1000.0/32768.0;
          break;
    case GFS_2000DPS:
          MPU9250gRes = 2000.0/32768.0;
          break;
  }
}

void MPU9250getAres() {
  switch (MPU9250Ascale)
  {
 	// Possible accelerometer scales (and their register bit settings) are:
	// 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11). 
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case AFS_2G:
          MPU9250aRes = 2.0/32768.0;
          break;
    case AFS_4G:
          MPU9250aRes = 4.0/32768.0;
          break;
    case AFS_8G:
          MPU9250aRes = 8.0/32768.0;
          break;
    case AFS_16G:
          MPU9250aRes = 16.0/32768.0;
          break;
  }
}

void mpu9250_init(nrf_drv_twi_t const * twi_driver, mpu9250_t const * mpu9250)
{
    // HACK: move
    MPU9250getAres();
    MPU9250getGres();

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

//int16_t mpu9250_read_temperature(nrf_drv_twi_t const * twi_driver, mpu9250_t const * mpu9250)
//{
//    // TODO: don't recreate these?
//    uint8_t rawData[2];  // temp register data stored here
//    read_register(twi_driver, mpu9250->address, MPU9250_TEMP_OUT, &rawData[0], 2);  // Read the two raw data registers sequentially into data array 
//    return ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a 16-bit value
//}

void mpu9250_read_accelerometer(nrf_drv_twi_t const * twi_driver, mpu9250_t * mpu9250)
{
    // TODO: don't recreate these?
    // TODO: can read all of these at once and just ignore temp
    //uint8_t rawData[6];  // x/y/z accel register data stored here
    read_register(twi_driver, mpu9250->address, MPU9250_ACCEL_OUT, &mpu9250->raw_accelerometer[0], 6);  // Read the six raw data registers into data array
    mpu9250->accelerometer[0] = ((int16_t)mpu9250->raw_accelerometer[0] << 8) | mpu9250->raw_accelerometer[1] ;  // Turn the MSB and LSB into a signed 16-bit value
    mpu9250->accelerometer[1] = ((int16_t)mpu9250->raw_accelerometer[2] << 8) | mpu9250->raw_accelerometer[3] ;  
    mpu9250->accelerometer[2] = ((int16_t)mpu9250->raw_accelerometer[4] << 8) | mpu9250->raw_accelerometer[5] ; 
}

void mpu9250_read_gyroscope(nrf_drv_twi_t const * twi_driver, mpu9250_t * mpu9250)
{
    // TODO: don't recreate these?
    //uint8_t rawData[6];  // x/y/z gyro register data stored here
    read_register(twi_driver, mpu9250->address, MPU9250_GYRO_OUT, &mpu9250->raw_gyroscope[0], 6);  // Read the six raw data registers sequentially into data array
    mpu9250->gyroscope[0] = ((int16_t)mpu9250->raw_gyroscope[0] << 8) | mpu9250->raw_gyroscope[1] ;  // Turn the MSB and LSB into a signed 16-bit value
    mpu9250->gyroscope[1] = ((int16_t)mpu9250->raw_gyroscope[2] << 8) | mpu9250->raw_gyroscope[3] ;  
    mpu9250->gyroscope[2] = ((int16_t)mpu9250->raw_gyroscope[4] << 8) | mpu9250->raw_gyroscope[5] ;
}

#endif //MPU9250_H
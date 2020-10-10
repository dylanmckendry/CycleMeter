#ifndef BMP280_H
#define BMP280_H

#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"

#include "twi_common.h"


// set pressure amd temperature output data rate
uint8_t BMP280Posr = P_OSR_16, BMP280Tosr = T_OSR_02, BMP280Mode = normal, BMP280IIRFilter = BW0_042ODR, BMP280SBy = t_62_5ms;

// t_fine carries fine temperature as global value for BMP280
int32_t t_fine;

// BMP280 compensation parameters
uint16_t dig_T1, dig_P1;
int16_t  dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

typedef struct
{
    uint8_t address;
} bmp280;

void bmp280_init(nrf_drv_twi_t const * twi_driver, bmp280 const * bmp280)
{
    write_register(twi_driver, bmp280->address, BMP280_CTRL_MEAS, BMP280Tosr << 5 | BMP280Posr << 2 | BMP280Mode);
    write_register(twi_driver, bmp280->address, BMP280_CONFIG, BMP280SBy << 5 | BMP280IIRFilter << 2);

    // Read and store calibration data
    uint8_t calib[24];
    read_register(twi_driver, bmp280->address, BMP280_CALIB00, &calib[0], 24);
    dig_T1 = (uint16_t)(((uint16_t) calib[1] << 8) | calib[0]);
    dig_T2 = ( int16_t)((( int16_t) calib[3] << 8) | calib[2]);
    dig_T3 = ( int16_t)((( int16_t) calib[5] << 8) | calib[4]);
    dig_P1 = (uint16_t)(((uint16_t) calib[7] << 8) | calib[6]);
    dig_P2 = ( int16_t)((( int16_t) calib[9] << 8) | calib[8]);
    dig_P3 = ( int16_t)((( int16_t) calib[11] << 8) | calib[10]);
    dig_P4 = ( int16_t)((( int16_t) calib[13] << 8) | calib[12]);
    dig_P5 = ( int16_t)((( int16_t) calib[15] << 8) | calib[14]);
    dig_P6 = ( int16_t)((( int16_t) calib[17] << 8) | calib[16]);
    dig_P7 = ( int16_t)((( int16_t) calib[19] << 8) | calib[18]);
    dig_P8 = ( int16_t)((( int16_t) calib[21] << 8) | calib[20]);
    dig_P9 = ( int16_t)((( int16_t) calib[23] << 8) | calib[22]);
}


int32_t bmp280_compensate_temperature(int32_t adc_T)
{
  int32_t var1, var2, T;
  var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
  t_fine = var1 + var2;
  T = (t_fine * 5 + 128) >> 8;
  return T;
}

uint32_t bmp280_compensate_pressure(int32_t adc_P)
{
  long long var1, var2, p;
  var1 = ((long long)t_fine) - 128000;
  var2 = var1 * var1 * (long long)dig_P6;
  var2 = var2 + ((var1*(long long)dig_P5)<<17);
  var2 = var2 + (((long long)dig_P4)<<35);
  var1 = ((var1 * var1 * (long long)dig_P3)>>8) + ((var1 * (long long)dig_P2)<<12);
  var1 = (((((long long)1)<<47)+var1))*((long long)dig_P1)>>33;
  if(var1 == 0)
  {
    return 0;
    // avoid exception caused by division by zero
  }
  p = 1048576 - adc_P;
  p = (((p<<31) - var2)*3125)/var1;
  var1 = (((long long)dig_P9) * (p>>13) * (p>>13)) >> 25;
  var2 = (((long long)dig_P8) * p)>> 19;
  p = ((p + var1 + var2) >> 8) + (((long long)dig_P7)<<4);
  return (uint32_t)p;
}

int32_t bmp280_read_temperature(nrf_drv_twi_t const * twi_driver, bmp280 const * bmp280)
{
  uint8_t rawData[3];  // 20-bit pressure register data stored here
  read_register(twi_driver, bmp280->address, BMP280_TEMP_MSB, &rawData[0], 3);  
  return bmp280_compensate_temperature((int32_t) (((int32_t) rawData[0] << 16 | (int32_t) rawData[1] << 8 | rawData[2]) >> 4));
}

int32_t bmp280_read_pressure(nrf_drv_twi_t const * twi_driver, bmp280 const * bmp280)
{
  uint8_t rawData[3];  // 20-bit pressure register data stored here
  read_register(twi_driver, bmp280->address, BMP280_PRESS_MSB, &rawData[0], 3);  
  return bmp280_compensate_pressure((int32_t) (((int32_t) rawData[0] << 16 | (int32_t) rawData[1] << 8 | rawData[2]) >> 4));
}

#endif //BMP280_H
#ifndef BMP280_REGISTERS_H
#define BMP280_REGISTERS_H

// BMP280 registers
const uint8_t BMP280_TEMP_XLSB = 0xFC;
const uint8_t BMP280_TEMP_LSB = 0xFB;
const uint8_t BMP280_TEMP_MSB = 0xFA;
const uint8_t BMP280_PRESS_XLSB = 0xF9;
const uint8_t BMP280_PRESS_LSB = 0xF8;
const uint8_t BMP280_PRESS_MSB = 0xF7;
const uint8_t BMP280_CONFIG = 0xF5;
const uint8_t BMP280_CTRL_MEAS = 0xF4;
const uint8_t BMP280_STATUS = 0xF3;
const uint8_t BMP280_RESET = 0xE0;
const uint8_t BMP280_ID = 0xD0;  // should be 0x58
const uint8_t BMP280_CALIB00 = 0x88;

enum BMP280Posr {
  P_OSR_00 = 0,  // no op
  P_OSR_01,
  P_OSR_02,
  P_OSR_04,
  P_OSR_08,
  P_OSR_16
};

enum BMP280Tosr {
  T_OSR_00 = 0,  // no op
  T_OSR_01,
  T_OSR_02,
  T_OSR_04,
  T_OSR_08,
  T_OSR_16
};

enum BMP280IIRFilter {
  full = 0,  // bandwidth at full sample rate
  BW0_223ODR,
  BW0_092ODR,
  BW0_042ODR,
  BW0_021ODR // bandwidth at 0.021 x sample rate
};

enum BMP280Mode {
  Sleep = 0,
  forced,
  forced2,
  normal
};

enum BMP280SBy {
  t_00_5ms = 0,
  t_62_5ms,
  t_125ms,
  t_250ms,
  t_500ms,
  t_1000ms,
  t_2000ms,
  t_4000ms,
};

#endif // BMP280_REGISTERS_H
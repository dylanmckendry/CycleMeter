#ifndef MPU9250_REGISTERS_H
#define MPU9250_REGISTERS_H

#include <stdint.h>

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



#endif //MPU9250_REGISTERS_H
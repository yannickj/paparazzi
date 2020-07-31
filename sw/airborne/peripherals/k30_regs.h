/*
 * Copyright (C)
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */

/**
 * @file modules/sensors/k30_i2c_regs.h
 * @brief Sensor driver for Senseair K30 CO2 sensor via I2C
 *
 */

#ifndef K30_REGS_H
#define K30_REGS_H

#include "std.h"

/**
 * @brief Use double, single (float) or integer for temperature and pressure compensation
 */
#define K30_DOUBLE_PRECISION_COMPENSATION 1
#define K30_SINGLE_PRECISION_COMPENSATION 2
#define K30_INTEGER_COMPENSATION 3

/**
 * @brief By default use double precision compensation
 */
#ifndef K30_COMPENSATION
#define K30_COMPENSATION K30_DOUBLE_PRECISION_COMPENSATION
#endif

/**
 * @brief Status enum
 */
enum K30Status {
  K30_STATUS_UNINIT,
  K30_STATUS_GET_CALIB,
  K30_STATUS_CONFIGURE,
  K30_STATUS_READ_DATA,
  K30_STATUS_ERROR,
  K30_STATUS_READ_ERROR_STATUS
};

/**
 * @brief Status enum for calibration step
 */
enum K30CalibrationStatus {
  K30_CALIB_OLD,
  K30_CALIB_ZERO,
  K30_CALIB_BCC,
  K30_CALIB_ZERO_TRIM,
  K30_CALIB_START_ZERO,
  K30_CALIB_START_BACKGROUND
};

struct k30_reg_calib_data {
  uint16_t par_t1;
  uint16_t old;
  uint16_t zero;
  uint16_t bcc;
  int16_t zero_trim;
  int16_t background_trim;
};

//for DOUBLE_PRECISION_COMPENSATION
/**
 * @brief Quantized Trim Variables
 */
struct k30_quantized_calib_data {
  double par_t1;
  double t_lin;
};


#define K30_METER_CONTROL        0x3E // EEPROM

#define K30_SENSOR_TYPE_ID       0x2C //RAM
#define K30_SENSOR_SERIAL_NUMBER 0x28 //RAM
#define K30_MEMORYP_MAP_ID       0x2F //RAM
#define K30_ABC_PERIOD           0x40 // 2 bytes (MS at lower address), unit 1 hour, write 0 to disable ABC 
#define K30_ABC_OFF           // 1 = off ; 0 = on bit 1 in METER_CONTROL EEPROM

// read and write operation
#define K30_WRITE_RAM                   0x10
#define K30_READ_RAM                    0x20
#define K30_WRITE_EEPROM                0X30
#define K30_READ_EEPROM                 0x40

// TODO define these registers, remove #define K30_DEFAULT_REGISTER after
#define K30_DEFAULT_REGISTER 0
#define K30_CALIB_DATA_ADDR             K30_DEFAULT_REGISTER
#define K30_CALIB_DATA_LEN              K30_DEFAULT_REGISTER
#define K30_SENS_STATUS_REG_ADDR        K30_DEFAULT_REGISTER
#define K30_C02_AND_T_HEADER_DATA_LEN   K30_DEFAULT_REGISTER
#define K30_ALL                         K30_DEFAULT_REGISTER
#define K30_I2C_ADDR                    0xD0 // 0x68 7 bits
#define K30_CO2_HEADER_DATA_LEN         4
#define K30_CO2_REQUEST_LEN             5

/** Measured value and status (MS byte are always at lower address) */
#define K30_PADDING                     0x00
#define K30_CO2_ADDR                    0x08
#define K30_TEMP_ADDR                   0x12 // K33 and K45 only
#define K30_RH_ADDR                     0x14 // K33 only
#define K30_ERROR_STATUS                0x1E

/** Fractional filters (not valid for K45)*/
#define K30_FRACTIONAL_ALGO_OFF         K30_DEFAULT_REGISTER //TODO bit 2 IN METER_CONTROL (1 disabled)
#define K30_DYN_FRACTIONAL_ALGO_OFF     K30_DEFAULT_REGISTER //TODO bit 3 in METER_CONTROL
#define K30_DEFAULT_FRAC            0x4A // (1..8 range)

/** Signal and calibration parameters (not valid for K45)*/
#define K30_CALIB_AUTO_ADDR             0x67 // RAM
#define K30_CALIB_ZERO_CMD              0x7C07
#define K30_CALIB_BACKGROUND_CMD        0x7C06
#define K30_CALIB_OLD_ADDR              0x06 // RAM
#define K30_CALIB_ZERO_ADDR             0x38 // EEPROM READ ONLY !
#define K30_CALIB_ZERO_TRIM_ADDR        0x48 // EEPROM
#define K30_CALIB_BCC_ADDR              0x3C // EEPROM READ ONLY !

/** Error code*/
#define K30_FATAL_ERROR                 0x01
#define K30_OFFSET_REG_ERROR            0x02
#define K30_ALGO_ERROR                  0x04
#define K30_OUPUT_ERROR                 0x08
#define K30_SELF_DIAG_ERROR             0x10
#define K30_OUT_RANGE_ERROR             0x20
#define K30_MEMORY_ERROR                0x40

/** Operation status from the sensor.*/
#define WRITE_RAM_COMPLETE              0x11
#define READ_RAM_COMPLETE               0x21
#define WRITE_EEPROM_COMPLETE           0x31
#define READ_EEPROM_COMPLETE            0x41

#endif /* K30_REGS_H */

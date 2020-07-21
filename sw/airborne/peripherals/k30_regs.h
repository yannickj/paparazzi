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
  K30_STATUS_READ_DATA
};

struct k30_reg_calib_data {
  uint16_t par_t1;
};

//for DOUBLE_PRECISION_COMPENSATION
/**
 * @brief Quantized Trim Variables
 */
struct k30_quantized_calib_data {
  double par_t1;
}

/** MS byte are always at lower address */
#define K30_CO2                  0x08 //in RAM
// #define K30_TEMP                 0x12 //in RAM only K33

#define K30_METER_CONTROL        0x3E // EEPROM

#define K30_SENSOR_TYPE_ID       0x2C //RAM
#define K30_SENSOR_SERIAL_NUMBER 0x28 //RAM
#define K30_MEMORYP_MAP_ID       0x2F //RAM
#define K30_I2C_ADDR_RD          0x20 //RAM (read)
#define K30_I2C_ADDR_WR          0x00 //EEPROM (write (restart needed)) 
#define K30_ABC_PERIOD           0x40 // 2 bytes (MS at lower address), unit 1 hour, write 0 to disable ABC 
#define K30_ABC_ON_OFF           // bit 1 in METER_CONTROL EEPROM

/** Fractional filters */
#define K30_FRACTIONAL_ALGO_OFF   // bit 2 IN METER_CONTROL (1 disabled)
#define K30_DYN_FRACTIONAL_ALGO_OFF // bit 3 in METER_CONTROL
#define K30_DEFAULT_FRAC            0x4A // (1..8 range)

#endif /* K30_REGS_H */


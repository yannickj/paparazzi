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
 * @file modules/sensors/co2_k30.h
 * @brief Sensor driver for Senseair K30 CO2 sensor via I2C
 *
 */

#ifndef K30_I2C_H
#define K30_I2C_H

/* Header includes */
#include "mcu_periph/i2c.h"
#include "peripherals/k30_regs.h"

struct K30_I2c {
  struct i2c_periph *i2c_p;
  struct i2c_transaction i2c_trans;
  enum K30Status status;           ///< state machine status
  enum K30Status last_status;      ///< in order to restart the state machine
  enum K30CalibrationStatus calibration_status; ///< state machine for the configuration step
  bool initialized;                 ///< config done flag
  volatile bool data_available;     ///< data ready flag
  struct k30_reg_calib_data calib; ///< calibration data
#if (K30_COMPENSATION == K30_DOUBLE_PRECISION_COMPENSATION) || ( K30_COMPENSATION == K30_SINGLE_PRECISION_COMPENSATION)
  struct k30_quantized_calib_data quant_calib; ///< quantized calibration data
#endif
  uint16_t raw_co2;                 ///< uncompensated co2 concentration
  uint32_t raw_temperature;         ///< uncompensated temperature
  float co2;                        ///< co2 concentration in ppm
  float temperature;                ///< temperature in deg Celcius
  uint8_t raw_co2_bytes[4];
  uint8_t raw_error_bytes[3];
  uint8_t error_status;
  uint8_t debug_flag;
};

// extern void k30_i2c_read_eeprom_calib(struct K30_I2c *k30);
extern void k30_i2c_init(struct K30_I2c *k30, struct i2c_periph *i2c_p, uint8_t addr);
extern void k30_i2c_periodic(struct K30_I2c *k30);
extern void k30_i2c_event(struct K30_I2c *k30);


#endif /* K30_I2C_H */

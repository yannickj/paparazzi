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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file peripherals/bme280_i2c_spi.h
 * @brief Driver for the BME280 humidity, temperature & pressure sensor
 * The BME280 uses I2C or SPI to communicate, 2 or 4 pins are required
 * to interface.
 * 
 * Modified for Paparazzi from BME280 driver library from Adafruit
 * see https://github.com/adafruit/Adafruit_BME280_I2C_SPI_Library
 * for original code and license
 */

#ifndef BME280_I2C_SPI_H
#define BME280_I2C_SPI_H

#include "mcu_periph/i2c.h"
#include "peripherals/bme280_regs.h"

/**
 * @brief  calibration data
 */
struct bme280_calib_data {
  uint16_t dig_T1; ///< temperature compensation value
  int16_t dig_T2;  ///< temperature compensation value
  int16_t dig_T3;  ///< temperature compensation value

  uint16_t dig_P1; ///< pressure compensation value
  int16_t dig_P2;  ///< pressure compensation value
  int16_t dig_P3;  ///< pressure compensation value
  int16_t dig_P4;  ///< pressure compensation value
  int16_t dig_P5;  ///< pressure compensation value
  int16_t dig_P6;  ///< pressure compensation value
  int16_t dig_P7;  ///< pressure compensation value
  int16_t dig_P8;  ///< pressure compensation value
  int16_t dig_P9;  ///< pressure compensation value

  uint8_t dig_H1;  ///< humidity compensation value
  int16_t dig_H2;  ///< humidity compensation value
  uint8_t dig_H3;  ///< humidity compensation value
  int16_t dig_H4;  ///< humidity compensation value
  int16_t dig_H5;  ///< humidity compensation value
  int8_t dig_H6;   ///< humidity compensation value
};

/**
 * @brief  config register
 */
struct config {
  // inactive duration (standby time) in normal mode
  // 000 = 0.5 ms
  // 001 = 62.5 ms
  // 010 = 125 ms
  // 011 = 250 ms
  // 100 = 500 ms
  // 101 = 1000 ms
  // 110 = 10 ms
  // 111 = 20 ms
  uint8_t t_sb; ///< inactive duration (standby time) in normal mode

  // filter settings
  // 000 = filter off
  // 001 = 2x filter
  // 010 = 4x filter
  // 011 = 8x filter
  // 100 and above = 16x filter
  uint8_t filter; ///< filter settings

  // unused - don't set
  uint8_t none; ///< unused - don't set
  uint8_t spi3w_en; ///< unused - don't set

  /// @return combined config register
  // uint8_t get() { return (t_sb << 5) | (filter << 2) | spi3w_en; }
};

// C++ -> OOP : issue of code conversion with that structure. (keep the get() ?)
// typedef struct config config, *cfg_get;
// struct config {
//   // inactive duration (standby time) in normal mode
//   // 000 = 0.5 ms
//   // 001 = 62.5 ms
//   // 010 = 125 ms
//   // 011 = 250 ms
//   // 100 = 500 ms
//   // 101 = 1000 ms
//   // 110 = 10 ms
//   // 111 = 20 ms
//   uint8_t t_sb; ///< inactive duration (standby time) in normal mode
//
//   // filter settings
//   // 000 = filter off
//   // 001 = 2x filter
//   // 010 = 4x filter
//   // 011 = 8x filter
//   // 100 and above = 16x filter
//   uint8_t filter; ///< filter settings
//
//   // unused - don't set
//   uint8_t none; ///< unused - don't set
//   uint8_t spi3w_en; ///< unused - don't set
//
//   /// @return combined config register
//   // uint8_t get() { return (t_sb << 5) | (filter << 2) | spi3w_en; }
//   cfg_get (*get)(config *);
// };
// cfg_get config_get(config *c){return (void *) ((c->t_sb << 5) | (c->filter << 2) | c->spi3w_en); }

/**
 * @brief  ctrl_meas register
 */
struct ctrl_meas {
  // temperature oversampling
  // 000 = skipped
  // 001 = x1
  // 010 = x2
  // 011 = x4
  // 100 = x8
  // 101 and above = x16
  uint8_t osrs_t; ///< temperature oversampling

  // pressure oversampling
  // 000 = skipped
  // 001 = x1
  // 010 = x2
  // 011 = x4
  // 100 = x8
  // 101 and above = x16
  uint8_t osrs_p; ///< pressure oversampling

  // device mode
  // 00       = sleep
  // 01 or 10 = forced
  // 11       = normal
  uint8_t mode; ///< device mode

  /// @return combined ctrl register
//  uint8_t get() { return (osrs_t << 5) | (osrs_p << 2) | mode; }
};

/**
 * @brief  ctrl_hum register
 */
struct ctrl_hum {
  // pressure oversampling
  // 000 = skipped
  // 001 = x1
  // 010 = x2
  // 011 = x4
  // 100 = x8
  // 101 and above = x16
  uint8_t osrs_h; ///< pressure oversampling

  /// @return combined ctrl hum register
  // uint8_t get() { return (osrs_h); }
};

struct Bme280_I2c_Spi {
  struct i2c_periph *i2c_p;
  struct i2c_transaction i2c_trans;
  enum Bme280Status status;                         ///< state machine status
  enum Bme280CalibStatus calib_status;
  enum Bme280HumidityStatus humidity_status;
  enum Bme280SetSamplingStatus set_sampling_status;
  bool initialized;                                 ///< config done flag
  volatile bool data_available;                     ///< data ready flag
  struct bme280_calib_data calib;                   ///< calibration data
  uint32_t raw_pressure;                            ///< uncompensated pressure
  int32_t raw_temperature;                          ///< uncompensated temperature
  uint32_t raw_humidity;                            ///< uncompensated humidity
  float pressure;                                   ///< pressure in Pa
  float temperature;                                ///< temperature in deg Celcius
  float humidity;                                   ///< humidity in %
  int32_t t_fine;                                   ///< temperature with high resolution, used for 
                                                    ///< temperature compensation in humidity and pressure
  int32_t t_fine_adjust;                            ///< add to compensate temperature readings (-> impact on t_fine)
  struct config config_reg;                         ///< config register object
  struct ctrl_meas ctrl_meas_reg;                   ///< measurement register object
  struct ctrl_hum ctrl_hum_reg;                     ///< hum register object
  uint8_t debug_flag;
  //TODO SPI interface
};

extern void bme280_i2c_spi_init(struct Bme280_I2c_Spi *bme280, struct i2c_periph *i2c_p, uint8_t addr);
extern void bme280_i2c_spi_periodic(struct Bme280_I2c_Spi *bme280);
extern void bme280_i2c_spi_event(struct Bme280_I2c_Spi *bme280);



#endif /* BME280_I2C_SPI_H */

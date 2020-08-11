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
 * @file peripherals/bme280_regs.h
 * @brief Driver for the BME280 humidity, temperature & pressure sensor
 * The BME280 uses I2C or SPI to communicate, 2 or 4 pins are required
 * to interface.
 * 
 * Modified for Paparazzi from BME280 driver library from Adafruit
 * see https://github.com/adafruit/Adafruit_BME280_I2C_SPI_Library
 * for original code and license
 */

#ifndef BME280_REGS_H
#define BME280_REGS_H

/*!
 *  @brief  default I2C address, 
 *  0b111011X 6-first bits fixed
 *  Last bit fix by SDO pin (GND = 0, VDDIO = 1)
 */
#define BME280_I2C_ADDR               0xEC // 0x76 (7 bits)
#define BME280_I2C_ADDR_ALT           0xEE // 0x77 (7 bits)

#define BME280_CHIP_SENSOR_ID         0x60

#define BME280_REGISTER_DIG_T1        0x88
#define BME280_REGISTER_DIG_T2        0x8A
#define BME280_REGISTER_DIG_T3        0x8C

#define BME280_REGISTER_DIG_P1        0x8E
#define BME280_REGISTER_DIG_P2        0x90
#define BME280_REGISTER_DIG_P3        0x92
#define BME280_REGISTER_DIG_P4        0x94
#define BME280_REGISTER_DIG_P5        0x96
#define BME280_REGISTER_DIG_P6        0x98
#define BME280_REGISTER_DIG_P7        0x9A
#define BME280_REGISTER_DIG_P8        0x9C
#define BME280_REGISTER_DIG_P9        0x9E

#define BME280_REGISTER_DIG_H1        0xA1
#define BME280_REGISTER_DIG_H2        0xE1
#define BME280_REGISTER_DIG_H3        0xE3
#define BME280_REGISTER_DIG_H4        0xE4
#define BME280_REGISTER_DIG_H5        0xE5
#define BME280_REGISTER_DIG_H6        0xE7

#define BME280_REGISTER_CHIPID        0xD0
#define BME280_REGISTER_VERSION       0xD1

#define BME280_REGISTER_SOFTRESET     0xE0
#define BME280_COMMAND_SOFTRESET      0xB6

#define BME280_REGISTER_CAL26         0xE1 // R calibration stored in 0xE1-0xF0

#define BME280_REGISTER_CONTROLHUMID  0xF2
#define BME280_REGISTER_STATUS        0XF3
#define BME280_REGISTER_CONTROL       0xF4
#define BME280_REGISTER_CONFIG        0xF5
#define BME280_REGISTER_PRESSUREDATA  0xF7
#define BME280_REGISTER_TEMPDATA      0xFA
#define BME280_REGISTER_HUMIDDATA     0xFD

#define BME280_DATA_ADDR BME280_REGISTER_PRESSUREDATA

/**
 * @brief sampling rates
 */
#define BME280_SAMPLING_NONE          0b000
#define BME280_SAMPLING_X1            0b001
#define BME280_SAMPLING_X2            0b010
#define BME280_SAMPLING_X4            0b011
#define BME280_SAMPLING_X8            0b100
#define BME280_SAMPLING_X16           0b101

/**
 * @brief power modes
 */
#define BME280_MODE_SLEEP             0b00
#define BME280_MODE_FORCED            0b01
#define BME280_MODE_NORMAL            0b11

/**
 * @brief filter values
 */
#define BME280_FILTER_OFF             0b000
#define BME280_FILTER_X2              0b001
#define BME280_FILTER_X4              0b010
#define BME280_FILTER_X8              0b011
#define BME280_FILTER_X16             0b100

/**
 * @brief standby duration in ms
 */
#define BME280_STANDBY_MS_0_5         0b000
#define BME280_STANDBY_MS_10          0b110
#define BME280_STANDBY_MS_20          0b111
#define BME280_STANDBY_MS_62_5        0b001
#define BME280_STANDBY_MS_125         0b010
#define BME280_STANDBY_MS_250         0b011
#define BME280_STANDBY_MS_500         0b100
#define BME280_STANDBY_MS_1000        0b101

/**
 * @brief Status enum
 */
enum Bme280Status {
  BME280_STATUS_UNINIT,
  BME280_STATUS_GET_CALIB,
  BME280_STATUS_CONFIGURE,
  BME280_STATUS_READ_DATA,
  BME280_STATUS_SOFTRESET,
  BME280_STATUS_RESTARTING,
  BME280_STATUS_SET_SAMP
};

enum Bme280HumidityStatus{
  BME280_HUMIDITY_STATUS_H1,
  BME280_HUMIDITY_STATUS_HX
};

enum Bme280CalibStatus{
  BME280_CALIB_STATUS_P_T,
  BME280_CALIB_STATUS_H
};

enum Bme280SetSamplingStatus{
  SET_SAMP_STATUS_SLEEP_MODE,
  SET_SAMP_STATUS_SET_HUMIDITY,
  SET_SAMP_STATUS_SET_REG_CONFIG,
  SET_SAMP_STATUS_SET_REG_CONTROL
};

#endif /* BME280_REGS_H */

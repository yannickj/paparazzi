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
 * @file modules/sensors/co2_k30.c
 * @brief Sensor driver for Senseair K30 CO2 sensor via I2C
 *
 */

#include "peripherals/k30_i2c.h"
#include <math.h>

/** local function to extract raw data from i2c buffer
 *  and compute compensation with selected precision
 */
static void parse_sensor_data(struct K30_I2c *k30);
static void parse_calib_data(struct K30_I2c *k30);
#if K30_COMPENSATION == K30_DOUBLE_PRECISION_COMPENSATION
PRINT_CONFIG_MSG("K30 double precision compensation")
static double compensate_co2(struct K30_I2c *k30);
static double compensate_temperature(struct K30_I2c *k30);
#else
#error "K30: Unknown compensation type"
#endif



/**
 * init function
 */
void k30_i2c_init(struct K30_I2c *k30, struct i2c_periph *i2c_p, uint8_t addr)
{
  /* set i2c_peripheral */
  k30->i2c_p = i2c_p;

  /* slave address */
  k30->i2c_trans.slave_addr = addr;
  /* set initial status: Done */
  k30->i2c_trans.status = I2CTransDone;

  k30->data_available = false;
  k30->initialized = false;
  k30->status = K30_STATUS_UNINIT;
}

/**
 * Start new measurement if sensor ready
 */
void k30_i2c_periodic(struct K30_I2c *k30)
{
  if (k30->i2c_trans.status != I2CTransDone) {
    return; // transaction not finished
  }

  switch (k30->status) {
    case K30_STATUS_UNINIT:
      k30->data_available = false;
      k30->initialized = false;
      k30->status = K30_STATUS_GET_CALIB;
      break;

    case K30_STATUS_GET_CALIB:
      // request calibration data
      k30->i2c_trans.buf[0] = K30_CALIB_DATA_ADDR;
      i2c_transceive(k30->i2c_p, &k30->i2c_trans, k30->i2c_trans.slave_addr, 1, K30_CALIB_DATA_LEN);
      break;

    case K30_STATUS_CONFIGURE:
      // // From datasheet, recommended config for drone usecase:
      // // osrs_p = 8, osrs_t = 1
      // // IIR filter = 2 (note: this one doesn't exist...)
      // // ODR = 50
      // k30->i2c_trans.buf[0] = K30_PWR_CTRL_ADDR;
      // k30->i2c_trans.buf[1] = K30_ALL | K30_NORMAL_MODE << 4;
      // k30->i2c_trans.buf[2] = K30_OSR_ADDR;
      // k30->i2c_trans.buf[3] = K30_OVERSAMPLING_8X | K30_NO_OVERSAMPLING << 3;
      // k30->i2c_trans.buf[4] = K30_ODR_ADDR;
      // k30->i2c_trans.buf[5] = K30_ODR_50_HZ;
      // k30->i2c_trans.buf[6] = K30_CONFIG_ADDR;
      // k30->i2c_trans.buf[7] = K30_IIR_FILTER_COEFF_3 << 1;
      // i2c_transmit(k30->i2c_p, &k30->i2c_trans, k30->i2c_trans.slave_addr, 8);
      break;

    case K30_STATUS_READ_DATA:
      /* read data */
      k30->i2c_trans.buf[0] = K30_SENS_STATUS_REG_ADDR;
      i2c_transceive(k30->i2c_p, &k30->i2c_trans, k30->i2c_trans.slave_addr, 1, K30_C02_AND_T_HEADER_DATA_LEN);
      break;

    default:
      break;
  }
}


void k30_i2c_event(struct K30_I2c *k30)
{
  if (k30->i2c_trans.status == I2CTransSuccess) {
    switch (k30->status) {
      case K30_STATUS_GET_CALIB:
        // compute calib
        parse_calib_data(k30);
        k30->status = K30_STATUS_CONFIGURE;
        break;

      case K30_STATUS_CONFIGURE:
        // nothing else to do, start reading
        k30->status = K30_STATUS_READ_DATA;
        k30->initialized = true;
        break;

      case K30_STATUS_READ_DATA:
        // check status byte
        if (k30->i2c_trans.buf[0] & (K30_ALL << 5)) {
          // parse sensor data, compensate temperature first, then co2 concentration
          parse_sensor_data(k30);
          compensate_temperature(k30);
          compensate_co2(k30);
          k30->data_available = true;
        }
        break;

      default:
        break;
    }
    k30->i2c_trans.status = I2CTransDone;
  } else if (k30->i2c_trans.status == I2CTransFailed) {
    /* try again */
    if (!k30->initialized) {
      k30->status = K30_STATUS_UNINIT;
    }
    k30->i2c_trans.status = I2CTransDone;
  }
}


static void parse_sensor_data(struct K30_I2c *k30)
{
  /* Temporary variables to store the sensor data */

  /* Store the parsed register values for co2 concentration data */
  // k30->raw_co2 = ;

  /* Store the parsed register values for temperature data */
  // k30->raw_temperature = ;
}


#if K30_COMPENSATION == K30_DOUBLE_PRECISION_COMPENSATION

/**
 *  @brief This internal API is used to parse the calibration data, compensates
 *  it and store it in device structure
 */
static void parse_calib_data(struct K30_I2c *k30)
{
  
}

/**
 * @brief This internal API is used to compensate the raw temperature data and
 * return the compensated temperature data in double data type.
 */
static double compensate_temperature(struct K30_I2c *k30)
{
  return k30->quant_calib.t_lin;
}

/**
 * @brief This internal API is used to compensate the raw co2 concentration data and
 * return the compensated co2 concentration data in double data type.
 */
static double compensate_co2(struct K30_I2c *k30)
{
  /* Variable to store the compensated co2 concentration */
  double comp_press;

  return comp_press;
}

#endif


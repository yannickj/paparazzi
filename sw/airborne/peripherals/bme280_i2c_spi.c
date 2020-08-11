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
 * along with paparazzi; see the file COPYING. If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file peripherals/bme280_i2c_spi.c
 * @brief Driver for the BME280 humidity, temperature & pressure sensor
 * The BME280 uses I2C or SPI to communicate, 2 or 4 pins are required
 * to interface.
 * 
 * Modified for Paparazzi from BME280 driver library from Adafruit
 * see https://github.com/adafruit/Adafruit_BME280_I2C_SPI_Library
 * for original code and license
 */

#include "mcu_periph/sys_time_arch.h"
#include "bme280_i2c_spi.h"

static void init_default_config(struct Bme280_I2c_Spi *bme280);

static void send_request(struct Bme280_I2c_Spi *bme280, int request_buf_size, int response_buf_size);

static void request_set_sampling(struct Bme280_I2c_Spi *bme280);
static void parse_set_sampling(struct Bme280_I2c_Spi *bme280);

static void request_is_reading_calibration(struct Bme280_I2c_Spi *bme280);
static inline void parse_is_reading_calibration(struct Bme280_I2c_Spi *bme280);

static void request_p_t_calibration(struct Bme280_I2c_Spi *bme280);
static void request_h_calibration(struct Bme280_I2c_Spi *bme280);
static void parse_p_t_calibration(struct Bme280_I2c_Spi *bme280);
static void parse_h_calibration(struct Bme280_I2c_Spi *bme280);

static void request_data(struct Bme280_I2c_Spi *bme280);
static void parse_data(struct Bme280_I2c_Spi *bme280);

static inline void compensate_data(struct Bme280_I2c_Spi *bme280);
static void compensate_pressure(struct Bme280_I2c_Spi *bme280);
static void compensate_temperature(struct Bme280_I2c_Spi *bme280);
static void compensate_humidity(struct Bme280_I2c_Spi *bme280);

static inline uint16_t read16_LE(volatile uint8_t buf[], uint8_t index){
  uint16_t v = (buf[index] << 8) | buf[index +1];
  return (v >> 8) | (v << 8);
}

static inline int16_t readS16_LE(volatile uint8_t buf[], uint8_t index)
{
  return (int16_t) read16_LE(buf, index);
}

static void init_default_config(struct Bme280_I2c_Spi *bme280)
{
  // default config for the config register
  bme280->config_reg.t_sb = BME280_STANDBY_MS_0_5;
  bme280->config_reg.filter = BME280_FILTER_OFF;
  // bme280->config_reg.spi3w_en = 1; ///< currently unused
  // default config for the constrol measure register
  bme280->ctrl_meas_reg.osrs_t = BME280_SAMPLING_X16;
  bme280->ctrl_meas_reg.osrs_p = BME280_SAMPLING_X16;
  bme280->ctrl_meas_reg.mode = BME280_MODE_NORMAL;
  // default config for the humidity measure register
  bme280->ctrl_hum_reg.osrs_h = BME280_SAMPLING_X16;
}

/**
 * @brief init function
 */
void bme280_i2c_spi_init(struct Bme280_I2c_Spi *bme280, struct i2c_periph *i2c_p, uint8_t addr)
{
  /* set i2c_peripheral */
  bme280->i2c_p = i2c_p;
  /* slave address */
  bme280->i2c_trans.slave_addr = addr;
  /* set initial status: Done */
  bme280->i2c_trans.status = I2CTransDone;
  bme280->data_available = false;
  bme280->initialized = false;
  // init the state machines
  bme280->status = BME280_STATUS_UNINIT;
  bme280->calib_status = BME280_CALIB_STATUS_P_T;
  bme280->humidity_status = BME280_HUMIDITY_STATUS_H1;
  bme280->set_sampling_status = SET_SAMP_STATUS_SLEEP_MODE;
  // currently no offset, to adjust the compensate temperature
  bme280->t_fine_adjust = 0;
  init_default_config(bme280);
  bme280->debug_flag = 1;
}

/**
 * @brief periodic function. If sensor ready, requests data.
 */
void bme280_i2c_spi_periodic(struct Bme280_I2c_Spi *bme280)
{
  if (bme280->i2c_trans.status != I2CTransDone) {
    return; // transaction not finished
  }
  switch (bme280->status) {
    case BME280_STATUS_UNINIT:
      bme280->debug_flag = 2;
      bme280->data_available = false;
      bme280->initialized = false;
      // check if sensor, i.e. the chip ID is correct
      bme280->i2c_trans.buf[0] = BME280_REGISTER_CHIPID;
      send_request(bme280, 1, 1);
      break;
    case BME280_STATUS_SOFTRESET:
      bme280->i2c_trans.buf[0] = BME280_REGISTER_SOFTRESET;
      bme280->i2c_trans.buf[1] = BME280_COMMAND_SOFTRESET;
      i2c_transmit(bme280->i2c_p, &bme280->i2c_trans, bme280->i2c_trans.slave_addr, 2);
      break;
    case BME280_STATUS_RESTARTING:
      request_is_reading_calibration(bme280);
      break;
    case BME280_STATUS_SET_SAMP:
      request_set_sampling(bme280);
      break;
    case BME280_STATUS_GET_CALIB:
      switch (bme280->calib_status){
        case BME280_CALIB_STATUS_P_T:
          request_p_t_calibration(bme280);
          break;
        case BME280_CALIB_STATUS_H:
          request_h_calibration(bme280);
          break;
        default:
          break;
      }
      break;
    case BME280_STATUS_READ_DATA:
      request_data(bme280);
      bme280->debug_flag = 6;
      break;
    default:
      break;
  }
}

void bme280_i2c_spi_event(struct Bme280_I2c_Spi *bme280)
{
  if (bme280->i2c_trans.status == I2CTransSuccess) {
    switch (bme280->status) {
      case BME280_STATUS_UNINIT:
        if (bme280->i2c_trans.buf[0] == BME280_CHIP_SENSOR_ID){
          bme280->status = BME280_STATUS_SOFTRESET;
        }
        break;
      case BME280_STATUS_SOFTRESET:
        // wait for chip to wake up.
        sys_time_msleep(10);
        bme280->status = BME280_STATUS_RESTARTING;
        break;
      case BME280_STATUS_RESTARTING:
        parse_is_reading_calibration(bme280);
        break;
      case BME280_STATUS_SET_SAMP:
        parse_set_sampling(bme280);
        break;
      case BME280_STATUS_GET_CALIB:
        switch (bme280->calib_status){
          case BME280_CALIB_STATUS_P_T:
            parse_p_t_calibration(bme280);
            bme280->calib_status = BME280_CALIB_STATUS_H;
            break;
          case BME280_CALIB_STATUS_H:
            parse_h_calibration(bme280);
            bme280->status = BME280_STATUS_READ_DATA;
            bme280->initialized = true;
            break;
          default:
            break;
        }
        break;
      case BME280_STATUS_READ_DATA:
          parse_data(bme280);
          compensate_data(bme280);
          bme280->debug_flag = 14;
          bme280->data_available = true;
        break;
      default:
        break;
    }
    bme280->i2c_trans.status = I2CTransDone;
  } else if (bme280->i2c_trans.status == I2CTransFailed) {
    bme280->debug_flag = 10;
    /* try again */
    if (!bme280->initialized) {
      bme280->status = BME280_STATUS_UNINIT;
      bme280->debug_flag = 11;
    }
    bme280->i2c_trans.status = I2CTransDone;
  }
}

/**
 * @brief Equivalent to i2c_transceive with a break-time between transmit and receive
 * if needed for the sensor.
 */
static void send_request(struct Bme280_I2c_Spi *bme280, int request_buf_size, int response_buf_size)
{
  i2c_transmit(bme280->i2c_p, &bme280->i2c_trans, bme280->i2c_trans.slave_addr, request_buf_size);
  // sys_time_msleep(20); //TODO needed ? correct time ?
  i2c_receive(bme280->i2c_p, &bme280->i2c_trans, bme280->i2c_trans.slave_addr, response_buf_size);
  // sys_time_msleep(35); needed ?
  // bme280->debug_flag = 5;
}
/**
 * @brief This internal API is used to configure the sampling mode for 
 * the temperature, pressure and humidity data.
 */
static void request_set_sampling(struct Bme280_I2c_Spi *bme280)
{
  switch (bme280->set_sampling_status){
    case SET_SAMP_STATUS_SLEEP_MODE:
      // making sure sensor is in sleep mode before setting configuration
      // as it otherwise may be ignored
      bme280->i2c_trans.buf[0] = BME280_REGISTER_CONTROL;
      bme280->i2c_trans.buf[1] = BME280_MODE_SLEEP;
      i2c_transmit(bme280->i2c_p, &bme280->i2c_trans, bme280->i2c_trans.slave_addr, 2);
      break;
    case SET_SAMP_STATUS_SET_HUMIDITY:
      // you must make sure to also set REGISTER_CONTROL after setting the
      // CONTROLHUMID register, otherwise the values won't be applied (see
      // DS 5.4.3)
      bme280->i2c_trans.buf[0] = BME280_REGISTER_CONTROLHUMID;
      bme280->i2c_trans.buf[1] = bme280->ctrl_hum_reg.osrs_h;
      i2c_transmit(bme280->i2c_p, &bme280->i2c_trans, bme280->i2c_trans.slave_addr, 2);
      break;
    case SET_SAMP_STATUS_SET_REG_CONFIG:
      bme280->i2c_trans.buf[0] = BME280_REGISTER_CONFIG;
      bme280->i2c_trans.buf[1] = (bme280->config_reg.t_sb << 5) | (bme280->config_reg.filter << 2) | bme280->config_reg.spi3w_en;
      i2c_transmit(bme280->i2c_p, &bme280->i2c_trans, bme280->i2c_trans.slave_addr, 2);
      break;
    case SET_SAMP_STATUS_SET_REG_CONTROL:
      bme280->i2c_trans.buf[0] = BME280_REGISTER_CONTROL;
      bme280->i2c_trans.buf[1] = (bme280->ctrl_meas_reg.osrs_t << 5) | (bme280->ctrl_meas_reg.osrs_p << 2) | bme280->ctrl_meas_reg.mode;
      i2c_transmit(bme280->i2c_p, &bme280->i2c_trans, bme280->i2c_trans.slave_addr, 2);
      break;
    default:
      break;
  }
}

static void parse_set_sampling(struct Bme280_I2c_Spi *bme280)
{
  switch (bme280->set_sampling_status){
    case SET_SAMP_STATUS_SLEEP_MODE:
      bme280->set_sampling_status = SET_SAMP_STATUS_SET_HUMIDITY;
      break;
    case SET_SAMP_STATUS_SET_HUMIDITY:
      bme280->set_sampling_status = SET_SAMP_STATUS_SET_REG_CONFIG;
      break;
    case SET_SAMP_STATUS_SET_REG_CONFIG:
      bme280->set_sampling_status = SET_SAMP_STATUS_SET_REG_CONTROL;
      break;
    case SET_SAMP_STATUS_SET_REG_CONTROL:
      bme280->status = BME280_STATUS_GET_CALIB;
      break;
    default:
      break;
  }
}

static void request_is_reading_calibration(struct Bme280_I2c_Spi *bme280)
{
  bme280->i2c_trans.buf[0] = BME280_REGISTER_STATUS;
  send_request(bme280, 1, 1);
}

static inline void parse_is_reading_calibration(struct Bme280_I2c_Spi *bme280){
  if (!bme280->i2c_trans.buf[0]){
    bme280->status = BME280_STATUS_SET_SAMP;
  }
}

/**
 *  @brief This internal API is used to request the calibration data from the device.
 */
static void request_p_t_calibration(struct Bme280_I2c_Spi *bme280)
{
  bme280->i2c_trans.buf[0] = BME280_REGISTER_DIG_T1;
  send_request(bme280, 1, 24);
}

static void request_h_calibration(struct Bme280_I2c_Spi *bme280)
{
  switch (bme280->humidity_status){
    case BME280_HUMIDITY_STATUS_H1:
      bme280->i2c_trans.buf[0] = BME280_REGISTER_DIG_H1;
      send_request(bme280, 1, 1);
      break;
    case BME280_HUMIDITY_STATUS_HX:
      bme280->i2c_trans.buf[0] = BME280_REGISTER_DIG_H1;
      send_request(bme280, 1, 7);
      break;
    default:
      break;
  }
}

/**
 *  @brief This internal API is used to parse the calibration data, compensates
 *  it and store it in device structure.
 */
static void parse_p_t_calibration(struct Bme280_I2c_Spi *bme280)
{
  // parse temperature calibration data
  bme280->calib.dig_T1 = read16_LE(bme280->i2c_trans.buf, 0);
  bme280->calib.dig_T2 = readS16_LE(bme280->i2c_trans.buf, 2);
  bme280->calib.dig_T3 = readS16_LE(bme280->i2c_trans.buf, 4);
  // parse pressure calibration data
  bme280->calib.dig_P1 = read16_LE(bme280->i2c_trans.buf, 6);
  bme280->calib.dig_P2 = readS16_LE(bme280->i2c_trans.buf, 8);
  bme280->calib.dig_P3 = readS16_LE(bme280->i2c_trans.buf, 10);
  bme280->calib.dig_P4 = readS16_LE(bme280->i2c_trans.buf, 12);
  bme280->calib.dig_P5 = readS16_LE(bme280->i2c_trans.buf, 14);
  bme280->calib.dig_P6 = readS16_LE(bme280->i2c_trans.buf, 16);
  bme280->calib.dig_P7 = readS16_LE(bme280->i2c_trans.buf, 18);
  bme280->calib.dig_P8 = readS16_LE(bme280->i2c_trans.buf, 20);
  bme280->calib.dig_P9 = readS16_LE(bme280->i2c_trans.buf, 22);
}

static void parse_h_calibration(struct Bme280_I2c_Spi *bme280)
{
  switch (bme280->humidity_status){
    case BME280_HUMIDITY_STATUS_H1:
      bme280->calib.dig_H1 = read16_LE(bme280->i2c_trans.buf, 0);
      bme280->humidity_status = BME280_HUMIDITY_STATUS_HX;
      break;
    case BME280_HUMIDITY_STATUS_HX:
      bme280->calib.dig_H2 = readS16_LE(bme280->i2c_trans.buf, 0);
      bme280->calib.dig_H3 = bme280->i2c_trans.buf[2];
      bme280->calib.dig_H4 = ((int8_t) bme280->i2c_trans.buf[3] << 4) |
                            (bme280->i2c_trans.buf[4] & 0xF);
      bme280->calib.dig_H5 = ((int8_t) bme280->i2c_trans.buf[5] << 4) |
                            (bme280->i2c_trans.buf[4] >> 4);
      bme280->calib.dig_H6 = (int8_t) bme280->i2c_trans.buf[6];
      break;
    default:
      break;
  }
}

/**
 * @brief This internal API is used to request the raw temperature, pressure and humidity
 * data (24 bits).
 */
static void request_data(struct Bme280_I2c_Spi *bme280)
{
  // read 3 bytes for each measure (pressure -> temperature -> humidity)
  bme280->i2c_trans.buf[0] = BME280_DATA_ADDR;
  send_request(bme280, 1, 9);
}

/**
 * @brief This internal API is used to parse the raw temperature, pressure and humidity
 * data (24 bits).
 */
static void parse_data(struct Bme280_I2c_Spi *bme280)
{
  // parse raw data
  bme280->raw_pressure = ((uint32_t) bme280->i2c_trans.buf[0] << 16) | ((uint32_t) bme280->i2c_trans.buf[1] << 8) | bme280->i2c_trans.buf[2];
  bme280->raw_temperature = ((uint32_t) bme280->i2c_trans.buf[3] << 16) | ((uint32_t) bme280->i2c_trans.buf[4] << 8) | bme280->i2c_trans.buf[5];
  bme280->raw_humidity = ((uint32_t) bme280->i2c_trans.buf[6] << 16) | ((uint32_t) bme280->i2c_trans.buf[7] << 8) | bme280->i2c_trans.buf[8];
  // compensation of temperature, pressure, humidity
  compensate_data(bme280);
  bme280->debug_flag = 30;
}

/**
 * @brief This internal API is used to compensate the raw temperature, pressure and humidity
 * data and return the compensated co2 concentration data in float data type.
 */
static inline void compensate_data(struct Bme280_I2c_Spi *bme280)
{
  // should not change the order (pressure and humidity need the t_fine updated in compensate_temperature)
  compensate_temperature(bme280);
  compensate_pressure(bme280);
  compensate_humidity(bme280);
}

static void compensate_temperature(struct Bme280_I2c_Spi *bme280)
{
  if (bme280->raw_temperature == 0x800000) // value in case temperature measurement was disabled
    return;
  int32_t var1, var2;
  int32_t adc_T = bme280->raw_temperature >> 4;
  var1 = ((((adc_T >> 3) - ((int32_t)bme280->calib.dig_T1 << 1))) * ((int32_t)bme280->calib.dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((int32_t)bme280->calib.dig_T1)) * ((adc_T >> 4) - ((int32_t)bme280->calib.dig_T1))) >> 12) * ((int32_t)bme280->calib.dig_T3)) >> 14;
  
  bme280->t_fine = var1 + var2 + bme280->t_fine_adjust;
  bme280->temperature = ((float) ((bme280->t_fine * 5 + 128) >> 8)) / 100;
}

static void compensate_pressure(struct Bme280_I2c_Spi *bme280)
{
  if (bme280->raw_pressure == 0x800000) // value in case pressure measurement was disabled
    return;
  int32_t adc_P = bme280->raw_pressure >> 4;
  int64_t var1, var2, p;
  var1 = ((int64_t)bme280->t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)bme280->calib.dig_P6;
  var2 += (var1 * (int64_t)bme280->calib.dig_P5) << 17;
  var2 += ((int64_t)bme280->calib.dig_P4) << 35;
  var1 = ((var1 * var1 * (int64_t)bme280->calib.dig_P3) >> 8) +
         ((var1 * (int64_t)bme280->calib.dig_P2) << 12);
  var1 = ((((int64_t)1) << 47) + var1) * ((int64_t)bme280->calib.dig_P1) >> 33;

  if (var1 == 0) {
    return; // avoid exception caused by division by zero
  }
  p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)bme280->calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)bme280->calib.dig_P8) * p) >> 19;

  bme280->pressure = ((float) ((p + var1 + var2) >> 8) + (((int64_t)bme280->calib.dig_P7) << 4)) / 256;
}

static void compensate_humidity(struct Bme280_I2c_Spi *bme280)
{
  if (bme280->raw_humidity == 0x800000) // value in case pressure measurement was disabled
    return;
  int32_t adc_H = bme280->raw_humidity >> 4;
  int32_t v_x1_u32r;

  v_x1_u32r = (bme280->t_fine - ((int32_t)76800));
  v_x1_u32r = ((((adc_H << 14) - (((int32_t)bme280->calib.dig_H4) << 20) -
                  (((int32_t)bme280->calib.dig_H5) * v_x1_u32r)) +
                 ((int32_t)16384)) >>
                15) *
               (((((((v_x1_u32r * ((int32_t)bme280->calib.dig_H6)) >> 10) *
                    (((v_x1_u32r * ((int32_t)bme280->calib.dig_H3)) >> 11) +
                     ((int32_t)32768))) >> 10) +
                  ((int32_t)2097152)) * ((int32_t)bme280->calib.dig_H2) + 8192) >> 14);

  v_x1_u32r = v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
                             ((int32_t)bme280->calib.dig_H1)) >> 4);

  v_x1_u32r = (v_x1_u32r < 0) ? 0 : v_x1_u32r;
  v_x1_u32r = (v_x1_u32r > 419430400) ? 419430400 : v_x1_u32r;
  bme280->humidity = ((float) (v_x1_u32r >> 12)) / 1024.0;
}

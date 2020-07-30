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
#include "mcu_periph/sys_time_arch.h"

#define ZERO_TRIM_WRITE 0  // 0 = skip the zero_trim (write in EEPROM operation)
#define ZERO_TRIM_READ (! ZERO_TRIM_WRITE) // until now code works in read or write only for zero_trim

/** local function to extract raw data from i2c buffer
 *  and compute compensation with selected precision
 */
static void send_request(struct K30_I2c *k30, int request_buf_size, int response_buf_size);
static void request_co2(struct K30_I2c *k30);
static void request_error_status(struct K30_I2c *k30);

static void request_calib_old(struct K30_I2c *k30);
static void request_calib_bcc(struct K30_I2c *k30);
static void request_calib_zero(struct K30_I2c *k30);

static void parse_co2_data(struct K30_I2c *k30);
static void parse_error_status(struct K30_I2c *k30);
static void parse_calib_old(struct K30_I2c *k30);
static void parse_calib_bcc(struct K30_I2c *k30);
static void parse_calib_zero(struct K30_I2c *k30);

#if ZERO_TRIM_READ
static void request_calib_zero_trim_read(struct K30_I2c *k30);
static void parse_calib_zero_trim_read(struct K30_I2c *k30);
#endif

#if ZERO_TRIM_WRITE
static void parse_calib_zero_trim_write(struct K30_I2c *k30);
static void request_calib_zero_trim_write(struct K30_I2c *k30);
#endif

static void compensate_co2(struct K30_I2c *k30);

// #if K30_COMPENSATION == K30_DOUBLE_PRECISION_COMPENSATION
// PRINT_CONFIG_MSG("K30 double precision compensation")
// static double compensate_co2(struct K30_I2c *k30);
// static double compensate_co2(struct K30_I2c *k30);
// static double compensate_temperature(struct K30_I2c *k30);
// #else
// #error "K30: Unknown compensation type"
// #endif


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
  k30->debug_flag = 1;
  for(int i=0; i<4; i++){
    k30->raw_co2_bytes[i] = 0;
  }
  for(int i=0; i<3; i++){
    k30->raw_error_bytes[i] = 0;
  }
  k30->error_status = 0;
  k30->calib.old = 0;
  k30->calib.zero = 0;
  k30->calib.bcc = 0;
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
    case K30_STATUS_ERROR:
      //TODO Error treatment
      request_error_status(k30);
      break;
    case K30_STATUS_UNINIT:
    k30->debug_flag = 2;
      k30->data_available = false;
      k30->initialized = false;
      k30->status = K30_STATUS_GET_CALIB;
      break;
    case K30_STATUS_GET_CALIB:
      switch (k30->calibration_status){
        case K30_CALIB_OLD:
          request_calib_old(k30);
          break;

        case K30_CALIB_ZERO:
          request_calib_zero(k30);
          break;
        
        case K30_CALIB_BCC:
          request_calib_bcc(k30);
          break;
        
        case K30_CALIB_ZERO_TRIM:
          #if ZERO_TRIM_READ
          request_calib_zero_trim_read(k30);
          #endif
          #if ZERO_TRIM_WRITE
          // for zero calibration (0 ppm)
          k30->calib.zero_trim = 2048 * 61440 / k30->calib.old - k30->calib.zero; // zero calibration
          // k30->calib.zero_trim = 400; //test
          // for backgroud calibration (400 ppm)
          k30->calib.background_trim = 2048 * k30->calib.bcc / k30->calib.old - k30->calib.zero;
          request_calib_zero_trim_write(k30); // write eeprom K30_CALIB_ZERO_TRIM_ADDR
          //TODO restart sensor
          #endif
          break;

        default:
          break;
      }
      break;
    case K30_STATUS_CONFIGURE:
      k30->debug_flag = 3;
      k30->status = K30_STATUS_READ_ERROR_STATUS;
      k30->initialized = true;
      break;
    case K30_STATUS_READ_ERROR_STATUS:
      k30->debug_flag = 4;
      request_error_status(k30);
      break;
    case K30_STATUS_READ_DATA:
      /* read data */
      request_co2(k30);
      k30->debug_flag = 6;
      //i2c_transceive(k30->i2c_p, &k30->i2c_trans, k30->i2c_trans.slave_addr, K30_CO2_REQUEST_LEN, K30_CO2_HEADER_DATA_LEN);
      break;
    default:
      break;
  }
}


void k30_i2c_event(struct K30_I2c *k30)
{
  if (k30->i2c_trans.status == I2CTransSuccess) {
    switch (k30->status) {
      case K30_STATUS_ERROR:
        //TODO Error treatment
        parse_error_status(k30);
        k30->debug_flag = 15;
        break;
      case K30_STATUS_UNINIT:
        // TODO
        k30->last_status = K30_STATUS_UNINIT;
        break;
      case K30_STATUS_GET_CALIB:
        switch (k30->calibration_status){
          case K30_CALIB_OLD:
            parse_calib_old(k30);
            break;
          case K30_CALIB_ZERO:
            parse_calib_zero(k30);
            break;  
          case K30_CALIB_BCC:
            parse_calib_bcc(k30);
            break;
          case K30_CALIB_ZERO_TRIM:
            #if ZERO_TRIM_READ  
              parse_calib_zero_trim_read(k30);
            #endif
            #if ZERO_TRIM_WRITE
              parse_calib_zero_trim_write(k30);
            #endif
            break;   
          default:
            break;
        }
        break;
      case K30_STATUS_CONFIGURE:
        // nothing else to do, start reading
        k30->last_status = k30->status;
        k30->status = K30_STATUS_READ_ERROR_STATUS;
        k30->initialized = true;
        break;
      case K30_STATUS_READ_ERROR_STATUS:
        k30->debug_flag = 13;
        k30->last_status = k30->status;
        parse_error_status(k30);
        // k30->data_available = true;
        break;
      case K30_STATUS_READ_DATA:
        // check status byte
        // if (k30->i2c_trans.buf[0] & (K30_ALL << 5)) {
          // parse sensor data, compensate temperature first, then co2 concentration
          k30->last_status = k30->status;
          parse_co2_data(k30);
          // compensate_temperature(k30);
          // compensate_co2(k30);
          k30->debug_flag = 14;
          k30->data_available = true;
        // }
        break;
      default:
        break;
    }
    k30->i2c_trans.status = I2CTransDone;
  } else if (k30->i2c_trans.status == I2CTransFailed) {
    k30->debug_flag = 10;
    /* try again */
    if (!k30->initialized) {
      k30->status = K30_STATUS_UNINIT;
      k30->debug_flag = 11;
    }
    k30->i2c_trans.status = I2CTransDone;
  }
}

static void send_request(struct K30_I2c *k30, int request_buf_size, int response_buf_size)
{
  i2c_transmit(k30->i2c_p, &k30->i2c_trans, k30->i2c_trans.slave_addr, request_buf_size);
  sys_time_msleep(20); // necessary for the device
  i2c_receive(k30->i2c_p, &k30->i2c_trans, k30->i2c_trans.slave_addr, response_buf_size);
  sys_time_msleep(35);
  // k30->debug_flag = 5;
}

static void request_co2(struct K30_I2c *k30)
{
  k30->i2c_trans.buf[0] = K30_READ_RAM + 2; // command number + 2 bytes to read 
  k30->i2c_trans.buf[1] = K30_PADDING; // 0x00;
  k30->i2c_trans.buf[2] = K30_CO2_ADDR; //0x08; //adress to read
  k30->i2c_trans.buf[3] = k30->i2c_trans.buf[0] + k30->i2c_trans.buf[1] + k30->i2c_trans.buf[2]; // 0x2A; //checksum to compare
  send_request(k30, 4, 4);  
}

static void request_error_status(struct K30_I2c *k30)
{
  k30->i2c_trans.buf[0] = K30_READ_RAM + 1;
  k30->i2c_trans.buf[1] = K30_PADDING;
  k30->i2c_trans.buf[2] = K30_ERROR_STATUS;
  k30->i2c_trans.buf[3] = k30->i2c_trans.buf[0] + k30->i2c_trans.buf[1] + k30->i2c_trans.buf[2];
  send_request(k30, 4, 3);  
}

static void request_calib_old(struct K30_I2c *k30)
{
  k30->i2c_trans.buf[0] = K30_READ_RAM + 2;
  k30->i2c_trans.buf[1] = K30_PADDING;
  k30->i2c_trans.buf[2] = K30_CALIB_OLD_ADDR;
  k30->i2c_trans.buf[3] = k30->i2c_trans.buf[0] + k30->i2c_trans.buf[1] + k30->i2c_trans.buf[2];
  k30->debug_flag = 20;
  send_request(k30, 4, 4);  
}

static void request_calib_zero(struct K30_I2c *k30)
{
  k30->i2c_trans.buf[0] = K30_READ_EEPROM + 2;
  k30->i2c_trans.buf[1] = K30_PADDING;
  k30->i2c_trans.buf[2] = K30_CALIB_ZERO_ADDR;
  k30->i2c_trans.buf[3] = k30->i2c_trans.buf[0] + k30->i2c_trans.buf[1] + k30->i2c_trans.buf[2];
  k30->debug_flag = 21;
  send_request(k30, 4, 4);  
}

static void request_calib_bcc(struct K30_I2c *k30)
{
  k30->i2c_trans.buf[0] = K30_READ_EEPROM + 2;
  k30->i2c_trans.buf[1] = K30_PADDING;
  k30->i2c_trans.buf[2] = K30_CALIB_BCC_ADDR;
  k30->i2c_trans.buf[3] = k30->i2c_trans.buf[0] + k30->i2c_trans.buf[1] + k30->i2c_trans.buf[2];
  k30->debug_flag = 22;
  send_request(k30, 4, 4);  
}

#if ZERO_TRIM_READ
static void request_calib_zero_trim_read(struct K30_I2c *k30)
{
  k30->i2c_trans.buf[0] = K30_READ_EEPROM + 2;
  k30->i2c_trans.buf[1] = K30_PADDING;
  k30->i2c_trans.buf[2] = K30_CALIB_ZERO_TRIM_ADDR;
  k30->i2c_trans.buf[3] = k30->i2c_trans.buf[0] + k30->i2c_trans.buf[1] + k30->i2c_trans.buf[2];
  k30->debug_flag = 22;
  send_request(k30, 4, 4);  
}
#endif

#if ZERO_TRIM_WRITE
static void request_calib_zero_trim_write(struct K30_I2c *k30)
{
  // not used still in dev
  k30->i2c_trans.buf[0] = K30_WRITE_EEPROM + 2;
  k30->i2c_trans.buf[1] = K30_PADDING;
  k30->i2c_trans.buf[2] = K30_CALIB_ZERO_TRIM_ADDR;
  k30->i2c_trans.buf[3] = (uint8_t) (k30->calib.zero_trim >> 8);
  k30->i2c_trans.buf[4] = (uint8_t) (k30->calib.zero_trim & 255);
  k30->i2c_trans.buf[5] = k30->i2c_trans.buf[0] + k30->i2c_trans.buf[1] + k30->i2c_trans.buf[2] + k30->i2c_trans.buf[3] + k30->i2c_trans.buf[4];
  k30->debug_flag = 23;
  send_request(k30, 5, 2);  
}
#endif

static void parse_co2_data(struct K30_I2c *k30)
{
  /* Temporary variables to store the sensor data */
  uint8_t data_checksum, data_msb, data_lsb, data_op_status;
  data_op_status = k30->i2c_trans.buf[0];
  data_msb = k30->i2c_trans.buf[1];
  data_lsb = k30->i2c_trans.buf[2];
  data_checksum = k30->i2c_trans.buf[3];
  for(int i=0; i<4; i++){
    k30->raw_co2_bytes[i] = k30->i2c_trans.buf[i];
  }
  if (data_checksum == (data_op_status + data_msb + data_lsb)%256){
    // correct checksum -> save the raw measure
    k30->raw_co2 = ((uint16_t)data_msb << 8) | ((uint16_t)data_lsb);
    compensate_co2(k30); //save value if error value stocked 
    k30->debug_flag = 30;
  }
  else
  {
    k30->raw_co2 = ((uint16_t)data_msb << 8) | ((uint16_t)data_lsb); // for debug print even false value
    k30->debug_flag = 31;
  } 
  k30->status = K30_STATUS_READ_ERROR_STATUS;
}

static void parse_error_status(struct K30_I2c *k30)
{
  uint8_t data_checksum, data_error_status, data_op_status;
  data_op_status = k30->i2c_trans.buf[0];
  data_error_status = k30->i2c_trans.buf[1];
  data_checksum = k30->i2c_trans.buf[2];
  // for(int i=0; i<3; i++){ //for debug
  //   k30->raw_error_bytes[i] = k30->i2c_trans.buf[i];
  // }
  if (data_checksum == (data_op_status + data_error_status)%256){
    if (data_error_status != 0){
      k30->last_status = k30->status;
      k30->status = K30_STATUS_ERROR;
    }else{
      if(k30->last_status == K30_STATUS_ERROR){
        k30->status = K30_STATUS_UNINIT;
      }else{
        // nominal case
        k30->status = K30_STATUS_READ_DATA;
      }
    }
  }
}

/**
 *  @brief This internal API is used to parse the calibration data, compensates
 *  it and store it in device structure
 */
static void parse_calib_old(struct K30_I2c *k30)
{
  uint8_t data_checksum, data_msb, data_lsb, data_op_status;
  data_op_status = k30->i2c_trans.buf[0];
  data_msb = k30->i2c_trans.buf[1];
  data_lsb = k30->i2c_trans.buf[2];
  data_checksum = k30->i2c_trans.buf[3];
  if ((data_checksum == (data_op_status + data_msb + data_lsb)%256) && (data_op_status == READ_RAM_COMPLETE)){ 
    // save the measure
    k30->calib.old = ((uint16_t)data_msb << 8) | ((uint16_t)data_lsb);
    k30->status = K30_STATUS_GET_CALIB;
    k30->calibration_status = K30_CALIB_ZERO;
  }
  k30->debug_flag = 32;
}

static void parse_calib_zero(struct K30_I2c *k30)
{
  uint8_t data_checksum, data_msb, data_lsb, data_op_status;
  data_op_status = k30->i2c_trans.buf[0];
  data_msb = k30->i2c_trans.buf[1];
  data_lsb = k30->i2c_trans.buf[2];
  data_checksum = k30->i2c_trans.buf[3];
  if ((data_checksum == (data_op_status + data_msb + data_lsb)%256) && (data_op_status == READ_EEPROM_COMPLETE)){
    // save the measure
    k30->calib.zero = ((uint16_t)data_msb << 8) | ((uint16_t)data_lsb);
    k30->status = K30_STATUS_GET_CALIB;
    k30->calibration_status = K30_CALIB_BCC;
  }
  k30->debug_flag = 33;
}

static void parse_calib_bcc(struct K30_I2c *k30)
{
  uint8_t data_checksum, data_msb, data_lsb, data_op_status;
  data_op_status = k30->i2c_trans.buf[0];
  data_msb = k30->i2c_trans.buf[1];
  data_lsb = k30->i2c_trans.buf[2];
  data_checksum = k30->i2c_trans.buf[3];
  if ((data_checksum == (data_op_status + data_msb + data_lsb)%256) && (data_op_status == READ_EEPROM_COMPLETE)){ 
    // save the measure
    k30->calib.bcc = ((uint16_t)data_msb << 8) | ((uint16_t)data_lsb);
    k30->status = K30_STATUS_GET_CALIB;
    k30->calibration_status = K30_CALIB_ZERO_TRIM;
  }
  k30->debug_flag = 34;
}

#if ZERO_TRIM_READ
static void parse_calib_zero_trim_read(struct K30_I2c *k30)
{
  uint8_t data_checksum, data_msb, data_lsb, data_op_status;
  data_op_status = k30->i2c_trans.buf[0];
  data_msb = k30->i2c_trans.buf[1];
  data_lsb = k30->i2c_trans.buf[2];
  data_checksum = k30->i2c_trans.buf[3];
  if ((data_checksum == (data_op_status + data_msb + data_lsb)%256) && (data_op_status == READ_EEPROM_COMPLETE)){
    // save the measure
    k30->calib.zero_trim = ((int16_t)data_msb << 8) | ((int16_t)data_lsb);
    k30->status = K30_STATUS_CONFIGURE;
  }
  k30->debug_flag = 35;
}
#endif


#if ZERO_TRIM_WRITE
static void parse_calib_zero_trim_write(struct K30_I2c *k30)
{
  uint8_t data_checksum, data_op_status;
  
  data_op_status = k30->i2c_trans.buf[0];
  data_checksum = k30->i2c_trans.buf[1];
   if ((data_checksum == data_op_status) && (data_op_status == WRITE_EEPROM_COMPLETE)){
    k30->status = K30_STATUS_CONFIGURE;
  }
  k30->debug_flag = 36;
}
#endif

/**
 * @brief This internal API is used to compensate the raw temperature data and
 * return the compensated temperature data in double data type.
 */
// static double compensate_temperature(struct K30_I2c *k30)
// {
//   return k30->quant_calib.t_lin;
// }

/**
 * @brief This internal API is used to compensate the raw co2 concentration data and
 * return the compensated co2 concentration data in double data type.
 */
// static double compensate_co2(struct K30_I2c *k30)
// {
//   /* Variable to store the compensated co2 concentration */
//   // double comp_press;
//   k30->co2 = (float) k30->raw_co2;
//   // return comp_press;
// }

static void compensate_co2(struct K30_I2c *k30)
{
  /* Variable to store the compensated co2 concentration */
  k30->co2 = (float) k30->raw_co2;
}

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
 * Senseair K30 CO2 sensor interface
 *
 * This reads the values for CO2 and temperature from the Senseair K30 sensor through I2C.
 */

#include "co2_k30.h"
#include <stdio.h>
#include "modules/ineris/ineris_utils.h"

// not useful until now
// #include "subsystems/abi.h"
// #include "pprzlink/messages.h"
// #include "subsystems/datalink/downlink.h"

/** default slave address */
#ifndef K30_SLAVE_ADDR
#define K30_SLAVE_ADDR K30_I2C_ADDR
#endif

static void co2_k30_print_error(void);

struct K30_I2c co2_k30;

#if LOG_INERIS
static bool log_co2_k30_started;
static void co2_k30_log_data_ascii(void);
#endif


void co2_k30_init(void)
{
  k30_i2c_init(&co2_k30, &K30_I2C_DEV, K30_SLAVE_ADDR);
  #if LOG_INERIS
  log_co2_k30_started = false;
  #endif
}

void co2_k30_periodic(void)
{ 
  k30_i2c_periodic(&co2_k30);
  char str[14] = "co2 meas i2c :";
  print_float_uart(co2_k30.co2, str, 14);
  char raw[14] = "raw meas i2c :";
  print_float_uart(co2_k30.raw_co2, raw, 14);
  // debug comm' details co2_bytes
  // char str_for[3] = " - ";
  // for(int i = 0; i<4; i++){
  //   print_uint8_uart(co2_k30.raw_co2_bytes[i], str_for, 3);
  // }
  if(co2_k30.error_status != 0){
    co2_k30_print_error();
  }
  // debug for calibration  
  print_char_uart("Calib : ---\n", 12);
  print_uint16_uart(co2_k30.calib.old, "old:  ", 6);
  print_uint16_uart(co2_k30.calib.zero, "zero: ", 6);
  print_uint16_uart(co2_k30.calib.bcc, "bcc:  ", 6);
  print_uint16_uart(co2_k30.calib.zero_trim, "z_tr: ", 6);
  print_uint16_uart(co2_k30.calib.background_trim, "b_tr: ", 6);
  print_char_uart( "------------\n", 13);
  
  // debug comm' details error_bytes
  // uart_put_buffer(my_dev, 0, "error bytes:\n", 13);
  // for(int i = 0; i<3; i++){
  //   print_uint8_uart(co2_k30.raw_error_bytes[i], str_for, 3);
  // }

  // Log data
  #if LOG_INERIS
  co2_k30_log_data_ascii();
  #endif
  co2_k30.data_available = false;
}

void co2_k30_event(void)
{
  k30_i2c_event(&co2_k30);

  if (co2_k30.data_available) {
    // uint32_t now_ts = get_sys_time_usec();
    // send ABI message
    // AbiSendMsgBARO_ABS(BARO_K30_SENDER_ID, now_ts, co2_k30.pressure);
    // AbiSendMsgTEMPERATURE(BARO_K30_SENDER_ID, co2_k30.temperature);
    co2_k30.data_available = false;

#ifdef K30_SYNC_SEND
    int32_t uc = (int32_t) co2_k30.raw_co2;
    int32_t ut = (int32_t) co2_k30.raw_temperature;
    int32_t c = (int32_t) co2_k30.co2;
    int32_t t = (int32_t) (10.f * co2_k30.temperature);
    DOWNLINK_SEND_K30_STATUS(DefaultChannel, DefaultDevice, &uc, &ut, &c, &t);
#endif
  }
}

static void co2_k30_print_error(void){
  print_char_uart("Error :", 7);
  char str[] = "";
  print_uint8_uart(co2_k30.error_status, str, 1);
}

#if LOG_INERIS
static void co2_k30_log_data_ascii(void)
{
  if (pprzLogFile != -1){
    if (!log_co2_k30_started){
      char intro[] = "raw_co2(ppm) co2_meas(ppm) old zero bcc z_tr b_tr";
      sdLogWriteLog(pprzLogFile, intro);
      log_co2_k30_started = true;
    }else{
      sdLogWriteLog(pprzLogFile, "%d %f %d %d %d %d %d %d",
                    co2_k30.raw_co2,
                    co2_k30.co2,
                    co2_k30.calib.old,
                    co2_k30.calib.zero,
                    co2_k30.calib.bcc,
                    co2_k30.calib.zero_trim,
                    co2_k30.calib.background_trim,
                    co2_k30.debug_flag);
    }
  } 
}
#endif

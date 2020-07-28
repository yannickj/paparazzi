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

#include "subsystems/abi.h"
#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"

#include <stdio.h>
#include <math.h>

/** default slave address */
#ifndef K30_SLAVE_ADDR
#define K30_SLAVE_ADDR K30_I2C_ADDR
#endif


static void led_static_pattern(int id){
  if (id == 1){
    LED_TOGGLE(2);
    sys_time_msleep(20);
    LED_TOGGLE(2);
    LED_TOGGLE(2);
    sys_time_msleep(20);
    LED_TOGGLE(2);
    sys_time_msleep(20);
    LED_TOGGLE(2);
    sys_time_msleep(20);
    LED_OFF(2);
    return;
  }
  if (id == 2){
    LED_ON(2);
    sys_time_msleep(200);
    LED_OFF(2);
    LED_ON(3);
    sys_time_msleep(200);
    LED_OFF(3);
    LED_ON(2);
    sys_time_msleep(200);
    LED_OFF(2);
    sys_time_msleep(200);
    return;
  }
}


struct uart_periph *my_dev; //ænt

static void print_float_uart(float f, char str[], int str_size){
  char * data = NULL;
  int size = str_size + 2 +1 + 1 +2; // 2 for spaces + (log +1)  for float;
  if (f >= 10){
    size += log10(f);
  }else{
    size ++;
  }
  data = malloc(size  * sizeof(char));
  if (data == NULL){
    led_static_pattern(2);
    return;
  }
  snprintf(data, size, "%s %4.1f\n", str, f);
  uart_put_buffer(my_dev, 0, (uint8_t *) data, size);
  free(data);
}


struct K30_I2c co2_k30;

void co2_k30_init(void)
{
  my_dev = &(INERIS_SENSORS_DEV);
  k30_i2c_init(&co2_k30, &K30_I2C_DEV, K30_SLAVE_ADDR);
}

void co2_k30_periodic(void)
{ 
  led_static_pattern(1);
  char str[14] = "co2 meas i2c :";
  print_float_uart(co2_k30.co2, str, 14);
  char raw[14] = "raw meas i2c :";
  print_float_uart(co2_k30.raw_co2, raw, 14);
  char str_for[3] = " - ";
  for(int i = 0; i<4; i++){
    print_float_uart(co2_k30.raw_co2_bytes[i], str_for, 3);
  }
  k30_i2c_periodic(&co2_k30);
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

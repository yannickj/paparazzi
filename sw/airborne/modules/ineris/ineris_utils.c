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
 * @file modules/ineris/ineris_utils.c
 */

#include "modules/ineris/ineris_utils.h"

struct uart_periph *dev;

void print_float_uart(float f, char str[], int str_size){
  char * data = NULL;
  int size = str_size + 2 + 1 + 1 + 2; // 2 for spaces + (log +1)  for float;
  if (f >= 10){
    size += log10(f);
  }else{
    size ++;
  }
  data = malloc(size  * sizeof(char));
  if (data == NULL){
    led_pattern(2);
    return;
  }
  snprintf(data, size, "%s %4.1f\n", str, f);
  uart_put_buffer(dev, 0, (uint8_t *) data, size);
  free(data);
}

void print_uint8_uart(uint8_t val, char str[], int str_size){
  char * data = NULL;
  int size = str_size + 9 + 2 + 1; // 9 for characters + 2 for (log10 & 'log16') + 1 for \0 ?
  if (val > 0){
    size += log10(val);
  }
  if (val > 16){
    size ++;
  }
  data = malloc(size  * sizeof(char));
  if (data == NULL){
    led_pattern(2);
    return;
  }
  snprintf(data, size, "%s %d (0x: %x)\n", str, val, val);
  uart_put_buffer(dev, 0, (uint8_t *) data, size);
  free(data);
}

void print_uint16_uart(uint16_t val, char str[], int str_size){
  char * data = NULL;
  int size = str_size + 2 + 1 + 1; // 2 for (space + \n) + 1 for the log10 + 1 for \0 ?
  if(val != 0){
    size += log10(val);
  }
  data = malloc(size  * sizeof(char));
  if (data == NULL){
    led_pattern(2);
    return;
  }
  snprintf(data, size, "%s %d\n", str, val);
  uart_put_buffer(dev, 0, (uint8_t *) data, size);
  free(data);
}

void print_char_uart(char str[], int str_size){
  uart_put_buffer(dev, 0, (uint8_t *) str, str_size);
}

void led_pattern(int id){
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

void ineris_utils_init(void){
  dev = &(INERIS_SENSORS_DEV);
}

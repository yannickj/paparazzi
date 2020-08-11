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
 * @file modules/ineris/ineris_utils.h
 *
 * This contains diverse functions (blink, print on uart)...
 */
#ifndef INERIS_UTILS_H
#define INERIS_UTILS_H

#include "std.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "led.h"
#include "mcu_periph/uart.h"
#include "mcu_periph/sys_time_arch.h"



void print_float_uart(float f, char str[], int str_size);
void print_uint8_uart(uint8_t val, char str[], int str_size);
void print_uint16_uart(uint16_t val, char str[], int str_size);
void print_uint32_uart(uint32_t val, char str[], int str_size);
void print_char_uart(char str[], int str_size);
void led_pattern(int id);
void ineris_utils_init(void);

#endif /* INERIS_UTILS_H */
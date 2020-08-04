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
 */

/**
 * @file peripherals/adc161s626.c
 *
 * ADC161S626 module.
 */

#include "adcs/adc161s626.h"
#include "modules/ineris/ineris_utils.h"

struct Adc161s626 adc161s626;

void adc161s626_init(void)
{
  adc161s626_spi_init(&adc161s626.adc, &ADC_161S626_SPI_DEV, ADC_161S626_SPI_SLAVE_IDX);
}

void adc161s626_periodic(void)
{
  adc161s626_spi_periodic(& adc161s626.adc);
}

void adc161s626_event(void)
{
  adc161s626_spi_event(&adc161s626.adc);
  // for debug
  if(adc161s626.adc.data_available){
    print_char_uart("SPI : -----\n", 12);
    print_uint16_uart(adc161s626.adc.data, "adc : ", 6); //data is not unsigned !
    print_uint16_uart(adc161s626.adc.debug_flag, "dbg : ", 6);
    adc161s626.adc.data_available = false;
  }
}

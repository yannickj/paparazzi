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
 * @file peripherals/adc161s626_spi.h
 *
 * ADC161S626 SPI driver.
 */

#ifndef ADC161S626_SPI_H
#define ADC161S626_SPI_H

#include "mcu_periph/spi.h"

#define ADC161S626_BUFFER_LEN 3

struct Adc161s626_Spi{
  struct spi_periph *spi_p;
  struct spi_transaction spi_trans;
  volatile uint8_t tx_buf[2];
  volatile uint8_t rx_buf[ADC161S626_BUFFER_LEN];
  volatile bool data_available;
  int32_t data;
  uint8_t debug_flag;
  bool initialized;
};

extern void adc161s626_spi_init(struct Adc161s626_Spi *adc, struct spi_periph *spi_p, uint8_t addr);
extern void adc161s626_spi_periodic(struct Adc161s626_Spi *adc);
extern void adc161s626_spi_event(struct Adc161s626_Spi *adc);

#endif /* ADC161S626_SPI_H */

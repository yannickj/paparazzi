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
 * @file peripherals/adc161s626_spi.c
 *
 * ADC161S626 SPI driver.
 */

#include "peripherals/adc161s626_spi.h"


void adc161s626_spi_init(struct Adc161s626_Spi *adc, struct spi_periph *spi, uint8_t slave_idx){
  adc->spi_p = spi;
  //configure SPI transaction
  adc->spi_trans.cpol = SPICpolIdleHigh;
  adc->spi_trans.cpha = SPICphaEdge2;
  adc->spi_trans.dss = SPIDss8bit;
  adc->spi_trans.bitorder = SPIMSBFirst;
  adc->spi_trans.cdiv = SPIDiv64;
  adc->spi_trans.select = SPISelectUnselect;
  adc->spi_trans.slave_idx = slave_idx;
  adc->spi_trans.output_length = 0;  // no data to write (no wire)
  adc->spi_trans.input_length = ADC161S626_BUFFER_LEN;
  adc->spi_trans.before_cb = NULL;
  adc->spi_trans.after_cb = NULL;
  adc->spi_trans.input_buf = &(adc->rx_buf[0]);
  adc->spi_trans.output_buf = NULL; // no data to write (no wire)
  
  adc->spi_trans.status = SPITransDone;
  
  adc->data_available = false;
  // no config 
  adc->initialized = true;
  adc->debug_flag = 0;
}

void adc161s626_spi_read(struct Adc161s626_Spi *adc){
  if (adc->initialized && adc->spi_trans.status == SPITransDone) { //always initialized...
    // mpu->spi_trans.input_length = 1 + mpu->config.nb_bytes;
    adc->spi_trans.input_length = ADC161S626_BUFFER_LEN; //no change
    spi_submit(adc->spi_p, &(adc->spi_trans));
  }
}

void adc161s626_spi_periodic(struct Adc161s626_Spi *adc){
  if(adc->initialized){
    adc161s626_spi_read(adc);
    adc->debug_flag = 1;
  }else{
    //adc161s626_spi_start_configure(); // no config...
  }
}

void adc161s626_spi_event(struct Adc161s626_Spi *adc){
  if(adc->initialized){
    // TODO SPITransFailed ?
    if(adc->spi_trans.status == SPITransSuccess){
      // if(bit_is_set(adc->rx_buf[0], 0)){ // first bit that start protocol (0) in buf or not ?
      // }                                     -> 3 bytes instead of 2.
      // new data
      adc->data = adc->rx_buf[0] << 8 | adc->rx_buf[1];
      adc->data_available = true;
      adc->debug_flag = 3;
    }
    adc->spi_trans.status  = SPITransDone;
  }
}

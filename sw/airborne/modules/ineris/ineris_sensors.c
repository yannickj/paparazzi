/*
 * Copyright (C) 2005-2012 The Paparazzi Team
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/**
 * @file modules/ineris/ineris_sensors.c
 * @brief DigiPicco I2C sensor interface
 *
 *   This reads the values for CO and CO2 concentration, temp, etc. from the INERIS sensors such as K30 sensor through UART.
 */
#include "std.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "mcu_periph/uart.h"


#include "modules/ineris/ineris_sensors.h"



// uart periph
struct uart_periph *dev;

void k30_init(){
  // palSetLine(LINE_B07_LED2, HIGH);
  // LED_ON(2);
  // save_in_log_init();
  led_pattern(2);
  dev = &(INERIS_SENSORS_DEV);
}

void k30_periodic(){
  // read output OUT2 1-5V (or 0-5 V) -> should read OUT1 + configure
  float k30_meas = palReadLine(LINE_C01_K30_MEAS);
  // float k30_meas = palReadLine(LINE_D07_K30_MEAS);
  if (k30_meas > 1 && k30_meas < 3){
    LED_ON(2);
  }
  else{
    if (k30_meas < 1){
      LED_ON(3);
    }
    else{
      LED_ON(2);
      LED_ON(3);
    }
  }
  
  // PRINTF("k30_meas : %f\n", k30_meas); non valable : pas simu
  // float lmp50_meas = palReadLine(LINE_C02_LMP50_MEAS);
  // PRINTF("k30_meas : %f\n", lmp50_meas);
  // save_in_log(k30_meas);
  // save_in_log(lmp50_meas);
  // save_in_log(360123.);
  // read UART 

  // read I2C

  // fflush(stdout);

  // led_pattern(2);

  test_uart();

  // led_pattern(2);  
}

void led_pattern(int id){
  if (id == 1){
    LED_TOGGLE(2);
    chThdSleepMilliseconds(20);
    LED_TOGGLE(2);
    chThdSleepMilliseconds(20);
    LED_TOGGLE(2);
    chThdSleepMilliseconds(20);
    LED_TOGGLE(2);
    chThdSleepMilliseconds(20);
    LED_TOGGLE(2);
    chThdSleepMilliseconds(20);
    LED_OFF(2);
    return;
  }
  if (id == 2){
    LED_ON(2);
    chThdSleepMilliseconds(200);
    LED_OFF(2);
    LED_ON(3);
    chThdSleepMilliseconds(200);
    LED_OFF(3);
    LED_ON(2);
    chThdSleepMilliseconds(200);
    LED_OFF(2);
    chThdSleepMilliseconds(200);
    return;
  }
}

void test_uart(){
  // char data[6];
  uint8_t data[6];
  data[0] = 'h';
  data[1] = 'e';
  data[2] = 'l';
  data[3] = 'l';
  data[4] = 'o';
  data[5] = '\n';


  uart_put_buffer(dev, 0, data, 6);
  // uint8_t c = '2';
  // uart_put_byte(dev, 0, c);
}

// output file
/*
char * output_file = "sw/airborne/modules/ineris/sensor_log";
void save_in_log_init(){
  // FILE *f = fopen(output_file, "w");
  // fclose(f);
}

void save_in_log(float data){
  // FILE * f = fopen(output_file, "a");
  char data_s[20];
  snprintf(data_s, 20, "%f", data);
  char buff[21];
  strcpy(buff, data_s);
  strcat(buff, "\n");
  // fputs(buff, f);
  // fclose(f);
}
*/
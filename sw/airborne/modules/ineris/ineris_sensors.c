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
#include <math.h>

#include "mcu_periph/uart.h"
#include "mcu_periph/adc.h"
#include "mcu_periph/sys_time_arch.h"

#include "modules/ineris/ineris_sensors.h"

// #include "peripherals/k30_i2c.h"


// uart periph
struct uart_periph *dev; // for printing on the screen

// measures
union{
  float k30_meas;
  uint8_t meas_int[4];
} k30;

bool k30_adc_sync_send = false;
// static float k30_meas_adc;

#ifndef K30_ADC_NB_SAMPLES
#define K30_ADC_NB_SAMPLES DEFAULT_AV_NB_SAMPLE
#endif

#ifdef K30_ADC_CHANNEL1
static struct adc_buf k30_buf; // OUT1 10 V
#endif
#ifdef K30_ADC_CHANNEL2
static struct adc_buf k30_buf2; // OUT2 5 V
#endif


// command packet
uint8_t cmd_read_CO2[] =  {0xFE, 0X44, 0X00, 0X08, 0X02, 0X9F, 0X25};
uint8_t cmd_read_Temp[] = {0xFE, 0X44, 0X00, 0X12, 0X02, 0X94, 0X45};
uint8_t cmd_read_RH[] =   {0xFE, 0x44, 0x00, 0x14, 0x02, 0x97, 0xE5};
uint8_t cmd_init[] =      {0xFE, 0X41, 0X00, 0X60, 0X01, 0X35, 0XE8, 0x53};
uint8_t response[] = {0,0,0,0,0,0,0};

enum request_t {CO2_REQUEST, TEMP_REQUEST, RH_REQUEST, INIT_REQUEST};

// void print_int_tab_uart(float f, uint8_t *str){
//   // max size : 50;
//   // char data[50];
//   // uint8_t s[14] = "k30 CO2 (ppm):";
//   // snprintf(data, 50, "%s %d %d %d %d\n", str, tab[0], tab[1], tab[2], tab[3]);
//   // snprintf(data, 50, "%s %f\n", str, f);
//   // uart_put_buffer(dev, 0, data, 50);
// }

void print_float_uart(float f, char str[], int str_size){
  char * data = NULL;
  int size = str_size + 2 + 6 + 1; // 2 for spaces + 6 for float
  data = malloc(size  * sizeof(char));
  if (data == NULL){
    return;
  }
  snprintf(data, size, "%s %4.1f\n", str, f);
  uart_put_buffer(dev, 0, (uint8_t *) data, size);
  free(data);
}

void print_int16_uart(uint16_t val, char str[], int str_size){
  char * data = NULL;
  int val_width = 0;
  if (val == 0){
    val_width = 1;
  }
  else{
    val_width = (int)(log10(val)) + 1;
  }
  int size = str_size + 2 + val_width + 1; // 2 for spaces
  data = malloc(size  * sizeof(char));
  if (data == NULL){
    return;
  }
  snprintf(data, size, "%s %d\n", str, val); 
  uart_put_buffer(dev, 0, (uint8_t *) data, size);
  // uart_put_buffer(dev, 0, (const unsigned char) data, size);
  free(data);
}

void k30_adc_init(void){
  #ifdef K30_ADC_CHANNEL1
    adc_buf_channel(K30_ADC_CHANNEL1, &k30_buf, K30_ADC_NB_SAMPLES);
  #endif
  #ifdef K30_ADC_CHANNEL2
    adc_buf_channel(K30_ADC_CHANNEL2, &k30_buf2, K30_ADC_NB_SAMPLES);
  #endif
}

void k30_adc_periodic(void){
  #ifdef K30_ADC_CHANNEL1
  uint16_t adc_raw;
    adc_raw = k30_buf.sum / k30_buf.av_nb_sample;
  char str[] = "adc1 raw :";
  print_int16_uart(adc_raw, str, 10);
  char str1[] = "OUT1 (ppm):";
  float out1 = ((float) adc_raw) * 43 * 50 / 1000;
  print_float_uart(out1, str1, 11);
  #endif

  #ifdef K30_ADC_CHANNEL2
  uint16_t adc_raw2;
    adc_raw2 = k30_buf2.sum / k30_buf2.av_nb_sample;
  char str2[] = "adc2 raw :";
  print_int16_uart(adc_raw2, str2, 10);
  float out2 = ((float) adc_raw2 * 2 - 1) * 1250 / 1000;
  char str3[] = "OUT2 (ppm):";
  print_float_uart(out2, str3, 11);
   
  #endif
  
  
}


void send_request_k30(enum request_t req){
  // while serial available (keep sengind request until k30 send a response)
  switch(req){
    case CO2_REQUEST:
      // uart_put_buffer(k_30_serial, 0, cmd_read_CO2, 7);
      // i2c...
      break;
    case TEMP_REQUEST:
      // uart_put_buffer(k_30_serial, 0, cmd_read_Temp, 7);
      break;
    case RH_REQUEST:
      // uart_put_buffer(k_30_serial, 0, cmd_read_RH, 7);
      break;
    case INIT_REQUEST:
      // uart_put_buffer(k_30_serial, 0, cmd_init, 8);
      break;
  }
  sys_time_msleep(100); //FIXME Deal it with State management instead of sleep

  // uint8_t timeout = 0;
  // wait to get a 7 byte response (arduino : while serial.availabble() < 7 )
  /*{
    timeout++;
    if(timeout > 10){
      //flush serial (Serial.read)
      //break
    }
    sys_time_msleep(50);
  }*/
  sys_time_msleep(250); //FIXME
  
  for(int i = 0; i < 7; i++){
    // response[i] = uart_getch(k_30_serial);
  }
}

uint16_t get_value_k30(uint8_t * response){
  uint8_t high = response[3];
  uint8_t low = response[4];
  uint16_t val = high * 256 + low;
  return val;
}


void k30_init(){
  // palSetLine(LINE_B07_LED2, HIGH);
  // LED_ON(2);
  // save_in_log_init();
  led_pattern(2);
  dev = &(INERIS_SENSORS_DEV);
  // k_30_serial = &(K_30_SERIAL_DEV);
  k30_adc_init();
}

void k30_periodic(){
  k30_adc_periodic();
  // read output OUT2 1-5V (or 0-5 V) -> should read OUT1 + configure
  // k30_meas = palReadLine(LINE_C01_K30_MEAS);
  // k30_meas = palReadLine(LINE_D07_K30_MEAS);
  /*
  if (k30.k30_meas > 1 && k30.k30_meas < 3){
    LED_ON(2);
  }
  else{
    if (k30.k30_meas < 1){
      LED_ON(3);
    }
    else{
      LED_ON(2);
      LED_ON(3);
    }
  }
  */

  // PRINTF("k30_meas : %f\n", k30_meas); non valable : pas simu
  // float lmp50_meas = palReadLine(LINE_C02_LMP50_MEAS);
  // PRINTF("k30_meas : %f\n", lmp50_meas);
  // save_in_log(k30_meas);
  // save_in_log(lmp50_meas);
  // save_in_log(360123.); 

  // read I2C

  // fflush(stdout);

  // led_pattern(2);

  // test_uart();

  
  k30.k30_meas = palReadLine(LINE_A03_K30_MEAS);
  // k30.k30_meas = 103.5;

  // uint8_t str[] = "k30 value :";
  // print_float_uart((k30.k30_meas), str, 11);
  
  // send_request_k30(CO2_REQUEST);
  // uint16_t k_30_value = get_value_k30(response);
  // uint8_t s[14] = "k30 CO2 (ppm):";
  // print_int16_uart(k_30_value, s, 14);
  // print_int_tab_uart(response, s);

  // led_pattern(2);  
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

void test_uart(){
  char s[14] = "K30 CO2 (ppm):";
  print_float_uart(k30.k30_meas, s, 14);

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
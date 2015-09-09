/*
* Copyright (C) 2010 ENAC
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


/* driver for the analog Barometer Mpxa6115 using ADC ads1114 (16 bits I2C 860SpS max) from Texas instruments
 * Navarro & Gorraz & Hattenberger
 */

#include "std.h"
#include "modules/sensors/baro_mcp355x.h"
#include "peripherals/mcp355x.h"
#include "subsystems/abi.h"
#include "led.h"

/** scale factor to convert raw ADC measurement to pressure in Pascal.
 *
 * supply voltage Vs = 5V
 * real sensor sensitivity Vout = Vs * (0.009 P - 0.095) with P in kPa
 * 22 bit signed ADC -> only 21 useful bits ADC = 5/2^21 = 2.384e-6 V / LSB
 *
 * offset = 5*0.095/2.384e-6 = 199229 (LSB)
 * sensitivity = (1000/0.009)*2.384e-6/5 = 0.05298 Pa/LSB
 *
 */
#ifndef MCP355X_BARO_SENS
#define MCP355X_BARO_SENS 0.05298
#endif

#ifndef MCP355X_BARO_OFFSET
#define MCP355X_BARO_OFFSET 199229
#endif

#ifndef BARO_MCP355X_SENDER_ID
#define BARO_MCP355X_SENDER_ID 42
#endif

/** Use an extra median filter to filter baro data
 */
#if USE_BARO_MEDIAN_FILTER
#include "filters/median_filter.h"
struct MedianFilterInt baro_median;
#endif

/* Counter to init mcp355x at startup */
#define BARO_STARTUP_COUNTER 200
static uint16_t startup_cnt;

void baro_mcp355x_init(void)
{
  mcp355x_init();
#if USE_BARO_MEDIAN_FILTER
  init_median_filter(&baro_median);
#endif
  startup_cnt = BARO_STARTUP_COUNTER;
}

void baro_mcp355x_periodic(void)
{
  // Run some loops to get correct readings from the adc
  if (startup_cnt > 0) {
    --startup_cnt;
  }
  // Read the ADC (at 50/4 Hz, conversion time is 68 ms)
  mcp355x_read();
}

void baro_mcp355x_event(void)
{
  mcp355x_event();
  if (mcp355x_data_available) {
    if (startup_cnt == 0) {
      // Send data when init phase is done
#if USE_BARO_MEDIAN_FILTER
      int32_t pressure_raw = MCP355X_BARO_SENS * (mcp355x_data + MCP355X_BARO_OFFSET);
      float pressure = (float)update_median_filter(&baro_median, pressure_raw);
#else
      float pressure = MCP355X_BARO_SENS * (mcp355x_data + MCP355X_BARO_OFFSET);
#endif
      AbiSendMsgBARO_ABS(BARO_MCP355X_SENDER_ID, &pressure);
    }
    mcp355x_data_available = FALSE;
  }
}

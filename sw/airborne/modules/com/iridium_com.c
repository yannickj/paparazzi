/*
 * Copyright (C) 2014 Gautier Hattenberger
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
 * @file modules/com/iridium_com.c
 *
 * Communication using an Iridium Satcom module
 *
 * This module can only work using ChibiOS
 */

#include "std.h"

#include "modules/com/iridium_com.h"
#include "subsystems/chibios-libopencm3/satCom.h"

#include "state.h"
#include "subsystems/gps.h"
#include "subsystems/electrical.h"
#include "generated/airframe.h"
#include "inter_mcu.h"
#include "firmwares/fixedwing/autopilot.h"
#include "subsystems/navigation/common_nav.h"

void iridium_com_init( void ) {
  satcomInit();
  satcomSetPollingInterval(60); // Not sleeping between attempts : more electrical consumption, less data consumption
}

#define FillBufWith32bit(_buf, _index, _value) {  \
  _buf[_index] = (uint8_t) (_value);              \
  _buf[_index+1] = (uint8_t) ((_value) >> 8);     \
  _buf[_index+2] = (uint8_t) ((_value) >> 16);    \
  _buf[_index+3] = (uint8_t) ((_value) >> 24);    \
}

#define FillBufWith16bit(_buf, _index, _value) {  \
  _buf[_index] = (uint8_t) (_value);              \
  _buf[_index+1] = (uint8_t) ((_value) >> 8);     \
}

#define IRIDIUM_BUF_SIZE 23

void iridium_com_periodic( void ) {

  uint8_t buf[IRIDIUM_BUF_SIZE];

  FillBufWith32bit(buf, 0, gps.lla_pos.lat);
  FillBufWith32bit(buf, 4, gps.lla_pos.lon);
  FillBufWith16bit(buf, 8, (int16_t)(gps.lla_pos.alt/1000)); // altitude (meters)
  FillBufWith16bit(buf, 10, gps.gspeed); // ground speed (cm/s)
  FillBufWith16bit(buf, 12, (int16_t)(gps.course/1e4)); // course (1e3rad)
  FillBufWith16bit(buf, 14, (uint16_t)((*stateGetAirspeed_f())*100)); // TAS (cm/s)
  buf[16] = electrical.vsupply; // decivolts
  buf[17] = (uint8_t)(energy/100); // deciAh
  buf[18] = (uint8_t)(ap_state->commands[COMMAND_THROTTLE]*100/MAX_PPRZ);
  buf[19] = pprz_mode;
  buf[20] = nav_block;
  FillBufWith16bit(buf, 21, autopilot_flight_time);
  satcomSendBuffer(buf, IRIDIUM_BUF_SIZE);

}

void iridium_com_event(void)
{
}


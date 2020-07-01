/*
 * Copyright (C) Murat BRONZ
 *
 * This file is part of paparazzi
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
 * @file "modules/fault/fault.c"
 * @author Murat BRONZ
 * Generates faults on the actuators
 */

#include "modules/fault/fault.h"

#if FLIGHTRECORDER_SDLOG
#include "subsystems/datalink/telemetry.h"
#include "modules/loggers/pprzlog_tp.h"
#include "modules/loggers/sdlog_chibios.h"
#endif

// #include "modules/datalink/extra_pprz_dl.h"
// #include "subsystems/datalink/telemetry.h"
#include <string.h>
#include "generated/airframe.h"
#include "subsystems/datalink/datalink.h"
#include "subsystems/datalink/downlink.h"

float fault_right;
float fault_left;
float fault_offset_right;
float fault_offset_left;
int flight_status;

static void fault_downlink(struct transport_tx *trans, struct link_device *dev){
  pprz_msg_send_FAULT_TELEMETRY(trans, dev, AC_ID, &flight_status);
}

void fault_init(void) {
fault_right = 1.0;
fault_left  = 1.0; 
fault_offset_left  = 0.0; 
fault_offset_right  = 0.0; 

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_FAULT_TELEMETRY, fault_downlink);
#endif
}

void fault_Set_Right(float _v)
{
  fault_right = _v;
#if FLIGHTRECORDER_SDLOG
          if (flightRecorderLogFile != -1) {
            DOWNLINK_SEND_SETTINGS(pprzlog_tp, flightrecorder_sdlog,
                &fault_right, &fault_left, &fault_offset_left, &fault_offset_right);
          }
#endif
}

void fault_Set_Left(float _v)
{
  fault_left = _v;
#if FLIGHTRECORDER_SDLOG
          if (flightRecorderLogFile != -1) {
            DOWNLINK_SEND_SETTINGS(pprzlog_tp, flightrecorder_sdlog,
                &fault_right, &fault_left, &fault_offset_left, &fault_offset_right);
          }
#endif
}

void fault_Set_Offset_Right(float _v)
{
  fault_offset_right = _v;
#if FLIGHTRECORDER_SDLOG
          if (flightRecorderLogFile != -1) {
            DOWNLINK_SEND_SETTINGS(pprzlog_tp, flightrecorder_sdlog,
                &fault_right, &fault_left, &fault_offset_left, &fault_offset_right);
          }
#endif
}

void fault_Set_Offset_Left(float _v)
{
  fault_offset_left = _v;
#if FLIGHTRECORDER_SDLOG
          if (flightRecorderLogFile != -1) {
            DOWNLINK_SEND_SETTINGS(pprzlog_tp, flightrecorder_sdlog,
                &fault_right, &fault_left, &fault_offset_left, &fault_offset_right);
          }
#endif
}

void fault_parse_FAULT_INFO(uint8_t *buf)
{
  flight_status =  DL_FAULT_INFO_info(buf);
}

void fault_status_report(void)
{
  // DOWNLINK_SEND_FAULT_TELEMETRY()
  fault_downlink(&(DefaultChannel).trans_tx, &(DefaultDevice).device);
}

// void fault_periodic() {}
// void fault_datalink_callback() {}



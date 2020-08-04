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
 * @file modules/sensors/co_lmp91000.c
 * LMP91000 sensor interface
 *
 * This reads the values for *** from the *** sensor through I2C.
 */

#include "co_lmp91000.h"

#include "subsystems/abi.h"
#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"
#include "modules/ineris/ineris_utils.h"

/** default slave address */
#ifndef LMP91000_SLAVE_ADDR
#define LMP91000_SLAVE_ADDR LMP91000_I2C_ADDR
#endif

struct Lmp91000_I2c co_lmp91000;

void co_lmp91000_init(void)
{
  lmp91000_i2c_init(&co_lmp91000, &LMP91000_I2C_DEV, LMP91000_SLAVE_ADDR);
}

void co_lmp91000_periodic(void)
{
  lmp91000_i2c_periodic(&co_lmp91000);
  print_float_uart(co_lmp91000.co, "lmp co meas : ", 14);
}

void co_lmp91000_event(void)
{
  lmp91000_i2c_event(&co_lmp91000);

  if (co_lmp91000.data_available) {
    // uint32_t now_ts = get_sys_time_usec();
    // send ABI message
    // AbiSendMsgBARO_ABS(BARO_LMP91000_SENDER_ID, now_ts, co_lmp91000.pressure);
    // AbiSendMsgTEMPERATURE(BARO_LMP91000_SENDER_ID, co_lmp91000.temperature);
    co_lmp91000.data_available = false;

#ifdef LMP91000_SYNC_SEND
    // int32_t uc = (int32_t) co_lmp91000.raw_co2;
    // int32_t ut = (int32_t) co_lmp91000.raw_temperature;
    // int32_t c = (int32_t) co_lmp91000.co2;
    // int32_t t = (int32_t) (10.f * co_lmp91000.temperature);
    DOWNLINK_SEND_LMP91000_STATUS(DefaultChannel, DefaultDevice, &uc, &ut, &c, &t);
#endif
  }
}

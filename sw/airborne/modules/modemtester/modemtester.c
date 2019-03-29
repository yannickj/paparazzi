/*
 * Copyright (C) Fabien Garcia <fabien.garcia@enac.fr>
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
 * @file "modules/modemtester/modemtester.c"
 * @author ls
 * This module uses a serial line to test a modem. It requires a ground program in order to drive the tests.
 */

#include <ch.h>
#include "modules/modemtester/modemtester.h"
#include "pprzlink/messages.h"
#include "subsystems/datalink/telemetry.h"
#include "mcu_periph/uart.h"
#include "fsm.h"
#include <stdlib.h>

struct uart_periph* modem_tester_dev=&MODEMTEST_UART_DEV;

static THD_WORKING_AREA(wa_modem_tester, 1024);
static __attribute__((noreturn)) void modem_tester (void *arg)
{
  (void) arg;
  chRegSetThreadName("modem_tester");

  struct pprzlink_msg msg;

  msg.trans = &(DefaultChannel).trans_tx;
  msg.dev = &(DefaultDevice).device;
  msg.sender_id = AC_ID;
  msg.receiver_id = 0; // To ground
  msg.component_id = 0;
  uint8_t data[4];

  data[0]=(UART6_BAUD>>24)&0xFF;
  data[1]=(UART6_BAUD>>16)&0xFF;
  data[2]=(UART6_BAUD>>8)&0xFF;
  data[3]=UART6_BAUD&0xFF;
  
  pprzlink_msg_v2_send_DEBUG(&msg,4,data);
  
  while(true) {
    if (uart_char_available(modem_tester_dev)) {
      uint8_t command=uart_getch(modem_tester_dev);
#if SEND_DEBUG_MSG
      if (command >= end_standard_codes) {
	data[0]='D';
	data[1]=command;
	data[2]=getCurrentStateId();
	pprzlink_msg_v2_send_DEBUG(&msg,3,data);
      }
      dispatch(command);
#endif
    }
    else {
      chThdSleepMilliseconds(1);
    }
  }
}

void init_modemtester() 
{
  
  PPRZ_MUTEX_INIT(modemtester_mutex);
  chThdCreateStatic(wa_modem_tester, sizeof(wa_modem_tester), NORMALPRIO + 2 , modem_tester, NULL);
}



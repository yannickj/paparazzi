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

#include "modules/modemtester/modemtester.h"
#include "pprzlink/messages.h"
#include "subsystems/datalink/telemetry.h"
#include <ch.h>
#include "mcu_periph/uart.h"

static THD_WORKING_AREA(wa_modem_tester, 1024);

struct uart_periph* dev=&MODEMTEST_UART_DEV;

static __attribute__((noreturn)) void modem_tester (void *arg)
{
    (void) arg;
    chRegSetThreadName("modem_tester");
    uint8_t data='Z';
    
    struct pprzlink_msg msg;

    msg.trans = &(DefaultChannel).trans_tx;
    msg.dev = &(DefaultDevice).device;
    msg.sender_id = AC_ID;
    msg.receiver_id = 0; // To ground
    msg.component_id = 0;
    
    while(true) {
        if (uart_char_available(dev)) {
            data=uart_getch(dev);
            uart_put_buffer(dev,0,&data,1);
            pprzlink_msg_v2_send_DEBUG(&msg,1,&data);
        }
        else {
            chThdSleepMilliseconds(500);
        }
    }

}

void init_modem() 
{
    chThdCreateStatic(wa_modem_tester, sizeof(wa_modem_tester), NORMALPRIO + 2 , modem_tester, NULL);
}



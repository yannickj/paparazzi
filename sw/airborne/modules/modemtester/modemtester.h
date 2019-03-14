/*
 * Copyright (C)  Fabien Garcia <fabien.garcia@enac.fr>
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
 * @file "modules/modemtester/modemtester.h"
 * @author ls
 * This module uses a serial line to test a modem. It requires a ground program in order to drive the tests.
 */
/**
 * @file /modules/modemtester/modemtester.h
 * Main header file of module modemtester
 */

#ifndef MODEMTESTER_H
#define MODEMTESTER_H

extern struct uart_periph* modem_tester_dev;

extern void init_modemtester(void);

#define SWAP_ENDIAN32(x) ( (((x)>>24) & 0x000000FFL) \
			    | (((x)>>8)  & 0x0000FF00L) \
			    | (((x)<<8)  & 0x00FF0000L) \
			    | (((x)<<24) & 0xFF000000L) )

#endif


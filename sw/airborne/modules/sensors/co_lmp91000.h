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
 * @file modules/sensors/lmp91000.c
 * LMP91000 sensor interface
 *
 * This reads the values for *** from the *** sensor through I2C.
 */

#ifndef CO_LMP91000_H
#define CO_LMP91000_H

#include "peripherals/lmp91000_i2c.h"

extern struct Lmp91000_I2c co_lmp91000;

void co_lmp91000_init(void);
void co_lmp91000_periodic(void);
void co_lmp91000_event(void);

#endif

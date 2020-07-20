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
 * @file modules/sensors/co2_k30.h
 * Senseair K30 CO2 sensor interface
 *
 * This reads the values for CO2 and temperature from the Senseair K30 sensor through I2C.
 */

#ifndef CO2_K30_H
#define CO2_K30_H

#include "peripherals/k30_i2c.h"

extern struct K30_I2c co2_k30;

void co2_k30_init(void);
void co2_k30_periodic(void);
void co2_k30_event(void);

#endif

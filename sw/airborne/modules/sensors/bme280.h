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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/sensors/bme280.h
 */

#ifndef BME280_H
#define BME280_H

/*#ifndef LOG_INERIS
#define LOG_INERIS TRUE
#endif*/

#include "peripherals/bme280_i2c_spi.h"
#include "modules/ineris/ineris_utils.h"

extern struct Bme280_I2c_Spi bme280;

void bme280_init(void);
void bme280_periodic(void);
void bme280_event(void);

/*#if LOG_INERIS
#include "modules/loggers/sdlog_chibios.h"
#endif*/
#endif /* BME280_H */

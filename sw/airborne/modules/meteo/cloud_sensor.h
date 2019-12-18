/*
 * Copyright (C) 2019 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *                    Titouan Verdu <titouan.verdu@enac.fr>
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
 */

/**
 * @file "modules/meteo/cloud_sensor.h"
 *
 * Get data from Cloud Sensor
 * - compute Liquid Water Content (LWC) value from PAYLOAD_FLOAT data
 * - get already computed LWC from PAYLOAD_COMMAND data
 */

#ifndef CLOUD_SENSOR_H
#define CLOUD_SENSOR_H

#include "std.h"

/**
 * variables for settings
 */
extern bool cloud_sensor_compute_lwc;
extern float cloud_sensor_threshold;

/** Init function
 */
extern void cloud_sensor_init(void);

/** New message/data callback
 */
extern void cloud_sensor_callback(uint8_t *buf);
extern void LWC_callback(uint8_t *buf);

/** Report function
 */
extern void cloud_sensor_report(void);

#endif


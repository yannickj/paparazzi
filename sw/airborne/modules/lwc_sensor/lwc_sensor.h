/*
 * Copyright (C) VERDU Titouan
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/lwc_sensor/lwc_sensor.h"
 * @author VERDU Titouan
 * This module will do some action on the flight plan according to the value of LWC given by the python agent basedon the MesoNh cloud simulation.
 */

#include <stdio.h>
#include <stdint.h>


#ifndef LWC_SENSOR_H
#define LWC_SENSOR_H

extern void lwc_sensor_init(void);
extern void LWC_sensor_in_out_callback(void);

#endif


/*
 * Copyright (C) 2018-2019 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *                         Titouan Verdu <titouan.verdu@enac.fr>
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
 * @file "modules/meteo/lwc_sim.h"
 * @author VERDU Titouan
 *
 * This module fetch the Liquid Water Content (LWC) given by the python agent basedon the MesoNh cloud simulation.
 * Since data are received from datalink, it can be used in real flight with a virtual cloud.
 */

#ifndef LWC_SIM_H
#define LWC_SIM_H

#include "std.h"
#include "math/pprz_geodetic_float.h"

/** LWC struture
 */
struct LWCSim {
  float value;          ///< current LWC value
  struct EnuCoor_f pos; ///< position of the last measure
  float time;           ///< time of the last measure
  bool inside_cloud;    ///< in/out status
};

extern struct LWCSim lwc_sim;

/** Init function
 */
extern void lwc_sim_init(void);

/** New data callback function
 */
extern void lwc_sim_msg_callback(void);

#endif


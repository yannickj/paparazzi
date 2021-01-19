/*
 * Copyright (C) 2021 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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

/** @file "modules/meteo/cloud_sim.c"
 * @author Gautier Hattenberger <gautier.hattenberger@enac.fr>
 * Basic cloud simulation for testing adaptive navigation patterns
 */

#include "modules/meteo/cloud_sim.h"
#include "subsystems/navigation/common_nav.h"
#include "generated/flight_plan.h"
#include "subsystems/abi.h"
#include "state.h"

// default radius in WP mode
#ifndef CLOUD_SIM_RADIUS
#define CLOUD_SIM_RADIUS 150.f
#endif

#ifdef CLOUD_SIM_WP_POLYGON
static uint8_t cloud_sim_polygon[] = CLOUD_SIM_WP_POLYGON;
#endif

#ifdef CLOUD_SIM_WP_SPEED
static struct FloatVect2 cloud_sim_speed[] = CLOUD_SIM_WP_SPEED;
#else
static struct FloatVect2 cloud_sim_speed[] = {0};
#endif

#if !(defined InsideCloud) && (defined CLOUD_SIM_WP_POLYGON)
#pragma error "You must define a Cloud sector in your flight plan"
#endif

struct CloudSim cloud_sim;

/*********************
 * Utility functions *
 *********************/

static float distance_to_wp(struct EnuCoor_f * pos, uint8_t id)
{
  if (id < nb_waypoint) {
    struct FloatVect2 diff = { pos->x - waypoints[id].x, pos->y - waypoints[id].y };
    return float_vect2_norm(&diff);
  } else {
    return -1.f; // invalid WP id
  }
}

void cloud_sim_init(void)
{
  cloud_sim.reset = false;
#if (defined CLOUD_SIM_WP_ID) && (CLOUD_SIM_WPS_NB > 0)
  cloud_sim.mode = CLOUD_SIM_WP;
  cloud_sim.ids[0] = CLOUD_SIM_WP_ID;
#elif (defined CLOUD_SIM_WP_POLYGON)
  cloud_sim.mode = CLOUD_SIM_POLYGON;
  for (int i = 0; i < CLOUD_SIM_WPS_NB; i++) {
    cloud_sim.ids[i] = cloud_sim_polygon[i];
  }
#else
#pragma error "CLOUD_SIM: wrong WP ou Polygon configuration"
#endif
  for (int i = 0; i < CLOUD_SIM_WPS_NB; i++) {
    cloud_sim.speed[i] = cloud_sim_speed[i];
  }
}

void cloud_sim_periodic(void)
{
  // your periodic code here.
  // freq = 1.0 Hz

  uint32_t stamp = get_sys_time_usec();
  struct EnuCoor_f * pos = stateGetPositionEnu_f();
  uint8_t inside = 0; // 1: inside, 0: outside

  switch (cloud_sim.mode) {
    case CLOUD_SIM_WP:
      // Test the distance to the reference waypoint
      if (distance_to_wp(pos, cloud_sim.ids[0]) > CLOUD_SIM_RADIUS) {
        inside = 0;
      } else {
        inside = 1;
      }
      AbiSendMsgPAYLOAD_DATA(CLOUD_SENSOR_ID, stamp, 1 /* CLOUD_BORDER */, 1, &inside);
      break;
#ifdef InsideCloud
    case CLOUD_SIM_POLYGON:
      inside = (uint8_t) InsideCloud(pos->x, pos->y);
      AbiSendMsgPAYLOAD_DATA(CLOUD_SENSOR_ID, stamp, 1 /* CLOUD_BORDER */, 1, &inside);
      break;
#endif
    default:
      break;
  }
}



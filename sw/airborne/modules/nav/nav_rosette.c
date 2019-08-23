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
 * @file modules/nav/nav_rosette.c
 *
 * Adaptive flower pattern for cloud exploration
 */

#include "modules/nav/nav_rosette.h"

#include "firmwares/fixedwing/nav.h"
#include "state.h"
#include "autopilot.h"
#include "generated/flight_plan.h"

enum RosetteStatus {
  RSTT_ENTER,
  RSTT_CROSSING,
  RSTT_TURNING
};

struct NavRosette {
  enum RosetteStatus status;
  bool inside_cloud;
  struct EnuCoor_f target;
  struct EnuCoor_f circle;
  float radius;
};

static struct NavRosette nav_rosette;

#if USE_MISSION
#include "modules/mission/mission_common.h"

static bool nav_rosette_mission(uint8_t nb, float *params, enum MissionRunFlag flag)
{
  if (nb != 2) {
    return false; // wrong number of parameters
  }
  if (flag == MissionInit) {
    struct EnuCoor_f start = { params[0], params[1], params[2] };
    nav_rosette_setup(start);
  }
  return nav_rosette_run();
}
#endif

// ABI message

#ifndef NAV_ROSETTE_LWC_ID
#define NAV_ROSETTE_LWC_ID ABI_BROADCAST
#endif

static abi_event lwc_ev;

static lwc_cb(uint8_t sender_id UNUSED, uint32_t stamp UNUSED, int32_t data_type, uint32_t size, uint8_t * data) {
  if (data_type == 1 && size == 1) {
    nav_rosette.inside_cloud = (bool) data[0];
  }
}


void nav_rosette_init(void)
{
  nav_rosette.status = RSTT_ENTER;
  nav_rosette.radius = DEFAULT_CIRCLE_RADIUS;
  nav_rosette.inside_cloud = false;

  AbiBindMsg(NAV_ROSETTE_LWC_ID, &lwc_ev, lwc_cb);

#if USE_MISSION
  mission_register(nav_rosette_mission, "RSTT");
#endif
}

void nav_rosette_setup(struct EnuCoor_f start_point)
{
  nav_rosette.target = start_point;
  nav_rosette.status = RSTT_ENTER;
  nav_rosette.inside_cloud = false;
}

bool nav_rosette_run(void)
{
  switch (nav_rosette.status) {
    case RSTT_ENTER:
      break;
    case RSTT_CROSSING:
      break;
    case RSTT_TURNING:
      break;
    default:
      // error, leaving
      return false;
  }

  return true;
}

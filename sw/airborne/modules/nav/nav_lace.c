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
 * @file modules/nav/nav_lace.c
 *
 * Adaptive border pattern for cloud exploration
 */

#include "modules/nav/nav_lace.h"

#include "firmwares/fixedwing/nav.h"
#include "state.h"
#include "autopilot.h"
#include "generated/flight_plan.h"

enum LaceStatus {
  LACE_ENTER,
  LACE_INSIDE,
  LACE_OUTSIDE
};

struct NavLace {
  enum LaceStatus status;
  bool inside_cloud;
  struct EnuCoor_f target;
  struct EnuCoor_f circle;
  float radius;
};

static struct NavLace nav_lace;

#if USE_MISSION
#include "modules/mission/mission_common.h"

static bool nav_lace_mission(uint8_t nb, float *params, bool init)
{
  if (nb != 2) {
    return false; // wrong number of parameters
  }
  if (init) {
    struct EnuCoor_f start = { params[0], params[1], params[2] };
    nav_lace_setup(start);
  }
  return nav_lace_run();
}
#endif


void nav_lace_init(void)
{
  nav_lace.status = LACE_ENTER;
  nav_lace.radius = DEFAULT_CIRCLE_RADIUS;
  nav_lace.inside_cloud = false;

  // TODO bind to ABI message

#if USE_MISSION
  mission_register(nav_lace_mission, "LACE");
#endif
}

void nav_lace_setup(struct EnuCoor_f start_point)
{
  nav_lace.target = start_point;
  nav_lace.status = LACE_ENTER;
  nav_lace.inside_cloud = false;
}

bool nav_lace_run(void)
{
  switch (nav_lace.status) {
    case LACE_ENTER:
      break;
    case LACE_INSIDE:
      break;
    case LACE_OUTSIDE:
      break;
    default:
      // error, leaving
      return false;
  }

  return true;
}

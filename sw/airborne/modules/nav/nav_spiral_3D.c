/*
 * Copyright (C) 2019 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file modules/nav/nav_spiral_3D.c
 *
 * Fixedwing navigation in a 3D spiral.
 *
 */

#include "modules/nav/nav_spiral_3D.h"

#include "firmwares/fixedwing/nav.h"
#include "state.h"
#include "autopilot.h"
#include "generated/flight_plan.h"

#ifndef NAV_SPIRAL_3D_DIST_DIFF
#define NAV_SPIRAL_3D_DIST_DIFF 10.f // horizontal distance difference before starting
#endif

#ifndef NAV_SPIRAL_3D_ALT_DIFF
#define NAV_SPIRAL_3D_ALT_DIFF 10.f // vertical distance difference before starting
#endif

#ifndef NAV_SPIRAL_3D_MIN_CIRCLE_RADIUS
#define NAV_SPIRAL_3D_MIN_CIRCLE_RADIUS 60.f
#endif

enum Spiral3DStatus { Spiral3DOutside, Spiral3DStartCircle, Spiral3DCircle };

struct NavSpiral3D {
  struct FloatVect3 center;
  struct FloatVect3 pos_incr;
  float radius;
  float radius_min;
  float radius_start;
  float radius_stop;
  float radius_increment;
  enum Spiral3DStatus status;
};

struct NavSpiral3D nav_spiral_3D;

static const float nav_dt = 1.f / NAVIGATION_FREQUENCY;

#if USE_MISSION
#include "modules/mission/mission_common.h"

static bool nav_spiral_3D_mission(uint8_t nb, float *params, enum MissionRunFlag flag)
{
}
#endif

void nav_spiral_3D_init(void)
{
  // no speed by default
  FLOAT_VECT3_ZERO(nav_spiral_3D.speed);

#if USE_MISSION
  mission_register(nav_spiral_3D_mission, "SPIR3");
#endif
}

void nav_spiral_3D_setup(float center_x, float center_y,
                         float alt_start, float alt_stop,
                         float radius_start, float radius_stop,
                         float vx, float vy, float vz)
{
  // initial center position
  VECT3_ASSIGN(nav_spiral_3D.center, center_x, center_y, alt_start);
  // increment based on speed
  VECT3_ASSIGN(nav_spiral_3D.pos_incr, vx*nav_dt, vy*nav_dt, vz*nav_dt);
  nav_spiral_3D.radius_start = radius_start;
  nav_spiral_3D.radius_min = NAV_SPIRAL_MIN_CIRCLE_RADIUS;
  if (nav_spiral_3D.radius_start < nav_spiral_3D.radius_min) {
    nav_spiral_3D.radius_start = nav_spiral_3D.radius_min;
  }
  nav_spiral_3D.radius_stop = radius_stop;
  if (nav_spiral_3D.radius_stop < nav_spiral_3D.radius_min) {
    nav_spiral_3D.radius_stop = nav_spiral_3D.radius_min;
  }
  // Compute radius increment
  // Rinc = deltaR / deltaT
  // deltaT = deltaZ / Vz
  float deltaZ = alt_stop - alt_start;
  if (fabsf(deltaZ) < 1.f) {
    // alt diff is too small, use Vz as increment rate at fix altitude
    nav_spiral_3D.radius_increment = x;
}

bool nav_spiral_3D_run(void)
{
  struct EnuCoor_f pos_enu = *stateGetPositionEnu_f();

  struct FloatVect2 pos_diff;
  VECT2_DIFF(pos_diff, pos_enu, nav_spiral_3D.center);
  float dist_to_center = float_vect2_norm(&pos_diff);
  float dist_diff, alt_diff;

  switch (nav_spiral_3D.status) {
    case Spiral3DStart:
      // fly to start circle until dist to center and alt are acceptable
      // flys until center of the helix is reached and start helix
      VECT2_DIFF(pos_diff, pos_enu, nav_spiral_3D.center);
      dist_diff = fabs(dist_to_center - nav_spiral_3D.radius_start);
      alt_diff = fabs(stateGetPositionUtm_f()->alt - alt_start);
      if (dist_diff < NAV_SPIRAL_3D_DIST_DIFF && alt_start < NAV_SPIRAL_3D_ALT_DIFF) {
        nav_spiral_3D.status = Spiral3DCircle;
      }
      break;

    case SpiralCircle:
      // increment center position
      VECT3_ADD(nav_spiral_3D.center, nav_spiral_3D.pos_incr);
      // increment radius
      nav_spiral_3D.radius += nav_spiral_3D.radius_increment;
      // test end condition
      dist_diff = fabs(dist_to_center - nav_spiral_3D.radius_stop);
      alt_diff = fabs(stateGetPositionUtm_f()->alt - alt_stop);
      if (dist_diff < NAV_SPIRAL_3D_DIST_DIFF && alt_start < NAV_SPIRAL_3D_ALT_DIFF) {
        // reaching desired altitude and radius
        return false; // spiral is finished
      }
      break;

    default:
      break;
  }

  // horizontal control setting
  nav_circle_XY(nav_spiral_3D.center.x, nav_spiral_3D.center.y, nav_spiral_3D.radius);
  // vertical control setting
  NavVerticalAutoThrottleMode(0.); /* No pitch */
  NavVerticalAltitudeMode(nav_spiral_3D.center.z, 0.); /* No preclimb */

  return true;
}

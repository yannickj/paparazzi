/*
 * Copyright (C) Titouan Verdu
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
/**
 * @file "modules/nav/nav_trinity.c"
 * @author Titouan Verdu
 * Adaptative trinity pattern for cloud exploration.
 */

#include "modules/nav/nav_trinity.h"

#include "firmwares/fixedwing/nav.h"
#include "state.h"
#include "autopilot.h"
#include "generated/flight_plan.h"
#include "subsystems/abi.h"

enum TrinityStatus {
  TRINITY_ENTER,
  TRINITY_INSIDE,
  TRINITY_OUTSIDE
};

enum RotationDir {
  TRINITY_LEFT,
  TRINITY_RIGHT
};

struct NavTrinity {
  enum TrinityStatus status;
  enum RotationDir rotation;
  bool inside_cloud;
  struct EnuCoor_f actual;
  struct EnuCoor_f target;
  struct EnuCoor_f circle;
  struct FloatVect3 pos_incr;
  float direction;
  float radius;
  float radius_sign;
};

static struct NavTrinity nav_trinity;

static const float nav_dt = 1.f / NAVIGATION_FREQUENCY;

static float change_rep(float dir)
{
  return M_PI_2 - dir;
}

static struct EnuCoor_f process_new_point_trinity(struct EnuCoor_f *position, float alt_sp, float uav_direction)
{
  struct EnuCoor_f new_point;
  float rot_angle;

  if (nav_trinity.rotation == TRINITY_RIGHT) {
    rot_angle = -M_PI_2;
  } else{
    rot_angle = M_PI_2;
  }

  new_point.x = position->x + (cos(rot_angle + uav_direction) * nav_trinity.radius);
  new_point.y = position->y + (sin(rot_angle + uav_direction) * nav_trinity.radius);
  new_point.z = alt_sp;

  return new_point;
}

static struct EnuCoor_f process_first_point_trinity(struct EnuCoor_f *position, float alt_sp, float uav_direction)
{
  struct EnuCoor_f new_point;
  float rot_angle;

  if (nav_trinity.rotation == TRINITY_RIGHT) {
    rot_angle = -M_PI_2;
    nav_trinity.rotation = TRINITY_LEFT;
  } else{
    rot_angle = M_PI_2;
    nav_trinity.rotation = TRINITY_RIGHT;
  }

  new_point.x = position->x + (cos(rot_angle + uav_direction) * nav_trinity.radius);
  new_point.y = position->y + (sin(rot_angle + uav_direction) * nav_trinity.radius);
  new_point.z = alt_sp;

  return new_point;
}

#if USE_MISSION
#include "modules/mission/mission_common.h"

static bool nav_trinity_mission(uint8_t nb, float *params, enum MissionRunFlag flag)
{
  if (flag == MissionInit && nb == 8) {
    float start_x = params[0];
    float start_y = params[1];
    float start_z = params[2];
    int first_turn = params[3];
    float circle_radius = params[4];
    float vx = params[5];
    float vy = params[6];
    float vz = params[7];
    nav_trinity_setup(start_x, start_y, start_z, first_turn, circle_radius, vx, vy, vz);
    return true;
  }
  else if (flag == MissionUpdate && nb == 2) {
    // update horizontal speed
    float vx = params[0];
    float vy = params[1];
    nav_trinity.pos_incr.x = vx * nav_dt;
    nav_trinity.pos_incr.y = vy * nav_dt;
    return true;
  }
  else if (flag == MissionUpdate && nb == 1) {
    // update vertical speed
    float vz = params[0];
    nav_trinity.pos_incr.z = vz * nav_dt;
    return true;
  }
  else if (flag == MissionRun) {
    return nav_trinity_run();
  }
  return false; // not a valid case
}
#endif

// ABI message

#ifndef NAV_TRINITY_LWC_ID
#define NAV_TRINITY_LWC_ID ABI_BROADCAST
#endif

static abi_event lwc_ev;

static void lwc_cb(uint8_t sender_id UNUSED, uint32_t stamp UNUSED, int32_t data_type, uint32_t size, uint8_t * data) {
  if (data_type == 1 && size == 1) {
    nav_trinity.inside_cloud = (bool) data[0];
  }
}

void nav_trinity_init(void)
{
  nav_trinity.status = TRINITY_ENTER;
  nav_trinity.radius = DEFAULT_CIRCLE_RADIUS;
  nav_trinity.inside_cloud = false;

  AbiBindMsgPAYLOAD_DATA(NAV_TRINITY_LWC_ID, &lwc_ev, lwc_cb);

#if USE_MISSION
  mission_register(nav_trinity_mission, "TRNTY");
#endif
}

void nav_trinity_setup(float init_x, float init_y,
                    float init_z, int turn,
                    float desired_radius, float vx,
                    float vy, float vz)
{
  struct EnuCoor_f start = {init_x, init_y, init_z};
  // increment based on speed
  VECT3_ASSIGN(nav_trinity.pos_incr, vx*nav_dt, vy*nav_dt, vz*nav_dt);

  nav_trinity.target = start;
  nav_trinity.status = TRINITY_ENTER;
  nav_trinity.inside_cloud = false;
  nav_trinity.radius = desired_radius;

  if (turn == 1) {
    nav_trinity.rotation = TRINITY_RIGHT;
    nav_trinity.radius_sign = 1.0f;
  } else {
    nav_trinity.rotation = TRINITY_LEFT;
    nav_trinity.radius_sign = -1.0f;
  }

  nav_trinity.actual = *stateGetPositionEnu_f();
}

bool nav_trinity_run(void)
{
  float pre_climb = 0.f;

  NavVerticalAutoThrottleMode(0.f); /* No pitch */

  switch (nav_trinity.status) {
    case TRINITY_ENTER:
      nav_route_xy(nav_trinity.actual.x, nav_trinity.actual.y, nav_trinity.target.x, nav_trinity.target.y);
      NavVerticalAltitudeMode(nav_trinity.target.z + ground_alt, pre_climb);

      if (nav_trinity.inside_cloud) {
        nav_trinity.status = TRINITY_INSIDE;
        nav_trinity.actual = *stateGetPositionEnu_f();
        nav_trinity.direction = change_rep(stateGetHorizontalSpeedDir_f());
        nav_trinity.circle = process_first_point_trinity(&nav_trinity.actual, nav_trinity.target.z, nav_trinity.direction);
      }
      break;
    case TRINITY_INSIDE:
      // increment center position
      VECT3_ADD(nav_trinity.circle, nav_trinity.pos_incr);
      nav_circle_XY(nav_trinity.circle.x, nav_trinity.circle.y , nav_trinity.radius_sign * nav_trinity.radius);
      NavVerticalAltitudeMode(nav_trinity.circle.z + ground_alt, pre_climb);

      if (!nav_trinity.inside_cloud) {
        nav_trinity.status = TRINITY_OUTSIDE;
        nav_trinity.actual = *stateGetPositionEnu_f();
        nav_trinity.direction = change_rep(stateGetHorizontalSpeedDir_f());
        nav_trinity.circle = process_new_point_trinity(&nav_trinity.actual, nav_trinity.target.z, nav_trinity.direction);
      }
      pre_climb = nav_trinity.pos_incr.z / nav_dt;
      break;
    case TRINITY_OUTSIDE:
      VECT3_ADD(nav_trinity.circle, nav_trinity.pos_incr);
      nav_circle_XY(nav_trinity.circle.x, nav_trinity.circle.y , nav_trinity.radius_sign * nav_trinity.radius);
      NavVerticalAltitudeMode(nav_trinity.circle.z + ground_alt, pre_climb);

      if(nav_trinity.inside_cloud){
        nav_trinity.status = TRINITY_INSIDE;
      }
      pre_climb = nav_trinity.pos_incr.z / nav_dt;
      break;
    default:
      // error, leaving
      return false;
  }

  return true;
}

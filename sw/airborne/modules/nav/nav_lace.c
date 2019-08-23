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
 * @author VERDU Titouan
 *
 * Adaptive border pattern for cloud exploration
 */

#include "modules/nav/nav_lace.h"

#include "firmwares/fixedwing/nav.h"
#include "state.h"
#include "autopilot.h"
#include "generated/flight_plan.h"
#include "subsystems/abi.h"

enum LaceStatus {
  LACE_ENTER,
  LACE_INSIDE,
  LACE_OUTSIDE
};

enum RotationDir {
  LACE_LEFT,
  LACE_RIGHT
};

struct NavLace {
  enum LaceStatus status;
  enum RotationDir rotation;
  bool inside_cloud;
  struct EnuCoor_f *actual;
  struct EnuCoor_f target;
  struct EnuCoor_f circle;
  float direction;
  float radius;
  float radius_sign;
  int v_speed;
};

static struct NavLace nav_lace;

static float change_rep(float dir)
{
  return M_PI_2 - dir;
}

static struct EnuCoor_f process_new_point_lace(struct EnuCoor_f *position, float uav_direction)
{
  struct EnuCoor_f new_point;
  float rot_angle;

  if (nav_lace.rotation == LACE_RIGHT) {
    rot_angle = -M_PI_2;
    nav_lace.rotation = LACE_LEFT;
  } else{
    rot_angle = M_PI_2;
    nav_lace.rotation = LACE_RIGHT;
  }

  new_point.x = position->x + (cos(rot_angle + uav_direction) * nav_lace.radius);
  new_point.y = position->y + (sin(rot_angle + uav_direction) * nav_lace.radius);
  new_point.z = position->z + ground_alt;

  return new_point;
}

#if USE_MISSION
#include "modules/mission/mission_common.h"

static bool nav_lace_mission(uint8_t nb, float *params, enum MissionRunFlag flag)
{
  if (nb != 5) {
    return false; // wrong number of parameters
  }
  if (flag == MissionInit) {
    float start_x = params[0];
    float start_y = params[1];
    float start_z = params[2];
    int first_turn = params[3];
    float circle_radius = params[4];
    int vertical_speed = params[5];
    nav_lace_setup(start_x, start_y, start_z, first_turn, circle_radius, vertical_speed);
  }
  return nav_lace_run();
}
#endif

// ABI message

#ifndef NAV_LACE_LWC_ID
#define NAV_LACE_LWC_ID ABI_BROADCAST
#endif

static abi_event lwc_ev;

static lwc_cb(uint8_t sender_id UNUSED, uint32_t stamp UNUSED, int32_t data_type, uint32_t size, uint8_t * data) {
  if (data_type == 1 && size == 1) {
    nav_lace.inside_cloud = (bool) data[0];
  }
}

void nav_lace_init(void)
{
  nav_lace.status = LACE_ENTER;
  nav_lace.radius = DEFAULT_CIRCLE_RADIUS;
  nav_lace.inside_cloud = false;

  AbiBindMsgPAYLOAD_DATA(NAV_LACE_LWC_ID, &lwc_ev, lwc_cb);

#if USE_MISSION
  mission_register(nav_lace_mission, "LACE");
#endif
}

void nav_lace_setup(float init_x, float init_y, float init_z, int turn, float desired_radius, int vert_speed)
{
  struct EnuCoor_f start = {init_x, init_y, init_z};
  nav_lace.target = start;
  nav_lace.status = LACE_ENTER;
  nav_lace.inside_cloud = false;
  nav_lace.radius = desired_radius;
  nav_lace.v_speed = vert_speed;

  if (turn == 1) {
    nav_lace.rotation = LACE_RIGHT;
    nav_lace.radius_sign = 1.0f;
  } else {
    nav_lace.rotation = LACE_LEFT;
    nav_lace.radius_sign = -1.0f;
  }

  nav_lace.actual = stateGetPositionEnu_f();
}

bool nav_lace_run(void)
{
  
  //NavVerticalAutoThrottleMode(); /* Pitch set according to desired VSpeed */

  switch (nav_lace.status) {
    case LACE_ENTER:
      nav_route_xy(nav_lace.actual->x, nav_lace.actual->y, nav_lace.target.x, nav_lace.target.y);
      if (nav_lace.inside_cloud) {
        nav_lace.status = LACE_INSIDE;
        nav_lace.actual = stateGetPositionEnu_f();
        nav_lace.direction = change_rep(stateGetHorizontalSpeedDir_f());
        nav_lace.circle = process_new_point_lace(nav_lace.actual, nav_lace.direction);
        NavVerticalClimbMode(nav_lace.v_speed);
      }
      break;
    case LACE_INSIDE:
      nav_circle_XY(nav_lace.circle.x, nav_lace.circle.y , nav_lace.radius_sign * nav_lace.radius);
      if (!nav_lace.inside_cloud) {
        nav_lace.status = LACE_OUTSIDE;
        nav_lace.actual = stateGetPositionEnu_f();
        nav_lace.direction = change_rep(stateGetHorizontalSpeedDir_f());
        nav_lace.circle = process_new_point_lace(nav_lace.actual, nav_lace.direction);
        nav_lace.radius_sign = -1.0 * nav_lace.radius_sign;
      }
      break;
    case LACE_OUTSIDE:
      nav_circle_XY(nav_lace.circle.x, nav_lace.circle.y , nav_lace.radius_sign * nav_lace.radius);
      if(nav_lace.inside_cloud){
        nav_lace.status = LACE_INSIDE;
        nav_lace.actual = stateGetPositionEnu_f();
        nav_lace.direction = change_rep(stateGetHorizontalSpeedDir_f());
        nav_lace.circle = process_new_point_lace(nav_lace.actual, nav_lace.direction);
        nav_lace.radius_sign = -1.0 * nav_lace.radius_sign;
      }
      break;
    default:
      // error, leaving
      return false;
  }

  return true;
}

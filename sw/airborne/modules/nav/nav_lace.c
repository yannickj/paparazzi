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

enum RotationDir {
  LEFT,
  RIGHT
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
  int v_speed;

};

static float change_rep(float dir){
  return M_PI_2-dir;
}

static struct EnuCoor_f process_new_point_lace(struct EnuCoor_f *position, float uav_direction){
  struct EnuCoor_f new_point;
  float rot_angle;
  float radius_sign;

  if(nav_lace.rotation == RIGHT){
    rot_angle = -M_PI_2;
    radius_sign = 1.0;
  }
  else{
    rot_angle = M_PI_2;
    radius_sign = -1.0;
  }

  new_point.x = position->x + (cos(rot_angle + uav_direction) * radius);
  new_point.y = position->y + (sin(rot_angle + uav_direction) * radius);
  new_point.z = position->z + ground_alt;

  return new_point;
}

static struct NavLace nav_lace;

#if USE_MISSION
#include "modules/mission/mission_common.h"

static bool nav_lace_mission(uint8_t nb, float *params, bool init)
{
  if (nb != 4) {
    return false; // wrong number of parameters
  }
  if (init) {
    struct EnuCoor_f start = { params[0], params[1], params[2] };
    int first_turn = params[3];
    int vertical_speed = params[4];
    nav_lace_setup(start, first_turn, vertical_speed);
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

void nav_lace_setup(struct EnuCoor_f start_point, int turn, int vert_speed)
{
  nav_lace.target = start_point;
  nav_lace.status = LACE_ENTER;
  nav_lace.inside_cloud = false;
  nav_lace.v_speed = vert_speed;

  if(turn == 1){
    nav_lace.rotation = RIGHT;
  }
  else{
    nav_lace.rotation = LEFT;
  }
  
  nav_lace.actual = stateGetPositionEnu_f();
}

bool nav_lace_run(void)
{
  
  NavVerticalAutoThrottleMode(); /* Pitch set according to desired VSpeed */

  switch (nav_lace.status) {
    case LACE_ENTER:
      nav_route_xy(nav_lace.actual->x, nav_lace.actual->y, target.x, target.y); /*Attention il faut du enucoor_i et pas f? */
      if (nav_lace.inside_cloud){
        nav_lace.status = LACE_INSIDE;
        nav_lace.actual = stateGetPositionEnu_f();
        nav_lace.direction = change_rep(stateGetHorizontalSpeedDir_f());
        nav_lace.circle = process_new_point_lace(actual, direction);
      }
      break;
    case LACE_INSIDE:
      nav_circle_xy(nav_lace.circle.x, nav_circle_xy.y , nav_lace.radius);
      if(!nav_lace.inside_cloud){
        nav_lace.status = LACE_OUTSIDE;
        nav_lace.actual = stateGetPositionEnu_f();
        nav_lace.direction = change_rep(stateGetHorizontalSpeedDir_f());
        nav_lace.circle = process_new_point_lace(actual, direction);
      }
      break;
    case LACE_OUTSIDE:
      nav_circle_xy(nav_lace.circle.x, nav_circle_xy.y , nav_lace.radius);
      if(nav_lace.inside_cloud){
        nav_lace.status = LACE_INSIDE;
        nav_lace.actual = stateGetPositionEnu_f();
        nav_lace.direction = change_rep(stateGetHorizontalSpeedDir_f());
        nav_lace.circle = process_new_point_lace(actual, direction);
      }
      break;
    default:
      // error, leaving
      return false;
  }

  return true;
}

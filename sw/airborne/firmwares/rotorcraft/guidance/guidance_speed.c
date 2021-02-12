/*
 * Copyright (C) 2021 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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

/** @file firmwares/rotorcraft/guidance/guidance_speed.c
 *  Speed guidance loop for rotorcraft
 *  Horizontal and vertical
 *
 */

#include "firmwares/rotorcraft/guidance/guidance_speed.h"
#include "firmwares/rotorcraft/navigation.h"
#include "subsystems/radio_control.h"
#include "filters/pid.h"
#include "state.h"

/** Use horizontal guidance reference trajectory.
 * Default is TRUE, define to FALSE to always disable it.
 */
#ifndef GUIDANCE_SPEED_USE_REF
#define GUIDANCE_SPEED_USE_REF TRUE
#endif
PRINT_CONFIG_VAR(GUIDANCE_SPEED_USE_REF)

// Navigation can set heading freely
// This is false if sideslip is a problem
#ifndef GUIDANCE_HEADING_IS_FREE
#define GUIDANCE_HEADING_IS_FREE TRUE
#endif

#ifndef GUIDANCE_SPEED_MAX_H_SPEED
#define GUIDANCE_SPEED_MAX_H_SPEED 5.f
#endif

#ifndef GUIDANCE_SPEED_MAX_V_SPEED
#define GUIDANCE_SPEED_MAX_V_SPEED 3.f
#endif

// Global variable
struct GuidanceSpeed guidance_speed;

/*
 * internal variables
 */

struct GuidanceSpeedReference {
  struct FloatVect3 pos;
  struct FloatVect3 speed;
  struct FloatVect3 accel;
};

struct GuidanceSpeedPrivate {
  struct FloatVect3 pos_err;
  struct FloatVect3 speed_err;
  struct FloatVect3 cmd_earth;
  //struct GuidanceSpeedReference ref; ///< reference calculated from setpoints
  struct PID_f pid_vx;
  struct PID_f pid_vy;
  struct PID_f pid_vz;
};

// Internal private variables
static struct GuidanceSpeedPrivate gsp;

//static void guidance_speed_update_reference(void);

static const float guidance_dt = (1.f / PERIODIC_FREQUENCY);

void guidance_speed_init(void)
{
  guidance_speed.h_mode = GUIDANCE_SPEED_MODE_KILL;
  guidance_speed.v_mode = GUIDANCE_SPEED_MODE_KILL;
  guidance_speed.use_ref = GUIDANCE_SPEED_USE_REF;

  guidance_speed.sp.mask = 0; // pos x,y,z,yaw
  FLOAT_VECT3_ZERO(guidance_speed.sp.pos);
  FLOAT_VECT3_ZERO(guidance_speed.commanded_speed);
  FLOAT_VECT3_ZERO(guidance_speed.rc_sp);
  guidance_speed.rc_yaw_sp = 0.f;
  guidance_speed.sp.heading = 0.f;
  guidance_speed.sp.heading_rate = 0.f;
  guidance_speed.h_gains.p = GUIDANCE_SPEED_H_PGAIN;
  guidance_speed.h_gains.i = GUIDANCE_SPEED_H_IGAIN;
  guidance_speed.h_gains.d = GUIDANCE_SPEED_H_DGAIN;
  guidance_speed.v_gains.p = GUIDANCE_SPEED_V_PGAIN;
  guidance_speed.v_gains.i = GUIDANCE_SPEED_V_IGAIN;
  guidance_speed.v_gains.d = GUIDANCE_SPEED_V_DGAIN;
  guidance_speed.max_h_speed = GUIDANCE_SPEED_MAX_H_SPEED;
  guidance_speed.max_v_speed = GUIDANCE_SPEED_MAX_V_SPEED;

  // init PID
  init_pid_f(&gsp.pid_vx, guidance_speed.h_gains.p, guidance_speed.h_gains.d, guidance_speed.h_gains.i, 2.f);
  init_pid_f(&gsp.pid_vy, guidance_speed.h_gains.p, guidance_speed.h_gains.d, guidance_speed.h_gains.i, 2.f);
  init_pid_f(&gsp.pid_vz, guidance_speed.v_gains.p, guidance_speed.v_gains.d, guidance_speed.v_gains.i, 2.f);
}


//static inline void reset_guidance_reference_from_current_position(void)
//{
//  VECT3_COPY(guidance_speed.ref.pos, *stateGetPositionNed_i());
//  VECT3_COPY(guidance_speed.ref.speed, *stateGetSpeedNed_i());
//  FLOAT_VECT3_ZERO(guidance_speed.ref.accel);
//  //gh_set_ref(guidance_speed.ref.pos, guidance_speed.ref.speed, guidance_speed.ref.accel);
//
//}


void guidance_speed_read_rc(bool in_flight)
{
  // TODO set speed / yaw rate / alt-vz from RC

  if (in_flight) {
    // negative pitch is forward
    int64_t rc_x = -radio_control.values[RADIO_PITCH];
    int64_t rc_y = radio_control.values[RADIO_ROLL];
    int64_t rc_z = radio_control.values[RADIO_THROTTLE];
    DeadBand(rc_x, MAX_PPRZ / 20);
    DeadBand(rc_y, MAX_PPRZ / 20);
    DeadBand(rc_z, MAX_PPRZ / 20);

    rc_x = rc_x * guidance_speed.max_h_speed / MAX_PPRZ;
    rc_y = rc_y * guidance_speed.max_h_speed / MAX_PPRZ;
    rc_z = rc_z * guidance_speed.max_v_speed / MAX_PPRZ;

    /* Rotate from body to NED frame by negative psi angle */
    float psi = -stateGetNedToBodyEulers_f()->psi;
    float s_psi = sinf(psi);
    float c_psi = cosf(psi);
    guidance_speed.rc_sp.x = c_psi * rc_x + s_psi * rc_y;
    guidance_speed.rc_sp.y = -s_psi * rc_x + c_psi * rc_y;
    // TODO Z, yaw
  } else {
    FLOAT_VECT3_ZERO(guidance_speed.rc_sp);
    guidance_speed.rc_yaw_sp = 0.f;
  }
}


//static void guidance_speed_update_reference(void)
//{
//  /* compute reference even if usage temporarily disabled via guidance_speed_use_ref */
//#if GUIDANCE_H_USE_REF
//  if (bit_is_set(guidance_speed.sp.mask, 5)) {
//    gh_update_ref_from_speed_sp(guidance_speed.sp.speed);
//  } else {
//    gh_update_ref_from_pos_sp(guidance_speed.sp.pos);
//  }
//#endif
//
//  /* either use the reference or simply copy the pos setpoint */
//  if (guidance_speed.use_ref) {
//    /* convert our reference to generic representation */
//    INT32_VECT2_RSHIFT(guidance_speed.ref.pos,   gh_ref.pos, (GH_POS_REF_FRAC - INT32_POS_FRAC));
//    INT32_VECT2_LSHIFT(guidance_speed.ref.speed, gh_ref.speed, (INT32_SPEED_FRAC - GH_SPEED_REF_FRAC));
//    INT32_VECT2_LSHIFT(guidance_speed.ref.accel, gh_ref.accel, (INT32_ACCEL_FRAC - GH_ACCEL_REF_FRAC));
//  } else {
//    VECT2_COPY(guidance_speed.ref.pos, guidance_speed.sp.pos);
//    INT_VECT2_ZERO(guidance_speed.ref.speed);
//    INT_VECT2_ZERO(guidance_speed.ref.accel);
//  }
//
//#if GUIDANCE_H_USE_SPEED_REF
//  if (guidance_speed.mode == GUIDANCE_H_MODE_HOVER) {
//    VECT2_COPY(guidance_speed.sp.pos, guidance_speed.ref.pos); // for display only
//  }
//#endif
//
//  /* update heading setpoint from rate */
//  if (bit_is_set(guidance_speed.sp.mask, 7)) {
//    guidance_speed.sp.heading += guidance_speed.sp.heading_rate / PERIODIC_FREQUENCY;
//    FLOAT_ANGLE_NORMALIZE(guidance_speed.sp.heading);
//  }
//}

#define MAX_POS_ERR   20.
#define MAX_SPEED_ERR 10.

void guidance_speed_run(bool in_flight)
{
  // horizontal speed
  if (!bit_is_set(guidance_speed.sp.mask, GUIDANCE_SPEED_VXY_BIT)) {
    // compute position error and saturate TODO use ref ?
    VECT2_DIFF(gsp.pos_err, guidance_speed.sp.pos, *stateGetPositionNed_f());
    VECT2_STRIM(gsp.pos_err, -MAX_POS_ERR, MAX_POS_ERR);
    // compute speed error and saturate TODO use ref ?
    struct FloatVect2 zero = { 0.f, 0.f };
    VECT2_DIFF(gsp.speed_err, zero, *stateGetSpeedNed_f());
    VECT2_STRIM(gsp.speed_err, -MAX_SPEED_ERR, MAX_SPEED_ERR);
    // run PID
    gsp.cmd_earth.x = update_pid_derivative_f(&gsp.pid_vx, gsp.pos_err.x, gsp.speed_err.x, guidance_dt);
    gsp.cmd_earth.y = update_pid_derivative_f(&gsp.pid_vy, gsp.pos_err.y, gsp.speed_err.y, guidance_dt);
  }
  else {
    VECT2_COPY(gsp.cmd_earth, guidance_speed.sp.speed);
  }

  // vertical speed
  if (!bit_is_set(guidance_speed.sp.mask, GUIDANCE_SPEED_VZ_BIT)) {
    // compute position error and saturate TODO use ref ?
    gsp.pos_err.z = guidance_speed.sp.pos.z - stateGetPositionNed_f()->z;
    BoundAbs(gsp.pos_err.z, MAX_POS_ERR);
    // compute speed error and saturate TODO use ref ?
    gsp.speed_err.z = - stateGetSpeedNed_f()->z;
    BoundAbs(gsp.speed_err.z , MAX_SPEED_ERR);
    // run PID
    gsp.cmd_earth.z = update_pid_derivative_f(&gsp.pid_vz, gsp.pos_err.z, gsp.speed_err.z, guidance_dt);
  }
  else {
    gsp.cmd_earth.z = guidance_speed.sp.speed.z;
  }

  // TODO yaw

  // reset integrators if not in flight
  if (!in_flight) {
    set_integral_pid_f(&gsp.pid_vx, 0.f);
    set_integral_pid_f(&gsp.pid_vy, 0.f);
    set_integral_pid_f(&gsp.pid_vz, 0.f);
  }

  // saturate commanded speed
  VECT2_STRIM(gsp.cmd_earth, -guidance_speed.max_h_speed, guidance_speed.max_h_speed);
  BoundAbs(gsp.cmd_earth.z, guidance_speed.max_v_speed);

  // commanded speed, in body frame for now
  // TODO option to have it in earth or body frame ?
  float psi = stateGetNedToBodyEulers_f()->psi;
  float s_psi = sinf(psi);
  float c_psi = cosf(psi);
  guidance_speed.commanded_speed.x =  c_psi * gsp.cmd_earth.x + s_psi * gsp.cmd_earth.y;
  guidance_speed.commanded_speed.y = -s_psi * gsp.cmd_earth.x + c_psi * gsp.cmd_earth.y;
  guidance_speed.commanded_speed.z = gsp.cmd_earth.z;
}

void guidance_speed_hover_enter(void)
{
  /* reset speed setting */
  FLOAT_VECT3_ZERO(guidance_speed.sp.speed);

  /* disable horizontal velocity setpoints */
  ClearBit(guidance_speed.sp.mask, GUIDANCE_SPEED_VXY_BIT);
  ClearBit(guidance_speed.sp.mask, GUIDANCE_SPEED_VZ_BIT);
  ClearBit(guidance_speed.sp.mask, GUIDANCE_SPEED_VYAW_BIT);

  /* setpoint to current position */
  VECT3_COPY(guidance_speed.sp.pos, *stateGetPositionNed_f());

  /* reset guidance reference */
  //reset_guidance_reference_from_current_position();

  /* set guidance to current heading and position */
  guidance_speed.rc_yaw_sp = stateGetNedToBodyEulers_f()->psi;
  guidance_speed.sp.heading = guidance_speed.rc_yaw_sp;
}

void guidance_speed_nav_enter(void)
{
  /* disable horizontal velocity setpoints */
  ClearBit(guidance_speed.sp.mask, GUIDANCE_SPEED_VXY_BIT);
  ClearBit(guidance_speed.sp.mask, GUIDANCE_SPEED_VZ_BIT);
  ClearBit(guidance_speed.sp.mask, GUIDANCE_SPEED_VYAW_BIT);

  /* horizontal position setpoint from navigation/flightplan with ENU int to NED float */
  VECT3_ASSIGN(guidance_speed.sp.pos,
      POS_FLOAT_OF_BFP(navigation_carrot.y),
      POS_FLOAT_OF_BFP(navigation_carrot.x),
      -POS_FLOAT_OF_BFP(navigation_carrot.z)); // FIXME check if correct alt

  //reset_guidance_reference_from_current_position();

  // set navigation heading to current heading
  nav_heading = stateGetNedToBodyEulers_i()->psi;
  guidance_speed.sp.heading = stateGetNedToBodyEulers_f()->psi;
}

void guidance_speed_from_nav(bool in_flight)
{
  if (!in_flight) {
    guidance_speed_nav_enter();
  }

  // nothing to do in manual and attitude submodes
  if (horizontal_mode != HORIZONTAL_MODE_MANUAL && horizontal_mode != HORIZONTAL_MODE_ATTITUDE) {
    /* horizontal position setpoint from navigation/flightplan with ENU int to NED float */
    VECT3_ASSIGN(guidance_speed.sp.pos,
        POS_FLOAT_OF_BFP(navigation_carrot.y),
        POS_FLOAT_OF_BFP(navigation_carrot.x),
        -POS_FLOAT_OF_BFP(navigation_carrot.z)); // FIXME check if correct alt
    //guidance_speed_update_reference();

#if GUIDANCE_HEADING_IS_FREE
    /* set psi command */
    guidance_speed.sp.heading = ANGLE_FLOAT_OF_BFP(nav_heading);
    FLOAT_ANGLE_NORMALIZE(guidance_speed.sp.heading);
#endif

    /* compute commands */
    guidance_speed_run(in_flight);
  }
}

void guidance_speed_set_h_pgain(float pgain)
{
  guidance_speed.h_gains.p = pgain;
  set_gains_pid_f(&gsp.pid_vx, guidance_speed.h_gains.p, guidance_speed.h_gains.d, guidance_speed.h_gains.i);
  set_gains_pid_f(&gsp.pid_vy, guidance_speed.h_gains.p, guidance_speed.h_gains.d, guidance_speed.h_gains.i);
}

void guidance_speed_set_h_dgain(float dgain)
{
  guidance_speed.h_gains.d = dgain;
  set_gains_pid_f(&gsp.pid_vx, guidance_speed.h_gains.p, guidance_speed.h_gains.d, guidance_speed.h_gains.i);
  set_gains_pid_f(&gsp.pid_vy, guidance_speed.h_gains.p, guidance_speed.h_gains.d, guidance_speed.h_gains.i);
}

void guidance_speed_set_h_igain(float igain)
{
  guidance_speed.h_gains.i = igain;
  set_gains_pid_f(&gsp.pid_vx, guidance_speed.h_gains.p, guidance_speed.h_gains.d, guidance_speed.h_gains.i);
  set_gains_pid_f(&gsp.pid_vy, guidance_speed.h_gains.p, guidance_speed.h_gains.d, guidance_speed.h_gains.i);
  set_integral_pid_f(&gsp.pid_vx, 0.f);
  set_integral_pid_f(&gsp.pid_vz, 0.f);
}

void guidance_speed_set_v_pgain(float pgain)
{
  guidance_speed.v_gains.p = pgain;
  set_gains_pid_f(&gsp.pid_vz, guidance_speed.v_gains.p, guidance_speed.v_gains.d, guidance_speed.v_gains.i);
}

void guidance_speed_set_v_dgain(float dgain)
{
  guidance_speed.v_gains.d = dgain;
  set_gains_pid_f(&gsp.pid_vz, guidance_speed.v_gains.p, guidance_speed.v_gains.d, guidance_speed.v_gains.i);
}

void guidance_speed_set_v_igain(float igain)
{
  guidance_speed.v_gains.i = igain;
  set_gains_pid_f(&gsp.pid_vz, guidance_speed.v_gains.p, guidance_speed.v_gains.d, guidance_speed.v_gains.i);
  set_integral_pid_f(&gsp.pid_vz, 0.f);
}


#ifdef AP_MODE_GUIDED

void guidance_speed_guided_run(bool in_flight)
{
  /* guidance_speed.sp.pos and guidance_speed.sp.heading need to be set from external source */
  if (!in_flight) {
    guidance_speed_hover_enter();
  }
  //guidance_speed_update_reference();
  /* compute commands */
  guidance_speed_run(in_flight);
}

bool guidance_h_set_guided_pos(float x, float y)
{
  if (autopilot_get_mode() == AP_MODE_GUIDED) {
    ClearBit(guidance_speed.sp.mask, GUIDANCE_SPEED_VXY_BIT);
    guidance_speed.sp.pos.x = x;
    guidance_speed.sp.pos.y = y;
    return true;
  }
  return false;
}

bool guidance_h_set_guided_heading(float heading)
{
  if (autopilot_get_mode() == AP_MODE_GUIDED) {
    ClearBit(guidance_speed.sp.mask, GUIDANCE_SPEED_VYAW_BIT);
    guidance_speed.sp.heading = heading;
    FLOAT_ANGLE_NORMALIZE(guidance_speed.sp.heading);
    return true;
  }
  return false;
}

bool guidance_h_set_guided_body_vel(float vx, float vy)
{
  float psi = stateGetNedToBodyEulers_f()->psi;
  float newvx =  cosf(-psi) * vx + sinf(-psi) * vy;
  float newvy = -sinf(-psi) * vx + cosf(-psi) * vy;
  return guidance_h_set_guided_vel(newvx, newvy);
}

bool guidance_h_set_guided_vel(float vx, float vy)
{
  if (autopilot_get_mode() == AP_MODE_GUIDED) {
    SetBit(guidance_speed.sp.mask, GUIDANCE_SPEED_VXY_BIT);
    guidance_speed.sp.speed.x = vx;
    guidance_speed.sp.speed.y = vy;
    return true;
  }
  return false;
}

bool guidance_h_set_guided_heading_rate(float rate)
{
  if (autopilot_get_mode() == AP_MODE_GUIDED) {
    SetBit(guidance_speed.sp.mask, GUIDANCE_SPEED_VYAW_BIT);
    guidance_speed.sp.heading_rate = rate;
    return true;
  }
  return false;
}

bool guidance_v_set_guided_z(float z)
{
  if (autopilot_get_mode() == AP_MODE_GUIDED) {
    ClearBit(guidance_speed.sp.mask, GUIDANCE_SPEED_VZ_BIT);
    guidance_speed.sp.pos.z = z;
    return true;
  }
  return false;
}

bool guidance_v_set_guided_vz(float vz)
{
  if (autopilot_get_mode() == AP_MODE_GUIDED) {
    SetBit(guidance_speed.sp.mask, GUIDANCE_SPEED_VZ_BIT);
    guidance_speed.sp.speed.z = vz;
    return true;
  }
  return false;
}

#endif // AP_MODE_GUIDED

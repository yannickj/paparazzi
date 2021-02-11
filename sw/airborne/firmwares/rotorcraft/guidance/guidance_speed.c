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
#include "state.h"

PRINT_CONFIG_VAR(GUIDANCE_H_USE_REF)
PRINT_CONFIG_VAR(GUIDANCE_H_USE_SPEED_REF)

// Navigation can set heading freely
// This is false if sideslip is a problem
#ifndef GUIDANCE_HEADING_IS_FREE
#define GUIDANCE_HEADING_IS_FREE TRUE
#endif

struct GuidanceSpeed guidance_speed;

/*
 * internal variables
 */
static struct FloatVect3 guidance_speed_pos_err;
static struct FloatVect3 guidance_speed_speed_err;
static struct FloatVect3 guidance_speed_trim_att_integrator;

/** horizontal guidance command.
 */
static struct FloatVect3  guidance_speed_cmd_earth;

static void guidance_speed_update_reference(void);
static void guidance_speed_traj_run(bool in_flight);
static void read_rc_setpoint_speed_i(struct FloatVect3 *speed_sp, bool in_flight);


void guidance_speed_init(void)
{
  guidance_speed.h_mode = GUIDANCE_SPEED_H_MODE_KILL;
  guidance_speed.use_ref = GUIDANCE_SPEED_USE_REF;

  guidance_speed.sp.mask = 0; // pos x,y,z,yaw
  FLOAT_VECT3_ZERO(guidance_speed.sp.pos);
  FLOAT_VECT3_ZERO(guidance_speed_trim_att_integrator);
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

  //gh_ref_init();
}


static inline void reset_guidance_reference_from_current_position(void)
{
  VECT3_COPY(guidance_speed.ref.pos, *stateGetPositionNed_i());
  VECT3_COPY(guidance_speed.ref.speed, *stateGetSpeedNed_i());
  FLOAT_VECT3_ZERO(guidance_speed.ref.accel);
  //gh_set_ref(guidance_speed.ref.pos, guidance_speed.ref.speed, guidance_speed.ref.accel);

  FLOAT_VECT3_ZERO(guidance_speed_trim_att_integrator);
}


void guidance_speed_read_rc(bool in_flight)
{
  // TODO set speed / yaw rate / alt-vz from RC

  if (in_flight) {
    // negative pitch is forward
    int64_t rc_x = -radio_control.values[RADIO_PITCH];
    int64_t rc_y = radio_control.values[RADIO_ROLL];
    DeadBand(rc_x, MAX_PPRZ / 20);
    DeadBand(rc_y, MAX_PPRZ / 20);
    DeadBand(rc_Z, MAX_PPRZ / 20);

    rc_x = rc_x * guidance_speed.max_h_speed / MAX_PPRZ;
    rc_y = rc_y * guidance_speed.max_h_speed / MAX_PPRZ;
    rc_z = rc_z * guidance_speed.max_v_speed / MAX_PPRZ

    /* Rotate from body to NED frame by negative psi angle */
    float psi = -stateGetNedToBodyEulers_f()->psi;
    float s_psi = sinf(psi);
    float c_psi = cosf(psi);
    guidance_speed.rc_sp.x = (int32_t)(((int64_t)c_psi * rc_x + (int64_t)s_psi * rc_y) >> INT32_TRIG_FRAC);
    guidance_speed.rc_sp.y = (int32_t)((-(int64_t)s_psi * rc_x + (int64_t)c_psi * rc_y) >> INT32_TRIG_FRAC);
    // TODO Z, yaw
  } else {
    FLOAT_VECT3_ZERO(guidance_speed.rc_sp);
    guidance_speed.rc_yaw_sp = 0.f;
  }
}

void guidance_speed_run(bool  in_flight)
{
  switch (guidance_speed.mode) {

    case GUIDANCE_H_MODE_HOVER:
      /* set psi command from RC */
      guidance_speed.sp.heading = guidance_speed.rc_sp.psi;
      /* fall trough to GUIDED to update ref, run traj and set final attitude setpoint */

      /* Falls through. */
    case GUIDANCE_H_MODE_GUIDED:
      guidance_speed_guided_run(in_flight);
      break;

    case GUIDANCE_H_MODE_NAV:
      guidance_speed_from_nav(in_flight);
      break;
    default:
      break;
  }
}


static void guidance_speed_update_reference(void)
{
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
}

#define MAX_POS_ERR   20.
#define MAX_SPEED_ERR 10.

#if !GUIDANCE_INDI
static void guidance_speed_traj_run(bool in_flight)
{
  /* maximum bank angle: default 20 deg, max 40 deg*/
  static const int32_t traj_max_bank = Min(BFP_OF_REAL(GUIDANCE_H_MAX_BANK, INT32_ANGLE_FRAC),
                                       BFP_OF_REAL(RadOfDeg(40), INT32_ANGLE_FRAC));
  static const int32_t total_max_bank = BFP_OF_REAL(RadOfDeg(45), INT32_ANGLE_FRAC);

  /* compute position error    */
  VECT2_DIFF(guidance_speed_pos_err, guidance_speed.ref.pos, *stateGetPositionNed_i());
  /* saturate it               */
  VECT2_STRIM(guidance_speed_pos_err, -MAX_POS_ERR, MAX_POS_ERR);

  /* compute speed error    */
  VECT2_DIFF(guidance_speed_speed_err, guidance_speed.ref.speed, *stateGetSpeedNed_i());
  /* saturate it               */
  VECT2_STRIM(guidance_speed_speed_err, -MAX_SPEED_ERR, MAX_SPEED_ERR);

  /* run PID */
  int32_t pd_x =
    ((guidance_speed.gains.p * guidance_speed_pos_err.x) >> (INT32_POS_FRAC - GH_GAIN_SCALE)) +
    ((guidance_speed.gains.d * (guidance_speed_speed_err.x >> 2)) >> (INT32_SPEED_FRAC - GH_GAIN_SCALE - 2));
  int32_t pd_y =
    ((guidance_speed.gains.p * guidance_speed_pos_err.y) >> (INT32_POS_FRAC - GH_GAIN_SCALE)) +
    ((guidance_speed.gains.d * (guidance_speed_speed_err.y >> 2)) >> (INT32_SPEED_FRAC - GH_GAIN_SCALE - 2));
  guidance_speed_cmd_earth.x = pd_x +
                           ((guidance_speed.gains.v * guidance_speed.ref.speed.x) >> (INT32_SPEED_FRAC - GH_GAIN_SCALE)) + /* speed feedforward gain */
                           ((guidance_speed.gains.a * guidance_speed.ref.accel.x) >> (INT32_ACCEL_FRAC -
                               GH_GAIN_SCALE));   /* acceleration feedforward gain */
  guidance_speed_cmd_earth.y = pd_y +
                           ((guidance_speed.gains.v * guidance_speed.ref.speed.y) >> (INT32_SPEED_FRAC - GH_GAIN_SCALE)) + /* speed feedforward gain */
                           ((guidance_speed.gains.a * guidance_speed.ref.accel.y) >> (INT32_ACCEL_FRAC -
                               GH_GAIN_SCALE));   /* acceleration feedforward gain */

  /* trim max bank angle from PD */
  VECT2_STRIM(guidance_speed_cmd_earth, -traj_max_bank, traj_max_bank);

  /* Update pos & speed error integral, zero it if not in_flight.
   * Integrate twice as fast when not only POS but also SPEED are wrong,
   * but do not integrate POS errors when the SPEED is already catching up.
   */
  if (in_flight) {
    /* ANGLE_FRAC (12) * GAIN (8) * LOOP_FREQ (9) -> INTEGRATOR HIGH RES ANGLE_FRAX (28) */
    guidance_speed_trim_att_integrator.x += (guidance_speed.gains.i * pd_x);
    guidance_speed_trim_att_integrator.y += (guidance_speed.gains.i * pd_y);
    /* saturate it  */
    VECT2_STRIM(guidance_speed_trim_att_integrator, -(traj_max_bank << (INT32_ANGLE_FRAC + GH_GAIN_SCALE * 2)),
                (traj_max_bank << (INT32_ANGLE_FRAC + GH_GAIN_SCALE * 2)));
    /* add it to the command */
    guidance_speed_cmd_earth.x += (guidance_speed_trim_att_integrator.x >> (INT32_ANGLE_FRAC + GH_GAIN_SCALE * 2));
    guidance_speed_cmd_earth.y += (guidance_speed_trim_att_integrator.y >> (INT32_ANGLE_FRAC + GH_GAIN_SCALE * 2));
  } else {
    INT_VECT2_ZERO(guidance_speed_trim_att_integrator);
  }

  /* compute a better approximation of force commands by taking thrust into account */
  if (guidance_speed.approx_force_by_thrust && in_flight) {
    static int32_t thrust_cmd_filt;
    int32_t vertical_thrust = (stabilization_cmd[COMMAND_THRUST] * guidance_v_thrust_coeff) >> INT32_TRIG_FRAC;
    thrust_cmd_filt = (thrust_cmd_filt * GUIDANCE_H_THRUST_CMD_FILTER + vertical_thrust) /
                      (GUIDANCE_H_THRUST_CMD_FILTER + 1);
    guidance_speed_cmd_earth.x = ANGLE_BFP_OF_REAL(atan2f((guidance_speed_cmd_earth.x * MAX_PPRZ / INT32_ANGLE_PI_2),
                             thrust_cmd_filt));
    guidance_speed_cmd_earth.y = ANGLE_BFP_OF_REAL(atan2f((guidance_speed_cmd_earth.y * MAX_PPRZ / INT32_ANGLE_PI_2),
                             thrust_cmd_filt));
  }

  VECT2_STRIM(guidance_speed_cmd_earth, -total_max_bank, total_max_bank);
}
#endif

void guidance_speed_hover_enter(void)
{
  /* reset speed setting */
  guidance_speed.sp.speed.x = 0;
  guidance_speed.sp.speed.y = 0;

  /* disable horizontal velocity setpoints,
   * might still be activated in guidance_speed_read_rc if GUIDANCE_H_USE_SPEED_REF
   */
  ClearBit(guidance_speed.sp.mask, 5);
  ClearBit(guidance_speed.sp.mask, 7);

  /* set horizontal setpoint to current position */
  VECT2_COPY(guidance_speed.sp.pos, *stateGetPositionNed_i());

  /* reset guidance reference */
  reset_guidance_reference_from_current_position();

  /* set guidance to current heading and position */
  guidance_speed.rc_sp.psi = stateGetNedToBodyEulers_f()->psi;
  guidance_speed.sp.heading = guidance_speed.rc_sp.psi;
}

void guidance_speed_nav_enter(void)
{
  ClearBit(guidance_speed.sp.mask, 5);
  ClearBit(guidance_speed.sp.mask, 7);

  /* horizontal position setpoint from navigation/flightplan */
  INT32_VECT2_NED_OF_ENU(guidance_speed.sp.pos, navigation_carrot);

  reset_guidance_reference_from_current_position();

  nav_heading = stateGetNedToBodyEulers_i()->psi;
  guidance_speed.sp.heading = stateGetNedToBodyEulers_f()->psi;
}

void guidance_speed_from_nav(bool in_flight)
{
  if (!in_flight) {
    guidance_speed_nav_enter();
  }

  if (horizontal_mode == HORIZONTAL_MODE_MANUAL) {
    stabilization_cmd[COMMAND_ROLL]  = nav_cmd_roll;
    stabilization_cmd[COMMAND_PITCH] = nav_cmd_pitch;
    stabilization_cmd[COMMAND_YAW]   = nav_cmd_yaw;
  } else if (horizontal_mode == HORIZONTAL_MODE_ATTITUDE) {
    struct Int32Eulers sp_cmd_i;
    sp_cmd_i.phi = nav_roll;
    sp_cmd_i.theta = nav_pitch;
    sp_cmd_i.psi = nav_heading;
    stabilization_attitude_set_rpy_setpoint_i(&sp_cmd_i);
    stabilization_attitude_run(in_flight);
  } else {
    INT32_VECT2_NED_OF_ENU(guidance_speed.sp.pos, navigation_carrot);
    guidance_speed_update_reference();

#if GUIDANCE_HEADING_IS_FREE
    /* set psi command */
    guidance_speed.sp.heading = ANGLE_FLOAT_OF_BFP(nav_heading);
    FLOAT_ANGLE_NORMALIZE(guidance_speed.sp.heading);
#endif

    /* compute x,y earth commands */
    guidance_speed_traj_run(in_flight);
    /* set final attitude setpoint */
    int32_t heading_sp_i = ANGLE_BFP_OF_REAL(guidance_speed.sp.heading);
    stabilization_attitude_set_earth_cmd_i(&guidance_speed_cmd_earth,
                                           heading_sp_i);

    stabilization_attitude_run(in_flight);
  }
}

void guidance_speed_set_igain(uint32_t igain)
{
  guidance_speed.gains.i = igain;
  INT_VECT2_ZERO(guidance_speed_trim_att_integrator);
}


void guidance_speed_guided_run(bool in_flight)
{
  /* guidance_speed.sp.pos and guidance_speed.sp.heading need to be set from external source */
  if (!in_flight) {
    guidance_speed_hover_enter();
  }

  guidance_speed_update_reference();

#if GUIDANCE_INDI
  guidance_indi_run(&guidance_speed.sp.heading);
#else
  /* compute x,y earth commands */
  guidance_speed_traj_run(in_flight);
  /* set final attitude setpoint */
  int32_t heading_sp_i = ANGLE_BFP_OF_REAL(guidance_speed.sp.heading);
  stabilization_attitude_set_earth_cmd_i(&guidance_speed_cmd_earth, heading_sp_i);
#endif
  stabilization_attitude_run(in_flight);
}

bool guidance_speed_set_guided_pos(float x, float y)
{
  if (guidance_speed.mode == GUIDANCE_H_MODE_GUIDED) {
    ClearBit(guidance_speed.sp.mask, 5);
    guidance_speed.sp.pos.x = POS_BFP_OF_REAL(x);
    guidance_speed.sp.pos.y = POS_BFP_OF_REAL(y);
    return true;
  }
  return false;
}

bool guidance_speed_set_guided_heading(float heading)
{
  if (guidance_speed.mode == GUIDANCE_H_MODE_GUIDED) {
    ClearBit(guidance_speed.sp.mask, 7);
    guidance_speed.sp.heading = heading;
    FLOAT_ANGLE_NORMALIZE(guidance_speed.sp.heading);
    return true;
  }
  return false;
}

bool guidance_speed_set_guided_body_vel(float vx, float vy)
{
  float psi = stateGetNedToBodyEulers_f()->psi;
  float newvx =  cosf(-psi) * vx + sinf(-psi) * vy;
  float newvy = -sinf(-psi) * vx + cosf(-psi) * vy;
  return guidance_speed_set_guided_vel(newvx, newvy);
}

bool guidance_speed_set_guided_vel(float vx, float vy)
{
  if (guidance_speed.mode == GUIDANCE_H_MODE_GUIDED) {
    SetBit(guidance_speed.sp.mask, 5);
    guidance_speed.sp.speed.x = SPEED_BFP_OF_REAL(vx);
    guidance_speed.sp.speed.y = SPEED_BFP_OF_REAL(vy);
    return true;
  }
  return false;
}

bool guidance_speed_set_guided_heading_rate(float rate)
{
  if (guidance_speed.mode == GUIDANCE_H_MODE_GUIDED) {
    SetBit(guidance_speed.sp.mask, 7);
    guidance_speed.sp.heading_rate = rate;
    return true;
  }
  return false;
}

const struct Int32Vect2 *guidance_speed_get_pos_err(void)
{
  return &guidance_speed_pos_err;
}

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

/** @file firmwares/rotorcraft/guidance/guidance_speed.h
 *  Speed guidance loop for rotorcraft
 *  Horizontal and vertical
 *
 */

#ifndef GUIDANCE_SPEED_H
#define GUIDANCE_SPEED_H

#ifdef __cplusplus
extern "C" {
#endif

#include "math/pprz_algebra_int.h"
#include "math/pprz_algebra_float.h"

#include "firmwares/rotorcraft/guidance/guidance_h_ref.h"
#include "generated/airframe.h"
#include "std.h"

/** Use horizontal guidance reference trajectory.
 * Default is TRUE, define to FALSE to always disable it.
 */
#ifndef GUIDANCE_SPEED_USE_REF
#define GUIDANCE_SPEED_USE_REF TRUE
#endif

/** Use horizontal guidance speed reference.
 * This also allows to give velocity commands via RC in GUIDANCE_H_MODE_HOVER.
 * Default is TRUE, define to FALSE to always disable it.
 */
#ifndef GUIDANCE_SPEED_USE_SPEED_REF
#define GUIDANCE_SPEED_USE_SPEED_REF TRUE
#endif

#define GUIDANCE_SPEED_H_MODE_KILL        0
#define GUIDANCE_SPEED_H_MODE_HOVER       3
#define GUIDANCE_SPEED_H_MODE_NAV         4
#define GUIDANCE_SPEED_H_MODE_GUIDED      10


struct GuidanceSpeedSetpoint {
  struct FloatVect3 pos;
  struct FloatVect3 speed;
  float heading;
  float heading_rate;
  uint8_t mask;             ///< bit 5: vx & vy, bit 6: vz, bit 7: vyaw
};

struct GuidanceSpeedReference {
  struct FloatVect3 pos;
  struct FloatVect3 speed;
  struct FloatVect3 accel;
};

struct GuidanceSpeedGains {
  float p;
  float d;
  float i;
};

struct GuidanceSpeed {
  uint8_t h_mode;
  uint8_t v_mode;
  /* configuration options */
  bool use_ref;
  /* gains */
  struct GuidanceSpeedGains h_gains;
  struct GuidanceSpeedGains v_gains;

  struct GuidanceSpeedSetpoint sp; ///< setpoints
  struct GuidanceSpeedReference ref; ///< reference calculated from setpoints

  float max_h_speed;
  float max_v_speed;
  float max_yax_rate;
  struct FloatVect3 rc_sp;
  float rc_yaw_sp;
};

extern struct GuidanceSpeed guidance_speed;

extern void guidance_speed_init(void);
extern void guidance_speed_read_rc(bool in_flight);
extern void guidance_speed_run(bool in_flight);

extern void guidance_speed_hover_enter(void);
extern void guidance_speed_nav_enter(void);

/** Set horizontal guidance from NAV and run control loop
 */
extern void guidance_h_from_nav(bool in_flight);

extern void guidance_speed_set_h_igain(uint32_t igain);
extern void guidance_speed_set_v_igain(uint32_t igain);

/** Run GUIDED mode control
 */
extern void guidance_speed_guided_run(bool in_flight);

/** Set horizontal position setpoint in GUIDED mode.
 * @param x North position (local NED frame) in meters.
 * @param y East position (local NED frame) in meters.
 * @return TRUE if setpoints were set (currently in GUIDANCE_H_MODE_GUIDED)
 */
extern bool guidance_speed_set_guided_pos(float x, float y);

/** Set heading setpoint in GUIDED mode.
 * @param heading Setpoint in radians.
 * @return TRUE if setpoint was set (currently in GUIDANCE_H_MODE_GUIDED)
 */
extern bool guidance_speed_set_guided_heading(float heading);

/** Set body relative horizontal velocity setpoint in GUIDED mode.
 * @param vx forward velocity (body frame) in meters/sec.
 * @param vy right velocity (body frame) in meters/sec.
 * @return TRUE if setpoints were set (currently in GUIDANCE_H_MODE_GUIDED)
 */
extern bool guidance_speed_set_guided_body_vel(float vx, float vy);

/** Set horizontal velocity setpoint in GUIDED mode.
 * @param vx North velocity (local NED frame) in meters/sec.
 * @param vy East velocity (local NED frame) in meters/sec.
 * @return TRUE if setpoints were set (currently in GUIDANCE_H_MODE_GUIDED)
 */
extern bool guidance_speed_set_guided_vel(float vx, float vy);

/** Set heading rate setpoint in GUIDED mode.
 * @param rate Heading rate in radians.
 * @return TRUE if setpoints were set (currently in GUIDANCE_H_MODE_GUIDED)
 */
extern bool guidance_speed_set_guided_heading_rate(float rate);

/* Make sure that ref can only be temporarily disabled for testing,
 * but not enabled if GUIDANCE_H_USE_REF was defined to FALSE.
 */
#define guidance_speed_SetUseRef(_val) {                        \
    guidance_speed.use_ref = _val && GUIDANCE_SPEED_USE_REF;    \
  }

static inline void guidance_speed_SetMaxSpeed(float speed)
{
  gh_set_max_speed(speed);
}

static inline void guidance_speed_SetOmega(float omega)
{
  gh_set_omega(omega);
}

static inline void guidance_speed_SetZeta(float zeta)
{
  gh_set_zeta(zeta);
}

static inline void guidance_speed_SetTau(float tau)
{
  gh_set_tau(tau);
}

#ifdef __cplusplus
}
#endif

#endif /* GUIDANCE_SPEED_H */

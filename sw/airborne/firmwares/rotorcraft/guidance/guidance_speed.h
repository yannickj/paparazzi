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
#include "generated/airframe.h"
#include "autopilot.h"
#include "std.h"

#define GUIDANCE_SPEED_MODE_KILL  0
#define GUIDANCE_SPEED_MODE_SPEED 1
#define GUIDANCE_SPEED_MODE_POS   2

#define GUIDANCE_SPEED_VXY_BIT  5
#define GUIDANCE_SPEED_VZ_BIT   6
#define GUIDANCE_SPEED_VYAW_BIT 7

struct GuidanceSpeedSetpoint {
  struct FloatVect3 pos;
  struct FloatVect3 speed;
  float heading;
  float heading_rate;
  uint8_t mask;             ///< bit 5: vx & vy, bit 6: vz, bit 7: vyaw
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

  struct GuidanceSpeedSetpoint sp;    ///< setpoints (input)
  struct FloatVect3 commanded_speed;  ///< commands (output)

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
extern void guidance_speed_from_nav(bool in_flight);

// settings handlers
extern void guidance_speed_set_h_pgain(float pgain);
extern void guidance_speed_set_h_dgain(float dgain);
extern void guidance_speed_set_h_igain(float igain);
extern void guidance_speed_set_v_pgain(float pgain);
extern void guidance_speed_set_v_dgain(float dgain);
extern void guidance_speed_set_v_igain(float igain);

#ifdef AP_MODE_GUIDED

/** Run GUIDED mode control
 */
extern void guidance_speed_guided_run(bool in_flight);

/** Set horizontal position setpoint in GUIDED mode.
 * @param x North position (local NED frame) in meters.
 * @param y East position (local NED frame) in meters.
 * @return TRUE if setpoints were set
 */
extern bool guidance_h_set_guided_pos(float x, float y);

/** Set heading setpoint in GUIDED mode.
 * @param heading Setpoint in radians.
 * @return TRUE if setpoint was set
 */
extern bool guidance_h_set_guided_heading(float heading);

/** Set body relative horizontal velocity setpoint in GUIDED mode.
 * @param vx forward velocity (body frame) in meters/sec.
 * @param vy right velocity (body frame) in meters/sec.
 * @return TRUE if setpoints were set
 */
extern bool guidance_h_set_guided_body_vel(float vx, float vy);

/** Set horizontal velocity setpoint in GUIDED mode.
 * @param vx North velocity (local NED frame) in meters/sec.
 * @param vy East velocity (local NED frame) in meters/sec.
 * @return TRUE if setpoints were set
 */
extern bool guidance_h_set_guided_vel(float vx, float vy);

/** Set heading rate setpoint in GUIDED mode.
 * @param rate Heading rate in radians.
 * @return TRUE if setpoints were set
 */
extern bool guidance_h_set_guided_heading_rate(float rate);

/** Set z setpoint in GUIDED mode.
 * @param z Setpoint (down is positive) in meters.
 * @return TRUE if setpoint was set
 */
extern bool guidance_v_set_guided_z(float z);

/** Set z velocity setpoint in GUIDED mode.
 * @param vz Setpoint (down is positive) in meters/second.
 * @return TRUE if setpoint was set
 */
extern bool guidance_v_set_guided_vz(float vz);

#endif // AP_MODE_GUIDED

/* Make sure that ref can only be temporarily disabled for testing,
 * but not enabled if GUIDANCE_H_USE_REF was defined to FALSE.
 */
#define guidance_speed_SetUseRef(_val) {                        \
    guidance_speed.use_ref = _val && GUIDANCE_SPEED_USE_REF;    \
  }

#ifdef __cplusplus
}
#endif

#endif /* GUIDANCE_SPEED_H */

/*
 * Copyright (C) 2018 Gautier Hattenberger
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

/** @file filters/pid.h
 *  @brief Several forms of PID controllers
 *
 */

#ifndef PID_H
#define PID_H

/** Distcrete time PID structure.
 *  floating point, fixed frequency.
 *
 * u_k = u_k-1 + a * e_k  + b * e_k-1 + c * e_k-2
 *
 * with:
 *  u = outputs
 *  e = inputs
 *  a = Kp + Ki Ts/2 + Kd/Ts
 *  b = -Kp + Ki Ts/2 - 2 Kd/Ts
 *  c = Kd/Ts
 *  Kp = proportional gain
 *  Kd = derivative gain
 *  Ki = integral gain
 *  Ts = sampling frequency
 *
 */
struct PID_f {
  float u[2]; ///< output
  float e[3]; ///< input
  float g[3]; ///< controller gains
};

/** Init PID struct.
 *
 * @param pid pointer to PID structure
 * @param Kp proportional gain
 * @param Kd derivative gain
 * @param Ki integral gain
 * @param Ts sampling time
 */
static inline void init_pid_f(struct PID_f *pid, float Kp, float Kd, float Ki, float Ts)
{
  *pid = (struct PID_f) {
    { 0.f, 0.f },
    { 0.f ,0.f , 0.f },
    { Kp + Ki * Ts / 2.f + Kd / Ts,
     -Kp + Ki * Ts / 2.f - 2.f * Kd / Ts,
      Kd / Ts }
  };
}

/** Update PID with a new value and return new command.
 *
 * @param pid pointer to PID structure
 * @param value new input value of the PID
 * @return new output command
 */
static inline float update_pid_f(struct PID_f *pid, float value)
{
  pid->e[2] = pid->e[1];
  pid->e[1] = pid->e[0];
  pid->e[0] = value;
  pid->u[1] = pid->u[0];
  pid->u[0] = pid->u[1] + pid->g[0] * pid->e[0] + pid->g[1] * pid->e[1] + pid->g[2] * pid->e[2];
  return pid->u[0];
}

/** Get current value of the PID command.
 *
 * @param pid pointer to PID structure
 * @return current value of PID command
 */
static inline float get_pid_f(struct PID_f *pid)
{
  return pid->u[0];
}

/** Reset PID struture, gains left unchanged.
 *
 * @param pid pointer to PID structure
 */
static inline void reset_pid_f(struct PID_f *pid)
{
  pid->u[0] = 0.f;
  pid->u[1] = 0.f;
  pid->e[0] = 0.f;
  pid->e[1] = 0.f;
  pid->e[2] = 0.f;
}

/** Set gains of the PID struct.
 *
 * @param pid pointer to PID structure
 * @param Kp proportional gain
 * @param Kd derivative gain
 * @param Ki integral gain
 * @param Ts sampling time
 */
static inline void set_gains_pid_f(struct PID_f *pid, float Kp, float Kd, float Ki, float Ts)
{
  pid->g[0] = Kp + Ki * Ts / 2.f + Kd / Ts;
  pid->g[1] = -Kp + Ki * Ts / 2.f - 2.f * Kd / Ts;
  pid->g[2] = Kd / Ts;
}

/** Distcrete time PI-D structure.
 *  derivative term is directly provided as input
 *  as it may be available directly from a sensor
 *  or estimated separately.
 *  floating point, fixed frequency.
 *
 * u_k = u_k-1 + a * e_k  + b * e_k-1 + Kd * d_k
 *
 * with:
 *  u = outputs
 *  e = inputs
 *  d = derivative of input
 *  a = Kp + Ki Ts/2
 *  b = -Kp + Ki Ts/2
 *  Kp = proportional gain
 *  Kd = derivative gain
 *  Ki = integral gain
 *  Ts = sampling frequency
 *
 */
struct PI_D_f {
  float u[2]; ///< output
  float e[2]; ///< input
  float g[3]; ///< controller gains
};

/** Init PI-D struct.
 *
 * @param pid pointer to PID structure
 * @param Kp proportional gain
 * @param Kd derivative gain
 * @param Ki integral gain
 * @param Ts sampling time
 */
static inline void init_pi_d_f(struct PI_D_f *pid, float Kp, float Kd, float Ki, float Ts)
{
  *pid = (struct PI_D_f) {
    { 0.f, 0.f },
    { 0.f ,0.f },
    { Kp + Ki * Ts / 2.f,
     -Kp + Ki * Ts / 2.f,
      Kd }
  };
}

/** Update PI-D with a new value and return new command.
 *
 * @param pid pointer to PI-D structure
 * @param value new input value of the PI-D
 * @param deriv new input derivative
 * @return new output command
 */
static inline float update_pi_d_f(struct PI_D_f *pid, float value, float deriv)
{
  pid->e[1] = pid->e[0];
  pid->e[0] = value;
  pid->u[1] = pid->u[0];
  pid->u[0] = pid->u[1] + pid->g[0] * pid->e[0] + pid->g[1] * pid->e[1] + pid->g[2] * deriv;
  return pid->u[0];
}

/** Get current value of the PI-D command.
 *
 * @param pid pointer to PI-D structure
 * @return current value of PI-D command
 */
static inline float get_pi_d_f(struct PI_D_f *pid)
{
  return pid->u[0];
}

/** Reset PI-D struture, gains left unchanged.
 *
 * @param pid pointer to PI-D structure
 */
static inline void reset_pi_d_f(struct PI_D_f *pid)
{
  pid->u[0] = 0.f;
  pid->u[1] = 0.f;
  pid->e[0] = 0.f;
  pid->e[1] = 0.f;
}

/** Set gains PI-D struct.
 *
 * @param pid pointer to PID structure
 * @param Kp proportional gain
 * @param Kd derivative gain
 * @param Ki integral gain
 * @param Ts sampling time
 */
static inline void set_gains_pi_d_f(struct PI_D_f *pid, float Kp, float Kd, float Ki, float Ts)
{
  pid->g[0] = Kp + Ki * Ts / 2.f;
  pid->g[1] = -Kp + Ki * Ts / 2.f;
  pid->g[2] = Kd;
}

#endif


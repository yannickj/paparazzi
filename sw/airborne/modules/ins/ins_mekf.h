/*
 * Copyright (C) 2017 Marton Brossard <martin.brossard@mines-paristech.fr>
 *                    Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file modules/ins/ins_mekf.h
 *
 * Multiplicative Extended Kalman Filter in rotation matrix formulation.
 *
 * Estimate attitude, ground speed, position, gyro bias, accelerometer bias.
 */

#ifndef INS_MEKF_H
#define INS_MEKF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "std.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_geodetic_float.h"

// Settings
struct ins_mekf_parameters {
  float Q_gyro;       ///< gyro process noise
  float Q_accel;      ///< accel process noise
  float Q_rates_bias; ///< rates bias process noise
  float Q_accel_bias; ///< accel bias process noise
  float Q_baro_bias;  ///< baro bias process noise
  float Q_wind;       ///< wind process noise
  float R_speed;      ///< speed measurement noise
  float R_pos;        ///< pos measurement noise
  float R_speed_z;    ///< vertical speed measurement noise
  float R_pos_z;      ///< vertical pos measurement noise
  float R_mag;        ///< mag measurement noise
  float R_baro;       ///< baro measurement noise
};

extern struct ins_mekf_parameters ins_mekf_params;

// Init functions
extern void ins_mekf_init(void);
extern void ins_mekf_align(struct FloatRates *gyro_bias,
                           struct FloatQuat *quat);
extern void ins_mekf_set_mag_h(const struct FloatVect3 *mag_h);
extern void ins_mekf_reset(void);

// Filtering functions
extern void ins_mekf_propagate(struct FloatRates* gyro,
                               struct FloatVect3* accel, float dt);
extern void ins_mekf_propagate_ahrs(struct FloatRates* gyro,
                                    struct FloatVect3* accel, float dt);
extern void ins_mekf_update_mag(struct FloatVect3* mag, bool attitude_only);
extern void ins_mekf_update_baro(float baro_alt);
extern void ins_mekf_update_pos_speed(struct FloatVect3 *pos,
                                      struct FloatVect3 *speed);

// Getter/Setter functions
extern struct NedCoor_f ins_mekf_get_pos_ned(void);
extern void ins_mekf_set_pos_ned(struct NedCoor_f *p);
extern struct NedCoor_f ins_mekf_get_speed_ned(void);
extern void ins_mekf_set_speed_ned(struct NedCoor_f *s);
extern struct NedCoor_f ins_mekf_get_accel_ned(void);
extern struct FloatQuat ins_mekf_get_quat(void);
extern void ins_mekf_set_quat(struct FloatQuat *quat);
extern struct FloatRates ins_mekf_get_body_rates(void);
extern struct FloatVect3 ins_mekf_get_accel_bias(void);
extern struct FloatRates ins_mekf_get_rates_bias(void);
extern float ins_mekf_get_baro_bias(void);

// Settings handlers
extern void ins_mekf_update_params(void);

#define ins_mekf_update_Q_gyro(_v) { \
  ins_mekf_params.Q_gyro = _v; \
  ins_mekf_update_params(); \
}

#define ins_mekf_update_Q_accel(_v) { \
  ins_mekf_params.Q_accel = _v; \
  ins_mekf_update_params(); \
}

#define ins_mekf_update_Q_rates_bias(_v) { \
  ins_mekf_params.Q_rates_bias = _v; \
  ins_mekf_update_params(); \
}

#define ins_mekf_update_Q_accel_bias(_v) { \
  ins_mekf_params.Q_accel_bias = _v; \
  ins_mekf_update_params(); \
}

#define ins_mekf_update_Q_baro_bias(_v) { \
  ins_mekf_params.Q_baro_bias = _v; \
  ins_mekf_update_params(); \
}

#define ins_mekf_update_R_speed(_v) { \
  ins_mekf_params.R_speed = _v; \
  ins_mekf_update_params(); \
}

#define ins_mekf_update_R_pos(_v) { \
  ins_mekf_params.R_pos = _v; \
  ins_mekf_update_params(); \
}

#define ins_mekf_update_R_speed_z(_v) { \
  ins_mekf_params.R_speed_z = _v; \
  ins_mekf_update_params(); \
}

#define ins_mekf_update_R_pos_z(_v) { \
  ins_mekf_params.R_pos_z = _v; \
  ins_mekf_update_params(); \
}

#define ins_mekf_update_R_mag(_v) { \
  ins_mekf_params.R_mag = _v; \
  ins_mekf_update_params(); \
}

#define ins_mekf_update_R_baro(_v) { \
  ins_mekf_params.R_baro = _v; \
  ins_mekf_update_params(); \
}


#ifdef __cplusplus
}
#endif

#endif /* INS_MEKF_H */


/*
 * Copyright (C) 2017 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/ins/ins_mekf_wrapper.h
 *
 * Paparazzi specific wrapper to run MEKF INS filter.
 */

#ifndef INS_MEKF_WRAPPER_H
#define INS_MEKF_WRAPPER_H

#include "std.h"
#include "math/pprz_orientation_conversion.h"

/** filter structure
 */
struct InsMekf {
  struct OrientationReps body_to_imu;
  bool is_aligned;
  bool baro_initialized;
  bool gps_initialized;
  bool reset;
};

extern struct InsMekf ins_mekf;

extern void ins_mekf_wrapper_init(void);

#define ins_mekf_wrapper_Reset(_v) { \
  ins_mekf.reset = false;            \
  ins_mekf.baro_initialized = false; \
  ins_mekf.gps_initialized = false;     \
  ins_mekf_reset();                  \
}

#endif /* INS_MEKF_WRAPPER_H */


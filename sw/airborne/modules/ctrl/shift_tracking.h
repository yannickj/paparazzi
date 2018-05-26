/*
 * Copyright (C) 2018 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file "modules/ctrl/shift_tracking.h"
 * @author Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *
 * pilot the nav shift variable to track an offset trajectory
 * based on the POSITION_ESTIMATE ABI message
 */

#ifndef SHIFT_TRACKING_H
#define SHIFT_TRACKING_H

#include "std.h"

struct shift_tracking_t {
  float kp;     ///< proportional gain
  float kd;     ///< derivative gain
  float ki;     ///< integral gain
  float shift;  ///< shift command
};

extern struct shift_tracking_t shift_tracking;

/** init function
 */
extern void shift_tracking_init(void);

/** run function
 *
 *  should be called in flight plan pre_call
 *
 *  @param[out] shift pointer to the navigation shift to control
 */
extern void shift_tracking_run(float *shift);

/** reset function
 *
 *  reset integral and offset command
 */
extern void shift_tracking_reset(void);

#endif


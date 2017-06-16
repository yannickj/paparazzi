/*
 * Copyright (C) 2016 Xavier Paris
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

#ifndef CAM_ROLL_L6234_H
#define CAM_ROLL_L6234_H

#include "std.h"

// Configurable by telemetry settings
extern uint8_t Power;
extern uint8_t Proportional;
extern uint8_t Derivative;
extern bool    RotationDir;

extern void cam_roll_l6234_init(void);
extern void cam_roll_l6234_periodic(void);

#endif /* CAM_ROLL_L6234_H */

/*
 * Copyright (C) Gautier Hattenberger <gautier.hattenberger@enac.fr>
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

/** @file "modules/tracking/tag_tracking.h"
 * @author Gautier Hattenberger <gautier.hattenberger@enac.fr>
 * Track poistion of a tag (ArUco, QRcode, ...) detected by an onboard camera
 * The tag detection and pose computation is done outside of the module, only the estimation
 * by fusion of AHRS and visual detection with a Kalman filter is performed in this module
 */

#ifndef TAG_TRACKING_H
#define TAG_TRACKING_H



extern void tag_tracking_init(void);
extern void tag_tracking_propagate(void);
extern void tag_tracking_propagate_start(void);
extern void tag_tracking_report(void);
// extern void compute_command(void);

extern float tag_tracking_roll;
extern float tag_tracking_pitch;
extern float tag_tracking_climb;

extern float tag_tracking_kp;
extern float tag_tracking_kd;


void visualizer_init();
void visualizer_write(float tag_tracking_roll, float tag_tracking_pitch, float tag_tracking_climb);


#endif  // TAG_TRACKING_H

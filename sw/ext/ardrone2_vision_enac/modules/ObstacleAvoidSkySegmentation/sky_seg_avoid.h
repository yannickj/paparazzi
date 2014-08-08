/*
 * Copyright (C) 2012-2013
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file obstacle_avoid.h
 * Outdoor obstacle avoidance for ardrone2.
 *
 * Use the tcp output of a custom GStreamer framework plugin to receive
 */

#ifndef OBSTACLE_AVOID_H
#define OBSTACLE_AVOID_H


// Settings
extern int obstacle_avoid_adjust_factor;


// Module functions
extern void sky_seg_avoid_init(void);
extern void sky_seg_avoid_run(void);
extern void sky_seg_avoid_start(void);
extern void sky_seg_avoid_stop(void);



#endif /* OBSTACLE_AVOID_H */

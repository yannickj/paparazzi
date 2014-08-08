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
 * @file VIEW_VIDEO.h
 */

#ifndef VIEW_VIDEO_H
#define VIEW_VIDEO_H

// Module functions
extern void viewvideo_run(void);
extern void viewvideo_start(void);
extern void viewvideo_stop(void);

// Save picture on disk at full resolution
// can be called from flight plan
extern int viewvideo_save_shot(void);

extern int viewvideo_shot;
#define viewvideo_SaveShot(_v) {  \
  viewvideo_shot = 1;             \
  viewvideo_save_shot();          \
}

#endif /* VIEW_VIDEO_H */

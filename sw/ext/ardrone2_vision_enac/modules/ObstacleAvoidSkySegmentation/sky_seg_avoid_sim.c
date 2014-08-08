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


// Own header
#include "sky_seg_avoid.h"

// UDP Message with GST vision
#include "udp/socket.h"
#include "video_message_structs.h"

// Navigate Based On Vision
#include "avoid_navigation.h"

// Paparazzi State: Attitude -> Vision
#include "state.h" // for attitude


struct gst2ppz_message_struct gst2ppz;
struct ppz2gst_message_struct ppz2gst;
int obstacle_avoid_adjust_factor;


void sky_seg_avoid_init(void) {
  // Simulated vision: obstacle at 0,0
  gst2ppz.ID = 0x0004;
  for (int i=0; i<N_BINS; i++)
  {
    gst2ppz.obstacle_bins[i] = 0;
  }
  obstacle_avoid_adjust_factor = 4;

  // Navigation Code
  init_avoid_navigation();
}


void sky_seg_avoid_run(void) {
  static int counter = 0;

  counter++;
  // Read Latest GST Module Results
  if (counter >= (512/15))
  {
    float dh = (17.0f - stateGetPositionEnu_f()->z) * 10.0f;
    for (int i=0; i<N_BINS; i++)
    {
      gst2ppz.obstacle_bins[i] = dh;
    }

    counter = 0;
    run_avoid_navigation_onvision();
  }
}


void sky_seg_avoid_start(void) {
}


void sky_seg_avoid_stop(void) {
}

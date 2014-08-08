/*
 * Copyright (C) 2014 Gautier Hattenberger
 *
 * This file is part of paparazzi

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

#include "modules/sonar/ground_alt.h"

#include "subsystems/abi.h"

float agl_dist_valid;
float agl_dist_value;
float agl_dist_value_filtered;

/** default sonar to use in INS */
#ifndef INS_SONAR_ID
#define INS_SONAR_ID ABI_BROADCAST
#endif
#ifndef INS_SONAR_MAX_RANGE
#define INS_SONAR_MAX_RANGE 5.0
#endif
#ifndef INS_SONAR_MIN_RANGE
#define INS_SONAR_MIN_RANGE 0.001
#endif
abi_event sonar_ev;

#ifndef INS_SONAR_FILTER
#define INS_SONAR_FILTER 5
#endif

static void sonar_cb(uint8_t sender_id, const float *distance);

void agl_dist_init(void) {
  agl_dist_valid = FALSE;
  agl_dist_value = 0.;
  agl_dist_value_filtered = 0.;

  // Bind to AGL message
  AbiBindMsgAGL(INS_SONAR_ID, &sonar_ev, sonar_cb);
}


static void sonar_cb(uint8_t __attribute__((unused)) sender_id, const float *distance) {

  /* update filter assuming a flat ground */
  if (*distance < INS_SONAR_MAX_RANGE && *distance > INS_SONAR_MIN_RANGE) {
    agl_dist_value = *distance;
    agl_dist_valid = TRUE;
    agl_dist_value_filtered = (INS_SONAR_FILTER * agl_dist_value_filtered + agl_dist_value) / (INS_SONAR_FILTER + 1);
  }
  else {
    agl_dist_valid = FALSE;
  }
}

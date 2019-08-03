/*
 * Copyright (C) 2019 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file "modules/computer_vision/cv_target_localization.c"
 * @author Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *
 * Compute georeferenced position of a target from visual detection
 * assuming that the target is on a flat ground
 */

#include "modules/computer_vision/cv_target_localization.h"
#include "std.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_geodetic_float.h"
#include "state.h"
#include "subsystems/abi.h"

// Default parameters
// Camera is looking down and is placed at the center of the frame
// With cam X axis pointing to the right, Y down and Z forward of image frame,
// the camera is just rotated of pi/2 around body Z axis

#ifndef TARGET_LOC_BODY_TO_CAM_PHI
#define TARGET_LOC_BODY_TO_CAM_PHI 0.f
#endif

#ifndef TARGET_LOC_BODY_TO_CAM_THETA
#define TARGET_LOC_BODY_TO_CAM_THETA 0.f
#endif

#ifndef TARGET_LOC_BODY_TO_CAM_PSI
#define TARGET_LOC_BODY_TO_CAM_PSI M_PI_2
#endif

#ifndef TARGET_LOC_CAM_POS_X
#define TARGET_LOC_CAM_POS_X 0.f
#endif

#ifndef TARGET_LOC_CAM_POS_Y
#define TARGET_LOC_CAM_POS_Y 0.f
#endif

#ifndef TARGET_LOC_CAM_POS_Z
#define TARGET_LOC_CAM_POS_Z 0.f
#endif

// Detection and target
struct target_loc_t {
  int16_t px;                   ///< Target in camera frame
  int16_t py;                   ///< Target in camera frame

  struct FloatRMat body_to_cam; ///< Body to camera rotation
  struct FloatVect3 cam_pos;    ///< Position of camera in body frame

  struct FloatVect3 target;     ///< Target position in LTP-NED frame
  struct LlaCoor_f pos_lla;     ///< Target global position in LLA

  bool valid;                   ///< True if a target have been seen
  uint8_t type;                 ///< Type of target
};

static struct target_loc_t target_loc;


// Abi bindings
#ifndef TARGET_LOC_ID
#define TARGET_LOC_ID ABI_BROADCAST
#endif

abi_event detection_ev;

static void detection_cb(uint8_t sender_id UNUSED,
    int16_t pixel_x, int16_t pixel_y,
    int16_t pixel_width UNUSED, int16_t pixel_height UNUSED,
    int32_t quality, int16_t extra)
{
  target_loc.px = pixel_x;
  target_loc.py = pixel_y;

  // Prepare rotation matrices
  struct FloatRMat *ltp_to_body_rmat = stateGetNedToBodyRMat_f();
  struct FloatRmat ltp_to_cam_rmat;
  float_rmat_comp(&ltp_to_cam_rmat, ltp_to_body_rmat, &target_loc.body_to_cam);
  // Prepare cam world position
  // C_w = P_w + R_w2b * C_b
  struct FloatVect3 cam_pos_ltp;
  float_rmat_vmult(&cam_pos_ltp, ltp_to_body_rmat, &target_loc.cam_pos);
  VECT3_ADD(cam_pos_ltp, stateGetPositionNed_f());

  // Compute target position here

  target_loc.type = (uint8_t) extra; // use 'extra' field to encode the type of target
  target_loc.valid = true;
}

void target_localization_init(void)
{
  // Init struct
  target_loc.px = 0;
  target_loc.py = 0;

  struct FloatEulers euler = {
    TARGET_LOC_BODY_TO_CAM_PHI,
    TARGET_LOC_BODY_TO_CAM_THETA,
    TARGET_LOC_BODY_TO_CAM_PSI
  };
  float_rmat_of_eulers(&target_loc.body_to_cam, &euler);
  VECT3_ASSIGN(target_loc.camera_pos,
      TARGET_LOC_CAM_POS_X,
      TARGET_LOC_CAM_POS_Y,
      TARGET_LOC_CAM_POS_Z);

  VECT3_ZERO(target_loc.target);

  target_loc.valid = false;

  // Bind to ABI message
  AbiBindMsgVISUAL_DETECTION(TARGET_LOC_ID, &detection_ev, detection_cb);
}

void target_localization_report(void)
{
  if (taget_loc.valid) {
    DOWNLINK_SEND_MARK(DefaultChannel, DefaultDevice, &taget_loc.type,
        &target_loc.pos_lla.lat, &target_loc.pos_lla.lon);
    target_loc.valid = false;
  }
}


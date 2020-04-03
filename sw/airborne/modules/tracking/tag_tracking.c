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
#include "modules/tracking/kalman.h"
#include "modules/tracking/tag_tracking.h"
#include "generated/modules.h"
#include "math/pprz_algebra_float.h"
#include "state.h"
#include "subsystems/abi.h"


#if defined SITL
#include "generated/flight_plan.h"
static void tag_tracking_sim(void);
static void tag_motion_sim(void);

// use WP_TARGET by default
#if !(defined TAG_TRACKING_SIM_WP) && (defined WP_TARGET)
#define TAG_TRACKING_SIM_WP WP_TARGET
#endif

// select print function for debug
#include <stdio.h>
//#define PRINTF printf
#define PRINTF(...) {}

#define TAG_MOTION_NONE 0
#define TAG_MOTION_LINE 1
#define TAG_MOTION_CIRCLE 2

#define TAG_MOTION_SPEED_X 0.5f
#define TAG_MOTION_SPEED_Y 0.f
#define TAG_MOTION_RANGE_X 4.f
#define TAG_MOTION_RANGE_Y 4.f

static uint8_t tag_motion_sim_type = TAG_MOTION_LINE;
static struct FloatVect3 tag_motion_speed = { TAG_MOTION_SPEED_X, TAG_MOTION_SPEED_Y, 0.f };
#endif

// Default parameters
// Camera is looking down and is placed at the center of the frame
// With cam X axis pointing to the right, Y down and Z forward of image frame,
// the camera is just rotated of pi/2 around body Z axis

#ifndef TAG_TRACKING_BODY_TO_CAM_PHI
#define TAG_TRACKING_BODY_TO_CAM_PHI 0.f
#endif

#ifndef TAG_TRACKING_BODY_TO_CAM_THETA
#define TAG_TRACKING_BODY_TO_CAM_THETA 0.f
#endif

#ifndef TAG_TRACKING_BODY_TO_CAM_PSI
#define TAG_TRACKING_BODY_TO_CAM_PSI M_PI_2
#endif

#ifndef TAG_TRACKING_CAM_POS_X
#define TAG_TRACKING_CAM_POS_X 0.f
#endif

#ifndef TAG_TRACKING_CAM_POS_Y
#define TAG_TRACKING_CAM_POS_Y 0.f
#endif

#ifndef TAG_TRACKING_CAM_POS_Z
#define TAG_TRACKING_CAM_POS_Z 0.f
#endif

#define R 1
#define Q_SIGMA2 1
#define DT 1
#define P0_POS 10
#define P0_SPEED 10
// generated in modules.h
static const float tag_track_dt = TAG_TRACKING_PROPAGATE_PERIOD;

// global state structure
struct tag_tracking {
  struct FloatVect3 meas;       ///< measured position
  struct FloatVect3 pos;        ///< estimated position
  struct FloatVect3 speed;      ///< estimated speed
  float heading;                ///< estimated heading

  struct FloatRMat body_to_cam; ///< Body to camera rotation
  struct FloatVect3 cam_pos;    ///< Position of camera in body frame
};

static struct tag_tracking tag_track;
struct Kalman kalman;

// Abi bindings
#ifndef TAG_TRACKING_ID
#define TAG_TRACKING_ID ABI_BROADCAST
#endif

static abi_event tag_track_ev;

static void tag_track_cb(uint8_t sender_id UNUSED,
     uint8_t type, char * id UNUSED,
     uint8_t nb UNUSED, int16_t * coord, uint16_t * dim UNUSED,
     struct FloatQuat quat UNUSED, char * extra UNUSED)
{
  if (type == JEVOIS_MSG_D3) {
    // store data from Jevois detection
    tag_track.meas.x = coord[0];
    tag_track.meas.y = coord[1];
    tag_track.meas.z = coord[2];

    // TODO call correction step from here
    kalman_update(&kalman, tag_track.meas);
  }
}

// Init function
void tag_tracking_init()
{
  // Init structure
  FLOAT_VECT3_ZERO(tag_track.meas);
  FLOAT_VECT3_ZERO(tag_track.pos);
  FLOAT_VECT3_ZERO(tag_track.speed);
  struct FloatEulers euler = {
    TAG_TRACKING_BODY_TO_CAM_PHI,
    TAG_TRACKING_BODY_TO_CAM_THETA,
    TAG_TRACKING_BODY_TO_CAM_PSI
  };
  float_rmat_of_eulers(&tag_track.body_to_cam, &euler);
  VECT3_ASSIGN(tag_track.cam_pos,
      TAG_TRACKING_CAM_POS_X,
      TAG_TRACKING_CAM_POS_Y,
      TAG_TRACKING_CAM_POS_Z);

  // Bind to ABI message
  AbiBindMsgJEVOIS_MSG(TAG_TRACKING_ID, &tag_track_ev, tag_track_cb);
}

// Propagation function
void tag_tracking_propagate()
{
#if defined SITL && defined TAG_TRACKING_SIM_WP
  if (tag_motion_sim_type != TAG_MOTION_NONE) {
    tag_motion_sim();
  }
  tag_tracking_sim();
#endif

  // TODO call kalman propagation step
  kalman_predict(&kalman);
}

// Propagation start function (called at each start state
void tag_tracking_propagate_start()
{    
  // your periodic start code here.

  kalman_init(&kalman, P0_POS, P0_SPEED, Q_SIGMA2, R, DT);
  kalman_set_state(&kalman, tag_track.meas, tag_track.speed);
  // TODO reset kalman state ?
}

// Report function
void tag_tracking_report()
{
  // your periodic code here.
  // freq = 1.0 Hz
}


// Simulate detection using a WP coordinate
#if defined SITL && defined TAG_TRACKING_SIM_WP
static void tag_tracking_sim(void)
{
  // Compute image coordinates of a WP given fake camera parameters
  struct FloatRMat *ltp_to_body_rmat = stateGetNedToBodyRMat_f();
  struct FloatRMat ltp_to_cam_rmat;
  float_rmat_comp(&ltp_to_cam_rmat, ltp_to_body_rmat, &tag_track.body_to_cam);
  // Prepare cam world position
  // C_w = P_w + R_w2b * C_b
  struct FloatVect3 cam_pos_ltp;
  float_rmat_vmult(&cam_pos_ltp, ltp_to_body_rmat, &tag_track.cam_pos);
  VECT3_ADD(cam_pos_ltp, *stateGetPositionNed_f());
  PRINTF("Cw %f %f %f\n", cam_pos_ltp.x, cam_pos_ltp.y, cam_pos_ltp.z);
  // Target
  struct NedCoor_f target_ltp;
  ENU_OF_TO_NED(target_ltp, waypoints[TAG_TRACKING_SIM_WP].enu_f);
  target_ltp.z = 0.f; // force on the ground
  PRINTF("Tw %f %f %f\n", target_ltp.x, target_ltp.y, target_ltp.z);
  // Compute target in camera frame Pc = R * (Pw - C)
  struct FloatVect3 target_cam, tmp;
  VECT3_DIFF(tmp, target_ltp, cam_pos_ltp);
  float_rmat_vmult(&target_cam, &ltp_to_cam_rmat, &tmp);
  PRINTF("Tc %f %f %f\n", target_cam.x, target_cam.y, target_cam.z);
  if (fabsf(target_cam.z) > 1.) {
    // If we are not too close from target
    // Compute target in image frame x = X/Z, y = X/Z
    if (fabsf(target_cam.x / target_cam.z) < 0.3f &&
        fabsf(target_cam.y / target_cam.z) < 0.3f) {
      // If in field of view (~tan(60)/2)
      // send coordinates in millimeter
      int16_t coord[3] = {
        (int16_t) (target_cam.x * 1000.f),
        (int16_t) (target_cam.y * 1000.f),
        (int16_t) (target_cam.z * 1000.f)
      };
      uint16_t dim[3] = { 100, 100, 0 };
      struct FloatQuat quat; // TODO
      float_quat_identity(&quat);
      PRINTF("Sending Abi Msg %d %d %d\n", coord[0], coord[1], coord[2]);
      AbiSendMsgJEVOIS_MSG(42, JEVOIS_MSG_D3, "1", 3, coord, dim, quat, "");
    }
  }
  fflush(stdout);
}

static void tag_motion_sim(void)
{
  switch (tag_motion_sim_type) {
    case TAG_MOTION_LINE:
      {
        struct EnuCoor_f pos = waypoints[TAG_TRACKING_SIM_WP].enu_f;
        struct FloatVect3 speed_dt = tag_motion_speed;
        VECT2_SMUL(speed_dt, speed_dt, tag_track_dt);
        if (pos.x < -TAG_MOTION_RANGE_X || pos.x > TAG_MOTION_RANGE_X ||
            pos.y < -TAG_MOTION_RANGE_Y || pos.y > TAG_MOTION_RANGE_Y) {
          tag_motion_speed.x = -tag_motion_speed.x;
          tag_motion_speed.y = -tag_motion_speed.y;
          speed_dt.x = -speed_dt.x;
          speed_dt.y = -speed_dt.y;
        }
        VECT2_ADD(pos, speed_dt);
        struct EnuCoor_i pos_i;
        ENU_BFP_OF_REAL(pos_i, pos);
        //waypoint_set_enu(TAG_TRACKING_SIM_WP, &pos);
        waypoint_move_enu_i(TAG_TRACKING_SIM_WP, &pos_i);
        break;
      }
    default:
      break;
  }
}

#endif



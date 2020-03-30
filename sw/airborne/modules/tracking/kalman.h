#ifndef KALMAN_H
#define KALMAN_H

#include "std.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_geodetic_float.h"
#include "math/pprz_matrix_simple.h"

#define EKF_RANGE_DIM 6

/** EKF_range structure
 *
 * state vector: X = [ x xd y yd z zd ]'
 * command vector: U = 0 (constant velocity model)
 * dynamic model: basic kinematic model x_k+1 = x_k + xd_k * dt
 * measures: distance between (fixed and known) anchors and UAV
 *
 * */
struct EKFRange {
  float state[EKF_RANGE_DIM];             ///< state vector
  float P[EKF_RANGE_DIM][EKF_RANGE_DIM];  ///< covariance matrix
  float Q[EKF_RANGE_DIM][EKF_RANGE_DIM];  ///< process noise matrix
  float R_dist;                           ///< measurement noise on distances (assumed the same for all anchors)
  float R_speed;                          ///< measurement noise on speed (assumed the same for all axis)
  float dt;                               ///< prediction step (in seconds)
};

/** Init EKF_range internal struct
 *
 * @param[in] ekf_range EKFRange structure
 * @param[in] P0_pos initial covariance on position
 * @param[in] P0_speed initial covariance on speed
 * @param[in] Q_sigma2 process noise
 * @param[in] R_dist measurement noise on distance
 * @param[in] R_speed measurement noise on speed
 * @param[in] dt prediction time step in seconds
 */
extern void ekf_range_init(struct EKFRange *ekf_range, float P0_pos, float P0_speed, float Q_sigma2, float R_dist, float R_speed, float dt);

/** Set initial state vector
 *
 * This function should be called after initialization of the ekf struct and before
 * running the filter for better results and faster convergence
 *
 * @param[in] ekf_range EKFRange structure
 * @param[in] pos initial position
 * @param[in] speed initial speed
 */
extern void ekf_range_set_state(struct EKFRange *ekf_range, struct EnuCoor_f pos, struct EnuCoor_f speed);

/** Get current state
 *
 * @param[in] ekf_range EKFRange structure
 * @param[out] pos current position
 * @param[out] speed current speed
 */
extern void ekf_range_get_state(struct EKFRange *ekf_range, struct EnuCoor_f *pos, struct EnuCoor_f *speed);

/** Get current pos
 *
 * @param[in] ekf_range EKFRange structure
 * @return current position
 */
extern struct EnuCoor_f ekf_range_get_pos(struct EKFRange *ekf_range);

/** Get current speed
 *
 * @param[in] ekf_range EKFRange structure
 * @return current speed
 */
extern struct EnuCoor_f ekf_range_get_speed(struct EKFRange *ekf_range);

/** Update process and measurement noises
 *
 * @param[in] ekf_range EKFRange structure
 * @param[in] Q_sigma2 process noise
 * @param[in] R_dist measurement noise on distance
 * @param[in] R_speed measurement noise on speed
 */
extern void ekf_range_update_noise(struct EKFRange *ekf_range, float Q_sigma2, float R_dist, float R_speed);

/** Prediction step
 *
 * @param[in] ekf_range EKFRange structure
 */
extern void ekf_range_predict(struct EKFRange *ekf_range);

/** Update step based on each new distance data
 *
 * @param[in] ekf_range EKFRange structure
 * @param[in] dist new distance measurement
 * @param[in] anchor position of the anchor from which the distance is measured
 */
extern void ekf_range_update_dist(struct EKFRange *ekf_range, float dist, struct EnuCoor_f anchor);

/** Update step based on speed measure
 *
 * @param[in] ekf_range EKFRange structure
 * @param[in] speed new speed measurement
 * @param[in] type 1: horizontal ground speed norm, 2: vertical ground speed norm, 3: 3D ground speed norm
 */
extern void ekf_range_update_speed(struct EKFRange *ekf_range, float speed, uint8_t type);


#endif



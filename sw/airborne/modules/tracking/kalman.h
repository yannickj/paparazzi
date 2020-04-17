#ifndef KALMAN_H
#define KALMAN_H


#include "std.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_geodetic_float.h"
#include "subsystems/commands.h"
#define KALMAN_DIM 6

/** Kalman structure
 *
 * state vector: X = [ x xd y yd z zd ]'
 * command vector: U = 0 (constant velocity model)
 * dynamic model: basic kinematic model x_k+1 = x_k + xd_k * dt
 * measures: distance between (fixed and known) anchors and UAV
 *
 * */
struct Kalman
{
  float state[KALMAN_DIM];         ///< state vector
  float P[KALMAN_DIM][KALMAN_DIM]; ///< covariance matrix
  float Q[KALMAN_DIM][KALMAN_DIM]; ///< process noise matrix
  float r;                         ///< measurement noise (assumed the same for all anchors)
  float dt;                        ///< prediction step (in seconds)
  float F[KALMAN_DIM][KALMAN_DIM];
  float H[KALMAN_DIM/2][KALMAN_DIM];
  
};

/** Init Kalman internal struct
 *
 * @param[in] kalman Kalman structure
 * @param[in] P0_pos initial covariance on position
 * @param[in] P0_speed initial covariance on speed
 * @param[in] Q_sigma2 process noise
 * @param[in] r measurement noise 
 * @param[in] dt prediction time step in seconds
 */
extern void kalman_init(struct Kalman *kalman, float P0_pos, float P0_speed, float Q_sigma2, float r, float dt);

/** Set initial state vector
 *
 * This function should be called after initialization of the kalman struct and before
 * running the filter for better results and faster convergence
 *
 * @param[in] kalman Kalman structure
 * @param[in] pos initial position
 * @param[in] speed initial speed
 */
extern void kalman_set_state(struct Kalman *kalman, struct FloatVect3 pos, struct FloatVect3 speed);

/** Get current state
 *
 * @param[in] kalman Kalman structure
 * @param[out] pos current position
 * @param[out] speed current speed
 */
extern void kalman_get_state(struct Kalman *kalman, struct FloatVect3 *pos, struct FloatVect3 *speed);

/** Get current pos
 *
 * @param[in] kalman Kalman structure
 * @return current position
 */
extern struct FloatVect3 kalman_get_pos(struct Kalman *kalman);

/** Get current speed
 *
 * @param[in] kalman Kalman structure
 * @return current speed
 */
extern struct FloatVect3 kalman_get_speed(struct Kalman *kalman);

/** Update process and measurement noises
 *
 * @param[in] kalman Kalman structure
 * @param[in] Q_sigma2 process noise
 * @param[in] r measurement noise 
 */
extern void kalman_update_noise(struct Kalman *kalman, float Q_sigma2, float r);

/** Prediction step
 *
 * @param[in] kalman Kalman structure
 */
extern void kalman_predict(struct Kalman *kalman);

/** Update step based on each new distance data
 *
 * @param[in] kalman Kalman structure
 * @param[in] anchor position of the anchor from which the distance is measured
 */
extern void kalman_update(struct Kalman *kalman, struct FloatVect3 anchor);

/** Update step based on speed measure
 *
 * @param[in] kalman Kalman structure
 * @param[in] speed new speed measurement
 * @param[in] type 1: horizontal ground speed norm, 2: vertical ground speed norm, 3: 3D ground speed norm
 */
extern void kalman_update_speed(struct Kalman *kalman, float speed, uint8_t type);

#endif

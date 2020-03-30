#include "ekf_range.h"
#include <math.h>

void ekf_range_init(struct EKFRange *ekf_range, float P0_pos, float P0_speed, float Q_sigma2, float R_dist, float R_speed, float dt)
{
  int i,j;
  const float dt2 = dt * dt;
  const float dt3 = dt2 * dt / 2.f;
  const float dt4 = dt2 * dt2 / 4.f;
  for (i = 0; i < EKF_RANGE_DIM; i++) {
    ekf_range->state[i] = 0.f; // don't forget to call set_state before running the filter for better results
    for (j = 0; j < EKF_RANGE_DIM; j++) {
      ekf_range->P[i][j] = 0.f;
      ekf_range->Q[i][j] = 0.f;
    }
  }
  for (i = 0; i < EKF_RANGE_DIM; i += 2) {
    ekf_range->P[i][i] = P0_pos;
    ekf_range->P[i+1][i+1] = P0_speed;
    ekf_range->Q[i][i] = Q_sigma2 * dt4;
    ekf_range->Q[i+1][i] = Q_sigma2 * dt3;
    ekf_range->Q[i][i+1] = Q_sigma2 * dt3;
    ekf_range->Q[i+1][i+1] = Q_sigma2 * dt2;
  }
  ekf_range->R_dist = R_dist;
  ekf_range->R_speed = R_speed;
  ekf_range->dt = dt;

  ekf_range->F = {{1 ;dt; 0; 0 ; 0; 0};
                 {0; 1 ; 0; 0;  0; 0};
                 {0; 0;  1; dt 0; 0};
                 {0; 0;  0; 1  0; 0};
                 {0; 0;  0; 0;  1 ;dt};
                 {0; 0;  0; 0;  0; 1 }}
  ekf_range->H = {{1;0;0;0;0;0};
                  {0;0;1;0;0;0};
                  {0;0;0;0;1;0}}
}

void ekf_range_set_state(struct EKFRange *ekf_range, struct EnuCoor_f pos, struct EnuCoor_f speed)
{
  ekf_range->state[0] = pos.x;
  ekf_range->state[1] = speed.x;
  ekf_range->state[2] = pos.y;
  ekf_range->state[3] = speed.y;
  ekf_range->state[4] = pos.z;
  ekf_range->state[5] = speed.z;
}

void ekf_range_get_state(struct EKFRange *ekf_range, struct EnuCoor_f *pos, struct EnuCoor_f *speed)
{
  pos->x = ekf_range->state[0];
  pos->y = ekf_range->state[2];
  pos->z = ekf_range->state[4];
  speed->x = ekf_range->state[1];
  speed->y = ekf_range->state[3];
  speed->z = ekf_range->state[5];
}

struct EnuCoor_f ekf_range_get_pos(struct EKFRange *ekf_range)
{
  struct EnuCoor_f pos;
  pos.x = ekf_range->state[0];
  pos.y = ekf_range->state[2];
  pos.z = ekf_range->state[4];
  return pos;
}

struct EnuCoor_f ekf_range_get_speed(struct EKFRange *ekf_range)
{
  struct EnuCoor_f speed;
  speed.x = ekf_range->state[1];
  speed.y = ekf_range->state[3];
  speed.z = ekf_range->state[5];
  return speed;
}

void ekf_range_update_noise(struct EKFRange *ekf_range, float Q_sigma2, float R_dist, float R_speed)
{
  int i;
  const float dt = ekf_range->dt;
  const float dt2 = dt * dt;
  const float dt3 = dt2 * dt / 2.f;
  const float dt4 = dt2 * dt2 / 4.f;
  for (i = 0; i < EKF_RANGE_DIM; i += 2) {
    ekf_range->Q[i][i] = Q_sigma2 * dt4;
    ekf_range->Q[i+1][i] = Q_sigma2 * dt3;
    ekf_range->Q[i][i+1] = Q_sigma2 * dt3;
    ekf_range->Q[i+1][i+1] = Q_sigma2 * dt2;
  }
  ekf_range->R_dist = R_dist;
  ekf_range->R_speed = R_speed;
}

/** propagate dynamic model
 *
 * F = [ 1 dt 0 0  0 0
 *       0 1  0 0  0 0
 *       0 0  1 dt 0 0
 *       0 0  0 1  0 0
 *       0 0  0 0  1 dt
 *       0 0  0 0  0 1  ]
 */
void ekf_range_predict(struct EKFRange *ekf_range)
{
  int i;
  for (i = 0; i < EKF_RANGE_DIM; i += 2) {
    // kinematic equation of the dynamic model X = F*X
    ekf_range->state[i] += ekf_range->state[i+1] * ekf_range->dt;
  // ekf_range->state = matrix_product(ekf_range->F, ekf_range->state, EKF_RANGE_DIM, EKF_RANGE_DIM, 1);
    
    // propagate covariance P = F*P*Ft + Q
    // since F is diagonal by block, P can be updated by block here as well
    // let's unroll the matrix operations as it is simple
  // ekf_range->P = matrix_product(ekf_range->F, matrix_product);
    
    const float d_dt = ekf_range->P[i+1][i+1] * ekf_range->dt;
    ekf_range->P[i][i] += ekf_range->P[i+1][i] * ekf_range->dt
      + ekf_range->dt * (ekf_range->P[i][i+1] + d_dt) + ekf_range->Q[i][i];
    ekf_range->P[i][i+1] += d_dt + ekf_range->Q[i][i+1];
    ekf_range->P[i+1][i] += d_dt + ekf_range->Q[i+1][i];
    ekf_range->P[i+1][i+1] += ekf_range->Q[i+1][i+1];
  }
}

/** correction step
 *
 * K = PHt(HPHt+R)^1
 * X = X + K(z-h(X))
 * P = (I-KH)P
 */
void ekf_range_update_dist(struct EKFRange *ekf_range, float dist, struct EnuCoor_f anchor)
{
  const float dx = ekf_range->state[0] - anchor.x;
  const float dy = ekf_range->state[2] - anchor.y;
  const float dz = ekf_range->state[4] - anchor.z;
  const float norm = sqrtf(dx * dx + dy * dy + dz * dz);
  // build measurement error
  const float res = dist - norm;
  // build Jacobian of observation model for anchor i
  // float Hi[] = { dx / norm, 0.f, dy / norm, 0.f, dz / norm, 0.f };
  
  // compute kalman gain K = P*Ht (H*P*Ht + R)^-1
  // S = H*P*Ht + R
  
  // const float S =
  //   Hi[0] * Hi[0] * ekf_range->P[0][0] +
  //   Hi[2] * Hi[2] * ekf_range->P[2][2] +
  //   Hi[4] * Hi[4] * ekf_range->P[4][4] +
  //   Hi[0] * Hi[2] * (ekf_range->P[0][2] + ekf_range->P[2][0]) +
  //   Hi[0] * Hi[4] * (ekf_range->P[0][4] + ekf_range->P[4][0]) +
  //   Hi[2] * Hi[4] * (ekf_range->P[2][4] + ekf_range->P[4][2]) +
  //   ekf_range->R_dist;

  const float S[3][3] = { {p[0][0] + r; p[0][2]; p[0][4]};
                            {p[2][0]; p[2][2]+r ; p[2][4]};
                            {p[4][0]; p[4][2]; p[4][4]+r}};


  if (fabsf(S[0][0]) + fabsf(S[1][1]) + fabsf(S[2][2]) < 1e-5) {
    return; // don't inverse S if it is too small
  }
  // finally compute gain and correct state
  // float K[6];

  float invS[3][3];
  MAT_INV33(invS, S);

  float K[6][3];
  float HinvS_tmp[6][3];
  MAT_MUL(6, 3, 3, HinvS_tmp, matrix_transpose(ekf_range->H) , invS);
  MAT_MUL(6, 6, 3, K, P, HinvS_tmp);

  // for (int i = 0; i < 6; i++) {
  //   K[i] = (Hi[0] * ekf_range->P[i][0] + Hi[2] * ekf_range->P[i][2] + Hi[4] * ekf_range->P[i][4]) / S;
  //   ekf_range->state[i] += K[i] * res;
  // }

  // precompute K*H and store current P
  float KH_tmp[6][6];
  float P_tmp[6][6];
  MAT_MUL(6,3,6, KH_tmp, K, ekf_range->H);
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      // KH_tmp[i][j] = K[i] * Hi[j];
      P_tmp[i][j] = ekf_range->P[i][j];
    }
  }
  // correct covariance P = (I-K*H)*P = P - K*H*P
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      for (int k = 0; k < 6; k++) {
        ekf_range->P[i][j] -= KH_tmp[i][k] * P_tmp[k][j];
      }
    }
  }
}

void ekf_range_update_speed(struct EKFRange *ekf_range, float speed, uint8_t type)
{
  (void) ekf_range;
  (void) speed;
  (void) type;
  // TODO
}

float** matrix_product(float** a, float** b, int n, int p, int q){
  float c[n][q];
  for (int i=0; i<n; i++){
    for (int j=0; j<q; j++){
      c[i][j] = 0;
      for (int k=0, k<p, k++){
        c[i][j] += a[i][k]*b[k][j];
      }
    }
  }
  return c;
}

float** matrix_transpose(float** m, int n, int p){
  float mt[p][n];
  for (int i=0; i<n, i++){
    for (int j=0; j<p, j++){
      mt[i][j]=m[j][i];
    }
  }
  return mt;
}

float** matrix_inverse(float** m, int n, int p){
  //todo
  float mi[n][p];
  return mi;
}
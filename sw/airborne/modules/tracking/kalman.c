#include "kalman.h"
#include <math.h>

void kalman_init(struct Kalman *kalman, float P0_pos, float P0_speed, float Q_sigma2, float r, float dt)
{
  int i,j;
  const float dt2 = dt * dt;
  const float dt3 = dt2 * dt / 2.f;
  const float dt4 = dt2 * dt2 / 4.f;
  for (i = 0; i < KALMAN_DIM; i++) {
    kalman->state[i] = 0.f; // don't forget to call set_state before running the filter for better results
    for (j = 0; j < KALMAN_DIM; j++) {
      kalman->P[i][j] = 0.f;
      kalman->Q[i][j] = 0.f;
    }
  }
  for (i = 0; i < KALMAN_DIM; i += 2) {
    kalman->P[i][i] = P0_pos;
    kalman->P[i+1][i+1] = P0_speed;
    kalman->Q[i][i] = Q_sigma2 * dt4;
    kalman->Q[i+1][i] = Q_sigma2 * dt3;
    kalman->Q[i][i+1] = Q_sigma2 * dt3;
    kalman->Q[i+1][i+1] = Q_sigma2 * dt2;
  }
  kalman->r = r;
  kalman->dt = dt;

  kalman->F = {{1 ;dt; 0; 0 ; 0; 0};
                 {0; 1 ; 0; 0;  0; 0};
                 {0; 0;  1; dt 0; 0};
                 {0; 0;  0; 1  0; 0};
                 {0; 0;  0; 0;  1 ;dt};
                 {0; 0;  0; 0;  0; 1 }}
  kalman->H = {{1;0;0;0;0;0};
                  {0;0;1;0;0;0};
                  {0;0;0;0;1;0}}
}

void kalman_set_state(struct Kalman *kalman, struct FloatVect3 pos, struct FloatVect3 speed)
{
  kalman->state[0] = pos.x;
  kalman->state[1] = speed.x;
  kalman->state[2] = pos.y;
  kalman->state[3] = speed.y;
  kalman->state[4] = pos.z;
  kalman->state[5] = speed.z;
}

void kalman_get_state(struct Kalman *kalman, struct FloatVect3 *pos, struct FloatVect3 *speed)
{
  pos->x = kalman->state[0];
  pos->y = kalman->state[2];
  pos->z = kalman->state[4];
  speed->x = kalman->state[1];
  speed->y = kalman->state[3];
  speed->z = kalman->state[5];
}

struct FloatVect3 kalman_get_pos(struct Kalman *kalman)
{
  struct FloatVect3 pos;
  pos.x = kalman->state[0];
  pos.y = kalman->state[2];
  pos.z = kalman->state[4];
  return pos;
}

struct FloatVect3 kalman_get_speed(struct Kalman *kalman)
{
  struct FloatVect3 speed;
  speed.x = kalman->state[1];
  speed.y = kalman->state[3];
  speed.z = kalman->state[5];
  return speed;
}

void kalman_update_noise(struct Kalman *kalman, float Q_sigma2, float r)
{
  int i;
  const float dt = kalman->dt;
  const float dt2 = dt * dt;
  const float dt3 = dt2 * dt / 2.f;
  const float dt4 = dt2 * dt2 / 4.f;
  for (i = 0; i < KALMAN_DIM; i += 2) {
    kalman->Q[i][i] = Q_sigma2 * dt4;
    kalman->Q[i+1][i] = Q_sigma2 * dt3;
    kalman->Q[i][i+1] = Q_sigma2 * dt3;
    kalman->Q[i+1][i+1] = Q_sigma2 * dt2;
  }
  kalman->r = r;
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
void kalman_predict(struct Kalman *kalman)
{
  int i;
  for (i = 0; i < KALMAN_DIM; i += 2) {
    // kinematic equation of the dynamic model X = F*X
    kalman->state[i] += kalman->state[i+1] * kalman->dt;
  // kalman->state = matrix_product(kalman->F, kalman->state, KALMAN_DIM, KALMAN_DIM, 1);
    
    // propagate covariance P = F*P*Ft + Q
    // since F is diagonal by block, P can be updated by block here as well
    // let's unroll the matrix operations as it is simple
  // kalman->P = matrix_product(kalman->F, matrix_product);
    
    const float d_dt = kalman->P[i+1][i+1] * kalman->dt;
    kalman->P[i][i] += kalman->P[i+1][i] * kalman->dt
      + kalman->dt * (kalman->P[i][i+1] + d_dt) + kalman->Q[i][i];
    kalman->P[i][i+1] += d_dt + kalman->Q[i][i+1];
    kalman->P[i+1][i] += d_dt + kalman->Q[i+1][i];
    kalman->P[i+1][i+1] += kalman->Q[i+1][i+1];
  }
}

/** correction step
 *
 * K = PHt(HPHt+R)^-1 = PHtS^-1
 * X = X + K(Z-HX)
 * P = (I-KH)P
 */
void kalman_update(struct Kalman *kalman, struct FloatVect3 anchor)
{
  //const float dx = kalman->state[0] - anchor.x;
  //const float dy = kalman->state[2] - anchor.y;
  //const float dz = kalman->state[4] - anchor.z;
  //const float norm = sqrtf(dx * dx + dy * dy + dz * dz);
  // build measurement error
  //const float res = dist - norm;
  // build Jacobian of observation model for anchor i
  // float Hi[] = { dx / norm, 0.f, dy / norm, 0.f, dz / norm, 0.f };
  
  // compute kalman gain K = P*Ht (H*P*Ht + R)^-1
  // S = H*P*Ht + R
  
  // const float S =
  //   Hi[0] * Hi[0] * kalman->P[0][0] +
  //   Hi[2] * Hi[2] * kalman->P[2][2] +
  //   Hi[4] * Hi[4] * kalman->P[4][4] +
  //   Hi[0] * Hi[2] * (kalman->P[0][2] + kalman->P[2][0]) +
  //   Hi[0] * Hi[4] * (kalman->P[0][4] + kalman->P[4][0]) +
  //   Hi[2] * Hi[4] * (kalman->P[2][4] + kalman->P[4][2]) +
  //   kalman->r;

  const float S[3][3] = { {p[0][0] + Kalman->r; p[0][2]; p[0][4]};
                            {p[2][0]; p[2][2] + Kalman->r ; p[2][4]};
                            {p[4][0]; p[4][2]; p[4][4] + Kalman->r}};


  if (fabsf(S[0][0]) + fabsf(S[1][1]) + fabsf(S[2][2]) < 1e-5) {
    return; // don't inverse S if it is too small
  }
  // finally compute gain and correct state
  // float K[6];

  float invS[3][3];
  MAT_INV33(invS, S);

  float K[6][3];
  float HinvS_tmp[6][3];
  MAT_MUL(6, 3, 3, HinvS_tmp, matrix_transpose(kalman->H) , invS);
  MAT_MUL(6, 6, 3, K, P, HinvS_tmp);

  float X[6][1] = {kalman->state}; //K(Z-HX)
  float Z[3][1] = {anchor};
  float HX_tmp[3][1];
  MAT_MUL(3, 6, 1, HX_tmp, H, X);
  float Z_HX[3][1];
  MAT_SUB(3,1, Z_HX ,Z, HX_tmp);
  float K_ZHX_tmp[6][1];
  MAT_MUL(6,3,1, K_ZHX_tmp, K, Z_HX);
  MAT_SUM(6,1, X, X, K_ZHX_tmp);

  for (int i = 0; i<6; i++){
    kalman->state[i]= X[i];
  }

  // for (int i = 0; i < 6; i++) {
  //   K[i] = (Hi[0] * kalman->P[i][0] + Hi[2] * kalman->P[i][2] + Hi[4] * kalman->P[i][4]) / S;
  //   kalman->state[i] += K[i] * res;
  // }

  // precompute K*H and store current P
  float KH_tmp[6][6];
  float P_tmp[6][6];
  MAT_MUL(6,3,6, KH_tmp, K, kalman->H);
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      // KH_tmp[i][j] = K[i] * Hi[j];
      P_tmp[i][j] = kalman->P[i][j];
    }
  }
  // correct covariance P = (I-K*H)*P = P - K*H*P
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      for (int k = 0; k < 6; k++) {
        kalman->P[i][j] -= KH_tmp[i][k] * P_tmp[k][j];
      }
    }
  }
}

void kalman_update_speed(struct Kalman *kalman, float speed, uint8_t type)
{
  (void) kalman;
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
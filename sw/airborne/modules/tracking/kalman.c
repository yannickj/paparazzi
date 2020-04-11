#include "modules/tracking/kalman.h"
#include <math.h>
#include <stdio.h>

// use WP_PRED by default
#include "generated/flight_plan.h"
#if !(defined TAG_PRED_SIM_WP) && (defined WP_PRED)
#define TAG_PRED_SIM_WP WP_PRED
#endif

#define PRINTF printf
//#define PRINTF(...) {}

void kalman_init(struct Kalman *kalman, float P0_pos, float P0_speed, float Q_sigma2, float r, float dt)
{
  int i, j;
  const float dt2 = dt * dt;
  const float dt3 = dt2 * dt / 2.f;
  const float dt4 = dt2 * dt2 / 4.f;
  for (i = 0; i < KALMAN_DIM; i++)
  {
    kalman->state[i] = 0.f; // don't forget to call set_state before running the filter for better results
    for (j = 0; j < KALMAN_DIM; j++)
    {
      kalman->P[i][j] = 0.f;
      kalman->Q[i][j] = 0.f;
    }
  }
  for (i = 0; i < KALMAN_DIM; i += 2)
  {
    kalman->P[i][i] = P0_pos;
    kalman->P[i + 1][i + 1] = P0_speed;
    kalman->Q[i][i] = Q_sigma2 * dt4;
    kalman->Q[i + 1][i] = Q_sigma2 * dt3;
    kalman->Q[i][i + 1] = Q_sigma2 * dt3;
    kalman->Q[i + 1][i + 1] = Q_sigma2 * dt2;
  }
  kalman->r = r;
  kalman->dt = dt;

  // kalman->F = {{1, dt, 0, 0 , 0, 0},
  //                {0, 1 , 0, 0,  0, 0},
  //                {0, 0,  1, dt 0, 0},
  //                {0, 0,  0, 1  0, 0},
  //                {0, 0,  0, 0,  1 ,dt}
  //                {0, 0,  0, 0,  0, 1 }};

  for (int i = 0; i < KALMAN_DIM; i++)
  {
    for (int j = 0; j < KALMAN_DIM; j++)
    {
      if (i == j)
      {
        kalman->F[i][i] = 1;
      }
      else if (i % 2 == 0 && j == i + 1)
      {
        kalman->F[i][j] = dt;
      }
      else
      {
        kalman->F[i][j] = 0;
      }
    }
  }

  // kalman->H = {{1,0,0,0,0,0},
  //                 {0,0,1,0,0,0},
  //                 {0,0,0,0,1,0}};

  for (int i = 0; i < KALMAN_DIM / 2; i++)
  {
    for (int j = 0; j < KALMAN_DIM; j++)
    {
      if (2 * i == j)
      {
        kalman->H[i][j] = 1;
      }
      else
      {
        kalman->H[i][j] = 0;
      }
    }
  }
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
  for (i = 0; i < KALMAN_DIM; i += 2)
  {
    kalman->Q[i][i] = Q_sigma2 * dt4;
    kalman->Q[i + 1][i] = Q_sigma2 * dt3;
    kalman->Q[i][i + 1] = Q_sigma2 * dt3;
    kalman->Q[i + 1][i + 1] = Q_sigma2 * dt2;
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
void kalman_predict(struct Kalman *kalman, float *tag_tracking_roll, float *tag_tracking_pitch, float *tag_tracking_climb)
{
  int i;
  for (i = 0; i < KALMAN_DIM; i += 2)
  {
    // kinematic equation of the dynamic model X = F*X
    kalman->state[i] += kalman->state[i + 1] * kalman->dt;

    // propagate covariance P = F*P*Ft + Q
    // since F is diagonal by block, P can be updated by block here as well
    // let's unroll the matrix operations as it is simple

    const float d_dt = kalman->P[i + 1][i + 1] * kalman->dt;
    kalman->P[i][i] += kalman->P[i + 1][i] * kalman->dt + kalman->dt * (kalman->P[i][i + 1] + d_dt) + kalman->Q[i][i];
    kalman->P[i][i + 1] += d_dt + kalman->Q[i][i + 1];
    kalman->P[i + 1][i] += d_dt + kalman->Q[i + 1][i];
    kalman->P[i + 1][i + 1] += kalman->Q[i + 1][i + 1];
  }

  struct EnuCoor_f pos;

  pos.x = kalman->state[0];
  pos.y = kalman->state[2];
  pos.z = kalman->state[4];

  // float k = 4000;
  // *tag_tracking_roll = pos.y / k;
  // *tag_tracking_pitch = -pos.x / k;
  *tag_tracking_climb = 0;
  /**tag_tracking_roll = 5;
  *tag_tracking_pitch = 0;*/

  if (pos.x > 200)
  {
    *tag_tracking_pitch = 0.25;
  }
  else if (pos.x < -200)
  {
    *tag_tracking_pitch = 0.25;
  }
  else
  {
    *tag_tracking_pitch = 0;
  }

  if (pos.y > 200)
  {
    *tag_tracking_roll = 0.25;
  }
  else if (pos.y < -200)
  {
    *tag_tracking_roll = -0.25;
  }
  else
  {
    *tag_tracking_roll = 0;
  }

  PRINTF("pos.x : %f\n", pos.x);
  PRINTF("pos.y : %f\n", pos.y);
  PRINTF("pos.z : %f\n", pos.z);

  //PRINTF("roll : %f\n", *tag_tracking_roll);
  // PRINTF("pitch : %f\n", *tag_tracking_pitch);
  //PRINTF("vz : %f\n", *tag_tracking_climb);

  struct EnuCoor_i pos_i;
  ENU_BFP_OF_REAL(pos_i, pos);
  //PRINTF("%d %d %d\n", pos_i.x, pos_i.y, pos_i.z);
  fflush(stdout);
  waypoint_move_enu_i(TAG_PRED_SIM_WP, &pos_i);
}

/** correction step
 *
 * K = PHt(HPHt+R)^-1 = PHtS^-1
 * X = X + K(Z-HX)
 * P = (I-KH)P
 */
void kalman_update(struct Kalman *kalman, struct FloatVect3 anchor)
{
  PRINTF("anchor.x : %f\n", anchor.x);
  PRINTF("anchor.y : %f\n", anchor.y);
  PRINTF("anchor.z : %f\n", anchor.z);
  fflush(stdout);

  const float S[3][3] = {{kalman->P[0][0] + kalman->r, kalman->P[0][2], kalman->P[0][4]},
                         {kalman->P[2][0], kalman->P[2][2] + kalman->r, kalman->P[2][4]},
                         {kalman->P[4][0], kalman->P[4][2], kalman->P[4][4] + kalman->r}};

  if (fabsf(S[0][0]) + fabsf(S[1][1]) + fabsf(S[2][2]) < 1e-5)
  {
    //PRINTF("RETURN\n");
    //fflush(stdout);
    return; // don't inverse S if it is too small
  }
  //PRINTF("PAS RETURN\n");
  //fflush(stdout);
  // finally compute gain and correct state
  float invS[3][3];
  MAKE_MATRIX_PTR(_S, S, 3);
  MAKE_MATRIX_PTR(_invS, invS, 3);
  float_mat_invert(_invS, _S, 3);
  float HinvS_tmp[6][3];
  MAKE_MATRIX_PTR(_H, kalman->H, 3);

  /*
  for (int i = 0; i < 3; i++)
  {
    PRINTF("S : %f %f %f\n", S[i][0], S[i][1], S[i][2]);

  }
  fflush(stdout);
  */
  /*  for (int i = 0; i < 3; i++)
  {
    PRINTF("H : %f %f %f %f %f %f\n", kalman->H[i][0], kalman->H[i][1], kalman->H[i][2], kalman->H[i][3], kalman->H[i][4], kalman->H[i][5]);
    fflush(stdout);
  }*/
  float Ht[6][3];
  MAKE_MATRIX_PTR(_Ht, Ht, 6);

  float_mat_transpose(_Ht, _H, 3, 6);
  MAKE_MATRIX_PTR(_HinvS_tmp, HinvS_tmp, 6);

  float_mat_mul(_HinvS_tmp, _Ht, _invS, 6, 3, 3);
  float K[6][3];
  MAKE_MATRIX_PTR(_K, K, 6);

  MAKE_MATRIX_PTR(_P, kalman->P, 6);
  float_mat_mul(_K, _P, _HinvS_tmp, 6, 6, 3);
  // PRINTF("K : %f %f %f %f %f %f\n %f %f %f %f %f %f\n %f %f %f %f %f %f\n ", _K[0][0], _K[0][1], _K[0][2], _K[0][3], _K[0][4], _K[0][5], _K[1][0], _K[1][1], _K[1][2], _K[1][3], _K[1][4], _K[1][5], _K[2][0], _K[2][1], _K[2][2], _K[2][3], _K[2][4], _K[2][5]);
  /*fflush(stdout);
  for (int i = 0; i < 6; i++)
  {
    PRINTF("P : %f %f %f %f %f %f\n", _P[i][0], _P[i][1], _P[i][2], _P[i][3], _P[i][4], _P[i][5]);
    fflush(stdout);}
*/

  float HX_tmp[3];
  float_mat_vect_mul(HX_tmp, _H, kalman->state, 3, 6);
  float Z_HX[3];

  float Z[3] = {anchor.x, anchor.y, anchor.z};
  // PRINTF("Z : %f, %f, %f\n", Z[0], Z[1], Z[2]);
  float_vect_diff(Z_HX, Z, HX_tmp, 3);
  //PRINTF(" : %f, %f, %f\n", Z_HX[0], Z_HX[1], Z_HX[2]);

  float K_ZHX_tmp[6];
  float_mat_vect_mul(K_ZHX_tmp, _K, Z_HX, 6, 3);
  float_vect_add(kalman->state, K_ZHX_tmp, 6);
  //PRINTF("KZHX : %f %f %f %f %f %f \n", K_ZHX_tmp[0], K_ZHX_tmp[1], K_ZHX_tmp[2], K_ZHX_tmp[3], K_ZHX_tmp[4], K_ZHX_tmp[5]);
  fflush(stdout);
  //PRINTF("state : %f, %f, %f\n", kalman->state[0], kalman->state[2], kalman->state[4]);

  // precompute K*H and store current P
  float KH_tmp[6][6];
  MAKE_MATRIX_PTR(_KH_tmp, KH_tmp, 6);
  float P_tmp[6][6];
  float_mat_mul(_KH_tmp, _K, _H, 6, 3, 6);
  for (int i = 0; i < 6; i++)
  {
    for (int j = 0; j < 6; j++)
    {
      P_tmp[i][j] = kalman->P[i][j];
    }
  }
  // correct covariance P = (I-K*H)*P = P - K*H*P
  for (int i = 0; i < 6; i++)
  {
    for (int j = 0; j < 6; j++)
    {
      for (int k = 0; k < 6; k++)
      {
        kalman->P[i][j] -= KH_tmp[i][k] * P_tmp[k][j];
      }
    }
  }

  PRINTF("VU!\n");
  fflush(stdout);
}

void kalman_update_speed(struct Kalman *kalman, float speed, uint8_t type)
{
  (void)kalman;
  (void)speed;
  (void)type;
  // TODO
}

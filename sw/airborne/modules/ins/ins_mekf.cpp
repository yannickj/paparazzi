/*
 * Copyright (C) 2017 Marton Brossard <martin.brossard@mines-paristech.fr>
 *                    Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *
 * This file is part of paparazzi.
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
 * @file modules/ins/ins_mekf.cpp
 *
 * Multiplicative Extended Kalman Filter in rotation matrix formulation.
 *
 * Estimate attitude, ground speed, position, gyro bias, accelerometer bias.
 *
 * Using Eigen library
 */


#include "modules/ins/ins_mekf.h"
#include "generated/airframe.h"

#ifndef SITL
// Redifine Eigen assert so it doesn't use memory allocation
#define eigen_assert(_cond) { if (!(_cond)) { while(1) ; } }
#endif

// Eigen headers
#pragma GCC diagnostic ignored "-Wint-in-bool-context"
#pragma GCC diagnostic ignored "-Wshadow"
#include <Eigen/Dense>
#pragma GCC diagnostic pop

using namespace Eigen;

/** Covariance matrix elements and size
 */
enum MekfCovVar {
  MEKF_qx, MEKF_qy, MEKF_qz,
  MEKF_vx, MEKF_vy, MEKF_vz,
  MEKF_px, MEKF_py, MEKF_pz,
  MEKF_rbp, MEKF_rbq, MEKF_rbr,
  MEKF_abx, MEKF_aby, MEKF_abz,
  MEKF_bb,
  MEKF_COV_SIZE
};

typedef Matrix<float, MEKF_COV_SIZE, MEKF_COV_SIZE> MEKFCov;

/** Process noise elements and size
 */
enum MekfPNoiseVar {
  MEKF_qgp, MEKF_qgq, MEKF_qgr,
  MEKF_qax, MEKF_qay, MEKF_qaz,
  MEKF_qrbp, MEKF_qrbq, MEKF_qrbr,
  MEKF_qabx, MEKF_qaby, MEKF_qabz,
  MEKF_qbb,
  MEKF_PROC_NOISE_SIZE
};

typedef Matrix<float, MEKF_PROC_NOISE_SIZE, MEKF_PROC_NOISE_SIZE> MEKFPNoise;

/** Measurement noise elements and size
 */
enum MekfMNoiseVar {
  MEKF_rvx, MEKF_rvy, MEKF_rvz,
  MEKF_rpx, MEKF_rpy, MEKF_rpz,
  MEKF_rmx, MEKF_rmy, MEKF_rmz,
  MEKF_rb,
  MEKF_MEAS_NOISE_SIZE
};

typedef Matrix<float, MEKF_MEAS_NOISE_SIZE, MEKF_MEAS_NOISE_SIZE> MEKFMNoise;

/** filter state vector
 */
struct MekfState {
	Quaternionf quat;
	Vector3f speed;
	Vector3f pos;
	Vector3f accel;
	Vector3f rates_bias;
	Vector3f accel_bias;
  float baro_bias;
};

/** filter command vector
 */
struct MekfInputs {
	Vector3f rates;
	Vector3f accel;
};

/** filter measurement vector
 */
struct MekfMeasurements {
	Vector3f speed;
	Vector3f pos;
	Vector3f mag;
	float baro_alt;
};

/** private filter structure
 */
struct InsMekfPrivate {
  struct MekfState state;
  struct MekfInputs inputs;
  struct MekfMeasurements measurements;

  MEKFCov P;
  MEKFPNoise Q;
  MEKFMNoise R;

  /* earth magnetic model */
  Vector3f mag_h;
};


// Initial covariance parameters
#ifndef INS_MEKF_P0_QUAT
#define INS_MEKF_P0_QUAT       0.007615f
#endif
#ifndef INS_MEKF_P0_SPEED
#define INS_MEKF_P0_SPEED      1.E+2f
#endif
#ifndef INS_MEKF_P0_POS
#define INS_MEKF_P0_POS        1.E+1f
#endif
#ifndef INS_MEKF_P0_RATES_BIAS
#define INS_MEKF_P0_RATES_BIAS 1.E-3f
#endif
#ifndef INS_MEKF_P0_ACCEL_BIAS
#define INS_MEKF_P0_ACCEL_BIAS 1.E-3f
#endif
#ifndef INS_MEKF_P0_BARO_BIAS
#define INS_MEKF_P0_BARO_BIAS  1.E-3f
#endif

// Initial process noise parameters
#ifndef INS_MEKF_Q_GYRO
#define INS_MEKF_Q_GYRO        1.E-2f
#endif
#ifndef INS_MEKF_Q_ACCEL
#define INS_MEKF_Q_ACCEL       1.E-2f
#endif
#ifndef INS_MEKF_Q_RATES_BIAS
#define INS_MEKF_Q_RATES_BIAS  1.E-6f
#endif
#ifndef INS_MEKF_Q_ACCEL_BIAS
#define INS_MEKF_Q_ACCEL_BIAS  1.E-6f
#endif
#ifndef INS_MEKF_Q_BARO_BIAS
#define INS_MEKF_Q_BARO_BIAS   1.E-3f
#endif

// Initial measurements noise parameters
#ifndef INS_MEKF_R_SPEED
#define INS_MEKF_R_SPEED       0.1f
#endif
#ifndef INS_MEKF_R_SPEED_Z
#define INS_MEKF_R_SPEED_Z     0.2f
#endif
#ifndef INS_MEKF_R_POS
#define INS_MEKF_R_POS         2.0f
#endif
#ifndef INS_MEKF_R_POS_Z
#define INS_MEKF_R_POS_Z       4.0f
#endif
#ifndef INS_MEKF_R_MAG
#define INS_MEKF_R_MAG         1.f
#endif
#ifndef INS_MEKF_R_BARO
#define INS_MEKF_R_BARO        2.f
#endif

// paramters
struct ins_mekf_parameters ins_mekf_params;

// internal structure
static struct InsMekfPrivate mekf_private;
// short name
#define mp mekf_private

/* earth gravity model */
static const Vector3f gravity( 0.f, 0.f, 9.81f );


/* init state and measurements */
static void init_mekf_state(void)
{
  // init state
  mekf_private.state.quat = Quaternionf::Identity();
  mekf_private.state.speed = Vector3f::Zero();
  mekf_private.state.pos = Vector3f::Zero();
  mekf_private.state.rates_bias = Vector3f::Zero();
  mekf_private.state.accel_bias = Vector3f::Zero();
  mekf_private.state.baro_bias = 0.f;

  // init measures
  mekf_private.measurements.speed = Vector3f::Zero();
  mekf_private.measurements.pos = Vector3f::Zero();
  mekf_private.measurements.mag = Vector3f::Zero();
  mekf_private.measurements.baro_alt = 0.f;

  // init input
  mekf_private.inputs.rates = Vector3f::Zero();
  mekf_private.inputs.accel = Vector3f::Zero();

  // init state covariance
  mekf_private.P = MEKFCov::Zero();
  mekf_private.P(MEKF_qx,MEKF_qx) = INS_MEKF_P0_QUAT;
  mekf_private.P(MEKF_qy,MEKF_qy) = INS_MEKF_P0_QUAT;
  mekf_private.P(MEKF_qz,MEKF_qz) = INS_MEKF_P0_QUAT;
  mekf_private.P(MEKF_vx,MEKF_vx) = INS_MEKF_P0_SPEED;
  mekf_private.P(MEKF_vy,MEKF_vy) = INS_MEKF_P0_SPEED;
  mekf_private.P(MEKF_vz,MEKF_vz) = INS_MEKF_P0_SPEED;
  mekf_private.P(MEKF_px,MEKF_px) = INS_MEKF_P0_POS;
  mekf_private.P(MEKF_py,MEKF_py) = INS_MEKF_P0_POS;
  mekf_private.P(MEKF_pz,MEKF_pz) = INS_MEKF_P0_POS;
  mekf_private.P(MEKF_rbp,MEKF_rbp) = INS_MEKF_P0_RATES_BIAS;
  mekf_private.P(MEKF_rbq,MEKF_rbq) = INS_MEKF_P0_RATES_BIAS;
  mekf_private.P(MEKF_rbr,MEKF_rbr) = INS_MEKF_P0_RATES_BIAS;
  mekf_private.P(MEKF_abx,MEKF_abx) = INS_MEKF_P0_ACCEL_BIAS;
  mekf_private.P(MEKF_aby,MEKF_aby) = INS_MEKF_P0_ACCEL_BIAS;
  mekf_private.P(MEKF_abz,MEKF_abz) = INS_MEKF_P0_ACCEL_BIAS;
  mekf_private.P(MEKF_bb,MEKF_bb) = INS_MEKF_P0_BARO_BIAS;

  // init process and measurements noise
  ins_mekf_update_params();
}

// Some matrix and quaternion utility functions
static Quaternionf quat_add(const Quaternionf& q1, const Quaternionf& q2) {
  return Quaternionf(q1.w() + q2.w(), q1.x() + q2.x(), q1.y() + q2.y(), q1.z() + q2.z());
}

static Quaternionf quat_smul(const Quaternionf& q1, float scal) {
  return Quaternionf(q1.w() * scal, q1.x() * scal, q1.y() * scal, q1.z() * scal);
}

/**
 * build skew symetric matrix from vector
 * m = [     0, -v(2),  v(1) ]
 *     [  v(2),     0, -v(0) ]
 *     [ -v(1),  v(0),     0 ]
 */
static Matrix3f skew_sym(const Vector3f& v) {
  Matrix3f m = Matrix3f::Zero();
  m(0,1) = -v(2);
  m(0,2) = v(1);
  m(1,0) = v(2);
  m(1,2) = -v(0);
  m(2,0) = -v(1);
  m(2,1) = v(0);
  return m;
}

/**
 * Init function
 */
void ins_mekf_init(void)
{
  // init parameters
  ins_mekf_params.Q_gyro       = INS_MEKF_Q_GYRO;
  ins_mekf_params.Q_accel      = INS_MEKF_Q_ACCEL;
  ins_mekf_params.Q_rates_bias = INS_MEKF_Q_RATES_BIAS;
  ins_mekf_params.Q_accel_bias = INS_MEKF_Q_ACCEL_BIAS;
  ins_mekf_params.Q_baro_bias  = INS_MEKF_Q_BARO_BIAS;
  ins_mekf_params.R_speed      = INS_MEKF_R_SPEED;
  ins_mekf_params.R_speed_z    = INS_MEKF_R_SPEED_Z;
  ins_mekf_params.R_pos        = INS_MEKF_R_POS;
  ins_mekf_params.R_pos_z      = INS_MEKF_R_POS_Z;
  ins_mekf_params.R_mag        = INS_MEKF_R_MAG;
  ins_mekf_params.R_baro       = INS_MEKF_R_BARO;

  // init state and measurements
  init_mekf_state();

  // init local earth magnetic field
  mekf_private.mag_h = Vector3f(1.0f, 0.f, 0.f);
}

void ins_mekf_set_mag_h(const struct FloatVect3 *mag_h)
{
  // update local earth magnetic field
  mekf_private.mag_h(0) = mag_h->x;
  mekf_private.mag_h(1) = mag_h->y;
  mekf_private.mag_h(2) = mag_h->z;
}

void ins_mekf_reset(void)
{
  init_mekf_state();
}

/** Full INS propagation
 */
void ins_mekf_propagate(struct FloatRates *gyro, struct FloatVect3 *acc, float dt)
{
  Quaternionf q_tmp;

  mekf_private.inputs.rates = Vector3f(gyro->p, gyro->q, gyro->r);
  mekf_private.inputs.accel = Vector3f(acc->x, acc->y, acc->z);

  const Vector3f gyro_unbiased = mp.inputs.rates - mp.state.rates_bias;
  const Vector3f accel_unbiased = mp.inputs.accel - mp.state.accel_bias;
  // propagate state
  // q_dot = 1/2 q * (rates - rates_bias)
  q_tmp.w() = 0.f;
  q_tmp.vec() = gyro_unbiased;
  const Quaternionf q_d = quat_smul(mp.state.quat * q_tmp, 0.5f);
  // speed_d = q * (accel - accel_bias) * q^-1 + g
  q_tmp.vec() = accel_unbiased;
  // store NED accel
  mp.state.accel = (mp.state.quat * q_tmp * mp.state.quat.inverse()).vec() + gravity;

  // Euler integration

  //mp.state.quat = (mp.state.quat + q_d * dt).normalize();
  mp.state.quat = quat_add(mp.state.quat, quat_smul(q_d, dt));
  mp.state.quat.normalize();
  mp.state.speed = mp.state.speed + mp.state.accel * dt;
  mp.state.pos = mp.state.pos + mp.state.speed * dt;

  // propagate covariance
  const Matrix3f Rq = mp.state.quat.toRotationMatrix();
  const Matrix3f Rqdt = Rq * dt;
  const Matrix3f RqA = skew_sym(Rq * accel_unbiased);
  const Matrix3f RqAdt = RqA * dt;
  const Matrix3f RqAdt2 = RqAdt * dt;

  MEKFCov A = MEKFCov::Identity();
  A.block<3,3>(MEKF_qx,MEKF_rbp) = -Rqdt;
  A.block<3,3>(MEKF_vx,MEKF_qx) = -RqAdt;
  A.block<3,3>(MEKF_vx,MEKF_rbp) = RqAdt2;
  A.block<3,3>(MEKF_vx,MEKF_abx) = -Rqdt;
  A.block<3,3>(MEKF_px,MEKF_qx) = -RqAdt2;
  A.block<3,3>(MEKF_px,MEKF_vx) = Matrix3f::Identity() * dt;
  A.block<3,3>(MEKF_px,MEKF_rbp) = RqAdt2 * dt;
  A.block<3,3>(MEKF_px,MEKF_abx) = -Rqdt * dt;

  Matrix<float, MEKF_COV_SIZE, MEKF_PROC_NOISE_SIZE> An;
  An.setZero();
  An.block<3,3>(MEKF_qx,MEKF_qgp) = Rq;
  An.block<3,3>(MEKF_vx,MEKF_qax) = Rq;
  An.block<3,3>(MEKF_rbp,MEKF_qrbp) = Matrix3f::Identity();
  An.block<3,3>(MEKF_abx,MEKF_qabx) = Matrix3f::Identity();
  An(MEKF_bb,MEKF_qbb) = 1.0f;

  MEKFCov At(A);
  At.transposeInPlace();
  Matrix<float, MEKF_PROC_NOISE_SIZE, MEKF_COV_SIZE> Ant;
  Ant = An.transpose();

  mp.P = A * mp.P * At + An * mp.Q * Ant * dt;
}

/** AHRS-only propagation + accel correction
 */
void ins_mekf_propagate_ahrs(struct FloatRates *gyro, struct FloatVect3 *acc, float dt)
{
  Quaternionf q_tmp;

  mekf_private.inputs.rates = Vector3f(gyro->p, gyro->q, gyro->r);
  mekf_private.inputs.accel = Vector3f(acc->x, acc->y, acc->z);

  const Vector3f gyro_unbiased = mp.inputs.rates - mp.state.rates_bias;
  const Vector3f accel_unbiased = mp.inputs.accel - mp.state.accel_bias;
  // propagate state
  // q_dot = 1/2 q * (rates - rates_bias)
  q_tmp.w() = 0.f;
  q_tmp.vec() = gyro_unbiased;
  const Quaternionf q_d = quat_smul(mp.state.quat * q_tmp, 0.5f);

  // Euler integration

  //mp.state.quat = (mp.state.quat + q_d * dt).normalize();
  mp.state.quat = quat_add(mp.state.quat, quat_smul(q_d, dt));
  mp.state.quat.normalize();

  // propagate covariance
  const Matrix3f Rq = mp.state.quat.toRotationMatrix();
  const Matrix3f Rqdt = Rq * dt;

  MEKFCov A = MEKFCov::Zero();
  A.block<3,3>(MEKF_qx,MEKF_qx) = Matrix3f::Identity();
  A.block<3,3>(MEKF_qx,MEKF_rbp) = -Rqdt;
  A.block<3,3>(MEKF_rbp,MEKF_rbp) = Matrix3f::Identity();

  Matrix<float, MEKF_COV_SIZE, MEKF_PROC_NOISE_SIZE> An;
  An.setZero();
  An.block<3,3>(MEKF_qx,MEKF_qgp) = Rq;
  An.block<3,3>(MEKF_rbp,MEKF_qrbp) = Matrix3f::Identity(); // TODO check index

  MEKFCov At(A);
  At.transposeInPlace();
  Matrix<float, MEKF_PROC_NOISE_SIZE, MEKF_COV_SIZE> Ant;
  Ant = An.transpose();

  mp.P = A * mp.P * At + An * mp.Q * Ant * dt;

  // correction from accel measurements
  const Matrix3f Rqt = Rq.transpose();
  Matrix<float, 3, MEKF_COV_SIZE> H = Matrix<float, 3, MEKF_COV_SIZE>::Zero();
  H.block<3,3>(0,0) = - Rqt * skew_sym(gravity);
  Matrix<float, MEKF_COV_SIZE, 3> Ht = H.transpose();
  // S = H*P*Ht + Hn*N*Hnt
  Matrix3f S = H * mp.P * Ht + mp.R.block<3,3>(MEKF_rmx,MEKF_rmx); // FIXME currently abusing mag noise ????
  // K = P*Ht*S^-1
  Matrix<float, MEKF_COV_SIZE, 3> K = mp.P * Ht * S.inverse();
  // Residual z_a - h(z)
  Vector3f res = accel_unbiased + (Rqt * gravity);
  // Update state
  q_tmp.w() = 1.f;
  q_tmp.vec() = 0.5f * K.block<3,3>(MEKF_qx,0) * res;
  q_tmp.normalize();
  mp.state.quat = q_tmp * mp.state.quat;
  mp.state.quat.normalize();
  mp.state.rates_bias  += K.block<3,3>(MEKF_rbp,0) * res;
  // Update covariance
  mp.P = (MEKFCov::Identity() - K * H) * mp.P;
}


void ins_mekf_align(struct FloatRates *gyro_bias, struct FloatQuat *quat)
{
  /* Compute an initial orientation from accel and mag directly as quaternion */
  mp.state.quat.w() = quat->qi;
  mp.state.quat.x() = quat->qx;
  mp.state.quat.y() = quat->qy;
  mp.state.quat.z() = quat->qz;

  /* use average gyro as initial value for bias */
  mp.state.rates_bias(0) = gyro_bias->p;
  mp.state.rates_bias(1) = gyro_bias->q;
  mp.state.rates_bias(2) = gyro_bias->r;
}

void ins_mekf_update_mag(struct FloatVect3* mag, bool attitude_only)
{
  mp.measurements.mag(0) = mag->x;
  mp.measurements.mag(1) = mag->y;
  mp.measurements.mag(2) = mag->z;

  // H and Ht matrices
  const Matrix3f Rqt = mp.state.quat.toRotationMatrix().transpose();
  Matrix<float, 3, MEKF_COV_SIZE> H = Matrix<float, 3, MEKF_COV_SIZE>::Zero();
  H.block<3,3>(0,0) = Rqt * skew_sym(mp.mag_h);
  Matrix<float, MEKF_COV_SIZE, 3> Ht = H.transpose();
  // S = H*P*Ht + Hn*N*Hnt
  Matrix3f S = H * mp.P * Ht + mp.R.block<3,3>(MEKF_rmx,MEKF_rmx);
  // K = P*Ht*S^-1
  Matrix<float, MEKF_COV_SIZE, 3> K = mp.P * Ht * S.inverse();
  // Residual z_m - h(z)
  Vector3f res = mp.measurements.mag - (Rqt * mp.mag_h);
  // Update state
  Quaternionf q_tmp;
  q_tmp.w() = 1.f;
  q_tmp.vec() = 0.5f * K.block<3,3>(MEKF_qx,0) * res;
  q_tmp.normalize();
  mp.state.quat = q_tmp * mp.state.quat;
  mp.state.quat.normalize();
  if (attitude_only) {
    mp.state.rates_bias  += K.block<3,3>(MEKF_rbp,0) * res;
  } else {
    mp.state.speed       += K.block<3,3>(MEKF_vx,0) * res;
    mp.state.pos         += K.block<3,3>(MEKF_px,0) * res;
    mp.state.rates_bias  += K.block<3,3>(MEKF_rbp,0) * res;
    mp.state.accel_bias  += K.block<3,3>(MEKF_abx,0) * res;
    mp.state.baro_bias   += K.block<1,3>(MEKF_bb,0) * res;
  }
  // Update covariance
  mp.P = (MEKFCov::Identity() - K * H) * mp.P;
}

void ins_mekf_update_baro(float baro_alt)
{
  mp.measurements.baro_alt = baro_alt;

  // H and Ht matrices
  Matrix<float, 1, MEKF_COV_SIZE> H = Matrix<float, 1, MEKF_COV_SIZE>::Zero();
  H(0,MEKF_pz) = 1.0f; // TODO check index
  H(0,MEKF_bb) = -1.0f;
  Matrix<float, MEKF_COV_SIZE, 1> Ht = H.transpose();
  // S = H*P*Ht + Hn*N*Hnt -> only pos.z component
  float S = mp.P(MEKF_pz,MEKF_pz) - mp.P(MEKF_bb,MEKF_bb) + mp.R(MEKF_rb,MEKF_rb);
  // K = P*Ht*S^-1
  Matrix<float, MEKF_COV_SIZE, 1> K = mp.P * Ht / S;
  // Residual z_m - h(z)
  float res = mp.measurements.baro_alt - (mp.state.pos(2) - mp.state.baro_bias);
  // Update state
  Quaternionf q_tmp;
  q_tmp.w() = 1.f;
  q_tmp.vec() = 0.5f * K.block<3,1>(MEKF_qx,0) * res;
  q_tmp.normalize();
  mp.state.quat = q_tmp * mp.state.quat;
  mp.state.quat.normalize();
  mp.state.speed       += K.block<3,1>(MEKF_vx,0) * res;
  mp.state.pos         += K.block<3,1>(MEKF_px,0) * res;
  mp.state.rates_bias  += K.block<3,1>(MEKF_rbp,0) * res;
  mp.state.accel_bias  += K.block<3,1>(MEKF_abx,0) * res;
  mp.state.baro_bias   += K(MEKF_bb,0) * res;
  // Update covariance
  mp.P = (MEKFCov::Identity() - K * H) * mp.P;
}

void ins_mekf_update_pos_speed(struct FloatVect3 *pos, struct FloatVect3 *speed)
{
  mp.measurements.pos(0) = pos->x;
  mp.measurements.pos(1) = pos->y;
  mp.measurements.pos(2) = pos->z;
  mp.measurements.speed(0) = speed->x;
  mp.measurements.speed(1) = speed->y;
  mp.measurements.speed(2) = speed->z;

  // H and Ht matrices
  Matrix<float, 6, MEKF_COV_SIZE> H = Matrix<float, 6, MEKF_COV_SIZE>::Zero();
  H.block<6,6>(0,MEKF_vx) = Matrix<float,6,6>::Identity();
  Matrix<float, MEKF_COV_SIZE, 6> Ht = H.transpose();
  // S = H*P*Ht + Hn*N*Hnt
  Matrix<float, 6, 6> S = mp.P.block<6,6>(MEKF_vx,MEKF_vx) + mp.R.block<6,6>(MEKF_rvx,MEKF_rvx);
  // K = P*Ht*S^-1
  Matrix<float, MEKF_COV_SIZE, 6> K = mp.P * Ht * S.inverse();
  // Residual z_m - h(z)
  Matrix<float, 6, 1> res = Matrix<float, 6, 1>::Zero();
  res.block<3,1>(0,0) = mp.measurements.speed - mp.state.speed;
  res.block<3,1>(3,0) = mp.measurements.pos - mp.state.pos;
  // Update state
  Quaternionf q_tmp;
  q_tmp.w() = 1.f;
  q_tmp.vec() = 0.5f * K.block<3,6>(MEKF_qx,0) * res;
  q_tmp.normalize();
  mp.state.quat = q_tmp * mp.state.quat;
  mp.state.speed       += K.block<3,6>(MEKF_vx,0) * res;
  mp.state.pos         += K.block<3,6>(MEKF_px,0) * res;
  mp.state.rates_bias  += K.block<3,6>(MEKF_rbp,0) * res;
  mp.state.accel_bias  += K.block<3,6>(MEKF_abx,0) * res;
  mp.state.baro_bias   += K.block<1,6>(MEKF_bb,0) * res;
  // Update covariance
  mp.P = (MEKFCov::Identity() - K * H) * mp.P;
}

/**
 * Getter/Setter functions
 */
struct NedCoor_f ins_mekf_get_pos_ned(void)
{
  const struct NedCoor_f p = {
    .x = mp.state.pos(0),
    .y = mp.state.pos(1),
    .z = mp.state.pos(2)
  };
  return p;
}

void ins_mekf_set_pos_ned(struct NedCoor_f *p)
{
  mp.state.pos(0) = p->x;
  mp.state.pos(1) = p->y;
  mp.state.pos(2) = p->z;
}

struct NedCoor_f ins_mekf_get_speed_ned(void)
{
  const struct NedCoor_f s = {
    .x = mp.state.speed(0),
    .y = mp.state.speed(1),
    .z = mp.state.speed(2)
  };
  return s;
}

void ins_mekf_set_speed_ned(struct NedCoor_f *s)
{
  mp.state.speed(0) = s->x;
  mp.state.speed(1) = s->y;
  mp.state.speed(2) = s->z;
}

struct NedCoor_f ins_mekf_get_accel_ned(void)
{
  const struct NedCoor_f a = {
    .x = mp.state.accel(0),
    .y = mp.state.accel(1),
    .z = mp.state.accel(2)
  };
  return a;
}

struct FloatQuat ins_mekf_get_quat(void)
{
  const struct FloatQuat q = {
    .qi = mp.state.quat.w(),
    .qx = mp.state.quat.x(),
    .qy = mp.state.quat.y(),
    .qz = mp.state.quat.z()
  };
  return q;
}

void ins_mekf_set_quat(struct FloatQuat *quat)
{
  mp.state.quat.w() = quat->qi;
  mp.state.quat.x() = quat->qx;
  mp.state.quat.y() = quat->qy;
  mp.state.quat.z() = quat->qz;
}

struct FloatRates ins_mekf_get_body_rates(void)
{
  const struct FloatRates r = {
    .p = mp.inputs.rates(0) - mp.state.rates_bias(0),
    .q = mp.inputs.rates(1) - mp.state.rates_bias(1),
    .r = mp.inputs.rates(2) - mp.state.rates_bias(2)
  };
  return r;
}

struct FloatVect3 ins_mekf_get_accel_bias(void)
{
  const struct FloatVect3 ab = {
    .x = mp.state.accel_bias(0),
    .y = mp.state.accel_bias(1),
    .z = mp.state.accel_bias(2)
  };
  return ab;
}

struct FloatRates ins_mekf_get_rates_bias(void)
{
  const struct FloatRates rb = {
    .p = mp.state.rates_bias(0),
    .q = mp.state.rates_bias(1),
    .r = mp.state.rates_bias(2)
  };
  return rb;
}

float ins_mekf_get_baro_bias(void)
{
  return mp.state.baro_bias;
}

void ins_mekf_update_params(void)
{
  Matrix<float, MEKF_PROC_NOISE_SIZE, 1> vp;
  vp(MEKF_qgp) = vp(MEKF_qgq) = vp(MEKF_qgr) = ins_mekf_params.Q_gyro;
  vp(MEKF_qax) = vp(MEKF_qay) = vp(MEKF_qaz) = ins_mekf_params.Q_accel;
  vp(MEKF_qrbp) = vp(MEKF_qrbq) = vp(MEKF_qrbr) = ins_mekf_params.Q_rates_bias;
  vp(MEKF_qabx) = vp(MEKF_qaby) = vp(MEKF_qabz) = ins_mekf_params.Q_accel_bias;
  vp(MEKF_qbb) = ins_mekf_params.Q_baro_bias;
  mekf_private.Q = vp.asDiagonal();

  Matrix<float, MEKF_MEAS_NOISE_SIZE, 1> vm;
  vm(MEKF_rvx) = vm(MEKF_rvy) = ins_mekf_params.R_speed;
  vm(MEKF_rvz) = ins_mekf_params.R_speed_z;
  vm(MEKF_rpx) = vm(MEKF_rpy) = ins_mekf_params.R_pos;
  vm(MEKF_rpz) = ins_mekf_params.R_pos_z;
  vm(MEKF_rmx) = vm(MEKF_rmy) = vm(MEKF_rmz) = ins_mekf_params.R_mag;
  vm(MEKF_rb) = ins_mekf_params.R_baro;
  mekf_private.R = vm.asDiagonal();
}


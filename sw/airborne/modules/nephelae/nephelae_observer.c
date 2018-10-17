/*
 * Copyright (C) Cabarbaye/Verdu
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/nephelae/nephelae_observer.c"
 * @author Cabarbaye/Verdu
 * Observer of state vector based on airplane caracteristics
 */

#include "modules/nephelae/nephelae_observer.h"

#include <stdio.h>
#include <math.h>
#include "subsystems/abi.h"
#include "std.h"
#include "generated/airframe.h"
#include "subsystems/datalink/telemetry.h"

/** ABI binding for IMU acceleration
*/
#ifndef IMU_ACC_NEPH_ID
#define IMU_ACC_NEPH_ID ABI_BROADCAST
#endif
static abi_event acc_ev;

/** ABI binding for IMU gyrometer
 */
#ifndef IMU_GYRO_NEPH_ID
#define IMU_GYRO_NEPH_ID ABI_BROADCAST
#endif
static abi_event gyro_ev;

struct ObsVect neph_obs_state;

static struct ObsVect neph_obs_state_mem;

static struct FloatRates neph_gyro_f;
uint32_t neph_stamp_gyro = 0;
uint32_t neph_stamp_gyro_mem = 0;


static struct FloatVect3 neph_accel_f;
uint32_t neph_stamp_accel = 0;

float delta_f_motor = 0.f;
float delta_q_motor = 0.f;

float omega_obs;
float omega_zero;

bool processing = false;

static void accel_cb(uint8_t sender_id __attribute__((unused)),
                     uint32_t stamp __attribute__((unused)),
                     struct Int32Vect3 *accel){

	ACCELS_FLOAT_OF_BFP(neph_accel_f, *accel);
  neph_stamp_accel = stamp;
}

static void gyro_cb(uint8_t sender_id __attribute__((unused)),
                    uint32_t stamp __attribute__((unused)),
                    struct Int32Rates *gyro){

  neph_stamp_gyro_mem = neph_stamp_gyro;
	RATES_FLOAT_OF_BFP(neph_gyro_f, *gyro);
  neph_stamp_gyro = stamp;

  if ((fabs(neph_stamp_gyro - neph_stamp_accel) < fabs(neph_stamp_gyro_mem - neph_stamp_accel)) && processing == true){
    integration();
  }
}

void omega_process(void){
  float A = NEPH_R * NEPH_KV * NEPH_KQW;
  float B = (1 / NEPH_KV) + ( NEPH_R * NEPH_KV * NEPH_KQX * (neph_obs_state.airspeed.x + NEPH_V0));
  float C = (NEPH_R * NEPH_KV * NEPH_KQU) * powf((neph_obs_state.airspeed.x + NEPH_V0), 2) + (NEPH_R * NEPH_I0) - neph_ctrl_state.v_motor;
  float delta_obs = powf(B,2) - (4*A*C);
  omega_obs = 0.f;

  if (delta_obs > 0.0){
    omega_obs = (-B + sqrt(delta_obs))/(2*A);
  }
  else{
    omega_obs = -B /(2*A);
  }
}

void omega_zero_process(void){
  float A = NEPH_KFW;
  float B = NEPH_KFX * NEPH_V0;
  float C = -trim_thrust * NEPH_M;
  float delta_obs = powf(B,2) - (4*A*C);
  omega_zero = 0.f;

  if (delta_obs > 0.0){
    omega_zero = (-B + sqrt(delta_obs))/(2*A);
  }
  else{
    omega_zero = -B /(2*A);
  }
}

void motor_process(void){
  delta_f_motor = (((NEPH_KFW * powf(omega_obs,2)) + (NEPH_KFX * omega_obs * (neph_obs_state.airspeed.x + NEPH_V0))) / NEPH_M) - (trim_thrust);
  delta_q_motor = (((NEPH_KQW * powf(omega_obs,2)) + (NEPH_KQX * omega_obs * (neph_obs_state.airspeed.x + NEPH_V0)) +
                  (NEPH_KQU * powf((neph_obs_state.airspeed.x + NEPH_V0),2))) - 
                  ((NEPH_KQW * powf(omega_zero,2)) + (NEPH_KQX * omega_zero * NEPH_V0) +
                  (NEPH_KQU * powf((NEPH_V0),2)))) / NEPH_IXX;
}

void integration(void){
  processing = false;




  processing = true;
}

/** IMU Acc and Gyro Observer for state feedback control. Called at startup.
 *  Bind ABI messages
 */
void nephelae_observer_init(void) {

  neph_obs_state.airspeed.x = 0.0;
  neph_obs_state.airspeed.y = 0.0;
  neph_obs_state.airspeed.z = 0.0;

  neph_obs_state.rotspeed.p = 0.0;
  neph_obs_state.rotspeed.q = 0.0;
  neph_obs_state.rotspeed.r = 0.0;

  neph_obs_state.attitude.phi = 0.0;
  neph_obs_state.attitude.theta = 0.0;
  neph_obs_state.attitude.psi = 0.0;

  neph_obs_state_mem = neph_obs_state;

	AbiBindMsgIMU_ACCEL_INT32(IMU_ACC_NEPH_ID, &acc_ev, accel_cb);
	AbiBindMsgIMU_GYRO_INT32(IMU_GYRO_NEPH_ID, &gyro_ev, gyro_cb);
}


// neph_ctrl_state.aileron
// neph_ctrl_state.flap
// neph_ctrl_state.v_motor

// trim_elevator
// trim_aileron
// trim_thrust
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
#include "subsystems/abi.h"
#include "std.h"
#include "math/pprz_algebra_int.h"
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

/** ABI binding for Nephelae State
 */
#ifndef NEPH_CTRL_ID
#define NEPH_CTRL_ID ABI_BROADCAST
#endif
static abi_event neph_ev;

/**ABI publisher for Neph State
 */
#define SENDER_ID 24

uint32_t neph_stamp = 0;
int32_t nephelae_gyro_p = 0;
int32_t nephelae_gyro_q = 0;
int32_t nephelae_gyro_r = 0;


bool flag_acc = false;
bool flag_gyro = false;

uint8_t Ctrl = 0;
uint8_t Obs = 0;

uint8_t module = 0;

static void accel_cb(uint8_t sender_id __attribute__((unused)),
                     uint32_t stamp __attribute__((unused)),
                     struct Int32Vect3 *accel){
	flag_acc = true;
}

static void gyro_cb(uint8_t sender_id __attribute__((unused)),
                    uint32_t stamp __attribute__((unused)),
                    struct Int32Rates *gyro){

	flag_gyro = true;
	neph_stamp = stamp;
	nephelae_gyro_p = gyro->p;
	nephelae_gyro_q = gyro->q;
	nephelae_gyro_r = gyro->r;
}

static void send_debug(struct transport_tx *trans, struct link_device *dev)
{
  module ++;
  if(flag_gyro && flag_acc){
  	pprz_msg_send_NEPHELAE_TEST_DEBUG(trans, dev, AC_ID,&neph_stamp, &nephelae_gyro_p, &nephelae_gyro_q, &nephelae_gyro_r);
  	pprz_msg_send_PPRZ_DEBUG(trans, dev, AC_ID, &module, &Ctrl);
  	AbiSendMsgNEPHELAE_OBS(SENDER_ID, &Obs, &Ctrl);
  }
  flag_gyro = false;
  flag_acc = false;
}

static void neph_cb(uint8_t sender_id __attribute__((unused)),
					uint8_t NbObserver, uint8_t NbControler){
	Ctrl = NbControler;
	Obs = NbObserver;
	Obs++;
}

/** IMU Acc and Gyro Observer for state feedback control. Called at startup.
 *  Bind ABI messages
 */
void nephelae_observer_init(void) {

	AbiBindMsgIMU_ACCEL_INT32(IMU_ACC_NEPH_ID, &acc_ev, accel_cb);
	AbiBindMsgIMU_GYRO_INT32(IMU_GYRO_NEPH_ID, &gyro_ev, gyro_cb);
	AbiBindMsgNEPHELAE_CTRL(NEPH_CTRL_ID, &neph_ev, neph_cb);

	#if PERIODIC_TELEMETRY
  	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_NEPHELAE_TEST_DEBUG, send_debug);
  	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_PPRZ_DEBUG, send_debug);
	#endif
}



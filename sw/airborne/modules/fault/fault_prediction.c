/*
 * Copyright (C) 2012-2013 Jean-Philippe Condomines, Gautier Hattenberger
 *               2015 Felix Ruess <felix.ruess@gmail.com>
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
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file subsystems/ins/ins_float_invariant_wrapper.c
 *
 * Paparazzi specific wrapper to run SVM classification to label data
 */

#include "modules/fault/fault_prediction.h"
#include "modules/fault/predictFault/predictFault.h"
#include "subsystems/abi.h"

static cell_wrap_0 cell;

/** last accel measurement */
static struct FloatVect3 last_accel;

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"
//static void send_fault(struct transport_tx *trans, struct link_device *dev)
//{
//}
#endif


/*
 * ABI bindings
 */
/** IMU (gyro, accel) */
#ifndef FAULT_PREDICTION_IMU_ID
#define FAULT_PREDICTION_IMU_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(FAULT_PREDICTION_IMU_ID)

static abi_event gyro_ev;
static abi_event accel_ev;

/**
 * Call ins_float_invariant_propagate on new gyro measurements.
 * Since acceleration measurement is also needed for propagation,
 * use the last stored accel from #ins_finv_accel.
 */
static void gyro_cb(uint8_t sender_id __attribute__((unused)),
                   uint32_t stamp __attribute__((unused)), struct Int32Rates *gyro)
{
  struct FloatRates gyro_f;
  RATES_FLOAT_OF_BFP(gyro_f, *gyro);

  // Do something here
  float input[6];
  input[0] = last_accel.x;
  input[1] = last_accel.y;
  input[2] = last_accel.z;
  input[3] = gyro_f.p;
  input[4] = gyro_f.q;
  input[5] = gyro_f.r;

  predictFault(input, &cell);

}

static void accel_cb(uint8_t sender_id __attribute__((unused)),
                     uint32_t stamp __attribute__((unused)),
                     struct Int32Vect3 *accel)
{
  ACCELS_FLOAT_OF_BFP(last_accel, *accel);
}


void fault_prediction_init(void)
{
  // Init stuff here
  predictFault_initialize();

  // Bind to ABI messages
  AbiBindMsgIMU_GYRO_INT32(FAULT_PREDICTION_IMU_ID, &gyro_ev, gyro_cb);
  AbiBindMsgIMU_ACCEL_INT32(FAULT_PREDICTION_IMU_ID, &accel_ev, accel_cb);

}

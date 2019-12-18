/*
 * Copyright (C) 2019 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *                    Titouan Verdu <titouan.verdu@enac.fr>
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
 * @file "modules/meteo/cloud_sensor.c"
 *
 * Get data from Cloud Sensor
 * - compute Liquid Water Content (LWC) value from PAYLOAD_FLOAT data
 * - get already computed LWC from PAYLOAD_COMMAND data
 */

#include "modules/meteo/cloud_sensor.h"

#include "generated/airframe.h"
#include "state.h"
#include "subsystems/abi.h"
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"
#include "modules/loggers/sdlog_chibios.h"
#include "modules/loggers/pprzlog_tp.h"
#include "math/pprz_geodetic_float.h"
#include "math/pprz_stat.h"
#include "subsystems/gps.h"

// log to flight recorder by default
#ifndef CLOUD_SENSOR_LOG_FILE
#define CLOUD_SENSOR_LOG_FILE flightrecorder_sdlog
#endif

// max number of values received from cloud sensor
#ifndef CLOUD_SENSOR_RAW_MAX
#define CLOUD_SENSOR_RAW_MAX 16
#endif

// number of actual cloud sensor channels
#ifndef CLOUD_SENSOR_NB
#define CLOUD_SENSOR_NB 4
#endif

// offset inside the raw value array to get cloud sensor channels
#ifndef CLOUD_SENSOR_OFFSET
#define CLOUD_SENSOR_OFFSET 4
#endif

// LWC threshold for cloud border
#ifndef LWC_BORDER_THRESHOLD
#define LWC_BORDER_THRESHOLD 0.05
#endif

// Type of data
#define LWC_RAW 0 // LWC value
#define LWC_BORDER 1 // crossing border

// Default frequencies of cloud sensor leds
// blue, orange, iri1, iri2
#ifndef CLOUD_SENSOR_LAMBDA
#define CLOUD_SENSOR_LAMBDA { 505.f, 590.f, 840.f, 840.f }
#endif
static float lambdas[] = CLOUD_SENSOR_LAMBDA;

/**
 * Structure used to compute the linear regression that provides the
 * angstrom coefficient related to LWC.
 * The slope of the linear regression is computed as cov(X,Y)/var(X).
 * var(X) can be precomputed as LED frequencies are not changing.
 */
struct LWCLinReg {
  float lambda[CLOUD_SENSOR_NB];  ///< list of sensor LED frequencies
  float var_lambda;               ///< variance of lambda
};

/** Cloud sensor structure
 */
struct CloudSensor {
  float raw[CLOUD_SENSOR_RAW_MAX];  ///< raw cloud sensor values
  float values[CLOUD_SENSOR_NB];    ///< preprocessed cloud sensor values
  float lwc;                        ///< computed LWC value
  bool inside_cloud;                ///< in/out status flag
  struct LWCLinReg reg;             ///< linear regression parameters
  uint8_t nb_raw;                   ///< number of raw data in array
};

static struct CloudSensor cloud_sensor;

/** Extern variables
 */
bool cloud_sensor_compute_lwc;
float cloud_sensor_threshold;

// handle precomputed LWC
static float lwc_from_buffer(uint8_t *buf)
{
  int res = 0;
  for (int i = 0; i < DL_PAYLOAD_COMMAND_command_length(buf); i++) {
    res = res * 10 + DL_PAYLOAD_COMMAND_command(buf)[i];
  }
  float lwc = 0.65f * (float) res / 255.0f;
  return lwc;
}

// send CLOUD_SENSOR message
static void send_cloud_sensor_data(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_CLOUD_SENSOR(trans, dev, AC_ID,
      &stateGetPositionLla_i()->lat,
      &stateGetPositionLla_i()->lon,
      &gps.hmsl,
      &gps.tow,
      &cloud_sensor.lwc,
      CLOUD_SENSOR_NB,
      cloud_sensor.raw);
}

// send DC_SHOT message when crossing border
static void border_send_shot_position(void)
{
  static int16_t border_point_nr = 0;
  // angles in decideg
  int16_t phi = DegOfRad(stateGetNedToBodyEulers_f()->phi * 10.0f);
  int16_t theta = DegOfRad(stateGetNedToBodyEulers_f()->theta * 10.0f);
  int16_t psi = DegOfRad(stateGetNedToBodyEulers_f()->psi * 10.0f);
  // course in decideg
  int16_t course = DegOfRad(stateGetHorizontalSpeedDir_f()) * 10;
  // ground speed in cm/s
  uint16_t speed = stateGetHorizontalSpeedNorm_f() * 10;

  DOWNLINK_SEND_DC_SHOT(DefaultChannel, DefaultDevice,
                        &border_point_nr,
                        &stateGetPositionLla_i()->lat,
                        &stateGetPositionLla_i()->lon,
                        &stateGetPositionLla_i()->alt,
                        &gps.hmsl,
                        &phi,
                        &theta,
                        &psi,
                        &course,
                        &speed,
                        &gps.tow);

  border_point_nr++;
}

// test border crossing
static void check_border(void)
{
  if (cloud_sensor.lwc > cloud_sensor_threshold && cloud_sensor.inside_cloud == false) {
    border_send_shot_position();
    cloud_sensor.inside_cloud = true;
  } else if (cloud_sensor.lwc <= cloud_sensor_threshold && cloud_sensor.inside_cloud == true) {
    border_send_shot_position();
    cloud_sensor.inside_cloud = false;
  }
}

// send ABI message
static void send_data(uint32_t stamp)
{
  AbiSendMsgPAYLOAD_DATA(CLOUD_SENSOR_ID, stamp, LWC_RAW, sizeof(float), (uint8_t *)(&cloud_sensor.lwc));
  uint8_t inside = (uint8_t) cloud_sensor.inside_cloud;
  AbiSendMsgPAYLOAD_DATA(CLOUD_SENSOR_ID, stamp, LWC_BORDER, 1, &inside);
}

// init
void cloud_sensor_init(void)
{
  for (int i = 0; i < CLOUD_SENSOR_NB; i++) {
    cloud_sensor.values[i] = 0.f;
    cloud_sensor.reg.lambda[i] = logf(lambdas[i]);
  }
  cloud_sensor.lwc = 0.f;
  cloud_sensor.inside_cloud = false;
  cloud_sensor.reg.var_lambda = variance_f(cloud_sensor.reg.lambda, CLOUD_SENSOR_NB);

  cloud_sensor.nb_raw = 0;

  cloud_sensor_compute_lwc = false;
  cloud_sensor_threshold = LWC_BORDER_THRESHOLD;
}

void cloud_sensor_callback(uint8_t *buf)
{
  uint8_t nb = pprzlink_get_PAYLOAD_FLOAT_values_length(buf);
  cloud_sensor.nb_raw = Min(nb, CLOUD_SENSOR_RAW_MAX);

  if (nb > 0) {
    // new data
    float *b = pprzlink_get_DL_PAYLOAD_FLOAT_values(buf);

    // check that frame is long enough
    if ((nb >= CLOUD_SENSOR_OFFSET + CLOUD_SENSOR_NB) && cloud_sensor_compute_lwc) {
      uint32_t stamp = get_sys_time_usec();

      for (int i = 0; i < CLOUD_SENSOR_NB; i++) {
        cloud_sensor.values[i] = logf(b[i + CLOUD_SENSOR_OFFSET]);
      }

      // compute LWC with a linear regression from cloud sensor raw data
      float cov = covariance_f(cloud_sensor.reg.lambda, cloud_sensor.values, CLOUD_SENSOR_NB);
      float angstrom = cov / cloud_sensor.reg.var_lambda;
      cloud_sensor.lwc = angstrom; // That's it ????

      // test border crossing and send data over ABI
      check_border();
      send_data(stamp);
    }

    // store raw values
    memcpy(cloud_sensor.raw, b, cloud_sensor.nb_raw * sizeof(float));

#if CLOUD_SENSOR_LOG_FILE
    // Log on SD card in flight recorder
    if (*(CLOUD_SENSOR_LOG_FILE.file) != -1) {
      send_cloud_sensor_data(&pprzlog_tp.trans_tx, &(CLOUD_SENSOR_LOG_FILE).device);
      //// store all buff to SD card
      //float raw[nb];
      //memcpy(raw, b, nb * sizeof(float));
      //DOWNLINK_SEND_PAYLOAD_FLOAT(pprzlog_tp, CLOUD_SENSOR_LOG_FILE, nb, raw);
    }
#endif

  }
}

void LWC_callback(uint8_t *buf)
{
  if (DL_PAYLOAD_COMMAND_ac_id(dl_buffer) == AC_ID) {
    uint32_t stamp = get_sys_time_usec();

    // get LWC from ground or external computer
    cloud_sensor.lwc = lwc_from_buffer(buf);

    // test border crossing and send data over ABI
    check_border();
    send_data(stamp);
  }
}

void cloud_sensor_report(void)
{
  send_cloud_sensor_data(&(DefaultChannel).trans_tx, &(DefaultDevice).device);
}


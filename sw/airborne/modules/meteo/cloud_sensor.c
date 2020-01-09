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

// threshold for cloud border
#ifndef CLOUD_SENSOR_BORDER_THRESHOLD
#define CLOUD_SENSOR_BORDER_THRESHOLD 0.05
#endif

// Type of data
#define CLOUD_RAW 0 // LWC value
#define CLOUD_BORDER 1 // crossing border

// Default frequencies of cloud sensor leds
// blue, orange, iri1, iri2
#ifndef CLOUD_SENSOR_LAMBDA
#define CLOUD_SENSOR_LAMBDA { 505.f, 590.f, 840.f, 840.f }
#endif
static float lambdas[] = CLOUD_SENSOR_LAMBDA;

/**
 * Structure used to compute the linear regression that provides the
 * angstrom coefficient.
 * The slope of the linear regression is computed as cov(X,Y)/var(X).
 * var(X) can be precomputed as LED frequencies are not changing.
 */
struct LinReg {
  float lambda[CLOUD_SENSOR_NB];  ///< list of sensor LED frequencies
  float var_lambda;               ///< variance of lambda
};

/** Cloud sensor structure
 */
struct CloudSensor {
  float raw[CLOUD_SENSOR_RAW_MAX];  ///< raw cloud sensor values
  float values[CLOUD_SENSOR_NB];    ///< preprocessed cloud sensor values
  float coef;                       ///< scalar coeff related to cloud parameter (LWC, angstrom, extinction,...)
  bool inside_cloud;                ///< in/out status flag
  struct LinReg reg;                ///< linear regression parameters
  uint8_t nb_raw;                   ///< number of raw data in array
};

static struct CloudSensor cloud_sensor;

/** Extern variables
 */
bool cloud_sensor_compute_coef;
float cloud_sensor_threshold;
float cloud_sensor_background;

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
      &cloud_sensor.coef,
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
  if (cloud_sensor.coef > cloud_sensor_threshold && cloud_sensor.inside_cloud == false) {
    border_send_shot_position();
    cloud_sensor.inside_cloud = true;
  } else if (cloud_sensor.coef <= cloud_sensor_threshold && cloud_sensor.inside_cloud == true) {
    border_send_shot_position();
    cloud_sensor.inside_cloud = false;
  }
}

// send ABI message
static void send_data(uint32_t stamp)
{
  AbiSendMsgPAYLOAD_DATA(CLOUD_SENSOR_ID, stamp, CLOUD_RAW, sizeof(float), (uint8_t *)(&cloud_sensor.coef));
  uint8_t inside = (uint8_t) cloud_sensor.inside_cloud;
  AbiSendMsgPAYLOAD_DATA(CLOUD_SENSOR_ID, stamp, CLOUD_BORDER, 1, &inside);
}

// init
void cloud_sensor_init(void)
{
  for (int i = 0; i < CLOUD_SENSOR_NB; i++) {
    cloud_sensor.values[i] = 0.f;
    cloud_sensor.reg.lambda[i] = logf(lambdas[i]);
  }
  cloud_sensor.coef = 0.f;
  cloud_sensor.inside_cloud = false;
  cloud_sensor.reg.var_lambda = variance_f(cloud_sensor.reg.lambda, CLOUD_SENSOR_NB);

  cloud_sensor.nb_raw = 0;

  cloud_sensor_compute_coef = CLOUD_SENSOR_COEF_SINGLE; // coef from single channel by default
  cloud_sensor_threshold = LWC_BORDER_THRESHOLD;
  cloud_sensor_background = 0.f; // this should be found during the flight
}

void cloud_sensor_callback(uint8_t *buf)
{
  uint8_t nb = pprzlink_get_PAYLOAD_FLOAT_values_length(buf);
  cloud_sensor.nb_raw = Min(nb, CLOUD_SENSOR_RAW_MAX);

  if (nb > 0) {
    // new data
    float *b = pprzlink_get_DL_PAYLOAD_FLOAT_values(buf);
    uint32_t stamp = get_sys_time_usec();

    if (cloud_sensor_compute_coef == CLOUD_SENSOR_COEF_SINGLE) {
      // compute coef from a single channel
    }
    else if (cloud_sensor_compute_coef == CLOUD_SENSOR_COEF_ANGSTROM) {
      // compute angstrom coef from available channels
      // first check that frame is long enough
      if ((nb >= CLOUD_SENSOR_OFFSET + CLOUD_SENSOR_NB)) {
        for (int i = 0; i < CLOUD_SENSOR_NB; i++) {
          cloud_sensor.values[i] = logf(b[i + CLOUD_SENSOR_OFFSET]);
        }

        // compute coef with a linear regression from cloud sensor raw data
        float cov = covariance_f(cloud_sensor.reg.lambda, cloud_sensor.values, CLOUD_SENSOR_NB);
        float angstrom = cov / cloud_sensor.reg.var_lambda;
        cloud_sensor.coef = angstrom; // That's it ????
      }
    }
    // else the coef is computed elsewhere and is assumed available

    // test border crossing and send data over ABI
    check_border();
    send_data(stamp);

    // store raw values
    memcpy(cloud_sensor.raw, b, cloud_sensor.nb_raw * sizeof(float));

#if CLOUD_SENSOR_LOG_FILE
    // Log on SD card in flight recorder
    if (*(CLOUD_SENSOR_LOG_FILE.file) != -1) {
      send_cloud_sensor_data(&pprzlog_tp.trans_tx, &(CLOUD_SENSOR_LOG_FILE).device);
    }
#endif

  }
}

void LWC_callback(uint8_t *buf)
{
  if (DL_PAYLOAD_COMMAND_ac_id(dl_buffer) == AC_ID) {
    uint32_t stamp = get_sys_time_usec();

    // get LWC from ground or external computer
    cloud_sensor.coef = lwc_from_buffer(buf);

    // test border crossing and send data over ABI
    check_border();
    send_data(stamp);
  }
}

void cloud_sensor_report(void)
{
  send_cloud_sensor_data(&(DefaultChannel).trans_tx, &(DefaultDevice).device);
}


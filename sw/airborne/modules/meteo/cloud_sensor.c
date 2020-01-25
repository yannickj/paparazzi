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
 * - compute coef value from PAYLOAD_FLOAT data
 *   - Liquid Water Content (LWC)
 *   - Angstrom coef
 *   - single sensor
 * - get already computed LWC from PAYLOAD_COMMAND data
 */

#include "modules/meteo/cloud_sensor.h"

#include "generated/airframe.h"
#include "state.h"
#include "subsystems/abi.h"
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"
#include "math/pprz_geodetic_float.h"
#include "math/pprz_stat.h"
#include "subsystems/gps.h"

#ifndef SITL
#include "modules/loggers/sdlog_chibios.h"
#include "modules/loggers/pprzlog_tp.h"
#endif

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

// histeresis for cloud border
#ifndef CLOUD_SENSOR_BORDER_HYSTERESIS
#define CLOUD_SENSOR_BORDER_HYSTERESIS 0.01
#endif

// default cloud sensor channel for single detection
#ifndef CLOUD_SENSOR_SINGLE_CHANNEL
#define CLOUD_SENSOR_SINGLE_CHANNEL 1
#endif

// default coef for auto-threshold from background data
#ifndef CLOUD_SENSOR_BACKGROUND_THRESHOLD_COEF
#define CLOUD_SENSOR_BACKGROUND_THRESHOLD_COEF 2.f
#endif

// default coef for auto-hysteresis from background data
#ifndef CLOUD_SENSOR_BACKGROUND_HYSTERESIS_COEF
#define CLOUD_SENSOR_BACKGROUND_HYSTERESIS_COEF 0.5f
#endif

// Type of data
#define CLOUD_RAW 0 // LWC value
#define CLOUD_BORDER 1 // crossing border

// don't report border detection by default
#ifndef CLOUD_SENSOR_REPORT_BORDER_CROSSING
#define CLOUD_SENSOR_REPORT_BORDER_CROSSING FALSE
#endif

// default frequencies of cloud sensor leds
// blue, orange, iri1, iri2
#ifndef CLOUD_SENSOR_LAMBDA
#define CLOUD_SENSOR_LAMBDA { 505.f, 590.f, 840.f, 840.f }
#endif
static float lambdas[] = CLOUD_SENSOR_LAMBDA;

// default calibration coefficient for Angstrom coef
#ifndef CLOUD_SENSOR_ANGSTROM_COEF
#define CLOUD_SENSOR_ANGSTROM_COEF { 1.f, 1.f, 1.f, 1.f }
#endif
static float angstrom_coef[] = CLOUD_SENSOR_ANGSTROM_COEF;

// default calibration offset for Angstrom coef
#ifndef CLOUD_SENSOR_ANGSTROM_OFFSET
#define CLOUD_SENSOR_ANGSTROM_OFFSET { 1.f, 1.f, 1.f, 1.f }
#endif
static float angstrom_offset[] = CLOUD_SENSOR_ANGSTROM_OFFSET;

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

// number of samples to compute the background caracteristics
#ifndef CLOUD_SENSOR_BACKGROUND_NB
#define CLOUD_SENSOR_BACKGROUND_NB 30
#endif

/**
 * Structure used to compute the background and the threshold
 * which is a list of raw data, the mean and the standard deviation
 */
struct Background {
  float raw[CLOUD_SENSOR_NB][CLOUD_SENSOR_BACKGROUND_NB]; ///< lists of raw values
  float mean[CLOUD_SENSOR_NB];                            ///< mean of the lists
  float std[CLOUD_SENSOR_NB];                             ///< std of the lists
  uint8_t idx;                                            ///< current index
};

/** Cloud sensor structure
 */
struct CloudSensor {
  float raw[CLOUD_SENSOR_RAW_MAX];  ///< raw cloud sensor values
  float values[CLOUD_SENSOR_NB];    ///< preprocessed cloud sensor values
  float coef;                       ///< scalar coeff related to cloud parameter (LWC, angstrom, extinction,...)
  bool inside_cloud;                ///< in/out status flag
  struct LinReg reg;                ///< linear regression parameters
  struct Background background;     ///< structure for background parameters
  uint8_t nb_raw;                   ///< number of raw data in array
};

/** Local variables
 */
static struct CloudSensor cloud_sensor;
static bool log_tagged;

/** Extern variables
 */
uint8_t cloud_sensor_compute_coef;
uint8_t cloud_sensor_compute_background;
float cloud_sensor_threshold;
float cloud_sensor_hysteresis;
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
      cloud_sensor.nb_raw,
      cloud_sensor.raw);
}

#if CLOUD_SENSOR_REPORT_BORDER_CROSSING
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
#endif

// test border crossing
static void check_border(void)
{
  if (cloud_sensor.coef > (cloud_sensor_threshold + cloud_sensor_hysteresis) && cloud_sensor.inside_cloud == false) {
#if CLOUD_SENSOR_REPORT_BORDER_CROSSING
    border_send_shot_position();
#endif
    cloud_sensor.inside_cloud = true;
  } else if (cloud_sensor.coef <= (cloud_sensor_threshold - cloud_sensor_hysteresis) && cloud_sensor.inside_cloud == true) {
#if CLOUD_SENSOR_REPORT_BORDER_CROSSING
    border_send_shot_position();
#endif
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
    cloud_sensor.background.mean[i] = 0.f;
    cloud_sensor.background.std[i] = -1.f; // means invalid data or not ready
    cloud_sensor.background.idx = 0;
  }
  cloud_sensor.coef = 0.f;
  cloud_sensor.inside_cloud = false;
  cloud_sensor.reg.var_lambda = variance_f(cloud_sensor.reg.lambda, CLOUD_SENSOR_NB);

  cloud_sensor.nb_raw = 0;

  cloud_sensor_compute_coef = CLOUD_SENSOR_COEF_SINGLE; // coef from single channel by default
  cloud_sensor_threshold = CLOUD_SENSOR_BORDER_THRESHOLD;
  cloud_sensor_hysteresis = CLOUD_SENSOR_BORDER_HYSTERESIS;
  cloud_sensor_background = 0.f; // this should be found during the flight

  log_tagged = false;
}

void cloud_sensor_callback(uint8_t *buf)
{
  uint8_t nb = pprzlink_get_PAYLOAD_FLOAT_values_length(buf);
  cloud_sensor.nb_raw = Min(nb, CLOUD_SENSOR_RAW_MAX);

  if (nb > 0) {
    // new data
    //float *values = pprzlink_get_DL_PAYLOAD_FLOAT_values(buf);
    float values[nb];
    memcpy(values, pprzlink_get_DL_PAYLOAD_FLOAT_values(buf), nb*sizeof(float));
    uint32_t stamp = get_sys_time_usec();

    if (cloud_sensor_compute_coef == CLOUD_SENSOR_COEF_SINGLE) {
      const uint8_t channel = CLOUD_SENSOR_SINGLE_CHANNEL; // short name for single channel
      // first check that frame is long enough
      if (nb > CLOUD_SENSOR_OFFSET + channel) {
        if (cloud_sensor_compute_background) {
          // store a list of raw values
          uint8_t idx = cloud_sensor.background.idx;
          cloud_sensor.background.raw[channel][idx++] = values[CLOUD_SENSOR_OFFSET + channel];
          if (idx == CLOUD_SENSOR_BACKGROUND_NB) {
            // raw value buffer is full
            // compute the background as the mean of the list
            // compute threshold as n-times the std of the list
            cloud_sensor.background.mean[channel] = mean_f(cloud_sensor.background.raw[channel], idx);
            cloud_sensor.background.std[channel] = sqrtf(variance_f(cloud_sensor.background.raw[channel], idx));
            // use the copy to be able to change the value by hand from settings
            cloud_sensor_background = cloud_sensor.background.mean[channel];
            // set the threshold
            cloud_sensor_threshold = CLOUD_SENSOR_BACKGROUND_THRESHOLD_COEF * cloud_sensor.background.std[channel];
            cloud_sensor_hysteresis = CLOUD_SENSOR_BACKGROUND_HYSTERESIS_COEF * cloud_sensor.background.std[channel];
            // reset index
            cloud_sensor.background.idx = 0;
            // end procedure
            cloud_sensor_compute_background = 0;
          }
          else {
            cloud_sensor.background.idx = idx; // increment index
          }
        }
        else {
          // normal run
          // compute coef from a single channel
          cloud_sensor.coef = values[CLOUD_SENSOR_OFFSET + channel] - cloud_sensor_background;
          // test border crossing and send data over ABI
          check_border();
          send_data(stamp);
        }
      }
    }
    else if (cloud_sensor_compute_coef == CLOUD_SENSOR_COEF_ANGSTROM) {
      // compute angstrom coef from available channels
      // first check that frame is long enough
      if ((nb >= CLOUD_SENSOR_OFFSET + CLOUD_SENSOR_NB)) {
        if (cloud_sensor_compute_background) {
          uint8_t idx = cloud_sensor.background.idx;
          // store a list of raw values for all channels
          for (int i = 0; i < CLOUD_SENSOR_NB; i++) {
            cloud_sensor.background.raw[idx][i] = values[CLOUD_SENSOR_OFFSET + i];
          }
          idx++;
          if (idx == CLOUD_SENSOR_BACKGROUND_NB) {
            // raw value buffer is full
            // compute the background as the mean of the list
            for (int i = 0; i < CLOUD_SENSOR_NB; i++) {
              cloud_sensor.background.mean[i] = mean_f(cloud_sensor.background.raw[i], idx);
              cloud_sensor.background.std[i] = sqrtf(variance_f(cloud_sensor.background.raw[i], idx));
            }
            // TODO compute some threshold and hysteresis
            // reset index
            cloud_sensor.background.idx = 0;
            // end procedure
            cloud_sensor_compute_background = 0;
          }
          else {
            cloud_sensor.background.idx = idx; // increment index
          }
        }
        else {
          // normal run
          for (int i = 0; i < CLOUD_SENSOR_NB; i++) {
            float scaled = (values[CLOUD_SENSOR_OFFSET + i] - cloud_sensor.background.mean[i]) * angstrom_coef[i] + angstrom_offset[i];
            cloud_sensor.values[i] = logf(scaled);
          }

          // compute coef with a linear regression from cloud sensor raw data
          float cov = covariance_f(cloud_sensor.reg.lambda, cloud_sensor.values, CLOUD_SENSOR_NB);
          float angstrom = cov / cloud_sensor.reg.var_lambda;
          cloud_sensor.coef = angstrom; // That's it ????

          // test border crossing and send data over ABI
          check_border();
          send_data(stamp);
        }
      }
    }
    // else don't check border from sensor

    // store raw values
    memcpy(cloud_sensor.raw, values, cloud_sensor.nb_raw * sizeof(float));

#if CLOUD_SENSOR_LOG_FILE
    // Log on SD card in flight recorder
    if (CLOUD_SENSOR_LOG_FILE.file != NULL && *(CLOUD_SENSOR_LOG_FILE.file) != -1) {
      if (log_tagged == false && GpsFixValid()) {
        // write at least once ALIVE and GPS messages
        // to log for correct extraction of binary data
        DOWNLINK_SEND_ALIVE(pprzlog_tp, CLOUD_SENSOR_LOG_FILE, 16, MD5SUM);
        // Log GPS for time reference
        uint8_t foo_u8 = 0;
        int16_t foo_16 = 0;
        uint16_t foo_u16 = 0;
        struct UtmCoor_f utm = *stateGetPositionUtm_f();
        int32_t east = utm.east * 100;
        int32_t north = utm.north * 100;
        DOWNLINK_SEND_GPS(pprzlog_tp, CLOUD_SENSOR_LOG_FILE, &gps.fix,
                          &east, &north, &foo_16, &gps.hmsl, &foo_u16, &foo_16,
                          &gps.week, &gps.tow, &utm.zone, &foo_u8);
        log_tagged = true;
      }
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


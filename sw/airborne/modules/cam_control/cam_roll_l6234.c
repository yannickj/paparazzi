/*
 *
 * Copyright (C) 2016 Xavier Paris
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */
/** \file cam_roll_l6234.c
 *  \brief Tilt camera library direct drive L6234
 *
 */

#include "modules/cam_control/cam_roll_l6234.h"
#include "autopilot.h"
#include "generated/modules.h"
#include "state.h"
#include "inter_mcu.h"


// Scale for a mechanical rotation for motor 12N14P
#define MecanicScale 4.0f 
#define MaxPower MAX_PPRZ

#ifndef L6234_ROTATION_DIR
#define L6234_ROTATION_DIR true
#endif

// Configurable by telemetry settings
uint8_t Power;	            
uint8_t Proportional;
uint8_t Derivative;
bool    RotationDir;

static const float F_PI = 3.141592654f;
static const float EPSILON = 0.0174533; // 1 deg

float phi_c;
float resolution=0.0f;
float mecanic_step=0.0f;
float angle=0.0f;
uint8_t rescale_period_cpt=0;
pprz_t *cmd[3];


void computeControl(void);

PRINT_CONFIG_VAR(CAM_ROLL_L6234_PERIODIC_FREQ)



void cam_roll_l6234_init(void)
{
  Power=50;              // from 0 to 100  step 1
  Proportional=50;       // from 0 to 200  step 1
  Derivative=2;          // from 1 to 4    step 1
  RotationDir=L6234_ROTATION_DIR;

  resolution=(2.0f*F_PI/CAM_ROLL_L6234_PERIODIC_FREQ);
  mecanic_step=(MecanicScale * resolution);

  cmd[0]=&(ap_state->commands[COMMAND_PWM1]);
  cmd[1]=&(ap_state->commands[COMMAND_PWM2]);
  cmd[2]=&(ap_state->commands[COMMAND_PWM3]);
}



void cam_roll_l6234_periodic(void)
{
  uint8_t tmp,speed_control;
  float power_f;

  phi_c = stateGetNedToBodyEulers_f()->phi;

  // set correction speed from error
  // the bigger the error, the quicker the correction
  // range from 1 to 4 (speedest)
  if((2*fabs(phi_c))<F_PI) speed_control=1+(uint8_t)(3.0f*fabs(sinf(phi_c)));
  else speed_control=4;

  // control correction speed
  tmp = 5 - speed_control;
  if(rescale_period_cpt==0) {
    // Time to update control if needed
    if((fabs(phi_c)-EPSILON)>0.0f) computeControl();
  }
  rescale_period_cpt++;
  if(rescale_period_cpt>=tmp)rescale_period_cpt=0;

  if(Power > 100) Power=100;
  power_f = ((float)MaxPower * Power / 100.0f);

  // Send cmd[] each cycle to preserve duty cycle 
  // cmd[0]+cmd[1]+cmd[2] = MAX_PPRZ 
  *cmd[0] = (power_f * (1+sinf(angle)))/2.0f;
  *cmd[1] = (power_f * (1+sinf(angle + (2.0f*F_PI/3.0f)))/2.0f);
  *cmd[2] = (power_f * (1+sinf(angle + (2.0f*F_PI*2.0f/3.0f)))/2.0f);
} 



void computeControl()
{
  int8_t dir;
  float step;
  float angle_control;

  // set correction angle from error
  // the bigger the error, the bigger the correction
  // range 0 to 200 (biggest)
  if((2*fabs(phi_c))<F_PI) angle_control=200.0f*fabs(sinf(phi_c));
  else angle_control=200.0f;

  // control correction angle
  if(RotationDir) dir=1;
  else dir=-1;
  step = dir * mecanic_step * angle_control / 100.0f;
  if(phi_c>0.0f) angle = angle + step;
  else           angle = angle - step;
  if(angle>=( 2.0f*F_PI)) angle=(angle-2.0f*F_PI);
  if(angle<=(-2.0f*F_PI)) angle=(angle+2.0f*F_PI); 
} 

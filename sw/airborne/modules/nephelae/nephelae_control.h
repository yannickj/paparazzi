/*
 * Copyright (C) Carbarbaye/Verdu
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/nephelae/nephelae_control.h"
 * @author Carbarbaye/Verdu
 * Control based on the nephelae_observer and so the airplane caracteristics
 */

#ifndef NEPHELAE_CONTROL_H
#define NEPHELAE_CONTROL_H

#include <inttypes.h>
#include "std.h"
#include "paparazzi.h"
#include "generated/airframe.h"

struct CtrlVect{
	float aileron;
	float flap;
	float v_motor;
};

extern struct CtrlVect neph_ctrl_state;

/* inner roll loop parameters */
extern float  neph_h_ctl_roll_setpoint;
extern pprz_t neph_h_ctl_aileron_setpoint;

/* inner pitch loop parameters */
extern float  neph_h_ctl_pitch_setpoint;
extern pprz_t neph_h_ctl_elevator_setpoint;

extern pprz_t neph_v_ctl_throttle_setpoint;
extern pprz_t neph_v_ctl_power_setpoint;

extern float trim_elevator;
extern float trim_aileron;
extern float trim_thrust;
extern float speed_rate;

extern void nephelae_control_init(void);
extern void neph_h_ctl_attitude_loop(void);

#endif


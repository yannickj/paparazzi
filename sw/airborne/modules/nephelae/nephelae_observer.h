/*
 * Copyright (C) Cabarbaye/Verdu
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/nephelae/nephelae_observer.h"
 * @author Cabarbaye/Verdu
 * Observer of state vector based on airplane caracteristics
 */

#ifndef NEPHELAE_OBSERVER_H
#define NEPHELAE_OBSERVER_H

#include "math/pprz_algebra_int.h"
#include "math/pprz_algebra_float.h"

extern float neph_kfw;

struct ObsVect{
	//uint32_t current_time;
	uint32_t gyro_time;
	uint32_t acc_time;
	float debug;
	struct FloatVect3 airspeed;
	struct FloatRates rotspeed;
	struct FloatEulers attitude;
};

extern struct ObsVect neph_obs_state;

extern void nephelae_observer_init(void); //enlever extern

void omega_process(void);
void omega_zero_process(void);
void motor_process(void);
void observer_derivative(void);
void integration(void);

#endif


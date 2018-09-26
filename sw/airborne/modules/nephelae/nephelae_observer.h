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
	struct FloatVect3 airspeed;
	struct FloatRates rotspeed;
	struct FloatEulers attitude;
};

extern struct ObsVect neph_obs_state;

extern void nephelae_observer_init(void);

#endif


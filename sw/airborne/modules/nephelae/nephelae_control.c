/*
 * Copyright (C) Carbarbaye/Verdu
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/nephelae/nephelae_control.c"
 * @author Carbarbaye/Verdu
 * Control based on the nephelae_observer and so the airplane caracteristics
 */

#include "modules/nephelae/nephelae_control.h"

#include <stdio.h>
#include "subsystems/abi.h"
#include "std.h"
#include "math/pprz_algebra_int.h"
#include "subsystems/datalink/telemetry.h"

/** ABI binding for Neph state
*/
#ifndef NEPH_OBS_ID
#define NEPH_OBS_ID ABI_BROADCAST
#endif
static abi_event neph_ev;

/**ABI publisher for Neph State
 */
#define SENDER_ID 25

uint8_t Obs;
uint8_t Ctrl;

static void neph_cb(uint8_t sender_id __attribute__((unused)),
                    uint8_t NbObserver, uint8_t NbControler){
	Obs = NbObserver;
	Ctrl = NbControler;
	Ctrl ++;
	
	//AbiSendMsgNEPHELAE_CTRL(SENDER_ID,&Obs, &Ctrl);
}

void nephelae_control_init(void) {

	AbiBindMsgNEPHELAE_OBS(NEPH_OBS_ID, &neph_ev, neph_cb);

}



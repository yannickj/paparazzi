/*
 * Copyright (C) 2013 Gautier Hattenberger, Alexandre Bustico
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
 */

/*
 * @file firmwares/fixedwing/chibios-libopencm3/chibios_init.c
 *
 */

#include <ch.h>
#include <hal.h>
#include <string.h>
#include "subsystems/chibios-libopencm3/chibios_sdlog.h"
#include "printf.h"
#include "sdLog.h"
#include "usbStorage.h"
#include "pprz_stub.h"
#include "rtcAccess.h"
#include "generated/airframe.h"
#include "chibios_init.h"


#define IRIDIUM_SATCOM_DEBUG 1
#undef IRIDIUM_SATCOM_DEBUG // 0
//#undef USE_IRIDIUM_SATCOM

#ifdef  IRIDIUM_SATCOM_DEBUG
#include "chibios_sdlog.h"
static __attribute__((noreturn)) msg_t thdSatcomReceive(void *arg) ;
#endif

// Delay before starting SD log
#ifndef SDLOG_START_DELAY
#define SDLOG_START_DELAY 30
#endif

#ifndef  SYS_TIME_FREQUENCY
#error SYS_TIME_FREQUENCY should be defined in Makefile.chibios or airframe.xml and be equal to CH_FREQUENCY
#elif SYS_TIME_FREQUENCY != CH_FREQUENCY
#error SYS_TIME_FREQUENCY should be equal to CH_FREQUENCY
#elif  CH_FREQUENCY < (2 * PERIODIC_FREQUENCY)
#error CH_FREQUENCY and SYS_TIME_FREQUENCY should be >= 2 x PERIODIC_FREQUENCY
#endif

static __attribute__((noreturn)) msg_t thd_heartbeat(void *arg);
#define MAX(x , y)  (((x) > (y)) ? (x) : (y))
#define ARRAY_LEN(a) (sizeof(a)/sizeof(a[0]))

Thread *pprzThdPtr = NULL;

static WORKING_AREA(wa_thd_heartbeat, 2048);
void chibios_launch_heartbeat (void);
bool_t sdOk = FALSE;

#if LOG_PROCESS_STATE
static int32_t get_stack_free (Thread *tp);
#endif


#ifdef IRIDIUM_SATCOM_DEBUG
static WORKING_AREA(waThdSatcomReceive, 1024);
#endif

/*
 * Init ChibiOS HAL and Sys
 */
bool_t chibios_init(void) {
  halInit();
  chSysInit();

  PWR->CSR &= ~PWR_CSR_BRE;
  DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;

  chThdCreateStatic(wa_thd_heartbeat, sizeof(wa_thd_heartbeat),
      NORMALPRIO, thd_heartbeat, NULL);

  usbStorageStartPolling ();

#ifdef  IRIDIUM_SATCOM_DEBUG
  chThdCreateStatic(waThdSatcomReceive, sizeof(waThdSatcomReceive),
		    NORMALPRIO, thdSatcomReceive, NULL);
#endif
  

  return RDY_OK;
}

static WORKING_AREA(pprzThd, 4096);
void launch_pprz_thd (int32_t (*thd) (void *arg))
{
  pprzThdPtr = chThdCreateStatic(pprzThd, sizeof(pprzThd), NORMALPRIO+1, thd, NULL);
}


/*
 * Heartbeat thread
 */

typedef struct {
  uint32_t gps;
  uint32_t satcom;
} TimeStamps;

static __attribute__((noreturn)) msg_t thd_heartbeat(void *arg)
{
  (void) arg;
  chRegSetThreadName("pprz heartbeat");

  chThdSleepSeconds (SDLOG_START_DELAY);
  if (usbStorageIsItRunning ())
    chThdSleepSeconds (20000); // stuck here for hours
  else
    sdOk = chibios_logInit(true);

  while (TRUE) {
    palTogglePad (GPIOC, GPIOC_LED3);
    chThdSleepMilliseconds (sdOk == TRUE ? 1000 : 200);
    static TimeStamps timestamps = {.gps=0, .satcom=0};

#if LOG_PROCESS_STATE
    sdLogWriteLog (&processLogFile, "    addr    stack  frestk prio refs  state  time name\r\n");
#endif

    // chSysDisable ();
#if LOG_PROCESS_STATE
    Thread *tp = chRegFirstThread();
    do {

      sdLogWriteLog (&processLogFile, "%.8lx %.8lx %6lu %4lu %4lu [S:%d] %5lu %s\r\n",
          (uint32_t)tp, (uint32_t)tp->p_ctx.r13,
          get_stack_free (tp),
          (uint32_t)tp->p_prio, (uint32_t)(tp->p_refs - 1),
          tp->p_state, (uint32_t)tp->p_time,
          chRegGetThreadName(tp));


      tp = chRegNextThread(tp);
    } while (tp != NULL);
#endif
    // chSysEnable ();

    // we sync gps time to rtc every 5 seconds
    if (chTimeNow() - timestamps.gps > 5000) {
      timestamps.gps = chTimeNow();
      if (getGpsTimeOfWeek() != 0) {
        setRtcFromGps (getGpsWeek(), getGpsTimeOfWeek());
      }
    }
 
    // IRIDIUM DEBUG AND TEST
    // we send a message every 120 seconds
#ifdef  IRIDIUM_SATCOM_DEBUG
    if (chTimeNow() - timestamps.satcom > 120000) {
      static int cnt=0;
      char buff[16];
      
      timestamps.satcom = chTimeNow();
      chsnprintf (buff, sizeof(buff), "iri_%d", cnt++);
      satcomSendBuffer ((uint8_t *) buff, strlen(buff));
    }
#endif
  }
}


#if LOG_PROCESS_STATE
static int32_t get_stack_free (Thread *tp)
{
  int32_t index = 0;
  const uint8_t *maxRamAddr =  (uint8_t*) (0x20000000 + (128*1024));
  const int32_t internalStructSize = 80;

  unsigned long long *stkAdr =  (unsigned long long *) ((uint8_t *) tp  + internalStructSize);
  //unsigned long long *stkAdr =  (unsigned long long *) tp;

  while ((stkAdr[index] == 0x5555555555555555) && ( ((uint8_t *) &(stkAdr[index])) < maxRamAddr))
    index++;

  const int32_t freeBytes = index * sizeof(long long);
  return MAX(0, freeBytes - internalStructSize);
}
#endif


#ifdef  IRIDIUM_SATCOM_DEBUG
static __attribute__((noreturn)) msg_t thdSatcomReceive(void *arg) 
{
  (void)arg;
  uint8_t recBuf[64];
  int32_t  recBytes;

  chRegSetThreadName("SatcomReceive");
  chThdSleepSeconds (60);


  while (TRUE) { 
    recBytes = satcomReceiveBuffer (recBuf, sizeof(recBuf));
    if (recBytes > 0) {
      recBuf[recBytes] = 0;
      sdLogWriteLog(&pprzLogFile, "Satcom thread has received %d bytes : «%s»",
		    recBytes, recBuf);
    } else if (recBytes < 0) {
      sdLogWriteLog(&pprzLogFile, "Satcom thread satcomReceiveBuffer return ERROR %d",
		    recBytes);
    }
    chThdSleepSeconds (1);
  }
}
#endif

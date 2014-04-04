/*
 * Copyright (C) 2014 Gautier Hattenberger, Alexandre Bustico
 *
 * port to C langage, chibios HAL from c++  IridiumSBD - An Arduino library for Iridium
 * http://arduiniana.org.
 * IridiumSBD - An Arduino library for Iridium SBD ("Short Burst Data") Communications
 * Suggested and generously supported by Rock Seven Location Technology
 * (http://rock7mobile.com), makers of the brilliant RockBLOCK satellite modem.
 * Copyright (C) 2013 Mikal Hart
 * All rights reserved.

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
 * @file subsystems/chibios-libopencm3/iridium.h
 * @brief iridium satcom access for high range com
 *
 */

#pragma once

#include <ch.h>
#include <hal.h>

#define IRIDIUM_DIAGS			   1
#define IRIDIUM_ENABLE_FLOW_CONTROL	   1
#define IRIDIUM_LIBRARY_REVISION           1
#define IRIDIUM_SD			   (&SD3)
#define IRIDIUM_BAUD_RATE		   19200

#define IRIDIUM_DEFAULT_AT_TIMEOUT         20
#define IRIDIUM_DEFAULT_CSQ_INTERVAL        8
#define IRIDIUM_DEFAULT_CSQ_INTERVAL_USB   20
#define IRIDIUM_DEFAULT_SBDIX_INTERVAL     24
#define IRIDIUM_DEFAULT_SBDIX_INTERVAL_USB 30
#define IRIDIUM_DEFAULT_SENDRECEIVE_TIME   300
#define IRIDIUM_STARTUP_MAX_TIME           240
#define IRIDIUM_DEFAULT_CSQ_MINIMUM        2

#define IRIDIUM_SUCCESS             0
#define IRIDIUM_ALREADY_AWAKE       1
#define IRIDIUM_SERIAL_FAILURE      2
#define IRIDIUM_PROTOCOL_ERROR      3
#define IRIDIUM_NO_MODEM_DETECTED   5
#define IRIDIUM_SBDIX_FATAL_ERROR   6
#define IRIDIUM_SENDRECEIVE_TIMEOUT 7
#define IRIDIUM_RX_OVERFLOW         8
#define IRIDIUM_REENTRANT           9
#define IRIDIUM_IS_ASLEEP           10



typedef struct {
   // Timings
   int      csqInterval;
   int      sbdixInterval;
   uint32_t atTimeout;
   uint32_t sendReceiveTimeout;

   // State variables
   int remainingMessages;
   bool_t asleep;
   bool_t powered;
   bool_t reentrant;
  //   int  sleepPin;
   int  minimumCSQ;
   bool_t useWorkaround;

} Iridium;


void   iridium_init (Iridium *irdm);
int    iridium_begin (Iridium *irdm);
int    iridium_sendSBDText (Iridium *irdm, const char *message);
int    iridium_sendSBDBinary (Iridium *irdm, const uint8_t *txData, size_t txDataSize);
int    iridium_sendReceiveSBDText (Iridium *irdm, const char *message, uint8_t *rxBuffer,
			       size_t *rxBufferSize);
int    iridium_sendReceiveSBDBinary (Iridium *irdm, const uint8_t *txData, size_t txDataSize,
				 uint8_t *rxBuffer, size_t *rxBufferSize);
int    iridium_getSignalQuality (Iridium *irdm, int *quality);

int    iridium_getWaitingMessageCount (Iridium *irdm);
int    iridium_sleep (Iridium *irdm);
bool_t iridium_isAsleep (Iridium *irdm);
bool_t iridium_isSatlinkPresent (void);
bool_t iridium_isMessagePendingPinUp (void); // warn: when msg pending, send just a pulse, not a level
void   iridium_setPowerProfile (Iridium *irdm, int profile);  // 0 = direct connect  (default), 1 = USB
void   iridium_adjustATTimeout (Iridium *irdm, int seconds);          // default value = 20 seconds
void   iridium_adjustSendReceiveTimeout (Iridium *irdm, int seconds); // default value = 300 seconds
void   iridium_setMinimumSignalQuality (Iridium *irdm, int quality);  // a number between 1 and 5,
						    // default IRIDIUM_DEFAULT_CSQ_MINIMUM
void   iridium_useMSSTMWorkaround (Iridium *irdm, bool_t useWorkAround); // true to use
						   // workaround from Iridium Alert 5/7








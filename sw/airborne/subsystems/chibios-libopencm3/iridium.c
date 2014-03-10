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
 * @file subsystems/chibios-libopencm3/iridium.c
 * @brief iridium satcom access for high range com
 *
 */

#include <ch.h>
#include <hal.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include "iridium.h"
#include "printf.h"

/*
  MACRO HELPER to port from arduino to stm32/chibios
*/
#define millis() (chTimeNow())
#define stream_available() (chQSizeI(&(IRIDIUM_SD->iqueue)))

// Internal utilities
static  bool_t smartWait (uint32_t seconds);
static  bool_t waitForATResponse (Iridium *irdm, char *response, int responseSize, 
				  const char *prompt, const char *terminator);
// char *response=NULL, int responseSize=0, const char *prompt=NULL, const char *terminator="OK\r\n");

static  int  internalBegin (Iridium *irdm);
static  int  internalSendReceiveSBD (Iridium *irdm, const char *txTxtMessage, const uint8_t *txData,
				     size_t txDataSize, uint8_t *rxBuffer, size_t *prxBufferSize);
static  int  internalGetSignalQuality (Iridium *irdm, int *quality);
static  int  internalMSSTMWorkaround (Iridium *irdm, bool_t *okToProceed);
static  int  internalSleep (Iridium *irdm);

static  int  doSBDIX (Iridium *irdm, uint16_t *moCode, uint16_t *moMSN, uint16_t *mtCode, 
		      uint16_t *mtMSN, uint16_t *mtLen, uint16_t *mtRemaining);
static  int  doSBDRB (Iridium *irdm, uint8_t *rxBuffer, size_t *prxBufferSize); // in/out
static  void power (Iridium *irdm, bool_t on);

static  void sendStr (const char *str);
static  void sendInt (uint16_t n);

static  bool_t cancelled (Iridium *irdm);
static void dbg(const char*  msg);



void iridium_init (Iridium *irdm) 
{
  irdm->csqInterval = IRIDIUM_DEFAULT_CSQ_INTERVAL;
  irdm->sbdixInterval = IRIDIUM_DEFAULT_SBDIX_INTERVAL;
  irdm->atTimeout = IRIDIUM_DEFAULT_AT_TIMEOUT;
  irdm->sendReceiveTimeout = IRIDIUM_DEFAULT_SENDRECEIVE_TIME;
  irdm->remainingMessages = -1;
  irdm->asleep = true;
  irdm->reentrant = false;
  irdm->minimumCSQ = IRIDIUM_DEFAULT_CSQ_MINIMUM;
  irdm->useWorkaround = true;
  
  palSetPadMode (IRIDIUM_SLEEP_GPIO, IRIDIUM_SLEEP_PIN, PAL_MODE_OUTPUT_PUSHPULL);
}




// Power on the RockBLOCK or return from sleep
int    iridium_begin (Iridium *irdm)
{
  if (irdm->reentrant)
    return IRIDIUM_REENTRANT;
  
  irdm->reentrant = true;
  int ret = internalBegin(irdm);
  irdm->reentrant = false;

  // Absent a successful startup, keep the device turned off
  if (ret != IRIDIUM_SUCCESS)
    power(irdm, false);
  
  return ret;
}

// Transmit a binary message
int    iridium_sendSBDBinary (Iridium *irdm, const uint8_t *txData, size_t txDataSize)
{
  if (irdm->reentrant)
    return IRIDIUM_REENTRANT;
  
  irdm->reentrant = true;
  int ret = internalSendReceiveSBD(irdm, NULL, txData, txDataSize, NULL, NULL);
  irdm->reentrant = false;
  return ret;
}

// Transmit and receive a binary message
int    iridium_sendReceiveSBDBinary (Iridium *irdm, const uint8_t *txData, size_t txDataSize, 
				     uint8_t *rxBuffer, size_t *rxBufferSize)
{
  if (irdm->reentrant)
    return IRIDIUM_REENTRANT;
  
  irdm->reentrant = true;
  int ret = internalSendReceiveSBD(irdm, NULL, txData, txDataSize, rxBuffer, rxBufferSize);
  irdm->reentrant = false;
  return ret;
}

// Transmit a text message
int    iridium_sendSBDText (Iridium *irdm, const char *message)
{
  if (irdm->reentrant)
    return IRIDIUM_REENTRANT;
  
  irdm->reentrant = true;
  int ret = internalSendReceiveSBD(irdm, message, NULL, 0, NULL, NULL);
  irdm->reentrant = false;
  return ret;
}

// Transmit a text message and receive reply
int    iridium_sendReceiveSBDText (Iridium *irdm, const char *message, uint8_t *rxBuffer, 
				   size_t *rxBufferSize)
{
  if (irdm->reentrant)
    return IRIDIUM_REENTRANT;
  
  irdm->reentrant = true;
  int ret = internalSendReceiveSBD(irdm, message, NULL, 0, rxBuffer, rxBufferSize);
  irdm->reentrant = false;
  return ret;
}

// High-level wrapper for AT+CSQ
int    iridium_getSignalQuality (Iridium *irdm, int *quality)
{
  if (irdm->reentrant)
    return IRIDIUM_REENTRANT;
  
  irdm->reentrant = true;
  int ret = internalGetSignalQuality(irdm, quality);
  irdm->reentrant = false;
  return ret;
}

// Gracefully put device to lower power mode (if sleep pin provided)
int    iridium_sleep (Iridium *irdm)
{
  if (irdm->reentrant)
    return IRIDIUM_REENTRANT;
  
  irdm->reentrant = true;
  int ret = internalSleep(irdm);
  irdm->reentrant = false;
  
  if (ret == IRIDIUM_SUCCESS)
    power(irdm, false); // power off
  return ret;
}

// Return sleep state
bool_t iridium_isAsleep (Iridium *irdm)
{
  return irdm->asleep;
}

// Return number of pending messages
int    iridium_getWaitingMessageCount (Iridium *irdm)
{
  return irdm->remainingMessages;
}

// Define capacitor recharge times
void   iridium_setPowerProfile (Iridium *irdm, int profile)  // 0 = direct connect  (default), 
							     // 1 = USB
{
  switch(profile)
    {
    case 0:
      irdm->csqInterval = IRIDIUM_DEFAULT_CSQ_INTERVAL;
      irdm->sbdixInterval = IRIDIUM_DEFAULT_SBDIX_INTERVAL;
      break;

    case 1:
      irdm->csqInterval = IRIDIUM_DEFAULT_CSQ_INTERVAL_USB;
      irdm->sbdixInterval = IRIDIUM_DEFAULT_SBDIX_INTERVAL_USB;
      break;
    }
}

// Tweak AT timeout 
void   iridium_adjustATTimeout (Iridium *irdm, int seconds)          // default value = 20 seconds
{
  irdm->atTimeout = seconds;
}

// Tweak Send/Receive SBDIX process timeout
void   iridium_adjustSendReceiveTimeout (Iridium *irdm, int seconds) // default value = 300 seconds
{
  irdm->sendReceiveTimeout = seconds;
}



void   iridium_setMinimumSignalQuality (Iridium *irdm, int quality)  // a number between 1 and 5, 
{
  if (quality >= 1 && quality <= 5)
    irdm->minimumCSQ = quality;
}


void   iridium_useMSSTMWorkaround (Iridium *irdm, bool_t useWorkAround) // true to use 
						  // workaround from Iridium Alert 5/7 
{
  irdm->useWorkaround = useWorkAround;
}


/*
  #                 ______   _              _      _                 
  #                /  ____| | |            | |    (_)                
  #                | (___   | |_     __ _  | |_    _     ___         
  #                 \___ \  | __|   / _` | | __|  | |   / __|        
  #                .____) | \ |_   | (_| | \ |_   | |  | (__         
  #                \_____/   \__|   \__,_|  \__|  |_|   \___|        
*/
static  int  internalBegin (Iridium *irdm)
{
  //   dbg("Calling internalBegin\r\n");

  if (!irdm->asleep)
    return IRIDIUM_ALREADY_AWAKE;
  
  power(irdm, true); // power on

  bool_t modemAlive = false;

  uint32_t startupTime = 500; //ms
  for (uint32_t start = millis(); millis() - start < startupTime;)
    if (cancelled (irdm))
      return IRIDIUM_CANCELLED;

  // Turn on modem and wait for a response from "AT" command to begin
  for (uint32_t start = millis(); !modemAlive && millis() - start < 1000 * IRIDIUM_STARTUP_MAX_TIME;)
    {
      sendStr("AT\r");
      modemAlive = waitForATResponse (irdm, NULL, 0, NULL, "OK\r\n");
      if (cancelled (irdm))
	return IRIDIUM_CANCELLED;
      chThdSleepMilliseconds(5);
    }

  if (!modemAlive)
    {
      //      dbg("No modem detected.\r\n");
      return IRIDIUM_NO_MODEM_DETECTED;
    }

  const char * strings[3] = { "ATE1\r", "AT&D0\r", "AT&K0\r" };
  for (int i=0; i<3; ++i)
    {
      sendStr(strings[i]); 
      if (!waitForATResponse (irdm, NULL, 0, NULL, "OK\r\n"))
	return cancelled (irdm) ? IRIDIUM_CANCELLED : IRIDIUM_PROTOCOL_ERROR;
      chThdSleepMilliseconds(5);
    }

  //  dbg("InternalBegin: success!\r\n");
  return IRIDIUM_SUCCESS;
}

static  int  internalSendReceiveSBD (Iridium *irdm, const char *txTxtMessage, const uint8_t *txData,
				     size_t txDataSize, uint8_t *rxBuffer, size_t *prxBufferSize)
{
  //  dbg("internalSendReceive\r\n"); 
  
  if (irdm->asleep)
    return IRIDIUM_IS_ASLEEP;
  
  // Binary transmission?
  if (txData && txDataSize)
    {
      sendStr("AT+SBDWB");
      sendInt(txDataSize);
      sendStr("\r");
      if (!waitForATResponse(irdm, NULL, 0, NULL, "READY\r\n"))
	return cancelled (irdm) ? IRIDIUM_CANCELLED : IRIDIUM_PROTOCOL_ERROR;
      
      uint16_t checksum = 0;
      for (uint32_t i=0; i<txDataSize; ++i)	{
	sdPut (IRIDIUM_SD, txData[i]);
	checksum += (uint16_t)txData[i];
      }
      
      //      dbg("Checksum:");
      //      dbg(checksum);
      //      dbg("\r\n");
      sdPut (IRIDIUM_SD, checksum >> 8);
      sdPut (IRIDIUM_SD, checksum & 0xFF);
      
      if (!waitForATResponse(irdm, NULL, 0, NULL, "0\r\n\r\nOK\r\n"))
	return cancelled (irdm) ? IRIDIUM_CANCELLED : IRIDIUM_PROTOCOL_ERROR;
    }
  
  else // Text transmission

    {
      sendStr("AT+SBDWT=");
      if (txTxtMessage) // It's ok to have a NULL txtTxtMessage if the transaction is RX only
	sendStr(txTxtMessage);
      sendStr("\r");
      if (!waitForATResponse (irdm, NULL, 0, NULL, "OK\r\n"))
	return cancelled (irdm) ? IRIDIUM_CANCELLED : IRIDIUM_PROTOCOL_ERROR;
    }
  
  // Long SBDIX loop begins here
  for (uint32_t start = millis(); millis() - start < 1000 * IRIDIUM_DEFAULT_SENDRECEIVE_TIME;)
    {
      int strength = 0;
      bool_t okToProceed = true;
      int ret = internalGetSignalQuality(irdm, &strength);
      if (ret != IRIDIUM_SUCCESS)
	return ret;
      
      if (irdm->useWorkaround && strength >= irdm->minimumCSQ)
	{
	  okToProceed = false;
	  ret = internalMSSTMWorkaround(irdm, &okToProceed);
	  if (ret != IRIDIUM_SUCCESS) 
            return ret; 
	} 
      
      if ( okToProceed && strength >= irdm->minimumCSQ)
	{ 
	   uint16_t moCode = 0, moMSN = 0, mtCode = 0, mtMSN = 0, mtLen = 0, mtRemaining = 0;
	   ret = doSBDIX(irdm, &moCode, &moMSN, &mtCode, &mtMSN, &mtLen, &mtRemaining);
	   if (ret != IRIDIUM_SUCCESS)
	     return ret;
	  
	  /* dbg("SBDIX MO code: "); */
	  /* dbg(moCode); */
	  /* dbg("\r\n"); */
	  
	  if (moCode <= 4) // successful return!
	    {
	      //	      dbg("SBDIX success!\r\n");
	      
	      irdm->remainingMessages = mtRemaining;
	      if (mtCode == 1 && rxBuffer) // retrieved 1 message
		{
		  //  dbg("Incoming message!\r\n");
		  return doSBDRB(irdm, rxBuffer, prxBufferSize);
		}
	      
	      else
            {
	      // No data returned
	      if (prxBufferSize) 
		*prxBufferSize = 0;
            }
	      return IRIDIUM_SUCCESS;
	    }

	  else if (moCode == 12 || moCode == 14 || moCode == 16) // fatal failure: no retry
	    {
	      dbg("SBDIX fatal!\r\n");
	      return IRIDIUM_SBDIX_FATAL_ERROR;
	    }

	  else // retry      
	    {
	      dbg("Waiting for SBDIX retry...\r\n");
	      if (!smartWait(irdm->sbdixInterval))
		return IRIDIUM_CANCELLED;
	    }
	}

      else // signal strength == 0
	{
	  dbg("Waiting for CSQ retry...\r\n");
	  if (!smartWait(irdm->csqInterval))
            return IRIDIUM_CANCELLED;
	}
      chThdSleepMilliseconds(5);
    } // big wait loop
  
  dbg("SBDIX timeout!\r\n");
  return IRIDIUM_SENDRECEIVE_TIMEOUT;
}

static  int  internalGetSignalQuality (Iridium *irdm, int *quality)
{
  if (irdm->asleep)
    return IRIDIUM_IS_ASLEEP;
  
  char csqResponseBuf[2];
  
  sendStr("AT+CSQ\r");
  if (!waitForATResponse(irdm, csqResponseBuf, sizeof(csqResponseBuf), "+CSQ:", "OK\r\n"))
    return cancelled (irdm) ? IRIDIUM_CANCELLED : IRIDIUM_PROTOCOL_ERROR;
  
  if (isdigit((uint8_t) csqResponseBuf[0])) {
    *quality = atoi(csqResponseBuf);
    return IRIDIUM_SUCCESS;
  }
  
  return IRIDIUM_PROTOCOL_ERROR;
}

static  int  internalMSSTMWorkaround (Iridium *irdm, bool_t *okToProceed)
{
  /*
    According to Iridium 9602 Product Bulletin of 7 May 2013, to overcome a system erratum:

    "Before attempting any of the following commands: +SBDDET, +SBDREG, +SBDI, +SBDIX, +SBDIXA the field application 
    should issue the AT command �MSSTM to the transceiver and evaluate the response to determine if it is valid or not:

    Valid Response: "---MSSTM: XXXXXXXX" where XXXXXXXX is an eight---digit hexadecimal number.

    Invalid Response: "---MSSTM: no network service"

    If the response is invalid, the field application should wait and recheck system time until a valid response is 
    obtained before proceeding. 

    This will ensure that the Iridium SBD transceiver has received a valid system time before attempting SBD communication. 
    The Iridium SBD transceiver will receive the valid system time from the Iridium network when it has a good link to the 
    satellite. Ensuring that the received signal strength reported in response to AT command +CSQ and +CIER is above 2---3 bars 
    before attempting SBD communication will protect against lockout.
  */
  char msstmResponseBuf[24];

  sendStr("AT-MSSTM\r");
  if (!waitForATResponse(irdm, msstmResponseBuf, sizeof(msstmResponseBuf), "-MSSTM: ", "OK\r\n"))
    return cancelled (irdm) ? IRIDIUM_CANCELLED : IRIDIUM_PROTOCOL_ERROR;

  // Response buf now contains either an 8-digit number or the string "no network service"
  *okToProceed = isdigit((uint8_t) msstmResponseBuf[0]);
  return IRIDIUM_SUCCESS;
}

static  int  internalSleep (Iridium *irdm)
{
  if (irdm->asleep)
    return IRIDIUM_IS_ASLEEP;
  
  // Best Practices Guide suggests this before shutdown
  sendStr("AT*F\r");
  
  if (!waitForATResponse (irdm, NULL, 0, NULL, "OK\r\n"))
    return cancelled (irdm) ? IRIDIUM_CANCELLED : IRIDIUM_PROTOCOL_ERROR;
  
  return IRIDIUM_SUCCESS;
}

static  bool_t smartWait (uint32_t seconds)
{
  for (uint32_t start=millis(); (millis() - start) < (1000 * seconds);) {
    chThdSleepMilliseconds (5);
  }
  
  return true;
}

// Wait for response from previous AT command.  This process terminates when "terminator" string is seen or upon timeout.
// If "prompt" string is provided (example "+CSQ:"), then all characters following prompt up to the next CRLF are
// stored in response buffer for later parsing by caller.
static  bool_t waitForATResponse (Iridium *irdm, char *response, int responseSize, 
				  const char *prompt, const char *terminator)
{
  /* C++ default value  */
  /* char *response=NULL */
  /* int responseSize=0 */
  /* const char *prompt=NULL */
  /* const char *terminator="OK\r\n" */


  if (response)
    memset(response, 0, responseSize);

  int matchPromptPos = 0; // Matches chars in prompt
  int matchTerminatorPos = 0; // Matches chars in terminator
  enum {LOOKING_FOR_PROMPT, GATHERING_RESPONSE, LOOKING_FOR_TERMINATOR};
  int promptState = prompt ? LOOKING_FOR_PROMPT : LOOKING_FOR_TERMINATOR;

  for (uint32_t start=millis(); millis() - start < 1000 * irdm->atTimeout;)
    {
      if (cancelled (irdm))
	return false;

      while (!sdGetWouldBlock(IRIDIUM_SD)) {
	char c = sdGet (IRIDIUM_SD);
	if (prompt)
	  switch(promptState)
	    {
            case LOOKING_FOR_PROMPT:
	      if (c == prompt[matchPromptPos])	{
		++matchPromptPos;
		if (prompt[matchPromptPos] == '\0')
		  promptState = GATHERING_RESPONSE;
	      } else {
		matchPromptPos = c == prompt[0] ? 1 : 0;
	      }

	      break;
            case GATHERING_RESPONSE: // gathering reponse from end of prompt to first \r
	      if (response)  {
		if (c == '\r' || responseSize < 2)  {
		  promptState = LOOKING_FOR_TERMINATOR;
		} else {
		  *response++ = c;
		  responseSize--;
		}
	      }
	      break;
	    }

	if (c == terminator[matchTerminatorPos]) {
	  ++matchTerminatorPos;
	  if (terminator[matchTerminatorPos] == '\0')
	    return true;
	} else {
	  matchTerminatorPos = c == terminator[0] ? 1 : 0;
	}
      } // while (stream_available() > 0)
    } // timer loop
  return false;
}

static  bool_t cancelled (Iridium *irdm)
{
  (void) irdm;
  //  if (ISBDCallback != NULL)
  //return !ISBDCallback();
  
  return false;
}

static void dbg(const char*  msg)
{
  (void) msg;
#if IRIDIUM_DIAGS
  DebugTrace (msg);
#endif
}


static  int  doSBDIX (Iridium *irdm, uint16_t *moCode, uint16_t *moMSN, uint16_t *mtCode, 
		      uint16_t *mtMSN, uint16_t *mtLen, uint16_t *mtRemaining)
{
  // xx, xxxxx, xx, xxxxx, xx, xxx
  char sbdixResponseBuf[32];
  sendStr("AT+SBDIX\r");
  if (!waitForATResponse(irdm, sbdixResponseBuf, sizeof(sbdixResponseBuf), "+SBDIX: ", "OK\r\n"))
    return cancelled (irdm) ? IRIDIUM_CANCELLED : IRIDIUM_PROTOCOL_ERROR;

  uint16_t *values[6] = { moCode, moMSN, mtCode, mtMSN, mtLen, mtRemaining };
  for (int i=0; i<6; ++i)   {
    char *p = strtok(i == 0 ? sbdixResponseBuf : NULL, ", ");
    if (p == NULL)
      return IRIDIUM_PROTOCOL_ERROR;
    *values[i] = (uint16_t) atoi (p);
  }
  return IRIDIUM_SUCCESS;
}

static  int  doSBDRB (Iridium *irdm, uint8_t *rxBuffer, size_t *prxBufferSize) // in/out
{
  bool_t rxOverflow = false;

  sendStr("AT+SBDRB\r");
  if (!waitForATResponse(irdm, NULL, 0, NULL, "AT+SBDRB\r")) // waits for its own echo
    return cancelled (irdm) ? IRIDIUM_CANCELLED : IRIDIUM_PROTOCOL_ERROR;

  // Time to read the binary data: size[2], body[size], checksum[2]
  uint32_t start = millis();
  while (millis() - start < 1000 * irdm->atTimeout)    {
    if (cancelled (irdm))
      return IRIDIUM_CANCELLED;
    if (stream_available() >= 2)
      break;
    chThdSleepMilliseconds(5);
  }

  if (stream_available() < 2)
    return IRIDIUM_SENDRECEIVE_TIMEOUT;

  uint16_t size = (256 * sdGet (IRIDIUM_SD)) + sdGet (IRIDIUM_SD);

  for (uint16_t bytesRead = 0; bytesRead < size;)   {
    if (cancelled (irdm))
      return IRIDIUM_CANCELLED;

    if (stream_available())	{
      uint8_t c = sdGet (IRIDIUM_SD);
      bytesRead++;
      if (rxBuffer && prxBufferSize) {
	if (*prxBufferSize > 0) {
	  *rxBuffer++ = c;
	  (*prxBufferSize)--; 
	}
      } else {
	rxOverflow = true;
      }
    }
    
    if (millis() - start >= 1000 * irdm->atTimeout)
      return IRIDIUM_SENDRECEIVE_TIMEOUT;
    chThdSleepMilliseconds(1);
  }

  while (millis() - start < 1000 * irdm->atTimeout)  {
    if (cancelled (irdm))
      return IRIDIUM_CANCELLED;
    if (stream_available() >= 2)
      break;
     chThdSleepMilliseconds(5);
  }

  if (stream_available() < 2)
    return IRIDIUM_SENDRECEIVE_TIMEOUT;

  uint16_t checksum = (256 * sdGet (IRIDIUM_SD)) + sdGet (IRIDIUM_SD);

  // Return actual size of returned buffer
  if (prxBufferSize) 
    *prxBufferSize = (size_t) size;

  return rxOverflow ? IRIDIUM_RX_OVERFLOW : IRIDIUM_SUCCESS;
}

static  void power (Iridium *irdm, bool_t on)
{
  static uint32_t lastPowerOnTime = 0;
  
  irdm->asleep = !on;
  
  if (on)   {
    //    dbg("Powering on RockBLOCK...!\r\n");
    palSetPad (IRIDIUM_SLEEP_GPIO, IRIDIUM_SLEEP_PIN); // HIGH = awake
    lastPowerOnTime = millis();
  } else  {
    // Best Practices Guide suggests waiting at least 2 seconds
    // before powering off again
    uint32_t elapsed = millis() - lastPowerOnTime;
    if (elapsed < 2000)
      chThdSleepMilliseconds(elapsed);
    
    dbg("Powering off RockBLOCK...!\r\n");
    palClearPad (IRIDIUM_SLEEP_GPIO, IRIDIUM_SLEEP_PIN); // LOW = irdm->asleep
  }
}


static  void sendStr (const char *str)
{
  // stream.print(str);
  sdWrite (IRIDIUM_SD, (const uint8_t *) str, strlen(str));
}

static  void sendInt (uint16_t n)
{
  //   stream.print(n);
  chprintf ((BaseSequentialStream *) IRIDIUM_SD, "%u", n); 
}

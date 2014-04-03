#include "ch.h"
#include "hal.h"
#include "varLengthMsgQ.h"
#include "iridium.h"
#include "satCom.h"
#include <string.h>

#ifdef USE_IRIDIUM_SATCOM


#if defined IRIDIUM_DIAGS && defined OLED_EMUL_SD
#include "printf.h"
#define dbgStr(...) {{ \
      chprintf ((BaseSequentialStream *)&OLED_EMUL_SD, "SATCOM> ");	\
      chprintf ((BaseSequentialStream *)&OLED_EMUL_SD, __VA_ARGS__);	\
      chprintf ((BaseSequentialStream *)&OLED_EMUL_SD, "\r\n");		\
      chThdSleepMilliseconds (100) ;  }};
#else
#define dbgStr(...) {}
#endif 

VARLEN_MSGQUEUE_DECL(static, FALSE, satComQueueIn, 512, 16, 
		     __attribute__ ((section(".ccmram"), aligned(8))));
VARLEN_MSGQUEUE_DECL(static, TRUE, satComQueueOut, 512, 16, 
		     __attribute__ ((section(".ccmram"), aligned(8))));

static Iridium irdm;
static Satcom  satcom = {.pollingInterval = SATCOM_DEFAULT_POLLING_INTERVAL, .msgPending = FALSE};

static WORKING_AREA(waThdSatcom, 1536);
static __attribute__((noreturn)) msg_t thdSatcom (void *arg);

static WORKING_AREA(waThdSatcomRiPolling, 128);
static __attribute__((noreturn)) msg_t thdSatcomRiPolling (void *arg);

void satcomInit (void)
{
  varLenMsgDynamicInit (&satComQueueIn);
  varLenMsgDynamicInit (&satComQueueOut);

  chThdCreateStatic(waThdSatcom, sizeof(waThdSatcom),
                    NORMALPRIO, thdSatcom, NULL);
  chThdCreateStatic(waThdSatcomRiPolling, sizeof(waThdSatcomRiPolling),
                    NORMALPRIO, thdSatcomRiPolling, NULL);
}

int32_t satcomSendBuffer (const uint8_t * const buffer, const uint8_t size)
{
  return  varLenMsgQueuePush (&satComQueueOut, buffer, size,
			      VarLenMsgQueue_REGULAR);
}


int32_t satcomReceiveBuffer (uint8_t * const buffer, const uint8_t bufferSize)
{
  if (varLenMsgQueueIsEmpty (&satComQueueIn)) {
    return 0;
  }
  
  return varLenMsgQueuePop (&satComQueueIn, buffer, bufferSize);
}



/*
  
 */
static __attribute__((noreturn)) msg_t thdSatcomRiPolling (void *arg)
{
  (void) arg;

  chRegSetThreadName("Satcom RI Polling");
  while (TRUE) {
    if (iridium_isMessagePendingPinUp ()) {
      satcom.msgPending = TRUE;
    }
    chThdSleepMilliseconds (100) ;
  }
}


static __attribute__((noreturn)) msg_t thdSatcom (void *arg)
{
  (void) arg;

  chRegSetThreadName("Satcom");
  uint8_t rxData[256];
  size_t  rxDataLen;
  int32_t txDataLen; // could be negative to indicate error condition
  int    status = IRIDIUM_SUCCESS;

  iridium_init (&irdm);
  iridium_adjustATTimeout (&irdm, 10);

  // The thread wait for iridium module to be ready
  while (iridium_begin (&irdm) != IRIDIUM_SUCCESS) {
    dbgStr ("iridium_begin FAIL");
    chThdSleepMilliseconds (100) ;    
  }

  dbgStr ("***************** INITIAL iridium begin ********************");

 outerLoop:
  while (TRUE) {
    /*
      TODO (13/03/2014)
      here, the test sould be smarter, and use a switch on status value instead of
      a dumb status != IRIDIUM_SUCCESS test
     */

    if ((status != IRIDIUM_SUCCESS) || varLenMsgQueueIsEmpty (&satComQueueOut)) {
      dbgStr ("varLenMsgQueueIsEmpty");
      if (satcom.pollingInterval >= 60) {
	iridium_sleep (&irdm); 
	dbgStr ("mode sleep");
      }
      chThdSleepSeconds (satcom.pollingInterval);
      
      if (satcom.pollingInterval >= 60) {
	while (iridium_begin (&irdm) != IRIDIUM_SUCCESS) {
	  dbgStr ("iridium_begin FAIL");
	  chThdSleepMilliseconds (100) ;    
	}
	dbgStr ("iridium_begin PASS : mode wake");
      }      
    }

    // We wait for satcom link to be present
    while (!iridium_isSatlinkPresent()) {
      palSetPad(GPIOC, GPIOC_LED2);
      //      chThdSleepMilliseconds (10);
      //#warning "change waiting time for DEBUG, should be changed"
      dbgStr ("NO LINK");
      chThdSleepMilliseconds (1000) ;    
    }
    palClearPad(GPIOC, GPIOC_LED2);
    dbgStr ("LINK OK");
    
    ChunkBufferRO cbro;
    txDataLen = varLenMsgQueuePopChunkTimeout (&satComQueueOut, &cbro, TIME_IMMEDIATE);

    dbgStr ("txDataLen of varLenMsgQueuePopTimeout = %d", txDataLen);
    // send to iridium


    // if there where a msg to send
    if (txDataLen > 0) {
      const uint8_t* txData = cbro.bptr;
      dbgStr ("** Send Msg «%s» [len=%d] **", txData, strlen ((char *) txData));
      rxDataLen = sizeof (rxData); // in out param of sendReceiveSBDBinary
      status = iridium_sendReceiveSBDBinary (&irdm, txData, txDataLen, rxData, &rxDataLen);
      varLenMsgQueueFreeChunk (&satComQueueOut, &cbro);
      if (status != IRIDIUM_SUCCESS) 
	goto outerLoop; // continue would work here, but not in some inner loop at end of function
      // if there was message from modem, push them on queue
      if (rxDataLen) {
	varLenMsgQueuePush (&satComQueueIn, rxData, rxDataLen,
			    VarLenMsgQueue_REGULAR);
#if IRIDIUM_DIAGS
	rxData[rxDataLen] = 0;
	dbgStr ("Receive Msg «%s» [len=%d]", rxData, rxDataLen);
#endif
      } else { // not receiving msg, so probably no pending message
	satcom.msgPending = FALSE;
      }
    } else {
      if (satcom.msgPending) { 
	dbgStr ("No msg to send, test for receive");
	// no message to send, but have to test is there is message to receive
	rxDataLen = sizeof (rxData); // in out param of sendReceiveSBDBinary
	status = iridium_sendReceiveSBDBinary (&irdm, NULL, 0, rxData, &rxDataLen);
	if (status != IRIDIUM_SUCCESS) 
	  goto outerLoop; // continue would work here, but not in some inner loop at end of function
	// if there was message from modem, push them on queue
	if (rxDataLen) {
	  varLenMsgQueuePush (&satComQueueIn, rxData, rxDataLen,
			      VarLenMsgQueue_REGULAR);
#if IRIDIUM_DIAGS
	  rxData[rxDataLen] = 0;
	  dbgStr ("No msg to send, ** Received Msg  «%s» [len=%d]**",  rxData, rxDataLen);
#endif
	} else { // not receiving msg, so probably no pending message
	  satcom.msgPending = FALSE;
	}
      } else {
      	dbgStr ("No Ring Condition Received");
      } 
    }
    
    // if there was pending messages, get them all
    while (iridium_getWaitingMessageCount (&irdm)) {
      dbgStr ("** loop for more Received Msg");
      rxDataLen = sizeof (rxData); // in out param of sendReceiveSBDBinaryiridium
      status = iridium_sendReceiveSBDBinary (&irdm, NULL, 0, rxData, &rxDataLen);
      if (status != IRIDIUM_SUCCESS) 
	goto outerLoop; // continue would NOT work here since we want to continue on outer loop
      if (rxDataLen) {
	varLenMsgQueuePush (&satComQueueIn, rxData, rxDataLen,
			    VarLenMsgQueue_REGULAR);
#if IRIDIUM_DIAGS
	rxData[rxDataLen] = 0;
	dbgStr ("** More Received Msg  «%s» [len=%d]**",  rxData, rxDataLen);
#endif
      }
    }
    satcom.msgPending = FALSE;
  }
  
}


void    satcomSetPollingInterval (const uint32_t pollingInterval)
{
  satcom.pollingInterval = pollingInterval;
}

#endif

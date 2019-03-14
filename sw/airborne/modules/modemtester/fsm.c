/*
 * Copyright (C) 2019 Fabien Garcia <fabien.garcia@enac.fr>
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

#include <stdlib.h>
#include "fsm.h"
#include "pprzlink/messages.h"
#include "subsystems/datalink/telemetry.h"

static int ack_timer_id;

/**
 * Defines the current state of the FSM
 */
static fsm_state_t * current_state_ptr;

#if SEND_DEBUG_MSG
static inline void send_debug_msg(fsm_state_id_t id)
{
  struct pprzlink_msg msg;
  uint8_t data=id;
  msg.trans = &(DefaultChannel).trans_tx;
  msg.dev = &(DefaultDevice).device;
  msg.sender_id = AC_ID;
  msg.receiver_id = 0; // To ground
  msg.component_id = 0;
  pprzlink_msg_v2_send_DEBUG(&msg,1,&data);
}
#endif

/**
 * Function that transition the state machine to another state.
 * It calls the exit() function of the current state, 
 * make new state current ,then
 * the entry() function of the new state.
 */
static void transition(fsm_state_t* state)
{
  if (current_state_ptr->exit!=NULL) {
      current_state_ptr->exit();
    }
  current_state_ptr = state;
  if (current_state_ptr->entry!=NULL) {
      current_state_ptr->entry();
    }
#if SEND_DEBUG_MSG
  send_debug_msg(state->state_id);
#endif
}

// Forward declarations for the states (react, entry and exit functions)
static void react_idle(fsm_event_id_t e);
static void react_availability_command_received(fsm_event_id_t e);
static void send_ack(void);
static void cancel_ack_timer(void);
static void react_throughput_command_received(fsm_event_id_t e);
static void react_availability_mode(fsm_event_id_t e);
static void react_throughput_mode(fsm_event_id_t e);
static void exit_throughput_mode(void);
static void react_end_mode_received(fsm_event_id_t e);

/**
 * Declaration of the states
 */
fsm_state_t idle_state = {idle, &react_idle, NULL, NULL};
fsm_state_t availability_command_received_state = {availability_command_received, &react_availability_command_received, &send_ack, &cancel_ack_timer};
fsm_state_t throughput_command_received_state= {throughput_command_received, &react_throughput_command_received, &send_ack, &cancel_ack_timer};
fsm_state_t availability_mode_state= {availability_mode, &react_availability_mode, NULL, NULL};
fsm_state_t throughput_mode_state= {throughput_mode, &react_throughput_mode, NULL, &exit_throughput_mode};
fsm_state_t end_mode_received_state= {end_mode_received, &react_end_mode_received, &send_ack, &cancel_ack_timer};

// Definitions of the functions for the different states

// idle
static void react_idle(fsm_event_id_t e)
{
  switch (e) {
    case availability_mode_command:
      transition(&availability_command_received_state);
      break;
  case throughput_mode_command:
    transition(&throughput_command_received_state);
      break;
    default:
      break;
    }
}

// availability_command_received
static void react_availability_command_received(fsm_event_id_t e)
{
  switch(e) {
  case master_ack:
    transition(&availability_mode_state);
    break;
  case timeout:
    transition(&availability_command_received_state);
    break;
  default:
    break;
  }
}

// throughput_command_received
static void react_throughput_command_received(fsm_event_id_t e)
{
  switch(e) {
  case master_ack:
    transition(&throughput_mode_state);
    break;
  case timeout:
    transition(&throughput_command_received_state);
    break;
  default:
    break;
  }
}

//  availability_mode
static void react_availability_mode(fsm_event_id_t e)
{
  switch(e) {
  case end_mode:
    transition(&end_mode_received_state);
    break;
  default:
    if (e>= begin_standard_codes && e<= end_standard_codes) {
      uint8_t c=(uint8_t)e;
      uart_put_buffer(modem_tester_dev,0,&c,1);
    }
    break;
  }
}

// throughput_mode
static float first=0;
static float last=0;
static uint32_t bytes=0;

static void exit_throughput_mode(void)
{
  bytes=0;
  first=last=0.0;
}

static void react_throughput_mode(fsm_event_id_t e)
{
  uint32_t data[2];
  switch(e) {
  case end_mode:
    transition(&end_mode_received_state);
    break;
  case end_standard_codes:
    data[0]=SWAP_ENDIAN32(bytes);
    data[1]=SWAP_ENDIAN32((uint32_t)((last-first)*1000000.0));
#if SEND_DEBUG_MSG
    struct pprzlink_msg msg;
    msg.trans = &(DefaultChannel).trans_tx;
    msg.dev = &(DefaultDevice).device;
    msg.sender_id = AC_ID;
    msg.receiver_id = 0; // To ground
    msg.component_id = 0;
    pprzlink_msg_v2_send_DEBUG(&msg,2*sizeof(uint32_t),(uint8_t*)data);
#endif
    uart_put_buffer(modem_tester_dev,0,(uint8_t*)data,sizeof(uint32_t)*2);
    break;
  default:
    if (e>= begin_standard_codes && e< end_standard_codes) {
      last = get_sys_time_float();
      if (!first)
	{
	  first=last;
	}
      bytes++;
    }
    break;
  }
}

//  end_mode_received
static void react_end_mode_received(fsm_event_id_t e)
{
  switch(e) {
  case master_ack:
    transition(&idle_state);
  default:
    break;
  }
}

// For sending an remote ack and waiting a master ack
static void ack_timer(uint8_t id)
{
  (void)id;
  send_ack();
}

static void send_ack(void)
{
  uint8_t data=remote_ack;
  uart_put_buffer(modem_tester_dev,0,&data,1);
  ack_timer_id=sys_time_register_timer(1.0,ack_timer);
}

static void cancel_ack_timer(void)
{
  sys_time_cancel_timer(ack_timer_id);
}

/** Sets the initial state to idle */
static fsm_state_t * current_state_ptr = &idle_state;

/**
 * Dispatch an event to the FSM.
 * @param evt The event to dispatch to the FSM.
 */
void dispatch(fsm_event_id_t e)
{
  if (e==end_test){
    init();
  }
  else {
    current_state_ptr->react(e);
  }
}

/**
 * Reinitialise the FSM
 */
void init(void)
{
  // TODO More to do ? (cancel timers, ...)
  current_state_ptr->exit();
  current_state_ptr = &idle_state;
#if SEND_DEBUG_MSG
  send_debug_msg(current_state_ptr->state_id);
#endif
}

fsm_state_id_t getCurrentStateId(void)
{
  return current_state_ptr->state_id;
}

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

/**
 * @file /modules/modemtester/fsm.h
 * Header file with all definitions for the finite
 * state machine used in modemtester module.
 */

/**
 * Defines the possible states of the FSM.
 */
typedef enum {
  idle,
  availability_command_received,
  throughput_command_received,
  availability_mode,
  throughput_mode,
  end_mode_received,
} fsm_state_id_t;

/**
 * Defines the events of the FSM. The ones with negative values
 * cannot be sent (or received) over the network. @c begin_standard_codes 
 * and @c end_standard_codes are used when sending bytes of non reserved 
 * values.
 */
typedef enum  {
  timeout = -1,
  begin_standard_codes=0,
  end_standard_codes=240,
  availability_mode_command,
  throughput_mode_command,
  end_mode,
  remote_ack,
  master_ack,
  end_test,
} fsm_event_id_t;

/**
 * Defines a state of the FSM. The three
 * function pointers defines the behaviour
 * in this state. They @b MUST be set to 
 * @c NULL when not used.
 */
typedef struct {
  fsm_state_id_t state_id;
  void (*react)(fsm_event_id_t e); /**< function used to proccess incoming events */
  void (*entry)(void); /**< function used when entering the state */
  void (*exit)(void);/**< function used when exiting the state */
}fsm_state_t;

/**
 * Dispatch an event to the FSM.
 * @param evt The event to dispatch to the FSM.
 */
void dispatch(fsm_event_id_t e);

/**
 * Reinitialise the FSM
 */
void init(void);

fsm_state_id_t getCurrentStateId(void);

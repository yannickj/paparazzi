/*
 * Copyright (C) 2012 Piotr Esden-Tempski <piotr@esden.net>
 * Copyright (C) 2019 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 *
 */

/** \file mcu_periph/can.h
 *  \brief arch independent CANBUS API
 *
 */

#ifndef MCU_PERIPH_CAN_H
#define MCU_PERIPH_CAN_H

/**
 * @addtogroup mcu_periph
 * @{
 * @defgroup can CAN Interface
 * @{
 */

/** Receive callback definition
 */
typedef void(* can_rx_callback_t)(uint32_t id, uint8_t *buf, uint8_t len);

/** Init can driver
 *
 * @param callback callback function on new received message
 */
extern void can_init(can_rx_callback_t callback);

/** Basic transfer
 *
 * @param id message id
 * @param buf pointer to the data buffer to transmit
 * @param len number of max bytes to transmit (max 8 for basic can transfer)
 */
extern int can_transmit(uint32_t id, const uint8_t *buf, uint8_t len);

/** @}*/
/** @}*/

#endif /* MCU_PERIPH_CAN_H */

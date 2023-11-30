/******************************************************************************
 *
 *  $Id$
 *
 *  Copyright (C) 2006-2008  Florian Pose, Ingenieurgemeinschaft IgH
 *
 *  This file is part of the IgH EtherCAT Master.
 *
 *  The IgH EtherCAT Master is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License version 2, as
 *  published by the Free Software Foundation.
 *
 *  The IgH EtherCAT Master is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
 *  Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with the IgH EtherCAT Master; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 *  ---
 *
 *  The license mentioned above concerns the source code only. Using the
 *  EtherCAT technology and brand is only permitted in compliance with the
 *  industrial property and similar rights of Beckhoff Automation GmbH.
 *
 *****************************************************************************/

/**
   \file
   EtherCAT state change FSM.
*/

/*****************************************************************************/

#ifndef __EC_FSM_CHANGE_H__
#define __EC_FSM_CHANGE_H__

#include "globals.h"
#include "datagram.h"
#include "slave.h"

/*****************************************************************************/

/**
   状态更改状态机的模式。
*/

typedef enum
{
   EC_FSM_CHANGE_MODE_FULL,    /**< 完全状态更改 */
   EC_FSM_CHANGE_MODE_ACK_ONLY /**< 仅状态确认 */
} ec_fsm_change_mode_t;

/*****************************************************************************/

typedef struct ec_fsm_change ec_fsm_change_t; /**< \see ec_fsm_change */

/**
   EtherCAT状态更改状态机。
*/

struct ec_fsm_change
{
   ec_slave_t *slave;       /**< 运行该状态机的从站 */
   ec_datagram_t *datagram; /**< 在状态机中使用的数据报 */
   unsigned int retries;    /**< 数据报超时时的重试次数 */

   void (*state)(ec_fsm_change_t *, ec_datagram_t *); /**< 从站状态更改状态函数 */
   ec_fsm_change_mode_t mode;                         /**< 完全状态更改或仅确认。 */
   ec_slave_state_t requested_state;                  /**< 输入：状态 */
   ec_slave_state_t old_state;                        /**< 先前的从站状态 */
   unsigned long jiffies_start;                       /**< 更改计时器 */
   uint8_t take_time;                                 /**< 使用发送时间戳 */
   uint8_t spontaneous_change;                        /**< 检测到自发状态更改 */
};

/*****************************************************************************/

void ec_fsm_change_init(ec_fsm_change_t *);
void ec_fsm_change_clear(ec_fsm_change_t *);

void ec_fsm_change_start(ec_fsm_change_t *, ec_slave_t *, ec_slave_state_t);
void ec_fsm_change_ack(ec_fsm_change_t *, ec_slave_t *);

int ec_fsm_change_exec(ec_fsm_change_t *, ec_datagram_t *);
int ec_fsm_change_success(ec_fsm_change_t *);

/*****************************************************************************/

#endif

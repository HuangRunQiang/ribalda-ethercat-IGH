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
   EtherCAT CoE state machines.
*/

/*****************************************************************************/

#ifndef __EC_FSM_COE_H__
#define __EC_FSM_COE_H__

#include "globals.h"
#include "datagram.h"
#include "slave.h"
#include "sdo.h"
#include "sdo_request.h"

/*****************************************************************************/

typedef struct ec_fsm_coe ec_fsm_coe_t; /**< \see ec_fsm_coe */

/** CANopen over EtherCAT 协议的有限状态机。
 */
struct ec_fsm_coe
{
    ec_slave_t *slave;    /**< 运行该有限状态机的从站 */
    unsigned int retries; /**< 数据报超时后的重试次数 */

    void (*state)(ec_fsm_coe_t *, ec_datagram_t *); /**< CoE 状态函数 */
    ec_datagram_t *datagram;                        /**< 上一步使用的数据报。 */
    unsigned long jiffies_start;                    /**< CoE 时间戳。 */
    ec_sdo_t *sdo;                                  /**< 当前 SDO */
    uint8_t subindex;                               /**< 当前子索引 */
    ec_sdo_request_t *request;                      /**< SDO 请求 */
    uint32_t complete_size;                         /**< 分段时使用的大小。 */
    uint8_t toggle;                                 /**< 分段命令的切换位 */
    uint32_t offset;                                /**< 分段下载期间的数据偏移量 */
    uint32_t remaining;                             /**< 分段下载期间剩余的字节数 */
    size_t segment_size;                            /**< 当前分段大小。 */
};

/*****************************************************************************/

void ec_fsm_coe_init(ec_fsm_coe_t *);
void ec_fsm_coe_clear(ec_fsm_coe_t *);

void ec_fsm_coe_dictionary(ec_fsm_coe_t *, ec_slave_t *);
void ec_fsm_coe_transfer(ec_fsm_coe_t *, ec_slave_t *, ec_sdo_request_t *);

int ec_fsm_coe_exec(ec_fsm_coe_t *, ec_datagram_t *);
int ec_fsm_coe_success(const ec_fsm_coe_t *);

/*****************************************************************************/

#endif

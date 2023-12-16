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
 * @file
 * EtherCAT CoE（Sercos over EtherCAT）状态机。
 */

/*****************************************************************************/

#ifndef __EC_FSM_SOE_H__
#define __EC_FSM_SOE_H__

#include "globals.h"
#include "datagram.h"
#include "slave.h"
#include "soe_request.h"

/*****************************************************************************/

typedef struct ec_fsm_soe ec_fsm_soe_t; /**< \see ec_fsm_soe */

/** Sercos over EtherCAT协议的有限状态机。
 */
struct ec_fsm_soe
{
    ec_slave_t *slave;    /**< 运行有限状态机的从站 */
    unsigned int retries; /**< 数据报超时时的重试次数 */

    void (*state)(ec_fsm_soe_t *, ec_datagram_t *); /**< CoE状态函数 */
    ec_datagram_t *datagram;                        /**< 上一步使用的数据报 */
    unsigned long jiffies_start;                    /**< 时间戳 */
    ec_soe_request_t *request;                      /**< SoE请求 */
    off_t offset;                                   /**< 分段写入中的IDN数据偏移量 */
    size_t fragment_size;                           /**< 当前片段的大小 */
};

/*****************************************************************************/

void ec_fsm_soe_init(ec_fsm_soe_t *);
void ec_fsm_soe_clear(ec_fsm_soe_t *);

void ec_fsm_soe_transfer(ec_fsm_soe_t *, ec_slave_t *, ec_soe_request_t *);

int ec_fsm_soe_exec(ec_fsm_soe_t *, ec_datagram_t *);
int ec_fsm_soe_success(const ec_fsm_soe_t *);

/*****************************************************************************/

#endif

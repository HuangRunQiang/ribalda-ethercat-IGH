/******************************************************************************
 *
 *  $Id$
 *
 *  Copyright (C) 2008       Olav Zarges, imc Messsysteme GmbH
 *                2009-2012  Florian Pose <fp@igh-essen.com>
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
 * \file
 * EtherCAT FoE状态机。
 */

/*****************************************************************************/

#ifndef __EC_FSM_FOE_H__
#define __EC_FSM_FOE_H__

#include "globals.h"
#include "../include/ecrt.h"
#include "datagram.h"
#include "slave.h"
#include "foe_request.h"

/*****************************************************************************/

typedef struct ec_fsm_foe ec_fsm_foe_t; /**< \see ec_fsm_foe */

/** CANopen-over-EtherCAT协议的有限状态机。
 */
struct ec_fsm_foe
{
    ec_slave_t *slave;    /**< 运行FSM的从设备。 */
    unsigned int retries; /**< 数据报超时时的重试次数 */

    void (*state)(ec_fsm_foe_t *, ec_datagram_t *); /**< FoE状态函数。 */
    ec_datagram_t *datagram;                        /**< 在前一步中使用的数据报。 */
    unsigned long jiffies_start;                    /**< FoE时间戳。 */
    ec_foe_request_t *request;                      /**< FoE请求。 */

    uint32_t buffer_size;   /**< 传输/接收缓冲区的大小。 */
    uint32_t buffer_offset; /**< 下一个要传输/接收的数据的偏移量。 */
    uint32_t last_packet;   /**< 当前数据包是否为最后一个要发送/接收的数据包。 */
    uint32_t packet_no;     /**< FoE数据包编号。 */
    uint32_t current_size;  /**< 要发送的当前数据包的大小。 */
};

#endif 

/*****************************************************************************/

void ec_fsm_foe_init(ec_fsm_foe_t *);
void ec_fsm_foe_clear(ec_fsm_foe_t *);

int ec_fsm_foe_exec(ec_fsm_foe_t *, ec_datagram_t *);
int ec_fsm_foe_success(const ec_fsm_foe_t *);

void ec_fsm_foe_transfer(ec_fsm_foe_t *, ec_slave_t *, ec_foe_request_t *);

/*****************************************************************************/

#endif

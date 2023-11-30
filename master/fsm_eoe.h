/*****************************************************************************
 *
 *  $Id$
 *
 *  Copyright (C) 2006-2014  Florian Pose, Ingenieurgemeinschaft IgH
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
 ****************************************************************************/

/**
   \file
   EtherCAT EoE set IP parameter state machines.
*/

/****************************************************************************/

#ifndef __EC_FSM_EOE_H__
#define __EC_FSM_EOE_H__

#include "globals.h"
#include "datagram.h"
#include "slave.h"
#include "eoe_request.h"

/****************************************************************************/

typedef struct ec_fsm_eoe ec_fsm_eoe_t; /**< \see ec_fsm_eoe */

/** 以太网通过EtherCAT协议的有限状态机。
 */
struct ec_fsm_eoe
{
    ec_slave_t *slave;    /**< FSM运行的从站 */
    unsigned int retries; /**< 数据报超时时的重试次数 */

    void (*state)(ec_fsm_eoe_t *, ec_datagram_t *); /**< EoE状态函数 */
    ec_datagram_t *datagram;                        /**< 上一步中使用的数据报。 */
    unsigned long jiffies_start;                    /**< 时间戳。 */
    ec_eoe_request_t *request;                      /**< EoE请求 */
};

/****************************************************************************/

void ec_fsm_eoe_init(ec_fsm_eoe_t *);
void ec_fsm_eoe_clear(ec_fsm_eoe_t *);

void ec_fsm_eoe_set_ip_param(ec_fsm_eoe_t *, ec_slave_t *,
                             ec_eoe_request_t *);

int ec_fsm_eoe_exec(ec_fsm_eoe_t *, ec_datagram_t *);
int ec_fsm_eoe_success(const ec_fsm_eoe_t *);


/****************************************************************************/

#endif

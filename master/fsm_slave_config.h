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
   EtherCAT从站配置状态机。
*/

/*****************************************************************************/

#ifndef __EC_FSM_SLAVE_CONFIG_H__
#define __EC_FSM_SLAVE_CONFIG_H__

#include "globals.h"
#include "slave.h"
#include "datagram.h"
#include "fsm_change.h"
#include "fsm_coe.h"
#include "fsm_pdo.h"

/*****************************************************************************/

/** \see ec_fsm_slave_config */
typedef struct ec_fsm_slave_config ec_fsm_slave_config_t;

/** EtherCAT从站配置的有限状态机。
 */
struct ec_fsm_slave_config
{
    ec_datagram_t *datagram;     /**< 在状态机中使用的数据报。 */
    ec_fsm_change_t *fsm_change; /**< 状态变化状态机。 */
    ec_fsm_coe_t *fsm_coe;       /**< CoE状态机。 */
    ec_fsm_soe_t *fsm_soe;       /**< SoE状态机。 */
    ec_fsm_pdo_t *fsm_pdo;       /**< PDO配置状态机。 */

    ec_slave_t *slave;                                       /**< FSM运行的从站。 */
    void (*state)(ec_fsm_slave_config_t *, ec_datagram_t *); /**< 状态函数。 */
    unsigned int retries;                                    /**< 数据报超时时的重试次数。 */
    ec_sdo_request_t *request;                               /**< SDO请求用于SDO配置。 */
    ec_sdo_request_t request_copy;                           /**< 复制的SDO请求。 */
    ec_soe_request_t *soe_request;                           /**< SDO请求用于SDO配置。 */
    ec_soe_request_t soe_request_copy;                       /**< 复制的SDO请求。 */
    unsigned long last_diff_ms;                              /**< 用于同步报告。 */
    unsigned long jiffies_start;                             /**< 用于超时计算。 */
    unsigned int take_time;                                  /**< 在接收数据报后存储jiffies。 */
};

/*****************************************************************************/

void ec_fsm_slave_config_init(ec_fsm_slave_config_t *, ec_slave_t *,
                              ec_fsm_change_t *, ec_fsm_coe_t *, ec_fsm_soe_t *, ec_fsm_pdo_t *);
void ec_fsm_slave_config_clear(ec_fsm_slave_config_t *);

void ec_fsm_slave_config_start(ec_fsm_slave_config_t *);
void ec_fsm_slave_config_quick_start(ec_fsm_slave_config_t *);

int ec_fsm_slave_config_exec(ec_fsm_slave_config_t *, ec_datagram_t *);
int ec_fsm_slave_config_success(const ec_fsm_slave_config_t *);

/*****************************************************************************/

#endif

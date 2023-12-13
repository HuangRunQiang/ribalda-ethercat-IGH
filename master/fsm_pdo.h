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
   EtherCAT PDO配置状态机结构体。
*/

/*****************************************************************************/

#ifndef __EC_FSM_PDO_H__
#define __EC_FSM_PDO_H__

#include "globals.h"
#include "datagram.h"
#include "fsm_coe.h"
#include "fsm_pdo_entry.h"

/*****************************************************************************/

/**
 * \see ec_fsm_pdo
 */
typedef struct ec_fsm_pdo ec_fsm_pdo_t;

/** PDO配置状态机。
 */
struct ec_fsm_pdo
{
    void (*state)(ec_fsm_pdo_t *, ec_datagram_t *); /**< 状态函数。 */
    ec_fsm_coe_t *fsm_coe;                          /**< 使用的CoE状态机。 */
    ec_fsm_pdo_entry_t fsm_pdo_entry;               /**< PDO条目状态机。 */
    ec_pdo_list_t pdos;                             /**< PDO配置。 */
    ec_sdo_request_t request;                       /**< SDO请求。 */
    ec_pdo_t slave_pdo;                             /**< 实际出现在从站中的PDO。 */

    ec_slave_t *slave;      /**< 运行该状态机的从站。 */
    uint8_t sync_index;     /**< 当前同步管理器索引。 */
    ec_sync_t *sync;        /**< 当前同步管理器。 */
    ec_pdo_t *pdo;          /**< 当前PDO。 */
    unsigned int pdo_pos;   /**< 当前PDO的分配位置。 */
    unsigned int pdo_count; /**< 已分配的PDO数目。 */
};


/*****************************************************************************/

void ec_fsm_pdo_init(ec_fsm_pdo_t *, ec_fsm_coe_t *);
void ec_fsm_pdo_clear(ec_fsm_pdo_t *);

void ec_fsm_pdo_start_reading(ec_fsm_pdo_t *, ec_slave_t *);
void ec_fsm_pdo_start_configuration(ec_fsm_pdo_t *, ec_slave_t *);

int ec_fsm_pdo_exec(ec_fsm_pdo_t *, ec_datagram_t *);
int ec_fsm_pdo_success(const ec_fsm_pdo_t *);

/*****************************************************************************/

#endif

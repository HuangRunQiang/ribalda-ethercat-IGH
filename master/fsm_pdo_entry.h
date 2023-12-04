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

/** \file
 * EtherCAT PDO条目配置状态机结构。
 */

/*****************************************************************************/

#ifndef __EC_FSM_PDO_ENTRY_H__
#define __EC_FSM_PDO_ENTRY_H__

#include "globals.h"
#include "datagram.h"
#include "fsm_coe.h"

/*****************************************************************************/

/**
 * \see ec_fsm_pdo_entry
 */
typedef struct ec_fsm_pdo_entry ec_fsm_pdo_entry_t;

/** PDO配置状态机。
 */
struct ec_fsm_pdo_entry
{
        void (*state)(ec_fsm_pdo_entry_t *, ec_datagram_t *); /**< 状态函数 */
        ec_fsm_coe_t *fsm_coe;                                /**< 使用的CoE状态机 */
        ec_sdo_request_t request;                             /**< SDO请求。 */

        ec_slave_t *slave;           /**< 运行状态机的从站。 */
        ec_pdo_t *target_pdo;        /**< 要读取映射的PDO。 */
        const ec_pdo_t *source_pdo;  /**< 具有所需映射的PDO。 */
        const ec_pdo_t *cur_pdo;     /**< 具有当前映射的PDO（仅显示）。 */
        const ec_pdo_entry_t *entry; /**< 当前条目。 */
        unsigned int entry_count;    /**< 条目数。 */
        unsigned int entry_pos;      /**< PDO映射中的位置。 */
};



/*****************************************************************************/

void ec_fsm_pdo_entry_init(ec_fsm_pdo_entry_t *, ec_fsm_coe_t *);
void ec_fsm_pdo_entry_clear(ec_fsm_pdo_entry_t *);

void ec_fsm_pdo_entry_start_reading(ec_fsm_pdo_entry_t *, ec_slave_t *,
                                    ec_pdo_t *);
void ec_fsm_pdo_entry_start_configuration(ec_fsm_pdo_entry_t *, ec_slave_t *,
                                          const ec_pdo_t *, const ec_pdo_t *);

int ec_fsm_pdo_entry_exec(ec_fsm_pdo_entry_t *, ec_datagram_t *);
int ec_fsm_pdo_entry_success(const ec_fsm_pdo_entry_t *);

/*****************************************************************************/

#endif

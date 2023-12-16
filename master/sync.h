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
 * EtherCAT sync manager.
 */

/*****************************************************************************/

#ifndef __EC_SYNC_H__
#define __EC_SYNC_H__

#include "globals.h"
#include "pdo_list.h"
#include "sync_config.h"

/*****************************************************************************/

/** 同步管理器。
 */
typedef struct
{
    ec_slave_t *slave;               /**< 所属从站。 */
    uint16_t physical_start_address; /**< 物理起始地址。 */
    uint16_t default_length;         /**< 数据长度（字节）。 */
    uint8_t control_register;        /**< 控制寄存器值。 */
    uint8_t enable;                  /**< 使能位。 */
    ec_pdo_list_t pdos;              /**< 当前PDO分配。 */
} ec_sync_t;

/*****************************************************************************/

void ec_sync_init(ec_sync_t *, ec_slave_t *);
void ec_sync_init_copy(ec_sync_t *, const ec_sync_t *);
void ec_sync_clear(ec_sync_t *);
void ec_sync_page(const ec_sync_t *, uint8_t, uint16_t,
                  const ec_sync_config_t *, uint8_t, uint8_t *);
int ec_sync_add_pdo(ec_sync_t *, const ec_pdo_t *);
ec_direction_t ec_sync_default_direction(const ec_sync_t *);

/*****************************************************************************/

#endif

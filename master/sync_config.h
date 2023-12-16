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

#ifndef __EC_SYNC_CONFIG_H__
#define __EC_SYNC_CONFIG_H__

#include "globals.h"
#include "pdo_list.h"

/*****************************************************************************/

/** 同步管理器配置。
 */
typedef struct
{
    ec_direction_t dir;               /**< 同步管理器方向。 */
    ec_watchdog_mode_t watchdog_mode; /**< 看门狗模式。 */
    ec_pdo_list_t pdos;               /**< 当前PDO分配。 */
} ec_sync_config_t;

/*****************************************************************************/

void ec_sync_config_init(ec_sync_config_t *);
void ec_sync_config_clear(ec_sync_config_t *);

/*****************************************************************************/

#endif

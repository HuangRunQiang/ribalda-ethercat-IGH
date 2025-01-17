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
 * EtherCAT sync manager configuration methods.
 */

/*****************************************************************************/

#include "globals.h"
#include "sync_config.h"

/*****************************************************************************/

/** 构造函数。
 * 
 * 初始化同步管理器配置对象。
 *
 * @param sync_config 同步管理器配置对象。
 */
void ec_sync_config_init(
    ec_sync_config_t *sync_config /**< 同步管理器配置对象。 */
)
{
    sync_config->dir = EC_DIR_INVALID;
    sync_config->watchdog_mode = EC_WD_DEFAULT;
    ec_pdo_list_init(&sync_config->pdos);
}

/*****************************************************************************/

/** 析构函数。
 * 
 * 清除同步管理器配置对象的数据。
 *
 * @param sync_config 同步管理器配置对象。
 */
void ec_sync_config_clear(
    ec_sync_config_t *sync_config /**< 同步管理器配置对象。 */
)
{
    ec_pdo_list_clear(&sync_config->pdos);
}

/*****************************************************************************/

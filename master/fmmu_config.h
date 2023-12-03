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
 * EtherCAT FMMU配置结构体。
 */

/*****************************************************************************/

#ifndef __EC_FMMU_CONFIG_H__
#define __EC_FMMU_CONFIG_H__

#include "globals.h"
#include "sync.h"

/*****************************************************************************/

/** FMMU配置 */
typedef struct
{
    struct list_head list;          /**< 域使用的列表节点。 */
    const ec_slave_config_t *sc;    /**< EtherCAT从站配置。 */
    const ec_domain_t *domain;      /**< 域。 */
    uint8_t sync_index;             /**< 要使用的同步管理器的索引。 */
    ec_direction_t dir;             /**< FMMU方向。 */
    uint32_t logical_domain_offset; /**< 相对于domain->logical_base_address的逻辑偏移地址。 */
    unsigned int data_size;         /**< 覆盖的PDO大小。 */
} ec_fmmu_config_t;

/*****************************************************************************/

void ec_fmmu_config_init(ec_fmmu_config_t *, ec_slave_config_t *,
                         ec_domain_t *, uint8_t, ec_direction_t);

/**
 * @param fmmu EtherCAT FMMU配置。
 * @param logical_domain_offset 相对于domain->logical_base_address的逻辑偏移地址。
 * @param data_size 覆盖的PDO大小。
*/
void ec_fmmu_set_domain_offset_size(ec_fmmu_config_t *fmmu,
                                    uint32_t logical_domain_offset, unsigned data_size);

void ec_fmmu_config_page(const ec_fmmu_config_t *, const ec_sync_t *,
                         uint8_t *);

/*****************************************************************************/

#endif

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
 * EtherCAT FMMU configuration methods.
 */

/*****************************************************************************/

#include "globals.h"
#include "slave_config.h"
#include "master.h"

#include "fmmu_config.h"

/*****************************************************************************/

/**
 * @brief 初始化FMMU配置。
 * @param fmmu EtherCAT FMMU配置。
 * @param sc EtherCAT从站配置。
 * @param domain EtherCAT域。
 * @param sync_index 使用的同步管理器索引。
 * @param dir PDO的方向。
 * @return 无返回值。
 * @details 该函数用于初始化FMMU配置。它会初始化配置的各个字段，并将配置添加到域中。
 */
void ec_fmmu_config_init(
    ec_fmmu_config_t *fmmu, /**< EtherCAT FMMU配置。 */
    ec_slave_config_t *sc,  /**< EtherCAT从站配置。 */
    ec_domain_t *domain,    /**< EtherCAT域。 */
    uint8_t sync_index,     /**< 使用的同步管理器索引。 */
    ec_direction_t dir      /**< PDO的方向。 */
)
{
    INIT_LIST_HEAD(&fmmu->list);
    fmmu->sc = sc;
    fmmu->sync_index = sync_index;
    fmmu->dir = dir;

    fmmu->logical_domain_offset = 0;
    fmmu->data_size = 0;

    ec_domain_add_fmmu_config(domain, fmmu);
}

/**
 * @brief 设置FMMU配置的域偏移和大小。
 * @param fmmu EtherCAT FMMU配置。
 * @param logical_domain_offset 相对于域的逻辑偏移地址。
 * @param data_size 覆盖的PDO大小。
 * @return 无返回值。
 */
void ec_fmmu_set_domain_offset_size(
    ec_fmmu_config_t *fmmu,         /**< EtherCAT FMMU配置。 */
    uint32_t logical_domain_offset, /**< 相对于域的逻辑偏移地址。 */
    unsigned data_size              /**< 覆盖的PDO大小。 */
)
{
    fmmu->logical_domain_offset = logical_domain_offset;
    fmmu->data_size = data_size;
}

/*****************************************************************************/

/**
 * @brief 初始化FMMU配置页。
 * @param fmmu EtherCAT FMMU配置。
 * @param sync 同步管理器。
 * @param data 配置页内存。
 * @return 无返回值。
 * @details 该函数用于初始化FMMU配置页。传入的内存（\a data）大小必须至少为EC_FMMU_PAGE_SIZE字节。
 */
void ec_fmmu_config_page(
    const ec_fmmu_config_t *fmmu, /**< EtherCAT FMMU配置。 */
    const ec_sync_t *sync,        /**< 同步管理器。 */
    uint8_t *data                 /**> 配置页内存。 */
)
{
    EC_CONFIG_DBG(fmmu->sc, 1, "FMMU: LogOff 0x%08X, Size %3u,"
                               " PhysAddr 0x%04X, SM%u, Dir %s\n",
                  fmmu->logical_domain_offset, fmmu->data_size,
                  sync->physical_start_address, fmmu->sync_index,
                  fmmu->dir == EC_DIR_INPUT ? "in" : "out");

    EC_WRITE_U32(data, fmmu->domain->logical_base_address +
                           fmmu->logical_domain_offset);
    EC_WRITE_U16(data + 4, fmmu->data_size); // fmmu的大小
    EC_WRITE_U8(data + 6, 0x00);             // 逻辑起始位
    EC_WRITE_U8(data + 7, 0x07);             // 逻辑结束位
    EC_WRITE_U16(data + 8, sync->physical_start_address);
    EC_WRITE_U8(data + 10, 0x00); // 物理起始位
    EC_WRITE_U8(data + 11, fmmu->dir == EC_DIR_INPUT ? 0x01 : 0x02);
    EC_WRITE_U16(data + 12, 0x0001); // 使能
    EC_WRITE_U16(data + 14, 0x0000); // 保留
}
/*****************************************************************************/

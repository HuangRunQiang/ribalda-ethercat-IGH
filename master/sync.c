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
 * EtherCAT sync manager methods.
 */

/*****************************************************************************/

#include "globals.h"
#include "slave.h"
#include "master.h"
#include "pdo.h"
#include "sync.h"

/*****************************************************************************/

/** 构造函数。
 */
void ec_sync_init(
    ec_sync_t *sync,  /**< EtherCAT同步管理器。 */
    ec_slave_t *slave /**< EtherCAT从站。 */
)
{
    sync->slave = slave;
    sync->physical_start_address = 0x0000;
    sync->default_length = 0x0000;
    sync->control_register = 0x00;
    sync->enable = 0x00;
    ec_pdo_list_init(&sync->pdos);
}

/*****************************************************************************/

/** 复制构造函数。
 */
void ec_sync_init_copy(
    ec_sync_t *sync,       /**< EtherCAT同步管理器。 */
    const ec_sync_t *other /**< 要复制的同步管理器。 */
)
{
    sync->slave = other->slave;
    sync->physical_start_address = other->physical_start_address;
    sync->default_length = other->default_length;
    sync->control_register = other->control_register;
    sync->enable = other->enable;
    ec_pdo_list_init(&sync->pdos);
    ec_pdo_list_copy(&sync->pdos, &other->pdos);
}

/*****************************************************************************/

/** 析构函数。
 */
void ec_sync_clear(
    ec_sync_t *sync /**< EtherCAT同步管理器。 */
)
{
    ec_pdo_list_clear(&sync->pdos);
}

/*****************************************************************************/

/** 初始化同步管理器配置页。
 *
 * 引用的内存（\a data）必须至少为 \a EC_SYNC_SIZE 字节。
 */
void ec_sync_page(
    const ec_sync_t *sync,               /**< 同步管理器。 */
    uint8_t sync_index,                  /**< 同步管理器的索引。 */
    uint16_t data_size,                  /**< 数据大小。 */
    const ec_sync_config_t *sync_config, /**< 配置。 */
    uint8_t pdo_xfer,                    /**< 非零，如果PDO将通过此同步管理器传输。 */
    uint8_t *data                        /**> 配置内存。 */
)
{
    // 仅在（SII使能已设置或PDO传输）且大小大于0且SM不是虚拟的情况下启用
    uint16_t enable = ((sync->enable & 0x01) || pdo_xfer) && data_size && ((sync->enable & 0x04) == 0);
    uint8_t control = sync->control_register;

    if (sync_config)
    {

        switch (sync_config->dir)
        {
        case EC_DIR_OUTPUT:
        case EC_DIR_INPUT:
            EC_WRITE_BIT(&control, 2,
                         sync_config->dir == EC_DIR_OUTPUT ? 1 : 0);
            EC_WRITE_BIT(&control, 3, 0);
            break;
        default:
            break;
        }

        switch (sync_config->watchdog_mode)
        {
        case EC_WD_ENABLE:
        case EC_WD_DISABLE:
            EC_WRITE_BIT(&control, 6,
                         sync_config->watchdog_mode == EC_WD_ENABLE);
            break;
        default:
            break;
        }
    }

    EC_SLAVE_DBG(sync->slave, 1, "SM%u: 地址 0x%04X, 大小 %3u,"
                                 " 控制 0x%02X, 使能 %u\n",
                 sync_index, sync->physical_start_address,
                 data_size, control, enable);

    EC_WRITE_U16(data, sync->physical_start_address);
    EC_WRITE_U16(data + 2, data_size);
    EC_WRITE_U8(data + 4, control);
    EC_WRITE_U8(data + 5, 0x00); // 状态字节（只读）
    EC_WRITE_U16(data + 6, enable);
}

/*****************************************************************************/

/** 将PDO添加到已知映射PDO的列表中。
 *
 * \return 成功返回0，否则返回 < 0
 */
int ec_sync_add_pdo(
    ec_sync_t *sync,    /**< EtherCAT同步管理器。 */
    const ec_pdo_t *pdo /**< 要映射的PDO。 */
)
{
    return ec_pdo_list_add_pdo_copy(&sync->pdos, pdo);
}

/*****************************************************************************/

/** 根据控制寄存器确定默认方向。
 *
 * \return 方向。
 */
ec_direction_t ec_sync_default_direction(
    const ec_sync_t *sync /**< EtherCAT同步管理器。 */
)
{
    switch ((sync->control_register & 0x0C) >> 2)
    {
    case 0x0:
        return EC_DIR_INPUT;
    case 0x1:
        return EC_DIR_OUTPUT;
    default:
        return EC_DIR_INVALID;
    }
}

/*****************************************************************************/

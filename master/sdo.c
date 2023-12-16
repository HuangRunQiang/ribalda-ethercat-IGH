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
   CANopen SDO functions.
*/

/*****************************************************************************/

#include <linux/slab.h>

#include "master.h"

#include "sdo.h"

/*****************************************************************************/

/**
 * @brief 构造函数。
 * @param sdo SDO 对象的指针。
 * @param slave 父从站。
 * @param index SDO 索引。
 * @details
 * - 初始化 SDO 对象的成员变量。
 * - 将从站指针、索引、对象码、名称和最大子索引设置为初始值。
 * - 初始化 SDO 条目链表。
 */
void ec_sdo_init(
    ec_sdo_t *sdo,     /**< SDO 对象。 */
    ec_slave_t *slave, /**< 父从站。 */
    uint16_t index     /**< SDO 索引。 */
)
{
    sdo->slave = slave;
    sdo->index = index;
    sdo->object_code = 0x00;
    sdo->name = NULL;
    sdo->max_subindex = 0;
    INIT_LIST_HEAD(&sdo->entries);
}

/*****************************************************************************/

/**
 * @brief SDO 析构函数。
 * @param sdo SDO 对象的指针。
 * @details
 * - 清除并释放 SDO 对象。
 * - 释放所有 SDO 条目的内存。
 */
void ec_sdo_clear(
    ec_sdo_t *sdo /**< SDO 对象。 */
)
{
    ec_sdo_entry_t *entry, *next;

    // 释放所有条目
    list_for_each_entry_safe(entry, next, &sdo->entries, list)
    {
        list_del(&entry->list);
        ec_sdo_entry_clear(entry);
        kfree(entry);
    }

    if (sdo->name)
        kfree(sdo->name);
}

/*****************************************************************************/

/**
 * @brief 通过子索引从 SDO 中获取 SDO 条目。
 * @param sdo SDO 对象的指针。
 * @param subindex 条目的子索引。
 * @return 返回请求的 SDO 条目的指针。
 *         如果未找到 SDO 条目，则返回 NULL。
 */
ec_sdo_entry_t *ec_sdo_get_entry(
    ec_sdo_t *sdo,   /**< SDO 对象。 */
    uint8_t subindex /**< 条目的子索引。 */
)
{
    ec_sdo_entry_t *entry;

    list_for_each_entry(entry, &sdo->entries, list)
    {
        if (entry->subindex != subindex)
            continue;
        return entry;
    }

    return NULL;
}

/*****************************************************************************/

/**
 * @brief 通过子索引从 SDO 中获取 SDO 条目。
 * @param sdo SDO 对象的指针。
 * @param subindex 条目的子索引。
 * @return 返回请求的 SDO 条目的指针。
 *         如果未找到 SDO 条目，则返回 NULL。
 * @details
 * - const 版本。
 */
const ec_sdo_entry_t *ec_sdo_get_entry_const(
    const ec_sdo_t *sdo, /**< SDO 对象。 */
    uint8_t subindex     /**< 条目的子索引。 */
)
{
    const ec_sdo_entry_t *entry;

    list_for_each_entry(entry, &sdo->entries, list)
    {
        if (entry->subindex != subindex)
            continue;
        return entry;
    }

    return NULL;
}

/*****************************************************************************/

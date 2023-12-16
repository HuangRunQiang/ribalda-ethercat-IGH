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
   EtherCAT process data object entry methods.
*/

/*****************************************************************************/

#include <linux/slab.h>

#include "pdo_entry.h"

/*****************************************************************************/

/**
 * @brief PDO条目构造函数
 *
 * 初始化PDO条目对象。
 */
void ec_pdo_entry_init(
    ec_pdo_entry_t *entry /**< PDO条目对象。 */
)
{
    entry->name = NULL;
}

/*****************************************************************************/

/**
 * @brief PDO条目复制构造函数
 *
 * @retval  0 成功
 * @retval <0 错误码
 */
int ec_pdo_entry_init_copy(
    ec_pdo_entry_t *entry,      /**< 目标PDO条目对象。 */
    const ec_pdo_entry_t *other /**< 要复制的PDO条目对象。 */
)
{
    entry->index = other->index;
    entry->subindex = other->subindex;
    entry->name = NULL;
    entry->bit_length = other->bit_length;

    return ec_pdo_entry_set_name(entry, other->name);
}

/*****************************************************************************/

/**
 * @brief PDO条目析构函数
 *
 * 清理PDO条目对象的资源。
 */
void ec_pdo_entry_clear(ec_pdo_entry_t *entry /**< PDO条目对象。 */)
{
    if (entry->name)
        kfree(entry->name);
}

/*****************************************************************************/

/**
 * @brief 设置PDO条目名称
 *
 * @retval  0 成功
 * @retval <0 错误码
 */
int ec_pdo_entry_set_name(
    ec_pdo_entry_t *entry, /**< PDO条目对象。 */
    const char *name       /**< 新名称。 */
)
{
    unsigned int len;

    if (entry->name && name && !strcmp(entry->name, name))
        return 0;

    if (entry->name)
        kfree(entry->name);

    if (name && (len = strlen(name)))
    {
        if (!(entry->name = (char *)kmalloc(len + 1, GFP_KERNEL)))
        {
            EC_ERR("分配PDO条目名称失败。\n");
            return -ENOMEM;
        }
        memcpy(entry->name, name, len + 1);
    }
    else
    {
        entry->name = NULL;
    }

    return 0;
}

/*****************************************************************************/

/**
 * @brief 比较两个PDO条目是否相等
 *
 * @retval 1 条目相等
 * @retval 0 条目不相等
 */
int ec_pdo_entry_equal(
    const ec_pdo_entry_t *entry1, /**< 第一个PDO条目。 */
    const ec_pdo_entry_t *entry2  /**< 第二个PDO条目。 */
)
{
    return entry1->index == entry2->index && entry1->subindex == entry2->subindex && entry1->bit_length == entry2->bit_length;
}

/*****************************************************************************/

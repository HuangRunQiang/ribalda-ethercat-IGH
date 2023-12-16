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
   EtherCAT process data object methods.
*/

/*****************************************************************************/

#include <linux/slab.h>
#include <linux/err.h>

#include "pdo.h"

/*****************************************************************************/

/**
 * @brief PDO构造函数。
 * @param pdo EtherCAT PDO。
 * @return 无返回值。
 * @details 初始化给定的EtherCAT PDO。
 */
void ec_pdo_init(
    ec_pdo_t *pdo /**< EtherCAT PDO */
)
{
    pdo->sync_index = -1; // 未分配
    pdo->name = NULL;
    INIT_LIST_HEAD(&pdo->entries);
}

/*****************************************************************************/

/**
 * @brief PDO拷贝构造函数。
 * @param pdo 要创建的PDO。
 * @param other_pdo 要拷贝的PDO。
 * @return 0表示成功，小于0表示错误代码。
 * @details 从另一个PDO拷贝构造给定的PDO。
 */
int ec_pdo_init_copy(
    ec_pdo_t *pdo,            /**< 要创建的PDO。 */
    const ec_pdo_t *other_pdo /**< 要拷贝的PDO。 */
)
{
    int ret = 0;

    pdo->index = other_pdo->index;
    pdo->sync_index = other_pdo->sync_index;
    pdo->name = NULL;
    INIT_LIST_HEAD(&pdo->entries);

    ret = ec_pdo_set_name(pdo, other_pdo->name);
    if (ret < 0)
        goto out_return;

    ret = ec_pdo_copy_entries(pdo, other_pdo);
    if (ret < 0)
        goto out_clear;

    return 0;

out_clear:
    ec_pdo_clear(pdo);
out_return:
    return ret;
}

/*****************************************************************************/

/**
 * @brief PDO析构函数。
 * @param pdo EtherCAT PDO。
 * @return 无返回值。
 * @details 清除给定的EtherCAT PDO。
 */
void ec_pdo_clear(ec_pdo_t *pdo /**< EtherCAT PDO。 */)
{
    if (pdo->name)
        kfree(pdo->name);

    ec_pdo_clear_entries(pdo);
}

/*****************************************************************************/

/**
 * @brief 清除PDO条目列表。
 * @param pdo EtherCAT PDO。
 * @return 无返回值。
 * @details 清除给定的EtherCAT PDO的条目列表。
 */
void ec_pdo_clear_entries(ec_pdo_t *pdo /**< EtherCAT PDO。 */)
{
    ec_pdo_entry_t *entry, *next;

    // 释放所有PDO条目
    list_for_each_entry_safe(entry, next, &pdo->entries, list)
    {
        list_del(&entry->list);
        ec_pdo_entry_clear(entry);
        kfree(entry);
    }
}

/*****************************************************************************/

/**
 * @brief 设置PDO名称。
 * @param pdo PDO。
 * @param name 新名称。
 * @return 0表示成功，小于0表示错误代码。
 * @details 设置给定PDO的名称。
 */
int ec_pdo_set_name(
    ec_pdo_t *pdo,   /**< PDO。 */
    const char *name /**< 新名称。 */
)
{
    unsigned int len;

    if (pdo->name && name && !strcmp(pdo->name, name))
        return 0;

    if (pdo->name)
        kfree(pdo->name);

    if (name && (len = strlen(name)))
    {
        if (!(pdo->name = (char *)kmalloc(len + 1, GFP_KERNEL)))
        {
            EC_ERR("无法分配PDO名称。\n");
            return -ENOMEM;
        }
        memcpy(pdo->name, name, len + 1);
    }
    else
    {
        pdo->name = NULL;
    }

    return 0;
}

/*****************************************************************************/

/**
 * @brief 向配置中添加新的PDO条目。
 * @param pdo PDO。
 * @param index 新条目的索引。
 * @param subindex 新条目的子索引。
 * @param bit_length 新条目的位长度。
 * @return 指向添加的条目的指针，否则ERR_PTR()代码。
 * @details 向给定的PDO添加新的条目。
 */
ec_pdo_entry_t *ec_pdo_add_entry(
    ec_pdo_t *pdo,     /**< PDO。 */
    uint16_t index,    /**< 新条目的索引。 */
    uint8_t subindex,  /**< 新条目的子索引。 */
    uint8_t bit_length /**< 新条目的位长度。 */
)
{
    ec_pdo_entry_t *entry;

    if (!(entry = kmalloc(sizeof(ec_pdo_entry_t), GFP_KERNEL)))
    {
        EC_ERR("无法为PDO条目分配内存。\n");
        return ERR_PTR(-ENOMEM);
    }

    ec_pdo_entry_init(entry);
    entry->index = index;
    entry->subindex = subindex;
    entry->bit_length = bit_length;
    list_add_tail(&entry->list, &pdo->entries);
    return entry;
}

/*****************************************************************************/

/**
 * @brief 从另一个PDO拷贝条目。
 * @param pdo 要替换条目的PDO。
 * @param other 另一个拷贝条目的PDO。
 * @return 0表示成功，小于0表示错误代码。
 * @details 从另一个PDO拷贝条目到给定的PDO。
 */
int ec_pdo_copy_entries(
    ec_pdo_t *pdo,        /**< 要替换条目的PDO。 */
    const ec_pdo_t *other /**< 拷贝条目的PDO。 */
)
{
    ec_pdo_entry_t *entry, *other_entry;
    int ret;

    ec_pdo_clear_entries(pdo);

    list_for_each_entry(other_entry, &other->entries, list)
    {
        if (!(entry = (ec_pdo_entry_t *)
                  kmalloc(sizeof(ec_pdo_entry_t), GFP_KERNEL)))
        {
            EC_ERR("无法为PDO条目拷贝分配内存。\n");
            return -ENOMEM;
        }

        ret = ec_pdo_entry_init_copy(entry, other_entry);
        if (ret < 0)
        {
            kfree(entry);
            return ret;
        }

        list_add_tail(&entry->list, &pdo->entries);
    }

    return 0;
}

/*****************************************************************************/

/**
 * @brief 比较两个PDO的条目。
 * @param pdo1 第一个PDO。
 * @param pdo2 第二个PDO。
 * @return 1表示给定PDO的条目相等，0表示不同。
 * @details 仅比较给定PDO的条目，不比较PDO本身。
 */
int ec_pdo_equal_entries(
    const ec_pdo_t *pdo1, /**< 第一个PDO。 */
    const ec_pdo_t *pdo2  /**< 第二个PDO。 */
)
{
    const struct list_head *head1, *head2, *item1, *item2;
    const ec_pdo_entry_t *entry1, *entry2;

    head1 = item1 = &pdo1->entries;
    head2 = item2 = &pdo2->entries;

    while (1)
    {
        item1 = item1->next;
        item2 = item2->next;

        if ((item1 == head1) ^ (item2 == head2)) // 长度不相等
            return 0;
        if (item1 == head1) // 都已完成
            break;

        entry1 = list_entry(item1, ec_pdo_entry_t, list);
        entry2 = list_entry(item2, ec_pdo_entry_t, list);
        if (!ec_pdo_entry_equal(entry1, entry2))
            return 0;
    }

    return 1;
}

/*****************************************************************************/

/**
 * @brief 获取PDO条目的数量。
 * @param pdo PDO。
 * @return PDO条目的数量。
 * @details 获取给定PDO的条目数量。
 */
unsigned int ec_pdo_entry_count(
    const ec_pdo_t *pdo /**< PDO。 */
)
{
    const ec_pdo_entry_t *entry;
    unsigned int num = 0;

    list_for_each_entry(entry, &pdo->entries, list)
    {
        num++;
    }

    return num;
}

/*****************************************************************************/

/**
 * @brief 根据列表中的位置查找PDO条目。
 * @param pdo PDO。
 * @param pos 列表中的位置。
 * @return 搜索结果，或者NULL。
 * @details 根据给定的位置在列表中查找PDO条目，并返回一个常量指针。
 */
const ec_pdo_entry_t *ec_pdo_find_entry_by_pos_const(
    const ec_pdo_t *pdo, /**< PDO。 */
    unsigned int pos     /**< 列表中的位置。 */
)
{
    const ec_pdo_entry_t *entry;

    list_for_each_entry(entry, &pdo->entries, list)
    {
        if (pos--)
            continue;
        return entry;
    }

    return NULL;
}

/*****************************************************************************/

/**
 * @brief 输出列表中的PDO条目。
 * @param pdo PDO。
 * @return 无返回值。
 * @details 输出给定PDO的条目。
 */
void ec_pdo_print_entries(
    const ec_pdo_t *pdo /**< PDO。 */
)
{
    const ec_pdo_entry_t *entry;

    if (list_empty(&pdo->entries))
    {
        printk(KERN_CONT "(无)");
    }
    else
    {
        list_for_each_entry(entry, &pdo->entries, list)
        {
            printk(KERN_CONT "0x%04X:%02X/%u",
                   entry->index, entry->subindex, entry->bit_length);
            if (entry->list.next != &pdo->entries)
                printk(KERN_CONT " ");
        }
    }
}

/*****************************************************************************/

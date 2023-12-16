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
   EtherCAT PDO list methods.
*/

/*****************************************************************************/

#include <linux/module.h>

#include "globals.h"
#include "pdo.h"
#include "slave_config.h"
#include "master.h"

#include "pdo_list.h"

/*****************************************************************************/

/**
 * @brief 初始化PDO列表。
 * @param pl PDO列表。
 * @return 无返回值。
 * @details 初始化给定的PDO列表。
 */
void ec_pdo_list_init(
    ec_pdo_list_t *pl /**< PDO列表。 */
)
{
    INIT_LIST_HEAD(&pl->list);
}

/*****************************************************************************/

/**
 * @brief 清除PDO列表。
 * @param pl PDO列表。
 * @return 无返回值。
 * @details 清除给定的PDO列表。
 */
void ec_pdo_list_clear(ec_pdo_list_t *pl /**< PDO列表。 */)
{
    ec_pdo_list_clear_pdos(pl);
}

/*****************************************************************************/

/**
 * @brief 清除映射的PDO列表。
 * @param pl PDO列表。
 * @return 无返回值。
 * @details 清除给定的PDO列表中的映射的PDO。
 */
void ec_pdo_list_clear_pdos(ec_pdo_list_t *pl /**< PDO列表。 */)
{
    ec_pdo_t *pdo, *next;

    list_for_each_entry_safe(pdo, next, &pl->list, list)
    {
        list_del_init(&pdo->list);
        ec_pdo_clear(pdo);
        kfree(pdo);
    }
}

/*****************************************************************************/

/**
 * @brief 计算映射的PDO条目的总大小。
 * @param pl PDO列表。
 * @return 数据大小（字节）。
 * @details 计算给定PDO列表中映射的PDO条目的总大小。
 */
uint16_t ec_pdo_list_total_size(
    const ec_pdo_list_t *pl /**< PDO列表。 */
)
{
    unsigned int bit_size;
    const ec_pdo_t *pdo;
    const ec_pdo_entry_t *pdo_entry;
    uint16_t byte_size;

    bit_size = 0;
    list_for_each_entry(pdo, &pl->list, list)
    {
        list_for_each_entry(pdo_entry, &pdo->entries, list)
        {
            bit_size += pdo_entry->bit_length;
        }
    }

    if (bit_size % 8) // 向上取整到整字节
        byte_size = bit_size / 8 + 1;
    else
        byte_size = bit_size / 8;

    return byte_size;
}

/*****************************************************************************/

/**
 * @brief 向列表中添加新的PDO。
 * @param pl PDO列表。
 * @param index PDO索引。
 * @return 指向新PDO的指针，否则ERR_PTR()代码。
 * @details 向给定的PDO列表中添加新的PDO。
 */
ec_pdo_t *ec_pdo_list_add_pdo(
    ec_pdo_list_t *pl, /**< PDO列表。 */
    uint16_t index     /**< PDO索引。 */
)
{
    ec_pdo_t *pdo;

    if (!(pdo = (ec_pdo_t *)kmalloc(sizeof(ec_pdo_t), GFP_KERNEL)))
    {
        EC_ERR("无法为PDO分配内存。\n");
        return ERR_PTR(-ENOMEM);
    }

    ec_pdo_init(pdo);
    pdo->index = index;
    list_add_tail(&pdo->list, &pl->list);
    return pdo;
}

/*****************************************************************************/

/**
 * @brief 将现有PDO的副本添加到列表中。
 * @param pl PDO列表。
 * @param pdo 要添加的PDO。
 * @return 成功返回0，否则小于0。
 * @details 将现有PDO的副本添加到给定的PDO列表中。
 */
int ec_pdo_list_add_pdo_copy(
    ec_pdo_list_t *pl,  /**< PDO列表。 */
    const ec_pdo_t *pdo /**< 要添加的PDO。 */
)
{
    ec_pdo_t *mapped_pdo;
    int ret;

    // PDO已经映射？
    list_for_each_entry(mapped_pdo, &pl->list, list)
    {
        if (mapped_pdo->index != pdo->index)
            continue;
        EC_ERR("PDO 0x%04X已经映射！\n", pdo->index);
        return -EEXIST;
    }

    if (!(mapped_pdo = kmalloc(sizeof(ec_pdo_t), GFP_KERNEL)))
    {
        EC_ERR("无法分配PDO内存。\n");
        return -ENOMEM;
    }

    ret = ec_pdo_init_copy(mapped_pdo, pdo);
    if (ret < 0)
    {
        kfree(mapped_pdo);
        return ret;
    }

    list_add_tail(&mapped_pdo->list, &pl->list);
    return 0;
}

/*****************************************************************************/

/**
 * @brief 对另一个PDO列表进行深拷贝。
 * @param pl PDO列表。
 * @param other 要拷贝的PDO列表。
 * @return 成功返回0，否则小于0。
 * @details 对另一个PDO列表进行深拷贝到给定的PDO列表中。
 */
int ec_pdo_list_copy(
    ec_pdo_list_t *pl,         /**< PDO列表。 */
    const ec_pdo_list_t *other /**< 要拷贝的PDO列表。 */
)
{
    ec_pdo_t *other_pdo;
    int ret;

    ec_pdo_list_clear_pdos(pl);

    // PDO已经映射？
    list_for_each_entry(other_pdo, &other->list, list)
    {
        ret = ec_pdo_list_add_pdo_copy(pl, other_pdo);
        if (ret)
            return ret;
    }

    return 0;
}

/*****************************************************************************/

/**
 * @brief 比较两个PDO列表。
 * @param pl1 第一个列表。
 * @param pl2 第二个列表。
 * @return 1表示给定的PDO列表相等，0表示不同。
 * @details 仅比较列表，而不比较PDO条目（即PDO映射）。
 */
int ec_pdo_list_equal(
    const ec_pdo_list_t *pl1, /**< 第一个列表。 */
    const ec_pdo_list_t *pl2  /**< 第二个列表。 */
)
{
    const struct list_head *h1, *h2, *l1, *l2;
    const ec_pdo_t *p1, *p2;

    h1 = l1 = &pl1->list;
    h2 = l2 = &pl2->list;

    while (1)
    {
        l1 = l1->next;
        l2 = l2->next;

        if ((l1 == h1) ^ (l2 == h2)) // 长度不相等
            return 0;
        if (l1 == h1) // 都已完成
            break;

        p1 = list_entry(l1, ec_pdo_t, list);
        p2 = list_entry(l2, ec_pdo_t, list);

        if (p1->index != p2->index)
            return 0;
    }

    return 1;
}

/*****************************************************************************/

/**
 * @brief 根据索引查找PDO。
 * @param pl PDO列表。
 * @param index PDO索引。
 * @return 搜索结果，或者NULL。
 * @details 在给定的PDO列表中查找具有给定索引的PDO。
 */
ec_pdo_t *ec_pdo_list_find_pdo(
    const ec_pdo_list_t *pl, /**< PDO列表。 */
    uint16_t index           /**< PDO索引。 */
)
{
    ec_pdo_t *pdo;

    list_for_each_entry(pdo, &pl->list, list)
    {
        if (pdo->index != index)
            continue;
        return pdo;
    }

    return NULL;
}

/*****************************************************************************/

/**
 * @brief 根据索引查找PDO，并返回一个常量指针。
 * @param pl PDO列表。
 * @param index PDO索引。
 * @return 搜索结果，或者NULL。
 * @details 在给定的PDO列表中查找具有给定索引的PDO，并返回一个常量指针。
 */
const ec_pdo_t *ec_pdo_list_find_pdo_const(
    const ec_pdo_list_t *pl, /**< PDO列表。 */
    uint16_t index           /**< PDO索引。 */
)
{
    const ec_pdo_t *pdo;

    list_for_each_entry(pdo, &pl->list, list)
    {
        if (pdo->index != index)
            continue;
        return pdo;
    }

    return NULL;
}

/*****************************************************************************/

/**
 * @brief 根据列表中的位置查找PDO。
 * @param pl PDO列表。
 * @param pos 列表中的位置。
 * @return 成功返回0，否则小于0。
 * @details 根据给定的位置在列表中查找PDO，并返回一个常量指针。
 */
const ec_pdo_t *ec_pdo_list_find_pdo_by_pos_const(
    const ec_pdo_list_t *pl, /**< PDO列表。 */
    unsigned int pos         /**< 列表中的位置。 */
)
{
    const ec_pdo_t *pdo;

    list_for_each_entry(pdo, &pl->list, list)
    {
        if (pos--)
            continue;
        return pdo;
    }

    return NULL;
}

/*****************************************************************************/

/**
 * @brief 获取列表中PDO的数量。
 * @param pl PDO列表。
 * @return PDO的数量。
 * @details 获取给定PDO列表中的PDO数量。
 */
unsigned int ec_pdo_list_count(
    const ec_pdo_list_t *pl /**< PDO列表。 */
)
{
    const ec_pdo_t *pdo;
    unsigned int num = 0;

    list_for_each_entry(pdo, &pl->list, list)
    {
        num++;
    }

    return num;
}

/*****************************************************************************/

/**
 * @brief 输出列表中的PDO。
 * @param pl PDO列表。
 * @return 无返回值。
 * @details 输出给定PDO列表中的PDO。
 */
void ec_pdo_list_print(
    const ec_pdo_list_t *pl /**< PDO列表。 */
)
{
    const ec_pdo_t *pdo;

    if (list_empty(&pl->list))
    {
        printk(KERN_CONT "(无)");
    }
    else
    {
        list_for_each_entry(pdo, &pl->list, list)
        {
            printk(KERN_CONT "0x%04X", pdo->index);
            if (pdo->list.next != &pl->list)
                printk(KERN_CONT " ");
        }
    }
}

/*****************************************************************************/

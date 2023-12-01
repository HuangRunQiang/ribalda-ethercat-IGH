/******************************************************************************
 *
 *  $Id$
 *
 *  Copyright (C) 2012  Florian Pose, Ingenieurgemeinschaft IgH
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
 *  vim: expandtab
 *
 *****************************************************************************/

/** \file
 * EtherCAT CoE emergency ring buffer methods.
 */

/*****************************************************************************/

#include <linux/slab.h>

#include "coe_emerg_ring.h"

/*****************************************************************************/

/**
 * @brief 初始化紧急环形缓冲区。
 *
 * @param ring 紧急环形缓冲区对象。
 * @param sc 从设备配置对象。
 *
 * @details 此函数用于初始化紧急环形缓冲区。它设置紧急环形缓冲区对象的成员变量，并分配内存空间。
 * 紧急消息的读写索引均初始化为0，溢出计数初始化为0。
 */
void ec_coe_emerg_ring_init(
    ec_coe_emerg_ring_t *ring, /**< 紧急环形缓冲区。 */
    ec_slave_config_t *sc      /**< 从设备配置。 */
)
{
    ring->sc = sc;
    ring->msgs = NULL;
    ring->size = 0;
    ring->read_index = 0;
    ring->write_index = 0;
    ring->overruns = 0;
}

/*****************************************************************************/

/**
 * @brief 清理紧急环形缓冲区。
 *
 * @param ring 紧急环形缓冲区对象。
 *
 * @details 此函数用于清理紧急环形缓冲区。如果紧急消息缓冲区不为空，则释放其内存空间。
 */
void ec_coe_emerg_ring_clear(
    ec_coe_emerg_ring_t *ring /**< 紧急环形缓冲区。 */
)
{
    if (ring->msgs)
    {
        kfree(ring->msgs);
    }
}

/*****************************************************************************/

/**
 * @brief 设置环形缓冲区的大小。
 *
 * @param ring 紧急环形缓冲区对象。
 * @param size 环形缓冲区中消息的最大数量。
 * @return 如果设置成功，则返回0；否则返回负数的错误代码。
 *
 * @details 此函数用于设置环形缓冲区的大小。如果指定的大小小于0，则将其设置为0。
 * 然后，它会释放已存在的紧急消息缓冲区，并重新分配指定大小的内存空间。
 * 如果分配内存失败，则返回-ENOMEM表示内存不足。
 */
int ec_coe_emerg_ring_size(
    ec_coe_emerg_ring_t *ring, /**< 紧急环形缓冲区。 */
    size_t size                /**< 环形缓冲区中消息的最大数量。 */
)
{
    ring->size = 0;

    if (size < 0)
    {
        size = 0;
    }

    ring->read_index = ring->write_index = 0;

    if (ring->msgs)
    {
        kfree(ring->msgs);
    }
    ring->msgs = NULL;

    if (size == 0)
    {
        return 0;
    }

    ring->msgs = kmalloc(sizeof(ec_coe_emerg_msg_t) * (size + 1), GFP_KERNEL);
    if (!ring->msgs)
    {
        return -ENOMEM;
    }

    ring->size = size;
    return 0;
}

/*****************************************************************************/

/**
 * @brief 添加新的紧急消息。
 *
 * @param ring 紧急环形缓冲区对象。
 * @param msg 紧急消息的数据。
 *
 * @details 此函数用于向紧急环形缓冲区添加新的紧急消息。如果缓冲区已满，则溢出计数增加。
 */
void ec_coe_emerg_ring_push(
    ec_coe_emerg_ring_t *ring, /**< 紧急环形缓冲区。 */
    const u8 *msg              /**< 紧急消息。 */
)
{
    if (!ring->size ||
        (ring->write_index + 1) % (ring->size + 1) == ring->read_index)
    {
        ring->overruns++;
        return;
    }

    memcpy(ring->msgs[ring->write_index].data, msg,
           EC_COE_EMERGENCY_MSG_SIZE);
    ring->write_index = (ring->write_index + 1) % (ring->size + 1);
}

/*****************************************************************************/

/**
 * @brief 从环形缓冲区中移除紧急消息。
 *
 * @param ring 紧急环形缓冲区对象。
 * @param msg 存储紧急消息的内存。
 * @return 如果成功移除消息，则返回0；否则返回负数的错误代码。
 *
 * @details 此函数用于从环形缓冲区中移除紧急消息。如果缓冲区为空，则返回-ENOENT表示没有可移除的消息。
 */
int ec_coe_emerg_ring_pop(
    ec_coe_emerg_ring_t *ring, /**< 紧急环形缓冲区。 */
    u8 *msg                    /**< 存储紧急消息的内存。 */
)
{
    if (ring->read_index == ring->write_index)
    {
        return -ENOENT;
    }

    memcpy(msg, ring->msgs[ring->read_index].data, EC_COE_EMERGENCY_MSG_SIZE);
    ring->read_index = (ring->read_index + 1) % (ring->size + 1);
    return 0;
}

/*****************************************************************************/

/**
 * @brief 清除环形缓冲区。
 *
 * @param ring 紧急环形缓冲区对象。
 * @return 如果清除成功，则返回0；否则返回负数的错误代码。
 *
 * @details 此函数用于清除环形缓冲区。它将读写索引设置为相同的值，并将溢出计数重置为0。
 */
int ec_coe_emerg_ring_clear_ring(
    ec_coe_emerg_ring_t *ring /**< 紧急环形缓冲区。 */
)
{
    ring->read_index = ring->write_index;
    ring->overruns = 0;
    return 0;
}

/*****************************************************************************/

/**
 * @brief 读取溢出次数。
 *
 * @param ring 紧急环形缓冲区对象。
 * @return 溢出次数。
 *
 * @details 此函数用于读取环形缓冲区的溢出次数。
 */
int ec_coe_emerg_ring_overruns(
    ec_coe_emerg_ring_t *ring /**< 紧急环形缓冲区。 */
)
{
    return ring->overruns;
}

/*****************************************************************************/

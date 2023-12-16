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
 * Canopen over EtherCAT SDO request functions.
 */

/*****************************************************************************/

#include <linux/module.h>
#include <linux/jiffies.h>
#include <linux/slab.h>

#include "sdo_request.h"

/*****************************************************************************/

/** 等待 SDO 传输响应的默认超时时间（以毫秒为单位）。
 */
#define EC_SDO_REQUEST_RESPONSE_TIMEOUT 1000

/*****************************************************************************/

/** 清除 SDO 请求的数据。
 */
void ec_sdo_request_clear_data(ec_sdo_request_t *);

/*****************************************************************************/

/** SDO 请求的构造函数。
 */
void ec_sdo_request_init(
    ec_sdo_request_t *req /**< SDO 请求。 */
)
{
    req->complete_access = 0;
    req->data = NULL;
    req->mem_size = 0;
    req->data_size = 0;
    req->dir = EC_DIR_INVALID;
    req->issue_timeout = 0; // 无超时
    req->response_timeout = EC_SDO_REQUEST_RESPONSE_TIMEOUT;
    req->state = EC_INT_REQUEST_INIT;
    req->errno = 0;
    req->abort_code = 0x00000000;
}

/*****************************************************************************/

/**
 * @brief 清除 SDO 请求。
 * @param req SDO 请求的指针。
 * @details
 * - 清除 SDO 请求的数据。
 */
void ec_sdo_request_clear(
    ec_sdo_request_t *req /**< SDO 请求。 */
)
{
    ec_sdo_request_clear_data(req);
}

/*****************************************************************************/

/**
 * @brief 复制另一个 SDO 请求。
 * @param req SDO 请求的指针。
 * @param other 要复制的其他 SDO 请求的指针。
 * @return 返回操作结果的整数值。
 * @details
 * - 复制其他 SDO 请求的索引、子索引和数据。
 * - 返回 0 表示成功，否则返回负数的错误码。
 */
int ec_sdo_request_copy(
    ec_sdo_request_t *req,        /**< SDO 请求。 */
    const ec_sdo_request_t *other /**< 要复制的其他 SDO 请求。 */
)
{
    req->complete_access = other->complete_access;
    req->index = other->index;
    req->subindex = other->subindex;
    return ec_sdo_request_copy_data(req, other->data, other->data_size);
}

/*****************************************************************************/

/**
 * @brief 清除 SDO 请求的数据。
 * @param req SDO 请求的指针。
 * @details
 * - 如果数据不为空，释放数据的内存。
 * - 将内存大小和数据大小设置为 0。
 */
void ec_sdo_request_clear_data(
    ec_sdo_request_t *req /**< SDO 请求。 */
)
{
    if (req->data)
    {
        kfree(req->data);
        req->data = NULL;
    }

    req->mem_size = 0;
    req->data_size = 0;
}

/*****************************************************************************/

/**
 * @brief 预分配数据内存。
 * @param req SDO 请求的指针。
 * @param size 要分配的数据大小。
 * @return 返回操作结果的整数值。
 * @details
 * - 如果内存大小已经大于等于指定大小，不执行任何操作。
 * - 清除 SDO 请求的数据。
 * - 如果分配内存失败，输出错误信息并返回 -ENOMEM。
 * - 设置内存大小和数据大小。
 * - 返回 0 表示成功。
 */
int ec_sdo_request_alloc(
    ec_sdo_request_t *req, /**< SDO 请求。 */
    size_t size            /**< 要分配的数据大小。 */
)
{
    if (size <= req->mem_size)
        return 0;

    ec_sdo_request_clear_data(req);

    if (!(req->data = (uint8_t *)kmalloc(size, GFP_KERNEL)))
    {
        EC_ERR("无法分配 %zu 字节的 SDO 内存。\n", size);
        return -ENOMEM;
    }

    req->mem_size = size;
    req->data_size = 0;
    return 0;
}

/*****************************************************************************/

/**
 * @brief 从外部源复制 SDO 数据。
 * @param req SDO 请求的指针。
 * @param source 源数据的指针。
 * @param size 源数据的字节数。
 * @return 返回操作结果的整数值。
 * @details
 * - 如果内存大小不足，分配新的内存。
 * - 复制源数据到 SDO 请求的数据中。
 * - 设置数据大小。
 * - 返回 0 表示成功。
 */
int ec_sdo_request_copy_data(
    ec_sdo_request_t *req, /**< SDO 请求。 */
    const uint8_t *source, /**< 源数据。 */
    size_t size            /**< 源数据的字节数。 */
)
{
    int ret = ec_sdo_request_alloc(req, size);
    if (ret < 0)
        return ret;

    memcpy(req->data, source, size);
    req->data_size = size;
    return 0;
}

/*****************************************************************************/

/**
 * @brief 检查是否超时。
 * @param req SDO 请求的指针。
 * @return 如果超时返回非零值，否则返回零。
 */
int ec_sdo_request_timed_out(const ec_sdo_request_t *req /**< SDO 请求。 */)
{
    return req->issue_timeout && jiffies - req->jiffies_start > HZ * req->issue_timeout / 1000;
}

/*****************************************************************************
 * Application interface.
 ****************************************************************************/

/**
 * @brief 设置 SDO 请求的索引和子索引。
 * @param req SDO 请求的指针。
 * @param index 索引值。
 * @param subindex 子索引值。
 * @details
 * - 设置 SDO 请求的索引和子索引。
 * - 将 complete_access 设置为 0。
 */
void ecrt_sdo_request_index(ec_sdo_request_t *req, uint16_t index,
                            uint8_t subindex)
{
    req->index = index;
    req->subindex = subindex;
    req->complete_access = 0;
}

/*****************************************************************************/

/**
 * @brief 设置 SDO 请求的索引和完全访问标志。
 * @param req SDO 请求的指针。
 * @param index 索引值。
 * @details
 * - 设置 SDO 请求的索引。
 * - 将子索引设置为 0。
 * - 将 complete_access 设置为 1。
 */
void ecrt_sdo_request_index_complete(ec_sdo_request_t *req, uint16_t index)
{
    req->index = index;
    req->subindex = 0;
    req->complete_access = 1;
}

/*****************************************************************************/

/**
 * @brief 设置 SDO 请求的超时时间。
 * @param req SDO 请求的指针。
 * @param timeout 超时时间。
 */
void ecrt_sdo_request_timeout(ec_sdo_request_t *req, uint32_t timeout)
{
    req->issue_timeout = timeout;
}

/*****************************************************************************/

/**
 * @brief 获取 SDO 请求的数据指针。
 * @param req SDO 请求的指针。
 * @return 返回数据指针。
 */
uint8_t *ecrt_sdo_request_data(ec_sdo_request_t *req)
{
    return req->data;
}

/*****************************************************************************/

/**
 * @brief 获取 SDO 请求的数据大小。
 * @param req SDO 请求的指针。
 * @return 返回数据大小。
 */
size_t ecrt_sdo_request_data_size(const ec_sdo_request_t *req)
{
    return req->data_size;
}

/*****************************************************************************/

/**
 * @brief 获取 SDO 请求的状态。
 * @param req SDO 请求的指针。
 * @return 返回 SDO 请求的状态。
 */
ec_request_state_t ecrt_sdo_request_state(const ec_sdo_request_t *req)
{
    return ec_request_state_translation_table[req->state];
}

/*****************************************************************************/

/**
 * @brief 发起读取 SDO 请求。
 * @param req SDO 请求的指针。
 * @details
 * - 设置 SDO 请求的方向为输入。
 * - 设置 SDO 请求的状态为 QUEUED。
 * - 将错误码和中止码设置为初始值。
 * - 设置 SDO 请求的起始时间。
 */
void ecrt_sdo_request_read(ec_sdo_request_t *req)
{
    req->dir = EC_DIR_INPUT;
    req->state = EC_INT_REQUEST_QUEUED;
    req->errno = 0;
    req->abort_code = 0x00000000;
    req->jiffies_start = jiffies;
}

/*****************************************************************************/

/**
 * @brief 发起写入 SDO 请求。
 * @param req SDO 请求的指针。
 * @details
 * - 设置 SDO 请求的方向为输出。
 * - 设置 SDO 请求的状态为 QUEUED。
 * - 将错误码和中止码设置为初始值。
 * - 设置 SDO 请求的起始时间。
 */
void ecrt_sdo_request_write(ec_sdo_request_t *req)
{
    req->dir = EC_DIR_OUTPUT;
    req->state = EC_INT_REQUEST_QUEUED;
    req->errno = 0;
    req->abort_code = 0x00000000;
    req->jiffies_start = jiffies;
}

/*****************************************************************************/

/**
 * @brief 带大小的写入 SDO 请求。
 * @param req SDO 请求的指针。
 * @param size 要写入的数据大小。
 * @details
 * - 如果大小超过内存大小，输出错误信息并设置请求状态为 FAILURE。
 * - 设置请求的数据大小。
 * - 设置请求的方向为输出。
 * - 设置请求的状态为 QUEUED。
 * - 将错误码和中止码设置为初始值。
 * - 设置请求的起始时间。
 */
void ecrt_sdo_request_write_with_size(ec_sdo_request_t *req, size_t size)
{
    if (size > req->mem_size)
    {
        EC_ERR("请求将 %zu 字节写入大小为 %zu 字节的 SDO。\n", size, req->mem_size);
        req->state = EC_INT_REQUEST_FAILURE;
        return;
    }
    req->data_size = size;
    req->dir = EC_DIR_OUTPUT;
    req->state = EC_INT_REQUEST_QUEUED;
    req->errno = 0;
    req->abort_code = 0x00000000;
    req->jiffies_start = jiffies;
}

/*****************************************************************************/

/** \cond */

EXPORT_SYMBOL(ecrt_sdo_request_index);
EXPORT_SYMBOL(ecrt_sdo_request_index_complete);
EXPORT_SYMBOL(ecrt_sdo_request_timeout);
EXPORT_SYMBOL(ecrt_sdo_request_data);
EXPORT_SYMBOL(ecrt_sdo_request_data_size);
EXPORT_SYMBOL(ecrt_sdo_request_state);
EXPORT_SYMBOL(ecrt_sdo_request_read);
EXPORT_SYMBOL(ecrt_sdo_request_write);

/** \endcond */

/*****************************************************************************/

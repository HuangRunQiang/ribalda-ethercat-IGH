/******************************************************************************
 *
 *  $Id$
 *
 *  Copyright (C) 2019  Florian Pose, Ingenieurgemeinschaft IgH
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
 * EtherCAT Mailbox Gateway request functions.
 */

/*****************************************************************************/

#include <linux/module.h>
#include <linux/jiffies.h>
#include <linux/slab.h>

#include "mbox_gateway_request.h"

/*****************************************************************************/

/** Default timeout in ms to wait for Mbox Gateway responses.
 */
#define EC_MBG_REQUEST_RESPONSE_TIMEOUT 1000

/*****************************************************************************/

void ec_mbg_request_clear_data(ec_mbg_request_t *);

/*****************************************************************************/

/**
@brief 初始化Mbox Gateway请求。
@param req Mbox Gateway请求。
@return 无。
@details 初始化Mbox Gateway请求的各个字段。
*/
void ec_mbg_request_init(
    ec_mbg_request_t *req /**< Mbox Gateway请求。 */
)
{
    INIT_LIST_HEAD(&req->list);
    req->data = NULL;
    req->mem_size = 0;
    req->data_size = 0;
    req->response_timeout = EC_MBG_REQUEST_RESPONSE_TIMEOUT;
    req->state = EC_INT_REQUEST_INIT;
    req->jiffies_sent = 0U;
    req->error_code = 0x0000;
}

/*****************************************************************************/

/**
@brief 清除Mbox Gateway请求。
@param req Mbox Gateway请求。
@return 无。
@details 清除Mbox Gateway请求的数据字段。
*/
void ec_mbg_request_clear(
    ec_mbg_request_t *req /**< Mbox Gateway请求。 */
)
{
    ec_mbg_request_clear_data(req);
}

/*****************************************************************************/

/**
@brief 清除分配的内存。
@param req Mbox Gateway请求。
@return 无。
@details 清除Mbox Gateway请求中分配的内存。
*/
void ec_mbg_request_clear_data(
    ec_mbg_request_t *req /**< Mbox Gateway请求。 */
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
@brief 预分配数据内存。
@param req Mbox Gateway请求。
@param size 要分配的数据大小。
@return 成功返回0，否则返回-ENOMEM。
@details 如果mem_size已经大于等于size，则不进行任何操作。
*/
int ec_mbg_request_alloc(
    ec_mbg_request_t *req, /**< Mbox Gateway请求。 */
    size_t size            /**< 要分配的数据大小。 */
)
{
    if (size <= req->mem_size)
        return 0;

    ec_mbg_request_clear_data(req);

    if (!(req->data = (uint8_t *)kmalloc(size, GFP_KERNEL)))
    {
        EC_ERR("无法分配 %zu 字节的Mbox Gateway内存。\n", size);
        return -ENOMEM;
    }

    req->mem_size = size;
    req->data_size = 0;
    return 0;
}

/*****************************************************************************/

/**
@brief 从外部源复制Mbox Gateway数据。
@param req Mbox Gateway请求。
@param source 源数据。
@param size source中的字节数。
@return 成功返回0，否则返回错误代码。
@details 如果mem_size太小，会分配新的内存。
*/
int ec_mbg_request_copy_data(
    ec_mbg_request_t *req, /**< Mbox Gateway请求。 */
    const uint8_t *source, /**< 源数据。 */
    size_t size            /**< source中的字节数。 */
)
{
    int ret = ec_mbg_request_alloc(req, size);
    if (ret < 0)
        return ret;

    memcpy(req->data, source, size);
    req->data_size = size;
    return 0;
}

/*****************************************************************************/

/**
@brief 请求运行。
@param req Mbox Gateway请求。
@return 无。
@details 设置Mbox Gateway请求的状态为已排队。
*/
void ec_mbg_request_run(
    ec_mbg_request_t *req /**< Mbox Gateway请求。 */
)
{
    req->state = EC_INT_REQUEST_QUEUED;
    req->error_code = 0x0000;
}

/*****************************************************************************/

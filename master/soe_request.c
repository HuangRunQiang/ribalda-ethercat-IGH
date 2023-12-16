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
 * Sercos-over-EtherCAT request functions.
 */

/*****************************************************************************/

#include <linux/module.h>
#include <linux/jiffies.h>
#include <linux/slab.h>

#include "soe_request.h"

/*****************************************************************************/

/** 等待SoE响应的默认超时时间，以毫秒为单位。
 */
#define EC_SOE_REQUEST_RESPONSE_TIMEOUT 1000

/*****************************************************************************/

void ec_soe_request_clear_data(ec_soe_request_t *);

/*****************************************************************************/

/** SoE请求构造函数。
 * 
 * 初始化SoE请求对象。
 *
 * @param req SoE请求对象。
 */
void ec_soe_request_init(
    ec_soe_request_t *req /**< SoE请求对象。 */
)
{
    INIT_LIST_HEAD(&req->list);
    req->drive_no = 0x00;
    req->idn = 0x0000;
    req->al_state = EC_AL_STATE_INIT;
    req->data = NULL;
    req->mem_size = 0;
    req->data_size = 0;
    req->dir = EC_DIR_INVALID;
    req->state = EC_INT_REQUEST_INIT;
    req->jiffies_sent = 0U;
    req->error_code = 0x0000;
}

/*****************************************************************************/

/** SoE请求析构函数。
 * 
 * 清除SoE请求对象的数据。
 *
 * @param req SoE请求对象。
 */
void ec_soe_request_clear(
    ec_soe_request_t *req /**< SoE请求对象。 */
)
{
    ec_soe_request_clear_data(req);
}

/*****************************************************************************/

/** 复制另一个SoE请求。
 *
 * 从另一个SoE请求对象复制数据到当前SoE请求对象。
 * 
 * @param req 当前SoE请求对象。
 * @param other 要复制的另一个SoE请求对象。
 * @return 成功返回0，否则返回负数错误代码。
 */
int ec_soe_request_copy(
    ec_soe_request_t *req,        /**< 当前SoE请求对象。 */
    const ec_soe_request_t *other /**< 要复制的另一个SoE请求对象。 */
)
{
    req->drive_no = other->drive_no;
    req->idn = other->idn;
    req->al_state = other->al_state;
    return ec_soe_request_copy_data(req, other->data, other->data_size);
}

/*****************************************************************************/

/** 设置驱动号。
 *
 * @param req SoE请求对象。
 * @param drive_no 驱动号。
 */
void ec_soe_request_set_drive_no(
    ec_soe_request_t *req, /**< SoE请求对象。 */
    uint8_t drive_no       /** 驱动号。 */
)
{
    req->drive_no = drive_no;
}

/*****************************************************************************/

/** 设置IDN。
 *
 * @param req SoE请求对象。
 * @param idn IDN。
 */
void ec_soe_request_set_idn(
    ec_soe_request_t *req, /**< SoE请求对象。 */
    uint16_t idn           /** IDN。 */
)
{
    req->idn = idn;
}

/*****************************************************************************/

/** 释放分配的内存。
 *
 * 清除SoE请求对象中的数据内存。
 *
 * @param req SoE请求对象。
 */
void ec_soe_request_clear_data(
    ec_soe_request_t *req /**< SoE请求对象。 */
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

/** 预分配数据内存。
 *
 * 如果 \a mem_size 已经大于等于 \a size，则不执行任何操作。
 *
 * @param req SoE请求对象。
 * @param size 要分配的数据大小。
 * @return 成功返回0，否则返回 -ENOMEM。
 */
int ec_soe_request_alloc(
    ec_soe_request_t *req, /**< SoE请求对象。 */
    size_t size            /**< 要分配的数据大小。 */
)
{
    if (size <= req->mem_size)
        return 0;

    ec_soe_request_clear_data(req);

    if (!(req->data = (uint8_t *)kmalloc(size, GFP_KERNEL)))
    {
        EC_ERR("无法分配 %zu 字节的SoE内存。\n", size);
        return -ENOMEM;
    }

    req->mem_size = size;
    req->data_size = 0;
    return 0;
}

/*****************************************************************************/

/** 从外部源复制SoE数据。
 *
 * 如果 \a mem_size 太小，则分配新的内存。
 *
 * @param req SoE请求对象。
 * @param source 源数据。
 * @param size \a source 中的字节数。
 * @return 成功返回0，否则返回负数错误代码。
 */
int ec_soe_request_copy_data(
    ec_soe_request_t *req, /**< SoE请求对象。 */
    const uint8_t *source, /**< 源数据。 */
    size_t size            /**< \a source 中的字节数。 */
)
{
    int ret = ec_soe_request_alloc(req, size);
    if (ret < 0)
        return ret;

    memcpy(req->data, source, size);
    req->data_size = size;
    return 0;
}

/*****************************************************************************/

/** 从外部源追加SoE数据。
 *
 * 如果 \a mem_size 太小，则分配新的内存。
 *
 * @param req SoE请求对象。
 * @param source 源数据。
 * @param size \a source 中的字节数。
 * @return 成功返回0，否则返回负数错误代码。
 */
int ec_soe_request_append_data(
    ec_soe_request_t *req, /**< SoE请求对象。 */
    const uint8_t *source, /**< 源数据。 */
    size_t size            /**< \a source 中的字节数。 */
)
{
    if (req->data_size + size > req->mem_size)
    {
        size_t new_size = req->mem_size ? req->mem_size * 2 : size;
        uint8_t *new_data = (uint8_t *)kmalloc(new_size, GFP_KERNEL);
        if (!new_data)
        {
            EC_ERR("无法分配 %zu 字节的SoE内存。\n",
                   new_size);
            return -ENOMEM;
        }
        memcpy(new_data, req->data, req->data_size);
        kfree(req->data);
        req->data = new_data;
        req->mem_size = new_size;
    }

    memcpy(req->data + req->data_size, source, size);
    req->data_size += size;
    return 0;
}

/*****************************************************************************/

/** 请求读操作。
 * 
 * 设置SoE请求对象的方向为输入，将请求状态设置为已入队列，错误代码清零。
 *
 * @param req SoE请求对象。
 */
void ec_soe_request_read(
    ec_soe_request_t *req /**< SoE请求对象。 */
)
{
    req->dir = EC_DIR_INPUT;
    req->state = EC_INT_REQUEST_QUEUED;
    req->error_code = 0x0000;
}

/*****************************************************************************/

/** 请求写操作。
 * 
 * 设置SoE请求对象的方向为输出，将请求状态设置为已入队列，错误代码清零。
 *
 * @param req SoE请求对象。
 */
void ec_soe_request_write(
    ec_soe_request_t *req /**< SoE请求对象。 */
)
{
    req->dir = EC_DIR_OUTPUT;
    req->state = EC_INT_REQUEST_QUEUED;
    req->error_code = 0x0000;
}
/*****************************************************************************/

/******************************************************************************
 *
 *  $Id$
 *
 *  Copyright (C) 2008  Olav Zarges, imc Messsysteme GmbH
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
 * File-over-EtherCAT request functions.
 */

/*****************************************************************************/

#include <linux/module.h>
#include <linux/jiffies.h>
#include <linux/slab.h>

#include "foe_request.h"

/*****************************************************************************/

/** Default timeout in ms to wait for FoE transfer responses.
 */
#define EC_FOE_REQUEST_RESPONSE_TIMEOUT 3000

/*****************************************************************************/

void ec_foe_request_clear_data(ec_foe_request_t *);

/*****************************************************************************/

/** FoE请求构造函数。
 * 
 * @brief 初始化FoE请求。
 * @param req FoE请求。
 * @return 无返回值。
 * @details 该函数用于初始化FoE请求对象，并将其各成员变量进行初始化。
 *          - req: FoE请求对象。
 */
void ec_foe_request_init(
    ec_foe_request_t *req /**< FoE请求。 */
)
{
    INIT_LIST_HEAD(&req->list);
    req->buffer = NULL;
    req->file_name[0] = 0;
    req->password = 0;
    req->buffer_size = 0;
    req->data_size = 0;
    req->progress = 0;
    req->dir = EC_DIR_INVALID;
    req->issue_timeout = 0; // 无超时
    req->response_timeout = EC_FOE_REQUEST_RESPONSE_TIMEOUT;
    req->state = EC_INT_REQUEST_INIT;
    req->result = FOE_BUSY;
    req->error_code = 0x00000000;
}

/*****************************************************************************/

/** FoE请求析构函数。
 * 
 * @brief 清理FoE请求。
 * @param req FoE请求。
 * @return 无返回值。
 * @details 该函数用于清理FoE请求对象，并释放相关资源。
 *          - req: FoE请求对象。
 */
void ec_foe_request_clear(
    ec_foe_request_t *req /**< FoE请求。 */
)
{
    ec_foe_request_clear_data(req);
}

/*****************************************************************************/

/** FoE请求数据析构函数。
 * 
 * @brief 清理FoE请求数据。
 * @param req FoE请求。
 * @return 无返回值。
 * @details 该函数用于清理FoE请求对象的数据，并释放相关资源。
 *          - req: FoE请求对象。
 */
void ec_foe_request_clear_data(
    ec_foe_request_t *req /**< FoE请求。 */
)
{
    if (req->buffer)
    {
        kfree(req->buffer);
        req->buffer = NULL;
    }

    req->buffer_size = 0;
    req->data_size = 0;
}

/*****************************************************************************/

/** 预分配数据内存。
 * 
 * @brief 如果内部的buffer_size已经大于等于size，则不进行任何操作。
 * @param req FoE请求。
 * @param size 要分配的数据大小。
 * @return 成功返回0，否则返回负错误代码。
 * @details 该函数用于预分配FoE请求对象的数据内存空间。
 *          - req: FoE请求对象。
 *          - size: 要分配的数据大小。
 */
int ec_foe_request_alloc(
    ec_foe_request_t *req, /**< FoE请求。 */
    size_t size            /**< 要分配的数据大小。 */
)
{
    if (size <= req->buffer_size)
    {
        return 0;
    }

    ec_foe_request_clear_data(req);

    if (!(req->buffer = (uint8_t *)kmalloc(size, GFP_KERNEL)))
    {
        EC_ERR("Failed to allocate %zu bytes of FoE memory.\n", size);
        return -ENOMEM;
    }

    req->buffer_size = size;
    req->data_size = 0;
    return 0;
}

/*****************************************************************************/

/** 复制FoE数据从外部源。
 * 
 * @brief 如果buffer_size太小，则分配新的内存。
 * @param req FoE请求。
 * @param source 源数据。
 * @param size source中的字节数。
 * @return 成功返回0，否则返回负错误代码。
 * @details 该函数用于将外部源的FoE数据复制到FoE请求对象中。
 *          - req: FoE请求对象。
 *          - source: 源数据。
 *          - size: source中的字节数。
 */
int ec_foe_request_copy_data(
    ec_foe_request_t *req, /**< FoE请求。 */
    const uint8_t *source, /**< 源数据。 */
    size_t size            /**< source中的字节数。 */
)
{
    int ret;

    ret = ec_foe_request_alloc(req, size);
    if (ret)
    {
        return ret;
    }

    memcpy(req->buffer, source, size);
    req->data_size = size;
    return 0;
}

/*****************************************************************************/

/** 检查是否超时。
 * 
 * @brief 检查是否超时。
 * @param req FoE请求。
 * @return 超时返回非零，否则返回零。
 * @details 该函数用于检查FoE请求是否超时。
 *          - req: FoE请求对象。
 */
int ec_foe_request_timed_out(
    const ec_foe_request_t *req /**< FoE请求。 */
)
{
    return req->issue_timeout && jiffies - req->jiffies_start > HZ * req->issue_timeout / 1000;
}

/*****************************************************************************
 * Application interface.
 ****************************************************************************/

/** 设置请求超时时间。
 * 
 * @brief 设置请求的超时时间。
 * @param req FoE请求。
 * @param timeout 超时时间（毫秒）。
 * @return 无返回值。
 * @details 该函数用于设置FoE请求的超时时间。
 *          - req: FoE请求对象。
 *          - timeout: 超时时间（毫秒）。
 */
void ecrt_foe_request_timeout(
    ec_foe_request_t *req, /**< FoE请求。 */
    uint32_t timeout       /**< 超时时间（毫秒）。 */
)
{
    req->issue_timeout = timeout;
}

/*****************************************************************************/

/** 为请求选择新的文件。
 * 
 * @brief 为请求选择新的文件。
 * @param req FoE请求。
 * @param file_name 文件名。
 * @param password 密码。
 * @return 无返回值。
 * @details 该函数用于为FoE请求选择新的文件。
 *          - req: FoE请求对象。
 *          - file_name: 文件名。
 *          - password: 密码。
 */
void ecrt_foe_request_file(
    ec_foe_request_t *req, /**< FoE请求。 */
    const char *file_name, /** 文件名 */
    uint32_t password      /** 密码 */
)
{
    strlcpy((char *)req->file_name, file_name, sizeof(req->file_name));
    req->password = password;
}

/*****************************************************************************/

/** 返回请求数据的指针。
 * 
 * @brief 返回请求数据的指针。
 * @param req FoE请求。
 * @return 数据指针。
 * @details 该函数用于返回FoE请求数据的指针。
 *          - req: FoE请求对象。
 */
uint8_t *ecrt_foe_request_data(
    ec_foe_request_t *req /**< FoE请求。 */
)
{
    return req->buffer;
}

/*****************************************************************************/

/** 返回数据的大小。
 * 
 * @brief 返回数据的大小。
 * @param req FoE请求。
 * @return 数据大小。
 * @details 该函数用于返回FoE请求数据的大小。
 *          - req: FoE请求对象。
 */
size_t ecrt_foe_request_data_size(
    const ec_foe_request_t *req /**< FoE请求。 */
)
{
    return req->data_size;
}

/*****************************************************************************/

/** 返回请求的状态。
 * 
 * @brief 返回请求的状态。
 * @param req FoE请求。
 * @return 请求状态。
 * @details 该函数用于返回FoE请求的状态。
 *          - req: FoE请求对象。
 */
ec_request_state_t ecrt_foe_request_state(const ec_foe_request_t *req)
{
    return ec_request_state_translation_table[req->state];
}

/*****************************************************************************/

/** 返回请求的结果。
 * 
 * @brief 返回请求的结果。
 * @param req FoE请求。
 * @return 请求结果。
 * @details 该函数用于返回FoE请求的结果。
 *          - req: FoE请求对象。
 */
ec_foe_error_t ecrt_foe_request_result(const ec_foe_request_t *req)
{
    return req->result;
}

/*****************************************************************************/

/** 返回错误代码。
 * 
 * @brief 返回错误代码。
 * @param req FoE请求。
 * @return 错误代码。
 * @details 该函数用于返回FoE请求的错误代码。
 *          - req: FoE请求对象。
 */
uint32_t ecrt_foe_request_error_code(const ec_foe_request_t *req)
{
    return req->error_code;
}

/*****************************************************************************/

/** 返回当前@EC_REQUEST_BUSY传输的进度。
 * 
 * @brief 返回当前@EC_REQUEST_BUSY传输的进度。
 * @param req FoE请求。
 * @return 进度（字节）。
 * @details 该函数用于返回当前@EC_REQUEST_BUSY传输的进度。
 *          - req: FoE请求对象。
 */
size_t ecrt_foe_request_progress(
    const ec_foe_request_t *req /**< FoE请求。 */
)
{
    return req->progress;
}

/*****************************************************************************/

/** 准备读取请求（从从站到主站）。
 * 
 * @brief 准备读取请求（从从站到主站）。
 * @param req FoE请求。
 * @return 无返回值。
 * @details 该函数用于准备读取请求（从从站到主站）。
 *          - req: FoE请求对象。
 */
void ecrt_foe_request_read(
    ec_foe_request_t *req /**< FoE请求。 */
)
{
    req->data_size = 0;
    req->progress = 0;
    req->dir = EC_DIR_INPUT;
    req->state = EC_INT_REQUEST_QUEUED;
    req->result = FOE_BUSY;
    req->jiffies_start = jiffies;
}

/*****************************************************************************/

/** 准备写入请求（从主站到从站）。
 * 
 * @brief 准备写入请求（从主站到从站）。
 * @param req FoE请求。
 * @param data_size 数据大小。
 * @return 无返回值。
 * @details 该函数用于准备写入请求（从主站到从站）。
 *          - req: FoE请求对象。
 *          - data_size: 数据大小。
 */
void ecrt_foe_request_write(
    ec_foe_request_t *req, /**< FoE请求。 */
    size_t data_size       /**< 数据大小。 */
)
{
    if (data_size > req->buffer_size)
    {
        EC_ERR("Request to write %zu bytes to FoE buffer of size %zu.\n",
               data_size, req->buffer_size);
        req->state = EC_INT_REQUEST_FAILURE;
        return;
    }
    req->data_size = data_size;
    req->progress = 0;
    req->dir = EC_DIR_OUTPUT;
    req->state = EC_INT_REQUEST_QUEUED;
    req->result = FOE_BUSY;
    req->jiffies_start = jiffies;
}

/*****************************************************************************/

/** \cond */

EXPORT_SYMBOL(ecrt_foe_request_file);
EXPORT_SYMBOL(ecrt_foe_request_timeout);
EXPORT_SYMBOL(ecrt_foe_request_data);
EXPORT_SYMBOL(ecrt_foe_request_data_size);
EXPORT_SYMBOL(ecrt_foe_request_state);
EXPORT_SYMBOL(ecrt_foe_request_result);
EXPORT_SYMBOL(ecrt_foe_request_error_code);
EXPORT_SYMBOL(ecrt_foe_request_progress);
EXPORT_SYMBOL(ecrt_foe_request_read);
EXPORT_SYMBOL(ecrt_foe_request_write);

/** \endcond */

/*****************************************************************************/

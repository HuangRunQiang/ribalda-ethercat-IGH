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
 * 基于EtherCAT的供应商特定协议处理函数。
 */

/*****************************************************************************/

#include <linux/module.h>

#include "master.h"
#include "slave_config.h"
#include "mailbox.h"
#include "voe_handler.h"

/** VoE头部大小。
 */
#define EC_VOE_HEADER_SIZE 6

/** VoE响应超时时间（毫秒）。
 */
#define EC_VOE_RESPONSE_TIMEOUT 500

/*****************************************************************************/

void ec_voe_handler_state_write_start(ec_voe_handler_t *);
void ec_voe_handler_state_write_response(ec_voe_handler_t *);

void ec_voe_handler_state_read_start(ec_voe_handler_t *);
void ec_voe_handler_state_read_check(ec_voe_handler_t *);
void ec_voe_handler_state_read_response(ec_voe_handler_t *);
void ec_voe_handler_state_read_response_data(ec_voe_handler_t *);

void ec_voe_handler_state_read_nosync_start(ec_voe_handler_t *);
void ec_voe_handler_state_read_nosync_response(ec_voe_handler_t *);

void ec_voe_handler_state_end(ec_voe_handler_t *);
void ec_voe_handler_state_error(ec_voe_handler_t *);

/*****************************************************************************/

/**
@brief VoE处理器构造函数。
@return 返回ec_datagram_prealloc()的返回值。
*/
int ec_voe_handler_init(
    ec_voe_handler_t *voe, /**< VoE处理器。 */
    ec_slave_config_t *sc, /**< 父从站配置。 */
    size_t size            /**< 预留内存的大小。 */
)
{
    voe->config = sc;
    voe->vendor_id = 0x00000000;
    voe->vendor_type = 0x0000;
    voe->data_size = 0;
    voe->dir = EC_DIR_INVALID;
    voe->state = ec_voe_handler_state_error;
    voe->request_state = EC_INT_REQUEST_INIT;

    ec_datagram_init(&voe->datagram);
    return ec_datagram_prealloc(&voe->datagram,
                                size + EC_MBOX_HEADER_SIZE + EC_VOE_HEADER_SIZE);
}

/*****************************************************************************/

/** VoE处理器析构函数。
 */
void ec_voe_handler_clear(
    ec_voe_handler_t *voe /**< VoE处理器。 */
)
{
    ec_datagram_clear(&voe->datagram);
}

/*****************************************************************************/

/** 获取可用内存大小。
 *
 * \return 内存大小。
 */
size_t ec_voe_handler_mem_size(
    const ec_voe_handler_t *voe /**< VoE处理器。 */
)
{
    if (voe->datagram.mem_size >= EC_MBOX_HEADER_SIZE + EC_VOE_HEADER_SIZE)
        return voe->datagram.mem_size -
               (EC_MBOX_HEADER_SIZE + EC_VOE_HEADER_SIZE);
    else
        return 0;
}
/*****************************************************************************
 * Application interface.
 ****************************************************************************/

/**
@brief VoE处理器发送头部。
@param voe VoE处理器。
@param vendor_id 供应商ID。
@param vendor_type 供应商类型。
*/
void ecrt_voe_handler_send_header(ec_voe_handler_t *voe, uint32_t vendor_id,
                                  uint16_t vendor_type)
{
    voe->vendor_id = vendor_id;
    voe->vendor_type = vendor_type;
}

/*****************************************************************************/

/**
@brief VoE处理器接收到头部。
@param voe VoE处理器。
@param vendor_id 供应商ID。
@param vendor_type 供应商类型。
*/
void ecrt_voe_handler_received_header(const ec_voe_handler_t *voe,
                                      uint32_t *vendor_id, uint16_t *vendor_type)
{
    uint8_t *header = voe->datagram.data + EC_MBOX_HEADER_SIZE;

    if (vendor_id)
        *vendor_id = EC_READ_U32(header);
    if (vendor_type)
        *vendor_type = EC_READ_U16(header + 4);
}

/*****************************************************************************/

/**
@brief 获取VoE处理器的数据指针。
@param voe VoE处理器。
@return 数据指针。
*/
uint8_t *ecrt_voe_handler_data(ec_voe_handler_t *voe)
{
    return voe->datagram.data + EC_MBOX_HEADER_SIZE + EC_VOE_HEADER_SIZE;
}

/*****************************************************************************/

/**
@brief 获取VoE处理器的数据大小。
@param voe VoE处理器。
@return 数据大小。
*/
size_t ecrt_voe_handler_data_size(const ec_voe_handler_t *voe)
{
    return voe->data_size;
}

/*****************************************************************************/

/**
@brief 执行VoE处理器的读操作。
@param voe VoE处理器。
*/
void ecrt_voe_handler_read(ec_voe_handler_t *voe)
{
    voe->dir = EC_DIR_INPUT;
    voe->state = ec_voe_handler_state_read_start;
    voe->request_state = EC_INT_REQUEST_BUSY;
}

/*****************************************************************************/

/**
@brief 执行VoE处理器的非同步读操作。
@param voe VoE处理器。
*/
void ecrt_voe_handler_read_nosync(ec_voe_handler_t *voe)
{
    voe->dir = EC_DIR_INPUT;
    voe->state = ec_voe_handler_state_read_nosync_start;
    voe->request_state = EC_INT_REQUEST_BUSY;
}

/*****************************************************************************/

/**
@brief 执行VoE处理器的写操作。
@param voe VoE处理器。
@param size 数据大小。
*/
void ecrt_voe_handler_write(ec_voe_handler_t *voe, size_t size)
{
    voe->dir = EC_DIR_OUTPUT;
    voe->data_size = size;
    voe->state = ec_voe_handler_state_write_start;
    voe->request_state = EC_INT_REQUEST_BUSY;
}

/*****************************************************************************/

/**
@brief 执行VoE处理器的操作。
@param voe VoE处理器。
@return 请求状态。
*/
ec_request_state_t ecrt_voe_handler_execute(ec_voe_handler_t *voe)
{
    if (voe->config->slave)
    { // FIXME locking?
        voe->state(voe);
        if (voe->request_state == EC_INT_REQUEST_BUSY)
        {
            ec_master_queue_datagram(voe->config->master, &voe->datagram);
        }
    }
    else
    {
        voe->state = ec_voe_handler_state_error;
        voe->request_state = EC_INT_REQUEST_FAILURE;
    }

    return ec_request_state_translation_table[voe->request_state];
}

/******************************************************************************
 * State functions.
 *****************************************************************************/

/**
@brief 开始写入VoE数据。
@param voe VoE处理器。
@return 无。
@details
- 获取从站对象和数据指针。
- 如果调试级别大于0，则打印写入VoE数据的字节数和数据内容。
- 检查从站的SII数据是否可用，如果不可用，则设置状态为错误并返回。
- 检查从站是否支持VoE协议，如果不支持，则设置状态为错误并返回。
- 准备发送VoE数据报，如果失败，则设置状态为错误并返回。
- 将厂商ID和厂商类型写入数据报头部。
- 设置重试次数和起始时间。
- 设置状态为写入响应状态。
*/
void ec_voe_handler_state_write_start(ec_voe_handler_t *voe)
{
    ec_slave_t *slave = voe->config->slave;
    uint8_t *data;

    if (slave->master->debug_level)
    {
        EC_SLAVE_DBG(slave, 0, "正在写入 %zu 字节的VoE数据。\n",
                     voe->data_size);
        ec_print_data(ecrt_voe_handler_data(voe), voe->data_size);
    }

    if (!slave->sii_image)
    {
        EC_SLAVE_ERR(slave, "从站无法处理VoE写入请求。SII数据不可用。\n");
        voe->state = ec_voe_handler_state_error;
        voe->request_state = EC_INT_REQUEST_FAILURE;
        return;
    }

    if (!(slave->sii_image->sii.mailbox_protocols & EC_MBOX_VOE))
    {
        EC_SLAVE_ERR(slave, "从站不支持VoE！\n");
        voe->state = ec_voe_handler_state_error;
        voe->request_state = EC_INT_REQUEST_FAILURE;
        return;
    }

    data = ec_slave_mbox_prepare_send(slave, &voe->datagram,
                                      EC_MBOX_TYPE_VOE, EC_VOE_HEADER_SIZE + voe->data_size);
    if (IS_ERR(data))
    {
        voe->state = ec_voe_handler_state_error;
        voe->request_state = EC_INT_REQUEST_FAILURE;
        return;
    }

    EC_WRITE_U32(data, voe->vendor_id);
    EC_WRITE_U16(data + 4, voe->vendor_type);
    /* data already in datagram */

    voe->retries = EC_FSM_RETRIES;
    voe->jiffies_start = jiffies;
    voe->state = ec_voe_handler_state_write_response;
}

/*****************************************************************************/

/**
@brief 等待邮箱响应。
@param voe VoE处理器。
@return 无。
@details
- 获取数据报和从站对象。
- 如果数据报状态为超时并且重试次数大于0，则返回。
- 如果数据报状态不是接收到，则设置状态为错误，并打印接收VoE写入请求数据报的状态。
- 如果数据报的工作计数器不等于1，则进行以下判断：
  - 如果工作计数器为0，则计算时间差，如果小于VoE响应超时时间，则打印从站未响应VoE写入请求的消息，并返回。
- 设置状态为错误，并打印接收VoE写入请求失败的消息。
- 设置VoE请求状态为失败。
- 设置状态为结束状态。
*/
void ec_voe_handler_state_write_response(ec_voe_handler_t *voe)
{
    ec_datagram_t *datagram = &voe->datagram;
    ec_slave_t *slave = voe->config->slave;

    if (datagram->state == EC_DATAGRAM_TIMED_OUT && voe->retries--)
        return;

    if (datagram->state != EC_DATAGRAM_RECEIVED)
    {
        voe->state = ec_voe_handler_state_error;
        voe->request_state = EC_INT_REQUEST_FAILURE;
        EC_SLAVE_ERR(slave, "接收VoE写入请求数据报失败: ");
        ec_datagram_print_state(datagram);
        return;
    }

    if (datagram->working_counter != 1)
    {
        if (!datagram->working_counter)
        {
            unsigned long diff_ms =
                (jiffies - voe->jiffies_start) * 1000 / HZ;
            if (diff_ms < EC_VOE_RESPONSE_TIMEOUT)
            {
                EC_SLAVE_DBG(slave, 1, "从站未响应VoE写入请求。%lu毫秒后重试...\n",
                             diff_ms);
                // 没有响应；再次发送请求数据报
                return;
            }
        }
        voe->state = ec_voe_handler_state_error;
        voe->request_state = EC_INT_REQUEST_FAILURE;
        EC_SLAVE_ERR(slave, "接收VoE写入请求失败: ");
        ec_datagram_print_wc_error(datagram);
        return;
    }

    EC_CONFIG_DBG(voe->config, 1, "VoE写入请求成功。\n");

    voe->request_state = EC_INT_REQUEST_SUCCESS;
    voe->state = ec_voe_handler_state_end;
}

/*****************************************************************************/

/**
@brief 开始读取VoE数据。
@param voe VoE处理器。
@return 无。
@details
- 获取数据报、从站对象和主站对象。
- 打印读取VoE数据的消息。
- 检查从站的SII数据是否可用，如果不可用，则设置状态为错误并返回。
- 检查从站是否支持VoE协议，如果不支持，则设置状态为错误并返回。
- 设置起始时间。
- 如果邮箱读取锁定，则设置状态为读取响应数据，并将数据报状态设置为无效。
- 否则，准备邮箱检查，设置重试次数，并设置状态为读取检查状态。
*/
void ec_voe_handler_state_read_start(ec_voe_handler_t *voe)
{
    ec_datagram_t *datagram = &voe->datagram;
    ec_slave_t *slave = voe->config->slave;

    EC_SLAVE_DBG(slave, 1, "读取VoE数据。\n");

    if (!slave->sii_image)
    {
        EC_SLAVE_ERR(slave, "从站无法处理VoE请求。\n");
        voe->state = ec_voe_handler_state_error;
        voe->request_state = EC_INT_REQUEST_FAILURE;
        return;
    }

    if (!(slave->sii_image->sii.mailbox_protocols & EC_MBOX_VOE))
    {
        EC_SLAVE_ERR(slave, "从站不支持VoE！\n");
        voe->state = ec_voe_handler_state_error;
        voe->request_state = EC_INT_REQUEST_FAILURE;
        return;
    }

    voe->jiffies_start = jiffies;

    // 如果已经有读取请求正在进行中，则跳过邮箱读取检查
    if (ec_read_mbox_locked(slave))
    {
        voe->state = ec_voe_handler_state_read_response_data;
        // 数据报不被使用，标记为无效
        datagram->state = EC_DATAGRAM_INVALID;
    }
    else
    {
        ec_slave_mbox_prepare_check(slave, datagram); // 不会失败。
        voe->jiffies_start = jiffies;
        voe->retries = EC_FSM_RETRIES;
        voe->state = ec_voe_handler_state_read_check;
    }
}

/*****************************************************************************/

/**
@brief 检查邮箱中是否有新的数据。
@param voe VoE处理器。
@return 无。
@details
- 获取数据报、从站对象和主站对象。
- 如果数据报状态为超时并且重试次数大于0，则返回。
- 如果数据报状态不是接收到，则设置状态为错误，并打印接收VoE邮箱检查数据报的状态。
- 如果数据报的工作计数器不等于1，则进行以下判断：
  - 如果工作计数器为0，则计算时间差，如果小于VoE响应超时时间，则打印从站未响应VoE邮箱检查的消息，并返回。
- 设置状态为错误，并打印接收VoE邮箱检查数据报失败的消息。
- 设置VoE请求状态为失败。
- 设置状态为结束状态。
*/
void ec_voe_handler_state_read_check(ec_voe_handler_t *voe)
{
    ec_datagram_t *datagram = &voe->datagram;
    ec_slave_t *slave = voe->config->slave;

    if (datagram->state == EC_DATAGRAM_TIMED_OUT && voe->retries--)
        return;

    if (datagram->state != EC_DATAGRAM_RECEIVED)
    {
        voe->state = ec_voe_handler_state_error;
        ec_read_mbox_lock_clear(slave);
        voe->request_state = EC_INT_REQUEST_FAILURE;
        EC_SLAVE_ERR(slave, "接收VoE邮箱检查数据报失败: ");
        ec_datagram_print_state(datagram);
        return;
    }

    if (datagram->working_counter != 1)
    {
        voe->state = ec_voe_handler_state_error;
        ec_read_mbox_lock_clear(slave);
        voe->request_state = EC_INT_REQUEST_FAILURE;
        EC_SLAVE_ERR(slave, "接收VoE邮箱检查数据报失败: ");
        ec_datagram_print_wc_error(datagram);
        return;
    }

    if (!ec_slave_mbox_check(datagram))
    {
        unsigned long diff_ms = 0;

        // 检查数据是否已经被其他读取请求读取
        if (slave->mbox_voe_data.payload_size > 0)
        {
            ec_read_mbox_lock_clear(slave);
            voe->state = ec_voe_handler_state_read_response_data;
            voe->state(voe);
            return;
        }

        diff_ms = (datagram->jiffies_received - voe->jiffies_start) * 1000 / HZ;

        if (diff_ms >= EC_VOE_RESPONSE_TIMEOUT)
        {
            voe->state = ec_voe_handler_state_error;
            ec_read_mbox_lock_clear(slave);
            voe->request_state = EC_INT_REQUEST_FAILURE;
            EC_SLAVE_ERR(slave, "等待VoE数据超时。\n");
            return;
        }

        ec_slave_mbox_prepare_check(slave, datagram); // 不会失败。
        voe->retries = EC_FSM_RETRIES;
        return;
    }

    // 获取响应
    ec_slave_mbox_prepare_fetch(slave, datagram); // 不会失败。
    voe->retries = EC_FSM_RETRIES;
    voe->state = ec_voe_handler_state_read_response;
}

/*****************************************************************************/

/**
@brief 读取待处理的邮箱数据。
@param voe VoE处理器。
@return 无。
@details
- 获取数据报、从站对象和主站对象。
- 如果数据报状态为超时并且重试次数大于0，则返回。
- 如果数据报状态不是接收到，则设置状态为错误，并打印接收VoE读取数据报的状态。
- 如果数据报的工作计数器不等于1，则进行以下判断：
  - 只有在数据已经被其他读取请求读取时，才会出现错误。
- 清除邮箱读取锁定。
- 设置状态为读取响应数据。
- 调用读取响应数据的函数。
*/
void ec_voe_handler_state_read_response(ec_voe_handler_t *voe)
{
    ec_datagram_t *datagram = &voe->datagram;
    ec_slave_t *slave = voe->config->slave;

    if (datagram->state == EC_DATAGRAM_TIMED_OUT && voe->retries--)
        return;

    if (datagram->state != EC_DATAGRAM_RECEIVED)
    {
        voe->state = ec_voe_handler_state_error;
        ec_read_mbox_lock_clear(slave);
        voe->request_state = EC_INT_REQUEST_FAILURE;
        EC_SLAVE_ERR(slave, "接收VoE读取数据报失败: ");
        ec_datagram_print_state(datagram);
        return;
    }

    if (datagram->working_counter != 1)
    {
        // 只有在数据没有被其他读取请求读取时才会出现错误
        if (slave->mbox_voe_data.payload_size == 0)
        {
            voe->state = ec_voe_handler_state_error;
            ec_read_mbox_lock_clear(slave);
            voe->request_state = EC_INT_REQUEST_FAILURE;
            EC_SLAVE_ERR(slave, "接收VoE读取响应失败: ");
            ec_datagram_print_wc_error(datagram);
            return;
        }
    }
    ec_read_mbox_lock_clear(slave);
    voe->state = ec_voe_handler_state_read_response_data;
    voe->state(voe);
}

/*****************************************************************************/

/**
@brief 读取响应数据。
@param voe VoE处理器。
@return 无。
@details
- 获取数据报、从站对象和主站对象。
- 如果数据报状态为超时并且重试次数大于0，则返回。
- 如果数据报状态不是接收到，则设置状态为错误，并打印接收VoE读取响应数据报的状态。
- 如果数据报的工作计数器不等于1，则进行以下判断：
  - 只有在数据已经被其他读取请求读取时，才会出现错误。
- 清除邮箱读取锁定。
- 设置状态为读取响应数据。
- 调用读取响应数据的函数。
*/
void ec_voe_handler_state_read_response_data(ec_voe_handler_t *voe)
{
    ec_datagram_t *datagram = &voe->datagram;
    ec_slave_t *slave = voe->config->slave;
    ec_master_t *master = voe->config->master;
    uint8_t *data, mbox_prot;
    size_t rec_size;

    // 处理可用的数据或启动新的邮箱读取检查
    if (slave->mbox_voe_data.payload_size > 0)
    {
        slave->mbox_voe_data.payload_size = 0;
    }
    else
    {
        // 如果所需数据不可用，则启动新的邮箱读取检查
        if (ec_read_mbox_locked(slave))
        {
            // 等待当前读取请求，并将数据报标记为无效
            datagram->state = EC_DATAGRAM_INVALID;
        }
        else
        {
            ec_slave_mbox_prepare_check(slave, datagram); // 不会失败。
            voe->state = ec_voe_handler_state_read_check;
        }
        return;
    }

    data = ec_slave_mbox_fetch(slave, &slave->mbox_voe_data, &mbox_prot, &rec_size);
    if (IS_ERR(data))
    {
        voe->state = ec_voe_handler_state_error;
        voe->request_state = EC_INT_REQUEST_FAILURE;
        return;
    }

    if (mbox_prot != EC_MBOX_TYPE_VOE)
    {
        voe->state = ec_voe_handler_state_error;
        voe->request_state = EC_INT_REQUEST_FAILURE;
        EC_SLAVE_WARN(slave, "接收到0x%02X的邮箱协议作为响应。\n",
                      mbox_prot);
        ec_print_data(data, rec_size);
        return;
    }

    if (rec_size < EC_VOE_HEADER_SIZE)
    {
        voe->state = ec_voe_handler_state_error;
        voe->request_state = EC_INT_REQUEST_FAILURE;
        EC_SLAVE_ERR(slave, "接收到不完整的VoE头部（%zu字节）！\n",
                     rec_size);
        return;
    }

    if (master->debug_level)
    {
        EC_CONFIG_DBG(voe->config, 0, "VoE数据：\n");
        ec_print_data(data, rec_size);
    }

    voe->data_size = rec_size - EC_VOE_HEADER_SIZE;
    memcpy(voe->datagram.data + EC_MBOX_HEADER_SIZE, data, rec_size);
    voe->request_state = EC_INT_REQUEST_SUCCESS;
    voe->state = ec_voe_handler_state_end; // 成功
}

/*****************************************************************************/

/**
@brief 开始无同步消息读取VoE数据。
@param voe VoE处理器。
@return 无。
@details
- 获取数据报和从站对象。
- 打印读取VoE数据的消息。
- 检查从站的SII数据是否可用，如果不可用，则设置状态为错误并返回。
- 检查从站是否支持VoE协议，如果不支持，则设置状态为错误并返回。
- 准备邮箱读取，不会失败。
- 设置起始时间。
- 设置重试次数。
- 设置状态为读取无同步响应。
*/
void ec_voe_handler_state_read_nosync_start(ec_voe_handler_t *voe)
{
    ec_datagram_t *datagram = &voe->datagram;
    ec_slave_t *slave = voe->config->slave;

    EC_SLAVE_DBG(slave, 1, "正在读取VoE数据。\n");

    if (!slave->sii_image)
    {
        EC_SLAVE_ERR(slave, "从站无法处理VoE读取请求。SII数据不可用。\n");
        voe->state = ec_voe_handler_state_error;
        voe->request_state = EC_INT_REQUEST_FAILURE;
        return;
    }

    if (!(slave->sii_image->sii.mailbox_protocols & EC_MBOX_VOE))
    {
        EC_SLAVE_ERR(slave, "从站不支持VoE！\n");
        voe->state = ec_voe_handler_state_error;
        voe->request_state = EC_INT_REQUEST_FAILURE;
        return;
    }

    ec_slave_mbox_prepare_fetch(slave, datagram); // 不会失败。

    voe->jiffies_start = jiffies;
    voe->retries = EC_FSM_RETRIES;
    voe->state = ec_voe_handler_state_read_nosync_response;
}

/*****************************************************************************/

/**
@brief 读取无同步响应数据。
@param voe VoE处理器。
@return 无。
@details
- 获取数据报、从站对象和主站对象。
- 如果数据报状态为超时并且重试次数大于0，则返回。
- 如果数据报状态不是接收到，则设置状态为错误，并打印接收VoE读取无同步响应数据报的状态。
- 如果数据报的工作计数器为0，则设置状态为错误，并打印从站未发送VoE数据的消息。
- 如果数据报的工作计数器不等于1，则进行以下判断：
  - 只有在数据已经被其他读取请求读取时，才会出现错误。
- 清除邮箱读取锁定。
- 设置状态为读取无同步响应数据。
- 调用读取无同步响应数据的函数。
*/
void ec_voe_handler_state_read_nosync_response(ec_voe_handler_t *voe)
{
    ec_datagram_t *datagram = &voe->datagram;
    ec_slave_t *slave = voe->config->slave;
    ec_master_t *master = voe->config->master;
    uint8_t *data, mbox_prot;
    size_t rec_size;

    if (datagram->state == EC_DATAGRAM_TIMED_OUT && voe->retries--)
        return;

    if (datagram->state != EC_DATAGRAM_RECEIVED)
    {
        voe->state = ec_voe_handler_state_error;
        voe->request_state = EC_INT_REQUEST_FAILURE;
        EC_SLAVE_ERR(slave, "接收VoE读取数据报失败: ");
        ec_datagram_print_state(datagram);
        return;
    }

    if (datagram->working_counter == 0)
    {
        voe->state = ec_voe_handler_state_error;
        voe->request_state = EC_INT_REQUEST_FAILURE;
        EC_SLAVE_DBG(slave, 1, "从站未发送VoE数据。\n");
        return;
    }

    if (datagram->working_counter != 1)
    {
        voe->state = ec_voe_handler_state_error;
        voe->request_state = EC_INT_REQUEST_FAILURE;
        EC_SLAVE_WARN(slave, "接收VoE读取响应失败: ");
        ec_datagram_print_wc_error(datagram);
        return;
    }

    if (slave->mbox_voe_data.payload_size > 0)
    {
        slave->mbox_voe_data.payload_size = 0;
        data = ec_slave_mbox_fetch(slave, &slave->mbox_voe_data, &mbox_prot, &rec_size);
    }
    else
    {
        voe->state = ec_voe_handler_state_error;
        voe->request_state = EC_INT_REQUEST_FAILURE;
        return;
    }

    if (mbox_prot != EC_MBOX_TYPE_VOE)
    {
        voe->state = ec_voe_handler_state_error;
        voe->request_state = EC_INT_REQUEST_FAILURE;
        EC_SLAVE_WARN(slave, "接收到0x%02X的邮箱协议作为响应。\n",
                      mbox_prot);
        ec_print_data(data, rec_size);
        return;
    }

    if (rec_size < EC_VOE_HEADER_SIZE)
    {
        voe->state = ec_voe_handler_state_error;
        voe->request_state = EC_INT_REQUEST_FAILURE;
        EC_SLAVE_ERR(slave, "接收到不完整的VoE头部（%zu字节）！\n",
                     rec_size);
        return;
    }

    if (master->debug_level)
    {
        EC_CONFIG_DBG(voe->config, 1, "VoE数据：\n");
        ec_print_data(data, rec_size);
    }

    voe->data_size = rec_size - EC_VOE_HEADER_SIZE;
    memcpy(voe->datagram.data + EC_MBOX_HEADER_SIZE, data, rec_size);
    voe->request_state = EC_INT_REQUEST_SUCCESS;
    voe->state = ec_voe_handler_state_end; // 成功
}

/*****************************************************************************/

/**
@brief 结束状态函数。
@param voe VoE处理器。
@return 无。
@details 无。
*/
void ec_voe_handler_state_end(ec_voe_handler_t *voe)
{
}

/*****************************************************************************/

/**
@brief 错误状态函数。
@param voe VoE处理器。
@return 无。
@details 无。
*/
void ec_voe_handler_state_error(ec_voe_handler_t *voe)
{
}

/*****************************************************************************/

/** \cond */

EXPORT_SYMBOL(ecrt_voe_handler_send_header);
EXPORT_SYMBOL(ecrt_voe_handler_received_header);
EXPORT_SYMBOL(ecrt_voe_handler_data);
EXPORT_SYMBOL(ecrt_voe_handler_data_size);
EXPORT_SYMBOL(ecrt_voe_handler_read);
EXPORT_SYMBOL(ecrt_voe_handler_write);
EXPORT_SYMBOL(ecrt_voe_handler_execute);

/** \endcond */

/*****************************************************************************/

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
   Mailbox functionality.
*/

/*****************************************************************************/

#include <linux/slab.h>
#include <linux/delay.h>

#include "mailbox.h"
#include "datagram.h"
#include "master.h"

/*****************************************************************************/

```cpp
/**
 * @brief 准备发送邮箱数据报
 *
 * 此函数准备要发送的邮箱数据报，返回指向邮箱数据报数据的指针，或者返回ERR_PTR()错误代码。
 *
 * @param slave 从站
 * @param datagram 数据报
 * @param type 邮箱协议类型
 * @param size 数据大小
 * @return 指向邮箱数据报数据的指针，或者返回错误代码
 */
uint8_t *ec_slave_mbox_prepare_send(const ec_slave_t *slave, /**< 从站 */
                                    ec_datagram_t *datagram, /**< 数据报 */
                                    uint8_t type,            /**< 邮箱协议类型 */
                                    size_t size              /**< 数据大小 */
)
{
    size_t total_size;
    int ret;

    if (unlikely(!slave->sii_image))
    {
        EC_SLAVE_ERR(slave, "从站无法验证是否支持邮箱通信！\n");
        return ERR_PTR(-EAGAIN);
    }

    if (unlikely(!slave->sii_image->sii.mailbox_protocols))
    {
        EC_SLAVE_ERR(slave, "从站不支持邮箱通信！\n");
        return ERR_PTR(-EPROTONOSUPPORT);
    }

    total_size = EC_MBOX_HEADER_SIZE + size;

    if (unlikely(total_size > slave->configured_rx_mailbox_size))
    {
        EC_SLAVE_ERR(slave, "数据大小（%zu）超过邮箱大小（%u）！\n",
                     total_size, slave->configured_rx_mailbox_size);
        return ERR_PTR(-EOVERFLOW);
    }

    ret = ec_datagram_fpwr(datagram, slave->station_address,
                           slave->configured_rx_mailbox_offset,
                           slave->configured_rx_mailbox_size);
    if (ret)
    {
        return ERR_PTR(ret);
    }

    EC_WRITE_U16(datagram->data, size);                       // 邮箱服务数据长度
    EC_WRITE_U16(datagram->data + 2, slave->station_address); // 站地址
    EC_WRITE_U8(datagram->data + 4, 0x00);                    // 通道和优先级
    EC_WRITE_U8(datagram->data + 5, type);                    // 底层协议类型

    return datagram->data + EC_MBOX_HEADER_SIZE;
}

/*****************************************************************************/

/**
 * @brief 准备检查邮箱状态的数据报
 *
 * \todo 确定用于接收邮箱的同步管理器
 *
 * @param slave 从站
 * @param datagram 数据报
 * @return 成功返回0，否则返回小于0的值
 */
int ec_slave_mbox_prepare_check(const ec_slave_t *slave, /**< 从站 */
                                ec_datagram_t *datagram  /**< 数据报 */
)
{
    int ret = ec_datagram_fprd(datagram, slave->station_address, 0x808, 8);
    if (ret)
        return ret;

    ec_datagram_zero(datagram);
    return 0;
}

/*****************************************************************************/

/**
 * @brief 处理检查邮箱状态的数据报
 *
 * @param datagram 数据报
 * @return 成功返回0，否则返回小于0的值
 */
int ec_slave_mbox_check(const ec_datagram_t *datagram /**< 数据报 */)
{
    return EC_READ_U8(datagram->data + 5) & 8 ? 1 : 0;
}


/*****************************************************************************/

/**
 * @brief 准备获取邮箱数据的数据报
 *
 * @param slave 从站
 * @param datagram 数据报
 * @return 成功返回0，否则返回小于0的值
 *
 * @details 此函数用于准备获取邮箱数据的数据报。它通过从站的站地址、配置的发送邮箱偏移量和大小，
 * 从主站中读取邮箱数据，并将数据报清零。如果读取和清零操作成功，则返回0；否则返回小于0的值。
 */
int ec_slave_mbox_prepare_fetch(const ec_slave_t *slave, /**< 从站 */
                                ec_datagram_t *datagram  /**< 数据报 */
)
{
    int ret = ec_datagram_fprd(datagram, slave->station_address,
                               slave->configured_tx_mailbox_offset,
                               slave->configured_tx_mailbox_size);
    if (ret)
        return ret;

    ec_datagram_zero(datagram);
    return 0;
}


/*****************************************************************************/

/**
   邮箱错误代码。
*/

const ec_code_msg_t mbox_error_messages[] = {
    {0x00000001, "MBXERR_SYNTAX（语法错误）"},
    {0x00000002, "MBXERR_UNSUPPORTEDPROTOCOL（不支持的协议）"},
    {0x00000003, "MBXERR_INVAILDCHANNEL（无效的通道）"},
    {0x00000004, "MBXERR_SERVICENOTSUPPORTED（不支持的服务）"},
    {0x00000005, "MBXERR_INVALIDHEADER（无效的头部）"},
    {0x00000006, "MBXERR_SIZETOOSHORT（大小太短）"},
    {0x00000007, "MBXERR_NOMOREMEMORY（内存不足）"},
    {0x00000008, "MBXERR_INVALIDSIZE（无效的大小）"},
    {}};


/*****************************************************************************/

/**
 * @brief 处理接收到的邮箱数据
 *
 * @param slave 从站
 * @param response_data 响应数据
 * @param type 预期的邮箱协议
 * @param size 接收到的数据大小
 * @return 指向接收到的数据的指针，或者ERR_PTR()代码
 *
 * @details 此函数用于处理接收到的邮箱数据。它首先检查是否接收到了有效的响应数据，
 * 如果没有接收到数据，则返回一个错误代码。然后，它检查接收到的数据的大小是否正确，
 * 如果数据大小与配置的发送邮箱大小不匹配，则返回一个错误代码。接下来，它从数据中解析出邮箱协议类型和数据大小，
 * 并将其存储在相应的参数中。如果接收到的数据是一个邮箱错误响应，则打印错误信息并返回一个错误代码。
 * 否则，它返回指向接收到的数据的指针。
 */
uint8_t *ec_slave_mbox_fetch(const ec_slave_t *slave,       /**< 从站 */
                             ec_mbox_data_t *response_data, /**< 响应数据 */
                             uint8_t *type,                 /**< 预期的邮箱协议 */
                             size_t *size                   /**< 接收到的数据大小 */
)
{
    size_t data_size;

    if (!response_data->data)
    {
        EC_SLAVE_ERR(slave, "未接收到邮箱响应数据！\n");
        return ERR_PTR(-EPROTO);
    }

    data_size = EC_READ_U16(response_data->data);

    if (data_size + EC_MBOX_HEADER_SIZE > slave->configured_tx_mailbox_size)
    {
        EC_SLAVE_ERR(slave, "接收到的邮箱响应数据损坏！\n");
        ec_print_data(response_data->data, slave->configured_tx_mailbox_size);
        return ERR_PTR(-EPROTO);
    }

#if 0
    if (slave->master->debug_level) {
        EC_SLAVE_DBG(slave, 1, "邮箱数据:\n");
        ec_print_data(datagram->data, EC_MBOX_HEADER_SIZE + data_size);
    }
#endif

    *type = EC_READ_U8(response_data->data + 5) & 0x0F;
    *size = data_size;

    if (*type == 0x00)
    {
        const ec_code_msg_t *mbox_msg;
        uint16_t code = EC_READ_U16(response_data->data + 8);

        EC_SLAVE_ERR(slave, "接收到邮箱错误响应 - ");

        for (mbox_msg = mbox_error_messages; mbox_msg->code; mbox_msg++)
        {
            if (mbox_msg->code != code)
                continue;
            printk(KERN_CONT "Code 0x%04X: \"%s\".\n",
                   mbox_msg->code, mbox_msg->message);
            break;
        }

        if (!mbox_msg->code)
        {
            printk(KERN_CONT "未知的错误回复代码 0x%04X。\n", code);
        }

        if (slave->master->debug_level && data_size > 0)
        {
            ec_print_data(response_data->data + EC_MBOX_HEADER_SIZE, data_size);
        }

        return ERR_PTR(-EPROTO);
    }

    return response_data->data + EC_MBOX_HEADER_SIZE;
}


/*****************************************************************************/

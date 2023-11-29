/******************************************************************************
 *
 *  $Id$
 *
 *  Copyright (C) 2006-2014  Florian Pose, Ingenieurgemeinschaft IgH
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
   EtherCAT EoE state machines.
*/

/*****************************************************************************/

#include "globals.h"
#include "master.h"
#include "mailbox.h"
#include "fsm_eoe.h"

/*****************************************************************************/
/** 等待设置IP参数响应的最大时间。
 */
#define EC_EOE_RESPONSE_TIMEOUT 3000 // [毫秒]
/*****************************************************************************/

void ec_fsm_eoe_set_ip_start(ec_fsm_eoe_t *, ec_datagram_t *);
void ec_fsm_eoe_set_ip_request(ec_fsm_eoe_t *, ec_datagram_t *);
void ec_fsm_eoe_set_ip_check(ec_fsm_eoe_t *, ec_datagram_t *);
void ec_fsm_eoe_set_ip_response(ec_fsm_eoe_t *, ec_datagram_t *);
void ec_fsm_eoe_set_ip_response_data(ec_fsm_eoe_t *, ec_datagram_t *);

void ec_fsm_eoe_end(ec_fsm_eoe_t *, ec_datagram_t *);
void ec_fsm_eoe_error(ec_fsm_eoe_t *, ec_datagram_t *);

/*****************************************************************************/

/**
 * @brief 初始化EoE有限状态机。
 *
 * @param fsm 有限状态机对象。
 *
 * @details 此函数用于初始化EoE（以太网上的EtherCAT）有限状态机。
 * 它将有限状态机对象的成员变量初始化为默认值。
 */
void ec_fsm_eoe_init(
    ec_fsm_eoe_t *fsm /**< 有限状态机 */
)
{
    fsm->slave = NULL;
    fsm->retries = 0;
    fsm->state = NULL;
    fsm->datagram = NULL;
    fsm->jiffies_start = 0;
    fsm->request = NULL;
}

/*****************************************************************************/

/**
 * @brief 清除EoE有限状态机。
 *
 * @param fsm 有限状态机对象。
 *
 * @details 此函数用于清除EoE（以太网上的EtherCAT）有限状态机。
 * 它目前没有实现任何功能。
 */
void ec_fsm_eoe_clear(
    ec_fsm_eoe_t *fsm /**< 有限状态机 */
)
{
}

/*****************************************************************************/

/**
 * @brief 开始设置从站的EoE IP参数。
 *
 * @param fsm 有限状态机对象。
 * @param slave EtherCAT从站。
 * @param request EoE请求。
 *
 * @details 此函数用于开始设置从站的EoE（以太网上的EtherCAT）IP参数。
 * 它将有限状态机对象的成员变量设置为相应的值，并将状态设置为设置IP参数的起始状态。
 */
void ec_fsm_eoe_set_ip_param(
    ec_fsm_eoe_t *fsm,        /**< 有限状态机 */
    ec_slave_t *slave,        /**< EtherCAT从站 */
    ec_eoe_request_t *request /**< EoE请求 */
)
{
    fsm->slave = slave;
    fsm->request = request;
    fsm->state = ec_fsm_eoe_set_ip_start;
}

/*****************************************************************************/

/**
 * @brief 执行有限状态机的当前状态。
 *
 * @param fsm 有限状态机对象。
 * @param datagram 要使用的数据报。
 * @return 如果有限状态机仍在进行中，则返回1；否则返回0。
 *
 * @details 此函数用于执行有限状态机的当前状态。
 * 它根据有限状态机对象的状态成员调用相应的状态函数。
 * 如果有限状态机的状态为结束状态或错误状态，则返回0表示执行完成。
 * 如果有限状态机的状态为其他状态，则返回1表示仍在进行中。
 */
int ec_fsm_eoe_exec(
    ec_fsm_eoe_t *fsm,      /**< 有限状态机 */
    ec_datagram_t *datagram /**< 要使用的数据报 */
)
{
    if (fsm->state == ec_fsm_eoe_end || fsm->state == ec_fsm_eoe_error)
        return 0;

    fsm->state(fsm, datagram);

    if (fsm->state == ec_fsm_eoe_end || fsm->state == ec_fsm_eoe_error)
    {
        fsm->datagram = NULL;
        return 0;
    }

    fsm->datagram = datagram;
    return 1;
}

/*****************************************************************************/

/**
 * @brief 检查有限状态机是否成功终止。
 *
 * @param fsm 有限状态机对象。
 * @return 如果有限状态机成功终止，则返回非零值；否则返回零。
 *
 * @details 此函数用于检查有限状态机是否成功终止。
 * 如果有限状态机的状态为结束状态，则返回非零值表示成功终止。
 * 否则返回零表示未成功终止。
 */
int ec_fsm_eoe_success(const ec_fsm_eoe_t *fsm /**< 有限状态机 */)
{
    return fsm->state == ec_fsm_eoe_end;
}

/******************************************************************************
 * EoE set IP parameter state machine
 *****************************************************************************/

/**

@brief 准备设置IP参数的操作。

@param fsm 有限状态机

@param datagram 要使用的数据报。

@return 成功返回0，否则返回负错误代码。

@details 该函数用于准备设置IP参数的操作。根据给定的参数，构建相应的数据报并发送给从站。
*/
int ec_fsm_eoe_prepare_set(
ec_fsm_eoe_t *fsm, /< 有限状态机 */
ec_datagram_t *datagram /< 要使用的数据报。 */
)
{
uint8_t *data, *cur;
ec_slave_t *slave = fsm->slave;
ec_master_t *master = slave->master;
ec_eoe_request_t *req = fsm->request;

// 注意：基于wireshark的数据包过滤器建议EOE_INIT信息具有固定的大小和固定的信息位置。
// 参考：packet-ecatmb.h和packet-ecatmb.c
// 然而，TwinCAT 2.1的测试也表明，如果缺少某个信息，则所有后续的项目都会被忽略。
// 此外，如果要使用DHCP，则只需设置MAC地址。
size_t size = 8 + // 头部 + 标志
ETH_ALEN + // MAC地址
4 + // IP地址
4 + // 子网掩码
4 + // 网关
4 + // DNS服务器
EC_MAX_HOSTNAME_SIZE; // DNS名称

data = ec_slave_mbox_prepare_send(slave, datagram, EC_MBOX_TYPE_EOE,
size);
if (IS_ERR(data))
{
return PTR_ERR(data);
}

// 将数据清零
memset(data, 0, size);

// 头部
EC_WRITE_U8(data, EC_EOE_TYPE_INIT_REQ); // 设置IP参数请求
EC_WRITE_U8(data + 1, 0x00); // 未使用
EC_WRITE_U16(data + 2, 0x0000); // 未使用

EC_WRITE_U32(data + 4,
((req->mac_address_included != 0) << 0) |
((req->ip_address_included != 0) << 1) |
((req->subnet_mask_included != 0) << 2) |
((req->gateway_included != 0) << 3) |
((req->dns_included != 0) << 4) |
((req->name_included != 0) << 5));

cur = data + 8;

if (req->mac_address_included)
{
memcpy(cur, req->mac_address, ETH_ALEN);
}
cur += ETH_ALEN;

if (req->ip_address_included)
{
uint32_t swapped = htonl(req->ip_address);
memcpy(cur, &swapped, 4);
}
cur += 4;

if (req->subnet_mask_included)
{
uint32_t swapped = htonl(req->subnet_mask);
memcpy(cur, &swapped, 4);
}
cur += 4;

if (req->gateway_included)
{
uint32_t swapped = htonl(req->gateway);
memcpy(cur, &swapped, 4);
}
cur += 4;

if (req->dns_included)
{
uint32_t swapped = htonl(req->dns);
memcpy(cur, &swapped, 4);
}
cur += 4;

if (req->name_included)
{
memcpy(cur, req->name, EC_MAX_HOSTNAME_SIZE);
}
cur += EC_MAX_HOSTNAME_SIZE;

if (master->debug_level)
{
EC_SLAVE_DBG(slave, 0, "设置IP参数请求:\n");
ec_print_data(data, cur - data);
}

fsm->request->jiffies_sent = jiffies;

return 0;
}

/*****************************************************************************/

/**
 * @brief EoE状态：设置IP开始。
 *
 * @param fsm 有限状态机对象。
 * @param datagram 用于使用的数据报。
 *
 * @details 此函数用于设置EoE状态机的IP参数。
 * 首先，它会检查从状态机对象中获取的从站是否准备好执行EoE状态机。
 * 如果从站未准备好，将设置状态为错误并返回。
 * 然后，它会检查从站是否支持EoE协议。
 * 如果不支持，将设置状态为错误并返回。
 * 接下来，它会调用ec_fsm_eoe_prepare_set函数来准备设置IP参数。
 * 如果准备失败，将设置状态为错误并返回。
 * 最后，将重试次数设置为默认值，并将状态设置为设置IP请求。
 */
void ec_fsm_eoe_set_ip_start(
    ec_fsm_eoe_t *fsm,      /**< 有限状态机 */
    ec_datagram_t *datagram /**< 用于使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;

    EC_SLAVE_DBG(slave, 1, "Setting IP parameters.\n");

    if (!slave->sii_image)
    {
        EC_SLAVE_ERR(slave, "Slave not ready to execute EoE FSM\n");
        fsm->state = ec_fsm_eoe_error;
        return;
    }

    if (!(slave->sii_image->sii.mailbox_protocols & EC_MBOX_EOE))
    {
        EC_SLAVE_ERR(slave, "Slave does not support EoE!\n");
        fsm->state = ec_fsm_eoe_error;
        return;
    }

    if (ec_fsm_eoe_prepare_set(fsm, datagram))
    {
        fsm->state = ec_fsm_eoe_error;
        return;
    }

    fsm->retries = EC_FSM_RETRIES;
    fsm->state = ec_fsm_eoe_set_ip_request;
}
/*****************************************************************************/

/**
 * @brief EoE状态：设置IP开始。
 *
 * @param fsm 有限状态机对象。
 * @param datagram 用于使用的数据报。
 *
 * @details 此函数用于设置EoE状态机的IP参数。
 * 首先，它会检查从状态机对象中获取的从站是否准备好执行EoE状态机。
 * 如果从站未准备好，将设置状态为错误并返回。
 * 然后，它会检查从站是否支持EoE协议。
 * 如果不支持，将设置状态为错误并返回。
 * 接下来，它会调用ec_fsm_eoe_prepare_set函数来准备设置IP参数。
 * 如果准备失败，将设置状态为错误并返回。
 * 最后，将重试次数设置为默认值，并将状态设置为设置IP请求。
 */
void ec_fsm_eoe_set_ip_start(
    ec_fsm_eoe_t *fsm,      /**< 有限状态机 */
    ec_datagram_t *datagram /**< 用于使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;

    EC_SLAVE_DBG(slave, 1, "正在设置IP参数。\n");

    if (!slave->sii_image)
    {
        EC_SLAVE_ERR(slave, "从站未准备好执行EoE状态机。\n");
        fsm->state = ec_fsm_eoe_error;
        return;
    }

    if (!(slave->sii_image->sii.mailbox_protocols & EC_MBOX_EOE))
    {
        EC_SLAVE_ERR(slave, "从站不支持EoE协议！\n");
        fsm->state = ec_fsm_eoe_error;
        return;
    }

    if (ec_fsm_eoe_prepare_set(fsm, datagram))
    {
        fsm->state = ec_fsm_eoe_error;
        return;
    }

    fsm->retries = EC_FSM_RETRIES;
    fsm->state = ec_fsm_eoe_set_ip_request;
}

/*****************************************************************************/

/**
 * @brief EoE状态：设置IP请求。
 *
 * @param fsm 有限状态机对象。
 * @param datagram 用于使用的数据报。
 *
 * @details 此函数用于处理EoE状态机的设置IP请求。
 * 如果数据报的状态为超时并且重试次数不为零，则重新准备设置IP参数并返回。
 * 如果数据报的状态不是接收到，则将状态设置为错误，并打印错误信息。
 * 如果数据报的工作计数器不为1，则根据情况进行处理：
 * - 如果工作计数器为零且时间间隔小于EoE响应超时时间，则重新发送请求数据报并返回。
 * - 否则，将状态设置为错误，并打印错误信息。
 * 如果数据报的工作计数器为1，则记录起始时间，并根据情况进行处理：
 * - 如果从站的邮箱读取操作已经在进行中，则将状态设置为设置IP响应数据，并将数据报标记为无效。
 * - 否则，准备邮箱检查操作，并将重试次数设置为默认值，并将状态设置为设置IP检查。
 */
void ec_fsm_eoe_set_ip_request(
    ec_fsm_eoe_t *fsm,      /**< 有限状态机 */
    ec_datagram_t *datagram /**< 用于使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;

    if (fsm->datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--)
    {
        if (ec_fsm_eoe_prepare_set(fsm, datagram))
        {
            fsm->state = ec_fsm_eoe_error;
        }
        return;
    }

    if (fsm->datagram->state != EC_DATAGRAM_RECEIVED)
    {
        fsm->state = ec_fsm_eoe_error;
        EC_SLAVE_ERR(slave, "接收EoE设置IP参数请求失败：");
        ec_datagram_print_state(fsm->datagram);
        return;
    }

    if (fsm->datagram->working_counter != 1)
    {
        unsigned long diff_ms =
            (jiffies - fsm->request->jiffies_sent) * 1000 / HZ;

        if (!fsm->datagram->working_counter)
        {
            if (diff_ms < EC_EOE_RESPONSE_TIMEOUT)
            {
                // 无响应；重新发送请求数据报
                if (ec_fsm_eoe_prepare_set(fsm, datagram))
                {
                    fsm->state = ec_fsm_eoe_error;
                }
                return;
            }
        }
        fsm->state = ec_fsm_eoe_error;
        EC_SLAVE_ERR(slave, "等待EoE设置IP参数请求超时，耗时 %lu ms：", diff_ms);
        ec_datagram_print_wc_error(fsm->datagram);
        return;
    }

    fsm->jiffies_start = fsm->datagram->jiffies_sent;

    // 如果邮箱读取操作已经在进行中，则直接进入设置IP响应数据状态，并将数据报标记为无效
    if (ec_read_mbox_locked(slave))
    {
        fsm->state = ec_fsm_eoe_set_ip_response_data;
        // 数据报不会被使用，标记为无效
        datagram->state = EC_DATAGRAM_INVALID;
    }
    else
    {
        ec_slave_mbox_prepare_check(slave, datagram); // 不会失败
        fsm->retries = EC_FSM_RETRIES;
        fsm->state = ec_fsm_eoe_set_ip_check;
    }
}

/*****************************************************************************/

/**
 * @brief EoE状态：设置IP检查。
 *
 * @param fsm 有限状态机对象。
 * @param datagram 用于使用的数据报。
 *
 * @details 此函数用于处理EoE状态机的设置IP检查。
 * 如果数据报的状态为超时并且重试次数不为零，则准备邮箱检查操作并返回。
 * 如果数据报的状态不是接收到，则将状态设置为错误，并打印错误信息。
 * 如果数据报的工作计数器不为1，则将状态设置为错误，并打印错误信息。
 * 如果数据报的工作计数器为1，则根据情况进行处理：
 * - 如果从站的邮箱中已经接收到数据，则将状态设置为设置IP响应数据，并调用相应的函数处理数据。
 * - 否则，根据情况进行处理：
 *   - 如果等待时间超过EoE响应超时时间，则将状态设置为错误，并打印错误信息。
 *   - 否则，准备邮箱检查操作，并将重试次数设置为默认值。
 */
void ec_fsm_eoe_set_ip_check(
    ec_fsm_eoe_t *fsm,      /**< 有限状态机 */
    ec_datagram_t *datagram /**< 用于使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;

    if (fsm->datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--)
    {
        ec_slave_mbox_prepare_check(slave, datagram); // 不会失败
        return;
    }

    if (fsm->datagram->state != EC_DATAGRAM_RECEIVED)
    {
        fsm->state = ec_fsm_eoe_error;
        ec_read_mbox_lock_clear(slave);
        EC_SLAVE_ERR(slave, "接收EoE邮箱检查数据报失败：");
        ec_datagram_print_state(fsm->datagram);
        return;
    }

    if (fsm->datagram->working_counter != 1)
    {
        fsm->state = ec_fsm_eoe_error;
        ec_read_mbox_lock_clear(slave);
        EC_SLAVE_ERR(slave, "接收EoE邮箱检查数据报失败：");
        ec_datagram_print_wc_error(fsm->datagram);
        return;
    }

    if (!ec_slave_mbox_check(fsm->datagram))
    {
        unsigned long diff_ms;

        // 检查数据是否已经被其他读取请求接收
        if (slave->mbox_eoe_init_data.payload_size > 0)
        {
            ec_read_mbox_lock_clear(slave);
            fsm->state = ec_fsm_eoe_set_ip_response_data;
            fsm->state(fsm, datagram);
            return;
        }

        diff_ms = (fsm->datagram->jiffies_received - fsm->jiffies_start) * 1000 / HZ;
        if (diff_ms >= EC_EOE_RESPONSE_TIMEOUT)
        {
            fsm->state = ec_fsm_eoe_error;
            ec_read_mbox_lock_clear(slave);
            EC_SLAVE_ERR(slave, "等待设置IP参数响应超时，耗时 %lu ms。\n", diff_ms);
            return;
        }

        ec_slave_mbox_prepare_check(slave, datagram); // 不会失败
        fsm->retries = EC_FSM_RETRIES;
        return;
    }

    // 获取响应数据
    ec_slave_mbox_prepare_fetch(slave, datagram); // 不会失败
    fsm->retries = EC_FSM_RETRIES;
    fsm->state = ec_fsm_eoe_set_ip_response;
}

/*****************************************************************************/

/**
 * @brief EoE状态：设置IP响应。
 *
 * @param fsm 有限状态机对象。
 * @param datagram 用于使用的数据报。
 *
 * @details 此函数用于处理EoE状态机的设置IP响应。
 * 如果数据报的状态为超时并且重试次数不为零，则准备数据报并返回。
 * 如果数据报的状态不是接收到，则将状态设置为错误，并打印错误信息。
 * 如果数据报的工作计数器不为1，则根据情况进行处理：
 * - 如果数据已经被其他读取请求接收，则不会报错。
 * - 否则，将状态设置为错误，并打印错误信息。
 * 清除邮箱读取锁定，并将状态设置为设置IP响应数据，调用相应的函数处理数据。
 */
void ec_fsm_eoe_set_ip_response(
    ec_fsm_eoe_t *fsm,      /**< 有限状态机 */
    ec_datagram_t *datagram /**< 用于使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;

    if (fsm->datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--)
    {
        ec_slave_mbox_prepare_fetch(slave, datagram); // 不会失败
        return;
    }

    if (fsm->datagram->state != EC_DATAGRAM_RECEIVED)
    {
        fsm->state = ec_fsm_eoe_error;
        ec_read_mbox_lock_clear(slave);
        EC_SLAVE_ERR(slave, "接收EoE读取响应数据报失败：");
        ec_datagram_print_state(fsm->datagram);
        return;
    }

    if (fsm->datagram->working_counter != 1)
    {
        // 只有在数据还未被其他读取请求接收时才报错
        if (slave->mbox_eoe_init_data.payload_size == 0)
        {
            fsm->state = ec_fsm_eoe_error;
            ec_read_mbox_lock_clear(slave);
            EC_SLAVE_ERR(slave, "接收EoE读取响应失败：");
            ec_datagram_print_wc_error(fsm->datagram);
            return;
        }
    }
    ec_read_mbox_lock_clear(slave);
    fsm->state = ec_fsm_eoe_set_ip_response_data;
    fsm->state(fsm, datagram);
}

/*****************************************************************************/

/**
 * @brief EoE状态：设置IP响应数据。
 *
 * @param fsm 有限状态机对象。
 * @param datagram 用于使用的数据报。
 *
 * @details 此函数用于处理EoE状态机的设置IP响应数据。
 * 如果从站的邮箱中有可用的数据，则处理数据或发起新的邮箱读取检查操作。
 * 如果邮箱中没有可用的数据，则发起新的邮箱读取检查操作并返回。
 * 如果邮箱读取操作被锁定，则等待当前读取请求完成并将数据报标记为无效。
 * 如果邮箱读取操作没有被锁定，则准备邮箱检查操作，并将状态设置为设置IP检查。
 */
void ec_fsm_eoe_set_ip_response_data(
    ec_fsm_eoe_t *fsm,      /**< 有限状态机 */
    ec_datagram_t *datagram /**< 用于使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;
    ec_master_t *master = slave->master;
    uint8_t *data, mbox_prot, eoe_type;
    size_t rec_size;
    ec_eoe_request_t *req = fsm->request;

    // 处理可用的数据或发起新的邮箱读取检查操作
    if (slave->mbox_eoe_init_data.payload_size > 0)
    {
        slave->mbox_eoe_init_data.payload_size = 0;
    }
    else
    {
        // 如果需要的数据不可用，则发起新的邮箱读取检查操作
        if (ec_read_mbox_locked(slave))
        {
            // 等待当前的读取请求完成，并将数据报标记为无效
            datagram->state = EC_DATAGRAM_INVALID;
        }
        else
        {
            ec_slave_mbox_prepare_check(slave, datagram); // 不会失败
            fsm->state = ec_fsm_eoe_set_ip_check;
        }
        return;
    }

    data = ec_slave_mbox_fetch(slave, &slave->mbox_eoe_init_data, &mbox_prot, &rec_size);
    if (IS_ERR(data))
    {
        fsm->state = ec_fsm_eoe_error;
        return;
    }

    if (master->debug_level)
    {
        EC_SLAVE_DBG(slave, 0, "设置IP参数响应：\n");
        ec_print_data(data, rec_size);
    }

    if (mbox_prot != EC_MBOX_TYPE_EOE)
    {
        fsm->state = ec_fsm_eoe_error;
        EC_SLAVE_ERR(slave, "接收到错误的邮箱协议响应 0x%02X。\n",
                     mbox_prot);
        return;
    }

    if (rec_size < 4)
    {
        fsm->state = ec_fsm_eoe_error;
        EC_SLAVE_ERR(slave, "接收到损坏的EoE设置IP参数响应（%zu字节）！\n",
                     rec_size);
        ec_print_data(data, rec_size);
        return;
    }

    eoe_type = EC_READ_U8(data) & 0x0F;

    if (eoe_type != EC_EOE_TYPE_INIT_RES)
    {
        EC_SLAVE_ERR(slave, "EoE初始化处理程序接收到其他类型的EoE响应（类型 %x）。已忽略。\n",
                     eoe_type);
        ec_print_data(data, rec_size);
        fsm->state = ec_fsm_eoe_error;
        return;
    }

    req->result = EC_READ_U16(data + 2);

    if (req->result)
    {
        fsm->state = ec_fsm_eoe_error;
        EC_SLAVE_DBG(slave, 1, "EoE设置IP参数失败，结果代码为 0x%04X。\n",
                     req->result);
    }
    else
    {
        fsm->state = ec_fsm_eoe_end; // 成功
    }
}

/*****************************************************************************/

/** 状态：错误。
 */
void ec_fsm_eoe_error(
    ec_fsm_eoe_t *fsm,      /**< 有限状态机 */
    ec_datagram_t *datagram /**< 用于使用的数据报 */
)
{
}

/*****************************************************************************/

/** 状态：结束。
 */
void ec_fsm_eoe_end(
    ec_fsm_eoe_t *fsm,      /**< 有限状态机 */
    ec_datagram_t *datagram /**< 用于使用的数据报 */
)
{
}

/*****************************************************************************/

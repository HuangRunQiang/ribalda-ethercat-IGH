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
   EtherCAT SoE状态机。
*/

/*****************************************************************************/

#include "globals.h"
#include "master.h"
#include "mailbox.h"
#include "fsm_soe.h"

/*****************************************************************************/

/** SoE操作
 */
enum
{
    OPCODE_READ_REQUEST = 0x01,  /**< 读取请求。 */
    OPCODE_READ_RESPONSE = 0x02, /**< 读取响应。 */
    OPCODE_WRITE_REQUEST = 0x03, /**< 写入请求。 */
    OPCODE_WRITE_RESPONSE = 0x04 /**< 写入响应。 */
};

/** 所有SoE头部的大小。
 */
#define EC_SOE_SIZE 0x04

/** SoE头部的大小。
 */
#define EC_SOE_HEADER_SIZE (EC_MBOX_HEADER_SIZE + EC_SOE_SIZE)

/** SoE响应超时时间 [ms]。
 */
#define EC_SOE_RESPONSE_TIMEOUT 1000

/*****************************************************************************/

void ec_fsm_soe_read_start(ec_fsm_soe_t *, ec_datagram_t *);
void ec_fsm_soe_read_request(ec_fsm_soe_t *, ec_datagram_t *);
void ec_fsm_soe_read_check(ec_fsm_soe_t *, ec_datagram_t *);
void ec_fsm_soe_read_response(ec_fsm_soe_t *, ec_datagram_t *);
void ec_fsm_soe_read_response_data(ec_fsm_soe_t *, ec_datagram_t *);

void ec_fsm_soe_write_start(ec_fsm_soe_t *, ec_datagram_t *);
void ec_fsm_soe_write_request(ec_fsm_soe_t *, ec_datagram_t *);
void ec_fsm_soe_write_check(ec_fsm_soe_t *, ec_datagram_t *);
void ec_fsm_soe_write_response(ec_fsm_soe_t *, ec_datagram_t *);
void ec_fsm_soe_write_response_data(ec_fsm_soe_t *, ec_datagram_t *);

void ec_fsm_soe_end(ec_fsm_soe_t *, ec_datagram_t *);
void ec_fsm_soe_error(ec_fsm_soe_t *, ec_datagram_t *);

/*****************************************************************************/

extern const ec_code_msg_t soe_error_codes[];

/*****************************************************************************/

/**
 * @brief 输出SoE错误码。
 * @param slave EtherCAT从站。
 * @param error_code 错误码。
 * @return 无。
 * @details 根据错误码输出对应的SoE错误信息，如果错误码未知则输出未知错误信息。
 */
void ec_print_soe_error(const ec_slave_t *slave, uint16_t error_code)
{
    const ec_code_msg_t *error_msg;

    for (error_msg = soe_error_codes; error_msg->code; error_msg++)
    {
        if (error_msg->code == error_code)
        {
            EC_SLAVE_ERR(slave, "SoE错误 0x%04X: \"%s\"。\n",
                         error_msg->code, error_msg->message);
            return;
        }
    }

    EC_SLAVE_ERR(slave, "未知的SoE错误 0x%04X。\n", error_code);
}

/*****************************************************************************/

/**
 * @brief 构造函数。
 * @param fsm 有限状态机。
 * @return 无。
 * @details 初始化有限状态机的状态、数据报和片段大小。
 */
void ec_fsm_soe_init(
    ec_fsm_soe_t *fsm /**< 有限状态机 */
)
{
    fsm->state = NULL;
    fsm->datagram = NULL;
    fsm->fragment_size = 0;
}

/*****************************************************************************/

/**
 * @brief 析构函数。
 * @param fsm 有限状态机。
 * @return 无。
 */
void ec_fsm_soe_clear(
    ec_fsm_soe_t *fsm /**< 有限状态机 */
)
{
}

/*****************************************************************************/

/**
 * @brief 开始传输IDN到/从从站。
 * @param fsm 有限状态机。
 * @param slave EtherCAT从站。
 * @param request SoE请求。
 * @return 无。
 */
void ec_fsm_soe_transfer(
    ec_fsm_soe_t *fsm,        /**< 有限状态机 */
    ec_slave_t *slave,        /**< EtherCAT从站 */
    ec_soe_request_t *request /**< SoE请求 */
)
{
    fsm->slave = slave;
    fsm->request = request;

    if (request->dir == EC_DIR_OUTPUT)
    {
        fsm->state = ec_fsm_soe_write_start;
    }
    else
    {
        fsm->state = ec_fsm_soe_read_start;
    }
}

/*****************************************************************************/

/**
 * @brief 执行当前状态的状态机。
 * @param fsm 有限状态机。
 * @param datagram 使用的数据报。
 * @return 如果状态机仍在进行中，则返回1；否则返回0。
 */
int ec_fsm_soe_exec(
    ec_fsm_soe_t *fsm,      /**< 有限状态机 */
    ec_datagram_t *datagram /**< 使用的数据报 */
)
{
    if (fsm->state == ec_fsm_soe_end || fsm->state == ec_fsm_soe_error)
        return 0;

    fsm->state(fsm, datagram);

    if (fsm->state == ec_fsm_soe_end || fsm->state == ec_fsm_soe_error)
    {
        fsm->datagram = NULL;
        return 0;
    }

    fsm->datagram = datagram;
    return 1;
}

/*****************************************************************************/

/**
 * @brief 判断有限状态机是否成功终止。
 *
 * @param fsm 有限状态机。
 * @return 非零值表示成功。
 *
 * @details 判断有限状态机的状态是否等于 `ec_fsm_soe_end`。
 */
int ec_fsm_soe_success(const ec_fsm_soe_t *fsm /**< 有限状态机 */)
{
    return fsm->state == ec_fsm_soe_end;
}

/*****************************************************************************/

/**
 * @brief 输出关于失败的SoE传输的信息。
 *
 * @param fsm 有限状态机。
 *
 * @details 输出关于失败的SoE传输的信息。根据请求的方向，打印相应的消息。
 */
void ec_fsm_soe_print_error(ec_fsm_soe_t *fsm /**< 有限状态机 */)
{
    ec_soe_request_t *request = fsm->request;

    EC_SLAVE_ERR(fsm->slave, "");

    if (request->dir == EC_DIR_OUTPUT)
    {
        printk(KERN_CONT "写入ing");
    }
    else
    {
        printk(KERN_CONT "读取ing");
    }

    printk(KERN_CONT " IDN 0x%04X 失败。\n", request->idn);
}

/******************************************************************************
 * SoE read state machine
 *****************************************************************************/

/**
 * @brief 准备进行读操作。
 *
 * @param fsm 有限状态机
 * @param datagram 要使用的数据报
 * @return 返回0表示成功，否则返回负错误代码
 *
 * @details
 * - 为发送数据准备邮箱。
 * - 如果准备发送数据失败，则返回错误代码。
 * - 将读请求操作码和驱动器编号写入数据。
 * - 将请求值写入数据。
 * - 将IDN写入数据。
 * - 如果启用了调试级别，则打印读请求数据。
 * - 更新发送时间戳和状态。
 */
int ec_fsm_soe_prepare_read(
    ec_fsm_soe_t *fsm,      /**< 有限状态机 */
    ec_datagram_t *datagram /**< 要使用的数据报 */
)
{
    uint8_t *data;
    ec_slave_t *slave = fsm->slave;
    ec_master_t *master = slave->master;
    ec_soe_request_t *request = fsm->request;

    data = ec_slave_mbox_prepare_send(slave, datagram, EC_MBOX_TYPE_SOE, EC_SOE_SIZE);
    if (IS_ERR(data))
    {
        return PTR_ERR(data);
    }

    EC_WRITE_U8(data, OPCODE_READ_REQUEST | (request->drive_no & 0x07) << 5);
    EC_WRITE_U8(data + 1, 1 << 6); // 请求值
    EC_WRITE_U16(data + 2, request->idn);

    if (master->debug_level)
    {
        EC_SLAVE_DBG(slave, 0, "SSC 读请求：\n");
        ec_print_data(data, EC_SOE_SIZE);
    }

    fsm->request->jiffies_sent = jiffies;
    fsm->state = ec_fsm_soe_read_request;

    return 0;
}

/*****************************************************************************/

/**
 * @brief SoE 状态：读开始。
 *
 * @param fsm 有限状态机
 * @param datagram 要使用的数据报
 *
 * @details
 * - 打印读取的驱动器编号和IDN。
 * - 如果从站不支持SoE，则设置错误状态并打印错误信息。
 * - 如果SII数据不可用，则设置错误状态并打印错误信息。
 * - 初始化数据大小和重试次数。
 * - 准备读操作。
 */
void ec_fsm_soe_read_start(
    ec_fsm_soe_t *fsm,      /**< 有限状态机 */
    ec_datagram_t *datagram /**< 要使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;
    ec_soe_request_t *request = fsm->request;

    EC_SLAVE_DBG(slave, 1, "读取驱动器 %u 的 IDN 0x%04X。\n", request->drive_no, request->idn);

    if (!slave->sii_image)
    {
        EC_SLAVE_ERR(slave, "从站无法处理SoE读请求。SII数据不可用。\n");
        fsm->state = ec_fsm_soe_error;
        ec_fsm_soe_print_error(fsm);
        return;
    }

    if (!(slave->sii_image->sii.mailbox_protocols & EC_MBOX_SOE))
    {
        EC_SLAVE_ERR(slave, "从站不支持SoE！\n");
        fsm->state = ec_fsm_soe_error;
        ec_fsm_soe_print_error(fsm);
        return;
    }

    request->data_size = 0;
    fsm->retries = EC_FSM_RETRIES;

    if (ec_fsm_soe_prepare_read(fsm, datagram))
    {
        fsm->state = ec_fsm_soe_error;
        ec_fsm_soe_print_error(fsm);
    }
}

/*****************************************************************************/

/**
 * @brief SoE 状态：读请求。
 *
 * @param fsm 有限状态机
 * @param datagram 要使用的数据报
 *
 * @details
 * - 处理超时情况并重试发送数据报。
 * - 检查数据报状态，如果接收失败则设置错误状态并打印错误信息。
 * - 计算发送请求到现在的时间差。
 * - 如果工作计数器不为1：
 *   - 如果工作计数器为0且时间差小于响应超时时间，则再次发送请求数据报。
 *   - 设置错误状态并打印错误信息。
 * - 设置开始时间戳。
 * - 如果已经有一个读请求正在进行，则设置读响应数据的状态并标记数据报无效。
 * - 否则，准备邮箱检查数据报，并设置重试次数。
 */
void ec_fsm_soe_read_request(
    ec_fsm_soe_t *fsm,      /**< 有限状态机 */
    ec_datagram_t *datagram /**< 要使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;
    unsigned long diff_ms;

    if (fsm->datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--)
    {
        if (ec_fsm_soe_prepare_read(fsm, datagram))
        {
            fsm->state = ec_fsm_soe_error;
            ec_fsm_soe_print_error(fsm);
        }
        return;
    }

    if (fsm->datagram->state != EC_DATAGRAM_RECEIVED)
    {
        fsm->state = ec_fsm_soe_error;
        EC_SLAVE_ERR(slave, "接收SoE读请求失败：");
        ec_datagram_print_state(fsm->datagram);
        ec_fsm_soe_print_error(fsm);
        return;
    }

    diff_ms = (jiffies - fsm->request->jiffies_sent) * 1000 / HZ;

    if (fsm->datagram->working_counter != 1)
    {
        if (!fsm->datagram->working_counter)
        {
            if (diff_ms < EC_SOE_RESPONSE_TIMEOUT)
            {
                // 没有响应；再次发送请求数据报
                if (ec_fsm_soe_prepare_read(fsm, datagram))
                {
                    fsm->state = ec_fsm_soe_error;
                    ec_fsm_soe_print_error(fsm);
                }
                return;
            }
        }
        fsm->state = ec_fsm_soe_error;
        EC_SLAVE_ERR(slave, "在 %lu ms 后接收SoE读请求失败：", diff_ms);
        ec_datagram_print_wc_error(fsm->datagram);
        ec_fsm_soe_print_error(fsm);
        return;
    }

    fsm->jiffies_start = fsm->datagram->jiffies_sent;

    // 如果已经有一个读请求正在进行，则跳过邮箱读取检查
    if (ec_read_mbox_locked(slave))
    {
        fsm->state = ec_fsm_soe_read_response_data;
        // 数据报无效，不使用
        datagram->state = EC_DATAGRAM_INVALID;
    }
    else
    {
        ec_slave_mbox_prepare_check(slave, datagram); // 不会失败。
        fsm->retries = EC_FSM_RETRIES;
        fsm->state = ec_fsm_soe_read_check;
    }
}

/*****************************************************************************/

/**
 * @brief SoE 状态：读检查。
 *
 * @param fsm 有限状态机
 * @param datagram 要使用的数据报
 *
 * @details
 * - 处理超时情况并重试发送邮箱检查数据报。
 * - 检查数据报状态，如果接收失败则设置错误状态并打印错误信息。
 * - 检查工作计数器，如果不为1则设置错误状态并打印错误信息。
 * - 如果邮箱检查失败，则准备邮箱检查数据报并返回。
 * - 否则，准备邮箱获取数据报，并设置重试次数。
 */
void ec_fsm_soe_read_check(
    ec_fsm_soe_t *fsm,      /**< 有限状态机 */
    ec_datagram_t *datagram /**< 要使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;

    if (fsm->datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--)
    {
        ec_slave_mbox_prepare_check(slave, datagram); // 不会失败。
        return;
    }

    if (fsm->datagram->state != EC_DATAGRAM_RECEIVED)
    {
        fsm->state = ec_fsm_soe_error;
        ec_read_mbox_lock_clear(slave);
        EC_SLAVE_ERR(slave, "接收SoE邮箱检查数据报失败：");
        ec_datagram_print_state(fsm->datagram);
        ec_fsm_soe_print_error(fsm);
        return;
    }

    if (fsm->datagram->working_counter != 1)
    {
        fsm->state = ec_fsm_soe_error;
        ec_read_mbox_lock_clear(slave);
        EC_SLAVE_ERR(slave, "接收SoE邮箱检查数据报失败：");
        ec_datagram_print_wc_error(fsm->datagram);
        ec_fsm_soe_print_error(fsm);
        return;
    }

    if (!ec_slave_mbox_check(fsm->datagram))
    {
        unsigned long diff_ms = 0;

        // 检查数据是否已经被另一个读请求接收
        if (slave->mbox_soe_data.payload_size > 0)
        {
            ec_read_mbox_lock_clear(slave);
            fsm->state = ec_fsm_soe_read_response_data;
            fsm->state(fsm, datagram);
            return;
        }

        diff_ms = (fsm->datagram->jiffies_received - fsm->jiffies_start) * 1000 / HZ;

        if (diff_ms >= EC_SOE_RESPONSE_TIMEOUT)
        {
            fsm->state = ec_fsm_soe_error;
            ec_read_mbox_lock_clear(slave);
            EC_SLAVE_ERR(slave, "等待读响应超时，超过 %lu ms。\n", diff_ms);
            ec_fsm_soe_print_error(fsm);
            return;
        }

        ec_slave_mbox_prepare_check(slave, datagram); // 不会失败。
        fsm->retries = EC_FSM_RETRIES;
        return;
    }

    // 获取响应数据
    ec_slave_mbox_prepare_fetch(slave, datagram); // 不会失败。
    fsm->retries = EC_FSM_RETRIES;
    fsm->state = ec_fsm_soe_read_response;
}


/*****************************************************************************/

/**
 * @brief SoE状态：读响应。
 *
 * @param fsm 有限状态机
 * @param datagram 要使用的数据报
 *
 * @details
 * - 处理超时情况并重试获取邮箱数据报。
 * - 检查数据报状态，如果接收失败则设置错误状态并打印错误信息。
 * - 检查工作计数器，如果不为1则根据情况设置错误状态并打印错误信息。
 * - 清除邮箱读取锁。
 * - 设置状态为读响应数据。
 * - 调用读响应数据状态函数。
 */
void ec_fsm_soe_read_response(
    ec_fsm_soe_t *fsm,      /**< 有限状态机 */
    ec_datagram_t *datagram /**< 要使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;

    if (fsm->datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--)
    {
        ec_slave_mbox_prepare_fetch(slave, datagram); // 不会失败。
        return;
    }

    if (fsm->datagram->state != EC_DATAGRAM_RECEIVED)
    {
        fsm->state = ec_fsm_soe_error;
        ec_read_mbox_lock_clear(slave);
        EC_SLAVE_ERR(slave, "接收SoE读响应数据报失败：");
        ec_datagram_print_state(fsm->datagram);
        ec_fsm_soe_print_error(fsm);
        return;
    }

    if (fsm->datagram->working_counter != 1)
    {
        // 只有在数据没有被另一个读请求读取时才报错
        if (slave->mbox_soe_data.payload_size == 0)
        {
            fsm->state = ec_fsm_soe_error;
            ec_read_mbox_lock_clear(slave);
            EC_SLAVE_ERR(slave, "接收SoE读响应失败：");
            ec_datagram_print_wc_error(fsm->datagram);
            ec_fsm_soe_print_error(fsm);
            return;
        }
    }
    ec_read_mbox_lock_clear(slave);
    fsm->state = ec_fsm_soe_read_response_data;
    fsm->state(fsm, datagram);
}

/*****************************************************************************/

/**
 * @brief SoE状态：读响应数据。
 *
 * @param fsm 有限状态机
 * @param datagram 要使用的数据报
 *
 * @details
 * - 处理可用的数据或启动新的邮箱读取检查。
 * - 如果没有可用的数据，则启动新的邮箱读取检查。
 * - 检索邮箱数据并处理。
 * - 检查邮箱协议，如果不是SoE则设置错误状态并打印错误信息。
 * - 检查接收到的数据大小，如果小于SoE大小则设置错误状态并打印错误信息。
 * - 解析头部信息。
 * - 如果接收到错误响应，则设置错误代码并打印错误信息。
 * - 如果没有包含值，则设置错误状态并打印错误信息。
 * - 将数据追加到请求数据中。
 * - 如果数据不完整，则启动新的邮箱读取检查。
 * - 否则，设置状态为SoE结束。
 */
void ec_fsm_soe_read_response_data(
    ec_fsm_soe_t *fsm,      /**< 有限状态机 */
    ec_datagram_t *datagram /**< 要使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;
    ec_master_t *master = slave->master;
    uint8_t *data, mbox_prot, header, opcode, incomplete, error_flag,
        value_included;
    size_t rec_size, data_size;
    ec_soe_request_t *req = fsm->request;

    // 处理可用的数据或启动新的邮箱读取检查
    if (slave->mbox_soe_data.payload_size > 0)
    {
        slave->mbox_soe_data.payload_size = 0;
    }
    else
    {
        // 如果所需数据不可用，则启动新的邮箱读取检查
        if (ec_read_mbox_locked(slave))
        {
            // 等待当前读请求并将数据报标记为无效
            datagram->state = EC_DATAGRAM_INVALID;
        }
        else
        {
            ec_slave_mbox_prepare_check(slave, datagram); // 不会失败。
            fsm->state = ec_fsm_soe_read_check;
        }
        return;
    }

    data = ec_slave_mbox_fetch(slave, &slave->mbox_soe_data, &mbox_prot, &rec_size);
    if (IS_ERR(data))
    {
        fsm->state = ec_fsm_soe_error;
        ec_fsm_soe_print_error(fsm);
        return;
    }

    if (master->debug_level)
    {
        EC_SLAVE_DBG(slave, 0, "SSC读响应数据：\n");
        ec_print_data(data, rec_size);
    }

    if (mbox_prot != EC_MBOX_TYPE_SOE)
    {
        fsm->state = ec_fsm_soe_error;
        EC_SLAVE_ERR(slave, "接收到0x%02X作为响应的邮箱协议。\n", mbox_prot);
        ec_fsm_soe_print_error(fsm);
        return;
    }

    if (rec_size < EC_SOE_SIZE)
    {
        fsm->state = ec_fsm_soe_error;
        EC_SLAVE_ERR(slave, "接收到损坏的SoE读响应数据（%zu字节）！\n", rec_size);
        ec_print_data(data, rec_size);
        ec_fsm_soe_print_error(fsm);
        return;
    }

    header = EC_READ_U8(data);
    opcode = header & 0x7;
    incomplete = (header >> 3) & 1;
    error_flag = (header >> 4) & 1;

    if (opcode != OPCODE_READ_RESPONSE)
    {
        EC_SLAVE_ERR(slave, "接收到非读响应数据（操作码 %x）。\n", opcode);
        ec_print_data(data, rec_size);
        ec_fsm_soe_print_error(fsm);
        fsm->state = ec_fsm_soe_error;
        return;
    }

    if (error_flag)
    {
        req->error_code = EC_READ_U16(data + rec_size - 2);
        EC_SLAVE_ERR(slave, "接收到错误响应：\n");
        ec_print_soe_error(slave, req->error_code);
        ec_fsm_soe_print_error(fsm);
        fsm->state = ec_fsm_soe_error;
        return;
    }
    else
    {
        req->error_code = 0x0000;
    }

    value_included = (EC_READ_U8(data + 1) >> 6) & 1;
    if (!value_included)
    {
        EC_SLAVE_ERR(slave, "未包含值！\n");
        ec_fsm_soe_print_error(fsm);
        fsm->state = ec_fsm_soe_error;
        return;
    }

    data_size = rec_size - EC_SOE_SIZE;
    if (ec_soe_request_append_data(req, data + EC_SOE_SIZE, data_size))
    {
        fsm->state = ec_fsm_soe_error;
        ec_fsm_soe_print_error(fsm);
        return;
    }

    if (incomplete)
    {
        EC_SLAVE_DBG(slave, 1, "SoE数据不完整。等待偏移量为%zu的片段。\n", req->data_size);
        fsm->jiffies_start = fsm->datagram->jiffies_sent;
        ec_slave_mbox_prepare_check(slave, datagram); // 不会失败。
        fsm->retries = EC_FSM_RETRIES;
        fsm->state = ec_fsm_soe_read_check;
    }
    else
    {
        if (master->debug_level)
        {
            EC_SLAVE_DBG(slave, 0, "IDN数据：\n");
            ec_print_data(req->data, req->data_size);
        }

        fsm->state = ec_fsm_soe_end; // 成功
    }
}

/******************************************************************************
 * SoE write state machine
 *****************************************************************************/

/**
 * @brief SoE状态：写下一个片段。
 *
 * @param fsm 有限状态机
 * @param datagram 要使用的数据报
 *
 * @details
 * - 计算剩余数据大小和最大片段大小。
 * - 判断数据是否完整，计算剩余片段数。
 * - 准备发送邮箱数据报。
 * - 设置数据报头部信息。
 * - 将数据拷贝到邮箱数据报中。
 * - 如果启用了调试模式，则打印邮箱数据报。
 * - 记录发送时间。
 * - 设置状态为写请求。
 */
void ec_fsm_soe_write_next_fragment(
    ec_fsm_soe_t *fsm,      /**< 有限状态机 */
    ec_datagram_t *datagram /**< 要使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;
    ec_master_t *master = slave->master;
    ec_soe_request_t *req = fsm->request;
    uint8_t incomplete, *data;
    size_t max_fragment_size, remaining_size;
    uint16_t fragments_left;

    remaining_size = req->data_size - fsm->offset;
    max_fragment_size = slave->configured_rx_mailbox_size - EC_SOE_HEADER_SIZE;
    incomplete = remaining_size > max_fragment_size;
    fsm->fragment_size = incomplete ? max_fragment_size : remaining_size;
    fragments_left = remaining_size / fsm->fragment_size - 1;
    if (remaining_size % fsm->fragment_size)
    {
        fragments_left++;
    }

    data = ec_slave_mbox_prepare_send(slave, datagram, EC_MBOX_TYPE_SOE,
                                      EC_SOE_SIZE + fsm->fragment_size);
    if (IS_ERR(data))
    {
        fsm->state = ec_fsm_soe_error;
        ec_fsm_soe_print_error(fsm);
        return;
    }

    EC_WRITE_U8(data, OPCODE_WRITE_REQUEST | incomplete << 3 |
                          (req->drive_no & 0x07) << 5);
    EC_WRITE_U8(data + 1, 1 << 6); // 只包含值
    EC_WRITE_U16(data + 2, incomplete ? fragments_left : req->idn);
    memcpy(data + EC_SOE_SIZE, req->data + fsm->offset, fsm->fragment_size);

    if (master->debug_level)
    {
        EC_SLAVE_DBG(slave, 0, "SSC写请求：\n");
        ec_print_data(data, EC_SOE_SIZE + fsm->fragment_size);
    }

    req->jiffies_sent = jiffies;
    fsm->state = ec_fsm_soe_write_request;
}

/*****************************************************************************/

/**
 * @brief SoE状态：写开始。
 *
 * @param fsm 有限状态机
 * @param datagram 要使用的数据报
 *
 * @details
 * - 打印写入的IDN和驱动器信息。
 * - 检查从站是否支持SoE和SII数据是否可用。
 * - 检查邮箱大小是否足够。
 * - 初始化偏移量和重试次数。
 * - 调用写下一个片段函数。
 */
void ec_fsm_soe_write_start(
    ec_fsm_soe_t *fsm,      /**< 有限状态机 */
    ec_datagram_t *datagram /**< 要使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;
    ec_soe_request_t *req = fsm->request;

    EC_SLAVE_DBG(slave, 1, "正在写入IDN 0x%04X的驱动器%u（%zu字节）。\n",
                 req->idn, req->drive_no, req->data_size);

    if (!slave->sii_image)
    {
        EC_SLAVE_ERR(slave, "从站无法处理SoE写请求。SII数据不可用。\n");
        fsm->state = ec_fsm_soe_error;
        ec_fsm_soe_print_error(fsm);
        return;
    }

    if (!(slave->sii_image->sii.mailbox_protocols & EC_MBOX_SOE))
    {
        EC_SLAVE_ERR(slave, "从站不支持SoE！\n");
        fsm->state = ec_fsm_soe_error;
        ec_fsm_soe_print_error(fsm);
        return;
    }

    if (slave->configured_rx_mailbox_size <= EC_SOE_HEADER_SIZE)
    {
        EC_SLAVE_ERR(slave, "邮箱大小（%u字节）对于SoE写入来说太小。\n",
                     slave->configured_rx_mailbox_size);
        fsm->state = ec_fsm_soe_error;
        ec_fsm_soe_print_error(fsm);
        return;
    }

    fsm->offset = 0;
    fsm->retries = EC_FSM_RETRIES;
    ec_fsm_soe_write_next_fragment(fsm, datagram);
}

/*****************************************************************************/

/**
 * @brief SoE状态：写请求。
 *
 * @param fsm 有限状态机
 * @param datagram 要使用的数据报
 *
 * @details
 * - 处理超时情况并重试发送下一个片段。
 * - 检查数据报状态，如果接收失败则设置错误状态并打印错误信息。
 * - 计算发送时间差。
 * - 如果数据报的工作计数器不为1，则判断是否需要重发请求。
 * - 更新偏移量并判断是否发送完所有片段。
 * - 如果发送完所有片段，则开始查询响应。
 * - 如果邮箱已经被锁定，则设置状态为写响应数据并标记数据报为无效。
 * - 否则，准备邮箱读取检查。
 */
void ec_fsm_soe_write_request(
    ec_fsm_soe_t *fsm,      /**< 有限状态机 */
    ec_datagram_t *datagram /**< 要使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;
    unsigned long diff_ms;

    if (fsm->datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--)
    {
        ec_fsm_soe_write_next_fragment(fsm, datagram);
        return;
    }

    if (fsm->datagram->state != EC_DATAGRAM_RECEIVED)
    {
        fsm->state = ec_fsm_soe_error;
        EC_SLAVE_ERR(slave, "接收SoE写请求失败：");
        ec_datagram_print_state(fsm->datagram);
        ec_fsm_soe_print_error(fsm);
        return;
    }

    diff_ms = (jiffies - fsm->request->jiffies_sent) * 1000 / HZ;

    if (fsm->datagram->working_counter != 1)
    {
        if (!fsm->datagram->working_counter)
        {
            if (diff_ms < EC_SOE_RESPONSE_TIMEOUT)
            {
                // 没有响应；重新发送请求数据报
                ec_fsm_soe_write_next_fragment(fsm, datagram);
                return;
            }
        }
        fsm->state = ec_fsm_soe_error;
        EC_SLAVE_ERR(slave, "接收SoE写请求失败，经过%lu毫秒：", diff_ms);
        ec_datagram_print_wc_error(fsm->datagram);
        ec_fsm_soe_print_error(fsm);
        return;
    }

    // 片段成功发送
    fsm->offset += fsm->fragment_size;

    if (fsm->offset < fsm->request->data_size)
    {
        // 下一个片段
        fsm->retries = EC_FSM_RETRIES;
        ec_fsm_soe_write_next_fragment(fsm, datagram);
    }
    else
    {
        // 所有片段已发送；查询响应
        fsm->jiffies_start = fsm->datagram->jiffies_sent;

        // 如果邮箱已锁定，则跳过邮箱读取检查
        if (ec_read_mbox_locked(slave))
        {
            fsm->state = ec_fsm_soe_write_response_data;
            // 数据报不使用，标记为无效
            datagram->state = EC_DATAGRAM_INVALID;
        }
        else
        {
            ec_slave_mbox_prepare_check(slave, datagram); // 不会失败。
            fsm->retries = EC_FSM_RETRIES;
            fsm->state = ec_fsm_soe_write_check;
        }
    }
}

/*****************************************************************************/

/**
 * @brief SoE状态：写检查。
 *
 * @param fsm 有限状态机
 * @param datagram 要使用的数据报
 *
 * @details
 * - 处理超时情况并重试邮箱读取检查。
 * - 检查数据报状态，如果接收失败则设置错误状态并打印错误信息。
 * - 检查数据报的工作计数器，如果不为1则设置错误状态并打印错误信息。
 * - 检查数据是否已经被其他读请求接收。
 * - 计算时间差，如果超过响应超时时间，则设置错误状态并打印错误信息。
 * - 准备邮箱读取检查。
 */
void ec_fsm_soe_write_check(
    ec_fsm_soe_t *fsm,      /**< 有限状态机 */
    ec_datagram_t *datagram /**< 要使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;

    if (fsm->datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--)
    {
        ec_slave_mbox_prepare_check(slave, datagram); // 不会失败。
        return;
    }

    if (fsm->datagram->state != EC_DATAGRAM_RECEIVED)
    {
        fsm->state = ec_fsm_soe_error;
        ec_read_mbox_lock_clear(slave);
        EC_SLAVE_ERR(slave, "接收SoE写请求数据报失败：");
        ec_datagram_print_state(fsm->datagram);
        ec_fsm_soe_print_error(fsm);
        return;
    }

    if (fsm->datagram->working_counter != 1)
    {
        fsm->state = ec_fsm_soe_error;
        ec_read_mbox_lock_clear(slave);
        EC_SLAVE_ERR(slave, "接收SoE写请求数据报：");
        ec_datagram_print_wc_error(fsm->datagram);
        ec_fsm_soe_print_error(fsm);
        return;
    }

    if (!ec_slave_mbox_check(fsm->datagram))
    {
        unsigned long diff_ms = 0;

        // 检查数据是否已经被其他读请求接收
        if (slave->mbox_soe_data.payload_size > 0)
        {
            ec_read_mbox_lock_clear(slave);
            fsm->state = ec_fsm_soe_write_response_data;
            fsm->state(fsm, datagram);
            return;
        }

        diff_ms = (datagram->jiffies_received - fsm->jiffies_start) * 1000 / HZ;

        if (diff_ms >= EC_SOE_RESPONSE_TIMEOUT)
        {
            fsm->state = ec_fsm_soe_error;
            ec_read_mbox_lock_clear(slave);
            EC_SLAVE_ERR(slave, "等待写响应超时，经过%lu毫秒。\n", diff_ms);
            ec_fsm_soe_print_error(fsm);
            return;
        }

        ec_slave_mbox_prepare_check(slave, datagram); // 不会失败。
        fsm->retries = EC_FSM_RETRIES;
        return;
    }

    // 获取响应数据
    ec_slave_mbox_prepare_fetch(slave, datagram); // 不会失败。
    fsm->retries = EC_FSM_RETRIES;
    fsm->state = ec_fsm_soe_write_response;
}

/*****************************************************************************/

/**
 * @brief SoE状态：写响应。
 *
 * @param fsm 有限状态机
 * @param datagram 要使用的数据报
 *
 * @details
 * - 处理超时情况并重试邮箱读取。
 * - 检查数据报状态，如果接收失败则设置错误状态并打印错误信息。
 * - 检查数据报的工作计数器，如果不为1则设置错误状态并打印错误信息。
 * - 检查数据是否已经被其他读请求接收。
 * - 清除邮箱读取锁。
 * - 设置状态为写响应数据。
 * - 调用写响应数据状态函数。
 */
void ec_fsm_soe_write_response(
    ec_fsm_soe_t *fsm,      /**< 有限状态机 */
    ec_datagram_t *datagram /**< 要使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;

    if (fsm->datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--)
    {
        ec_slave_mbox_prepare_fetch(slave, datagram); // 不会失败。
        return;                                       // FIXME: request again?
    }

    if (fsm->datagram->state != EC_DATAGRAM_RECEIVED)
    {
        fsm->state = ec_fsm_soe_error;
        ec_read_mbox_lock_clear(slave);
        EC_SLAVE_ERR(slave, "接收SoE写响应数据报失败：");
        ec_datagram_print_state(fsm->datagram);
        ec_fsm_soe_print_error(fsm);
        return;
    }

    if (fsm->datagram->working_counter != 1)
    {
        // 只有在数据没有被另一个读请求读取时才报错
        if (slave->mbox_soe_data.payload_size == 0)
        {
            fsm->state = ec_fsm_soe_error;
            ec_read_mbox_lock_clear(slave);
            EC_SLAVE_ERR(slave, "接收SoE写响应失败：");
            ec_datagram_print_wc_error(fsm->datagram);
            ec_fsm_soe_print_error(fsm);
            return;
        }
    }
    ec_read_mbox_lock_clear(slave);
    fsm->state = ec_fsm_soe_write_response_data;
    fsm->state(fsm, datagram);
}

/*****************************************************************************/

/**
 * @brief SoE状态：写响应数据。
 *
 * @param fsm 有限状态机
 * @param datagram 要使用的数据报
 *
 * @details
 * - 处理可用的数据或启动新的邮箱读取检查。
 * - 如果没有可用数据，则启动新的邮箱读取检查。
 * - 如果邮箱已经被锁定，则等待当前读请求并将数据报标记为无效。
 * - 否则，准备邮箱读取检查。
 * - 获取数据，并检查邮箱协议、数据大小和操作码。
 * - 检查是否有错误标志，如果有则记录错误代码并设置状态为错误。
 * - 否则，将错误代码设置为0并设置状态为结束。
 */
void ec_fsm_soe_write_response_data(
    ec_fsm_soe_t *fsm,      /**< 有限状态机 */
    ec_datagram_t *datagram /**< 要使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;
    ec_master_t *master = slave->master;
    ec_soe_request_t *req = fsm->request;
    uint8_t *data, mbox_prot, opcode, error_flag;
    uint16_t idn;
    size_t rec_size;

    // 处理可用的数据或启动新的邮箱读取检查
    if (slave->mbox_soe_data.payload_size > 0)
    {
        slave->mbox_soe_data.payload_size = 0;
    }
    else
    {
        // 如果没有可用数据，则启动新的邮箱读取检查
        if (ec_read_mbox_locked(slave))
        {
            // 等待当前读请求并将数据报标记为无效
            datagram->state = EC_DATAGRAM_INVALID;
        }
        else
        {
            ec_slave_mbox_prepare_check(slave, datagram); // 不会失败。
            fsm->state = ec_fsm_soe_write_check;
        }
        return;
    }

    data = ec_slave_mbox_fetch(slave, &slave->mbox_soe_data, &mbox_prot, &rec_size);
    if (IS_ERR(data))
    {
        fsm->state = ec_fsm_soe_error;
        ec_fsm_soe_print_error(fsm);
        return;
    }

    if (master->debug_level)
    {
        EC_SLAVE_DBG(slave, 0, "SSC写响应：\n");
        ec_print_data(data, rec_size);
    }

    if (mbox_prot != EC_MBOX_TYPE_SOE)
    {
        fsm->state = ec_fsm_soe_error;
        EC_SLAVE_ERR(slave, "接收到0x%02X作为响应的邮箱协议。\n",
                     mbox_prot);
        ec_fsm_soe_print_error(fsm);
        return;
    }

    if (rec_size < EC_SOE_SIZE)
    {
        fsm->state = ec_fsm_soe_error;
        EC_SLAVE_ERR(slave, "接收到损坏的SoE写响应（%zu字节）！\n",
                     rec_size);
        ec_print_data(data, rec_size);
        ec_fsm_soe_print_error(fsm);
        return;
    }

    opcode = EC_READ_U8(data) & 0x7;
    if (opcode != OPCODE_WRITE_RESPONSE)
    {
        EC_SLAVE_ERR(slave, "未收到写响应（操作码 %x）。\n",
                     opcode);
        ec_print_data(data, rec_size);
        ec_fsm_soe_print_error(fsm);
        fsm->state = ec_fsm_soe_error;
        return;
    }

    idn = EC_READ_U16(data + 2);
    if (idn != req->idn)
    {
        EC_SLAVE_ERR(slave, "接收到错误的IDN 0x%04x的响应。\n",
                     idn);
        ec_print_data(data, rec_size);
        ec_fsm_soe_print_error(fsm);
        fsm->state = ec_fsm_soe_error;
        return;
    }

    error_flag = (EC_READ_U8(data) >> 4) & 1;
    if (error_flag)
    {
        if (rec_size < EC_SOE_SIZE + 2)
        {
            EC_SLAVE_ERR(slave, "接收到损坏的错误响应 - 错误标志已设置，但接收的大小为%zu。\n",
                         rec_size);
        }
        else
        {
            req->error_code = EC_READ_U16(data + EC_SOE_SIZE);
            EC_SLAVE_ERR(slave, "接收到错误响应：\n");
            ec_print_soe_error(slave, req->error_code);
        }
        ec_print_data(data, rec_size);
        ec_fsm_soe_print_error(fsm);
        fsm->state = ec_fsm_soe_error;
    }
    else
    {
        req->error_code = 0x0000;
        fsm->state = ec_fsm_soe_end; // 成功
    }
}

/*****************************************************************************/

/** SoE状态：错误。
 */
void ec_fsm_soe_error(
    ec_fsm_soe_t *fsm,      /**< 有限状态机 */
    ec_datagram_t *datagram /**< 要使用的数据报 */
)
{
}

/*****************************************************************************/

/** SoE状态：结束。
 */
void ec_fsm_soe_end(
    ec_fsm_soe_t *fsm,      /**< 有限状态机 */
    ec_datagram_t *datagram /**< 要使用的数据报 */
)
{
}

/*****************************************************************************/

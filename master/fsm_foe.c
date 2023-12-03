/******************************************************************************
 *
 *  $Id$
 *
 *  Copyright (C) 2008  Olav Zarges, imc Messsysteme GmbH
 *                2013  Florian Pose <fp@igh-essen.com>
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
 * EtherCAT FoE state machines.
 */

/*****************************************************************************/

#include "globals.h"
#include "master.h"
#include "mailbox.h"
#include "fsm_foe.h"

/*****************************************************************************/

/**
 * 读取字典时等待响应的最大时间（以jiffies为单位）。
 */
#define EC_FSM_FOE_TIMEOUT_JIFFIES (3 * HZ)

/**
 * FoE头部的大小。
 */
#define EC_FOE_HEADER_SIZE 6
// uint8_t  OpCode
// uint8_t  保留
// uint32_t PacketNo, Password, ErrorCode

// #define DEBUG_FOE

/*****************************************************************************/

/**
 * FoE操作码。
 */
enum
{
    EC_FOE_OPCODE_RRQ = 1,  /**< 读取请求。 */
    EC_FOE_OPCODE_WRQ = 2,  /**< 写入请求。 */
    EC_FOE_OPCODE_DATA = 3, /**< 数据。 */
    EC_FOE_OPCODE_ACK = 4,  /**< 确认。 */
    EC_FOE_OPCODE_ERR = 5,  /**< 错误。 */
    EC_FOE_OPCODE_BUSY = 6  /**< 忙。 */
};

/*****************************************************************************/

int ec_foe_prepare_data_send(ec_fsm_foe_t *, ec_datagram_t *);
int ec_foe_prepare_wrq_send(ec_fsm_foe_t *, ec_datagram_t *);
int ec_foe_prepare_rrq_send(ec_fsm_foe_t *, ec_datagram_t *);
int ec_foe_prepare_send_ack(ec_fsm_foe_t *, ec_datagram_t *);

void ec_foe_set_tx_error(ec_fsm_foe_t *, uint32_t);
void ec_foe_set_rx_error(ec_fsm_foe_t *, uint32_t);

void ec_fsm_foe_end(ec_fsm_foe_t *, ec_datagram_t *);
void ec_fsm_foe_error(ec_fsm_foe_t *, ec_datagram_t *);

void ec_fsm_foe_state_wrq_sent(ec_fsm_foe_t *, ec_datagram_t *);
void ec_fsm_foe_state_rrq_sent(ec_fsm_foe_t *, ec_datagram_t *);

void ec_fsm_foe_state_ack_check(ec_fsm_foe_t *, ec_datagram_t *);
void ec_fsm_foe_state_ack_read(ec_fsm_foe_t *, ec_datagram_t *);
void ec_fsm_foe_state_ack_read_data(ec_fsm_foe_t *, ec_datagram_t *);

void ec_fsm_foe_state_data_sent(ec_fsm_foe_t *, ec_datagram_t *);

void ec_fsm_foe_state_data_check(ec_fsm_foe_t *, ec_datagram_t *);
void ec_fsm_foe_state_data_read(ec_fsm_foe_t *, ec_datagram_t *);
void ec_fsm_foe_state_data_read_data(ec_fsm_foe_t *, ec_datagram_t *);
void ec_fsm_foe_state_sent_ack(ec_fsm_foe_t *, ec_datagram_t *);

void ec_fsm_foe_write_start(ec_fsm_foe_t *, ec_datagram_t *);
void ec_fsm_foe_read_start(ec_fsm_foe_t *, ec_datagram_t *);

/*****************************************************************************/

/**
@brief 函数作用：构造函数。
@param fsm 有限状态机。
@return 无返回值。
@details 初始化有限状态机的状态和数据报为空。
*/

void ec_fsm_foe_init(
    ec_fsm_foe_t *fsm /**< 有限状态机 */
)
{
    fsm->state = NULL;
    fsm->datagram = NULL;
}

/*****************************************************************************/

/**
@brief 函数作用：析构函数。
@param fsm 有限状态机。
@return 无返回值。
@details 该函数为空，表示在清除有限状态机时不执行任何操作。
*/

void ec_fsm_foe_clear(ec_fsm_foe_t *fsm /**< 有限状态机 */)
{
}

/*****************************************************************************/

/**
@brief 函数作用：执行有限状态机的当前状态。
@return 如果有限状态机仍在执行中，则返回1；否则返回0。
@details 调用当前状态的函数，如果有限状态机进入结束状态或错误状态，则返回0；否则返回1。
*/

int ec_fsm_foe_exec(
    ec_fsm_foe_t *fsm,      /**< 有限状态机 */
    ec_datagram_t *datagram /**< 使用的数据报 */
)
{
    if (fsm->state == ec_fsm_foe_end || fsm->state == ec_fsm_foe_error)
        return 0;

    fsm->state(fsm, datagram);

    if (fsm->state == ec_fsm_foe_end || fsm->state == ec_fsm_foe_error)
    {
        fsm->datagram = NULL;
        return 0;
    }

    fsm->datagram = datagram;
    return 1;
}

/*****************************************************************************/

/**
@brief 函数作用：判断有限状态机是否成功结束。
@return 如果有限状态机以成功结束，则返回非零值。
@details 如果有限状态机的状态为结束状态，则返回非零值，表示成功结束。
*/

int ec_fsm_foe_success(const ec_fsm_foe_t *fsm /**< 有限状态机 */)
{
    return fsm->state == ec_fsm_foe_end;
}

/*****************************************************************************/

/**
@brief 函数作用：准备一个FoE传输。
@return 如果成功则返回0，否则返回负数的错误代码。
@details 设置有限状态机的相关参数，并根据请求的方向选择相应的状态。
*/

void ec_fsm_foe_transfer(
    ec_fsm_foe_t *fsm,        /**< 状态机 */
    ec_slave_t *slave,        /**< EtherCAT从站 */
    ec_foe_request_t *request /**< SDO请求 */
)
{
    fsm->slave = slave;
    fsm->request = request;

    fsm->buffer_offset = 0;
    if (request->dir == EC_DIR_OUTPUT)
    {
        fsm->buffer_size = fsm->request->data_size;
        fsm->state = ec_fsm_foe_write_start;
    }
    else
    {
        fsm->buffer_size = fsm->request->buffer_size;
        fsm->state = ec_fsm_foe_read_start;
    }
}

/*****************************************************************************/

/**
@brief 函数作用：错误状态。
@param fsm 有限状态机。
@param datagram 使用的数据报。
@return 无返回值。
@details 该函数为空，表示在错误状态下不执行任何操作。
*/

void ec_fsm_foe_error(
    ec_fsm_foe_t *fsm,      /**< 有限状态机 */
    ec_datagram_t *datagram /**< 使用的数据报 */
)
{
#ifdef DEBUG_FOE
    EC_SLAVE_DBG(fsm->slave, 0, "%s()\n", __func__);
#endif
}

/*****************************************************************************/

/**
@brief 函数作用：结束状态。
@param fsm 有限状态机。
@param datagram 使用的数据报。
@return 无返回值。
@details 该函数为空，表示在结束状态下不执行任何操作。
*/

void ec_fsm_foe_end(
    ec_fsm_foe_t *fsm,      /**< 有限状态机 */
    ec_datagram_t *datagram /**< 使用的数据报 */
)
{
#ifdef DEBUG_FOE
    EC_SLAVE_DBG(fsm->slave, 0, "%s()\n", __func__);
#endif
}

/*****************************************************************************/

/**
@brief 函数作用：发送文件或下一个片段。
@return 如果成功则返回0，否则返回负数的错误代码。
@details 根据当前缓冲区的剩余大小和从站配置的发送邮箱大小，确定要发送的数据大小。然后准备发送数据的邮箱，并将数据写入邮箱。
*/

int ec_foe_prepare_data_send(
    ec_fsm_foe_t *fsm,      /**< 有限状态机 */
    ec_datagram_t *datagram /**< 使用的数据报 */
)
{
    size_t remaining_size, current_size;
    uint8_t *data;

    remaining_size = fsm->buffer_size - fsm->buffer_offset;
    current_size = fsm->slave->configured_tx_mailbox_size - EC_MBOX_HEADER_SIZE - EC_FOE_HEADER_SIZE;

    if (remaining_size < current_size)
    {
        current_size = remaining_size;
        fsm->last_packet = 1;
    }

    data = ec_slave_mbox_prepare_send(fsm->slave,
                                      datagram, EC_MBOX_TYPE_FOE, current_size + EC_FOE_HEADER_SIZE);
    if (IS_ERR(data))
    {
        return -1;
    }

    EC_WRITE_U16(data, EC_FOE_OPCODE_DATA); // OpCode = DataBlock req.
    EC_WRITE_U32(data + 2, fsm->packet_no); // PacketNo, Password
#ifdef DEBUG_FOE
    EC_SLAVE_DBG(fsm->slave, 0, "发送操作码 %u 数据包 %u\n",
                 EC_FOE_OPCODE_DATA, fsm->packet_no);
#endif

    memcpy(data + EC_FOE_HEADER_SIZE,
           fsm->request->buffer + fsm->buffer_offset, current_size);
    fsm->current_size = current_size;

    return 0;
}

/*****************************************************************************/

/**
@brief 函数作用：准备写请求（WRQ）并发送文件名。
@return 如果成功则返回0，否则返回负数的错误代码。
@details 准备写请求的数据，并将数据写入邮箱。
*/

int ec_foe_prepare_wrq_send(
    ec_fsm_foe_t *fsm,      /**< 有限状态机 */
    ec_datagram_t *datagram /**< 使用的数据报 */
)
{
    size_t current_size;
    uint8_t *data;

    current_size = strlen(fsm->request->file_name);

    data = ec_slave_mbox_prepare_send(fsm->slave, datagram,
                                      EC_MBOX_TYPE_FOE, current_size + EC_FOE_HEADER_SIZE);
    if (IS_ERR(data))
    {
        return -1;
    }

    EC_WRITE_U16(data, EC_FOE_OPCODE_WRQ);          // fsm写请求
    EC_WRITE_U32(data + 2, fsm->request->password); // 密码
#ifdef DEBUG_FOE
    EC_SLAVE_DBG(fsm->slave, 0, "发送操作码 %u\n",
                 EC_FOE_OPCODE_WRQ);
#endif

    memcpy(data + EC_FOE_HEADER_SIZE, fsm->request->file_name, current_size);

    return 0;
}

/*****************************************************************************/

/**
@brief 函数作用：初始化FoE写状态机。
@param fsm 有限状态机。
@param datagram 使用的数据报。
@return 无返回值。
@details 初始化FoE写状态机的相关参数，并检查从站是否支持FoE协议和SII数据是否可用。然后准备发送写请求的数据，并设置状态为写请求已发送。
*/

void ec_fsm_foe_write_start(
    ec_fsm_foe_t *fsm,      /**< 有限状态机 */
    ec_datagram_t *datagram /**< 使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;

    fsm->buffer_offset = 0;
    fsm->current_size = 0;
    fsm->packet_no = 0;
    fsm->last_packet = 0;

#ifdef DEBUG_FOE
    EC_SLAVE_DBG(fsm->slave, 0, "%s()\n", __func__);
#endif

    if (!slave->sii_image)
    {
        ec_foe_set_tx_error(fsm, FOE_BUSY);
        EC_SLAVE_ERR(slave, "从站无法处理FoE写请求。SII数据不可用。\n");
        return;
    }

    if (!(slave->sii_image->sii.mailbox_protocols & EC_MBOX_FOE))
    {
        ec_foe_set_tx_error(fsm, FOE_MBOX_PROT_ERROR);
        EC_SLAVE_ERR(slave, "从站不支持FoE协议！\n");
        return;
    }

    if (ec_foe_prepare_wrq_send(fsm, datagram))
    {
        ec_foe_set_tx_error(fsm, FOE_PROT_ERROR);
        return;
    }

    fsm->state = ec_fsm_foe_state_wrq_sent;
}

/*****************************************************************************/

/**
@brief 函数作用：检查确认。
@param fsm FoE状态机。
@param datagram 使用的数据报。
@return 无返回值。
@details 检查是否接收到确认的数据报，并进行相应的处理。如果接收到的数据报状态不是已接收状态，则设置接收错误并返回。如果接收到的数据报的工作计数器不为1，则设置接收错误并返回。如果从站还没有在邮箱中放入数据，则检查数据是否已被其他读请求接收。如果超时等待确认响应，则设置超时错误并返回。准备确认检查数据报，并设置重试次数。
*/

void ec_fsm_foe_state_ack_check(
    ec_fsm_foe_t *fsm,      /**< FoE状态机 */
    ec_datagram_t *datagram /**< 使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;

#ifdef DEBUG_FOE
    EC_SLAVE_DBG(fsm->slave, 0, "%s()\n", __func__);
#endif

    if (fsm->datagram->state != EC_DATAGRAM_RECEIVED)
    {
        ec_foe_set_rx_error(fsm, FOE_RECEIVE_ERROR);
        ec_read_mbox_lock_clear(slave);
        EC_SLAVE_ERR(slave, "接收FoE邮箱检查数据报失败：");
        ec_datagram_print_state(fsm->datagram);
        return;
    }

    if (fsm->datagram->working_counter != 1)
    {
        ec_foe_set_rx_error(fsm, FOE_WC_ERROR);
        ec_read_mbox_lock_clear(slave);
        EC_SLAVE_ERR(slave, "接收FoE邮箱检查数据报失败：");
        ec_datagram_print_wc_error(fsm->datagram);
        return;
    }

    if (!ec_slave_mbox_check(fsm->datagram))
    {
        // 从站尚未放入任何数据到邮箱中

        // 检查数据是否已被其他读请求接收
        if (slave->mbox_foe_data.payload_size > 0)
        {
            ec_read_mbox_lock_clear(slave);
            fsm->state = ec_fsm_foe_state_ack_read_data;
            fsm->state(fsm, datagram);
            return;
        }

        if (time_after(fsm->datagram->jiffies_received,
                       fsm->jiffies_start + EC_FSM_FOE_TIMEOUT_JIFFIES))
        {
            ec_foe_set_tx_error(fsm, FOE_TIMEOUT_ERROR);
            ec_read_mbox_lock_clear(slave);
            EC_SLAVE_ERR(slave, "等待确认响应超时。\n");
            return;
        }

        ec_slave_mbox_prepare_check(slave, datagram); // 不会失败
        fsm->retries = EC_FSM_RETRIES;
        return;
    }

    // 获取响应数据
    ec_slave_mbox_prepare_fetch(slave, datagram); // 不会失败

    fsm->retries = EC_FSM_RETRIES;
    fsm->state = ec_fsm_foe_state_ack_read;
}

/*****************************************************************************/

/**
@brief 函数作用：确认读操作。
@param fsm FoE状态机。
@param datagram 使用的数据报。
@return 无返回值。
@details 检查是否接收到确认的数据报，并进行相应的处理。如果接收到的数据报状态不是已接收状态，则设置接收错误并返回。如果接收到的数据报的工作计数器不为1，则只有在数据尚未被其他读请求读取时才设置接收错误并返回。
*/

void ec_fsm_foe_state_ack_read(
    ec_fsm_foe_t *fsm,      /**< FoE状态机 */
    ec_datagram_t *datagram /**< 使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;

#ifdef DEBUG_FOE
    EC_SLAVE_DBG(fsm->slave, 0, "%s()\n", __func__);
#endif

    if (fsm->datagram->state != EC_DATAGRAM_RECEIVED)
    {
        ec_foe_set_rx_error(fsm, FOE_RECEIVE_ERROR);
        ec_read_mbox_lock_clear(slave);
        EC_SLAVE_ERR(slave, "接收FoE确认响应数据报失败：");
        ec_datagram_print_state(fsm->datagram);
        return;
    }

    if (fsm->datagram->working_counter != 1)
    {
        // 只有当数据尚未被其他读请求读取时才设置接收错误
        if (slave->mbox_foe_data.payload_size == 0)
        {
            ec_foe_set_rx_error(fsm, FOE_WC_ERROR);
            ec_read_mbox_lock_clear(slave);
            EC_SLAVE_ERR(slave, "接收FoE确认响应失败：");
            ec_datagram_print_wc_error(fsm->datagram);
            return;
        }
    }
    ec_read_mbox_lock_clear(slave);
    fsm->state = ec_fsm_foe_state_ack_read_data;
    fsm->state(fsm, datagram);
}

/*****************************************************************************/

/**
@brief 函数作用：处理读操作。
@param fsm FoE状态机。
@param datagram 使用的数据报。
@return 无返回值。
@details 处理可用的数据或初始化新的邮箱读取检查。如果从站邮箱中的数据大小大于0，则将数据大小设置为0。否则，如果已锁定邮箱读取，则等待当前读请求并将数据报标记为无效。否则，准备邮箱读取检查，并设置状态为确认检查。如果数据报的状态不是已接收状态，则设置接收错误并返回。如果数据报的工作计数器不为1，则只有在数据尚未被其他读请求读取时才设置接收错误并返回。获取邮箱中的数据，并进行相应的处理。如果邮箱协议不是FoE协议，则设置协议错误并返回。如果接收到的操作码是忙碌码，则准备发送数据，并设置状态为数据已发送。如果接收到的操作码是错误码，则设置错误代码，并输出错误信息。如果接收到的操作码是确认码，则增加数据包编号和缓冲区偏移量，更新请求的进度。如果是最后一个数据包，则设置状态为结束状态。如果需要发送下一个数据包，则准备发送数据，并设置状态为数据已发送。
*/

void ec_fsm_foe_state_ack_read_data(
    ec_fsm_foe_t *fsm, / < FoE状态机 * /
                               ec_datagram_t * datagram /
                           < 使用的数据报 * /)
{
    ec_slave_t *slave = fsm->slave;
    uint8_t *data, mbox_prot;
    uint8_t opCode;
    size_t rec_size;

    // 处理可用的数据或初始化新的邮箱读取检查
    if (slave->mbox_foe_data.payload_size > 0)
    {
        slave->mbox_foe_data.payload_size = 0;
    }
    else
    {
        // 如果需要的数据不可用，则初始化新的邮箱读取检查
        if (ec_read_mbox_locked(slave))
        {
            // 等待当前读请求并标记数据报为无效
            datagram->state = EC_DATAGRAM_INVALID;
        }
        else
        {
            ec_slave_mbox_prepare_check(slave, datagram); // 不会失败
            fsm->state = ec_fsm_foe_state_ack_check;
        }
        return;
    }

    data = ec_slave_mbox_fetch(slave, &slave->mbox_foe_data, &mbox_prot, &rec_size);
    if (IS_ERR(data))
    {
        ec_foe_set_tx_error(fsm, FOE_PROT_ERROR);
        return;
    }

    if (mbox_prot != EC_MBOX_TYPE_FOE)
    {
        ec_foe_set_tx_error(fsm, FOE_MBOX_PROT_ERROR);
        EC_SLAVE_ERR(slave, "接收到的邮箱协议为0x%02X。\n", mbox_prot);
        return;
    }

    opCode = EC_READ_U8(data);
#ifdef DEBUG_FOE
    EC_SLAVE_DBG(fsm->slave, 0, "接收到的操作码 %u\n", opCode);
#endif

    if (opCode == EC_FOE_OPCODE_BUSY)
    {
        // 从站忙碌
        if (ec_foe_prepare_data_send(fsm, datagram))
        {
            ec_foe_set_tx_error(fsm, FOE_PROT_ERROR);
            EC_SLAVE_ERR(slave, "从站忙碌。\n");
            return;
        }
        fsm->state = ec_fsm_foe_state_data_sent;
        return;
    }

    if (opCode == EC_FOE_OPCODE_ERR)
    {
        fsm->request->error_code = EC_READ_U32(data + 2);
        EC_SLAVE_ERR(slave, "接收到FoE错误请求（代码0x%08x）。\n", fsm->request->error_code);
        if (rec_size > 6 && data[6])
        {
            uint8_t text[256];
            strncpy(text, data + 6, min(rec_size - 6, sizeof(text)));
            text[sizeof(text) - 1] = 0;
            EC_SLAVE_ERR(slave, "FoE错误文本：%s\n", text);
        }
        ec_foe_set_tx_error(fsm, FOE_OPCODE_ERROR);
        return;
    }

    if (opCode == EC_FOE_OPCODE_ACK)
    {
        fsm->packet_no++;
        fsm->buffer_offset += fsm->current_size;
        fsm->request->progress = fsm->buffer_offset;

        if (fsm->last_packet)
        {
            fsm->state = ec_fsm_foe_end;
            return;
        }

        if (ec_foe_prepare_data_send(fsm, datagram))
        {
            ec_foe_set_tx_error(fsm, FOE_PROT_ERROR);
            return;
        }
        fsm->state = ec_fsm_foe_state_data_sent;
        return;
    }
    ec_foe_set_tx_error(fsm, FOE_ACK_ERROR);
}

/*****************************************************************************/

/**
@brief 函数作用：状态：WRQ已发送。
@param fsm FoE状态机。
@param datagram 使用的数据报。
@return 无返回值。
@details 检查前一个传输数据报是否成功发送，并在需要时发送下一个片段。

- 检查前一个传输的数据报是否已成功接收，如果未成功接收，则设置接收错误并输出数据报的状态。
- 检查数据报的工作计数器是否为1，如果不为1，则表示从站尚未将任何数据放入邮箱中，设置接收错误并输出数据报的工作计数器错误信息。
- 设置起始时间为数据报发送的时间戳。
- 如果已锁定邮箱读取，则跳过邮箱读取检查，将状态设置为"ec_fsm_foe_state_ack_read_data"，并将数据报标记为无效。
- 否则，准备邮箱读取检查，设置重试次数为默认值，并将状态设置为"ec_fsm_foe_state_ack_check"。
*/

void ec_fsm_foe_state_wrq_sent(
    ec_fsm_foe_t *fsm,      /**< FoE状态机 */
    ec_datagram_t *datagram /**< 使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;

#ifdef DEBUG_FOE
    EC_SLAVE_DBG(fsm->slave, 0, "%s()\n", __func__);
#endif

    if (fsm->datagram->state != EC_DATAGRAM_RECEIVED)
    {
        ec_foe_set_rx_error(fsm, FOE_RECEIVE_ERROR);
        EC_SLAVE_ERR(slave, "发送FoE WRQ失败：");
        ec_datagram_print_state(fsm->datagram);
        return;
    }

    if (fsm->datagram->working_counter != 1)
    {
        // 从站尚未放入任何数据到邮箱中
        ec_foe_set_rx_error(fsm, FOE_WC_ERROR);
        EC_SLAVE_ERR(slave, "接收FoE WRQ失败：");
        ec_datagram_print_wc_error(fsm->datagram);
        return;
    }

    fsm->jiffies_start = fsm->datagram->jiffies_sent;

    // 如果已锁定邮箱读取，则跳过邮箱读取检查
    if (ec_read_mbox_locked(slave))
    {
        fsm->state = ec_fsm_foe_state_ack_read_data;
        // 数据报未使用并标记为无效
        datagram->state = EC_DATAGRAM_INVALID;
    }
    else
    {
        ec_slave_mbox_prepare_check(slave, datagram); // 不会失败
        fsm->retries = EC_FSM_RETRIES;
        fsm->state = ec_fsm_foe_state_ack_check;
    }
}

/*****************************************************************************/

/**
@brief 函数作用：状态：数据已发送。
@param fsm Foe状态机。
@param datagram 使用的数据报。
@return 无返回值。
@details 
该函数用于检查前一个传输的数据报是否已成功接收，并在需要时发送下一个片段。

- 检查前一个传输的数据报是否已成功接收，如果未成功接收，则设置发送错误并输出数据报的状态。
- 检查数据报的工作计数器是否为1，如果不为1，则表示从站尚未将任何数据放入邮箱中，设置发送错误并输出数据报的工作计数器错误信息。
- 设置起始时间为当前时间。
- 如果已锁定邮箱读取，则跳过邮箱读取检查，将状态设置为"ec_fsm_foe_state_ack_read_data"，并将数据报标记为无效。
- 否则，准备邮箱读取检查，设置重试次数为默认值，并将状态设置为"ec_fsm_foe_state_ack_check"。
*/

void ec_fsm_foe_state_data_sent(
    ec_fsm_foe_t *fsm,      /**< Foe状态机 */
    ec_datagram_t *datagram /**< 使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;

#ifdef DEBUG_FOE
    EC_SLAVE_DBG(fsm->slave, 0, "%s()\n", __func__);
#endif

    if (fsm->datagram->state != EC_DATAGRAM_RECEIVED)
    {
        ec_foe_set_tx_error(fsm, FOE_RECEIVE_ERROR);
        EC_SLAVE_ERR(slave, "接收FoE ack响应数据报失败：");
        ec_datagram_print_state(fsm->datagram);
        return;
    }

    if (fsm->datagram->working_counter != 1)
    {
        ec_foe_set_tx_error(fsm, FOE_WC_ERROR);
        EC_SLAVE_ERR(slave, "发送FoE数据失败：");
        ec_datagram_print_wc_error(fsm->datagram);
        return;
    }

    fsm->jiffies_start = jiffies;

    // 如果已锁定邮箱读取，则跳过邮箱读取检查
    if (ec_read_mbox_locked(slave))
    {
        fsm->state = ec_fsm_foe_state_ack_read_data;
        // 数据报未使用并标记为无效
        datagram->state = EC_DATAGRAM_INVALID;
    }
    else
    {
        ec_slave_mbox_prepare_check(slave, datagram);
        fsm->retries = EC_FSM_RETRIES;
        fsm->state = ec_fsm_foe_state_ack_check;
    }
}

/*****************************************************************************/

/**
@brief 函数作用：准备发送读请求（RRQ）并指定文件名。
@return 成功返回0，否则返回负数错误代码。
@details
该函数用于准备发送读请求（RRQ）并指定文件名。

- 获取文件名的长度。
- 调用ec_slave_mbox_prepare_send函数准备发送数据，并指定数据报类型为EC_MBOX_TYPE_FOE，数据长度为文件名长度加上EC_FOE_HEADER_SIZE。
- 如果返回值为错误代码，则返回-1。
- 在数据中写入操作码EC_FOE_OPCODE_RRQ（读请求）。
- 在数据中写入密码（fsm->request->password）。
- 将文件名复制到数据中。
- 如果从站的debug_level大于0，则输出调试信息。
- 返回0表示成功。
*/

int ec_foe_prepare_rrq_send(
    ec_fsm_foe_t *fsm,      /**< 有限状态机 */
    ec_datagram_t *datagram /**< 使用的数据报 */
)
{
    size_t current_size;
    uint8_t *data;

    current_size = strlen(fsm->request->file_name);

    data = ec_slave_mbox_prepare_send(fsm->slave, datagram,
                                      EC_MBOX_TYPE_FOE, current_size + EC_FOE_HEADER_SIZE);
    if (IS_ERR(data))
    {
        return -1;
    }

    EC_WRITE_U16(data, EC_FOE_OPCODE_RRQ);          // fsm读请求
    EC_WRITE_U32(data + 2, fsm->request->password); // 密码
    memcpy(data + EC_FOE_HEADER_SIZE, fsm->request->file_name, current_size);
#ifdef DEBUG_FOE
    EC_SLAVE_DBG(fsm->slave, 0, "发送操作码 %u\n", EC_FOE_OPCODE_RRQ);
#endif

    if (fsm->slave->master->debug_level)
    {
        EC_SLAVE_DBG(fsm->slave, 1, "FoE读请求：\n");
        ec_print_data(data, current_size + EC_FOE_HEADER_SIZE);
    }

    return 0;
}

/*****************************************************************************/

/**
@brief 函数作用：准备发送确认响应。
@return 成功返回0，否则返回负数错误代码。
@details
该函数用于准备发送确认响应。

- 调用ec_slave_mbox_prepare_send函数准备发送数据，并指定数据报类型为EC_MBOX_TYPE_FOE，数据长度为EC_FOE_HEADER_SIZE。
- 如果返回值为错误代码，则返回-1。
- 在数据中写入操作码EC_FOE_OPCODE_ACK（确认响应）。
- 在数据中写入数据包编号（fsm->packet_no）。
- 返回0表示成功。
*/

int ec_foe_prepare_send_ack(
    ec_fsm_foe_t *fsm,      /**< Foe状态机 */
    ec_datagram_t *datagram /**< 使用的数据报 */
)
{
    uint8_t *data;

    data = ec_slave_mbox_prepare_send(fsm->slave, datagram,
                                      EC_MBOX_TYPE_FOE, EC_FOE_HEADER_SIZE);
    if (IS_ERR(data))
    {
        return -1;
    }

    EC_WRITE_U16(data, EC_FOE_OPCODE_ACK);
    EC_WRITE_U32(data + 2, fsm->packet_no);
#ifdef DEBUG_FOE
    EC_SLAVE_DBG(fsm->slave, 0, "发送操作码 %u 数据包 %u\n",
                 EC_FOE_OPCODE_ACK, fsm->packet_no);
#endif

    return 0;
}

/*****************************************************************************/

/**
@brief 函数作用：状态：RRQ已发送。
@param fsm Foe状态机。
@param datagram 使用的数据报。
@return 无返回值。
@details 
该函数用于检查前一个传输的数据报是否已成功接收，并在需要时发送下一个片段。

- 检查前一个传输的数据报是否已成功接收，如果未成功接收，则设置接收错误并清除邮箱读取锁，并输出数据报的状态。
- 检查数据报的工作计数器是否为1，如果不为1，则表示从站尚未将任何数据放入邮箱中，设置接收错误并清除邮箱读取锁，并输出数据报的工作计数器错误信息。
- 设置起始时间为数据报的发送时间。
- 如果已锁定邮箱读取，则跳过邮箱读取检查，将状态设置为"ec_fsm_foe_state_data_read_data"，并将数据报标记为无效。
- 否则，准备邮箱读取检查，设置重试次数为默认值，并将状态设置为"ec_fsm_foe_state_data_check"。
*/

void ec_fsm_foe_state_rrq_sent(
    ec_fsm_foe_t *fsm,      /**< FoE状态机 */
    ec_datagram_t *datagram /**< 使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;

#ifdef DEBUG_FOE
    EC_SLAVE_DBG(fsm->slave, 0, "%s()\n", __func__);
#endif

    if (fsm->datagram->state != EC_DATAGRAM_RECEIVED)
    {
        ec_foe_set_rx_error(fsm, FOE_RECEIVE_ERROR);
        ec_read_mbox_lock_clear(slave);
        EC_SLAVE_ERR(slave, "发送FoE RRQ失败：");
        ec_datagram_print_state(fsm->datagram);
        return;
    }

    if (fsm->datagram->working_counter != 1)
    {
        // 从站尚未将任何数据放入邮箱中
        ec_foe_set_rx_error(fsm, FOE_WC_ERROR);
        ec_read_mbox_lock_clear(slave);
        EC_SLAVE_ERR(slave, "接收FoE RRQ失败：");
        ec_datagram_print_wc_error(fsm->datagram);
        return;
    }

    fsm->jiffies_start = fsm->datagram->jiffies_sent;

    // 如果已锁定邮箱读取，则跳过邮箱读取检查
    if (ec_read_mbox_locked(slave))
    {
        fsm->state = ec_fsm_foe_state_data_read_data;
        // 数据报未使用并标记为无效
        datagram->state = EC_DATAGRAM_INVALID;
    }
    else
    {
        ec_slave_mbox_prepare_check(slave, datagram); // 不会失败
        fsm->retries = EC_FSM_RETRIES;
        fsm->state = ec_fsm_foe_state_data_check;
    }
}

/*****************************************************************************/

/**
@brief 函数作用：读操作的起始状态。
@param fsm Foe状态机。
@param datagram 使用的数据报。
@return 无返回值。
@details
该函数用于开始读操作。

- 初始化缓冲区偏移量为0。
- 设置数据包编号为1。
- 设置最后一个数据包标志为0。
- 如果从站的sii_image为空，则设置接收错误为FOE_BUSY，并输出错误信息。
- 如果从站的sii_image的mailbox_protocols不支持EC_MBOX_FOE协议，则设置接收错误为FOE_MBOX_PROT_ERROR，并输出错误信息。
- 调用ec_foe_prepare_rrq_send函数准备发送读请求（RRQ），并检查返回值。
- 如果返回值为错误代码，则设置接收错误为FOE_PROT_ERROR。
- 将状态设置为ec_fsm_foe_state_rrq_sent。
*/

void ec_fsm_foe_read_start(
    ec_fsm_foe_t *fsm,      /**< Foe状态机 */
    ec_datagram_t *datagram /**< 使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;

    fsm->buffer_offset = 0;
    fsm->packet_no = 1;
    fsm->last_packet = 0;

#ifdef DEBUG_FOE
    EC_SLAVE_DBG(fsm->slave, 0, "%s()\n", __func__);
#endif

    if (!slave->sii_image)
    {
        ec_foe_set_rx_error(fsm, FOE_BUSY);
        EC_SLAVE_ERR(slave, "从站无法处理FoE读请求。SII数据不可用。\n");
        return;
    }

    if (!(slave->sii_image->sii.mailbox_protocols & EC_MBOX_FOE))
    {
        ec_foe_set_rx_error(fsm, FOE_MBOX_PROT_ERROR);
        EC_SLAVE_ERR(slave, "从站不支持FoE！\n");
        return;
    }

    if (ec_foe_prepare_rrq_send(fsm, datagram))
    {
        ec_foe_set_rx_error(fsm, FOE_PROT_ERROR);
        return;
    }

    fsm->state = ec_fsm_foe_state_rrq_sent;
}

/*****************************************************************************/

/**
@brief 函数作用：检查数据。
@param fsm Foe状态机。
@param datagram 使用的数据报。
@return 无返回值。
@details
该函数用于检查数据。

- 检查前一个传输的数据报是否已成功接收，如果未成功接收，则设置接收错误并清除邮箱读取锁，并输出数据报的状态。
- 检查数据报的工作计数器是否为1，如果不为1，则表示从站尚未将任何数据放入邮箱中，设置接收错误并清除邮箱读取锁，并输出数据报的工作计数器错误信息。
- 检查数据报是否已经被其他读请求接收到，如果是，则清除邮箱读取锁，将状态设置为"ec_fsm_foe_state_data_read_data"，并调用相应的函数。
- 如果超过了等待ack响应的超时时间，则设置发送错误为FOE_TIMEOUT_ERROR，并清除邮箱读取锁，并输出超时错误信息。
- 准备邮箱读取检查，设置重试次数为默认值。
- 将状态设置为"ec_fsm_foe_state_data_check"。
*/

void ec_fsm_foe_state_data_check(
    ec_fsm_foe_t *fsm,      /**< Foe状态机 */
    ec_datagram_t *datagram /**< 使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;

#ifdef DEBUG_FOE
    EC_SLAVE_DBG(fsm->slave, 0, "%s()\n", __func__);
#endif

    if (fsm->datagram->state != EC_DATAGRAM_RECEIVED)
    {
        ec_foe_set_rx_error(fsm, FOE_RECEIVE_ERROR);
        ec_read_mbox_lock_clear(slave);
        EC_SLAVE_ERR(slave, "发送FoE DATA READ失败：");
        ec_datagram_print_state(fsm->datagram);
        return;
    }

    if (fsm->datagram->working_counter != 1)
    {
        ec_foe_set_rx_error(fsm, FOE_WC_ERROR);
        ec_read_mbox_lock_clear(slave);
        EC_SLAVE_ERR(slave, "接收FoE DATA READ失败：");
        ec_datagram_print_wc_error(fsm->datagram);
        return;
    }

    if (!ec_slave_mbox_check(fsm->datagram))
    {
        // 检查数据是否已经被其他读请求接收到
        if (slave->mbox_foe_data.payload_size > 0)
        {
            ec_read_mbox_lock_clear(slave);
            fsm->state = ec_fsm_foe_state_data_read_data;
            fsm->state(fsm, datagram);
            return;
        }

        if (time_after(fsm->datagram->jiffies_received,
                       fsm->jiffies_start + EC_FSM_FOE_TIMEOUT_JIFFIES))
        {
            ec_foe_set_tx_error(fsm, FOE_TIMEOUT_ERROR);
            ec_read_mbox_lock_clear(slave);
            EC_SLAVE_ERR(slave, "等待ack响应超时。\n");
            return;
        }

        ec_slave_mbox_prepare_check(slave, datagram); // 不会失败
        fsm->retries = EC_FSM_RETRIES;
        return;
    }

    // 获取响应数据
    ec_slave_mbox_prepare_fetch(slave, datagram); // 不会失败

    fsm->retries = EC_FSM_RETRIES;
    fsm->state = ec_fsm_foe_state_data_read;
}

/*****************************************************************************/

/**
@brief 函数作用：开始读取数据。
@param fsm Foe状态机。
@param datagram 使用的数据报。
@return 无返回值。
@details
该函数用于开始读取数据。

- 检查前一个传输的数据报是否已成功接收，如果未成功接收，则设置接收错误并清除邮箱读取锁，并输出数据报的状态。
- 检查数据报的工作计数器是否为1，如果不为1，则表示从站尚未将任何数据放入邮箱中，设置接收错误并清除邮箱读取锁，并输出数据报的工作计数器错误信息。
- 如果数据报尚未被其他读请求接收到，则清除邮箱读取锁，将状态设置为"ec_fsm_foe_state_data_read_data"，并调用相应的函数。
- 如果数据报已经被其他读请求接收到，则检查是否超过了等待ack响应的超时时间。
- 如果超过了等待ack响应的超时时间，则设置发送错误为FOE_TIMEOUT_ERROR，并清除邮箱读取锁，并输出超时错误信息。
- 清除邮箱读取锁。
- 将状态设置为"ec_fsm_foe_state_data_read_data"。
- 调用相应的函数。
*/

void ec_fsm_foe_state_data_read(
    ec_fsm_foe_t *fsm,      /**< Foe状态机 */
    ec_datagram_t *datagram /**< 使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;

#ifdef DEBUG_FOE
    EC_SLAVE_DBG(fsm->slave, 0, "%s()\n", __func__);
#endif

    if (fsm->datagram->state != EC_DATAGRAM_RECEIVED)
    {
        ec_foe_set_rx_error(fsm, FOE_RECEIVE_ERROR);
        ec_read_mbox_lock_clear(slave);
        EC_SLAVE_ERR(slave, "接收FoE DATA READ数据报失败：");
        ec_datagram_print_state(fsm->datagram);
        return;
    }

    if (fsm->datagram->working_counter != 1)
    {
        // 只有数据未被其他读请求接收到时才会发生错误
        if (slave->mbox_foe_data.payload_size == 0)
        {
            ec_foe_set_rx_error(fsm, FOE_WC_ERROR);
            ec_read_mbox_lock_clear(slave);
            EC_SLAVE_ERR(slave, "接收FoE DATA READ失败：");
            ec_datagram_print_wc_error(fsm->datagram);
            return;
        }
    }
    ec_read_mbox_lock_clear(slave);
    fsm->state = ec_fsm_foe_state_data_read_data;
    fsm->state(fsm, datagram);
}

/*****************************************************************************/

/**
 * @brief 处理读取数据操作。
 * @param fsm FoE状态机。
 * @param datagram 要使用的数据报。
 * @return 无。
 * @details
 * - 处理可用的数据或启动新的邮箱读取检查。
 * - 如果slave->mbox_foe_data.payload_size大于0，则将其置为0。
 * - 否则，如果需要的数据不可用，则启动新的邮箱读取检查：
 *   - 如果ec_read_mbox_locked(slave)返回true，则等待当前读取请求，并将datagram标记为无效。
 *   - 否则，准备邮箱检查并将状态设置为ec_fsm_foe_state_data_check。
 * - 从slave中提取数据并检查其有效性：
 *   - 如果IS_ERR(data)为true，则将错误码设置为FOE_MBOX_FETCH_ERROR。
 *   - 如果mbox_prot不等于EC_MBOX_TYPE_FOE，则将错误码设置为FOE_PROT_ERROR。
 * - 解析接收到的操作码：
 *   - 如果opCode等于EC_FOE_OPCODE_BUSY，则表示设备忙：
 *     - 减少packet_no计数。
 *     - 如果ec_foe_prepare_send_ack(fsm, datagram)返回true，则将错误码设置为FOE_PROT_ERROR。
 *     - 否则，将状态设置为ec_fsm_foe_state_sent_ack。
 *   - 如果opCode等于EC_FOE_OPCODE_ERR，则表示接收到错误请求：
 *     - 将错误码设置为data + 2中的值。
 *     - 如果rec_size大于6且data[6]不为0，则将错误文本拷贝到text数组中。
 *     - 将错误码设置为FOE_OPCODE_ERROR。
 *   - 如果opCode不等于EC_FOE_OPCODE_DATA，则表示接收到错误的操作码：
 *     - 将错误码设置为0x00000000。
 *     - 将错误码设置为FOE_OPCODE_ERROR。
 * - 检查接收到的数据包编号是否与期望的packet_no相同：
 *   - 如果不相同，则将错误码设置为FOE_PACKETNO_ERROR。
 * - 计算有效数据的大小：
 *   - 减去EC_FOE_HEADER_SIZE。
 * - 如果fsm->buffer_size大于等于fsm->buffer_offset加上rec_size：
 *   - 将数据拷贝到fsm->request->buffer中。
 *   - 更新fsm->buffer_offset和fsm->request->progress。
 * - 设置fsm->last_packet的值：
 *   - 如果rec_size + EC_MBOX_HEADER_SIZE + EC_FOE_HEADER_SIZE不等于slave->configured_rx_mailbox_size，则设置为true。
 * - 检查是否为最后一个数据包或新的数据包适合已传递的缓冲区：
 *   - 如果是最后一个数据包或新的数据包适合已传递的缓冲区：
 *     - 如果ec_foe_prepare_send_ack(fsm, datagram)返回true，则将错误码设置为FOE_RX_DATA_ACK_ERROR。
 *     - 否则，将状态设置为ec_fsm_foe_state_sent_ack。
 *   - 否则，数据不适合接收缓冲区：
 *     - 设置错误码为FOE_READ_OVER_ERROR。
 */
void ec_fsm_foe_state_data_read_data(
    ec_fsm_foe_t *fsm,      /**< FoE状态机。 */
    ec_datagram_t *datagram /**< 要使用的数据报。 */
)
{
    ec_slave_t *slave = fsm->slave;
    size_t rec_size;
    uint8_t *data, opCode, packet_no, mbox_prot;

    // 处理可用的数据或启动新的邮箱读取检查
    if (slave->mbox_foe_data.payload_size > 0)
    {
        slave->mbox_foe_data.payload_size = 0;
    }
    else
    {
        // 如果需要的数据不可用，则启动新的邮箱读取检查
        if (ec_read_mbox_locked(slave))
        {
            // 等待当前读取请求并将datagram标记为无效
            datagram->state = EC_DATAGRAM_INVALID;
        }
        else
        {
            ec_slave_mbox_prepare_check(slave, datagram); // 不会失败
            fsm->state = ec_fsm_foe_state_data_check;
        }
        return;
    }

    data = ec_slave_mbox_fetch(slave, &slave->mbox_foe_data, &mbox_prot, &rec_size);
    if (IS_ERR(data))
    {
        ec_foe_set_rx_error(fsm, FOE_MBOX_FETCH_ERROR);
        return;
    }

    if (mbox_prot != EC_MBOX_TYPE_FOE)
    {
        EC_SLAVE_ERR(slave, "接收到的邮箱协议为0x%02X。\n",
                     mbox_prot);
        ec_foe_set_rx_error(fsm, FOE_PROT_ERROR);
        return;
    }

    opCode = EC_READ_U8(data);
#ifdef DEBUG_FOE
    EC_SLAVE_DBG(fsm->slave, 0, "接收到的操作码为%u。\n", opCode);
#endif

    if (opCode == EC_FOE_OPCODE_BUSY)
    {
        fsm->packet_no--;
        if (ec_foe_prepare_send_ack(fsm, datagram))
        {
            ec_foe_set_rx_error(fsm, FOE_PROT_ERROR);
        }
        else
        {
            fsm->state = ec_fsm_foe_state_sent_ack;
        }
#ifdef DEBUG_FOE
        EC_SLAVE_DBG(fsm->slave, 0, "%s() 忙碌。下一个数据包 %u。\n", __func__,
                     fsm->packet_no);
#endif
        return;
    }

    if (opCode == EC_FOE_OPCODE_ERR)
    {
        fsm->request->error_code = EC_READ_U32(data + 2);
        EC_SLAVE_ERR(slave, "接收到FoE错误请求（代码0x%08x）。\n",
                     fsm->request->error_code);
        if (rec_size > 6 && data[6])
        {
            uint8_t text[256];
            strncpy(text, data + 6, min(rec_size - 6, sizeof(text)));
            text[sizeof(text) - 1] = 0;
            EC_SLAVE_ERR(slave, "FoE错误文本：%s\n", text);
        }
        ec_foe_set_rx_error(fsm, FOE_OPCODE_ERROR);
        return;
    }

    if (opCode != EC_FOE_OPCODE_DATA)
    {
        EC_SLAVE_ERR(slave, "接收到的操作码为%x，期望为%x。\n",
                     opCode, EC_FOE_OPCODE_DATA);
        fsm->request->error_code = 0x00000000;
        ec_foe_set_rx_error(fsm, FOE_OPCODE_ERROR);
        return;
    }

    packet_no = EC_READ_U32(data + 2);
    if (packet_no != fsm->packet_no)
    {
        EC_SLAVE_ERR(slave, "接收到的数据包编号为%u，期望为%u。\n",
                     packet_no, fsm->packet_no);
        ec_foe_set_rx_error(fsm, FOE_PACKETNO_ERROR);
        return;
    }

    rec_size -= EC_FOE_HEADER_SIZE;

    if (fsm->buffer_size >= fsm->buffer_offset + rec_size)
    {
        memcpy(fsm->request->buffer + fsm->buffer_offset,
               data + EC_FOE_HEADER_SIZE, rec_size);
        fsm->buffer_offset += rec_size;
        fsm->request->progress = fsm->buffer_offset;
    }

    fsm->last_packet =
        (rec_size + EC_MBOX_HEADER_SIZE + EC_FOE_HEADER_SIZE != slave->configured_rx_mailbox_size);

    if (fsm->last_packet ||
        (slave->configured_rx_mailbox_size - EC_MBOX_HEADER_SIZE - EC_FOE_HEADER_SIZE + fsm->buffer_offset) <= fsm->buffer_size)
    {
        // 要么是最后一个数据包，要么新的数据包适合已传递的缓冲区
#ifdef DEBUG_FOE
        EC_SLAVE_DBG(fsm->slave, 0, "last_packet=true\n");
#endif
        if (ec_foe_prepare_send_ack(fsm, datagram))
        {
            ec_foe_set_rx_error(fsm, FOE_RX_DATA_ACK_ERROR);
            return;
        }

        fsm->state = ec_fsm_foe_state_sent_ack;
    }
    else
    {
        // 数据不适合接收缓冲区
        // ... 等待新的读取请求
        EC_SLAVE_ERR(slave, "数据不适合接收缓冲区！\n");
        printk("  rx_buffer_size = %d\n", fsm->buffer_size);
        printk("rx_buffer_offset = %d\n", fsm->buffer_offset);
        printk("        rec_size = %zd\n", rec_size);
        printk(" rx_mailbox_size = %d\n", slave->configured_rx_mailbox_size);
        printk("  rx_last_packet = %d\n", fsm->last_packet);
        fsm->request->data_size = fsm->buffer_offset;
        ec_foe_set_rx_error(fsm, FOE_READ_OVER_ERROR);
    }
}

/*****************************************************************************/

/**
 * @brief 发送确认信息。
 * @param fsm FoE状态机。
 * @param datagram 要使用的数据报。
 * @return 无。
 * @details
 * - 如果fsm->datagram->state不等于EC_DATAGRAM_RECEIVED，则设置接收错误并打印错误信息。
 * - 如果fsm->datagram->working_counter不等于1，则设置接收错误并打印工作计数错误信息。
 * - 设置fsm->jiffies_start为fsm->datagram->jiffies_sent。
 * - 如果是最后一个数据包：
 *   - 将packet_no重置为0。
 *   - 设置fsm->request->data_size为fsm->buffer_offset。
 *   - 将状态设置为ec_fsm_foe_end。
 * - 否则：
 *   - 增加packet_no计数。
 *   - 如果已经有一个读取请求正在进行，则将状态设置为ec_fsm_foe_state_data_read_data，并将datagram标记为无效。
 *   - 否则，准备邮箱检查并将状态设置为ec_fsm_foe_state_data_check。
 */

void ec_fsm_foe_state_sent_ack(
    ec_fsm_foe_t *fsm,      /**< FoE状态机。 */
    ec_datagram_t *datagram /**< 要使用的数据报。 */
)
{
    ec_slave_t *slave = fsm->slave;

#ifdef DEBUG_FOE
    EC_SLAVE_DBG(fsm->slave, 0, "%s()\n", __func__);
#endif

    if (fsm->datagram->state != EC_DATAGRAM_RECEIVED)
    {
        ec_foe_set_rx_error(fsm, FOE_RECEIVE_ERROR);
        EC_SLAVE_ERR(slave, "发送FoE ACK失败：");
        ec_datagram_print_state(fsm->datagram);
        return;
    }

    if (fsm->datagram->working_counter != 1)
    {
        // 从设备尚未将任何内容放入邮箱
        ec_foe_set_rx_error(fsm, FOE_WC_ERROR);
        EC_SLAVE_ERR(slave, "接收FoE ACK失败：");
        ec_datagram_print_wc_error(fsm->datagram);
        return;
    }

    fsm->jiffies_start = fsm->datagram->jiffies_sent;

    if (fsm->last_packet)
    {
        fsm->packet_no = 0;
        fsm->request->data_size = fsm->buffer_offset;
        fsm->state = ec_fsm_foe_end;
    }
    else
    {
        fsm->packet_no++;

        // 如果已经有一个读取请求正在进行，则跳过邮箱读取检查
        if (ec_read_mbox_locked(slave))
        {
            fsm->state = ec_fsm_foe_state_data_read_data;
            // 不使用datagram并将其标记为无效
            datagram->state = EC_DATAGRAM_INVALID;
        }
        else
        {
            ec_slave_mbox_prepare_check(slave, datagram); // 不会失败
            fsm->retries = EC_FSM_RETRIES;
            fsm->state = ec_fsm_foe_state_data_check;
        }
    }
}

/*****************************************************************************/

/**
 * @brief 设置错误码并进入发送错误状态。
 * @param fsm FoE状态机。
 * @param errorcode FoE错误码。
 * @return 无。
 */
void ec_foe_set_tx_error(
    ec_fsm_foe_t *fsm, /**< FoE状态机。 */
    uint32_t errorcode /**< FoE错误码。 */
)
{
    fsm->request->result = errorcode;
    fsm->state = ec_fsm_foe_error;
}

/*****************************************************************************/

/**
 * @brief 设置错误码并进入接收错误状态。
 * @param fsm FoE状态机。
 * @param errorcode FoE错误码。
 * @return 无。
 */
void ec_foe_set_rx_error(
    ec_fsm_foe_t *fsm, /**< FoE状态机。 */
    uint32_t errorcode /**< FoE错误码。 */
)
{
    fsm->request->result = errorcode;
    fsm->state = ec_fsm_foe_error;
}

/*****************************************************************************/

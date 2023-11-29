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
 * EtherCAT CoE state machines.
 */

/*****************************************************************************/

#include "globals.h"
#include "master.h"
#include "mailbox.h"
#include "fsm_coe.h"
#include "slave_config.h"

/*****************************************************************************/

/** Maximum time in ms to wait for responses when reading out the dictionary.
 */
#define EC_FSM_COE_DICT_TIMEOUT 1000

/** CoE download request header size.
 */
#define EC_COE_DOWN_REQ_HEADER_SIZE 10

/** CoE download segment request header size.
 */
#define EC_COE_DOWN_SEG_REQ_HEADER_SIZE 3

/** Minimum size of download segment.
 */
#define EC_COE_DOWN_SEG_MIN_DATA_SIZE 7

/** Enable debug output for CoE retries.
 */
#define DEBUG_RETRIES 0

/** Enable warning output if transfers take too long.
 */
#define DEBUG_LONG 0

/*****************************************************************************/

void ec_fsm_coe_dict_start(ec_fsm_coe_t *, ec_datagram_t *);
void ec_fsm_coe_dict_request(ec_fsm_coe_t *, ec_datagram_t *);
void ec_fsm_coe_dict_check(ec_fsm_coe_t *, ec_datagram_t *);
void ec_fsm_coe_dict_response(ec_fsm_coe_t *, ec_datagram_t *);
void ec_fsm_coe_dict_response_data(ec_fsm_coe_t *, ec_datagram_t *);
void ec_fsm_coe_dict_desc_request(ec_fsm_coe_t *, ec_datagram_t *);
void ec_fsm_coe_dict_desc_check(ec_fsm_coe_t *, ec_datagram_t *);
void ec_fsm_coe_dict_desc_response(ec_fsm_coe_t *, ec_datagram_t *);
void ec_fsm_coe_dict_desc_response_data(ec_fsm_coe_t *, ec_datagram_t *);
void ec_fsm_coe_dict_entry_request(ec_fsm_coe_t *, ec_datagram_t *);
void ec_fsm_coe_dict_entry_check(ec_fsm_coe_t *, ec_datagram_t *);
void ec_fsm_coe_dict_entry_response(ec_fsm_coe_t *, ec_datagram_t *);
void ec_fsm_coe_dict_entry_response_data(ec_fsm_coe_t *, ec_datagram_t *);

void ec_fsm_coe_down_start(ec_fsm_coe_t *, ec_datagram_t *);
void ec_fsm_coe_down_request(ec_fsm_coe_t *, ec_datagram_t *);
void ec_fsm_coe_down_check(ec_fsm_coe_t *, ec_datagram_t *);
void ec_fsm_coe_down_response(ec_fsm_coe_t *, ec_datagram_t *);
void ec_fsm_coe_down_response_data(ec_fsm_coe_t *, ec_datagram_t *);
void ec_fsm_coe_down_seg_check(ec_fsm_coe_t *, ec_datagram_t *);
void ec_fsm_coe_down_seg_response(ec_fsm_coe_t *, ec_datagram_t *);
void ec_fsm_coe_down_seg_response_data(ec_fsm_coe_t *, ec_datagram_t *);

void ec_fsm_coe_up_start(ec_fsm_coe_t *, ec_datagram_t *);
void ec_fsm_coe_up_request(ec_fsm_coe_t *, ec_datagram_t *);
void ec_fsm_coe_up_check(ec_fsm_coe_t *, ec_datagram_t *);
void ec_fsm_coe_up_response(ec_fsm_coe_t *, ec_datagram_t *);
void ec_fsm_coe_up_response_data(ec_fsm_coe_t *, ec_datagram_t *);
void ec_fsm_coe_up_seg_request(ec_fsm_coe_t *, ec_datagram_t *);
void ec_fsm_coe_up_seg_check(ec_fsm_coe_t *, ec_datagram_t *);
void ec_fsm_coe_up_seg_response(ec_fsm_coe_t *, ec_datagram_t *);
void ec_fsm_coe_up_seg_response_data(ec_fsm_coe_t *, ec_datagram_t *);

void ec_fsm_coe_end(ec_fsm_coe_t *, ec_datagram_t *);
void ec_fsm_coe_error(ec_fsm_coe_t *, ec_datagram_t *);

/*****************************************************************************/

/** SDO 中止消息。
 *
 * "中止 SDO 传输请求" 提供了一个中止代码，可以将其转换为明文。该表完成了代码和消息的映射。
 */
const ec_code_msg_t sdo_abort_messages[] = {
    {0x05030000, "切换位未改变"},
    {0x05040000, "SDO 协议超时"},
    {0x05040001, "客户端/服务器命令规范无效或未知"},
    {0x05040005, "内存不足"},
    {0x06010000, "不支持对对象进行访问"},
    {0x06010001, "尝试读取只写对象"},
    {0x06010002, "尝试写入只读对象"},
    {0x06020000, "该对象在对象目录中不存在"},
    {0x06040041, "无法映射对象到 PDO"},
    {0x06040042, "要映射的对象数量和长度将超过 PDO 长度"},
    {0x06040043, "一般参数不兼容的原因"},
    {0x06040047, "设备内部一般不兼容"},
    {0x06060000, "由于硬件错误而导致访问失败"},
    {0x06070010, "数据类型不匹配，服务参数的长度不匹配"},
    {0x06070012, "数据类型不匹配，服务参数的长度过高"},
    {0x06070013, "数据类型不匹配，服务参数的长度过低"},
    {0x06090011, "子索引不存在"},
    {0x06090030, "参数值超出范围"},
    {0x06090031, "参数值过高"},
    {0x06090032, "参数值过低"},
    {0x06090036, "最大值小于最小值"},
    {0x08000000, "一般错误"},
    {0x08000020, "无法将数据传输或存储到应用程序"},
    {0x08000021, "由于本地控制，无法将数据传输或存储到应用程序"},
    {0x08000022, "由于当前设备状态，无法将数据传输或存储到应用程序"},
    {0x08000023, "对象字典动态生成失败或不存在对象字典"},
    {}};

/*****************************************************************************/
/**
 * @brief 输出一个SDO中止消息。
 *
 * @param slave 从站。
 * @param abort_code 要查找的中止代码。
 *
 * @details 此函数用于输出一个SDO中止消息。
 * 首先，它会遍历SDO中止消息列表，查找与给定中止代码匹配的消息。
 * 如果找到匹配的消息，将打印相应的中止消息并返回。
 * 如果未找到匹配的消息，将打印未知的SDO中止代码并返回。
 */
void ec_canopen_abort_msg(
    const ec_slave_t *slave, /**< 从站。 */
    uint32_t abort_code      /**< 要查找的中止代码。 */
)
{
    const ec_code_msg_t *abort_msg;

    for (abort_msg = sdo_abort_messages; abort_msg->code; abort_msg++)
    {
        if (abort_msg->code == abort_code)
        {
            EC_SLAVE_ERR(slave, "SDO中止消息0x%08X: \"%s\"。\n",
                         abort_msg->code, abort_msg->message);
            return;
        }
    }

    EC_SLAVE_ERR(slave, "未知的SDO中止代码0x%08X。\n", abort_code);
}


/*****************************************************************************/

/**
 * @brief 构造函数。
 *
 * @param fsm 有限状态机。
 *
 * @details 此函数用于初始化有限状态机对象。
 * 将状态和数据报设置为NULL。
 */
void ec_fsm_coe_init(
    ec_fsm_coe_t *fsm /**< 有限状态机 */ 
)
{
    fsm->state = NULL;
    fsm->datagram = NULL;
}

/*****************************************************************************/

/**
 * @brief 析构函数。
 *
 * @param fsm 有限状态机。
 *
 * @details 此函数用于清除有限状态机对象。
 */
void ec_fsm_coe_clear(
    ec_fsm_coe_t *fsm /**< 有限状态机 */
)
{
}

/*****************************************************************************/

/**
 * @brief 开始读取从站的SDO字典。
 *
 * @param fsm 有限状态机。
 * @param slave EtherCAT从站。
 *
 * @details 此函数用于开始读取从站的SDO字典。
 * 将从站和状态设置为相应的值。
 */
void ec_fsm_coe_dictionary(
    ec_fsm_coe_t *fsm, /**< 有限状态机 */
    ec_slave_t *slave  /**< EtherCAT从站 */
)
{
    fsm->slave = slave;
    fsm->state = ec_fsm_coe_dict_start;
}


/*****************************************************************************/

/**
 * @brief 开始传输与从站的SDO相关的数据。
 *
 * @param fsm 有限状态机。
 * @param slave EtherCAT从站。
 * @param request SDO请求。
 *
 * @details 此函数用于开始传输与从站的SDO相关的数据。
 * 将从站和请求设置为相应的值。
 * 如果请求的方向是输出，将状态设置为向下开始。
 * 否则，将状态设置为向上开始。
 */
void ec_fsm_coe_transfer(
    ec_fsm_coe_t *fsm,        /**< 有限状态机。 */
    ec_slave_t *slave,        /**< EtherCAT从站。 */
    ec_sdo_request_t *request /**< SDO请求。 */
)
{
    fsm->slave = slave;
    fsm->request = request;

    if (request->dir == EC_DIR_OUTPUT)
    {
        fsm->state = ec_fsm_coe_down_start;
    }
    else
    {
        fsm->state = ec_fsm_coe_up_start;
    }
}

/*****************************************************************************/

/**
 * @brief 执行当前状态的有限状态机。
 *
 * @param fsm 有限状态机。
 * @param datagram 要使用的数据报。
 *
 * @return 如果有限状态机仍在进行中，则返回1，否则返回0。
 *
 * @details 此函数用于执行有限状态机的当前状态。
 * 如果有限状态机的状态为结束或错误，将返回0。
 * 否则，将调用相应的状态函数并返回1。
 */
int ec_fsm_coe_exec(
    ec_fsm_coe_t *fsm,      /**< 有限状态机。 */
    ec_datagram_t *datagram /**< 要使用的数据报。 */
)
{
    if (fsm->state == ec_fsm_coe_end || fsm->state == ec_fsm_coe_error)
        return 0;

    fsm->state(fsm, datagram);

    if (fsm->state == ec_fsm_coe_end || fsm->state == ec_fsm_coe_error)
    {
        fsm->datagram = NULL;
        return 0;
    }

    fsm->datagram = datagram;
    return 1;
}

/*****************************************************************************/

/**
 * @brief 返回有限状态机是否成功终止。
 *
 * @param fsm 有限状态机。
 *
 * @return 如果成功终止，则返回非零值。
 *
 * @details 此函数用于检查有限状态机是否成功终止。
 * 如果有限状态机的状态为结束，则返回非零值，否则返回0。
 */
int ec_fsm_coe_success(
    const ec_fsm_coe_t *fsm /**< 有限状态机 */
)
{
    return fsm->state == ec_fsm_coe_end;
}

/*****************************************************************************/

/**
 * @brief 检查接收到的数据是否为CoE紧急请求。
 *
 * @param fsm 有限状态机。
 * @param data CoE邮箱数据。
 * @param size CoE邮箱数据大小。
 *
 * @return 数据是否为紧急请求。
 *
 * @details 此函数用于检查接收到的数据是否为CoE紧急请求。
 * 如果检查结果为真，将输出紧急请求。
 * 如果数据大小小于2或((EC_READ_U16(data) >> 12) & 0x0F)不等于0x01，则返回0。
 * 如果数据大小小于10，则输出接收到不完整的CoE紧急请求。
 * 否则，将将紧急请求数据推入紧急请求环形缓冲区中，并输出紧急请求的详细信息。
 */
int ec_fsm_coe_check_emergency(
    ec_fsm_coe_t *fsm,   /**< 有限状态机 */
    const uint8_t *data, /**< CoE邮箱数据 */
    size_t size          /**< CoE邮箱数据大小 */
)
{
    if (size < 2 || ((EC_READ_U16(data) >> 12) & 0x0F) != 0x01)
        return 0;

    if (size < 10)
    {
        EC_SLAVE_WARN(fsm->slave, "接收到不完整的CoE紧急请求：\n");
        ec_print_data(data, size);
        return 1;
    }

    {
        ec_slave_config_t *sc = fsm->slave->config;
        if (sc)
        {
            ec_coe_emerg_ring_push(&sc->emerg_ring, data + 2);
        }
    }

    EC_SLAVE_WARN(fsm->slave, "接收到CoE紧急请求：\n"
                              "错误代码0x%04X，错误寄存器0x%02X，数据：\n",
                  EC_READ_U16(data + 2), EC_READ_U8(data + 4));
    ec_print_data(data + 5, 5);
    return 1;
}

/******************************************************************************
 *  CoE字典状态机
 *****************************************************************************/

/**
 * @brief 准备字典请求。
 *
 * @param fsm 有限状态机。
 * @param datagram 要使用的数据报。
 *
 * @return 如果成功，则返回0，否则返回负错误代码。
 *
 * @details 此函数用于准备字典请求。
 * 首先，它会调用ec_slave_mbox_prepare_send函数准备发送数据。
 * 如果准备失败，将返回相应的错误代码。
 * 然后，将SDO信息和Get OD List请求写入数据中。
 * 最后，将状态设置为字典请求。
 */
int ec_fsm_coe_prepare_dict(
    ec_fsm_coe_t *fsm,      /**< 有限状态机。 */
    ec_datagram_t *datagram /**< 要使用的数据报。 */
)
{
    ec_slave_t *slave = fsm->slave;
    uint8_t *data = ec_slave_mbox_prepare_send(slave, datagram,
                                               EC_MBOX_TYPE_COE, 8);
    if (IS_ERR(data))
    {
        return PTR_ERR(data);
    }

    EC_WRITE_U16(data, 0x8 << 12); // SDO信息
    EC_WRITE_U8(data + 2, 0x01);   // 获取OD列表请求
    EC_WRITE_U8(data + 3, 0x00);
    EC_WRITE_U16(data + 4, 0x0000);
    EC_WRITE_U16(data + 6, 0x0001); // 传递所有SDO！

    fsm->state = ec_fsm_coe_dict_request;
    return 0;
}

/*****************************************************************************/

/**
 * @brief CoE状态：字典开始。
 *
 * @param fsm 有限状态机。
 * @param datagram 要使用的数据报。
 *
 * @details 此函数用于CoE字典状态机的字典开始状态。
 * 首先，它会检查从站是否支持CoE字典请求。
 * 如果从站不支持，将设置状态为错误并返回。
 * 接下来，它会检查从站是否支持SDO信息服务。
 * 如果从站不支持，将设置状态为错误并返回。
 * 然后，将重试次数设置为默认值。
 * 最后，调用ec_fsm_coe_prepare_dict函数准备字典请求。
 * 如果准备失败，将设置状态为错误。
 */
void ec_fsm_coe_dict_start(
    ec_fsm_coe_t *fsm,      /**< 有限状态机。 */
    ec_datagram_t *datagram /**< 要使用的数据报。 */
)
{
    ec_slave_t *slave = fsm->slave;

    if (!slave->sii_image)
    {
        EC_SLAVE_ERR(slave, "从站无法处理CoE字典请求，SII数据不可用。\n");
        fsm->state = ec_fsm_coe_error;
        return;
    }

    if (!(slave->sii_image->sii.mailbox_protocols & EC_MBOX_COE))
    {
        EC_SLAVE_ERR(slave, "从站不支持CoE！\n");
        fsm->state = ec_fsm_coe_error;
        return;
    }

    if (slave->sii_image->sii.has_general && !slave->sii_image->sii.coe_details.enable_sdo_info)
    {
        EC_SLAVE_ERR(slave, "从站不支持SDO信息服务！\n");
        fsm->state = ec_fsm_coe_error;
        return;
    }

    fsm->retries = EC_FSM_RETRIES;

    if (ec_fsm_coe_prepare_dict(fsm, datagram))
    {
        fsm->state = ec_fsm_coe_error;
    }
}


/*****************************************************************************/

/**
 * @brief CoE状态：字典请求。
 *
 * @param fsm 有限状态机。
 * @param datagram 要使用的数据报。
 *
 * @details 此函数用于处理CoE状态机的字典请求状态。
 * 如果数据报的状态为超时且重试次数大于零，则重新准备字典请求并返回。
 * 如果数据报的状态不是接收到，将设置状态为错误并返回。
 * 如果数据报的工作计数器不等于1，将设置状态为错误并返回。
 * 如果邮箱读取请求已经在进行中，将设置状态为字典响应数据，并将数据报状态设置为无效。
 * 否则，将准备邮箱检查请求，并将重试次数设置为默认值，并将状态设置为字典检查。
 */
void ec_fsm_coe_dict_request(
    ec_fsm_coe_t *fsm,      /**< 有限状态机。 */
    ec_datagram_t *datagram /**< 要使用的数据报。 */
)
{
    ec_slave_t *slave = fsm->slave;

    if (fsm->datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--)
    {
        if (ec_fsm_coe_prepare_dict(fsm, datagram))
        {
            fsm->state = ec_fsm_coe_error;
        }
        return;
    }

    if (fsm->datagram->state != EC_DATAGRAM_RECEIVED)
    {
        fsm->state = ec_fsm_coe_error;
        EC_SLAVE_ERR(slave, "无法接收到CoE字典请求数据报：");
        ec_datagram_print_state(fsm->datagram);
        return;
    }

    if (fsm->datagram->working_counter != 1)
    {
        fsm->state = ec_fsm_coe_error;
        EC_SLAVE_ERR(slave, "接收CoE字典请求失败：");
        ec_datagram_print_wc_error(fsm->datagram);
        return;
    }

    fsm->jiffies_start = fsm->datagram->jiffies_sent;

    // 如果已经有邮箱读取请求在进行中，则跳过邮箱读取检查
    if (ec_read_mbox_locked(slave))
    {
        fsm->state = ec_fsm_coe_dict_response_data;
        // 数据报不会被使用，标记为无效
        datagram->state = EC_DATAGRAM_INVALID;
    }
    else
    {
        ec_slave_mbox_prepare_check(slave, datagram); // 不会失败
        fsm->retries = EC_FSM_RETRIES;
        fsm->state = ec_fsm_coe_dict_check;
    }
}


/*****************************************************************************/

/**
 * CoE状态: 字典检查。
 *
 * @param fsm 有限状态机。
 * @param datagram 要使用的数据报。
 *
 * @details 检查CoE字典是否正常。如果字典检查失败或超时，将更新状态机的状态并执行相应操作。
 *
 * 如果数据报状态为超时并且重试次数不为零，则准备CoE邮箱的检查操作，并返回。
 * 如果数据报状态不是已接收，则将状态机的状态设置为ec_fsm_coe_error，清除邮箱的读锁，并打印错误信息。
 * 如果工作计数器不为1，则将状态机的状态设置为ec_fsm_coe_error，清除邮箱的读锁，并打印错误信息。
 * 如果检查CoE邮箱失败，则进行以下操作：
 *     - 检查数据是否已被其他读取请求接收，如果是，则清除邮箱的读锁，将状态机的状态设置为ec_fsm_coe_dict_response_data，并调用相应的状态处理函数。
 *     - 计算接收到数据报的时间差，如果超过字典列表响应的超时时间，则将状态机的状态设置为ec_fsm_coe_error，清除邮箱的读锁，并打印超时错误信息。
 *     - 否则，准备CoE邮箱的检查操作，并更新重试次数。
 * 如果检查CoE邮箱成功，则准备CoE邮箱的数据提取操作，并更新重试次数和状态机的状态。
 *
 * 这个函数的目的是在CoE通信中确保字典的正确性，并根据不同的情况进行相应的处理。
 *
 * @return 无。
 */
void ec_fsm_coe_dict_check(
    ec_fsm_coe_t *fsm,      /**< 有限状态机。 */
    ec_datagram_t *datagram /**< 要使用的数据报。 */
)
{
    ec_slave_t *slave = fsm->slave;

    // 如果数据报状态为超时并且重试次数不为零
    if (fsm->datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--)
    {
        ec_slave_mbox_prepare_check(slave, datagram); // 不能失败。
        return;
    }

    // 如果数据报状态不是已接收
    if (fsm->datagram->state != EC_DATAGRAM_RECEIVED)
    {
        fsm->state = ec_fsm_coe_error;
        ec_read_mbox_lock_clear(slave);
        EC_SLAVE_ERR(slave, "接收CoE邮箱检查数据报失败：");
        ec_datagram_print_state(fsm->datagram);
        return;
    }

    // 如果工作计数器不为1
    if (fsm->datagram->working_counter != 1)
    {
        fsm->state = ec_fsm_coe_error;
        ec_read_mbox_lock_clear(slave);
        EC_SLAVE_ERR(slave, "接收CoE邮箱检查数据报失败：");
        ec_datagram_print_wc_error(fsm->datagram);
        return;
    }

    // 如果检查CoE邮箱失败
    if (!ec_slave_mbox_check(fsm->datagram))
    {
        unsigned long diff_ms = 0;

        // 检查数据是否已被其他读取请求接收
        if (slave->mbox_coe_data.payload_size > 0)
        {
            ec_read_mbox_lock_clear(slave);
            fsm->state = ec_fsm_coe_dict_response_data;
            fsm->state(fsm, datagram);
            return;
        }

        diff_ms = (fsm->datagram->jiffies_received - fsm->jiffies_start) *
                  1000 / HZ;

        // 如果超过字典列表响应的超时时间
        if (diff_ms >= EC_FSM_COE_DICT_TIMEOUT)
        {
            fsm->state = ec_fsm_coe_error;
            ec_read_mbox_lock_clear(slave);
            EC_SLAVE_ERR(slave, "等待SDO字典列表响应超时。\n");
            return;
        }

        ec_slave_mbox_prepare_check(slave, datagram); // 不能失败。
        fsm->retries = EC_FSM_RETRIES;
        return;
    }

    // 获取响应
    ec_slave_mbox_prepare_fetch(slave, datagram); // 不能失败。
    fsm->retries = EC_FSM_RETRIES;
    fsm->state = ec_fsm_coe_dict_response;
}


/*****************************************************************************/

/** 
 * 准备对象描述请求。
 *
 * \return 成功时返回零，否则返回负错误代码。
 *
 * @param fsm 有限状态机。
 * @param datagram 要使用的数据报。
 *
 * @details
 * 该函数用于准备一个对象描述请求。它将在给定的有限状态机和数据报上执行必要的操作，以发送对象描述请求。
 * 首先，它从有限状态机中获取从站对象(ec_slave_t)。
 * 然后，它调用ec_slave_mbox_prepare_send函数，为发送对象描述请求准备邮箱数据。
 * 如果该函数返回错误指针(IS_ERR(data))，则将返回相应的错误代码(PTR_ERR(data))。
 * 否则，它将在邮箱数据中写入特定的字节序列，以表示对象描述请求的类型和参数。
 * 最后，它将有限状态机的状态设置为ec_fsm_coe_dict_desc_request，并返回零表示成功。
 */
int ec_fsm_coe_dict_prepare_desc(
    ec_fsm_coe_t *fsm,      /**< 有限状态机。 */
    ec_datagram_t *datagram /**< 要使用的数据报。 */
)
{
    ec_slave_t *slave = fsm->slave;
    u8 *data = ec_slave_mbox_prepare_send(slave, datagram, EC_MBOX_TYPE_COE, 8);
    if (IS_ERR(data))
    {
        return PTR_ERR(data);
    }

    EC_WRITE_U16(data, 0x8 << 12); // SDO信息
    EC_WRITE_U8(data + 2, 0x03);   // 获取对象描述请求
    EC_WRITE_U8(data + 3, 0x00);
    EC_WRITE_U16(data + 4, 0x0000);
    EC_WRITE_U16(data + 6, fsm->sdo->index); // SDO索引

    fsm->state = ec_fsm_coe_dict_desc_request;
    return 0;
}

/*****************************************************************************/

/**
 * CoE状态：DICT RESPONSE。
 * \todo 超时行为
 *
 * @param fsm 有限状态机。
 * @param datagram 要使用的数据报。
 *
 * @details
 * 该函数用于处理CoE字典响应状态。根据不同的情况，它执行相应的操作。
 * 如果数据报状态为超时并且重试次数不为零，它将准备数据报的提取操作，并返回。
 * 如果数据报状态不是已接收，它将更新状态机的状态为ec_fsm_coe_error，清除邮箱的读锁，并打印错误信息。
 * 如果工作计数器不为1，它将检查数据是否已被其他读取请求读取，如果是，则清除邮箱的读锁，将状态机的状态设置为ec_fsm_coe_dict_response_data，并调用相应的状态处理函数。
 * 如果数据尚未被其他读取请求读取，它将将状态机的状态设置为ec_fsm_coe_error，清除邮箱的读锁，并打印错误信息。
 * 最后，它清除邮箱的读锁，将状态机的状态设置为ec_fsm_coe_dict_response_data，并调用相应的状态处理函数。
 */
void ec_fsm_coe_dict_response(
    ec_fsm_coe_t *fsm,      /**< 有限状态机。 */
    ec_datagram_t *datagram /**< 要使用的数据报。 */
)
{
    ec_slave_t *slave = fsm->slave;

    if (fsm->datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--)
    {
        ec_slave_mbox_prepare_fetch(slave, datagram); // 不能失败。
        return;
    }

    if (fsm->datagram->state != EC_DATAGRAM_RECEIVED)
    {
        fsm->state = ec_fsm_coe_error;
        ec_read_mbox_lock_clear(slave);
        EC_SLAVE_ERR(slave, "接收CoE字典响应数据报失败：");
        ec_datagram_print_state(fsm->datagram);
        return;
    }

    if (fsm->datagram->working_counter != 1)
    {
        // 仅当数据尚未被其他读取请求读取时才报错
        if (slave->mbox_coe_data.payload_size == 0)
        {
            fsm->state = ec_fsm_coe_error;
            ec_read_mbox_lock_clear(slave);
            EC_SLAVE_ERR(slave, "接收CoE字典响应失败：");
            ec_datagram_print_wc_error(fsm->datagram);
            return;
        }
    }
    ec_read_mbox_lock_clear(slave);
    fsm->state = ec_fsm_coe_dict_response_data;
    fsm->state(fsm, datagram);
}
/*****************************************************************************/

/** 
 * CoE状态: DICT RESPONSE DATA.
 *
 * @param fsm 有限状态机。
 * @param datagram 要使用的数据报。
 *
 * @details
 * 该函数用于处理CoE字典响应数据状态。根据不同的情况，它执行相应的操作。
 * 如果邮箱数据的有效载荷大小大于零，则将其大小设置为零。
 * 否则，如果邮箱数据不可用，则初始化一个新的邮箱读取检查。
 * 如果邮箱读取锁定，则等待当前读取请求，并将数据报状态设置为EC_DATAGRAM_INVALID。
 * 否则，准备邮箱检查操作，并将状态机的状态设置为ec_fsm_coe_dict_check。
 * 
 * 如果成功获取邮箱数据，并且邮箱协议为EC_MBOX_TYPE_COE，则进行进一步处理。
 * 首先，检查是否接收到紧急消息，如果是，则再次检查CoE响应。
 * 如果接收到的数据大小小于3个字节，则打印错误信息并将状态机的状态设置为ec_fsm_coe_error。
 * 如果接收到的数据不是SDO信息或Get OD List响应，则打印错误信息并将状态机的状态设置为ec_fsm_coe_dict_check。
 * 
 * 如果是第一个段的数据，则设置索引列表偏移量为8，否则设置为6。
 * 如果接收到的数据大小小于索引列表偏移量或数据大小为奇数，则打印错误信息并将状态机的状态设置为ec_fsm_coe_error。
 * 
 * 根据接收到的索引列表，创建SDO对象并添加到从站的SDO字典中。
 * 如果还有未接收的SDO列表片段，则打印调试信息。
 * 如果接收到的数据中标志位指示还有更多的消息等待，则再次进行邮箱检查。
 * 
 * 如果SDO字典为空，则表示没有SDO对象，将状态机的状态设置为ec_fsm_coe_end表示成功。
 * 否则，将状态机的状态设置为ec_fsm_coe_dict_desc_request，并继续处理SDO描述请求。
 */
void ec_fsm_coe_dict_response_data(
    ec_fsm_coe_t *fsm,      /**< 有限状态机。 */
    ec_datagram_t *datagram /**< 要使用的数据报。 */
)
{
    ec_slave_t *slave = fsm->slave;
    uint8_t *data, mbox_prot;
    size_t rec_size;
    unsigned int sdo_count, i;
    uint16_t sdo_index, fragments_left;
    ec_sdo_t *sdo;
    bool first_segment;
    size_t index_list_offset;

    // 处理可用的数据或启动新的邮箱读取检查
    if (slave->mbox_coe_data.payload_size > 0)
    {
        slave->mbox_coe_data.payload_size = 0;
    }
    else
    {
        // 如果所需的数据不可用，则启动新的邮箱读取检查
        if (ec_read_mbox_locked(slave))
        {
            // 等待当前读取请求并将数据报标记为无效
            datagram->state = EC_DATAGRAM_INVALID;
        }
        else
        {
            ec_slave_mbox_prepare_check(slave, datagram); // 不能失败。
            fsm->state = ec_fsm_coe_dict_check;
        }
        return;
    }

    data = ec_slave_mbox_fetch(slave, &slave->mbox_coe_data, &mbox_prot, &rec_size);
    if (IS_ERR(data))
    {
        fsm->state = ec_fsm_coe_error;
        return;
    }

    if (mbox_prot != EC_MBOX_TYPE_COE)
    {
        EC_SLAVE_ERR(slave, "接收到协议为 0x%02X 的邮箱响应。\n",
                     mbox_prot);
        fsm->state = ec_fsm_coe_error;
        return;
    }

    if (ec_fsm_coe_check_emergency(fsm, data, rec_size))
    {
        // 再次检查CoE响应
        ec_slave_mbox_prepare_check(slave, datagram); // 不能失败。
        fsm->retries = EC_FSM_RETRIES;
        fsm->state = ec_fsm_coe_dict_check;
        return;
    }

    if (rec_size < 3)
    {
        EC_SLAVE_ERR(slave, "接收到损坏的SDO字典响应 (大小 %zu)。\n",
                     rec_size);
        fsm->state = ec_fsm_coe_error;
        return;
    }

    if (EC_READ_U16(data) >> 12 == 0x8 && // SDO信息
        (EC_READ_U8(data + 2) & 0x7F) == 0x07)
    { // 错误响应
        EC_SLAVE_ERR(slave, "SDO信息错误响应！\n");
        if (rec_size < 10)
        {
            EC_SLAVE_ERR(slave, "不完整的SDO信息错误响应：\n");
            ec_print_data(data, rec_size);
        }
        else
        {
            ec_canopen_abort_msg(slave, EC_READ_U32(data + 6));
        }
        fsm->state = ec_fsm_coe_error;
        return;
    }

    if (EC_READ_U16(data) >> 12 != 0x8 || // SDO信息
        (EC_READ_U8(data + 2) & 0x7F) != 0x02)
    { // 获取OD列表响应
        if (fsm->slave->master->debug_level)
        {
            EC_SLAVE_DBG(slave, 1, "无效的SDO列表响应！正在重试...\n");
            ec_print_data(data, rec_size);
        }
        ec_slave_mbox_prepare_check(slave, datagram); // 不能失败。
        fsm->retries = EC_FSM_RETRIES;
        fsm->state = ec_fsm_coe_dict_check;
        return;
    }

    first_segment = list_empty(&slave->sdo_dictionary) ? true : false;
    index_list_offset = first_segment ? 8 : 6;

    if (rec_size < index_list_offset || rec_size % 2)
    {
        EC_SLAVE_ERR(slave, "无效的数据大小 %zu！\n", rec_size);
        ec_print_data(data, rec_size);
        fsm->state = ec_fsm_coe_error;
        return;
    }

    sdo_count = (rec_size - index_list_offset) / 2;

    for (i = 0; i < sdo_count; i++)
    {
        sdo_index = EC_READ_U16(data + index_list_offset + i * 2);
        if (!sdo_index)
        {
            EC_SLAVE_DBG(slave, 1, "SDO字典包含索引 0x0000。\n");
            continue;
        }

        if (!(sdo = (ec_sdo_t *)kmalloc(sizeof(ec_sdo_t), GFP_KERNEL)))
        {
            EC_SLAVE_ERR(slave, "为SDO分配内存失败！\n");
            fsm->state = ec_fsm_coe_error;
            return;
        }

        ec_sdo_init(sdo, slave, sdo_index);
        list_add_tail(&sdo->list, &slave->sdo_dictionary);
    }

    fragments_left = EC_READ_U16(data + 4);
    if (fragments_left)
    {
        EC_SLAVE_DBG(slave, 1, "SDO列表剩余片段：%u\n",
                     fragments_left);
    }

    if (EC_READ_U8(data + 2) & 0x80 || fragments_left)
    {
        // 还有更多的消息等待，再次进行邮箱检查
        fsm->jiffies_start = fsm->datagram->jiffies_sent;
        ec_slave_mbox_prepare_check(slave, datagram); // 不能失败。
        fsm->retries = EC_FSM_RETRIES;
        fsm->state = ec_fsm_coe_dict_check;
        return;
    }

    if (list_empty(&slave->sdo_dictionary))
    {
        // 字典中没有SDO对象，处理完成
        fsm->state = ec_fsm_coe_end; // 成功
        return;
    }

    // 获取SDO描述
    fsm->sdo = list_entry(slave->sdo_dictionary.next, ec_sdo_t, list);

    fsm->retries = EC_FSM_RETRIES;
    if (ec_fsm_coe_dict_prepare_desc(fsm, datagram))
    {
        fsm->state = ec_fsm_coe_error;
    }
}

/*****************************************************************************/

/**
 * CoE状态: DICT DESC REQUEST.
 * \todo 超时行为
 *
 * @param fsm 有限状态机。
 * @param datagram 要使用的数据报。
 *
 * @details
 * 该函数用于处理CoE字典描述请求状态。根据不同的情况，它执行相应的操作。
 * 如果数据报状态为超时并且重试次数不为零，则准备SDO描述请求并返回。
 * 如果数据报状态不是已接收，则将状态机的状态设置为ec_fsm_coe_error，并打印错误信息。
 * 如果数据报的工作计数器不为1，则将状态机的状态设置为ec_fsm_coe_error，并打印错误信息。
 * 
 * 如果成功准备了SDO描述请求，则将状态机的状态设置为ec_fsm_coe_dict_desc_response_data。
 */
void ec_fsm_coe_dict_desc_request(
    ec_fsm_coe_t *fsm,      /**< 有限状态机。 */
    ec_datagram_t *datagram /**< 要使用的数据报。 */
)
{
    ec_slave_t *slave = fsm->slave;

    if (fsm->datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--)
    {
        if (ec_fsm_coe_dict_prepare_desc(fsm, datagram))
        {
            fsm->state = ec_fsm_coe_error;
        }
        return;
    }

    if (fsm->datagram->state != EC_DATAGRAM_RECEIVED)
    {
        fsm->state = ec_fsm_coe_error;
        EC_SLAVE_ERR(slave, "接收CoE SDO描述请求数据报失败：");
        ec_datagram_print_state(fsm->datagram);
        return;
    }

    if (fsm->datagram->working_counter != 1)
    {
        fsm->state = ec_fsm_coe_error;
        EC_SLAVE_ERR(slave, "接收CoE SDO描述请求失败：");
        ec_datagram_print_wc_error(fsm->datagram);
        return;
    }

    fsm->jiffies_start = fsm->datagram->jiffies_sent;

    // 如果已经有一个读取请求正在进行，则跳过邮箱读取检查
    if (ec_read_mbox_locked(slave))
    {
        fsm->state = ec_fsm_coe_dict_desc_response_data;
        // 数据报不使用并标记为无效
        datagram->state = EC_DATAGRAM_INVALID;
    }
    else
    {
        ec_slave_mbox_prepare_check(slave, datagram); // 不能失败。
        fsm->retries = EC_FSM_RETRIES;
        fsm->state = ec_fsm_coe_dict_desc_check;
    }
}

/*****************************************************************************/

/**
 * CoE状态: DICT DESC CHECK.
 *
 * @param fsm 有限状态机。
 * @param datagram 要使用的数据报。
 *
 * @details
 * 该函数用于处理CoE字典描述检查状态。根据不同的情况，它执行相应的操作。
 * 如果数据报状态为超时并且重试次数不为零，则准备邮箱检查操作并返回。
 * 如果数据报状态不是已接收，则将状态机的状态设置为ec_fsm_coe_error，并打印错误信息。
 * 如果数据报的工作计数器不为1，则将状态机的状态设置为ec_fsm_coe_error，并打印错误信息。
 * 
 * 如果成功接收到邮箱检查数据报，则将状态机的状态设置为ec_fsm_coe_dict_desc_response。
 */
void ec_fsm_coe_dict_desc_check(
    ec_fsm_coe_t *fsm,      /**< 有限状态机。 */
    ec_datagram_t *datagram /**< 要使用的数据报。 */
)
{
    ec_slave_t *slave = fsm->slave;

    if (fsm->datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--)
    {
        ec_slave_mbox_prepare_check(slave, datagram); // 不能失败。
        return;
    }

    if (fsm->datagram->state != EC_DATAGRAM_RECEIVED)
    {
        fsm->state = ec_fsm_coe_error;
        ec_read_mbox_lock_clear(slave);
        EC_SLAVE_ERR(slave, "接收CoE邮箱检查数据报失败：");
        ec_datagram_print_state(fsm->datagram);
        return;
    }

    if (fsm->datagram->working_counter != 1)
    {
        fsm->state = ec_fsm_coe_error;
        ec_read_mbox_lock_clear(slave);
        EC_SLAVE_ERR(slave, "接收CoE邮箱检查数据报失败：");
        ec_datagram_print_wc_error(fsm->datagram);
        return;
    }

    if (!ec_slave_mbox_check(fsm->datagram))
    {
        unsigned long diff_ms = 0;

        // 检查数据是否已经被其他读取请求接收
        if (slave->mbox_coe_data.payload_size > 0)
        {
            ec_read_mbox_lock_clear(slave);
            fsm->state = ec_fsm_coe_dict_desc_response_data;
            fsm->state(fsm, datagram);
            return;
        }

        diff_ms = (fsm->datagram->jiffies_received - fsm->jiffies_start) *
                  1000 / HZ;

        if (diff_ms >= EC_FSM_COE_DICT_TIMEOUT)
        {
            fsm->state = ec_fsm_coe_error;
            ec_read_mbox_lock_clear(slave);
            EC_SLAVE_ERR(slave, "等待SDO 0x%04x对象描述响应超时。\n",
                         fsm->sdo->index);
            return;
        }

        ec_slave_mbox_prepare_check(slave, datagram); // 不能失败。
        fsm->retries = EC_FSM_RETRIES;
        return;
    }

    // 获取响应数据
    ec_slave_mbox_prepare_fetch(slave, datagram); // 不能失败。
    fsm->retries = EC_FSM_RETRIES;
    fsm->state = ec_fsm_coe_dict_desc_response;
}

/*****************************************************************************/

/** Prepare an entry description request.
 *
 * \return Zero on success, otherwise a negative error code.
 */
int ec_fsm_coe_dict_prepare_entry(
    ec_fsm_coe_t *fsm,      /**< Finite state machine */
    ec_datagram_t *datagram /**< Datagram to use. */
)
{
    ec_slave_t *slave = fsm->slave;
    u8 *data = ec_slave_mbox_prepare_send(slave, datagram, EC_MBOX_TYPE_COE,
                                          10);
    if (IS_ERR(data))
    {
        return PTR_ERR(data);
    }

    EC_WRITE_U16(data, 0x8 << 12); // SDO information
    EC_WRITE_U8(data + 2, 0x05);   // Get entry description request
    EC_WRITE_U8(data + 3, 0x00);
    EC_WRITE_U16(data + 4, 0x0000);
    EC_WRITE_U16(data + 6, fsm->sdo->index); // SDO index
    EC_WRITE_U8(data + 8, fsm->subindex);    // SDO subindex
    EC_WRITE_U8(data + 9, 0x01);             // value info (access rights only)

    fsm->state = ec_fsm_coe_dict_entry_request;
    return 0;
}

/*****************************************************************************/

/**
   CoE state: DICT DESC RESPONSE.
   \todo Timeout behavior
*/

void ec_fsm_coe_dict_desc_response(
    ec_fsm_coe_t *fsm,      /**< Finite state machine */
    ec_datagram_t *datagram /**< Datagram to use. */
)
{
    ec_slave_t *slave = fsm->slave;

    if (fsm->datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--)
    {
        ec_slave_mbox_prepare_fetch(slave, datagram); // can not fail.
        return;
    }

    if (fsm->datagram->state != EC_DATAGRAM_RECEIVED)
    {
        fsm->state = ec_fsm_coe_error;
        ec_read_mbox_lock_clear(slave);
        EC_SLAVE_ERR(slave, "Failed to receive CoE SDO description"
                            " response datagram: ");
        ec_datagram_print_state(fsm->datagram);
        return;
    }

    if (fsm->datagram->working_counter != 1)
    {
        // only an error if data has not already been read by another read request
        if (slave->mbox_coe_data.payload_size == 0)
        {
            fsm->state = ec_fsm_coe_error;
            ec_read_mbox_lock_clear(slave);
            EC_SLAVE_ERR(slave, "Reception of CoE SDO description"
                                " response failed: ");
            ec_datagram_print_wc_error(fsm->datagram);
            return;
        }
    }
    ec_read_mbox_lock_clear(slave);
    fsm->state = ec_fsm_coe_dict_desc_response_data;
    fsm->state(fsm, datagram);
}

/*****************************************************************************/

/**
   CoE state: DICT DESC RESPONSE DATA.

*/

void ec_fsm_coe_dict_desc_response_data(
    ec_fsm_coe_t *fsm,      /**< Finite state machine */
    ec_datagram_t *datagram /**< Datagram to use. */
)
{
    ec_slave_t *slave = fsm->slave;
    ec_sdo_t *sdo = fsm->sdo;
    uint8_t *data, mbox_prot;
    size_t rec_size, name_size;

    // process the data available or initiate a new mailbox read check
    if (slave->mbox_coe_data.payload_size > 0)
    {
        slave->mbox_coe_data.payload_size = 0;
    }
    else
    {
        // initiate a new mailbox read check if required data is not available
        if (ec_read_mbox_locked(slave))
        {
            // await current read request and mark the datagram as invalid
            datagram->state = EC_DATAGRAM_INVALID;
        }
        else
        {
            ec_slave_mbox_prepare_check(slave, datagram); // can not fail.
            fsm->state = ec_fsm_coe_dict_desc_check;
        }
        return;
    }

    data = ec_slave_mbox_fetch(slave, &slave->mbox_coe_data, &mbox_prot, &rec_size);
    if (IS_ERR(data))
    {
        fsm->state = ec_fsm_coe_error;
        return;
    }

    if (mbox_prot != EC_MBOX_TYPE_COE)
    {
        EC_SLAVE_ERR(slave, "Received mailbox protocol 0x%02X as response.\n",
                     mbox_prot);
        fsm->state = ec_fsm_coe_error;
        return;
    }

    if (ec_fsm_coe_check_emergency(fsm, data, rec_size))
    {
        // check for CoE response again
        ec_slave_mbox_prepare_check(slave, datagram); // can not fail.
        fsm->retries = EC_FSM_RETRIES;
        fsm->state = ec_fsm_coe_dict_desc_check;
        return;
    }

    if (rec_size < 3)
    {
        EC_SLAVE_ERR(slave, "Received corrupted SDO description response"
                            " (size %zu).\n",
                     rec_size);
        fsm->state = ec_fsm_coe_error;
        return;
    }

    if (EC_READ_U16(data) >> 12 == 0x8 && // SDO information
        (EC_READ_U8(data + 2) & 0x7F) == 0x07)
    { // error response
        EC_SLAVE_ERR(slave, "SDO information error response while"
                            " fetching SDO 0x%04X!\n",
                     sdo->index);
        ec_canopen_abort_msg(slave, EC_READ_U32(data + 6));
        fsm->state = ec_fsm_coe_error;
        return;
    }

    if (rec_size < 8)
    {
        EC_SLAVE_ERR(slave, "Received corrupted SDO"
                            " description response (size %zu).\n",
                     rec_size);
        fsm->state = ec_fsm_coe_error;
        return;
    }

    if (EC_READ_U16(data) >> 12 != 0x8 ||        // SDO information
        (EC_READ_U8(data + 2) & 0x7F) != 0x04 || // Object desc. response
        EC_READ_U16(data + 6) != sdo->index)
    { // SDO index
        if (fsm->slave->master->debug_level)
        {
            EC_SLAVE_DBG(slave, 1, "Invalid object description response while"
                                   " fetching SDO 0x%04X!\n",
                         sdo->index);
            ec_print_data(data, rec_size);
        }
        // check for CoE response again
        ec_slave_mbox_prepare_check(slave, datagram); // can not fail.
        fsm->retries = EC_FSM_RETRIES;
        fsm->state = ec_fsm_coe_dict_desc_check;
        return;
    }

    if (rec_size < 12)
    {
        EC_SLAVE_ERR(slave, "Invalid data size!\n");
        ec_print_data(data, rec_size);
        fsm->state = ec_fsm_coe_error;
        return;
    }

    sdo->max_subindex = EC_READ_U8(data + 10);
    sdo->object_code = EC_READ_U8(data + 11);

    name_size = rec_size - 12;
    if (name_size)
    {
        if (!(sdo->name = kmalloc(name_size + 1, GFP_KERNEL)))
        {
            EC_SLAVE_ERR(slave, "Failed to allocate SDO name!\n");
            fsm->state = ec_fsm_coe_error;
            return;
        }

        memcpy(sdo->name, data + 12, name_size);
        sdo->name[name_size] = 0;
    }

    if (EC_READ_U8(data + 2) & 0x80)
    {
        EC_SLAVE_ERR(slave, "Fragment follows (not implemented)!\n");
        fsm->state = ec_fsm_coe_error;
        return;
    }

    // start fetching entries

    fsm->subindex = 0;
    fsm->retries = EC_FSM_RETRIES;

    if (ec_fsm_coe_dict_prepare_entry(fsm, datagram))
    {
        fsm->state = ec_fsm_coe_error;
    }
}

/*****************************************************************************/

/**
   CoE state: DICT ENTRY REQUEST.
   \todo Timeout behavior
*/

void ec_fsm_coe_dict_entry_request(
    ec_fsm_coe_t *fsm,      /**< Finite state machine. */
    ec_datagram_t *datagram /**< Datagram to use. */
)
{
    ec_slave_t *slave = fsm->slave;

    if (fsm->datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--)
    {
        if (ec_fsm_coe_dict_prepare_entry(fsm, datagram))
        {
            fsm->state = ec_fsm_coe_error;
        }
        return;
    }

    if (fsm->datagram->state != EC_DATAGRAM_RECEIVED)
    {
        fsm->state = ec_fsm_coe_error;
        EC_SLAVE_ERR(slave, "Failed to receive CoE SDO entry"
                            " request datagram: ");
        ec_datagram_print_state(fsm->datagram);
        return;
    }

    if (fsm->datagram->working_counter != 1)
    {
        fsm->state = ec_fsm_coe_error;
        EC_SLAVE_ERR(slave, "Reception of CoE SDO entry request failed: ");
        ec_datagram_print_wc_error(fsm->datagram);
        return;
    }

    fsm->jiffies_start = fsm->datagram->jiffies_sent;

    // mailbox read check is skipped if a read request is already ongoing
    if (ec_read_mbox_locked(slave))
    {
        fsm->state = ec_fsm_coe_dict_entry_response_data;
        // the datagram is not used and marked as invalid
        datagram->state = EC_DATAGRAM_INVALID;
    }
    else
    {
        ec_slave_mbox_prepare_check(slave, datagram); // can not fail.
        fsm->retries = EC_FSM_RETRIES;
        fsm->state = ec_fsm_coe_dict_entry_check;
    }
}

/*****************************************************************************/

/**
   CoE state: DICT ENTRY CHECK.
*/

void ec_fsm_coe_dict_entry_check(
    ec_fsm_coe_t *fsm,      /**< Finite state machine. */
    ec_datagram_t *datagram /**< Datagram to use. */
)
{
    ec_slave_t *slave = fsm->slave;

    if (fsm->datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--)
    {
        ec_slave_mbox_prepare_check(slave, datagram); // can not fail
        return;
    }

    if (fsm->datagram->state != EC_DATAGRAM_RECEIVED)
    {
        fsm->state = ec_fsm_coe_error;
        ec_read_mbox_lock_clear(slave);
        EC_SLAVE_ERR(slave, "Failed to receive CoE mailbox check datagram: ");
        ec_datagram_print_state(fsm->datagram);
        return;
    }

    if (fsm->datagram->working_counter != 1)
    {
        fsm->state = ec_fsm_coe_error;
        ec_read_mbox_lock_clear(slave);
        EC_SLAVE_ERR(slave, "Reception of CoE mailbox check"
                            " datagram failed: ");
        ec_datagram_print_wc_error(fsm->datagram);
        return;
    }

    if (!ec_slave_mbox_check(fsm->datagram))
    {
        unsigned long diff_ms = 0;

        // check that data is not already received by another read request
        if (slave->mbox_coe_data.payload_size > 0)
        {
            ec_read_mbox_lock_clear(slave);
            fsm->state = ec_fsm_coe_dict_entry_response_data;
            fsm->state(fsm, datagram);
            return;
        }

        diff_ms = (fsm->datagram->jiffies_received - fsm->jiffies_start) *
                  1000 / HZ;

        if (diff_ms >= EC_FSM_COE_DICT_TIMEOUT)
        {
            fsm->state = ec_fsm_coe_error;
            ec_read_mbox_lock_clear(slave);
            EC_SLAVE_ERR(slave, "Timeout while waiting for"
                                " SDO entry 0x%04x:%x description response.\n",
                         fsm->sdo->index, fsm->subindex);
            return;
        }

        ec_slave_mbox_prepare_check(slave, datagram); // can not fail.
        fsm->retries = EC_FSM_RETRIES;
        return;
    }

    // Fetch response
    ec_slave_mbox_prepare_fetch(slave, datagram); // can not fail.
    fsm->retries = EC_FSM_RETRIES;
    fsm->state = ec_fsm_coe_dict_entry_response;
}

/*****************************************************************************/

/**
   CoE state: DICT ENTRY RESPONSE.
   \todo Timeout behavior
*/

void ec_fsm_coe_dict_entry_response(
    ec_fsm_coe_t *fsm,      /**< Finite state machine. */
    ec_datagram_t *datagram /**< Datagram to use. */
)
{
    ec_slave_t *slave = fsm->slave;

    if (fsm->datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--)
    {
        ec_slave_mbox_prepare_fetch(slave, datagram); // can not fail.
        return;
    }

    if (fsm->datagram->state != EC_DATAGRAM_RECEIVED)
    {
        fsm->state = ec_fsm_coe_error;
        ec_read_mbox_lock_clear(slave);
        EC_SLAVE_ERR(slave, "Failed to receive CoE SDO"
                            " description response datagram: ");
        ec_datagram_print_state(fsm->datagram);
        return;
    }

    if (fsm->datagram->working_counter != 1)
    {
        // only an error if data has not already been read by another read request
        if (slave->mbox_coe_data.payload_size == 0)
        {
            fsm->state = ec_fsm_coe_error;
            ec_read_mbox_lock_clear(slave);
            EC_SLAVE_ERR(slave, "Reception of CoE SDO description"
                                " response failed: ");
            ec_datagram_print_wc_error(fsm->datagram);
            return;
        }
    }
    ec_read_mbox_lock_clear(slave);
    fsm->state = ec_fsm_coe_dict_entry_response_data;
    fsm->state(fsm, datagram);
}

/*****************************************************************************/

/**
   CoE state: DICT ENTRY RESPONSE DATA.

*/

void ec_fsm_coe_dict_entry_response_data(
    ec_fsm_coe_t *fsm,      /**< Finite state machine. */
    ec_datagram_t *datagram /**< Datagram to use. */
)
{
    ec_slave_t *slave = fsm->slave;
    ec_sdo_t *sdo = fsm->sdo;
    uint8_t *data, mbox_prot;
    size_t rec_size, data_size;
    ec_sdo_entry_t *entry;
    u16 word;

    // process the data available or initiate a new mailbox read check
    if (slave->mbox_coe_data.payload_size > 0)
    {
        slave->mbox_coe_data.payload_size = 0;
    }
    else
    {
        // initiate a new mailbox read check if required data is not available
        if (ec_read_mbox_locked(slave))
        {
            // await current read request and mark the datagram as invalid
            datagram->state = EC_DATAGRAM_INVALID;
        }
        else
        {
            ec_slave_mbox_prepare_check(slave, datagram); // can not fail.
            fsm->state = ec_fsm_coe_dict_entry_check;
        }
        return;
    }

    data = ec_slave_mbox_fetch(slave, &slave->mbox_coe_data, &mbox_prot, &rec_size);
    if (IS_ERR(data))
    {
        fsm->state = ec_fsm_coe_error;
        return;
    }

    if (mbox_prot != EC_MBOX_TYPE_COE)
    {
        EC_SLAVE_ERR(slave, "Received mailbox protocol"
                            " 0x%02X as response.\n",
                     mbox_prot);
        fsm->state = ec_fsm_coe_error;
        return;
    }

    if (ec_fsm_coe_check_emergency(fsm, data, rec_size))
    {
        // check for CoE response again
        ec_slave_mbox_prepare_check(slave, datagram); // can not fail.
        fsm->retries = EC_FSM_RETRIES;
        fsm->state = ec_fsm_coe_dict_entry_check;
        return;
    }

    if (rec_size < 3)
    {
        EC_SLAVE_ERR(slave, "Received corrupted SDO entry"
                            " description response (size %zu).\n",
                     rec_size);
        fsm->state = ec_fsm_coe_error;
        return;
    }

    if (EC_READ_U16(data) >> 12 == 0x8 && // SDO information
        (EC_READ_U8(data + 2) & 0x7F) == 0x07)
    { // error response
        EC_SLAVE_WARN(slave, "SDO information error response while"
                             " fetching SDO entry 0x%04X:%02X!\n",
                      sdo->index, fsm->subindex);
        ec_canopen_abort_msg(slave, EC_READ_U32(data + 6));

        /* There may be gaps in the subindices, so try to continue with next
         * subindex. */
    }
    else
    {

        if (rec_size < 9)
        {
            EC_SLAVE_ERR(slave, "Received corrupted SDO entry"
                                " description response (size %zu).\n",
                         rec_size);
            fsm->state = ec_fsm_coe_error;
            return;
        }

        if (EC_READ_U16(data) >> 12 != 0x8 ||        // SDO information
            (EC_READ_U8(data + 2) & 0x7F) != 0x06 || // Entry desc. response
            EC_READ_U16(data + 6) != sdo->index ||   // SDO index
            EC_READ_U8(data + 8) != fsm->subindex)
        { // SDO subindex
            if (fsm->slave->master->debug_level)
            {
                EC_SLAVE_DBG(slave, 1, "Invalid entry description response"
                                       " while fetching SDO entry 0x%04X:%02X!\n",
                             sdo->index, fsm->subindex);
                ec_print_data(data, rec_size);
            }
            // check for CoE response again
            ec_slave_mbox_prepare_check(slave, datagram); // can not fail.
            fsm->retries = EC_FSM_RETRIES;
            fsm->state = ec_fsm_coe_dict_entry_check;
            return;
        }

        if (rec_size < 16)
        {
            EC_SLAVE_ERR(slave, "Invalid data size %zu!\n", rec_size);
            ec_print_data(data, rec_size);
            fsm->state = ec_fsm_coe_error;
            return;
        }

        data_size = rec_size - 16;

        if (!(entry = (ec_sdo_entry_t *)
                  kmalloc(sizeof(ec_sdo_entry_t), GFP_KERNEL)))
        {
            EC_SLAVE_ERR(slave, "Failed to allocate entry!\n");
            fsm->state = ec_fsm_coe_error;
            return;
        }

        ec_sdo_entry_init(entry, sdo, fsm->subindex);
        entry->data_type = EC_READ_U16(data + 10);
        entry->bit_length = EC_READ_U16(data + 12);

        // read access rights
        word = EC_READ_U16(data + 14);
        entry->read_access[EC_SDO_ENTRY_ACCESS_PREOP] = word & 0x0001;
        entry->read_access[EC_SDO_ENTRY_ACCESS_SAFEOP] =
            (word >> 1) & 0x0001;
        entry->read_access[EC_SDO_ENTRY_ACCESS_OP] = (word >> 2) & 0x0001;
        entry->write_access[EC_SDO_ENTRY_ACCESS_PREOP] = (word >> 3) & 0x0001;
        entry->write_access[EC_SDO_ENTRY_ACCESS_SAFEOP] =
            (word >> 4) & 0x0001;
        entry->write_access[EC_SDO_ENTRY_ACCESS_OP] = (word >> 5) & 0x0001;

        if (data_size)
        {
            uint8_t *desc;
            if (!(desc = kmalloc(data_size + 1, GFP_KERNEL)))
            {
                EC_SLAVE_ERR(slave, "Failed to allocate SDO entry name!\n");
                fsm->state = ec_fsm_coe_error;
                return;
            }
            memcpy(desc, data + 16, data_size);
            desc[data_size] = 0;
            entry->description = desc;
        }

        list_add_tail(&entry->list, &sdo->entries);
    }

    if (fsm->subindex < sdo->max_subindex)
    {

        fsm->subindex++;
        fsm->retries = EC_FSM_RETRIES;

        if (ec_fsm_coe_dict_prepare_entry(fsm, datagram))
        {
            fsm->state = ec_fsm_coe_error;
        }

        return;
    }

    // another SDO description to fetch?
    if (fsm->sdo->list.next != &slave->sdo_dictionary)
    {

        fsm->sdo = list_entry(fsm->sdo->list.next, ec_sdo_t, list);
        fsm->retries = EC_FSM_RETRIES;

        if (ec_fsm_coe_dict_prepare_desc(fsm, datagram))
        {
            fsm->state = ec_fsm_coe_error;
        }

        return;
    }

    fsm->state = ec_fsm_coe_end;
}

/******************************************************************************
 *  CoE state machine
 *****************************************************************************/

/** Prepare a donwnload request.
 *
 * \return Zero on success, otherwise a negative error code.
 */
int ec_fsm_coe_prepare_down_start(
    ec_fsm_coe_t *fsm,      /**< Finite state machine. */
    ec_datagram_t *datagram /**< Datagram to use. */
)
{
    u8 *data;
    ec_slave_t *slave = fsm->slave;
    ec_sdo_request_t *request = fsm->request;
    uint8_t data_set_size;

    if (request->data_size <= 4)
    { // use expedited transfer type
        data = ec_slave_mbox_prepare_send(slave, datagram, EC_MBOX_TYPE_COE,
                                          EC_COE_DOWN_REQ_HEADER_SIZE);
        if (IS_ERR(data))
        {
            request->errno = PTR_ERR(data);
            return PTR_ERR(data);
        }

        fsm->remaining = 0;

        data_set_size = 4 - request->data_size;

        EC_WRITE_U16(data, 0x2 << 12);                                                                        // SDO request
        EC_WRITE_U8(data + 2, (0x3                                                                            // size specified, expedited
                               | data_set_size << 2 | ((request->complete_access ? 1 : 0) << 4) | 0x1 << 5)); // Download request
        EC_WRITE_U16(data + 3, request->index);
        EC_WRITE_U8(data + 5,
                    request->complete_access ? 0x00 : request->subindex);
        memcpy(data + 6, request->data, request->data_size);
        memset(data + 6 + request->data_size, 0x00, 4 - request->data_size);

        if (slave->master->debug_level)
        {
            EC_SLAVE_DBG(slave, 1, "Expedited download request:\n");
            ec_print_data(data, EC_COE_DOWN_REQ_HEADER_SIZE);
        }
    }
    else
    { // request->data_size > 4, use normal transfer type
        size_t data_size,
            max_data_size =
                slave->configured_rx_mailbox_size - EC_MBOX_HEADER_SIZE,
            required_data_size =
                EC_COE_DOWN_REQ_HEADER_SIZE + request->data_size;

        if (max_data_size < required_data_size)
        {
            // segmenting needed
            data_size = max_data_size;
        }
        else
        {
            data_size = required_data_size;
        }

        data = ec_slave_mbox_prepare_send(slave, datagram, EC_MBOX_TYPE_COE,
                                          data_size);
        if (IS_ERR(data))
        {
            request->errno = PTR_ERR(data);
            return PTR_ERR(data);
        }

        fsm->offset = 0;
        fsm->remaining = request->data_size;

        EC_WRITE_U16(data, 0x2 << 12); // SDO request
        EC_WRITE_U8(data + 2,
                    0x1                                                          // size indicator, normal
                        | ((request->complete_access ? 1 : 0) << 4) | 0x1 << 5); // Download request
        EC_WRITE_U16(data + 3, request->index);
        EC_WRITE_U8(data + 5,
                    request->complete_access ? 0x00 : request->subindex);
        EC_WRITE_U32(data + 6, request->data_size);

        if (data_size > EC_COE_DOWN_REQ_HEADER_SIZE)
        {
            size_t segment_size = data_size - EC_COE_DOWN_REQ_HEADER_SIZE;
            memcpy(data + EC_COE_DOWN_REQ_HEADER_SIZE,
                   request->data, segment_size);
            fsm->offset += segment_size;
            fsm->remaining -= segment_size;
        }

        if (slave->master->debug_level)
        {
            EC_SLAVE_DBG(slave, 1, "Normal download request:\n");
            ec_print_data(data, data_size);
        }
    }

    fsm->state = ec_fsm_coe_down_request;
    return 0;
}

/****************************************************************************/

/** CoE state: DOWN START.
 */
void ec_fsm_coe_down_start(
    ec_fsm_coe_t *fsm,      /**< Finite state machine. */
    ec_datagram_t *datagram /**< Datagram to use. */
)
{
    ec_slave_t *slave = fsm->slave;
    ec_sdo_request_t *request = fsm->request;

    if (fsm->slave->master->debug_level)
    {
        char subidxstr[10];
        if (request->complete_access)
        {
            subidxstr[0] = 0x00;
        }
        else
        {
            sprintf(subidxstr, ":%02X", request->subindex);
        }
        EC_SLAVE_DBG(slave, 1, "Downloading SDO 0x%04X%s.\n",
                     request->index, subidxstr);
        ec_print_data(request->data, request->data_size);
    }

    if (!slave->sii_image)
    {
        EC_SLAVE_ERR(slave, "Slave cannot process CoE download request."
                            " SII data not available.\n");
        request->errno = EAGAIN;
        fsm->state = ec_fsm_coe_error;
        return;
    }

    if (!(slave->sii_image->sii.mailbox_protocols & EC_MBOX_COE))
    {
        EC_SLAVE_ERR(slave, "Slave does not support CoE!\n");
        request->errno = EPROTONOSUPPORT;
        fsm->state = ec_fsm_coe_error;
        return;
    }

    if (slave->configured_rx_mailbox_size <
        EC_MBOX_HEADER_SIZE + EC_COE_DOWN_REQ_HEADER_SIZE)
    {
        EC_SLAVE_ERR(slave, "Mailbox too small!\n");
        request->errno = EOVERFLOW;
        fsm->state = ec_fsm_coe_error;
        return;
    }

    fsm->request->jiffies_sent = jiffies;
    fsm->retries = EC_FSM_RETRIES;

    if (ec_fsm_coe_prepare_down_start(fsm, datagram))
    {
        fsm->state = ec_fsm_coe_error;
    }
}

/*****************************************************************************/

/**
   CoE state: DOWN REQUEST.
   \todo Timeout behavior
*/

void ec_fsm_coe_down_request(
    ec_fsm_coe_t *fsm,      /**< Finite state machine. */
    ec_datagram_t *datagram /**< Datagram to use. */
)
{
    ec_slave_t *slave = fsm->slave;
    unsigned long diff_ms;

    if (fsm->datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--)
    {
        if (ec_fsm_coe_prepare_down_start(fsm, datagram))
        {
            fsm->state = ec_fsm_coe_error;
        }
        return;
    }

    if (fsm->datagram->state != EC_DATAGRAM_RECEIVED)
    {
        fsm->request->errno = EIO;
        fsm->state = ec_fsm_coe_error;
        EC_SLAVE_ERR(slave, "Failed to receive CoE download"
                            " request datagram: ");
        ec_datagram_print_state(fsm->datagram);
        return;
    }

    diff_ms = (jiffies - fsm->request->jiffies_sent) * 1000 / HZ;

    if (fsm->datagram->working_counter != 1)
    {
        if (!fsm->datagram->working_counter)
        {
            if (diff_ms < fsm->request->response_timeout)
            {
#if DEBUG_RETRIES
                EC_SLAVE_DBG(slave, 1, "Slave did not respond to SDO"
                                       " download request. Retrying after %lu ms...\n",
                             diff_ms);
#endif
                // no response; send request datagram again
                if (ec_fsm_coe_prepare_down_start(fsm, datagram))
                {
                    fsm->state = ec_fsm_coe_error;
                }
                return;
            }
        }
        fsm->request->errno = EIO;
        fsm->state = ec_fsm_coe_error;
        EC_SLAVE_ERR(slave, "Reception of CoE download request"
                            " for SDO 0x%04x:%x failed with timeout after %lu ms: ",
                     fsm->request->index, fsm->request->subindex, diff_ms);
        ec_datagram_print_wc_error(fsm->datagram);
        return;
    }

#if DEBUG_LONG
    if (diff_ms > 200)
    {
        EC_SLAVE_WARN(slave, "SDO 0x%04x:%x download took %lu ms.\n",
                      fsm->request->index, fsm->request->subindex, diff_ms);
    }
#endif

    fsm->jiffies_start = fsm->datagram->jiffies_sent;

    // mailbox read check is skipped if a read request is already ongoing
    if (ec_read_mbox_locked(slave))
    {
        fsm->state = ec_fsm_coe_down_response_data;
        // the datagram is not used and marked as invalid
        datagram->state = EC_DATAGRAM_INVALID;
    }
    else
    {
        ec_slave_mbox_prepare_check(slave, datagram); // can not fail.
        fsm->retries = EC_FSM_RETRIES;
        fsm->state = ec_fsm_coe_down_check;
    }
}

/*****************************************************************************/

/** CoE state: DOWN CHECK.
 */
void ec_fsm_coe_down_check(
    ec_fsm_coe_t *fsm,      /**< Finite state machine. */
    ec_datagram_t *datagram /**< Datagram to use. */
)
{
    ec_slave_t *slave = fsm->slave;

    if (fsm->datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--)
    {
        ec_slave_mbox_prepare_check(slave, datagram); // can not fail.
        return;
    }

    if (fsm->datagram->state != EC_DATAGRAM_RECEIVED)
    {
        fsm->request->errno = EIO;
        fsm->state = ec_fsm_coe_error;
        ec_read_mbox_lock_clear(slave);
        EC_SLAVE_ERR(slave, "Failed to receive CoE mailbox check"
                            " datagram: ");
        ec_datagram_print_state(fsm->datagram);
        return;
    }

    if (fsm->datagram->working_counter != 1)
    {
        fsm->request->errno = EIO;
        fsm->state = ec_fsm_coe_error;
        ec_read_mbox_lock_clear(slave);
        EC_SLAVE_ERR(slave, "Reception of CoE mailbox check"
                            " datagram failed: ");
        ec_datagram_print_wc_error(fsm->datagram);
        return;
    }

    if (!ec_slave_mbox_check(fsm->datagram))
    {
        unsigned long diff_ms = 0;

        // check that data is not already received by another read request
        if (slave->mbox_coe_data.payload_size > 0)
        {
            ec_read_mbox_lock_clear(slave);
            fsm->state = ec_fsm_coe_down_response_data;
            fsm->state(fsm, datagram);
            return;
        }

        diff_ms = (fsm->datagram->jiffies_received - fsm->jiffies_start) *
                  1000 / HZ;

        if (diff_ms >= fsm->request->response_timeout)
        {
            fsm->request->errno = EIO;
            fsm->state = ec_fsm_coe_error;
            ec_read_mbox_lock_clear(slave);
            EC_SLAVE_ERR(slave, "Timeout after %lu ms while waiting"
                                " for SDO 0x%04x:%x download response.\n",
                         diff_ms,
                         fsm->request->index, fsm->request->subindex);
            return;
        }

        ec_slave_mbox_prepare_check(slave, datagram); // can not fail.
        fsm->retries = EC_FSM_RETRIES;
        return;
    }

    // Fetch response
    ec_slave_mbox_prepare_fetch(slave, datagram); // can not fail.
    fsm->retries = EC_FSM_RETRIES;
    fsm->state = ec_fsm_coe_down_response;
}

/*****************************************************************************/

/** Prepare a download segment request.
 */
void ec_fsm_coe_down_prepare_segment_request(
    ec_fsm_coe_t *fsm,      /**< Finite state machine. */
    ec_datagram_t *datagram /**< Datagram to use. */
)
{
    ec_slave_t *slave = fsm->slave;
    ec_sdo_request_t *request = fsm->request;
    size_t max_segment_size =
        slave->configured_rx_mailbox_size - EC_MBOX_HEADER_SIZE - EC_COE_DOWN_SEG_REQ_HEADER_SIZE;
    size_t data_size;
    uint8_t last_segment, seg_data_size, *data;

    if (fsm->remaining > max_segment_size)
    {
        fsm->segment_size = max_segment_size;
        last_segment = 0;
    }
    else
    {
        fsm->segment_size = fsm->remaining;
        last_segment = 1;
    }

    if (fsm->segment_size > EC_COE_DOWN_SEG_MIN_DATA_SIZE)
    {
        seg_data_size = 0x00;
        data_size = EC_COE_DOWN_SEG_REQ_HEADER_SIZE + fsm->segment_size;
    }
    else
    {
        seg_data_size = EC_COE_DOWN_SEG_MIN_DATA_SIZE - fsm->segment_size;
        data_size = EC_COE_DOWN_SEG_REQ_HEADER_SIZE + EC_COE_DOWN_SEG_MIN_DATA_SIZE;
    }

    data = ec_slave_mbox_prepare_send(slave, datagram, EC_MBOX_TYPE_COE,
                                      data_size);
    if (IS_ERR(data))
    {
        request->errno = PTR_ERR(data);
        fsm->state = ec_fsm_coe_error;
        return;
    }

    EC_WRITE_U16(data, 0x2 << 12);                                                                           // SDO request
    EC_WRITE_U8(data + 2, (last_segment ? 1 : 0) | (seg_data_size << 1) | (fsm->toggle << 4) | (0x00 << 5)); // Download segment request
    memcpy(data + EC_COE_DOWN_SEG_REQ_HEADER_SIZE,
           request->data + fsm->offset, fsm->segment_size);
    if (fsm->segment_size < EC_COE_DOWN_SEG_MIN_DATA_SIZE)
    {
        memset(data + EC_COE_DOWN_SEG_REQ_HEADER_SIZE + fsm->segment_size,
               0x00, EC_COE_DOWN_SEG_MIN_DATA_SIZE - fsm->segment_size);
    }

    if (slave->master->debug_level)
    {
        EC_SLAVE_DBG(slave, 1, "Download segment request:\n");
        ec_print_data(data, data_size);
    }

    fsm->state = ec_fsm_coe_down_seg_check;
}

/*****************************************************************************/

/**
   CoE state: DOWN RESPONSE.
   \todo Timeout behavior
*/

void ec_fsm_coe_down_response(
    ec_fsm_coe_t *fsm,      /**< Finite state machine. */
    ec_datagram_t *datagram /**< Datagram to use. */
)
{
    ec_slave_t *slave = fsm->slave;
    ec_sdo_request_t *request = fsm->request;

    if (fsm->datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--)
    {
        ec_slave_mbox_prepare_fetch(slave, datagram); // can not fail.
        return;
    }

    if (fsm->datagram->state != EC_DATAGRAM_RECEIVED)
    {
        request->errno = EIO;
        fsm->state = ec_fsm_coe_error;
        EC_SLAVE_ERR(slave, "Failed to receive CoE download"
                            " response datagram: ");
        ec_datagram_print_state(fsm->datagram);
        return;
    }

    if (fsm->datagram->working_counter != 1)
    {
        // only an error if data has not already been read by another read request
        if (slave->mbox_coe_data.payload_size == 0)
        {
            request->errno = EIO;
            fsm->state = ec_fsm_coe_error;
            ec_read_mbox_lock_clear(slave);
            EC_SLAVE_ERR(slave, "Reception of CoE download response failed: ");
            ec_datagram_print_wc_error(fsm->datagram);
            return;
        }
    }
    ec_read_mbox_lock_clear(slave);
    fsm->state = ec_fsm_coe_down_response_data;
    fsm->state(fsm, datagram);
}

/*****************************************************************************/

/**
   CoE state: DOWN RESPONSE DATA.

*/

void ec_fsm_coe_down_response_data(
    ec_fsm_coe_t *fsm,      /**< Finite state machine. */
    ec_datagram_t *datagram /**< Datagram to use. */
)
{
    ec_slave_t *slave = fsm->slave;
    uint8_t *data, mbox_prot;
    size_t rec_size;
    ec_sdo_request_t *request = fsm->request;

    // process the data available or initiate a new mailbox read check
    if (slave->mbox_coe_data.payload_size > 0)
    {
        slave->mbox_coe_data.payload_size = 0;
    }
    else
    {
        // initiate a new mailbox read check if required data is not available
        if (ec_read_mbox_locked(slave))
        {
            // await current read request and mark the datagram as invalid
            datagram->state = EC_DATAGRAM_INVALID;
        }
        else
        {
            ec_slave_mbox_prepare_check(slave, datagram); // can not fail.
            fsm->state = ec_fsm_coe_down_check;
        }
        return;
    }

    data = ec_slave_mbox_fetch(slave, &slave->mbox_coe_data, &mbox_prot, &rec_size);

    if (IS_ERR(data))
    {
        request->errno = PTR_ERR(data);
        fsm->state = ec_fsm_coe_error;
        return;
    }

    if (mbox_prot != EC_MBOX_TYPE_COE)
    {
        request->errno = EIO;
        fsm->state = ec_fsm_coe_error;
        EC_SLAVE_ERR(slave, "Received mailbox protocol 0x%02X as response.\n",
                     mbox_prot);
        return;
    }

    if (ec_fsm_coe_check_emergency(fsm, data, rec_size))
    {
        // check for CoE response again
        ec_slave_mbox_prepare_check(slave, datagram); // can not fail.
        fsm->retries = EC_FSM_RETRIES;
        fsm->state = ec_fsm_coe_down_check;
        return;
    }

    if (slave->master->debug_level)
    {
        EC_SLAVE_DBG(slave, 1, "Download response:\n");
        ec_print_data(data, rec_size);
    }

    if (rec_size < 6)
    {
        request->errno = EIO;
        fsm->state = ec_fsm_coe_error;
        EC_SLAVE_ERR(slave, "Received data are too small (%zu bytes):\n",
                     rec_size);
        ec_print_data(data, rec_size);
        return;
    }

    if (EC_READ_U16(data) >> 12 == 0x2 && // SDO request
        EC_READ_U8(data + 2) >> 5 == 0x4)
    { // abort SDO transfer request
        char subidxstr[10];
        request->errno = EIO;
        fsm->state = ec_fsm_coe_error;
        if (request->complete_access)
        {
            subidxstr[0] = 0x00;
        }
        else
        {
            sprintf(subidxstr, ":%02X", request->subindex);
        }
        EC_SLAVE_ERR(slave, "SDO download 0x%04X%s (%zu bytes) aborted.\n",
                     request->index, subidxstr, request->data_size);
        if (rec_size < 10)
        {
            EC_SLAVE_ERR(slave, "Incomplete abort command:\n");
            ec_print_data(data, rec_size);
        }
        else
        {
            fsm->request->abort_code = EC_READ_U32(data + 6);
            ec_canopen_abort_msg(slave, fsm->request->abort_code);
        }
        return;
    }

    if (EC_READ_U16(data) >> 12 != 0x3 ||          // SDO response
        EC_READ_U8(data + 2) >> 5 != 0x3 ||        // Download response
        EC_READ_U16(data + 3) != request->index || // index
        EC_READ_U8(data + 5) != request->subindex)
    { // subindex
        if (slave->master->debug_level)
        {
            EC_SLAVE_DBG(slave, 1, "Invalid SDO download response!"
                                   " Retrying...\n");
            ec_print_data(data, rec_size);
        }
        // check for CoE response again
        ec_slave_mbox_prepare_check(slave, datagram); // can not fail.
        fsm->retries = EC_FSM_RETRIES;
        fsm->state = ec_fsm_coe_down_check;
        return;
    }

    if (fsm->remaining)
    { // more segments to download
        fsm->toggle = 0;
        ec_fsm_coe_down_prepare_segment_request(fsm, datagram);
    }
    else
    {
        fsm->state = ec_fsm_coe_end; // success
    }
}

/*****************************************************************************/

/**
   CoE state: DOWN SEG CHECK.
*/

void ec_fsm_coe_down_seg_check(
    ec_fsm_coe_t *fsm,      /**< Finite state machine. */
    ec_datagram_t *datagram /**< Datagram to use. */
)
{
    ec_slave_t *slave = fsm->slave;

    if (fsm->datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--)
        return;

    if (fsm->datagram->state != EC_DATAGRAM_RECEIVED)
    {
        fsm->request->errno = EIO;
        fsm->state = ec_fsm_coe_error;
        ec_read_mbox_lock_clear(slave);
        EC_SLAVE_ERR(slave, "Failed to receive CoE mailbox check datagram: ");
        ec_datagram_print_state(fsm->datagram);
        return;
    }

    if (fsm->datagram->working_counter != 1)
    {
        fsm->request->errno = EIO;
        fsm->state = ec_fsm_coe_error;
        ec_read_mbox_lock_clear(slave);
        EC_SLAVE_ERR(slave, "Reception of CoE mailbox segment check"
                            " datagram failed: ");
        ec_datagram_print_wc_error(fsm->datagram);
        return;
    }

    if (!ec_slave_mbox_check(fsm->datagram))
    {
        unsigned long diff_ms = 0;

        // check that data is not already received by another read request
        if (slave->mbox_coe_data.payload_size > 0)
        {
            ec_read_mbox_lock_clear(slave);
            fsm->state = ec_fsm_coe_down_seg_response_data;
            fsm->state(fsm, datagram);
            return;
        }

        diff_ms = (fsm->datagram->jiffies_received - fsm->jiffies_start) *
                  1000 / HZ;

        if (diff_ms >= fsm->request->response_timeout)
        {
            fsm->request->errno = EIO;
            fsm->state = ec_fsm_coe_error;
            ec_read_mbox_lock_clear(slave);
            EC_SLAVE_ERR(slave, "Timeout while waiting for SDO download"
                                " segment response.\n");
            return;
        }

        ec_slave_mbox_prepare_check(slave, datagram); // can not fail.
        fsm->retries = EC_FSM_RETRIES;
        return;
    }

    // Fetch response
    ec_slave_mbox_prepare_fetch(slave, datagram); // can not fail.
    fsm->retries = EC_FSM_RETRIES;
    fsm->state = ec_fsm_coe_down_seg_response;
}

/*****************************************************************************/

/**
   CoE state: DOWN SEG RESPONSE.
   \todo Timeout behavior
*/

void ec_fsm_coe_down_seg_response(
    ec_fsm_coe_t *fsm,      /**< Finite state machine */
    ec_datagram_t *datagram /**< Datagram to use. */
)
{
    ec_slave_t *slave = fsm->slave;
    ec_sdo_request_t *request = fsm->request;

    if (fsm->datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--)
    {
        ec_slave_mbox_prepare_fetch(slave, datagram); // can not fail.
        return;
    }

    if (fsm->datagram->state != EC_DATAGRAM_RECEIVED)
    {
        request->errno = EIO;
        fsm->state = ec_fsm_coe_error;
        ec_read_mbox_lock_clear(slave);
        EC_SLAVE_ERR(slave, "Failed to receive CoE download response"
                            " datagram: ");
        ec_datagram_print_state(fsm->datagram);
        return;
    }

    if (fsm->datagram->working_counter != 1)
    {
        // only an error if data has not already been read by another read request
        if (slave->mbox_coe_data.payload_size == 0)
        {
            request->errno = EIO;
            fsm->state = ec_fsm_coe_error;
            ec_read_mbox_lock_clear(slave);
            EC_SLAVE_ERR(slave, "Reception of CoE download response failed: ");
            ec_datagram_print_wc_error(fsm->datagram);
            return;
        }
    }
    ec_read_mbox_lock_clear(slave);
    fsm->state = ec_fsm_coe_down_seg_response_data;
    fsm->state(fsm, datagram);
}

/*****************************************************************************/

/**
   CoE state: DOWN SEG RESPONSE DATA.

*/

void ec_fsm_coe_down_seg_response_data(
    ec_fsm_coe_t *fsm,      /**< Finite state machine. */
    ec_datagram_t *datagram /**< Datagram to use. */
)
{
    ec_slave_t *slave = fsm->slave;
    uint8_t *data, mbox_prot;
    size_t rec_size;
    ec_sdo_request_t *request = fsm->request;

    // process the data available or initiate a new mailbox read check
    if (slave->mbox_coe_data.payload_size > 0)
    {
        slave->mbox_coe_data.payload_size = 0;
    }
    else
    {
        // initiate a new mailbox read check if required data is not available
        if (ec_read_mbox_locked(slave))
        {
            // await current read request and mark the datagram as invalid
            datagram->state = EC_DATAGRAM_INVALID;
        }
        else
        {
            ec_slave_mbox_prepare_check(slave, datagram); // can not fail.
            fsm->state = ec_fsm_coe_down_seg_check;
        }
        return;
    }

    data = ec_slave_mbox_fetch(slave, &slave->mbox_coe_data, &mbox_prot, &rec_size);
    if (IS_ERR(data))
    {
        request->errno = PTR_ERR(data);
        fsm->state = ec_fsm_coe_error;
        return;
    }

    if (mbox_prot != EC_MBOX_TYPE_COE)
    {
        request->errno = EIO;
        fsm->state = ec_fsm_coe_error;
        EC_SLAVE_ERR(slave, "Received mailbox protocol 0x%02X as response.\n",
                     mbox_prot);
        return;
    }

    if (ec_fsm_coe_check_emergency(fsm, data, rec_size))
    {
        // check for CoE response again
        ec_slave_mbox_prepare_check(slave, datagram); // can not fail.
        fsm->retries = EC_FSM_RETRIES;
        fsm->state = ec_fsm_coe_down_check;
        return;
    }

    if (slave->master->debug_level)
    {
        EC_SLAVE_DBG(slave, 1, "Download response:\n");
        ec_print_data(data, rec_size);
    }

    if (rec_size < 6)
    {
        request->errno = EIO;
        fsm->state = ec_fsm_coe_error;
        EC_SLAVE_ERR(slave, "Received data are too small (%zu bytes):\n",
                     rec_size);
        ec_print_data(data, rec_size);
        return;
    }

    if (EC_READ_U16(data) >> 12 == 0x2 && // SDO request
        EC_READ_U8(data + 2) >> 5 == 0x4)
    { // abort SDO transfer request
        char subidxstr[10];
        request->errno = EIO;
        fsm->state = ec_fsm_coe_error;
        if (request->complete_access)
        {
            subidxstr[0] = 0x00;
        }
        else
        {
            sprintf(subidxstr, ":%02X", request->subindex);
        }
        EC_SLAVE_ERR(slave, "SDO download 0x%04X%s (%zu bytes) aborted.\n",
                     request->index, subidxstr, request->data_size);
        if (rec_size < 10)
        {
            EC_SLAVE_ERR(slave, "Incomplete abort command:\n");
            ec_print_data(data, rec_size);
        }
        else
        {
            fsm->request->abort_code = EC_READ_U32(data + 6);
            ec_canopen_abort_msg(slave, fsm->request->abort_code);
        }
        return;
    }

    if (EC_READ_U16(data) >> 12 != 0x3 ||
        ((EC_READ_U8(data + 2) >> 5) != 0x01))
    { // segment response
        if (slave->master->debug_level)
        {
            EC_SLAVE_DBG(slave, 1, "Invalid SDO download response!"
                                   " Retrying...\n");
            ec_print_data(data, rec_size);
        }
        // check for CoE response again
        ec_slave_mbox_prepare_check(slave, datagram); // can not fail.
        fsm->retries = EC_FSM_RETRIES;
        fsm->state = ec_fsm_coe_down_seg_check;
        return;
    }

    if (((EC_READ_U8(data + 2) >> 4) & 0x01) != fsm->toggle)
    {
        EC_SLAVE_ERR(slave, "Invalid toggle received during"
                            " segmented download:\n");
        ec_print_data(data, rec_size);
        request->errno = EIO;
        fsm->state = ec_fsm_coe_error;
        return;
    }

    fsm->offset += fsm->segment_size;
    fsm->remaining -= fsm->segment_size;

    if (fsm->remaining)
    { // more segments to download
        fsm->toggle = !fsm->toggle;
        ec_fsm_coe_down_prepare_segment_request(fsm, datagram);
    }
    else
    {
        fsm->state = ec_fsm_coe_end; // success
    }
}

/*****************************************************************************/

/** Prepare an upload request.
 *
 * \return Zero on success, otherwise a negative error code.
 */
int ec_fsm_coe_prepare_up(
    ec_fsm_coe_t *fsm,      /**< Finite state machine. */
    ec_datagram_t *datagram /**< Datagram to use. */
)
{
    ec_slave_t *slave = fsm->slave;
    ec_sdo_request_t *request = fsm->request;
    ec_master_t *master = slave->master;

    u8 *data = ec_slave_mbox_prepare_send(slave, datagram, EC_MBOX_TYPE_COE,
                                          10);
    if (IS_ERR(data))
    {
        request->errno = PTR_ERR(data);
        return PTR_ERR(data);
    }

    EC_WRITE_U16(data, 0x2 << 12); // SDO request
    EC_WRITE_U8(data + 2, 0x2 << 5 // initiate upload request
                              | ((request->complete_access ? 1 : 0) << 4));
    EC_WRITE_U16(data + 3, request->index);
    EC_WRITE_U8(data + 5, request->complete_access ? 0x00 : request->subindex);
    memset(data + 6, 0x00, 4);

    if (master->debug_level)
    {
        EC_SLAVE_DBG(slave, 1, "Upload request:\n");
        ec_print_data(data, 10);
    }

    fsm->state = ec_fsm_coe_up_request;
    return 0;
}

/*****************************************************************************/

/**
   CoE state: UP START.
*/

void ec_fsm_coe_up_start(
    ec_fsm_coe_t *fsm,      /**< Finite state machine. */
    ec_datagram_t *datagram /**< Datagram to use. */
)
{
    ec_slave_t *slave = fsm->slave;
    ec_sdo_request_t *request = fsm->request;
    char subidxstr[10];

    if (request->complete_access)
    {
        subidxstr[0] = 0x00;
    }
    else
    {
        sprintf(subidxstr, ":%02X", request->subindex);
    }

    EC_SLAVE_DBG(slave, 1, "Uploading SDO 0x%04X%s.\n",
                 request->index, subidxstr);

    if (!slave->sii_image)
    {
        EC_SLAVE_ERR(slave, "Slave cannot process CoE upload request."
                            " SII data not available.\n");
        request->errno = EAGAIN;
        fsm->state = ec_fsm_coe_error;
        return;
    }

    if (!(slave->sii_image->sii.mailbox_protocols & EC_MBOX_COE))
    {
        EC_SLAVE_ERR(slave, "Slave does not support CoE!\n");
        request->errno = EPROTONOSUPPORT;
        fsm->state = ec_fsm_coe_error;
        return;
    }

    fsm->retries = EC_FSM_RETRIES;
    fsm->request->jiffies_sent = jiffies;

    if (ec_fsm_coe_prepare_up(fsm, datagram))
    {
        fsm->state = ec_fsm_coe_error;
    }
}

/*****************************************************************************/
/**
   CoE state: UP REQUEST.
   \todo Timeout behavior
*/

void ec_fsm_coe_up_request(
    ec_fsm_coe_t *fsm,      /**< Finite state machine. */
    ec_datagram_t *datagram /**< Datagram to use. */
)
{
    ec_slave_t *slave = fsm->slave;
    unsigned long diff_ms;
    char subidxstr[10];

    if (fsm->request->complete_access)
    {
        subidxstr[0] = 0x00;
    }
    else
    {
        sprintf(subidxstr, ":%02X", fsm->request->subindex);
    }

    if (fsm->datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--)
    {
        if (ec_fsm_coe_prepare_up(fsm, datagram))
        {
            fsm->state = ec_fsm_coe_error;
        }
        return;
    }

    if (fsm->datagram->state != EC_DATAGRAM_RECEIVED)
    {
        fsm->request->errno = EIO;
        fsm->state = ec_fsm_coe_error;
        EC_SLAVE_ERR(slave, "Failed to receive CoE upload request: ");
        ec_datagram_print_state(fsm->datagram);
        return;
    }

    diff_ms = (jiffies - fsm->request->jiffies_sent) * 1000 / HZ;

    if (fsm->datagram->working_counter != 1)
    {
        if (!fsm->datagram->working_counter)
        {
            if (diff_ms < fsm->request->response_timeout)
            {
#if DEBUG_RETRIES
                EC_SLAVE_DBG(slave, 1, "Slave did not respond to"
                                       " SDO upload request. Retrying after %lu ms...\n",
                             diff_ms);
#endif
                // no response; send request datagram again
                if (ec_fsm_coe_prepare_up(fsm, datagram))
                {
                    fsm->state = ec_fsm_coe_error;
                }
                return;
            }
        }
        fsm->request->errno = EIO;
        fsm->state = ec_fsm_coe_error;
        EC_SLAVE_ERR(slave, "Reception of CoE upload request for"
                            " SDO 0x%04x%s failed with timeout after %lu ms: ",
                     fsm->request->index, subidxstr, diff_ms);
        ec_datagram_print_wc_error(fsm->datagram);
        return;
    }

#if DEBUG_LONG
    if (diff_ms > 200)
    {
        EC_SLAVE_WARN(slave, "SDO 0x%04x%s upload took %lu ms.\n",
                      fsm->request->index, subidxstr, diff_ms);
    }
#endif

    fsm->jiffies_start = fsm->datagram->jiffies_sent;
    // mailbox read check is skipped if a read request is already ongoing
    if (ec_read_mbox_locked(slave))
    {
        fsm->state = ec_fsm_coe_up_response_data;
        // the datagram is not used and marked as invalid
        datagram->state = EC_DATAGRAM_INVALID;
    }
    else
    {
        ec_slave_mbox_prepare_check(slave, datagram); // can not fail.
        fsm->retries = EC_FSM_RETRIES;
        fsm->state = ec_fsm_coe_up_check;
    }
}

/*****************************************************************************/

/**
   CoE state: UP CHECK.
*/

void ec_fsm_coe_up_check(
    ec_fsm_coe_t *fsm,      /**< Finite state machine. */
    ec_datagram_t *datagram /**< Datagram to use. */
)
{
    ec_slave_t *slave = fsm->slave;

    if (fsm->datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--)
    {
        ec_slave_mbox_prepare_check(slave, datagram); // can not fail.
        return;
    }

    if (fsm->datagram->state != EC_DATAGRAM_RECEIVED)
    {
        fsm->request->errno = EIO;
        fsm->state = ec_fsm_coe_error;
        ec_read_mbox_lock_clear(slave);
        EC_SLAVE_ERR(slave, "Failed to receive CoE mailbox check datagram: ");
        ec_datagram_print_state(fsm->datagram);
        return;
    }

    if (fsm->datagram->working_counter != 1)
    {
        fsm->request->errno = EIO;
        fsm->state = ec_fsm_coe_error;
        ec_read_mbox_lock_clear(slave);
        EC_SLAVE_ERR(slave, "Reception of CoE mailbox check"
                            " datagram failed: ");
        ec_datagram_print_wc_error(fsm->datagram);
        return;
    }

    if (!ec_slave_mbox_check(fsm->datagram))
    {
        unsigned long diff_ms = 0;

        // check that data is not already received by another read request
        if (slave->mbox_coe_data.payload_size > 0)
        {
            ec_read_mbox_lock_clear(slave);
            fsm->state = ec_fsm_coe_up_response_data;
            fsm->state(fsm, datagram);
            return;
        }

        diff_ms = (fsm->datagram->jiffies_received - fsm->jiffies_start) *
                  1000 / HZ;

        if (diff_ms >= fsm->request->response_timeout)
        {
            char subidxstr[10];

            if (fsm->request->complete_access)
            {
                subidxstr[0] = 0x00;
            }
            else
            {
                sprintf(subidxstr, ":%02X", fsm->request->subindex);
            }

            fsm->request->errno = EIO;
            fsm->state = ec_fsm_coe_error;
            ec_read_mbox_lock_clear(slave);
            EC_SLAVE_ERR(slave, "Timeout after %lu ms while waiting for"
                                " SDO 0x%04x%s upload response.\n",
                         diff_ms,
                         fsm->request->index, subidxstr);
            return;
        }

        ec_slave_mbox_prepare_check(slave, datagram); // can not fail.
        fsm->retries = EC_FSM_RETRIES;
        return;
    }

    // Fetch response
    ec_slave_mbox_prepare_fetch(slave, datagram); // can not fail.
    fsm->retries = EC_FSM_RETRIES;
    fsm->state = ec_fsm_coe_up_response;
}

/*****************************************************************************/

/** Prepare an SDO upload segment request.
 */
void ec_fsm_coe_up_prepare_segment_request(
    ec_fsm_coe_t *fsm,      /**< Finite state machine */
    ec_datagram_t *datagram /**< Datagram to use. */
)
{
    uint8_t *data =
        ec_slave_mbox_prepare_send(fsm->slave, datagram, EC_MBOX_TYPE_COE,
                                   10);
    if (IS_ERR(data))
    {
        fsm->request->errno = PTR_ERR(data);
        fsm->state = ec_fsm_coe_error;
        return;
    }

    EC_WRITE_U16(data, 0x2 << 12);          // SDO request
    EC_WRITE_U8(data + 2, (fsm->toggle << 4 // toggle
                           | 0x3 << 5));    // upload segment request
    memset(data + 3, 0x00, 7);

    if (fsm->slave->master->debug_level)
    {
        EC_SLAVE_DBG(fsm->slave, 1, "Upload segment request:\n");
        ec_print_data(data, 10);
    }
}

/*****************************************************************************/

/**
   CoE state: UP RESPONSE.
   \todo Timeout behavior
*/

void ec_fsm_coe_up_response(
    ec_fsm_coe_t *fsm,      /**< Finite state machine. */
    ec_datagram_t *datagram /**< Datagram to use. */
)
{
    ec_slave_t *slave = fsm->slave;
    ec_sdo_request_t *request = fsm->request;

    if (fsm->datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--)
    {
        ec_slave_mbox_prepare_fetch(slave, datagram); // can not fail.
        return;
    }

    if (fsm->datagram->state != EC_DATAGRAM_RECEIVED)
    {
        request->errno = EIO;
        fsm->state = ec_fsm_coe_error;
        ec_read_mbox_lock_clear(slave);
        EC_SLAVE_ERR(slave, "Failed to receive CoE upload response"
                            " datagram: ");
        ec_datagram_print_state(fsm->datagram);
        return;
    }

    if (fsm->datagram->working_counter != 1)
    {
        // only an error if data has not already been read by another read request
        if (slave->mbox_coe_data.payload_size == 0)
        {
            request->errno = EIO;
            fsm->state = ec_fsm_coe_error;
            ec_read_mbox_lock_clear(slave);
            EC_SLAVE_ERR(slave, "Reception of CoE upload response failed: ");
            ec_datagram_print_wc_error(fsm->datagram);
            return;
        }
    }
    ec_read_mbox_lock_clear(slave);
    fsm->state = ec_fsm_coe_up_response_data;
    fsm->state(fsm, datagram);
}

/*****************************************************************************/

/**
   CoE state: UP RESPONSE DATA.

*/

void ec_fsm_coe_up_response_data(
    ec_fsm_coe_t *fsm,      /**< Finite state machine. */
    ec_datagram_t *datagram /**< Datagram to use. */
)
{
    ec_slave_t *slave = fsm->slave;
    ec_master_t *master = slave->master;
    uint16_t rec_index;
    uint8_t *data, mbox_prot, rec_subindex;
    size_t rec_size, data_size;
    ec_sdo_request_t *request = fsm->request;
    unsigned int expedited, size_specified;
    int ret;
    char subidxstr[10];

    if (request->complete_access)
    {
        subidxstr[0] = 0x00;
    }
    else
    {
        sprintf(subidxstr, ":%02X", request->subindex);
    }

    // process the data available or initiate a new mailbox read check
    if (slave->mbox_coe_data.payload_size > 0)
    {
        slave->mbox_coe_data.payload_size = 0;
    }
    else
    {
        // initiate a new mailbox read check if required data is not available
        if (ec_read_mbox_locked(slave))
        {
            // await current read request and mark the datagram as invalid
            datagram->state = EC_DATAGRAM_INVALID;
        }
        else
        {
            ec_slave_mbox_prepare_check(slave, datagram); // can not fail.
            fsm->state = ec_fsm_coe_up_check;
        }
        return;
    }

    data = ec_slave_mbox_fetch(slave, &slave->mbox_coe_data, &mbox_prot, &rec_size);

    if (IS_ERR(data))
    {
        request->errno = PTR_ERR(data);
        fsm->state = ec_fsm_coe_error;
        return;
    }

    if (master->debug_level)
    {
        EC_SLAVE_DBG(slave, 1, "Upload response:\n");
        ec_print_data(data, rec_size);
    }

    if (mbox_prot != EC_MBOX_TYPE_COE)
    {
        request->errno = EIO;
        fsm->state = ec_fsm_coe_error;
        EC_SLAVE_WARN(slave, "Received mailbox protocol 0x%02X"
                             " as response.\n",
                      mbox_prot);
        return;
    }

    if (ec_fsm_coe_check_emergency(fsm, data, rec_size))
    {
        // check for CoE response again
        ec_slave_mbox_prepare_check(slave, datagram); // can not fail.
        fsm->retries = EC_FSM_RETRIES;
        fsm->state = ec_fsm_coe_up_check;
        return;
    }

    if (rec_size < 6)
    {
        request->errno = EIO;
        fsm->state = ec_fsm_coe_error;
        EC_SLAVE_ERR(slave, "Received currupted SDO upload response"
                            " (%zu bytes)!\n",
                     rec_size);
        ec_print_data(data, rec_size);
        return;
    }

    if (EC_READ_U16(data) >> 12 == 0x2 && // SDO request
        EC_READ_U8(data + 2) >> 5 == 0x4)
    { // abort SDO transfer request
        EC_SLAVE_ERR(slave, "SDO upload 0x%04X%s aborted.\n",
                     request->index, subidxstr);
        if (rec_size >= 10)
        {
            request->abort_code = EC_READ_U32(data + 6);
            ec_canopen_abort_msg(slave, request->abort_code);
        }
        else
        {
            EC_SLAVE_ERR(slave, "No abort message.\n");
        }
        request->errno = EIO;
        fsm->state = ec_fsm_coe_error;
        return;
    }

    if (EC_READ_U16(data) >> 12 != 0x3 || // SDO response
        EC_READ_U8(data + 2) >> 5 != 0x2)
    { // upload response
        EC_SLAVE_ERR(slave, "Received unknown response while"
                            " uploading SDO 0x%04X%s.\n",
                     request->index, subidxstr);
        ec_print_data(data, rec_size);
        request->errno = EIO;
        fsm->state = ec_fsm_coe_error;
        return;
    }

    rec_index = EC_READ_U16(data + 3);
    rec_subindex = EC_READ_U8(data + 5);

    if (rec_index != request->index || rec_subindex != request->subindex)
    {
        EC_SLAVE_ERR(slave, "Received upload response for wrong SDO"
                            " (0x%04X:%02X, requested: 0x%04X:%02X).\n",
                     rec_index, rec_subindex, request->index, request->subindex);
        ec_print_data(data, rec_size);

        // check for CoE response again
        ec_slave_mbox_prepare_check(slave, datagram); // can not fail.
        fsm->retries = EC_FSM_RETRIES;
        fsm->state = ec_fsm_coe_up_check;
        return;
    }

    // normal or expedited?
    expedited = EC_READ_U8(data + 2) & 0x02;

    if (expedited)
    {
        size_specified = EC_READ_U8(data + 2) & 0x01;
        if (size_specified)
        {
            fsm->complete_size = 4 - ((EC_READ_U8(data + 2) & 0x0C) >> 2);
        }
        else
        {
            fsm->complete_size = 4;
        }

        if (rec_size < 6 + fsm->complete_size)
        {
            request->errno = EIO;
            fsm->state = ec_fsm_coe_error;
            EC_SLAVE_ERR(slave, "Received corrupted SDO expedited upload"
                                " response (only %zu bytes)!\n",
                         rec_size);
            ec_print_data(data, rec_size);
            return;
        }

        ret = ec_sdo_request_copy_data(request, data + 6, fsm->complete_size);
        if (ret)
        {
            request->errno = -ret;
            fsm->state = ec_fsm_coe_error;
            return;
        }
    }
    else
    { // normal
        if (rec_size < 10)
        {
            request->errno = EIO;
            fsm->state = ec_fsm_coe_error;
            EC_SLAVE_ERR(slave, "Received currupted SDO normal upload"
                                " response (only %zu bytes)!\n",
                         rec_size);
            ec_print_data(data, rec_size);
            return;
        }

        data_size = rec_size - 10;
        fsm->complete_size = EC_READ_U32(data + 6);

        if (!fsm->complete_size)
        {
            request->errno = EIO;
            fsm->state = ec_fsm_coe_error;
            EC_SLAVE_ERR(slave, "No complete size supplied!\n");
            ec_print_data(data, rec_size);
            return;
        }

        ret = ec_sdo_request_alloc(request, fsm->complete_size);
        if (ret)
        {
            request->errno = -ret;
            fsm->state = ec_fsm_coe_error;
            return;
        }

        ret = ec_sdo_request_copy_data(request, data + 10, data_size);
        if (ret)
        {
            request->errno = -ret;
            fsm->state = ec_fsm_coe_error;
            return;
        }

        fsm->toggle = 0;

        if (data_size < fsm->complete_size)
        {
            EC_SLAVE_DBG(slave, 1, "SDO data incomplete (%zu / %u)."
                                   " Segmenting...\n",
                         data_size, fsm->complete_size);
            ec_fsm_coe_up_prepare_segment_request(fsm, datagram);
            fsm->retries = EC_FSM_RETRIES;
            fsm->state = ec_fsm_coe_up_seg_request;
            return;
        }
    }

    if (master->debug_level)
    {
        EC_SLAVE_DBG(slave, 1, "Uploaded data:\n");
        ec_print_data(request->data, request->data_size);
    }

    fsm->state = ec_fsm_coe_end; // success
}

/*****************************************************************************/

/**
   CoE state: UP REQUEST.
   \todo Timeout behavior
*/

void ec_fsm_coe_up_seg_request(
    ec_fsm_coe_t *fsm,      /**< Finite state machine. */
    ec_datagram_t *datagram /**< Datagram to use. */
)
{
    ec_slave_t *slave = fsm->slave;

    if (fsm->datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--)
    {
        ec_fsm_coe_up_prepare_segment_request(fsm, datagram);
        return;
    }

    if (fsm->datagram->state != EC_DATAGRAM_RECEIVED)
    {
        fsm->request->errno = EIO;
        fsm->state = ec_fsm_coe_error;
        EC_SLAVE_ERR(slave, "Failed to receive CoE upload segment"
                            " request datagram: ");
        ec_datagram_print_state(fsm->datagram);
        return;
    }

    if (fsm->datagram->working_counter != 1)
    {
        fsm->request->errno = EIO;
        fsm->state = ec_fsm_coe_error;
        EC_SLAVE_ERR(slave, "Reception of CoE upload segment"
                            " request failed: ");
        ec_datagram_print_wc_error(fsm->datagram);
        return;
    }

    fsm->jiffies_start = fsm->datagram->jiffies_sent;
    // mailbox read check is skipped if a read request is already ongoing
    if (ec_read_mbox_locked(slave))
    {
        fsm->state = ec_fsm_coe_up_seg_response_data;
        // the datagram is not used and marked as invalid
        datagram->state = EC_DATAGRAM_INVALID;
    }
    else
    {
        ec_slave_mbox_prepare_check(slave, datagram); // can not fail.
        fsm->retries = EC_FSM_RETRIES;
        fsm->state = ec_fsm_coe_up_seg_check;
    }
}

/*****************************************************************************/

/**
   CoE state: UP CHECK.
*/

void ec_fsm_coe_up_seg_check(
    ec_fsm_coe_t *fsm,      /**< Finite state machine. */
    ec_datagram_t *datagram /**< Datagram to use. */
)
{
    ec_slave_t *slave = fsm->slave;

    if (fsm->datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--)
    {
        ec_slave_mbox_prepare_check(slave, datagram); // can not fail.
        return;
    }

    if (fsm->datagram->state != EC_DATAGRAM_RECEIVED)
    {
        fsm->request->errno = EIO;
        fsm->state = ec_fsm_coe_error;
        ec_read_mbox_lock_clear(slave);
        EC_SLAVE_ERR(slave, "Failed to receive CoE mailbox check"
                            " datagram: ");
        ec_datagram_print_state(fsm->datagram);
        return;
    }

    if (fsm->datagram->working_counter != 1)
    {
        fsm->request->errno = EIO;
        fsm->state = ec_fsm_coe_error;
        ec_read_mbox_lock_clear(slave);
        EC_SLAVE_ERR(slave, "Reception of CoE mailbox check datagram"
                            " failed: ");
        ec_datagram_print_wc_error(fsm->datagram);
        return;
    }

    if (!ec_slave_mbox_check(fsm->datagram))
    {
        unsigned long diff_ms = 0;

        // check that data is not already received by another read request
        if (slave->mbox_coe_data.payload_size > 0)
        {
            ec_read_mbox_lock_clear(slave);
            fsm->state = ec_fsm_coe_up_seg_response_data;
            fsm->state(fsm, datagram);
            return;
        }

        diff_ms = (fsm->datagram->jiffies_received - fsm->jiffies_start) *
                  1000 / HZ;

        if (diff_ms >= fsm->request->response_timeout)
        {
            fsm->request->errno = EIO;
            fsm->state = ec_fsm_coe_error;
            ec_read_mbox_lock_clear(slave);
            EC_SLAVE_ERR(slave, "Timeout while waiting for SDO upload"
                                " segment response.\n");
            return;
        }

        ec_slave_mbox_prepare_check(slave, datagram); // can not fail.
        fsm->retries = EC_FSM_RETRIES;
        return;
    }

    // Fetch response
    ec_slave_mbox_prepare_fetch(slave, datagram); // can not fail.
    fsm->retries = EC_FSM_RETRIES;
    fsm->state = ec_fsm_coe_up_seg_response;
}

/*****************************************************************************/

/**
   CoE state: UP RESPONSE.
   \todo Timeout behavior
*/

void ec_fsm_coe_up_seg_response(
    ec_fsm_coe_t *fsm,      /**< Finite state machine. */
    ec_datagram_t *datagram /**< Datagram to use. */
)
{
    ec_slave_t *slave = fsm->slave;
    ec_sdo_request_t *request = fsm->request;

    if (fsm->datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--)
    {
        ec_slave_mbox_prepare_fetch(slave, datagram); // can not fail.
        return;
    }

    if (fsm->datagram->state != EC_DATAGRAM_RECEIVED)
    {
        request->errno = EIO;
        fsm->state = ec_fsm_coe_error;
        ec_read_mbox_lock_clear(slave);
        EC_SLAVE_ERR(slave, "Failed to receive CoE upload segment"
                            " response datagram: ");
        ec_datagram_print_state(fsm->datagram);
        return;
    }

    if (fsm->datagram->working_counter != 1)
    {
        // only an error if data has not already been read by another read request
        if (slave->mbox_coe_data.payload_size == 0)
        {
            request->errno = EIO;
            fsm->state = ec_fsm_coe_error;
            ec_read_mbox_lock_clear(slave);
            EC_SLAVE_ERR(slave, "Reception of CoE upload segment"
                                " response failed: ");
            ec_datagram_print_wc_error(fsm->datagram);
            return;
        }
    }
    ec_read_mbox_lock_clear(slave);
    fsm->state = ec_fsm_coe_up_seg_response_data;
    fsm->state(fsm, datagram);
}

/*****************************************************************************/

/**
   CoE state: UP RESPONSE DATA.

*/

void ec_fsm_coe_up_seg_response_data(
    ec_fsm_coe_t *fsm,      /**< Finite state machine. */
    ec_datagram_t *datagram /**< Datagram to use. */
)
{
    ec_slave_t *slave = fsm->slave;
    ec_master_t *master = slave->master;
    uint8_t *data, mbox_prot;
    size_t rec_size, data_size;
    ec_sdo_request_t *request = fsm->request;
    unsigned int last_segment;

    // process the data available or initiate a new mailbox read check
    if (slave->mbox_coe_data.payload_size > 0)
    {
        slave->mbox_coe_data.payload_size = 0;
    }
    else
    {
        // initiate a new mailbox read check if required data is not available
        if (ec_read_mbox_locked(slave))
        {
            // await current read request and mark the datagram as invalid
            datagram->state = EC_DATAGRAM_INVALID;
        }
        else
        {
            ec_slave_mbox_prepare_check(slave, datagram); // can not fail.
            fsm->state = ec_fsm_coe_up_seg_check;
        }
        return;
    }

    data = ec_slave_mbox_fetch(slave, &slave->mbox_coe_data, &mbox_prot, &rec_size);

    if (IS_ERR(data))
    {
        request->errno = PTR_ERR(data);
        fsm->state = ec_fsm_coe_error;
        return;
    }

    if (master->debug_level)
    {
        EC_SLAVE_DBG(slave, 1, "Upload segment response:\n");
        ec_print_data(data, rec_size);
    }

    if (mbox_prot != EC_MBOX_TYPE_COE)
    {
        EC_SLAVE_ERR(slave, "Received mailbox protocol 0x%02X as response.\n",
                     mbox_prot);
        request->errno = EIO;
        fsm->state = ec_fsm_coe_error;
        return;
    }

    if (ec_fsm_coe_check_emergency(fsm, data, rec_size))
    {
        // check for CoE response again
        ec_slave_mbox_prepare_check(slave, datagram); // can not fail.
        fsm->retries = EC_FSM_RETRIES;
        fsm->state = ec_fsm_coe_up_seg_check;
        return;
    }

    if (rec_size < 10)
    {
        EC_SLAVE_ERR(slave, "Received currupted SDO upload"
                            " segment response!\n");
        ec_print_data(data, rec_size);
        request->errno = EIO;
        fsm->state = ec_fsm_coe_error;
        return;
    }

    if (EC_READ_U16(data) >> 12 == 0x2 && // SDO request
        EC_READ_U8(data + 2) >> 5 == 0x4)
    { // abort SDO transfer request
        EC_SLAVE_ERR(slave, "SDO upload 0x%04X:%02X aborted.\n",
                     request->index, request->subindex);
        request->abort_code = EC_READ_U32(data + 6);
        ec_canopen_abort_msg(slave, request->abort_code);
        request->errno = EIO;
        fsm->state = ec_fsm_coe_error;
        return;
    }

    if (EC_READ_U16(data) >> 12 != 0x3 || // SDO response
        EC_READ_U8(data + 2) >> 5 != 0x0)
    { // upload segment response
        if (fsm->slave->master->debug_level)
        {
            EC_SLAVE_DBG(slave, 1, "Invalid SDO upload segment response!\n");
            ec_print_data(data, rec_size);
        }
        // check for CoE response again
        ec_slave_mbox_prepare_check(slave, datagram); // can not fail.
        fsm->retries = EC_FSM_RETRIES;
        fsm->state = ec_fsm_coe_up_seg_check;
        return;
    }

    data_size = rec_size - 3; /* Header of segment upload is smaller than
                                 normal upload */
    if (rec_size == 10)
    {
        uint8_t seg_size = (EC_READ_U8(data + 2) & 0xE) >> 1;
        data_size -= seg_size;
    }

    if (request->data_size + data_size > fsm->complete_size)
    {
        EC_SLAVE_ERR(slave, "SDO upload 0x%04X:%02X failed: Fragment"
                            " exceeding complete size!\n",
                     request->index, request->subindex);
        request->errno = EOVERFLOW;
        fsm->state = ec_fsm_coe_error;
        return;
    }

    memcpy(request->data + request->data_size, data + 3, data_size);
    request->data_size += data_size;

    last_segment = EC_READ_U8(data + 2) & 0x01;
    if (!last_segment)
    {
        fsm->toggle = !fsm->toggle;
        ec_fsm_coe_up_prepare_segment_request(fsm, datagram);
        fsm->retries = EC_FSM_RETRIES;
        fsm->state = ec_fsm_coe_up_seg_request;
        return;
    }

    if (request->data_size != fsm->complete_size)
    {
        EC_SLAVE_WARN(slave, "SDO upload 0x%04X:%02X: Assembled data"
                             " size (%zu) does not match complete size (%u)!\n",
                      request->index, request->subindex,
                      request->data_size, fsm->complete_size);
    }

    if (master->debug_level)
    {
        EC_SLAVE_DBG(slave, 1, "Uploaded data:\n");
        ec_print_data(request->data, request->data_size);
    }

    fsm->state = ec_fsm_coe_end; // success
}

/*****************************************************************************/

/**
   State: ERROR.
*/

void ec_fsm_coe_error(
    ec_fsm_coe_t *fsm,      /**< Finite state machine. */
    ec_datagram_t *datagram /**< Datagram to use. */
)
{
}

/*****************************************************************************/

/**
   State: END.
*/

void ec_fsm_coe_end(
    ec_fsm_coe_t *fsm,      /**< Finite state machine. */
    ec_datagram_t *datagram /**< Datagram to use. */
)
{
}

/*****************************************************************************/

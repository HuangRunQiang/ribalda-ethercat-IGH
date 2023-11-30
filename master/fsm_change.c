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
   EtherCAT state change FSM.
*/

/*****************************************************************************/

#include "globals.h"
#include "master.h"
#include "fsm_change.h"

/*****************************************************************************/

/** 等待AL状态更改的超时时间 [秒]。
*/
#define EC_AL_STATE_CHANGE_TIMEOUT 5

/*****************************************************************************/

void ec_fsm_change_state_start(ec_fsm_change_t *, ec_datagram_t *);
void ec_fsm_change_state_check(ec_fsm_change_t *, ec_datagram_t *);
void ec_fsm_change_state_status(ec_fsm_change_t *, ec_datagram_t *);
void ec_fsm_change_state_start_code(ec_fsm_change_t *, ec_datagram_t *);
void ec_fsm_change_state_code(ec_fsm_change_t *, ec_datagram_t *);
void ec_fsm_change_state_ack(ec_fsm_change_t *, ec_datagram_t *);
void ec_fsm_change_state_check_ack(ec_fsm_change_t *, ec_datagram_t *);
void ec_fsm_change_state_end(ec_fsm_change_t *, ec_datagram_t *);
void ec_fsm_change_state_error(ec_fsm_change_t *, ec_datagram_t *);

/*****************************************************************************/

/**
 * @brief 构造函数。
 *
 * @param fsm 有限状态机。
 *
 * @details 此函数用于初始化有限状态机。
 * 将有限状态机的状态和数据报设置为NULL，并将自发变化标志设置为0。
 */
void ec_fsm_change_init(ec_fsm_change_t *fsm) {
    fsm->state = NULL;
    fsm->datagram = NULL;
    fsm->spontaneous_change = 0;
}


/*****************************************************************************/

/**
 * @brief 析构函数。
 *
 * @param fsm 有限状态机。
 *
 * @details 此函数用于清理有限状态机。
 * 目前该函数没有任何具体的清理操作。
 */

void ec_fsm_change_clear(ec_fsm_change_t *fsm) {
    // 目前该函数没有任何具体的清理操作
}

/*****************************************************************************/

/**
 * @brief 启动状态机的变化。
 *
 * @param fsm 有限状态机。
 * @param slave EtherCAT从站。
 * @param state 请求的状态。
 *
 * @details 此函数用于启动状态机的变化。
 * 将有限状态机的模式设置为全模式（EC_FSM_CHANGE_MODE_FULL）。
 * 设置有限状态机的从站和请求的状态。
 * 将有限状态机的状态设置为ec_fsm_change_state_start。
 */

void ec_fsm_change_start(ec_fsm_change_t *fsm, ec_slave_t *slave, ec_slave_state_t state) {
    fsm->mode = EC_FSM_CHANGE_MODE_FULL;
    fsm->slave = slave;
    fsm->requested_state = state;
    fsm->state = ec_fsm_change_state_start;
}

/*****************************************************************************/

/**
 * @brief 仅确认从站状态的变化。
 *
 * @param fsm 有限状态机。
 * @param slave EtherCAT从站。
 *
 * @details 此函数用于启动仅确认从站状态的变化。
 * 将有限状态机的模式设置为仅确认模式（EC_FSM_CHANGE_MODE_ACK_ONLY）。
 * 设置有限状态机的从站为给定的从站。
 * 将有限状态机的请求状态设置为EC_SLAVE_STATE_UNKNOWN。
 * 将有限状态机的状态设置为ec_fsm_change_state_start_code。
 */

void ec_fsm_change_ack(ec_fsm_change_t *fsm, ec_slave_t *slave) {
    fsm->mode = EC_FSM_CHANGE_MODE_ACK_ONLY;
    fsm->slave = slave;
    fsm->requested_state = EC_SLAVE_STATE_UNKNOWN;
    fsm->state = ec_fsm_change_state_start_code;
}

/*****************************************************************************/

/**
 * @brief 执行当前状态的状态机。
 *
 * @param fsm 有限状态机。
 * @param datagram 要使用的数据报。
 *
 * @return 如果状态机已终止，则返回0；否则返回1。
 *
 * @details 此函数用于执行当前状态的状态机。
 * 如果状态机处于结束状态（ec_fsm_change_state_end）或错误状态（ec_fsm_change_state_error），则返回0。
 * 调用当前状态的函数，并将数据报传递给它。
 * 如果状态机处于结束状态或错误状态，则将数据报设置为NULL并返回0。
 * 否则，将数据报设置为给定的数据报并返回1。
 */

int ec_fsm_change_exec(ec_fsm_change_t *fsm, ec_datagram_t *datagram) {
    if (fsm->state == ec_fsm_change_state_end || fsm->state == ec_fsm_change_state_error)
        return 0;

    fsm->state(fsm, datagram);

    if (fsm->state == ec_fsm_change_state_end || fsm->state == ec_fsm_change_state_error) {
        fsm->datagram = NULL;
        return 0;
    }

    fsm->datagram = datagram;
    return 1;
}

/*****************************************************************************/

/**
 * @brief 返回状态机是否成功终止。
 *
 * @param fsm 有限状态机。
 *
 * @return 如果成功终止，则返回非零值。
 *
 * @details 此函数用于判断状态机是否成功终止。
 * 如果状态机的状态为结束状态（ec_fsm_change_state_end），则返回非零值，表示成功终止。
 * 否则，返回0，表示未成功终止。
 */

int ec_fsm_change_success(ec_fsm_change_t *fsm) {
    return fsm->state == ec_fsm_change_state_end;
}

/******************************************************************************
 *  数据报函数
 *****************************************************************************/

/**
 * @brief 准备写入请求状态。
 *
 * @param fsm 有限状态机。
 * @param datagram 要使用的数据报。
 *
 * @details 此函数用于准备写入请求状态。
 * 将数据报的目标地址设置为从站地址，索引为0x0120，子索引为2。
 * 将请求状态写入数据报的数据字段。
 */

static void ec_fsm_change_prepare_write_requested(ec_fsm_change_t *fsm, ec_datagram_t *datagram) {
    ec_datagram_fpwr(datagram, fsm->slave->station_address, 0x0120, 2);
    EC_WRITE_U16(datagram->data, fsm->requested_state);
}

/*****************************************************************************/

/**
 * @brief 准备写入当前状态。
 *
 * @param fsm 有限状态机。
 * @param datagram 要使用的数据报。
 *
 * @details 此函数用于准备写入当前状态。
 * 将数据报的目标地址设置为从站地址，索引为0x0120，子索引为2。
 * 将当前状态写入数据报的数据字段。
 */

static void ec_fsm_change_prepare_write_current(ec_fsm_change_t *fsm, ec_datagram_t *datagram) {
    ec_datagram_fpwr(datagram, fsm->slave->station_address, 0x0120, 2);
    EC_WRITE_U16(datagram->data, fsm->slave->current_state);
}

/*****************************************************************************/

/**
 * @brief 准备读取状态。
 *
 * @param fsm 有限状态机。
 * @param datagram 要使用的数据报。
 *
 * @details 此函数用于准备读取状态。
 * 将数据报的目标地址设置为从站地址，索引为0x0130，子索引为2。
 * 将数据报的数据字段清零。
 */

static void ec_fsm_change_prepare_read_state(ec_fsm_change_t *fsm, ec_datagram_t *datagram) {
    ec_datagram_fprd(datagram, fsm->slave->station_address, 0x0130, 2);
    ec_datagram_zero(datagram);
}

/*****************************************************************************/

/**
 * @brief 准备读取代码。
 *
 * @param fsm 有限状态机。
 * @param datagram 要使用的数据报。
 *
 * @details 此函数用于准备读取代码。
 * 将数据报的目标地址设置为从站地址，索引为0x0134，子索引为2。
 * 将数据报的数据字段清零。
 */

static void ec_fsm_change_prepare_read_code(ec_fsm_change_t *fsm, ec_datagram_t *datagram) {
    ec_datagram_fprd(datagram, fsm->slave->station_address, 0x0134, 2);
    ec_datagram_zero(datagram);
}

/******************************************************************************
 *  状态变化状态机
 *****************************************************************************/

/**
 * @brief 状态变化：开始。
 *
 * @param fsm 有限状态机。
 * @param datagram 要使用的数据报。
 *
 * @details 此函数用于处理状态变化的开始阶段。
 * 设置状态机的相关变量。
 * 将请求状态写入从站。
 * 将状态机的状态设置为ec_fsm_change_state_check。
 */

void ec_fsm_change_state_start(ec_fsm_change_t *fsm, ec_datagram_t *datagram) {
    fsm->take_time = 1;
    fsm->old_state = fsm->slave->current_state;

    // 将新状态写入从站
    ec_fsm_change_prepare_write_requested(fsm, datagram);
    fsm->retries = EC_FSM_RETRIES;
    fsm->state = ec_fsm_change_state_check;
}


/*****************************************************************************/

/**
 * @brief 状态变化：检查.
 *
 * @param fsm 有限状态机。
 * @param datagram 要使用的数据报。
 *
 * @details 此函数用于处理状态变化的检查阶段。
 * 检查数据报的状态，如果超时且重试次数未用尽，则重新准备写入请求状态。
 * 如果数据报的状态不是接收到，则设置状态机的状态为ec_fsm_change_state_error，并打印错误信息。
 * 如果需要计时，将计时标志设置为0，并记录开始时间。
 * 如果数据报的工作计数器为0，则根据时间判断是否超时。
 * 如果超时，则设置状态机的状态为ec_fsm_change_state_error，并打印错误信息。
 * 否则，重复准备写入请求状态。
 * 如果工作计数器大于1，则设置状态机的状态为ec_fsm_change_state_error，并打印错误信息。
 * 设置计时标志为1。
 * 准备读取从站的AL状态。
 * 设置重试次数。
 * 设置状态机的状态为ec_fsm_change_state_status。
 */

void ec_fsm_change_state_check(ec_fsm_change_t *fsm, ec_datagram_t *datagram) {
    ec_slave_t *slave = fsm->slave;

    if (fsm->datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--) {
        ec_fsm_change_prepare_write_requested(fsm, datagram);
        return;
    }

    if (fsm->datagram->state != EC_DATAGRAM_RECEIVED) {
        fsm->state = ec_fsm_change_state_error;
        EC_SLAVE_ERR(slave, "Failed to receive state datagram: ");
        ec_datagram_print_state(fsm->datagram);
        return;
    }

    if (fsm->take_time) {
        fsm->take_time = 0;
        fsm->jiffies_start = fsm->datagram->jiffies_sent;
    }

    if (fsm->datagram->working_counter == 0) {
        if (fsm->datagram->jiffies_received - fsm->jiffies_start >= 3 * HZ) {
            char state_str[EC_STATE_STRING_SIZE];
            ec_state_string(fsm->requested_state, state_str, 0);
            fsm->state = ec_fsm_change_state_error;
            EC_SLAVE_ERR(slave, "Failed to set state %s: ", state_str);
            ec_datagram_print_wc_error(fsm->datagram);
            return;
        }

        // 重复写入新状态到从站
        ec_fsm_change_prepare_write_requested(fsm, datagram);
        fsm->retries = EC_FSM_RETRIES;
        return;
    }

    if (unlikely(fsm->datagram->working_counter > 1)) {
        char state_str[EC_STATE_STRING_SIZE];
        ec_state_string(fsm->requested_state, state_str, 0);
        fsm->state = ec_fsm_change_state_error;
        EC_SLAVE_ERR(slave, "Failed to set state %s: ", state_str);
        ec_datagram_print_wc_error(fsm->datagram);
        return;
    }

    fsm->take_time = 1;

    // 读取从站的AL状态
    ec_fsm_change_prepare_read_state(fsm, datagram);
    fsm->retries = EC_FSM_RETRIES;
    fsm->spontaneous_change = 0;
    fsm->state = ec_fsm_change_state_status;
}

/*****************************************************************************/

/**
 * @brief 状态变化：状态.
 *
 * @param fsm 有限状态机。
 * @param datagram 要使用的数据报。
 *
 * @details 此函数用于处理状态变化的状态阶段。
 * 检查数据报的状态，如果超时且重试次数未用尽，则重新准备读取状态。
 * 如果数据报的状态不是接收到，则设置状态机的状态为ec_fsm_change_state_error，并打印错误信息。
 * 如果数据报的工作计数器不等于1，则设置状态机的状态为ec_fsm_change_state_error，并打印错误信息。
 * 如果需要计时，将计时标志设置为0，并记录开始时间。
 * 将从数据报中读取的当前状态设置为从站的当前状态。
 * 如果从站的当前状态等于请求的状态，则状态已成功设置，将状态机的状态设置为ec_fsm_change_state_end。
 * 如果从站的当前状态不等于旧状态，则表示状态已更改。
 * 如果从站的当前状态的低4位与旧状态的低4位不相同，则表示从站在新状态写入之前自发地更改了其状态。
 * 将自发更改标志设置为1，并将旧状态设置为从站的当前状态。
 * 打印警告信息。
 * 返回检查状态的阶段。
 * 如果状态变化错误，则将从站的错误标志设置为1。
 * 将请求的状态和当前状态转换为字符串，并打印错误信息。
 * 调用状态变化的开始代码函数。
 */

void ec_fsm_change_state_status(ec_fsm_change_t *fsm, ec_datagram_t *datagram) {
    ec_slave_t *slave = fsm->slave;

    if (fsm->datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--) {
        ec_fsm_change_prepare_read_state(fsm, datagram);
        return;
    }

    if (fsm->datagram->state != EC_DATAGRAM_RECEIVED) {
        fsm->state = ec_fsm_change_state_error;
        EC_SLAVE_ERR(slave, "Failed to receive state checking datagram: ");
        ec_datagram_print_state(fsm->datagram);
        return;
    }

    if (fsm->datagram->working_counter != 1) {
        char req_state[EC_STATE_STRING_SIZE];
        ec_state_string(fsm->requested_state, req_state, 0);
        fsm->state = ec_fsm_change_state_error;
        EC_SLAVE_ERR(slave, "Failed to check state %s: ", req_state);
        ec_datagram_print_wc_error(fsm->datagram);
        return;
    }

    if (fsm->take_time) {
        fsm->take_time = 0;
        fsm->jiffies_start = fsm->datagram->jiffies_sent;
    }

    slave->current_state = EC_READ_U8(fsm->datagram->data);

    if (slave->current_state == fsm->requested_state) {
        // 状态已成功设置
        fsm->state = ec_fsm_change_state_end;
        return;
    }

    if (slave->current_state != fsm->old_state) { // 状态已更改
        char req_state[EC_STATE_STRING_SIZE], cur_state[EC_STATE_STRING_SIZE];

        ec_state_string(slave->current_state, cur_state, 0);

        if ((slave->current_state & 0x0F) != (fsm->old_state & 0x0F)) {
            // 从站在新状态写入之前自发地更改了其状态
            // 将当前状态设置为旧状态，并等待状态变化
            fsm->spontaneous_change = 1;
            fsm->old_state = slave->current_state;
            EC_SLAVE_WARN(slave, "Changed to %s in the meantime.\n", cur_state);
            goto check_again;
        }

        // 状态变化错误

        slave->error_flag = 1;
        ec_state_string(fsm->requested_state, req_state, 0);

        EC_SLAVE_ERR(slave, "Failed to set %s state, slave refused state change (%s).\n",
                     req_state, cur_state);

        ec_fsm_change_state_start_code(fsm, datagram);
        return;
    }

    // 仍然是旧状态

    if (fsm->datagram->jiffies_received - fsm->jiffies_start >= EC_AL_STATE_CHANGE_TIMEOUT * HZ) {
        // 检查超时
        char state_str[EC_STATE_STRING_SIZE];
        ec_state_string(fsm->requested_state, state_str, 0);
        fsm->state = ec_fsm_change_state_error;
        EC_SLAVE_ERR(slave, "Timeout while setting state %s.\n", state_str);
        return;
    }

check_again:
    // 尚未超时，再次检查
    ec_fsm_change_prepare_read_state(fsm, datagram);
    fsm->retries = EC_FSM_RETRIES;
}

/*****************************************************************************/

/**
 * @brief 进入读取AL状态代码。
 *
 * @param fsm 有限状态机。
 * @param datagram 要使用的数据报。
 *
 * @details 此函数用于进入读取AL状态代码阶段。
 * 准备读取错误代码。
 * 设置重试次数。
 * 设置状态机的状态为ec_fsm_change_state_code。
 */

void ec_fsm_change_state_start_code(ec_fsm_change_t *fsm, ec_datagram_t *datagram) {
    // 获取AL状态错误代码
    ec_fsm_change_prepare_read_code(fsm, datagram);
    fsm->retries = EC_FSM_RETRIES;
    fsm->state = ec_fsm_change_state_code;
}

/*****************************************************************************/

/**
   应用层状态消息。
*/

const ec_code_msg_t al_status_messages[] = {
    {0x0000, "无错误"},
    {0x0001, "未指定的错误"},
    {0x0002, "无可用内存"},
    {0x0011, "无效的请求状态更改"},
    {0x0012, "未知的请求状态"},
    {0x0013, "不支持引导配置"},
    {0x0014, "无有效固件"},
    {0x0015, "无效的邮箱配置"},
    {0x0016, "无效的邮箱配置"},
    {0x0017, "无效的同步管理器配置"},
    {0x0018, "无有效的输入可用"},
    {0x0019, "无有效的输出"},
    {0x001A, "同步错误"},
    {0x001B, "同步管理器看门狗"},
    {0x001C, "无效的同步管理器类型"},
    {0x001D, "无效的输出配置"},
    {0x001E, "无效的输入配置"},
    {0x001F, "无效的看门狗配置"},
    {0x0020, "从站需要冷启动"},
    {0x0021, "从站需要 INIT"},
    {0x0022, "从站需要 PREOP"},
    {0x0023, "从站需要 SAFEOP"},
    {0x0024, "无效的输入映射"},
    {0x0025, "无效的输出映射"},
    {0x0026, "设置不一致"},
    {0x0027, "不支持自由运行"},
    {0x0028, "不支持同步"},
    {0x0029, "自由运行需要 3 缓冲模式"},
    {0x002A, "后台看门狗"},
    {0x002B, "无有效的输入和输出"},
    {0x002C, "致命同步错误"},
    {0x002D, "无同步错误"},
    {0x0030, "无效的 DC SYNCH 配置"},
    {0x0031, "无效的 DC latch 配置"},
    {0x0032, "PLL 错误"},
    {0x0033, "DC 同步 IO 错误"},
    {0x0034, "DC 同步超时错误"},
    {0x0035, "无效的 DC 同步周期时间"},
    {0x0036, "DC 同步0周期时间"},
    {0x0037, "DC 同步1周期时间"},
    {0x0041, "MBX_AOE"},
    {0x0042, "MBX_EOE"},
    {0x0043, "MBX_COE"},
    {0x0044, "MBX_FOE"},
    {0x0045, "MBX_SOE"},
    {0x004F, "MBX_VOE"},
    {0x0050, "EEPROM 无法访问"},
    {0x0051, "EEPROM 错误"},
    {0x0060, "从站本地重新启动"},
    {0xffff, ""}};

/*****************************************************************************/

/**
 * @brief 状态变化：CODE.
 *
 * @param fsm 有限状态机。
 * @param datagram 要使用的数据报。
 *
 * @details 此函数用于处理状态变化的CODE阶段。
 * 检查数据报的状态，如果超时且重试次数未用尽，则重新准备读取代码。
 * 如果数据报的状态不是接收到，则设置状态机的状态为ec_fsm_change_state_error，并打印错误信息。
 * 如果数据报的工作计数器不等于1，则打印警告信息，并将从站的last_al_error设置为0。
 * 否则，从数据报中读取错误代码，并将其设置为从站的last_al_error。
 * 遍历AL状态消息列表，查找与错误代码匹配的消息。
 * 如果找到匹配的消息，则打印AL状态消息和消息内容。
 * 如果未在列表中找到匹配的消息，则打印未知的AL状态代码。
 * 准备写入"旧"从站状态的确认。
 * 设置重试次数。
 * 设置状态机的状态为ec_fsm_change_state_ack。
 */

void ec_fsm_change_state_code(ec_fsm_change_t *fsm, ec_datagram_t *datagram) {
    uint32_t code;
    const ec_code_msg_t *al_msg;

    if (fsm->datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--) {
        ec_fsm_change_prepare_read_code(fsm, datagram);
        return;
    }

    if (fsm->datagram->state != EC_DATAGRAM_RECEIVED) {
        fsm->state = ec_fsm_change_state_error;
        EC_SLAVE_ERR(fsm->slave, "Failed to receive AL status code datagram: ");
        ec_datagram_print_state(fsm->datagram);
        return;
    }

    if (fsm->datagram->working_counter != 1) {
        EC_SLAVE_WARN(fsm->slave, "Reception of AL status code datagram failed: ");
        ec_datagram_print_wc_error(fsm->datagram);
        fsm->slave->last_al_error = 0;
    } else {
        code = EC_READ_U16(fsm->datagram->data);
        fsm->slave->last_al_error = code;
        for (al_msg = al_status_messages; al_msg->code != 0xffff; al_msg++) {
            if (al_msg->code != code) {
                continue;
            }

            EC_SLAVE_ERR(fsm->slave, "AL status message 0x%04X: \"%s\".\n",
                         al_msg->code, al_msg->message);
            break;
        }
        if (al_msg->code == 0xffff) { /* not found in our list. */
            EC_SLAVE_ERR(fsm->slave, "Unknown AL status code 0x%04X.\n",
                         code);
        }
    }

    // 确认"旧"从站状态
    ec_fsm_change_prepare_write_current(fsm, datagram);
    fsm->retries = EC_FSM_RETRIES;
    fsm->state = ec_fsm_change_state_ack;
}

/*****************************************************************************/

/**
 * @brief 状态变化：ACK.
 *
 * @param fsm 有限状态机。
 * @param datagram 要使用的数据报。
 *
 * @details 此函数用于处理状态变化的ACK阶段。
 * 检查数据报的状态，如果超时且重试次数未用尽，则重新准备写入当前状态。
 * 如果数据报的状态不是接收到，则设置状态机的状态为ec_fsm_change_state_error，并打印错误信息。
 * 如果数据报的工作计数器不等于1，则设置状态机的状态为ec_fsm_change_state_error，并打印错误信息。
 * 设置计时标志为1。
 * 准备读取新的AL状态。
 * 设置重试次数。
 * 设置状态机的状态为ec_fsm_change_state_check_ack。
 */

void ec_fsm_change_state_ack(ec_fsm_change_t *fsm, ec_datagram_t *datagram) {
    ec_slave_t *slave = fsm->slave;

    if (fsm->datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--) {
        ec_fsm_change_prepare_write_current(fsm, datagram);
        return;
    }

    if (fsm->datagram->state != EC_DATAGRAM_RECEIVED) {
        fsm->state = ec_fsm_change_state_error;
        EC_SLAVE_ERR(slave, "Failed to receive state ack datagram: ");
        ec_datagram_print_state(fsm->datagram);
        return;
    }

    if (fsm->datagram->working_counter != 1) {
        fsm->state = ec_fsm_change_state_error;
        EC_SLAVE_ERR(slave, "Reception of state ack datagram failed: ");
        ec_datagram_print_wc_error(fsm->datagram);
        return;
    }

    fsm->take_time = 1;

    // 读取新的AL状态
    ec_fsm_change_prepare_read_state(fsm, datagram);
    fsm->retries = EC_FSM_RETRIES;
    fsm->state = ec_fsm_change_state_check_ack;
}

/*****************************************************************************/

/**
 * @brief 状态变化：CHECK ACK.
 *
 * @param fsm 有限状态机。
 * @param datagram 要使用的数据报。
 *
 * @details 此函数用于处理状态变化的CHECK ACK阶段。
 * 检查数据报的状态，如果超时且重试次数未用尽，则重新准备读取状态。
 * 如果数据报的状态不是接收到，则设置状态机的状态为ec_fsm_change_state_error，并打印错误信息。
 * 如果数据报的工作计数器不等于1，则设置状态机的状态为ec_fsm_change_state_error，并打印错误信息。
 * 如果需要计时，将计时标志设置为0，并记录开始时间。
 * 将从数据报中读取的当前状态设置为从站的当前状态。
 * 如果从站的当前状态的ACK_ERR位为0，则表示状态已被确认。
 * 将请求的状态转换为字符串，并打印信息。
 * 如果是EC_FSM_CHANGE_MODE_FULL模式，则将状态机的状态设置为ec_fsm_change_state_error。
 * 否则，如果是EC_FSM_CHANGE_MODE_ACK_ONLY模式，则将状态机的状态设置为ec_fsm_change_state_end。
 * 返回。
 * 如果从数据报的接收时间减去开始时间大于等于EC_AL_STATE_CHANGE_TIMEOUT * HZ，则表示检查超时。
 * 将状态机的状态设置为ec_fsm_change_state_error，并打印错误信息。
 * 准备读取新的AL状态。
 * 设置重试次数。
 */

void ec_fsm_change_state_check_ack(ec_fsm_change_t *fsm, ec_datagram_t *datagram) {
    ec_slave_t *slave = fsm->slave;

    if (fsm->datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--) {
        ec_fsm_change_prepare_read_state(fsm, datagram);
        return;
    }

    if (fsm->datagram->state != EC_DATAGRAM_RECEIVED) {
        fsm->state = ec_fsm_change_state_error;
        EC_SLAVE_ERR(slave, "Failed to receive state ack check datagram: ");
        ec_datagram_print_state(fsm->datagram);
        return;
    }

    if (fsm->datagram->working_counter != 1) {
        fsm->state = ec_fsm_change_state_error;
        EC_SLAVE_ERR(slave, "Reception of state ack check datagram failed: ");
        ec_datagram_print_wc_error(fsm->datagram);
        return;
    }

    if (fsm->take_time) {
        fsm->take_time = 0;
        fsm->jiffies_start = fsm->datagram->jiffies_sent;
    }

    slave->current_state = EC_READ_U8(fsm->datagram->data);

    if (!(slave->current_state & EC_SLAVE_STATE_ACK_ERR)) {
        char state_str[EC_STATE_STRING_SIZE];
        ec_state_string(slave->current_state, state_str, 0);
        if (fsm->mode == EC_FSM_CHANGE_MODE_FULL) {
            fsm->state = ec_fsm_change_state_error;
        } else { // EC_FSM_CHANGE_MODE_ACK_ONLY
            fsm->state = ec_fsm_change_state_end;
        }
        EC_SLAVE_INFO(slave, "Acknowledged state %s.\n", state_str);
        return;
    }

    if (fsm->datagram->jiffies_received - fsm->jiffies_start >= EC_AL_STATE_CHANGE_TIMEOUT * HZ) {
        // 检查超时
        char state_str[EC_STATE_STRING_SIZE];
        ec_state_string(slave->current_state, state_str, 0);
        fsm->state = ec_fsm_change_state_error;
        EC_SLAVE_ERR(slave, "Timeout while acknowledging state %s.\n",
                     state_str);
        return;
    }

    // 重新读取新的AL状态
    ec_fsm_change_prepare_read_state(fsm, datagram);
    fsm->retries = EC_FSM_RETRIES;
}

/*****************************************************************************/

/**
 * 状态: 错误（ERROR）。
 *
 * 当有限状态机处于错误状态时调用该函数。
 *
 * @param fsm 有限状态机的指针。
 * @param datagram 要使用的数据报。
 */
void ec_fsm_change_state_error(
    ec_fsm_change_t *fsm,   /**< 有限状态机 */
    ec_datagram_t *datagram /**< 要使用的数据报 */
)
{
}

/*****************************************************************************/

/**
 * 状态: 结束（END）。
 *
 * 当有限状态机处于结束状态时调用该函数。
 *
 * @param fsm 有限状态机的指针。
 * @param datagram 要使用的数据报。
 */
void ec_fsm_change_state_end(
    ec_fsm_change_t *fsm,   /**< 有限状态机 */
    ec_datagram_t *datagram /**< 要使用的数据报 */
)
{
}


/*****************************************************************************/

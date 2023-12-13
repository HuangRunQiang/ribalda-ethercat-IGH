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
 *
 * EtherCAT从站配置状态机。
 */

/*****************************************************************************/

#include <asm/div64.h>

#include "globals.h"
#include "master.h"
#include "mailbox.h"
#include "slave_config.h"
#include "fsm_slave_config.h"

/*****************************************************************************/

/** 时钟差异的最大值（以纳秒为单位），在进入SAFEOP之前。
 *
 * 在要求SAFEOP之前，等待DC时间差异降到这个绝对值以下。
 */
#define EC_DC_MAX_SYNC_DIFF_NS 10000

/** 最长等待时钟纪律的时间（以毫秒为单位）。
 */
#define EC_DC_SYNC_WAIT_MS 5000

/** 周期性开始时间加上的时间偏移量（以纳秒为单位）。 */
 */
#define EC_DC_START_OFFSET 100000000ULL

/*****************************************************************************/

void ec_fsm_slave_config_state_start(ec_fsm_slave_config_t *, ec_datagram_t *);
void ec_fsm_slave_config_state_quick_start(ec_fsm_slave_config_t *, ec_datagram_t *);
void ec_fsm_slave_config_state_init(ec_fsm_slave_config_t *, ec_datagram_t *);
void ec_fsm_slave_config_state_clear_fmmus(ec_fsm_slave_config_t *, ec_datagram_t *);
void ec_fsm_slave_config_state_clear_sync(ec_fsm_slave_config_t *, ec_datagram_t *);
void ec_fsm_slave_config_state_dc_clear_assign(ec_fsm_slave_config_t *, ec_datagram_t *);
void ec_fsm_slave_config_state_mbox_sync(ec_fsm_slave_config_t *, ec_datagram_t *);
#ifdef EC_SII_ASSIGN
void ec_fsm_slave_config_state_assign_pdi(ec_fsm_slave_config_t *, ec_datagram_t *);
#endif
void ec_fsm_slave_config_state_boot_preop(ec_fsm_slave_config_t *, ec_datagram_t *);
#ifdef EC_SII_ASSIGN
void ec_fsm_slave_config_state_assign_ethercat(ec_fsm_slave_config_t *, ec_datagram_t *);
#endif
void ec_fsm_slave_config_state_sdo_conf(ec_fsm_slave_config_t *, ec_datagram_t *);
void ec_fsm_slave_config_state_soe_conf_preop(ec_fsm_slave_config_t *, ec_datagram_t *);
void ec_fsm_slave_config_state_watchdog_divider(ec_fsm_slave_config_t *, ec_datagram_t *);
void ec_fsm_slave_config_state_watchdog(ec_fsm_slave_config_t *, ec_datagram_t *);
void ec_fsm_slave_config_state_pdo_sync(ec_fsm_slave_config_t *, ec_datagram_t *);
void ec_fsm_slave_config_state_pdo_conf(ec_fsm_slave_config_t *, ec_datagram_t *);
void ec_fsm_slave_config_state_fmmu(ec_fsm_slave_config_t *, ec_datagram_t *);
void ec_fsm_slave_config_state_dc_cycle(ec_fsm_slave_config_t *, ec_datagram_t *);
void ec_fsm_slave_config_state_dc_sync_check(ec_fsm_slave_config_t *, ec_datagram_t *);
void ec_fsm_slave_config_state_dc_start(ec_fsm_slave_config_t *, ec_datagram_t *);
void ec_fsm_slave_config_state_dc_assign(ec_fsm_slave_config_t *, ec_datagram_t *);
void ec_fsm_slave_config_state_safeop(ec_fsm_slave_config_t *, ec_datagram_t *);
void ec_fsm_slave_config_state_soe_conf_safeop(ec_fsm_slave_config_t *, ec_datagram_t *);
void ec_fsm_slave_config_state_op(ec_fsm_slave_config_t *, ec_datagram_t *);

void ec_fsm_slave_config_enter_init(ec_fsm_slave_config_t *, ec_datagram_t *);
void ec_fsm_slave_config_enter_clear_sync(ec_fsm_slave_config_t *, ec_datagram_t *);
void ec_fsm_slave_config_enter_dc_clear_assign(ec_fsm_slave_config_t *, ec_datagram_t *);
void ec_fsm_slave_config_enter_mbox_sync(ec_fsm_slave_config_t *, ec_datagram_t *);
#ifdef EC_SII_ASSIGN
void ec_fsm_slave_config_enter_assign_pdi(ec_fsm_slave_config_t *, ec_datagram_t *);
#endif
void ec_fsm_slave_config_enter_boot_preop(ec_fsm_slave_config_t *, ec_datagram_t *);
void ec_fsm_slave_config_enter_sdo_conf(ec_fsm_slave_config_t *, ec_datagram_t *);
void ec_fsm_slave_config_enter_soe_conf_preop(ec_fsm_slave_config_t *, ec_datagram_t *);
void ec_fsm_slave_config_enter_pdo_conf(ec_fsm_slave_config_t *, ec_datagram_t *);
void ec_fsm_slave_config_enter_watchdog_divider(ec_fsm_slave_config_t *, ec_datagram_t *);
void ec_fsm_slave_config_enter_watchdog(ec_fsm_slave_config_t *, ec_datagram_t *);
void ec_fsm_slave_config_enter_pdo_sync(ec_fsm_slave_config_t *, ec_datagram_t *);
void ec_fsm_slave_config_enter_fmmu(ec_fsm_slave_config_t *, ec_datagram_t *);
void ec_fsm_slave_config_enter_dc_cycle(ec_fsm_slave_config_t *, ec_datagram_t *);
void ec_fsm_slave_config_enter_safeop(ec_fsm_slave_config_t *, ec_datagram_t *);
void ec_fsm_slave_config_enter_soe_conf_safeop(ec_fsm_slave_config_t *, ec_datagram_t *);
void ec_fsm_slave_config_enter_op(ec_fsm_slave_config_t *, ec_datagram_t *);

void ec_fsm_slave_config_state_end(ec_fsm_slave_config_t *, ec_datagram_t *);
void ec_fsm_slave_config_state_error(ec_fsm_slave_config_t *, ec_datagram_t *);

void ec_fsm_slave_config_reconfigure(ec_fsm_slave_config_t *, ec_datagram_t *);

/*****************************************************************************/

/**
 * @brief 构造函数。
 * @param fsm 从站状态机
 * @param slave 要配置的从站
 * @param fsm_change 状态变化状态机
 * @param fsm_coe CoE状态机
 * @param fsm_soe SoE状态机
 * @param fsm_pdo PDO配置状态机
 * @return 无
 * @details 初始化从站配置状态机的各个成员变量。
 */
void ec_fsm_slave_config_init(
    ec_fsm_slave_config_t *fsm,  /**< 从站状态机 */
    ec_slave_t *slave,           /**< 要配置的从站 */
    ec_fsm_change_t *fsm_change, /**< 状态变化状态机 */
    ec_fsm_coe_t *fsm_coe,       /**< CoE状态机 */
    ec_fsm_soe_t *fsm_soe,       /**< SoE状态机 */
    ec_fsm_pdo_t *fsm_pdo        /**< PDO配置状态机 */
)
{
    ec_sdo_request_init(&fsm->request_copy);
    ec_soe_request_init(&fsm->soe_request_copy);

    fsm->slave = slave;
    fsm->datagram = NULL;
    fsm->fsm_change = fsm_change;
    fsm->fsm_coe = fsm_coe;
    fsm->fsm_soe = fsm_soe;
    fsm->fsm_pdo = fsm_pdo;
}

/*****************************************************************************/

/**
 * @brief 析构函数。
 * @param fsm 从站状态机
 * @return 无
 * @details 清理从站配置状态机的资源。
 */
void ec_fsm_slave_config_clear(
    ec_fsm_slave_config_t *fsm /**< 从站状态机 */
)
{
    ec_sdo_request_clear(&fsm->request_copy);
    ec_soe_request_clear(&fsm->soe_request_copy);
}

/*****************************************************************************/

/**
 * @brief 启动从站配置状态机。
 * @param fsm 从站状态机
 * @return 无
 * @details 将从站配置状态机的状态设置为开始状态。
 */
void ec_fsm_slave_config_start(
    ec_fsm_slave_config_t *fsm /**< 从站状态机 */
)
{
    fsm->state = ec_fsm_slave_config_state_start;
}

/*****************************************************************************/

/**
 * @brief 启动从站配置状态机，用于“快速”SAFEOP->OP。
 * @param fsm 从站状态机
 * @return 无
 * @details 将从站配置状态机的状态设置为快速开始状态。
 */
void ec_fsm_slave_config_quick_start(
    ec_fsm_slave_config_t *fsm /**< 从站状态机 */
)
{
    fsm->state = ec_fsm_slave_config_state_quick_start;
}

/*****************************************************************************/

/**
 * @brief 检查从站配置状态机是否正在运行。
 * @param fsm 从站状态机
 * @return 如果状态机已终止，则返回false；否则返回true。
 * @details 检查从站配置状态机的状态是否为终止状态或错误状态。
 */
int ec_fsm_slave_config_running(
    const ec_fsm_slave_config_t *fsm /**< 从站状态机 */
)
{
    return fsm->state != ec_fsm_slave_config_state_end && fsm->state != ec_fsm_slave_config_state_error;
}

/*****************************************************************************/

/**
 * @brief 执行当前状态机的当前状态。
 * @param fsm 从站状态机
 * @param datagram 使用的数据报
 * @return 如果状态机已终止，则返回false；否则返回true。
 * @details 如果状态机的数据报尚未发送或接收，则将状态机的执行延迟到下一个周期。
 */
int ec_fsm_slave_config_exec(
    ec_fsm_slave_config_t *fsm, /**< 从站状态机 */
    ec_datagram_t *datagram     /**< 使用的数据报 */
)
{
    if (!ec_fsm_slave_config_running(fsm))
        return 0;

    fsm->state(fsm, datagram);

    if (!ec_fsm_slave_config_running(fsm))
    {
        fsm->datagram = NULL;
        return 0;
    }

    fsm->datagram = datagram;
    return 1;
}

/*****************************************************************************/

/**
 * @brief 检查从站配置状态机是否成功终止。
 * @param fsm 从站状态机
 * @return 如果状态机以正常终止，则返回true；否则返回false。
 * @details 检查从站配置状态机的状态是否为终止状态。
 */
int ec_fsm_slave_config_success(
    const ec_fsm_slave_config_t *fsm /**< 从站状态机 */
)
{
    return fsm->state == ec_fsm_slave_config_state_end;
}

/******************************************************************************
 * 从站配置状态机
 *****************************************************************************/

/**
 * @brief 从站配置状态：开始。
 * @param fsm 从站状态机
 * @param datagram 使用的数据报
 * @return 无
 * @details 进入初始化状态。
 */
void ec_fsm_slave_config_state_start(
    ec_fsm_slave_config_t *fsm, /**< 从站状态机 */
    ec_datagram_t *datagram     /**< 使用的数据报 */
)
{
    EC_SLAVE_DBG(fsm->slave, 1, "配置中...\n");
    ec_fsm_slave_config_enter_init(fsm, datagram);
}

/*****************************************************************************/

/**
 * @brief 从站配置状态：快速开始。
 * @param fsm 从站状态机
 * @param datagram 使用的数据报
 * @return 无
 * @details 进入安全操作状态的SoE配置。
 */
void ec_fsm_slave_config_state_quick_start(
    ec_fsm_slave_config_t *fsm, /**< 从站状态机 */
    ec_datagram_t *datagram     /**< 使用的数据报 */
)
{
    EC_SLAVE_DBG(fsm->slave, 1, "配置中（快速）...\n");
    ec_fsm_slave_config_enter_soe_conf_safeop(fsm, datagram);
}

/*****************************************************************************/

/**
 * @brief 进入初始化状态变化。
 * @param fsm 从站状态机
 * @param datagram 使用的数据报
 * @return 无
 * @details 启动状态变化到INIT状态的过程。
 */
void ec_fsm_slave_config_enter_init(
    ec_fsm_slave_config_t *fsm, /**< 从站状态机 */
    ec_datagram_t *datagram     /**< 使用的数据报 */
)
{
    ec_fsm_change_start(fsm->fsm_change, fsm->slave, EC_SLAVE_STATE_INIT);
    ec_fsm_change_exec(fsm->fsm_change, datagram);
    fsm->state = ec_fsm_slave_config_state_init;
}

/*****************************************************************************/

/**
 * @brief 从站配置状态：INIT。
 * @param fsm 从站状态机
 * @param datagram 使用的数据报
 * @return 无
 * @details 初始化状态的逻辑：
 * - 执行状态变化到INIT状态的过程。
 * - 如果状态变化过程执行成功，则继续下一步。
 * - 如果状态变化过程执行失败：
 *   - 如果不是自发变化，则设置从站的错误标志，并将状态设置为错误状态。
 *   - 如果是自发变化，则将状态设置为错误状态。
 * - 打印日志，表示从站已进入INIT状态。
 * - 如果从站没有基本的FMMU配置，跳过FMMU配置步骤，进入清除同步配置的步骤。
 * - 打印日志，表示正在清除FMMU配置。
 * - 清除FMMU配置。
 * - 设置重试次数为EC_FSM_RETRIES。
 * - 将状态设置为清除FMMU状态。
 * - 打印日志，表示正在清除邮箱检查标志。
 * - 清除邮箱检查标志。
 */
void ec_fsm_slave_config_state_init(
    ec_fsm_slave_config_t *fsm, /**< 从站状态机 */
    ec_datagram_t *datagram     /**< 使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;

    if (ec_fsm_change_exec(fsm->fsm_change, datagram))
        return;

    if (!ec_fsm_change_success(fsm->fsm_change))
    {
        if (!fsm->fsm_change->spontaneous_change)
            slave->error_flag = 1;
        fsm->state = ec_fsm_slave_config_state_error;
        return;
    }

    EC_SLAVE_DBG(slave, 1, "现在处于INIT状态。\n");

    if (!slave->base_fmmu_count)
    { // 跳过FMMU配置
        ec_fsm_slave_config_enter_clear_sync(fsm, datagram);
        return;
    }

    EC_SLAVE_DBG(slave, 1, "正在清除FMMU配置...\n");

    // 清除FMMU配置
    ec_datagram_fpwr(datagram, slave->station_address,
                     0x0600, EC_FMMU_PAGE_SIZE * slave->base_fmmu_count);
    ec_datagram_zero(datagram);
    fsm->retries = EC_FSM_RETRIES;
    fsm->state = ec_fsm_slave_config_state_clear_fmmus;

    EC_SLAVE_DBG(slave, 1, "正在清除邮箱检查标志...\n");

    ec_read_mbox_lock_clear(slave);
}

/*****************************************************************************/

/**
 * @brief 从站配置状态：CLEAR FMMU。
 * @param fsm 从站状态机
 * @param datagram 使用的数据报
 * @return 无
 * @details 清除FMMU配置的逻辑：
 * - 如果数据报超时且重试次数不为0，则重复发送数据报。
 * - 如果数据报状态不为接收到，则将状态设置为错误状态，并打印错误日志。
 * - 如果数据报的工作计数器不为1，则设置从站的错误标志，并将状态设置为错误状态。
 * - 进入清除同步配置的步骤。
 */
void ec_fsm_slave_config_state_clear_fmmus(
    ec_fsm_slave_config_t *fsm, /**< 从站状态机 */
    ec_datagram_t *datagram     /**< 使用的数据报 */
)
{
    if (fsm->datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--)
    {
        ec_datagram_repeat(datagram, fsm->datagram);
        return;
    }

    if (fsm->datagram->state != EC_DATAGRAM_RECEIVED)
    {
        fsm->state = ec_fsm_slave_config_state_error;
        EC_SLAVE_ERR(fsm->slave, "接收FMMU清除数据报失败。\n");
        return;
    }

    if (fsm->datagram->working_counter != 1)
    {
        fsm->slave->error_flag = 1;
        fsm->state = ec_fsm_slave_config_state_error;
        EC_SLAVE_ERR(fsm->slave, "清除FMMU配置失败：");
        ec_datagram_print_wc_error(fsm->datagram);
        return;
    }

    ec_fsm_slave_config_enter_clear_sync(fsm, datagram);
}

/*****************************************************************************/

/**
 * @brief 进入清除同步配置。
 * @param fsm 从站状态机
 * @param datagram 使用的数据报
 * @return 无
 * @details 清除同步配置的逻辑：
 * - 如果从站没有基本的同步配置，进入清除DC分配的步骤。
 * - 打印日志，表示正在清除同步配置。
 * - 清除同步配置。
 * - 设置重试次数为EC_FSM_RETRIES。
 * - 将状态设置为清除同步状态。
 */
void ec_fsm_slave_config_enter_clear_sync(
    ec_fsm_slave_config_t *fsm, /**< 从站状态机 */
    ec_datagram_t *datagram     /**< 使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;
    size_t sync_size;

    if (!slave->base_sync_count)
    {
        // 没有同步管理器
        ec_fsm_slave_config_enter_dc_clear_assign(fsm, datagram);
        return;
    }

    EC_SLAVE_DBG(slave, 1, "正在清除同步配置...\n");

    sync_size = EC_SYNC_PAGE_SIZE * slave->base_sync_count;

    // 清除同步配置
    ec_datagram_fpwr(datagram, slave->station_address, 0x0800, sync_size);
    ec_datagram_zero(datagram);
    fsm->retries = EC_FSM_RETRIES;
    fsm->state = ec_fsm_slave_config_state_clear_sync;
}

/*****************************************************************************/

/**
 * @brief 从站配置状态：CLEAR SYNC。
 * @param fsm 从站状态机
 * @param datagram 使用的数据报
 * @return 无
 * @details 清除同步配置的逻辑：
 * - 如果数据报超时且重试次数不为0，则重复发送数据报。
 * - 如果数据报状态不为接收到，则将状态设置为错误状态，并打印错误日志。
 * - 如果数据报的工作计数器不为1，则设置从站的错误标志，并将状态设置为错误状态。
 * - 进入清除DC分配的步骤。
 */
void ec_fsm_slave_config_state_clear_sync(
    ec_fsm_slave_config_t *fsm, /**< 从站状态机 */
    ec_datagram_t *datagram     /**< 使用的数据报 */
)
{
    if (fsm->datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--)
    {
        ec_datagram_repeat(datagram, fsm->datagram);
        return;
    }

    if (fsm->datagram->state != EC_DATAGRAM_RECEIVED)
    {
        fsm->state = ec_fsm_slave_config_state_error;
        EC_SLAVE_ERR(fsm->slave, "接收同步配置清除数据报失败。\n");
        return;
    }

    if (fsm->datagram->working_counter != 1)
    {
        fsm->slave->error_flag = 1;
        fsm->state = ec_fsm_slave_config_state_error;
        EC_SLAVE_ERR(fsm->slave, "清除同步配置失败：");
        ec_datagram_print_wc_error(fsm->datagram);
        return;
    }

    ec_fsm_slave_config_enter_dc_clear_assign(fsm, datagram);
}

/*****************************************************************************/

/**
 * @brief 进入清除DC分配。
 * @param fsm 从站状态机
 * @param datagram 使用的数据报
 * @return 无
 * @details 清除DC分配的逻辑：
 * - 如果从站不支持基于时间的同步或没有DC系统时间，进入邮箱同步的步骤。
 * - 打印日志，表示正在清除DC分配。
 * - 清除DC分配。
 * - 设置重试次数为EC_FSM_RETRIES。
 * - 将状态设置为清除DC分配状态。
 */
void ec_fsm_slave_config_enter_dc_clear_assign(
    ec_fsm_slave_config_t *fsm, /**< 从站状态机 */
    ec_datagram_t *datagram     /**< 使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;

    if (!slave->base_dc_supported || !slave->has_dc_system_time)
    {
        ec_fsm_slave_config_enter_mbox_sync(fsm, datagram);
        return;
    }

    EC_SLAVE_DBG(slave, 1, "正在清除DC分配...\n");

    ec_datagram_fpwr(datagram, slave->station_address, 0x0980, 2);
    ec_datagram_zero(datagram);
    fsm->retries = EC_FSM_RETRIES;
    fsm->state = ec_fsm_slave_config_state_dc_clear_assign;
}

/*****************************************************************************/

/**
 * @brief 从站配置状态：CLEAR DC ASSIGN。
 * @param fsm 从站状态机
 * @param datagram 使用的数据报
 * @return 无
 * @details 清除DC分配的逻辑：
 * - 如果数据报超时且重试次数不为0，则重复发送数据报。
 * - 如果数据报状态不为接收到，则将状态设置为错误状态，并打印错误日志。
 * - 如果数据报的工作计数器不为1：
 *   - 对于简单从站，清除DC分配不会成功。
 *   - 对于非简单从站，打印日志，表示清除DC分配失败。
 * - 进入邮箱同步的步骤。
 */
void ec_fsm_slave_config_state_dc_clear_assign(
    ec_fsm_slave_config_t *fsm, /**< 从站状态机 */
    ec_datagram_t *datagram     /**< 使用的数据报 */
)
{
    if (fsm->datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--)
    {
        ec_datagram_repeat(datagram, fsm->datagram);
        return;
    }

    if (fsm->datagram->state != EC_DATAGRAM_RECEIVED)
    {
        fsm->state = ec_fsm_slave_config_state_error;
        EC_SLAVE_ERR(fsm->slave, "接收DC分配清除数据报失败。\n");
        return;
    }

    if (fsm->datagram->working_counter != 1)
    {
        // 清除DC分配对于简单从站不会成功
        EC_SLAVE_DBG(fsm->slave, 1, "清除DC分配失败：");
        ec_datagram_print_wc_error(fsm->datagram);
    }

    ec_fsm_slave_config_enter_mbox_sync(fsm, datagram);
}

/*****************************************************************************/

/**
 * @brief 检查是否需要配置邮箱同步管理器。
 * @param fsm 从站状态机
 * @param datagram 使用的数据报
 * @return 无
 * @details 配置邮箱同步管理器的逻辑：
 * - 如果从站已经进入了INIT状态，则配置完成。
 * - 如果从站没有SII数据，则设置状态为错误状态，并打印错误日志。
 * - 如果从站不支持邮箱通信协议，则跳过邮箱同步配置步骤，进入分配PDI的步骤或启动预操作的步骤。
 * - 打印日志，表示正在配置邮箱同步管理器。
 * - 如果从站请求的状态为BOOT：
 *   - 配置引导阶段的邮箱同步管理器。
 *   - 设置从站的已配置接收邮箱偏移量和大小。
 *   - 设置从站的已配置发送邮箱偏移量和大小。
 * - 如果从站的SII数据提供了同步配置：
 *   - 配置同步管理器。
 *   - 设置从站的已配置接收邮箱偏移量和大小。
 *   - 设置从站的已配置发送邮箱偏移量和大小。
 * - 否则，从站没有提供邮箱同步管理器配置：
 *   - 打印日志，表示从站没有提供邮箱同步管理器配置。
 *   - 配置默认的同步管理器。
 *   - 设置从站的已配置接收邮箱偏移量和大小。
 *   - 设置从站的已配置发送邮箱偏移量和大小。
 * - 为支持的邮箱通信协议分配邮箱响应数据的内存。
 * - 设置需要花费时间的标志。
 * - 设置重试次数为EC_FSM_RETRIES。
 * - 将状态设置为邮箱同步状态。
 */
void ec_fsm_slave_config_enter_mbox_sync(
    ec_fsm_slave_config_t *fsm, /**< 从站状态机 */
    ec_datagram_t *datagram     /**< 使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;
    unsigned int i;

    // 从站已经进入INIT状态
    if (fsm->slave->current_state == slave->requested_state)
    {
        fsm->state = ec_fsm_slave_config_state_end; // 配置完成
        EC_SLAVE_DBG(slave, 1, "配置完成。\n");
        return;
    }

    if (!slave->sii_image)
    {
        fsm->state = ec_fsm_slave_config_state_error;
        EC_SLAVE_ERR(slave, "从站无法配置同步管理器。SII数据不可用。\n");
        return;
    }

    if (!slave->sii_image->sii.mailbox_protocols)
    {
        // 不支持邮箱通信协议
        EC_SLAVE_DBG(slave, 1, "从站不支持邮箱通信。\n");
#ifdef EC_SII_ASSIGN
        ec_fsm_slave_config_enter_assign_pdi(fsm, datagram);
#else
        ec_fsm_slave_config_enter_boot_preop(fsm, datagram);
#endif
        return;
    }

    EC_SLAVE_DBG(slave, 1, "正在配置邮箱同步管理器...\n");

    if (slave->requested_state == EC_SLAVE_STATE_BOOT)
    {
        ec_sync_t sync;

        ec_datagram_fpwr(datagram, slave->station_address, 0x0800,
                         EC_SYNC_PAGE_SIZE * 2);
        ec_datagram_zero(datagram);

        ec_sync_init(&sync, slave);
        sync.physical_start_address = slave->sii_image->sii.boot_rx_mailbox_offset;
        sync.control_register = 0x26;
        sync.enable = 1;
        ec_sync_page(&sync, 0, slave->sii_image->sii.boot_rx_mailbox_size,
                     EC_DIR_INVALID, // 使用默认方向
                     0,              // 没有PDO传输
                     datagram->data);
        slave->configured_rx_mailbox_offset =
            slave->sii_image->sii.boot_rx_mailbox_offset;
        slave->configured_rx_mailbox_size =
            slave->sii_image->sii.boot_rx_mailbox_size;

        ec_sync_init(&sync, slave);
        sync.physical_start_address = slave->sii_image->sii.boot_tx_mailbox_offset;
        sync.control_register = 0x22;
        sync.enable = 1;
        ec_sync_page(&sync, 1, slave->sii_image->sii.boot_tx_mailbox_size,
                     EC_DIR_INVALID, // 使用默认方向
                     0,              // 没有PDO传输
                     datagram->data + EC_SYNC_PAGE_SIZE);
        slave->configured_tx_mailbox_offset =
            slave->sii_image->sii.boot_tx_mailbox_offset;
        slave->configured_tx_mailbox_size =
            slave->sii_image->sii.boot_tx_mailbox_size;
    }
    else if (slave->sii_image->sii.sync_count >= 2)
    { // 提供了邮箱配置
        ec_datagram_fpwr(datagram, slave->station_address, 0x0800,
                         EC_SYNC_PAGE_SIZE * slave->sii_image->sii.sync_count);
        ec_datagram_zero(datagram);

        if (slave->sii_image->sii.syncs)
        {
            for (i = 0; i < 2; i++)
            {
                ec_sync_page(&slave->sii_image->sii.syncs[i], i,
                             slave->sii_image->sii.syncs[i].default_length,
                             NULL, // 使用默认的同步管理器配置
                             0,    // 没有PDO传输
                             datagram->data + EC_SYNC_PAGE_SIZE * i);
            }

            slave->configured_rx_mailbox_offset =
                slave->sii_image->sii.syncs[0].physical_start_address;
            slave->configured_rx_mailbox_size =
                slave->sii_image->sii.syncs[0].default_length;
            slave->configured_tx_mailbox_offset =
                slave->sii_image->sii.syncs[1].physical_start_address;
            slave->configured_tx_mailbox_size =
                slave->sii_image->sii.syncs[1].default_length;
        }
        else
        {
            EC_SLAVE_ERR(slave, "从站没有同步管理器\n");
        }
    }
    else
    { // 没有提供邮箱同步管理器配置
        ec_sync_t sync;

        EC_SLAVE_DBG(slave, 1, "从站没有提供邮箱同步管理器配置。\n");

        ec_datagram_fpwr(datagram, slave->station_address, 0x0800,
                         EC_SYNC_PAGE_SIZE * 2);
        ec_datagram_zero(datagram);

        ec_sync_init(&sync, slave);
        sync.physical_start_address = slave->sii_image->sii.std_rx_mailbox_offset;
        sync.control_register = 0x26;
        sync.enable = 1;
        ec_sync_page(&sync, 0, slave->sii_image->sii.std_rx_mailbox_size,
                     NULL, // 使用默认的同步管理器配置
                     0,    // 没有PDO传输
                     datagram->data);
        slave->configured_rx_mailbox_offset =
            slave->sii_image->sii.std_rx_mailbox_offset;
        slave->configured_rx_mailbox_size =
            slave->sii_image->sii.std_rx_mailbox_size;

        ec_sync_init(&sync, slave);
        sync.physical_start_address = slave->sii_image->sii.std_tx_mailbox_offset;
        sync.control_register = 0x22;
        sync.enable = 1;
        ec_sync_page(&sync, 1, slave->sii_image->sii.std_tx_mailbox_size,
                     NULL, // 使用默认的同步管理器配置
                     0,    // 没有PDO传输
                     datagram->data + EC_SYNC_PAGE_SIZE);
        slave->configured_tx_mailbox_offset =
            slave->sii_image->sii.std_tx_mailbox_offset;
        slave->configured_tx_mailbox_size =
            slave->sii_image->sii.std_tx_mailbox_size;
    }

    // 为支持的邮箱通信协议分配邮箱响应数据的内存
    ec_mbox_prot_data_prealloc(slave, slave->sii_image->sii.mailbox_protocols, slave->configured_tx_mailbox_size);

    fsm->take_time = 1;

    fsm->retries = EC_FSM_RETRIES;
    fsm->state = ec_fsm_slave_config_state_mbox_sync;
}

/*****************************************************************************/

/**
 * @brief 从站配置状态：邮箱同步。
 * @param fsm 从站状态机
 * @param datagram 使用的数据报
 * @return 无
 * @details 邮箱同步的逻辑：
 * - 如果数据报超时且重试次数不为0，则重复发送数据报。
 * - 如果数据报状态不为接收到，则将状态设置为错误状态，并打印错误日志。
 * - 如果需要计时：
 *   - 将计时标志设置为0。
 *   - 将起始节拍数设置为数据报发送的节拍数。
 * - 如果数据报的工作计数器为0：
 *   - 计算节拍数的差值。
 *   - 如果差值大于等于HZ（一秒的节拍数）：
 *     - 设置从站的错误标志，并将状态设置为错误状态。
 *     - 打印日志，表示配置邮箱同步管理器超时。
 *     - 返回。
 *   - 否则：
 *     - 打印日志，表示经过一定时间后重新发送数据报。
 *     - 重新发送配置数据报。
 *     - 设置重试次数为EC_FSM_RETRIES。
 *     - 返回。
 * - 如果数据报的工作计数器不为1：
 *   - 设置从站的错误标志，并将状态设置为错误状态。
 *   - 打印日志，表示配置同步管理器失败。
 *   - 返回。
 * - 根据编译选项决定下一步进入的状态：
 *   - 如果定义了EC_SII_ASSIGN：
 *     - 进入分配PDI的步骤。
 *   - 否则：
 *     - 进入启动预操作的步骤。
 */
void ec_fsm_slave_config_state_mbox_sync(
    ec_fsm_slave_config_t *fsm, /**< 从站状态机 */
    ec_datagram_t *datagram     /**< 使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;

    if (fsm->datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--)
    {
        ec_datagram_repeat(datagram, fsm->datagram);
        return;
    }

    if (fsm->datagram->state != EC_DATAGRAM_RECEIVED)
    {
        fsm->state = ec_fsm_slave_config_state_error;
        EC_SLAVE_ERR(slave, "接收同步管理器配置数据报失败：");
        ec_datagram_print_state(fsm->datagram);
        return;
    }

    if (fsm->take_time)
    {
        fsm->take_time = 0;
        fsm->jiffies_start = fsm->datagram->jiffies_sent;
    }

    /* 由于同步管理器配置在上一个周期中被清除，因此有些从站不会立即响应邮箱同步管理器配置数据报。
     * 因此，在一定时间内，如果从站没有响应，则重新发送数据报。
     */
    if (fsm->datagram->working_counter == 0)
    {
        unsigned long diff = fsm->datagram->jiffies_received - fsm->jiffies_start;

        if (diff >= HZ)
        {
            slave->error_flag = 1;
            fsm->state = ec_fsm_slave_config_state_error;
            EC_SLAVE_ERR(slave, "配置邮箱同步管理器超时。\n");
            return;
        }
        else
        {
            EC_SLAVE_DBG(slave, 1, "经过 %u 毫秒后重新发送...\n",
                         (unsigned int)diff * 1000 / HZ);
        }

        // 重新发送配置数据报
        ec_datagram_repeat(datagram, fsm->datagram);
        fsm->retries = EC_FSM_RETRIES;
        return;
    }
    else if (fsm->datagram->working_counter != 1)
    {
        slave->error_flag = 1;
        fsm->state = ec_fsm_slave_config_state_error;
        EC_SLAVE_ERR(slave, "配置同步管理器失败：");
        ec_datagram_print_wc_error(fsm->datagram);
        return;
    }

#ifdef EC_SII_ASSIGN
    ec_fsm_slave_config_enter_assign_pdi(fsm, datagram);
#else
    ec_fsm_slave_config_enter_boot_preop(fsm, datagram);
#endif
}

/*****************************************************************************/

#ifdef EC_SII_ASSIGN

/**
 * @brief 分配SII给PDI。
 * @param fsm 从站状态机
 * @param datagram 使用的数据报
 * @return 无
 * @details 分配SII给PDI的逻辑：
 * - 如果从站请求的状态不是BOOT：
 *   - 打印日志，表示正在分配SII访问给PDI。
 *   - 发送分配SII给PDI的数据报。
 *   - 设置重试次数为EC_FSM_RETRIES。
 *   - 将状态设置为分配PDI状态。
 * - 否则，进入启动预操作的步骤。
 */
void ec_fsm_slave_config_enter_assign_pdi(
    ec_fsm_slave_config_t *fsm, /**< 从站状态机 */
    ec_datagram_t *datagram     /**< 使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;

    if (fsm->slave->requested_state != EC_SLAVE_STATE_BOOT)
    {
        EC_SLAVE_DBG(slave, 1, "正在分配SII访问给PDI。\n");

        ec_datagram_fpwr(datagram, slave->station_address, 0x0500, 0x01);
        EC_WRITE_U8(datagram->data, 0x01); // PDI
        fsm->retries = EC_FSM_RETRIES;
        fsm->state = ec_fsm_slave_config_state_assign_pdi;
    }
    else
    {
        ec_fsm_slave_config_enter_boot_preop(fsm, datagram);
    }
}

/*****************************************************************************/

/**
 * @brief 从站配置状态：分配PDI。
 * @param fsm 从站状态机
 * @param datagram 使用的数据报
 * @return 无
 * @details 分配PDI的逻辑：
 * - 如果数据报超时且重试次数不为0，则重复发送数据报。
 * - 如果数据报状态不为接收到，则打印警告日志。
 * - 如果数据报的工作计数器不为1，则打印警告日志，表示分配SII给PDI失败。
 * - 进入启动预操作的步骤。
 */
void ec_fsm_slave_config_state_assign_pdi(
    ec_fsm_slave_config_t *fsm, /**< 从站状态机 */
    ec_datagram_t *datagram     /**< 使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;

    if (fsm->datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--)
    {
        ec_datagram_repeat(datagram, fsm->datagram);
        return;
    }

    if (fsm->datagram->state != EC_DATAGRAM_RECEIVED)
    {
        EC_SLAVE_WARN(slave, "接收SII分配数据报失败：");
        ec_datagram_print_state(fsm->datagram);
        goto cont_preop;
    }

    if (fsm->datagram->working_counter != 1)
    {
        EC_SLAVE_WARN(slave, "分配SII给PDI失败：");
        ec_datagram_print_wc_error(fsm->datagram);
    }

cont_preop:
    ec_fsm_slave_config_enter_boot_preop(fsm, datagram);
}

#endif


/*****************************************************************************/

/**
@brief 请求PREOP状态。
@param fsm 从站状态机
@param datagram 要使用的数据报
@return 无
@details
- 将状态机的状态设置为`ec_fsm_slave_config_state_boot_preop`。
- 如果从站的请求状态不是`EC_SLAVE_STATE_BOOT`，则调用`ec_fsm_change_start`函数，将状态机的状态改为`EC_SLAVE_STATE_PREOP`。
- 否则，调用`ec_fsm_change_start`函数，将状态机的状态改为`EC_SLAVE_STATE_BOOT`。
- 调用`ec_fsm_change_exec`函数，立即执行状态改变。
*/
void ec_fsm_slave_config_enter_boot_preop(
    ec_fsm_slave_config_t *fsm, /**< 从站状态机 */
    ec_datagram_t *datagram     /**< 要使用的数据报 */
)
{
    fsm->state = ec_fsm_slave_config_state_boot_preop;

    if (fsm->slave->requested_state != EC_SLAVE_STATE_BOOT)
    {
        ec_fsm_change_start(fsm->fsm_change,
                            fsm->slave, EC_SLAVE_STATE_PREOP);
    }
    else
    { // BOOT
        ec_fsm_change_start(fsm->fsm_change,
                            fsm->slave, EC_SLAVE_STATE_BOOT);
    }

    ec_fsm_change_exec(fsm->fsm_change, datagram); // 立即执行
}

/*****************************************************************************/

/** 
@brief 从站配置状态：BOOT/PREOP。
@param fsm 从站状态机
@param datagram 要使用的数据报
@return 无
@details
- 获取从站指针。
- 如果执行状态改变成功，返回。
- 如果执行状态改变失败，并且不是自发性改变，则将从站的错误标志设置为1。
- 将状态机的状态设置为`ec_fsm_slave_config_state_error`。
- 从站现在处于BOOT或PREOP状态。
- 将从站的预操作节拍数设置为接收到的数据报的节拍数。
- 打印调试信息，指示从站现在处于PREOP状态还是BOOT状态。
- 如果定义了`EC_SII_ASSIGN`宏：
  - 打印调试信息，指示将SII访问重新分配给EtherCAT。
  - 使用`ec_datagram_fpwr`函数，向数据报写入分配SII访问的指令。
  - 将数据报的数据字段写入值0x00，表示EtherCAT。
  - 将状态机的重试次数设置为`EC_FSM_RETRIES`。
  - 将状态机的状态设置为`ec_fsm_slave_config_state_assign_ethercat`。
- 否则，如果从站的当前状态等于请求的状态：
  - 将状态机的状态设置为`ec_fsm_slave_config_state_end`，表示配置成功结束。
  - 打印调试信息，指示配置已完成。
  - 返回。
- 调用`ec_fsm_slave_config_enter_sdo_conf`函数，进入SDO配置状态。
*/
void ec_fsm_slave_config_state_boot_preop(
    ec_fsm_slave_config_t *fsm, /**< 从站状态机 */
    ec_datagram_t *datagram     /**< 要使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;

    if (ec_fsm_change_exec(fsm->fsm_change, datagram))
    {
        return;
    }

    if (!ec_fsm_change_success(fsm->fsm_change))
    {
        if (!fsm->fsm_change->spontaneous_change)
            slave->error_flag = 1;
        fsm->state = ec_fsm_slave_config_state_error;
        return;
    }

    // 从站现在处于BOOT或PREOP状态
    slave->jiffies_preop = fsm->datagram->jiffies_received;

    EC_SLAVE_DBG(slave, 1, "现在处于%s状态。\n",
                 slave->requested_state != EC_SLAVE_STATE_BOOT ? "PREOP" : "BOOT");

#ifdef EC_SII_ASSIGN
    EC_SLAVE_DBG(slave, 1, "将SII访问重新分配给EtherCAT。\n");

    ec_datagram_fpwr(datagram, slave->station_address, 0x0500, 0x01);
    EC_WRITE_U8(datagram->data, 0x00); // EtherCAT
    fsm->retries = EC_FSM_RETRIES;
    fsm->state = ec_fsm_slave_config_state_assign_ethercat;
#else
    if (slave->current_state == slave->requested_state)
    {
        fsm->state = ec_fsm_slave_config_state_end; // 成功
        EC_SLAVE_DBG(slave, 1, "配置完成。\n");
        return;
    }

    ec_fsm_slave_config_enter_sdo_conf(fsm, datagram);
#endif
}

/*****************************************************************************/

#ifdef EC_SII_ASSIGN

/**
 * @brief 从EtherCAT分配从站配置状态。
 * @param fsm 从站状态机。
 * @param datagram 使用的数据报。
 * @return 无。
 * @details 该函数用于处理从站配置状态为ASSIGN_ETHERCAT的情况。它根据数据报的状态执行相应的操作，包括重发数据报、打印错误信息等。如果从站的当前状态和请求的状态相同，则表示配置完成，函数将状态设置为结束状态，并打印完成配置的信息。
 */
void ec_fsm_slave_config_state_assign_ethercat(
    ec_fsm_slave_config_t *fsm, /**< 从站状态机 */
    ec_datagram_t *datagram     /**< 使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;

    if (fsm->datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--)
    {
        ec_datagram_repeat(datagram, fsm->datagram);
        return;
    }

    if (fsm->datagram->state != EC_DATAGRAM_RECEIVED)
    {
        EC_SLAVE_WARN(slave, "接收SII分配数据报失败：");
        ec_datagram_print_state(fsm->datagram);
        goto cont_sdo_conf;
    }

    if (fsm->datagram->working_counter != 1)
    {
        EC_SLAVE_WARN(slave, "将SII重新分配给EtherCAT失败：");
        ec_datagram_print_wc_error(fsm->datagram);
    }

cont_sdo_conf:
    if (slave->current_state == slave->requested_state)
    {
        fsm->state = ec_fsm_slave_config_state_end; // 配置成功
        EC_SLAVE_DBG(slave, 1, "配置完成。\n");
        return;
    }

    ec_fsm_slave_config_enter_sdo_conf(fsm, datagram);
}

#endif

/*****************************************************************************/

/**
 * @brief 检查是否存在需要应用的SDO配置。
 * @param fsm 从站状态机。
 * @param datagram 使用的数据报。
 * @return 无。
 * @details 该函数用于检查是否存在需要应用的SDO配置。如果从站的配置为空，则跳过SDO配置，进入PDO同步配置状态；
 * 如果SDO配置为空，则跳过SDO配置，进入SOE_CONF_PREOP状态；否则，开始进行SDO配置，将状态设置为SDO_CONF，执行SDO配置操作。
 */
void ec_fsm_slave_config_enter_sdo_conf(
    ec_fsm_slave_config_t *fsm, /**< 从站状态机 */
    ec_datagram_t *datagram     /**< 使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;

    if (!slave->config)
    {
        ec_fsm_slave_config_enter_pdo_sync(fsm, datagram);
        return;
    }

    // 没有需要应用的CoE配置？
    if (list_empty(&slave->config->sdo_configs))
    { // 跳过SDO配置
        ec_fsm_slave_config_enter_soe_conf_preop(fsm, datagram);
        return;
    }

    // 开始进行SDO配置
    fsm->state = ec_fsm_slave_config_state_sdo_conf;
    fsm->request = list_entry(fsm->slave->config->sdo_configs.next,
                              ec_sdo_request_t, list);
    ec_sdo_request_copy(&fsm->request_copy, fsm->request);
    ecrt_sdo_request_write(&fsm->request_copy);
    ec_fsm_coe_transfer(fsm->fsm_coe, fsm->slave, &fsm->request_copy);
    ec_fsm_coe_exec(fsm->fsm_coe, datagram); // 立即执行
}

/*****************************************************************************/

/**
 * @brief 从站配置状态：SDO_CONF。
 * @param fsm 从站状态机。
 * @param datagram 使用的数据报。
 * @return 无。
 * @details 该函数用于处理从站配置状态为SDO_CONF的情况。它执行SDO配置操作，并根据执行结果进行相应的处理，包括打印错误信息、设置错误标志等。
 * 如果配置过程中从站的配置被移除，则重新配置；如果还有其他的SDO需要配置，则继续进行下一个SDO的配置；如果所有的SDO配置完成，则进入SOE_CONF_PREOP状态。
 */
void ec_fsm_slave_config_state_sdo_conf(
    ec_fsm_slave_config_t *fsm, /**< 从站状态机 */
    ec_datagram_t *datagram     /**< 使用的数据报 */
)
{
    if (ec_fsm_coe_exec(fsm->fsm_coe, datagram))
    {
        return;
    }

    if (!ec_fsm_coe_success(fsm->fsm_coe))
    {
        EC_SLAVE_ERR(fsm->slave, "SDO配置失败。\n");
        fsm->slave->error_flag = 1;
        fsm->state = ec_fsm_slave_config_state_error;
        return;
    }

    if (!fsm->slave->config)
    { // 配置在此期间被移除
        ec_fsm_slave_config_reconfigure(fsm, datagram);
        return;
    }

    // 还有其他的SDO需要配置？
    if (fsm->request->list.next != &fsm->slave->config->sdo_configs)
    {
        fsm->request = list_entry(fsm->request->list.next,
                                  ec_sdo_request_t, list);
        ec_sdo_request_copy(&fsm->request_copy, fsm->request);
        ecrt_sdo_request_write(&fsm->request_copy);
        ec_fsm_coe_transfer(fsm->fsm_coe, fsm->slave, &fsm->request_copy);
        ec_fsm_coe_exec(fsm->fsm_coe, datagram); // 立即执行
        return;
    }

    // 所有的SDO配置完成
    ec_fsm_slave_config_enter_soe_conf_preop(fsm, datagram);
}

/*****************************************************************************/

/**
@brief 检查是否需要应用SoE配置。
@param fsm 从站状态机
@param datagram 使用的数据报
@return 无
@details
- 如果从站的配置为空，则调用ec_fsm_slave_config_enter_pdo_sync函数并返回。
- 遍历从站配置中的所有SoE配置。
  - 如果配置的AL状态为PREOP：
    - 开始SoE配置。
    - 将状态机的状态设置为ec_fsm_slave_config_state_soe_conf_preop。
    - 设置状态机的soe_request为当前配置。
    - 复制soe_request到soe_request_copy。
    - 将soe_request_copy写入。
    - 调用ec_fsm_soe_transfer函数执行SoE传输。
    - 调用ec_fsm_soe_exec函数执行SoE操作。
    - 返回。
- 如果PREOP中没有需要应用的SoE配置：
  - 调用ec_fsm_slave_config_enter_pdo_conf函数并返回。
*/

void ec_fsm_slave_config_enter_soe_conf_preop(
    ec_fsm_slave_config_t *fsm, /**< 从站状态机 */
    ec_datagram_t *datagram     /**< 使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;
    ec_soe_request_t *req;

    if (!slave->config)
    {
        ec_fsm_slave_config_enter_pdo_sync(fsm, datagram);
        return;
    }

    list_for_each_entry(req, &slave->config->soe_configs, list)
    {
        if (req->al_state == EC_AL_STATE_PREOP)
        {
            // 开始SoE配置
            fsm->state = ec_fsm_slave_config_state_soe_conf_preop;
            fsm->soe_request = req;
            ec_soe_request_copy(&fsm->soe_request_copy, fsm->soe_request);
            ec_soe_request_write(&fsm->soe_request_copy);
            ec_fsm_soe_transfer(fsm->fsm_soe, fsm->slave,
                                &fsm->soe_request_copy);
            ec_fsm_soe_exec(fsm->fsm_soe, datagram);
            return;
        }
    }

    // PREOP中没有需要应用的SoE配置
    ec_fsm_slave_config_enter_pdo_conf(fsm, datagram);
}

/*****************************************************************************/

/**
@brief 从站配置状态：SOE_CONF。
@param fsm 从站状态机
@param datagram 使用的数据报
@return 无
@details
- 如果ec_fsm_soe_exec函数返回true，则返回。
- 如果ec_fsm_soe_success函数返回false：
  - 打印"SoE configuration failed."错误消息。
  - 设置从站的error_flag为1。
  - 设置状态机的状态为ec_fsm_slave_config_state_error。
  - 返回。
- 如果从站的配置为空：
  - 调用ec_fsm_slave_config_reconfigure函数重新配置。
  - 返回。
- 当前PREOP中是否还有其他IDN需要配置？
  - 如果有：
    - 更新soe_request为下一个IDN。
    - 如果soe_request的AL状态为PREOP：
      - 复制soe_request到soe_request_copy。
      - 将soe_request_copy写入。
      - 调用ec_fsm_soe_transfer函数执行SoE传输。
      - 调用ec_fsm_soe_exec函数执行SoE操作。
      - 返回。
- 所有PREOP中的IDN都已配置完成。
- 调用ec_fsm_slave_config_enter_pdo_conf函数并返回。
*/

void ec_fsm_slave_config_state_soe_conf_preop(
    ec_fsm_slave_config_t *fsm, /**< 从站状态机 */
    ec_datagram_t *datagram     /**< 使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;

    if (ec_fsm_soe_exec(fsm->fsm_soe, datagram))
    {
        return;
    }

    if (!ec_fsm_soe_success(fsm->fsm_soe))
    {
        EC_SLAVE_ERR(slave, "SoE配置失败。\n");
        fsm->slave->error_flag = 1;
        fsm->state = ec_fsm_slave_config_state_error;
        return;
    }

    if (!fsm->slave->config)
    { // 配置在此期间被移除
        ec_fsm_slave_config_reconfigure(fsm, datagram);
        return;
    }

    // 是否还有其他IDN需要在PREOP中配置？
    while (fsm->soe_request->list.next != &fsm->slave->config->soe_configs)
    {
        fsm->soe_request = list_entry(fsm->soe_request->list.next,
                                      ec_soe_request_t, list);
        if (fsm->soe_request->al_state == EC_AL_STATE_PREOP)
        {
            ec_soe_request_copy(&fsm->soe_request_copy, fsm->soe_request);
            ec_soe_request_write(&fsm->soe_request_copy);
            ec_fsm_soe_transfer(fsm->fsm_soe, fsm->slave,
                                &fsm->soe_request_copy);
            ec_fsm_soe_exec(fsm->fsm_soe, datagram);
            return;
        }
    }

    // 所有PREOP中的IDN都已配置完成
    ec_fsm_slave_config_enter_pdo_conf(fsm, datagram);
}

/*****************************************************************************/

/**
@brief PDO_CONF入口函数。
@param fsm 从站状态机
@param datagram 使用的数据报
@return 无
@details
- 开始配置PDOs。
- 设置状态机的状态为ec_fsm_slave_config_state_pdo_conf。
- 立即执行状态机。
*/

void ec_fsm_slave_config_enter_pdo_conf(
    ec_fsm_slave_config_t *fsm, /**< 从站状态机 */
    ec_datagram_t *datagram     /**< 使用的数据报 */
)
{
    // 开始配置PDOs
    ec_fsm_pdo_start_configuration(fsm->fsm_pdo, fsm->slave);
    fsm->state = ec_fsm_slave_config_state_pdo_conf;
    fsm->state(fsm, datagram); // 立即执行
}

/*****************************************************************************/

/**
@brief 从站配置状态：PDO_CONF。
@param fsm 从站状态机
@param datagram 使用的数据报
@return 无
@details
- TODO：在此处检查配置。
- 如果ec_fsm_pdo_exec函数返回true，则返回。
- 如果从站的配置为空：
  - 调用ec_fsm_slave_config_reconfigure函数重新配置。
  - 返回。
- 如果ec_fsm_pdo_success函数返回false：
  - 打印"PDO configuration failed."警告消息。
- 调用ec_fsm_slave_config_enter_watchdog_divider函数并返回。
*/

void ec_fsm_slave_config_state_pdo_conf(
    ec_fsm_slave_config_t *fsm, /**< 从站状态机 */
    ec_datagram_t *datagram     /**< 使用的数据报 */
)
{
    // TODO：在此处检查配置

    if (ec_fsm_pdo_exec(fsm->fsm_pdo, datagram))
    {
        return;
    }

    if (!fsm->slave->config)
    { // 配置在此期间被移除
        ec_fsm_slave_config_reconfigure(fsm, datagram);
        return;
    }

    if (!ec_fsm_pdo_success(fsm->fsm_pdo))
    {
        EC_SLAVE_WARN(fsm->slave, "PDO配置失败。\n");
    }

    ec_fsm_slave_config_enter_watchdog_divider(fsm, datagram);
}

/*****************************************************************************/

/**
@brief WATCHDOG_DIVIDER入口函数。
@param fsm 从站状态机
@param datagram 使用的数据报
@return 无
@details
- 如果配置不为空且watchdog_divider不为0：
  - 打印"设置看门狗分频器为%u."信息，其中%u为watchdog_divider的值。
  - 使用ec_datagram_fpwr函数设置数据报。
  - 写入watchdog_divider的值。
  - 设置状态机的retries为EC_FSM_RETRIES。
  - 设置状态机的状态为ec_fsm_slave_config_state_watchdog_divider。
- 否则：
  - 调用ec_fsm_slave_config_enter_watchdog函数并返回。
*/

void ec_fsm_slave_config_enter_watchdog_divider(
    ec_fsm_slave_config_t *fsm, /**< 从站状态机 */
    ec_datagram_t *datagram     /**< 使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;
    ec_slave_config_t *config = slave->config;

    if (config && config->watchdog_divider)
    {
        EC_SLAVE_DBG(slave, 1, "设置看门狗分频器为%u。\n",
                     config->watchdog_divider);

        ec_datagram_fpwr(datagram, slave->station_address, 0x0400, 2);
        EC_WRITE_U16(datagram->data, config->watchdog_divider);
        fsm->retries = EC_FSM_RETRIES;
        fsm->state = ec_fsm_slave_config_state_watchdog_divider;
    }
    else
    {
        ec_fsm_slave_config_enter_watchdog(fsm, datagram);
    }
}

/*****************************************************************************/

/**
@brief 从站配置状态：WATCHDOG_DIVIDER。
@param fsm 从站状态机
@param datagram 使用的数据报
@return 无
@details
- 如果datagram的state为EC_DATAGRAM_TIMED_OUT且retries不为0：
  - 使用ec_datagram_repeat函数重发数据报。
  - 返回。
- 如果datagram的state不为EC_DATAGRAM_RECEIVED：
  - 设置状态机的状态为ec_fsm_slave_config_state_error。
  - 打印"接收看门狗分频器配置数据报失败："错误消息。
  - 打印datagram的状态。
  - 返回。
- 如果datagram的working_counter不为1：
  - 设置从站的error_flag为1。
  - 打印"设置看门狗分频器失败："警告消息。
  - 打印datagram的working_counter错误。
  - 返回。
- 调用ec_fsm_slave_config_enter_watchdog函数并返回。
*/

void ec_fsm_slave_config_state_watchdog_divider(
    ec_fsm_slave_config_t *fsm, /**< 从站状态机 */
    ec_datagram_t *datagram     /**< 使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;

    if (fsm->datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--)
    {
        ec_datagram_repeat(datagram, fsm->datagram);
        return;
    }

    if (fsm->datagram->state != EC_DATAGRAM_RECEIVED)
    {
        fsm->state = ec_fsm_slave_config_state_error;
        EC_SLAVE_ERR(slave, "接收看门狗分频器配置数据报失败：");
        ec_datagram_print_state(fsm->datagram);
        return;
    }

    if (fsm->datagram->working_counter != 1)
    {
        slave->error_flag = 1;
        EC_SLAVE_WARN(slave, "设置看门狗分频器失败：");
        ec_datagram_print_wc_error(fsm->datagram);
        return;
    }

    ec_fsm_slave_config_enter_watchdog(fsm, datagram);
}

/*****************************************************************************/

/**
@brief WATCHDOG入口函数。
@param fsm 从站状态机
@param datagram 使用的数据报
@return 无
@details
- 如果配置不为空且watchdog_intervals不为0：
  - 打印"设置过程数据看门狗间隔为%u."信息，其中%u为watchdog_intervals的值。
  - 使用ec_datagram_fpwr函数设置数据报。
  - 写入watchdog_intervals的值。
  - 设置状态机的retries为EC_FSM_RETRIES。
  - 设置状态机的状态为ec_fsm_slave_config_state_watchdog。
- 否则：
  - 调用ec_fsm_slave_config_enter_pdo_sync函数并返回。
*/

void ec_fsm_slave_config_enter_watchdog(
    ec_fsm_slave_config_t *fsm, /**< 从站状态机 */
    ec_datagram_t *datagram     /**< 使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;
    ec_slave_config_t *config = slave->config;

    if (config && config->watchdog_intervals)
    {
        EC_SLAVE_DBG(slave, 1, "设置过程数据看门狗间隔为%u。\n",
                     config->watchdog_intervals);

        ec_datagram_fpwr(datagram, slave->station_address, 0x0420, 2);
        EC_WRITE_U16(datagram->data, config->watchdog_intervals);

        fsm->retries = EC_FSM_RETRIES;
        fsm->state = ec_fsm_slave_config_state_watchdog;
    }
    else
    {
        ec_fsm_slave_config_enter_pdo_sync(fsm, datagram);
    }
}

/*****************************************************************************/

/** 从站配置状态：WATCHDOG。
 */

void ec_fsm_slave_config_state_watchdog(
    ec_fsm_slave_config_t *fsm, /**< 从站状态机 */
    ec_datagram_t *datagram     /**< 使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;

    if (fsm->datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--)
    {
        ec_datagram_repeat(datagram, fsm->datagram);
        return;
    }

    if (fsm->datagram->state != EC_DATAGRAM_RECEIVED)
    {
        fsm->state = ec_fsm_slave_config_state_error;
        EC_SLAVE_ERR(slave, "接收同步管理器看门狗配置数据报失败：");
        ec_datagram_print_state(fsm->datagram);
        return;
    }

    if (fsm->datagram->working_counter != 1)
    {
        EC_SLAVE_WARN(slave, "设置过程数据看门狗间隔失败：");
        ec_datagram_print_wc_error(fsm->datagram);
    }

    ec_fsm_slave_config_enter_pdo_sync(fsm, datagram);
}

/*****************************************************************************/

/**
@brief 检查是否需要配置PDO同步管理器。
@param fsm 从站状态机
@param datagram 使用的数据报
@return 无
@details
- 如果从站的SII数据不可用：
  - 设置状态机的状态为ec_fsm_slave_config_state_error。
  - 打印"从站无法配置PDO SyncManager。SII数据不可用。\n"错误消息。
  - 返回。
- 如果从站的SII数据中有邮箱协议：
  - 设置偏移量为2。
- 否则：
  - 设置偏移量为0。
- 如果从站的SII数据中的同步计数减去偏移量小于等于0：
  - 没有需要配置的PDO同步管理器。
  - 调用ec_fsm_slave_config_enter_fmmu函数并返回。
- 计算需要配置的PDO同步管理器的数量。
- 使用ec_datagram_fpwr函数设置数据报，起始地址为0x0800 + EC_SYNC_PAGE_SIZE * 偏移量，长度为EC_SYNC_PAGE_SIZE * 需要配置的PDO同步管理器数量。
- 将数据报清零。
- 遍历需要配置的PDO同步管理器：
  - 获取同步管理器的配置。
  - 设置size为同步管理器中所有PDO的总大小。
  - 如果从站的配置不为空：
    - 获取从站配置中对应的同步管理器配置。
    - 如果存在：
      - 设置pdo_xfer为1。
      - 终止循环。
  - 否则：
    - 设置同步管理器配置为NULL。
    - 设置size为同步管理器的默认长度。
  - 使用ec_sync_page函数配置同步管理器。
- 设置状态机的retries为EC_FSM_RETRIES。
- 设置状态机的状态为ec_fsm_slave_config_state_pdo_sync。
*/

void ec_fsm_slave_config_enter_pdo_sync(
    ec_fsm_slave_config_t *fsm, /**< 从站状态机 */
    ec_datagram_t *datagram     /**< 使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;
    unsigned int i, j, offset, num_pdo_syncs;
    uint8_t sync_index;
    const ec_sync_t *sync;
    uint16_t size;

    if (!slave->sii_image)
    {
        fsm->state = ec_fsm_slave_config_state_error;
        EC_SLAVE_ERR(slave, "从站无法配置PDO SyncManager。SII数据不可用。\n");
        return;
    }

    if (slave->sii_image->sii.mailbox_protocols)
    {
        offset = 2; // 从站有邮箱
    }
    else
    {
        offset = 0;
    }

    if (slave->sii_image->sii.sync_count <= offset)
    {
        // 没有需要配置的PDO同步管理器
        ec_fsm_slave_config_enter_fmmu(fsm, datagram);
        return;
    }

    num_pdo_syncs = slave->sii_image->sii.sync_count - offset;

    // 配置过程数据的同步管理器
    ec_datagram_fpwr(datagram, slave->station_address,
                     0x0800 + EC_SYNC_PAGE_SIZE * offset,
                     EC_SYNC_PAGE_SIZE * num_pdo_syncs);
    ec_datagram_zero(datagram);

    for (i = 0; i < num_pdo_syncs; i++)
    {
        const ec_sync_config_t *sync_config;
        uint8_t pdo_xfer = 0;
        sync_index = i + offset;
        sync = &slave->sii_image->sii.syncs[sync_index];

        if (slave->config)
        {
            const ec_slave_config_t *sc = slave->config;
            sync_config = &sc->sync_configs[sync_index];
            size = ec_pdo_list_total_size(&sync_config->pdos);

            // 确定是否通过此SM传输PDOs
            // 在这种情况下，无论如何都启用同步管理器
            for (j = 0; j < sc->used_fmmus; j++)
            {
                if (sc->fmmu_configs[j].sync_index == sync_index)
                {
                    pdo_xfer = 1;
                    break;
                }
            }
        }
        else
        {
            sync_config = NULL;
            size = sync->default_length;
        }

        ec_sync_page(sync, sync_index, size, sync_config, pdo_xfer,
                     datagram->data + EC_SYNC_PAGE_SIZE * i);
    }

    fsm->retries = EC_FSM_RETRIES;
    fsm->state = ec_fsm_slave_config_state_pdo_sync;
}

/*****************************************************************************/

/**
@brief 配置PDO同步管理器。
@param fsm 从站状态机
@param datagram 使用的数据报
@return 无
@details
- 如果数据报的状态为EC_DATAGRAM_TIMED_OUT且retries不为0：
  - 使用ec_datagram_repeat函数重发数据报。
  - 返回。
- 如果数据报的状态不为EC_DATAGRAM_RECEIVED：
  - 设置状态机的状态为ec_fsm_slave_config_state_error。
  - 打印"接收过程数据同步管理器配置数据报失败："错误消息。
  - 打印数据报的状态。
  - 返回。
- 如果数据报的working_counter不为1：
  - 设置从站的error_flag为1。
  - 设置状态机的状态为ec_fsm_slave_config_state_error。
  - 打印"设置过程数据同步管理器失败："错误消息。
  - 打印数据报的working_counter错误。
  - 返回。
- 调用ec_fsm_slave_config_enter_fmmu函数并返回。
*/

void ec_fsm_slave_config_state_pdo_sync(
    ec_fsm_slave_config_t *fsm, /**< 从站状态机 */
    ec_datagram_t *datagram     /**< 使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;

    if (fsm->datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--)
    {
        ec_datagram_repeat(datagram, fsm->datagram);
        return;
    }

    if (fsm->datagram->state != EC_DATAGRAM_RECEIVED)
    {
        fsm->state = ec_fsm_slave_config_state_error;
        EC_SLAVE_ERR(slave, "接收过程数据同步管理器配置数据报失败：");
        ec_datagram_print_state(fsm->datagram);
        return;
    }

    if (fsm->datagram->working_counter != 1)
    {
        slave->error_flag = 1;
        fsm->state = ec_fsm_slave_config_state_error;
        EC_SLAVE_ERR(slave, "设置过程数据同步管理器失败：");
        ec_datagram_print_wc_error(fsm->datagram);
        return;
    }

    ec_fsm_slave_config_enter_fmmu(fsm, datagram);
}

/*****************************************************************************/

/**
@brief 检查是否需要配置FMMU。
@param fsm 从站状态机
@param datagram 使用的数据报
@return 无
@details
- 如果从站的配置为空：
  - 调用ec_fsm_slave_config_enter_safeop函数并返回。
- 如果从站的基本FMMU数量小于从站配置中使用的FMMU数量：
  - 设置从站的error_flag为1。
  - 设置状态机的状态为ec_fsm_slave_config_state_error。
  - 打印"从站的FMMU数量(%u)小于请求的数量(%u)。\n"错误消息，其中%u分别为从站的基本FMMU数量和从站配置中使用的FMMU数量。
  - 返回。
- 如果从站的基本FMMU数量为0：
  - 跳过FMMU配置。
  - 调用ec_fsm_slave_config_enter_dc_cycle函数并返回。
- 使用ec_datagram_fpwr函数设置数据报，起始地址为0x0600，长度为EC_FMMU_PAGE_SIZE * 从站的基本FMMU数量。
- 将数据报清零。
- 遍历从站配置中使用的FMMU：
  - 获取FMMU的配置。
  - 如果无法确定FMMU对应的PDO同步管理器：
    - 设置从站的error_flag为1。
    - 设置状态机的状态为ec_fsm_slave_config_state_error。
    - 打印"无法确定FMMU的PDO同步管理器！\n"错误消息。
    - 返回。
  - 使用ec_fmmu_config_page函数配置FMMU。
- 设置状态机的retries为EC_FSM_RETRIES。
- 设置状态机的状态为ec_fsm_slave_config_state_fmmu。
*/

void ec_fsm_slave_config_enter_fmmu(
    ec_fsm_slave_config_t *fsm, /**< 从站状态机 */
    ec_datagram_t *datagram     /**< 使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;
    unsigned int i;
    const ec_fmmu_config_t *fmmu;
    const ec_sync_t *sync;

    if (!slave->config)
    {
        ec_fsm_slave_config_enter_safeop(fsm, datagram);
        return;
    }

    if (slave->base_fmmu_count < slave->config->used_fmmus)
    {
        slave->error_flag = 1;
        fsm->state = ec_fsm_slave_config_state_error;
        EC_SLAVE_ERR(slave, "从站的FMMU数量(%u)小于请求的数量(%u)。\n",
                     slave->base_fmmu_count,
                     slave->config->used_fmmus);
        return;
    }

    if (!slave->base_fmmu_count)
    { // 跳过FMMU配置
        ec_fsm_slave_config_enter_dc_cycle(fsm, datagram);
        return;
    }

    // 配置FMMUs
    ec_datagram_fpwr(datagram, slave->station_address,
                     0x0600, EC_FMMU_PAGE_SIZE * slave->base_fmmu_count);
    ec_datagram_zero(datagram);
    for (i = 0; i < slave->config->used_fmmus; i++)
    {
        fmmu = &slave->config->fmmu_configs[i];
        if (!(sync = ec_slave_get_sync(slave, fmmu->sync_index)))
        {
            slave->error_flag = 1;
            fsm->state = ec_fsm_slave_config_state_error;
            EC_SLAVE_ERR(slave, "无法确定FMMU的PDO同步管理器！\n");
            return;
        }
        ec_fmmu_config_page(fmmu, sync,
                            datagram->data + EC_FMMU_PAGE_SIZE * i);
    }

    fsm->retries = EC_FSM_RETRIES;
    fsm->state = ec_fsm_slave_config_state_fmmu;
}

/*****************************************************************************/

/**
@brief 从站配置状态：FMMU。
@param fsm 从站状态机
@param datagram 使用的数据报
@return 无
@details
- 如果数据报的状态为EC_DATAGRAM_TIMED_OUT且retries不为0：
  - 使用ec_datagram_repeat函数重发数据报。
  - 返回。
- 如果数据报的状态不为EC_DATAGRAM_RECEIVED：
  - 设置状态机的状态为ec_fsm_slave_config_state_error。
  - 打印"接收FMMU数据报失败："错误消息。
  - 打印数据报的状态。
  - 返回。
- 如果数据报的working_counter不为1：
  - 设置从站的error_flag为1。
  - 设置状态机的状态为ec_fsm_slave_config_state_error。
  - 打印"设置FMMU失败："错误消息。
  - 打印数据报的working_counter错误。
  - 返回。
- 调用ec_fsm_slave_config_enter_dc_cycle函数并返回。
*/

void ec_fsm_slave_config_state_fmmu(
    ec_fsm_slave_config_t *fsm, /**< 从站状态机 */
    ec_datagram_t *datagram     /**< 使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;

    if (fsm->datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--)
    {
        ec_datagram_repeat(datagram, fsm->datagram);
        return;
    }

    if (fsm->datagram->state != EC_DATAGRAM_RECEIVED)
    {
        fsm->state = ec_fsm_slave_config_state_error;
        EC_SLAVE_ERR(slave, "接收FMMU数据报失败：");
        ec_datagram_print_state(fsm->datagram);
        return;
    }

    if (fsm->datagram->working_counter != 1)
    {
        slave->error_flag = 1;
        fsm->state = ec_fsm_slave_config_state_error;
        EC_SLAVE_ERR(slave, "设置FMMU失败：");
        ec_datagram_print_wc_error(fsm->datagram);
        return;
    }

    ec_fsm_slave_config_enter_dc_cycle(fsm, datagram);
}

/*****************************************************************************/

/**
@brief 检查是否需要配置DC周期。
@param fsm 从站状态机
@param datagram 使用的数据报
@return 无
@details
- 如果从站的配置为空：
  - 调用ec_fsm_slave_config_reconfigure函数并返回。
- 如果从站的配置中的dc_assign_activate为真：
  - 如果从站的base_dc_supported为假或者从站没有dc_system_time：
    - 打印"从站似乎不支持分布式时钟！"警告消息。
  - 打印"设置DC周期时间为%u / %u."信息，其中%u分别为从站配置中dc_sync数组的第一个元素的cycle_time和第二个元素的cycle_time加上第二个元素的shift_time。
  - 使用ec_datagram_fpwr函数设置数据报，起始地址为0x09A0，长度为8。
  - 写入第一个元素的cycle_time和第二个元素的cycle_time加上第二个元素的shift_time。
  - 设置状态机的retries为EC_FSM_RETRIES。
  - 设置状态机的状态为ec_fsm_slave_config_state_dc_cycle。
- 否则：
  - 调用ec_fsm_slave_config_enter_safeop函数并返回。
*/

void ec_fsm_slave_config_enter_dc_cycle(
    ec_fsm_slave_config_t *fsm, /**< 从站状态机 */
    ec_datagram_t *datagram     /**< 使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;
    ec_slave_config_t *config = slave->config;

    if (!config)
    { // 配置在此期间被删除
        ec_fsm_slave_config_reconfigure(fsm, datagram);
        return;
    }

    if (config->dc_assign_activate)
    {
        if (!slave->base_dc_supported || !slave->has_dc_system_time)
        {
            EC_SLAVE_WARN(slave, "从站似乎不支持分布式时钟！\n");
        }

        EC_SLAVE_DBG(slave, 1, "设置DC周期时间为%u / %u。\n",
                     config->dc_sync[0].cycle_time, config->dc_sync[1].cycle_time);

        // 设置DC周期时间
        ec_datagram_fpwr(datagram, slave->station_address, 0x09A0, 8);
        EC_WRITE_U32(datagram->data, config->dc_sync[0].cycle_time);
        EC_WRITE_U32(datagram->data + 4, config->dc_sync[1].cycle_time +
                                             config->dc_sync[1].shift_time);
        fsm->retries = EC_FSM_RETRIES;
        fsm->state = ec_fsm_slave_config_state_dc_cycle;
    }
    else
    {
        // DC未使用
        ec_fsm_slave_config_enter_safeop(fsm, datagram);
    }
}

/*****************************************************************************/

/**
@brief 从站配置状态：DC周期。
@param fsm 从站状态机
@param datagram 使用的数据报
@return 无
@details
- 如果从站的配置为空：
  - 调用ec_fsm_slave_config_reconfigure函数并返回。
- 如果数据报的状态为EC_DATAGRAM_TIMED_OUT且retries不为0：
  - 使用ec_datagram_repeat函数重发数据报。
  - 返回。
- 如果数据报的状态不为EC_DATAGRAM_RECEIVED：
  - 设置状态机的状态为ec_fsm_slave_config_state_error。
  - 打印"接收DC周期时间数据报失败："错误消息。
  - 打印数据报的状态。
  - 返回。
- 如果数据报的working_counter不为1：
  - 设置从站的error_flag为1。
  - 设置状态机的状态为ec_fsm_slave_config_state_error。
  - 打印"设置DC周期时间失败："错误消息。
  - 打印数据报的working_counter错误。
  - 返回。
- 打印"检查同步性。\n"信息。
- 将last_diff_ms设置为0。
- 将jiffies_start设置为jiffies。
- 使用ec_datagram_fprd函数读取数据报，起始地址为slave->station_address，长度为4。
- 将数据报清零。
- 设置状态机的retries为EC_FSM_RETRIES。
- 设置状态机的状态为ec_fsm_slave_config_state_dc_sync_check。
*/

void ec_fsm_slave_config_state_dc_cycle(
    ec_fsm_slave_config_t *fsm, /**< 从站状态机 */
    ec_datagram_t *datagram     /**< 使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;
    ec_slave_config_t *config = slave->config;

    if (!config)
    { // 配置在此期间被删除
        ec_fsm_slave_config_reconfigure(fsm, datagram);
        return;
    }

    if (fsm->datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--)
    {
        ec_datagram_repeat(datagram, fsm->datagram);
        return;
    }

    if (fsm->datagram->state != EC_DATAGRAM_RECEIVED)
    {
        fsm->state = ec_fsm_slave_config_state_error;
        EC_SLAVE_ERR(slave, "接收DC周期时间数据报失败：");
        ec_datagram_print_state(fsm->datagram);
        return;
    }

    if (fsm->datagram->working_counter != 1)
    {
        slave->error_flag = 1;
        fsm->state = ec_fsm_slave_config_state_error;
        EC_SLAVE_ERR(slave, "设置DC周期时间失败：");
        ec_datagram_print_wc_error(fsm->datagram);
        return;
    }

    EC_SLAVE_DBG(slave, 1, "检查同步性。\n");

    fsm->last_diff_ms = 0;
    fsm->jiffies_start = jiffies;
    ec_datagram_fprd(datagram, slave->station_address, 0x092c, 4);
    ec_datagram_zero(datagram);
    fsm->retries = EC_FSM_RETRIES;
    fsm->state = ec_fsm_slave_config_state_dc_sync_check;
}

/*****************************************************************************/

/**
@brief 从站配置状态：DC同步检查。
@param fsm 从站状态机
@param datagram 使用的数据报
@return 无
@details
- 如果从站的配置为空：
  - 调用ec_fsm_slave_config_reconfigure函数并返回。
- 如果数据报的状态为EC_DATAGRAM_TIMED_OUT且retries不为0：
  - 使用ec_datagram_repeat函数重发数据报。
  - 返回。
- 如果数据报的状态不为EC_DATAGRAM_RECEIVED：
  - 设置状态机的状态为ec_fsm_slave_config_state_error。
  - 打印"接收DC同步检查数据报失败："错误消息。
  - 打印数据报的状态。
  - 返回。
- 如果数据报的working_counter不为1：
  - 设置从站的error_flag为1。
  - 设置状态机的状态为ec_fsm_slave_config_state_error。
  - 打印"检查DC同步性失败："错误消息。
  - 打印数据报的working_counter错误。
  - 返回。
- 将abs_sync_diff设置为数据报中的前31位。
- 将negative设置为数据报中的最高位是否为1。
- 将diff_ms设置为(fsm->datagram->jiffies_received - fsm->jiffies_start) * 1000 / HZ。
- 如果abs_sync_diff大于EC_DC_MAX_SYNC_DIFF_NS：
  - 如果diff_ms大于等于EC_DC_SYNC_WAIT_MS：
    - 打印"从站在%lu ms后仍未同步。\n"警告消息，其中%lu为diff_ms的值。
  - 否则：
    - 如果(diff_ms < fsm->last_diff_ms)或者(diff_ms >= (fsm->last_diff_ms + 100))：
      - 更新fsm->last_diff_ms为diff_ms。
      - 打印"同步后%4lu ms：%10d ns\n"信息，其中%lu为diff_ms的值，%d为abs_sync_diff的值，如果negative为真则取-abs_sync_diff。
    - 使用ec_datagram_fprd函数读取数据报，起始地址为slave->station_address，长度为4。
    - 将数据报清零。
    - 设置状态机的retries为EC_FSM_RETRIES。
    - 返回。
- 否则：
  - 打印"%d ns差异，经过%lu ms。\n"信息，其中%d为abs_sync_diff的值，%lu为diff_ms的值。
- 将start_time设置为master->app_time + EC_DC_START_OFFSET。
- 如果sync0->cycle_time非零：
  - 如果master->dc_ref_time非零：
    - 计算diff为start_time - master->dc_ref_time。
    - 计算cycle为sync0->cycle_time + sync1->cycle_time。
    - 计算remainder为diff除以cycle的余数。
    - 计算start为start_time + cycle - remainder + sync0->shift_time。
    - 打印"   ref_time=%llu\n"信息，其中%llu为master->dc_ref_time的值。
    - 打印"   app_time=%llu\n"信息，其中%llu为master->app_time的值。
    - 打印" start_time=%llu\n"信息，其中%llu为start_time的值。
    - 打印"      cycle=%u\n"信息，其中%u为cycle的值。
    - 打印" shift_time=%i\n"信息，其中%i为sync0->shift_time的值。
    - 打印"  remainder=%u\n"信息，其中%u为remainder的值。
    - 打印"       start=%llu\n"信息，其中%llu为start的值。
    - 将start_time设置为start。
  - 否则：
    - 打印"未提供应用时间。循环开始时间将不同步。\n"警告消息。
- 打印"设置DC循环操作开始时间为%llu。\n"信息，其中%llu为start_time的值。
- 使用ec_datagram_fpwr函数设置数据报，起始地址为slave->station_address，长度为8。
- 写入start_time。
- 设置状态机的retries为EC_FSM_RETRIES。
- 设置状态机的状态为ec_fsm_slave_config_state_dc_start。
*/

void ec_fsm_slave_config_state_dc_sync_check(
    ec_fsm_slave_config_t *fsm, /**< 从站状态机 */
    ec_datagram_t *datagram     /**< 使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;
    ec_master_t *master = slave->master;
    ec_slave_config_t *config = slave->config;
    bool negative;
    uint32_t abs_sync_diff;
    unsigned long diff_ms;
    ec_sync_signal_t *sync0 = &config->dc_sync[0];
    ec_sync_signal_t *sync1 = &config->dc_sync[1];
    u64 start_time;

    if (!config)
    { // 配置在此期间被删除
        ec_fsm_slave_config_reconfigure(fsm, datagram);
        return;
    }

    if (fsm->datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--)
    {
        ec_datagram_repeat(datagram, fsm->datagram);
        return;
    }

    if (fsm->datagram->state != EC_DATAGRAM_RECEIVED)
    {
        fsm->state = ec_fsm_slave_config_state_error;
        EC_SLAVE_ERR(slave, "接收DC同步检查数据报失败：");
        ec_datagram_print_state(fsm->datagram);
        return;
    }

    if (fsm->datagram->working_counter != 1)
    {
        slave->error_flag = 1;
        fsm->state = ec_fsm_slave_config_state_error;
        EC_SLAVE_ERR(slave, "检查DC同步性失败：");
        ec_datagram_print_wc_error(fsm->datagram);
        return;
    }

    abs_sync_diff = EC_READ_U32(fsm->datagram->data) & 0x7fffffff;
    negative = (EC_READ_U32(fsm->datagram->data) & 0x80000000) != 0;
    diff_ms = (fsm->datagram->jiffies_received - fsm->jiffies_start) * 1000 / HZ;

    if (abs_sync_diff > EC_DC_MAX_SYNC_DIFF_NS)
    {

        if (diff_ms >= EC_DC_SYNC_WAIT_MS)
        {
            EC_SLAVE_WARN(slave, "从站在%lu ms后仍未同步。\n",
                          diff_ms);
        }
        else
        {
            if ((diff_ms < fsm->last_diff_ms) || (diff_ms >= (fsm->last_diff_ms + 100)))
            {
                fsm->last_diff_ms = diff_ms;
                EC_SLAVE_DBG(slave, 1, "同步后%4lu ms：%10d ns\n",
                             diff_ms, negative ? -abs_sync_diff : abs_sync_diff);
            }

            // 再次检查同步性
            ec_datagram_fprd(datagram, slave->station_address, 0x092c, 4);
            ec_datagram_zero(datagram);
            fsm->retries = EC_FSM_RETRIES;
            return;
        }
    }
    else
    {
        EC_SLAVE_DBG(slave, 1, "%d ns差异，经过%lu ms。\n",
                     negative ? -abs_sync_diff : abs_sync_diff, diff_ms);
    }

    // 设置DC开始时间（大致在未来，不同步）
    start_time = master->app_time + EC_DC_START_OFFSET; // 现在 + X ns

    if (sync0->cycle_time)
    {
        // 找到正确的相位
        if (master->dc_ref_time)
        {
            u64 diff, start;
            u32 remainder, cycle;

            diff = start_time - master->dc_ref_time;
            cycle = sync0->cycle_time + sync1->cycle_time;
            remainder = do_div(diff, cycle);

            start = start_time + cycle - remainder + sync0->shift_time;

            EC_SLAVE_DBG(slave, 1, "   ref_time=%llu\n", master->dc_ref_time);
            EC_SLAVE_DBG(slave, 1, "   app_time=%llu\n", master->app_time);
            EC_SLAVE_DBG(slave, 1, " start_time=%llu\n", start_time);
            EC_SLAVE_DBG(slave, 1, "      cycle=%u\n", cycle);
            EC_SLAVE_DBG(slave, 1, " shift_time=%i\n", sync0->shift_time);
            EC_SLAVE_DBG(slave, 1, "  remainder=%u\n", remainder);
            EC_SLAVE_DBG(slave, 1, "       start=%llu\n", start);
            start_time = start;
        }
        else
        {
            EC_SLAVE_WARN(slave, "未提供应用时间。循环开始时间将不同步。\n");
        }
    }

    EC_SLAVE_DBG(slave, 1, "设置DC循环操作开始时间为%llu。\n",
                 start_time);

    ec_datagram_fpwr(datagram, slave->station_address, 0x0990, 8);
    EC_WRITE_U64(datagram->data, start_time);
    fsm->retries = EC_FSM_RETRIES;
    fsm->state = ec_fsm_slave_config_state_dc_start;
}

/*****************************************************************************/

/**
@brief 从站配置状态：DC开始。
@param fsm 从站状态机
@param datagram 使用的数据报
@return 无
@details
- 如果从站的配置为空：
  - 调用ec_fsm_slave_config_reconfigure函数并返回。
- 如果数据报的状态为EC_DATAGRAM_TIMED_OUT且retries不为0：
  - 使用ec_datagram_repeat函数重发数据报。
  - 返回。
- 如果数据报的状态不为EC_DATAGRAM_RECEIVED：
  - 设置状态机的状态为ec_fsm_slave_config_state_error。
  - 打印"接收DC开始时间数据报失败："错误消息。
  - 打印数据报的状态。
  - 返回。
- 如果数据报的working_counter不为1：
  - 设置从站的error_flag为1。
  - 设置状态机的状态为ec_fsm_slave_config_state_error。
  - 打印"设置DC开始时间失败："错误消息。
  - 打印数据报的working_counter错误。
  - 返回。
- 打印"将DC AssignActivate设置为0x%04x。\n"信息，其中0x%04x为从站配置中的dc_assign_activate。
- 使用ec_datagram_fpwr函数设置数据报，起始地址为slave->station_address，长度为2。
- 写入config->dc_assign_activate。
- 设置状态机的retries为EC_FSM_RETRIES。
- 设置状态机的状态为ec_fsm_slave_config_state_dc_assign。
*/

void ec_fsm_slave_config_state_dc_start(
    ec_fsm_slave_config_t *fsm, /**< 从站状态机 */
    ec_datagram_t *datagram     /**< 使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;
    ec_slave_config_t *config = slave->config;

    if (!config)
    { // 配置在此期间被删除
        ec_fsm_slave_config_reconfigure(fsm, datagram);
        return;
    }

    if (fsm->datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--)
    {
        ec_datagram_repeat(datagram, fsm->datagram);
        return;
    }

    if (fsm->datagram->state != EC_DATAGRAM_RECEIVED)
    {
        fsm->state = ec_fsm_slave_config_state_error;
        EC_SLAVE_ERR(slave, "接收DC开始时间数据报失败：");
        ec_datagram_print_state(fsm->datagram);
        return;
    }

    if (fsm->datagram->working_counter != 1)
    {
        slave->error_flag = 1;
        fsm->state = ec_fsm_slave_config_state_error;
        EC_SLAVE_ERR(slave, "设置DC开始时间失败：");
        ec_datagram_print_wc_error(fsm->datagram);
        return;
    }

    EC_SLAVE_DBG(slave, 1, "将DC AssignActivate设置为0x%04x。\n",
                 config->dc_assign_activate);

    // 将同步单元分配给EtherCAT或PDI
    ec_datagram_fpwr(datagram, slave->station_address, 0x0980, 2);
    EC_WRITE_U16(datagram->data, config->dc_assign_activate);
    fsm->retries = EC_FSM_RETRIES;
    fsm->state = ec_fsm_slave_config_state_dc_assign;
}

/*****************************************************************************/

/**
@brief 从站配置状态：DC分配。
@param fsm 从站状态机
@param datagram 使用的数据报
@return 无
@details
- 如果数据报的状态为EC_DATAGRAM_TIMED_OUT且retries不为0：
  - 使用ec_datagram_repeat函数重发数据报。
  - 返回。
- 如果数据报的状态不为EC_DATAGRAM_RECEIVED：
  - 设置状态机的状态为ec_fsm_slave_config_state_error。
  - 打印"接收DC激活数据报失败："错误消息。
  - 打印数据报的状态。
  - 返回。
- 如果数据报的working_counter不为1：
  - 设置从站的error_flag为1。
  - 设置状态机的状态为ec_fsm_slave_config_state_error。
  - 打印"激活DC失败："错误消息。
  - 打印数据报的working_counter错误。
  - 返回。
- 调用ec_fsm_slave_config_enter_safeop函数并返回。
*/

void ec_fsm_slave_config_state_dc_assign(
    ec_fsm_slave_config_t *fsm, /**< 从站状态机 */
    ec_datagram_t *datagram     /**< 使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;

    if (fsm->datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--)
    {
        ec_datagram_repeat(datagram, fsm->datagram);
        return;
    }

    if (fsm->datagram->state != EC_DATAGRAM_RECEIVED)
    {
        fsm->state = ec_fsm_slave_config_state_error;
        EC_SLAVE_ERR(slave, "接收DC激活数据报失败：");
        ec_datagram_print_state(fsm->datagram);
        return;
    }

    if (fsm->datagram->working_counter != 1)
    {
        slave->error_flag = 1;
        fsm->state = ec_fsm_slave_config_state_error;
        EC_SLAVE_ERR(slave, "激活DC失败：");
        ec_datagram_print_wc_error(fsm->datagram);
        return;
    }

    ec_fsm_slave_config_enter_safeop(fsm, datagram);
}

/*****************************************************************************/

/**
@brief 请求SAFEOP状态。
@param fsm 从站状态机
@param datagram 使用的数据报
@return 无
@details
- 设置状态机的状态为ec_fsm_slave_config_state_safeop。
- 调用ec_fsm_change_start函数，将从站的状态设置为EC_SLAVE_STATE_SAFEOP。
- 调用ec_fsm_change_exec函数，立即执行状态变化。
*/

void ec_fsm_slave_config_enter_safeop(
    ec_fsm_slave_config_t *fsm, /**< 从站状态机 */
    ec_datagram_t *datagram     /**< 使用的数据报 */
)
{
    fsm->state = ec_fsm_slave_config_state_safeop;
    ec_fsm_change_start(fsm->fsm_change, fsm->slave, EC_SLAVE_STATE_SAFEOP);
    ec_fsm_change_exec(fsm->fsm_change, datagram); // 立即执行
}

/*****************************************************************************/

/**
@brief 从站配置状态：SAFEOP。
@param fsm 从站状态机
@param datagram 使用的数据报
@return 无
@details
- 如果ec_fsm_change_exec函数返回true，则返回。
- 如果ec_fsm_change_success函数返回false：
  - 如果fsm->fsm_change->spontaneous_change为假，则设置从站的error_flag为1。
  - 设置状态机的状态为ec_fsm_slave_config_state_error。
  - 返回。
- 从站现在处于SAFEOP状态。
- 打印"现在处于SAFEOP。\n"信息。
- 如果从站的current_state等于从站的requested_state：
  - 设置状态机的状态为ec_fsm_slave_config_state_end，表示配置成功。
  - 打印"配置完成。\n"信息。
  - 返回。
- 调用ec_fsm_slave_config_enter_soe_conf_safeop函数并返回。
*/

void ec_fsm_slave_config_state_safeop(
    ec_fsm_slave_config_t *fsm, /**< 从站状态机 */
    ec_datagram_t *datagram     /**< 使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;

    if (ec_fsm_change_exec(fsm->fsm_change, datagram))
        return;

    if (!ec_fsm_change_success(fsm->fsm_change))
    {
        if (!fsm->fsm_change->spontaneous_change)
            fsm->slave->error_flag = 1;
        fsm->state = ec_fsm_slave_config_state_error;
        return;
    }

    // 从站现在处于SAFEOP状态

    EC_SLAVE_DBG(slave, 1, "现在处于SAFEOP。\n");

    if (fsm->slave->current_state == fsm->slave->requested_state)
    {
        fsm->state = ec_fsm_slave_config_state_end; // 配置成功
        EC_SLAVE_DBG(slave, 1, "配置完成。\n");
        return;
    }

    ec_fsm_slave_config_enter_soe_conf_safeop(fsm, datagram);
}

/*****************************************************************************/

/**
@brief 检查是否需要在SAFEOP状态下应用SoE配置。
@param fsm 从站状态机
@param datagram 使用的数据报
@return 无
@details
- 如果从站的配置为空：
  - 调用ec_fsm_slave_config_enter_op函数并返回。
- 遍历从站配置中的soe_configs链表：
  - 如果req->al_state等于EC_AL_STATE_SAFEOP：
    - 设置状态机的状态为ec_fsm_slave_config_state_soe_conf_safeop，表示开始SoE配置。
    - 将fsm->soe_request设置为req。
    - 复制fsm->soe_request到fsm->soe_request_copy。
    - 写入fsm->soe_request_copy。
    - 调用ec_fsm_soe_transfer函数，传输SoE请求。
    - 调用ec_fsm_soe_exec函数，立即执行。
    - 返回。
- 没有需要在SAFEOP状态下应用的SoE配置。
- 调用ec_fsm_slave_config_enter_op函数并返回。
*/

void ec_fsm_slave_config_enter_soe_conf_safeop(
    ec_fsm_slave_config_t *fsm, /**< 从站状态机 */
    ec_datagram_t *datagram     /**< 使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;
    ec_soe_request_t *req;

    if (!slave->config)
    {
        ec_fsm_slave_config_enter_op(fsm, datagram);
        return;
    }

    list_for_each_entry(req, &slave->config->soe_configs, list)
    {
        if (req->al_state == EC_AL_STATE_SAFEOP)
        {
            // 开始SoE配置
            fsm->state = ec_fsm_slave_config_state_soe_conf_safeop;
            fsm->soe_request = req;
            ec_soe_request_copy(&fsm->soe_request_copy, fsm->soe_request);
            ec_soe_request_write(&fsm->soe_request_copy);
            ec_fsm_soe_transfer(fsm->fsm_soe, fsm->slave,
                                &fsm->soe_request_copy);
            ec_fsm_soe_exec(fsm->fsm_soe, datagram);
            return;
        }
    }

    // 在SAFEOP状态下没有需要应用的SoE配置
    ec_fsm_slave_config_enter_op(fsm, datagram);
}

/*****************************************************************************/

/**
@brief 从站配置状态：SOE_CONF。
@param fsm 从站状态机
@param datagram 使用的数据报
@return 无
@details
- 如果ec_fsm_soe_exec函数返回true，则返回。
- 如果ec_fsm_soe_success函数返回false：
  - 打印"SoE配置失败。\n"错误消息。
  - 设置从站的error_flag为1。
  - 设置状态机的状态为ec_fsm_slave_config_state_error。
  - 返回。
- 如果从站的配置为空：
  - 调用ec_fsm_slave_config_reconfigure函数并返回。
- 当fsm->soe_request->list.next不等于&fsm->slave->config->soe_configs时：
  - 将fsm->soe_request设置为fsm->soe_request->list.next。
  - 如果fsm->soe_request->al_state等于EC_AL_STATE_SAFEOP：
    - 复制fsm->soe_request到fsm->soe_request_copy。
    - 写入fsm->soe_request_copy。
    - 调用ec_fsm_soe_transfer函数，传输SoE请求。
    - 调用ec_fsm_soe_exec函数，立即执行。
    - 返回。
- 所有SAFEOP IDN都已配置完成。
- 调用ec_fsm_slave_config_enter_op函数并返回。
*/

void ec_fsm_slave_config_state_soe_conf_safeop(
    ec_fsm_slave_config_t *fsm, /**< 从站状态机 */
    ec_datagram_t *datagram     /**< 使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;

    if (ec_fsm_soe_exec(fsm->fsm_soe, datagram))
    {
        return;
    }

    if (!ec_fsm_soe_success(fsm->fsm_soe))
    {
        EC_SLAVE_ERR(slave, "SoE配置失败。\n");
        fsm->slave->error_flag = 1;
        fsm->state = ec_fsm_slave_config_state_error;
        return;
    }

    if (!fsm->slave->config)
    { // 配置在此期间被删除
        ec_fsm_slave_config_reconfigure(fsm, datagram);
        return;
    }

    // 还有其他IDN需要在SAFEOP状态下配置吗？
    while (fsm->soe_request->list.next != &fsm->slave->config->soe_configs)
    {
        fsm->soe_request = list_entry(fsm->soe_request->list.next,
                                      ec_soe_request_t, list);
        if (fsm->soe_request->al_state == EC_AL_STATE_SAFEOP)
        {
            ec_soe_request_copy(&fsm->soe_request_copy, fsm->soe_request);
            ec_soe_request_write(&fsm->soe_request_copy);
            ec_fsm_soe_transfer(fsm->fsm_soe, fsm->slave,
                                &fsm->soe_request_copy);
            ec_fsm_soe_exec(fsm->fsm_soe, datagram);
            return;
        }
    }

    // 所有SAFEOP IDN都已配置完成
    ec_fsm_slave_config_enter_op(fsm, datagram);
}

/*****************************************************************************/

/**
@brief 将从站配置为OP状态。
@param fsm 从站状态机
@param datagram 使用的数据报
@return 无
@details
- 设置状态机的状态为ec_fsm_slave_config_state_op。
- 调用ec_fsm_change_start函数，将从站的状态设置为EC_SLAVE_STATE_OP。
- 调用ec_fsm_change_exec函数，立即执行状态变化。
*/

void ec_fsm_slave_config_enter_op(
    ec_fsm_slave_config_t *fsm, /**< 从站状态机 */
    ec_datagram_t *datagram     /**< 使用的数据报 */
)
{
    // 将状态设置为OP
    fsm->state = ec_fsm_slave_config_state_op;
    ec_fsm_change_start(fsm->fsm_change, fsm->slave, EC_SLAVE_STATE_OP);
    ec_fsm_change_exec(fsm->fsm_change, datagram); // 立即执行
}

/*****************************************************************************/

/**
@brief 从站配置状态：OP。
@param fsm 从站状态机
@param datagram 使用的数据报
@return 无
@details
- 如果ec_fsm_change_exec函数返回true，则返回。
- 如果ec_fsm_change_success函数返回false：
  - 如果fsm->fsm_change->spontaneous_change为假，则设置从站的error_flag为1。
  - 设置状态机的状态为ec_fsm_slave_config_state_error。
  - 返回。
- 从站现在处于OP状态。
- 打印"现在处于OP。配置完成。\n"信息。
- 设置状态机的状态为ec_fsm_slave_config_state_end，表示配置成功。
*/

void ec_fsm_slave_config_state_op(
    ec_fsm_slave_config_t *fsm, /**< 从站状态机 */
    ec_datagram_t *datagram     /**< 使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;

    if (ec_fsm_change_exec(fsm->fsm_change, datagram))
        return;

    if (!ec_fsm_change_success(fsm->fsm_change))
    {
        if (!fsm->fsm_change->spontaneous_change)
            slave->error_flag = 1;
        fsm->state = ec_fsm_slave_config_state_error;
        return;
    }

    // 从站现在处于OP状态

    EC_SLAVE_DBG(slave, 1, "现在处于OP。配置完成。\n");

    fsm->state = ec_fsm_slave_config_state_end; // 配置成功
}

/*****************************************************************************/

/**
@brief 重新配置从INIT开始的从站。
@param fsm 从站状态机
@param datagram 使用的数据报
@return 无
@details
- 打印"在配置过程中，从站配置被分离。重新配置中。\n"信息。
- 调用ec_fsm_slave_config_enter_init函数，重新配置。
*/

void ec_fsm_slave_config_reconfigure(
    ec_fsm_slave_config_t *fsm, /**< 从站状态机 */
    ec_datagram_t *datagram     /**< 使用的数据报 */
)
{
    EC_SLAVE_DBG(fsm->slave, 1, "在配置过程中，从站配置被分离。重新配置中。\n");

    ec_fsm_slave_config_enter_init(fsm, datagram); // 重新配置
}

/*****************************************************************************/

/**
@brief 错误状态。
@param fsm 从站状态机
@param datagram 使用的数据报
@return 无
*/

void ec_fsm_slave_config_state_error(
    ec_fsm_slave_config_t *fsm, /**< 从站状态机 */
    ec_datagram_t *datagram     /**< 使用的数据报 */
)
{
}

/*****************************************************************************/

/**
@brief 结束状态。
@param fsm 从站状态机
@param datagram 使用的数据报
@return 无
*/

void ec_fsm_slave_config_state_end(
    ec_fsm_slave_config_t *fsm, /**< 从站状态机 */
    ec_datagram_t *datagram     /**< 使用的数据报 */
)
{
}

/*****************************************************************************/

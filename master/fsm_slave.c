/******************************************************************************
 *
 *  $Id$
 *
 *  Copyright (C) 2006-2012  Florian Pose, Ingenieurgemeinschaft IgH
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
 * EtherCAT slave (SDO) state machine.
 */

/*****************************************************************************/

#include "globals.h"
#include "master.h"
#include "mailbox.h"
#include "slave_config.h"

#include "fsm_slave.h"

/*****************************************************************************/

void ec_fsm_slave_state_idle(ec_fsm_slave_t *, ec_datagram_t *);
void ec_fsm_slave_state_ready(ec_fsm_slave_t *, ec_datagram_t *);
int ec_fsm_slave_action_scan(ec_fsm_slave_t *, ec_datagram_t *);
void ec_fsm_slave_state_scan(ec_fsm_slave_t *, ec_datagram_t *);
int ec_fsm_slave_action_config(ec_fsm_slave_t *, ec_datagram_t *);
void ec_fsm_slave_state_acknowledge(ec_fsm_slave_t *, ec_datagram_t *);
void ec_fsm_slave_state_config(ec_fsm_slave_t *, ec_datagram_t *);
int ec_fsm_slave_action_process_dict(ec_fsm_slave_t *, ec_datagram_t *);
void ec_fsm_slave_state_dict_request(ec_fsm_slave_t *, ec_datagram_t *);
int ec_fsm_slave_action_process_config_sdo(ec_fsm_slave_t *, ec_datagram_t *);
int ec_fsm_slave_action_process_sdo(ec_fsm_slave_t *, ec_datagram_t *);
void ec_fsm_slave_state_sdo_request(ec_fsm_slave_t *, ec_datagram_t *);
int ec_fsm_slave_action_process_reg(ec_fsm_slave_t *, ec_datagram_t *);
void ec_fsm_slave_state_reg_request(ec_fsm_slave_t *, ec_datagram_t *);
int ec_fsm_slave_action_process_foe(ec_fsm_slave_t *, ec_datagram_t *);
void ec_fsm_slave_state_foe_request(ec_fsm_slave_t *, ec_datagram_t *);
int ec_fsm_slave_action_process_soe(ec_fsm_slave_t *, ec_datagram_t *);
void ec_fsm_slave_state_soe_request(ec_fsm_slave_t *, ec_datagram_t *);
#ifdef EC_EOE
int ec_fsm_slave_action_process_eoe(ec_fsm_slave_t *, ec_datagram_t *);
void ec_fsm_slave_state_eoe_request(ec_fsm_slave_t *, ec_datagram_t *);
#endif
int ec_fsm_slave_action_process_mbg(ec_fsm_slave_t *, ec_datagram_t *);
void ec_fsm_slave_state_mbg_request(ec_fsm_slave_t *, ec_datagram_t *);

/*****************************************************************************/

/**
 * @brief 构造函数。
 *
 * @param fsm   从站状态机
 * @param slave EtherCAT 从站
 *
 * @details 此函数用于初始化从站状态机。它将从站状态机的成员变量进行初始化，并将从站设置为指定的从站实例。
 *          在使用从站状态机之前，必须先调用此函数进行初始化。
 */
void ec_fsm_slave_init(
    ec_fsm_slave_t *fsm, /**< 从站状态机 */
    ec_slave_t *slave    /**< EtherCAT 从站 */
)
{
    fsm->slave = slave;
    INIT_LIST_HEAD(&fsm->list); // 标记为未列出

    fsm->state = ec_fsm_slave_state_idle;
    fsm->datagram = NULL;
    fsm->sdo_request = NULL;
    fsm->reg_request = NULL;
    fsm->foe_request = NULL;
    fsm->soe_request = NULL;
#ifdef EC_EOE
    fsm->eoe_request = NULL;
#endif
    fsm->mbg_request = NULL;
    fsm->dict_request = NULL;

    ec_dict_request_init(&fsm->int_dict_request);

    // 初始化子状态机
    ec_fsm_coe_init(&fsm->fsm_coe);
    ec_fsm_foe_init(&fsm->fsm_foe);
    ec_fsm_soe_init(&fsm->fsm_soe);
#ifdef EC_EOE
    ec_fsm_eoe_init(&fsm->fsm_eoe);
#endif
    ec_fsm_mbg_init(&fsm->fsm_mbg);
    ec_fsm_pdo_init(&fsm->fsm_pdo, &fsm->fsm_coe);
    ec_fsm_change_init(&fsm->fsm_change);
    ec_fsm_slave_config_init(&fsm->fsm_slave_config, fsm->slave,
                             &fsm->fsm_change, &fsm->fsm_coe, &fsm->fsm_soe, &fsm->fsm_pdo);
    ec_fsm_slave_scan_init(&fsm->fsm_slave_scan, fsm->slave,
                           &fsm->fsm_slave_config, &fsm->fsm_pdo);
}

/*****************************************************************************/

/**
 * @brief 析构函数。
 *
 * @param fsm 从站状态机
 *
 * @details 此函数用于清除从站状态机。它会释放从站状态机占用的资源，并将从站状态设置为停止状态。
 *          在不再使用从站状态机时，应调用此函数进行清理。
 */
void ec_fsm_slave_clear(
    ec_fsm_slave_t *fsm /**< 从站状态机 */
)
{
    if (fsm->state != ec_fsm_slave_state_idle)
    {
        EC_SLAVE_DBG(fsm->slave, 1, "无法处理请求。\n");
    }

    // 处理当前正在进行的请求

    if (fsm->sdo_request)
    {
        fsm->sdo_request->state = EC_INT_REQUEST_FAILURE;
        wake_up_all(&fsm->slave->master->request_queue);
    }

    if (fsm->reg_request)
    {
        fsm->reg_request->state = EC_INT_REQUEST_FAILURE;
        wake_up_all(&fsm->slave->master->request_queue);
    }

    if (fsm->foe_request)
    {
        fsm->foe_request->state = EC_INT_REQUEST_FAILURE;
        wake_up_all(&fsm->slave->master->request_queue);
    }

    if (fsm->soe_request)
    {
        fsm->soe_request->state = EC_INT_REQUEST_FAILURE;
        wake_up_all(&fsm->slave->master->request_queue);
    }

#ifdef EC_EOE
    if (fsm->eoe_request)
    {
        fsm->eoe_request->state = EC_INT_REQUEST_FAILURE;
        wake_up_all(&fsm->slave->master->request_queue);
    }
#endif

    if (fsm->mbg_request)
    {
        fsm->mbg_request->state = EC_INT_REQUEST_FAILURE;
        wake_up_all(&fsm->slave->master->request_queue);
    }

    if (fsm->dict_request)
    {
        fsm->dict_request->state = EC_INT_REQUEST_FAILURE;
        wake_up_all(&fsm->slave->master->request_queue);
    }

    // 清除子状态机
    ec_fsm_slave_scan_clear(&fsm->fsm_slave_scan);
    ec_fsm_slave_config_clear(&fsm->fsm_slave_config);
    ec_fsm_change_clear(&fsm->fsm_change);
    ec_fsm_pdo_clear(&fsm->fsm_pdo);
    ec_fsm_coe_clear(&fsm->fsm_coe);
    ec_fsm_foe_clear(&fsm->fsm_foe);
    ec_fsm_soe_clear(&fsm->fsm_soe);
#ifdef EC_EOE
    ec_fsm_eoe_clear(&fsm->fsm_eoe);
#endif
    ec_fsm_mbg_clear(&fsm->fsm_mbg);
}

/*****************************************************************************/

/**
 * @brief 执行当前状态机的当前状态。
 *
 * @param fsm       从站状态机
 * @param datagram  要使用的新数据报
 * @return          如果使用了数据报则返回1，否则返回0
 *
 * @details 此函数用于执行从站状态机的当前状态。它会根据当前状态的不同执行相应的操作。
 *          如果执行过程中使用了数据报，则返回1，否则返回0。
 */
int ec_fsm_slave_exec(
    ec_fsm_slave_t *fsm,    /**< 从站状态机 */
    ec_datagram_t *datagram /**< 要使用的新数据报 */
)
{
    int datagram_used;

    fsm->state(fsm, datagram);

    datagram_used = fsm->state != ec_fsm_slave_state_idle &&
                    fsm->state != ec_fsm_slave_state_ready;

    if (datagram_used)
    {
        datagram->device_index = fsm->slave->device_index;
        fsm->datagram = datagram;
    }
    else
    {
        fsm->datagram = NULL;
    }

    return datagram_used;
}

/*****************************************************************************/

/**
 * @brief 将当前状态机的当前状态设置为 READY。
 *
 * @param fsm 从站状态机
 *
 * @details 此函数用于将从站状态机的当前状态设置为 READY。只有在当前状态为空闲状态时才能设置为 READY。
 *          在调用此函数之后，从站状态机将准备好处理请求。
 */
void ec_fsm_slave_set_ready(
    ec_fsm_slave_t *fsm /**< 从站状态机 */
)
{
    if (fsm->state == ec_fsm_slave_state_idle)
    {
        EC_SLAVE_DBG(fsm->slave, 1, "准备处理请求。\n");
        fsm->state = ec_fsm_slave_state_ready;
    }
}


/*****************************************************************************/

/**
 * @brief 检查是否有待处理的扫描。
 *
 * @param fsm 从站状态机。
 * @param datagram 要使用的数据报。
 * @return 如果已开始扫描则返回非零值，否则返回0。
 *
 * @details 此函数用于检查是否有待处理的扫描。如果从站需要进行扫描，则返回非零值。
 */
int ec_fsm_slave_action_scan(
    ec_fsm_slave_t *fsm,    /**< 从站状态机 */
    ec_datagram_t *datagram /**< 要使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;

    if (!slave->scan_required)
    {
        return 0;
    }

    EC_SLAVE_DBG(slave, 1, "正在扫描从站 %u，使用 %s 链接。\n",
                 slave->ring_position, ec_device_names[slave->device_index != 0]);
    fsm->state = ec_fsm_slave_state_scan;
    ec_fsm_slave_scan_start(&fsm->fsm_slave_scan);
    ec_fsm_slave_scan_exec(&fsm->fsm_slave_scan, datagram); // 立即执行
    return 1;
}


/*****************************************************************************/

#ifdef EC_EOE
/**
 * @brief 尝试重新连接到现有的EoE处理程序。
 *
 * @param slave EtherCAT从站。
 * @return 如果成功连接到EoE处理程序，则返回0；否则返回-1。
 *
 * @details 此函数用于尝试重新连接到现有的EoE（EtherCAT over Ethernet）处理程序。
 * 它首先根据从站的别名或环位置生成EoE处理程序的名称。然后，它遍历主站中的EoE处理程序列表，
 * 查找与生成的名称匹配且尚未与从站关联的处理程序。如果找到匹配的处理程序，
 * 则将从站与处理程序关联，并返回0表示连接成功。如果没有找到匹配的处理程序，
 * 则返回-1表示连接失败。
 */
static int ec_slave_reconnect_to_eoe_handler(
    ec_slave_t *slave /**< EtherCAT从站 */
)
{
    ec_master_t *master = slave->master;
    ec_eoe_t *eoe;
    char name[EC_DATAGRAM_NAME_SIZE];

    if (slave->effective_alias)
    {
        snprintf(name, EC_DATAGRAM_NAME_SIZE,
                 "eoe%ua%u", master->index, slave->effective_alias);
    }
    else
    {
        snprintf(name, EC_DATAGRAM_NAME_SIZE,
                 "eoe%us%u", master->index, slave->ring_position);
    }

    list_for_each_entry(eoe, &master->eoe_handlers, list)
    {
        if ((eoe->slave == NULL) &&
            (strncmp(name, ec_eoe_name(eoe), EC_DATAGRAM_NAME_SIZE) == 0))
        {
            ec_eoe_link_slave(eoe, slave);
            return 0;
        }
    }

    // 未找到匹配的处理程序
    return -1;
}

#endif

/*****************************************************************************/

/**
 * @brief 从站状态：扫描。
 *
 * @param fsm 从站状态机。
 * @param datagram 要使用的数据报。
 *
 * @details 此函数用于执行从站状态机的扫描状态。它首先调用`ec_fsm_slave_scan_exec`函数执行从站扫描操作。
 * 如果扫描成功完成，则函数返回。如果扫描被跳过或从站拒绝进入INIT状态，
 * 则假设从站邮箱数据有效，并将`valid_mbox_data`字段设置为1。
 * 
 * 如果从站的SII映像存在且支持EOE（EtherCAT over Ethernet）协议，
 * 则尝试重新连接到现有的EOE处理程序，如果成功则重新连接。
 * 如果无法重新连接到现有的EOE处理程序，并且启用了自动创建EOE处理程序的选项，
 * 则为该从站自动创建一个EOE处理程序，并将其添加到主站的EOE处理程序列表中。
 * 
 * 扫描完成后，禁用处理，等待主站状态机再次准备就绪，并将状态设置为闲置状态。
 */
void ec_fsm_slave_state_scan(
    ec_fsm_slave_t *fsm,    /**< 从站状态机 */
    ec_datagram_t *datagram /**< 要使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;

    if (ec_fsm_slave_scan_exec(&fsm->fsm_slave_scan, datagram))
    {
        return;
    }

    // 假设即使从站扫描跳过了清除邮箱状态（例如从站拒绝进入INIT状态），从站邮箱数据仍然有效。
    slave->valid_mbox_data = 1;

#ifdef EC_EOE
    if (slave->sii_image && (slave->sii_image->sii.mailbox_protocols & EC_MBOX_EOE))
    {
        // 尝试连接到现有的EOE处理程序，否则尝试创建新的处理程序（如果主站不活动）
        if (ec_slave_reconnect_to_eoe_handler(slave) == 0)
        {
            // 重新连接成功
        }
        else if (eoe_autocreate)
        {
            // 为该从站自动创建EOE处理程序
            ec_eoe_t *eoe;

            if (!(eoe = kmalloc(sizeof(ec_eoe_t), GFP_KERNEL)))
            {
                EC_SLAVE_ERR(slave, "Failed to allocate EoE handler memory!\n");
            }
            else if (ec_eoe_auto_init(eoe, slave))
            {
                EC_SLAVE_ERR(slave, "Failed to init EoE handler!\n");
                kfree(eoe);
            }
            else
            {
                list_add_tail(&eoe->list, &slave->master->eoe_handlers);
            }
        }
    }
#endif

    // 扫描完成后禁用处理，等待主站状态机再次准备就绪
    slave->scan_required = 0;
    fsm->state = ec_fsm_slave_state_idle;
}

/*****************************************************************************/

/**
 * @brief 检查是否存在待处理的配置。
 *
 * @return 非零值，如果配置已开始。
 *
 * @details 此函数用于检查是否存在待处理的配置。首先检查从站的错误标志，
 * 如果错误标志已设置，则返回0表示配置未开始。接下来，检查是否需要确认新的从站状态。
 * 如果当前从站状态需要确认错误，则将状态机的状态设置为ACKNOWLEDGE，并执行状态机的处理函数。
 * 返回1表示配置已开始。最后，检查从站是否需要进行配置。如果当前从站状态与请求的状态不同，
 * 或者强制进行配置，则执行配置操作，并将状态机的状态设置为CONFIG，并执行状态机的处理函数。
 * 返回1表示配置已开始。如果以上条件都不满足，则返回0表示配置未开始。
 */
int ec_fsm_slave_action_config(
    ec_fsm_slave_t *fsm,    /**< 从站状态机 */
    ec_datagram_t *datagram /**< 要使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;

    if (slave->error_flag)
    {
        return 0;
    }

    // 检查是否需要确认新的从站状态
    if (slave->current_state & EC_SLAVE_STATE_ACK_ERR)
    {
        fsm->state = ec_fsm_slave_state_acknowledge;
        ec_fsm_change_ack(&fsm->fsm_change, slave);
        fsm->state(fsm, datagram); // 立即执行
        return 1;
    }

    // 从站是否需要进行配置？
    if (slave->current_state != slave->requested_state || slave->force_config)
    {

        if (slave->master->debug_level)
        {
            char old_state[EC_STATE_STRING_SIZE],
                new_state[EC_STATE_STRING_SIZE];
            ec_state_string(slave->current_state, old_state, 0);
            ec_state_string(slave->requested_state, new_state, 0);
            EC_SLAVE_DBG(slave, 1, "Changing state from %s to %s%s.\n",
                         old_state, new_state,
                         slave->force_config ? " (forced)" : "");
        }

        ec_lock_down(&slave->master->config_sem);
        ++slave->master->config_busy;
        ec_lock_up(&slave->master->config_sem);

        fsm->state = ec_fsm_slave_state_config;
#ifdef EC_QUICK_OP
        if (!slave->force_config && slave->current_state == EC_SLAVE_STATE_SAFEOP && slave->requested_state == EC_SLAVE_STATE_OP && slave->last_al_error == 0x001B)
        {
            // 上次错误是同步看门狗超时；假设通信中断，并请求快速切换回OP状态
            ec_fsm_slave_config_quick_start(&fsm->fsm_slave_config);
        }
        else
#endif
        {
            ec_fsm_slave_config_start(&fsm->fsm_slave_config);
        }
        fsm->state(fsm, datagram); // 立即执行
        return 1;
    }
    return 0;
}

/*****************************************************************************/

/**
 * @brief 从站状态：确认。
 *
 * @param fsm 从站状态机。
 * @param datagram 要使用的数据报。
 *
 * @details 此函数用于处理从站状态机的确认状态。首先执行状态变更操作，
 * 如果操作成功，则返回。如果操作失败，则设置从站的错误标志，并输出错误信息。
 * 最后，将状态机的状态设置为READY。
 */
void ec_fsm_slave_state_acknowledge(
    ec_fsm_slave_t *fsm,    /**< 从站状态机 */
    ec_datagram_t *datagram /**< 要使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;

    if (ec_fsm_change_exec(&fsm->fsm_change, datagram))
    {
        return;
    }

    if (!ec_fsm_change_success(&fsm->fsm_change))
    {
        slave->error_flag = 1;
        EC_SLAVE_ERR(slave, "Failed to acknowledge state change.\n");
    }

    fsm->state = ec_fsm_slave_state_ready;
}

/*****************************************************************************/

/**
 * @brief 从站状态：配置。
 *
 * @param fsm 从站状态机。
 * @param datagram 要使用的数据报。
 *
 * @details 此函数用于处理从站状态机的配置状态。首先执行配置操作，
 * 如果操作成功，则返回。如果操作失败，则标记从站配置为失败（TODO）。
 * 然后，将从站的强制配置标志设置为0，并减少主站的配置繁忙计数。
 * 如果主站的配置繁忙计数为0，则唤醒等待的配置队列。
 * 最后，将状态机的状态设置为READY。
 */
void ec_fsm_slave_state_config(
    ec_fsm_slave_t *fsm,    /**< 从站状态机 */
    ec_datagram_t *datagram /**< 要使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;

    if (ec_fsm_slave_config_exec(&fsm->fsm_slave_config, datagram))
    {
        return;
    }

    if (!ec_fsm_slave_config_success(&fsm->fsm_slave_config))
    {
        // TODO: 标记从站配置为失败
    }

    slave->force_config = 0;

    ec_lock_down(&slave->master->config_sem);
    if (slave->master->config_busy)
    {
        if (--slave->master->config_busy == 0)
        {
            wake_up_interruptible(&slave->master->config_queue);
        }
    }
    ec_lock_up(&slave->master->config_sem);

    fsm->state = ec_fsm_slave_state_ready;
}

/*****************************************************************************/

/**
 * @brief 检查是否存在待处理的SDO字典读取请求。
 *
 * @return 非零值，如果已开始SDO字典读取。
 *
 * @details 此函数用于检查是否存在待处理的SDO字典读取请求。首先检查是否存在显式的字典请求需要处理。
 * 如果存在请求，则取出第一个请求进行处理。如果从站的SII映像尚未准备好，则返回0表示字典请求未开始。
 * 如果从站不支持SDO Info或者SDO Info未使能，则返回0表示字典请求未开始。如果字典已经上传完成，
 * 则返回0表示字典请求未开始。如果从站的当前状态需要确认错误，则返回0表示字典请求未开始。
 * 如果从站的当前状态为INIT，则返回0表示字典请求未开始。如果以上条件都不满足，则将状态机的状态设置为字典请求状态，
 * 并执行字典请求的处理函数。返回1表示字典请求已开始。如果没有显式的字典请求需要处理，则检查是否需要在启动时获取字典。
 * 如果从站的SII映像不存在、字典已经上传完成、从站的当前状态为INIT或UNKNOWN、从站的当前状态需要确认错误、
 * 从站不支持SDO Info或者SDO Info未使能，则返回0表示字典请求未开始。否则，将状态机的字典请求设置为内部字典请求，
 * 并将状态机的状态设置为字典请求状态，并执行字典请求的处理函数。返回1表示字典请求已开始。
 */
int ec_fsm_slave_action_process_dict(
    ec_fsm_slave_t *fsm,    /**< 从站状态机 */
    ec_datagram_t *datagram /**< 要使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;
    ec_dict_request_t *request;

    // 首先检查是否存在显式的字典请求需要处理
    if (!list_empty(&slave->dict_requests))
    {
        // 取出第一个请求进行处理
        request = list_entry(slave->dict_requests.next, ec_dict_request_t, list);
        list_del_init(&request->list); // 出队

        if (!slave->sii_image)
        {
            EC_SLAVE_ERR(slave, "从站尚未准备好以处理字典请求\n");
            request->state = EC_INT_REQUEST_FAILURE;
            wake_up_all(&slave->master->request_queue);
            fsm->state = ec_fsm_slave_state_idle;
            return 1;
        }

        if (!(slave->sii_image->sii.mailbox_protocols & EC_MBOX_COE) || (slave->sii_image->sii.has_general && !slave->sii_image->sii.coe_details.enable_sdo_info))
        {
            EC_SLAVE_INFO(slave, "中止字典请求，从站不支持SDO Info。\n");
            request->state = EC_INT_REQUEST_SUCCESS;
            wake_up_all(&slave->master->request_queue);
            fsm->dict_request = NULL;
            fsm->state = ec_fsm_slave_state_ready;
            return 1;
        }

        if (slave->sdo_dictionary_fetched)
        {
            EC_SLAVE_DBG(slave, 1, "中止字典请求，字典已上传完成。\n");
            request->state = EC_INT_REQUEST_SUCCESS;
            wake_up_all(&slave->master->request_queue);
            fsm->dict_request = NULL;
            fsm->state = ec_fsm_slave_state_ready;
            return 1;
        }

        if (slave->current_state & EC_SLAVE_STATE_ACK_ERR)
        {
            EC_SLAVE_WARN(slave, "中止字典请求，从站错误标志已设置。\n");
            request->state = EC_INT_REQUEST_FAILURE;
            wake_up_all(&slave->master->request_queue);
            fsm->state = ec_fsm_slave_state_idle;
            return 1;
        }

        if (slave->current_state == EC_SLAVE_STATE_INIT)
        {
            EC_SLAVE_WARN(slave, "中止字典请求，从站处于INIT状态。\n");
            request->state = EC_INT_REQUEST_FAILURE;
            wake_up_all(&slave->master->request_queue);
            fsm->state = ec_fsm_slave_state_idle;
            return 1;
        }

        fsm->dict_request = request;
        request->state = EC_INT_REQUEST_BUSY;

        // 找到待处理的字典请求，执行它！
        EC_SLAVE_DBG(slave, 1, "处理字典请求...\n");

        // 开始字典传输
        fsm->state = ec_fsm_slave_state_dict_request;
        ec_fsm_coe_dictionary(&fsm->fsm_coe, slave);
        ec_fsm_coe_exec(&fsm->fsm_coe, datagram); // 立即执行
        return 1;
    }

    // 否则检查是否是在启动时获取字典的时机
#if EC_SKIP_SDO_DICT
    return 0;
#else
    if (!slave->sii_image || slave->sdo_dictionary_fetched || slave->current_state == EC_SLAVE_STATE_INIT || slave->current_state == EC_SLAVE_STATE_UNKNOWN || slave->current_state & EC_SLAVE_STATE_ACK_ERR || !(slave->sii_image->sii.mailbox_protocols & EC_MBOX_COE) || (slave->sii_image->sii.has_general && !slave->sii_image->sii.coe_details.enable_sdo_info))
    {
        return 0;
    }

    fsm->dict_request = &fsm->int_dict_request;
    fsm->int_dict_request.state = EC_INT_REQUEST_BUSY;

    EC_SLAVE_DBG(slave, 1, "获取SDO字典。\n");

    // 开始字典传输
    fsm->state = ec_fsm_slave_state_dict_request;
    ec_fsm_coe_dictionary(&fsm->fsm_coe, slave);
    ec_fsm_coe_exec(&fsm->fsm_coe, datagram); // 立即执行
    return 1;
#endif
}

/*****************************************************************************/

/**
 * @brief 从站状态：DICT_REQUEST。
 */
void ec_fsm_slave_state_dict_request(
    ec_fsm_slave_t *fsm,    /**< 从站状态机 */
    ec_datagram_t *datagram /**< 要使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;
    ec_dict_request_t *request = fsm->dict_request;

    if (ec_fsm_coe_exec(&fsm->fsm_coe, datagram))
    {
        return;
    }

    if (!ec_fsm_coe_success(&fsm->fsm_coe))
    {
        EC_SLAVE_ERR(slave, "处理字典请求失败。\n");
#if !EC_SKIP_SDO_DICT
        if (request == &fsm->int_dict_request)
        {
            // 标记为已获取，以免重试
            slave->sdo_dictionary_fetched = 1;
        }
#endif
        request->state = EC_INT_REQUEST_FAILURE;
        wake_up_all(&slave->master->request_queue);
        fsm->dict_request = NULL;
        fsm->state = ec_fsm_slave_state_ready;
        return;
    }

    if (slave->master->debug_level)
    {
        unsigned int sdo_count, entry_count;
        ec_slave_sdo_dict_info(slave, &sdo_count, &entry_count);
        EC_SLAVE_DBG(slave, 1, "获取到%u个SDO和%u个条目。\n",
                     sdo_count, entry_count);
    }

    // 字典请求完成
    slave->sdo_dictionary_fetched = 1;

    // 附加PDO名称到字典
    ec_slave_attach_pdo_names(slave);

    request->state = EC_INT_REQUEST_SUCCESS;
    wake_up_all(&slave->master->request_queue);
    fsm->dict_request = NULL;
    fsm->state = ec_fsm_slave_state_ready;
}

/*****************************************************************************/

/**
 * @brief 检查是否存在待处理的内部SDO请求，并处理其中一个。
 *
 * @return 非零值，如果已处理SDO请求。
 *
 * @details 此函数用于检查是否存在待处理的内部SDO请求，并处理其中一个。如果从站的配置不存在，则返回0表示未处理SDO请求。
 * 遍历从站的SDO请求列表，找到状态为QUEUED的请求进行处理。如果请求超时，则将其状态设置为FAILURE，并输出错误信息。
 * 如果从站的当前状态需要确认错误，则将请求状态设置为FAILURE。如果从站的当前状态为INIT，则将请求状态设置为FAILURE。
 * 如果以上条件都不满足，则将请求状态设置为BUSY，并执行SDO请求的处理函数。返回1表示已处理SDO请求。
 */
int ec_fsm_slave_action_process_config_sdo(
    ec_fsm_slave_t *fsm,    /**< 从站状态机 */
    ec_datagram_t *datagram /**< 要使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;
    ec_sdo_request_t *request;

    if (!slave->config)
    {
        return 0;
    }

    list_for_each_entry(request, &slave->config->sdo_requests, list)
    {
        if (request->state == EC_INT_REQUEST_QUEUED)
        {
            if (ec_sdo_request_timed_out(request))
            {
                request->state = EC_INT_REQUEST_FAILURE;
                EC_SLAVE_DBG(slave, 1, "内部SDO请求超时。\n");
                continue;
            }

            if (slave->current_state & EC_SLAVE_STATE_ACK_ERR)
            {
                EC_SLAVE_WARN(slave, "中止SDO请求，从站错误标志已设置。\n");
                request->state = EC_INT_REQUEST_FAILURE;
                continue;
            }

            if (slave->current_state == EC_SLAVE_STATE_INIT)
            {
                EC_SLAVE_WARN(slave, "中止SDO请求，从站处于INIT状态。\n");
                request->state = EC_INT_REQUEST_FAILURE;
                continue;
            }

            request->state = EC_INT_REQUEST_BUSY;
            EC_SLAVE_DBG(slave, 1, "处理内部SDO请求...\n");
            fsm->sdo_request = request;
            fsm->state = ec_fsm_slave_state_sdo_request;
            ec_fsm_coe_transfer(&fsm->fsm_coe, slave, request);
            ec_fsm_coe_exec(&fsm->fsm_coe, datagram);
            return 1;
        }
    }
    return 0;
}

/*****************************************************************************/

/**
 * @brief 将状态机的当前状态设置为IDLE。
 *
 * @return 非零值，如果成功；否则状态机忙。
 *
 * @details 此函数用于将状态机的当前状态设置为IDLE。如果状态机的当前状态已经是IDLE，则返回1表示成功。
 * 如果状态机的当前状态是READY，则将状态机的状态设置为IDLE，并返回1表示成功。否则返回0表示状态机忙。
 */
int ec_fsm_slave_set_unready(
    ec_fsm_slave_t *fsm /**< 从站状态机 */
)
{
    if (fsm->state == ec_fsm_slave_state_idle)
    {
        return 1;
    }
    else if (fsm->state == ec_fsm_slave_state_ready)
    {
        EC_SLAVE_DBG(fsm->slave, 1, "不接受请求。\n");
        fsm->state = ec_fsm_slave_state_idle;
        return 1;
    }
    return 0;
}

/*****************************************************************************/

/**
 * @brief 返回状态机是否当前未忙并且准备执行。
 *
 * @return 非零值，如果准备就绪。
 *
 * @details 此函数用于检查状态机是否当前未忙并且准备执行。如果状态机的当前状态是READY，则返回非零值表示准备就绪。
 */
int ec_fsm_slave_is_ready(
    const ec_fsm_slave_t *fsm /**< 从站状态机 */
)
{
    return fsm->state == ec_fsm_slave_state_ready;
}

/******************************************************************************
 * Slave state machine
 *****************************************************************************/

/**
 * @brief 从站状态：IDLE。
 */
void ec_fsm_slave_state_idle(
    ec_fsm_slave_t *fsm,    /**< 从站状态机 */
    ec_datagram_t *datagram /**< 要使用的数据报 */
)
{
    // 什么都不做
}

/*****************************************************************************/

/**
 * @brief 从站状态：READY。
 */
void ec_fsm_slave_state_ready(
    ec_fsm_slave_t *fsm,    /**< 从站状态机 */
    ec_datagram_t *datagram /**< 要使用的数据报 */
)
{
    // 检查是否存在待处理的扫描请求
    if (ec_fsm_slave_action_scan(fsm, datagram))
    {
        return;
    }

    // 检查是否存在待处理的配置请求
    if (ec_fsm_slave_action_config(fsm, datagram))
    {
        return;
    }

    // 检查是否存在待处理的内部SDO请求
    if (ec_fsm_slave_action_process_config_sdo(fsm, datagram))
    {
        return;
    }

    // 检查从站是否需要读取SDO字典
    if (ec_fsm_slave_action_process_dict(fsm, datagram))
    {
        return;
    }

    // 检查是否存在待处理的外部SDO请求
    if (ec_fsm_slave_action_process_sdo(fsm, datagram))
    {
        return;
    }

    // 检查是否存在待处理的外部寄存器请求
    if (ec_fsm_slave_action_process_reg(fsm, datagram))
    {
        return;
    }

    // 检查是否存在待处理的FoE请求
    if (ec_fsm_slave_action_process_foe(fsm, datagram))
    {
        return;
    }

    // 检查是否存在待处理的SoE请求
    if (ec_fsm_slave_action_process_soe(fsm, datagram))
    {
        return;
    }

#ifdef EC_EOE
    // 检查是否存在待处理的EoE IP参数请求
    if (ec_fsm_slave_action_process_eoe(fsm, datagram))
    {
        return;
    }
#endif

    // 检查是否存在待处理的MBox Gateway请求
    if (ec_fsm_slave_action_process_mbg(fsm, datagram))
    {
        return;
    }
}

/*****************************************************************************/

/**
 * @brief 检查是否存在待处理的SDO请求并处理其中一个。
 *
 * @return 非零值，如果已处理SDO请求。
 *
 * @details 此函数用于检查是否存在待处理的SDO请求并处理其中一个。如果从站的配置不存在，则返回0表示未处理SDO请求。
 * 遍历从站的SDO请求列表，找到状态为QUEUED的请求进行处理。如果请求超时，则将其状态设置为FAILURE，并输出错误信息。
 * 如果从站的当前状态需要确认错误，则将请求状态设置为FAILURE。如果从站的当前状态为INIT，则将请求状态设置为FAILURE。
 * 如果以上条件都不满足，则将请求状态设置为BUSY，并执行SDO请求的处理函数。返回1表示已处理SDO请求。
 */
int ec_fsm_slave_action_process_sdo(
    ec_fsm_slave_t *fsm,    /**< 从站状态机 */
    ec_datagram_t *datagram /**< 要使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;
    ec_sdo_request_t *request;

    if (!slave->config)
    {
        return 0;
    }

    list_for_each_entry(request, &slave->config->sdo_requests, list)
    {
        if (request->state == EC_INT_REQUEST_QUEUED)
        {
            if (ec_sdo_request_timed_out(request))
            {
                request->state = EC_INT_REQUEST_FAILURE;
                EC_SLAVE_DBG(slave, 1, "内部SDO请求超时。\n");
                continue;
            }

            if (slave->current_state & EC_SLAVE_STATE_ACK_ERR)
            {
                EC_SLAVE_WARN(slave, "中止SDO请求，从站错误标志已设置。\n");
                request->state = EC_INT_REQUEST_FAILURE;
                continue;
            }

            if (slave->current_state == EC_SLAVE_STATE_INIT)
            {
                EC_SLAVE_WARN(slave, "中止SDO请求，从站处于INIT状态。\n");
                request->state = EC_INT_REQUEST_FAILURE;
                continue;
            }

            request->state = EC_INT_REQUEST_BUSY;
            EC_SLAVE_DBG(slave, 1, "处理内部SDO请求...\n");
            fsm->sdo_request = request;
            fsm->state = ec_fsm_slave_state_sdo_request;
            ec_fsm_coe_transfer(&fsm->fsm_coe, slave, request);
            ec_fsm_coe_exec(&fsm->fsm_coe, datagram);
            return 1;
        }
    }
    return 0;
}

/*****************************************************************************/

/**
 * @brief 从站状态：SDO_REQUEST。
 */
void ec_fsm_slave_state_sdo_request(
    ec_fsm_slave_t *fsm,    /**< 从站状态机 */
    ec_datagram_t *datagram /**< 要使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;
    ec_sdo_request_t *request = fsm->sdo_request;

    if (ec_fsm_coe_exec(&fsm->fsm_coe, datagram))
    {
        return;
    }

    if (!ec_fsm_coe_success(&fsm->fsm_coe))
    {
        EC_SLAVE_ERR(slave, "处理SDO请求失败。\n");
        request->state = EC_INT_REQUEST_FAILURE;
        wake_up_all(&slave->master->request_queue);
        fsm->sdo_request = NULL;
        fsm->state = ec_fsm_slave_state_ready;
        return;
    }

    EC_SLAVE_DBG(slave, 1, "SDO请求处理完成。\n");

    // SDO请求处理完成
    request->state = EC_INT_REQUEST_SUCCESS;
    wake_up_all(&slave->master->request_queue);
    fsm->sdo_request = NULL;
    fsm->state = ec_fsm_slave_state_ready;
}

/*****************************************************************************/

/**
 * @brief 检查是否存在待处理的寄存器请求并处理其中一个。
 *
 * @return 非零值，如果已处理寄存器请求。
 *
 * @details 此函数用于检查是否存在待处理的寄存器请求并处理其中一个。如果从站的配置不存在，则返回0表示未处理寄存器请求。
 * 遍历从站的寄存器请求列表，找到状态为QUEUED的请求进行处理。如果请求超时，则将其状态设置为FAILURE，并输出错误信息。
 * 如果从站的当前状态需要确认错误，则将请求状态设置为FAILURE。如果从站的当前状态为INIT，则将请求状态设置为FAILURE。
 * 如果以上条件都不满足，则将请求状态设置为BUSY，并执行寄存器请求的处理函数。返回1表示已处理寄存器请求。
 */
int ec_fsm_slave_action_process_reg(
    ec_fsm_slave_t *fsm,    /**< 从站状态机 */
    ec_datagram_t *datagram /**< 要使用的数据报 */
)
{
    static const char *direction_names[] = {
        "无效",     // EC_DIR_INVALID
        "下载",     // EC_DIR_OUTPUT
        "上传",     // EC_DIR_INPUT
        "下载+上传", // EC_DIR_BOTH
    };

    ec_slave_t *slave = fsm->slave;
    ec_reg_request_t *reg;

    fsm->reg_request = NULL;

    if (slave->config)
    {
        // 找到第一个要处理的内部寄存器请求
        list_for_each_entry(reg, &slave->config->reg_requests, list)
        {
            if (reg->state == EC_INT_REQUEST_QUEUED)
            {
                fsm->reg_request = reg;
                break;
            }
        }
    }

    if (!fsm->reg_request && !list_empty(&slave->reg_requests))
    {
        // 取出第一个要处理的外部请求
        fsm->reg_request =
            list_entry(slave->reg_requests.next, ec_reg_request_t, list);
        list_del_init(&fsm->reg_request->list); // 出队
    }

    if (!fsm->reg_request)
    { // 没有要处理的寄存器请求
        return 0;
    }

    if (slave->current_state & EC_SLAVE_STATE_ACK_ERR)
    {
        EC_SLAVE_WARN(slave, "中止寄存器请求，从站错误标志已设置。\n");
        fsm->reg_request->state = EC_INT_REQUEST_FAILURE;
        wake_up_all(&slave->master->request_queue);
        fsm->reg_request = NULL;
        fsm->state = ec_fsm_slave_state_idle;
        return 0;
    }

    // 找到待处理的寄存器请求，执行它！
    EC_SLAVE_DBG(slave, 1, "处理寄存器请求 %s 0x%04X-0x%04X...\n",
                 direction_names[fsm->reg_request->dir % EC_DIR_COUNT], fsm->reg_request->address,
                 fsm->reg_request->address + (unsigned)fsm->reg_request->transfer_size - 1u);

    fsm->reg_request->state = EC_INT_REQUEST_BUSY;

    // 开始寄存器访问
    switch (fsm->reg_request->dir)
    {
    case EC_DIR_INPUT:
        ec_datagram_fprd(datagram, slave->station_address,
                         fsm->reg_request->address, fsm->reg_request->transfer_size);
        ec_datagram_zero(datagram);
        break;
    case EC_DIR_OUTPUT:
        ec_datagram_fpwr(datagram, slave->station_address,
                         fsm->reg_request->address, fsm->reg_request->transfer_size);
        memcpy(datagram->data, fsm->reg_request->data,
               fsm->reg_request->transfer_size);
        break;
    case EC_DIR_BOTH:
        ec_datagram_fprw(datagram, slave->station_address,
                         fsm->reg_request->address, fsm->reg_request->transfer_size);
        memcpy(datagram->data, fsm->reg_request->data,
               fsm->reg_request->transfer_size);
        break;
    default:
        EC_SLAVE_WARN(slave, "中止寄存器请求，未知方向。\n");
        fsm->reg_request->state = EC_INT_REQUEST_FAILURE;
        wake_up_all(&slave->master->request_queue);
        fsm->reg_request = NULL;
        fsm->state = ec_fsm_slave_state_idle;
        return 1;
    }
    datagram->device_index = slave->device_index;
    fsm->state = ec_fsm_slave_state_reg_request;
    return 1;
}

/*****************************************************************************/

/** Slave state: 寄存器请求。
 */
void ec_fsm_slave_state_reg_request(
    ec_fsm_slave_t *fsm,    /**< 从站状态机 */
    ec_datagram_t *datagram /**< 要使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;
    ec_reg_request_t *reg = fsm->reg_request;

    if (!reg)
    {
        // 配置在此期间被清除
        fsm->state = ec_fsm_slave_state_ready;
        fsm->reg_request = NULL;
        return;
    }

    if (fsm->datagram->state != EC_DATAGRAM_RECEIVED)
    {
        EC_SLAVE_ERR(slave, "无法接收寄存器请求数据报：");
        ec_datagram_print_state(fsm->datagram);
        reg->state = EC_INT_REQUEST_FAILURE;
        wake_up_all(&slave->master->request_queue);
        fsm->reg_request = NULL;
        fsm->state = ec_fsm_slave_state_ready;
        return;
    }

    if (fsm->datagram->working_counter == ((reg->dir == EC_DIR_BOTH) ? 3 : 1))
    {
        if (reg->dir != EC_DIR_OUTPUT)
        { // 读取/读写请求
            memcpy(reg->data, fsm->datagram->data, reg->transfer_size);
        }

        reg->state = EC_INT_REQUEST_SUCCESS;
        EC_SLAVE_DBG(slave, 1, "寄存器请求成功。\n");
    }
    else
    {
        reg->state = EC_INT_REQUEST_FAILURE;
        ec_datagram_print_state(fsm->datagram);
        EC_SLAVE_ERR(slave, "寄存器请求失败（工作计数器为 %u）。\n",
                     fsm->datagram->working_counter);
    }

    wake_up_all(&slave->master->request_queue);
    fsm->reg_request = NULL;
    fsm->state = ec_fsm_slave_state_ready;
}

/*****************************************************************************/

/**
 * @brief 检查是否存在待处理的FoE请求并处理其中一个。
 *
 * @return 非零值，如果已处理FoE请求。
 *
 * @details 此函数用于检查是否存在待处理的FoE请求并处理其中一个。如果从站的配置不存在，则返回0表示未处理FoE请求。
 * 遍历从站的FoE请求列表，找到状态为QUEUED的请求进行处理。如果请求超时，则将其状态设置为FAILURE，并输出错误信息。
 * 如果从站的当前状态需要确认错误，则将请求状态设置为FAILURE。如果以上条件都不满足，则将请求状态设置为BUSY，并执行FoE请求的处理函数。返回1表示已处理FoE请求。
 */
int ec_fsm_slave_action_process_foe(
    ec_fsm_slave_t *fsm,    /**< 从站状态机 */
    ec_datagram_t *datagram /**< 要使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;
    ec_foe_request_t *request;

    if (!slave->config)
    {
        return 0;
    }

    list_for_each_entry(request, &slave->config->foe_requests, list)
    {
        if (request->state == EC_INT_REQUEST_QUEUED)
        {
            if (ec_foe_request_timed_out(request))
            {
                request->state = EC_INT_REQUEST_FAILURE;
                EC_SLAVE_DBG(slave, 1, "内部FoE请求超时。\n");
                continue;
            }

            if (slave->current_state & EC_SLAVE_STATE_ACK_ERR)
            {
                EC_SLAVE_WARN(slave, "中止FoE请求，从站错误标志已设置。\n");
                request->state = EC_INT_REQUEST_FAILURE;
                continue;
            }

            request->state = EC_INT_REQUEST_BUSY;
            EC_SLAVE_DBG(slave, 1, "处理FoE请求...\n");
            fsm->foe_request = request;
            fsm->state = ec_fsm_slave_state_foe_request;
            ec_fsm_foe_transfer(&fsm->fsm_foe, slave, request);
            ec_fsm_foe_exec(&fsm->fsm_foe, datagram);
            return 1;
        }
    }
    return 0;
}

/*****************************************************************************/

/**
 * @brief 从站状态：FOE REQUEST。
 */
void ec_fsm_slave_state_foe_request(
    ec_fsm_slave_t *fsm,    /**< 从站状态机 */
    ec_datagram_t *datagram /**< 要使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;
    ec_foe_request_t *request = fsm->foe_request;

    if (ec_fsm_foe_exec(&fsm->fsm_foe, datagram))
    {
        return;
    }

    if (!ec_fsm_foe_success(&fsm->fsm_foe))
    {
        EC_SLAVE_ERR(slave, "处理FoE请求失败。\n");
        request->state = EC_INT_REQUEST_FAILURE;
        wake_up_all(&slave->master->request_queue);
        fsm->foe_request = NULL;
        fsm->state = ec_fsm_slave_state_ready;
        return;
    }

    // FoE传输完成
    EC_SLAVE_DBG(slave, 1, "成功传输 %zu 字节的FoE数据。\n",
                 request->data_size);

    request->state = EC_INT_REQUEST_SUCCESS;
    wake_up_all(&slave->master->request_queue);
    fsm->foe_request = NULL;
    fsm->state = ec_fsm_slave_state_ready;
}

/*****************************************************************************/

/**
 * @brief 检查是否存在待处理的SoE请求并处理其中一个。
 *
 * @return 非零值，如果已处理请求。
 *
 * @details 此函数用于检查是否存在待处理的SoE请求并处理其中一个。如果从站的SoE请求列表为空，则返回0表示未处理请求。
 * 取出第一个要处理的请求进行处理。如果从站的当前状态需要确认错误，则将请求状态设置为FAILURE。
 * 如果从站的当前状态为INIT，则将请求状态设置为FAILURE。如果以上条件都不满足，则将请求状态设置为BUSY，并执行SoE请求的处理函数。返回1表示已处理请求。
 */
int ec_fsm_slave_action_process_soe(
    ec_fsm_slave_t *fsm,    /**< 从站状态机 */
    ec_datagram_t *datagram /**< 要使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;
    ec_soe_request_t *req;

    if (list_empty(&slave->soe_requests))
    {
        return 0;
    }

    // 取出第一个要处理的请求
    req = list_entry(slave->soe_requests.next, ec_soe_request_t, list);
    list_del_init(&req->list); // 出队

    if (slave->current_state & EC_SLAVE_STATE_ACK_ERR)
    {
        EC_SLAVE_WARN(slave, "中止SoE请求，从站错误标志已设置。\n");
        req->state = EC_INT_REQUEST_FAILURE;
        wake_up_all(&slave->master->request_queue);
        fsm->state = ec_fsm_slave_state_idle;
        return 0;
    }

    if (slave->current_state == EC_SLAVE_STATE_INIT)
    {
        EC_SLAVE_WARN(slave, "中止SoE请求，从站处于INIT状态。\n");
        req->state = EC_INT_REQUEST_FAILURE;
        wake_up_all(&slave->master->request_queue);
        fsm->state = ec_fsm_slave_state_idle;
        return 0;
    }

    fsm->soe_request = req;
    req->state = EC_INT_REQUEST_BUSY;

    // 找到待处理的请求，执行它！
    EC_SLAVE_DBG(slave, 1, "处理SoE请求...\n");

    // 开始SoE传输
    fsm->state = ec_fsm_slave_state_soe_request;
    ec_fsm_soe_transfer(&fsm->fsm_soe, slave, req);
    ec_fsm_soe_exec(&fsm->fsm_soe, datagram); // 立即执行
    return 1;
}

/*****************************************************************************/

/**
 * @brief 检查是否存在待处理的MBox Gateway请求并处理其中一个。
 *
 * @return 非零值，如果已处理请求。
 *
 * @details 此函数用于检查是否存在待处理的MBox Gateway请求并处理其中一个。如果从站的MBox Gateway请求列表为空，则返回0表示未处理请求。
 * 取出第一个要处理的请求进行处理。如果从站的当前状态需要确认错误，则将请求状态设置为FAILURE。
 * 如果从站的当前状态为INIT，则将请求状态设置为FAILURE。如果以上条件都不满足，则将请求状态设置为BUSY，并执行MBox Gateway请求的处理函数。返回1表示已处理请求。
 */
int ec_fsm_slave_action_process_mbg(
    ec_fsm_slave_t *fsm,    /**< 从站状态机 */
    ec_datagram_t *datagram /**< 要使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;
    ec_mbg_request_t *req;

    if (list_empty(&slave->mbg_requests))
    {
        return 0;
    }

    // 取出第一个要处理的请求
    req = list_entry(slave->mbg_requests.next, ec_mbg_request_t, list);
    list_del_init(&req->list); // 出队

    if (slave->current_state & EC_SLAVE_STATE_ACK_ERR)
    {
        EC_SLAVE_WARN(slave, "中止MBox Gateway请求，从站错误标志已设置。\n");
        req->state = EC_INT_REQUEST_FAILURE;
        wake_up_all(&slave->master->request_queue);
        fsm->state = ec_fsm_slave_state_idle;
        return 0;
    }

    if (slave->current_state == EC_SLAVE_STATE_INIT)
    {
        EC_SLAVE_WARN(slave, "中止MBox Gateway请求，从站处于INIT状态。\n");
        req->state = EC_INT_REQUEST_FAILURE;
        wake_up_all(&slave->master->request_queue);
        fsm->state = ec_fsm_slave_state_idle;
        return 0;
    }

    fsm->mbg_request = req;
    req->state = EC_INT_REQUEST_BUSY;

    // 找到待处理的请求，执行它！
    EC_SLAVE_DBG(slave, 1, "处理MBox Gateway请求...\n");

    // 开始MBox Gateway传输
    fsm->state = ec_fsm_slave_state_mbg_request;
    ec_fsm_mbg_transfer(&fsm->fsm_mbg, slave, req);
    ec_fsm_mbg_exec(&fsm->fsm_mbg, datagram); // 立即执行
    return 1;
}

/*****************************************************************************/
/**
 * @brief 从站状态: MBG_REQUEST（MBox Gateway请求）。
 *
 * @param fsm 从站状态机。
 * @param datagram 要使用的数据报。
 *
 * @details 此函数用于处理从站处于MBG_REQUEST状态时的操作。它执行MBG请求的处理，
 * 并根据处理结果更新请求状态。如果MBG执行失败，则将请求状态设置为EC_INT_REQUEST_FAILURE，
 * 并唤醒主站的请求队列。如果MBG执行成功，则将请求状态设置为EC_INT_REQUEST_SUCCESS，
 * 并唤醒主站的请求队列。最后，将MBG请求和从站状态设置为NULL，并将从站状态设置为准备就绪状态。
 */
void ec_fsm_slave_state_mbg_request(
    ec_fsm_slave_t *fsm,    /**< 从站状态机 */
    ec_datagram_t *datagram /**< 要使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;
    ec_mbg_request_t *request = fsm->mbg_request;

    if (ec_fsm_mbg_exec(&fsm->fsm_mbg, datagram))
    {
        return;
    }

    if (!ec_fsm_mbg_success(&fsm->fsm_mbg))
    {
        EC_SLAVE_ERR(slave, "处理MBox Gateway请求失败。\n");
        request->state = EC_INT_REQUEST_FAILURE;
        wake_up_all(&slave->master->request_queue);
        fsm->mbg_request = NULL;
        fsm->state = ec_fsm_slave_state_ready;
        return;
    }

    EC_SLAVE_DBG(slave, 1, "MBox Gateway请求处理完成。\n");

    // MBox Gateway请求处理完成
    request->state = EC_INT_REQUEST_SUCCESS;
    wake_up_all(&slave->master->request_queue);
    fsm->mbg_request = NULL;
    fsm->state = ec_fsm_slave_state_ready;
}

/*****************************************************************************/

/**
 * @brief 从站状态: SOE_REQUEST（SoE请求）。
 *
 * @param fsm 从站状态机。
 * @param datagram 要使用的数据报。
 *
 * @details 此函数用于处理从站处于SOE_REQUEST状态时的操作。它执行SoE请求的处理，
 * 并根据处理结果更新请求状态。如果SoE执行失败，则将请求状态设置为EC_INT_REQUEST_FAILURE，
 * 并唤醒主站的请求队列。如果SoE执行成功，则将请求状态设置为EC_INT_REQUEST_SUCCESS，
 * 并唤醒主站的请求队列。最后，将SoE请求和从站状态设置为NULL，并将从站状态设置为准备就绪状态。
 */
void ec_fsm_slave_state_soe_request(
    ec_fsm_slave_t *fsm,    /**< 从站状态机 */
    ec_datagram_t *datagram /**< 要使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;
    ec_soe_request_t *request = fsm->soe_request;

    if (ec_fsm_soe_exec(&fsm->fsm_soe, datagram))
    {
        return;
    }

    if (!ec_fsm_soe_success(&fsm->fsm_soe))
    {
        EC_SLAVE_ERR(slave, "处理SoE请求失败。\n");
        request->state = EC_INT_REQUEST_FAILURE;
        wake_up_all(&slave->master->request_queue);
        fsm->soe_request = NULL;
        fsm->state = ec_fsm_slave_state_ready;
        return;
    }

    EC_SLAVE_DBG(slave, 1, "SoE请求处理完成。\n");

    // SoE请求处理完成
    request->state = EC_INT_REQUEST_SUCCESS;
    wake_up_all(&slave->master->request_queue);
    fsm->soe_request = NULL;
    fsm->state = ec_fsm_slave_state_ready;
}

/*****************************************************************************/
#ifdef EC_EOE
/**
 * @brief 检查是否有待处理的EoE IP参数请求，并处理其中一个。
 *
 * @return 非零值，如果成功处理了一个请求。
 */
int ec_fsm_slave_action_process_eoe(
    ec_fsm_slave_t *fsm,    /**< 从站状态机 */
    ec_datagram_t *datagram /**< 要使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;
    ec_eoe_request_t *request;

    if (list_empty(&slave->eoe_requests))
    {
        return 0;
    }

    // 取出第一个待处理的请求
    request = list_entry(slave->eoe_requests.next, ec_eoe_request_t, list);
    list_del_init(&request->list); // 出队

    if (slave->current_state & EC_SLAVE_STATE_ACK_ERR)
    {
        EC_SLAVE_WARN(slave, "放弃EoE请求，从站错误标志已设置。\n");
        request->state = EC_INT_REQUEST_FAILURE;
        wake_up_all(&slave->master->request_queue);
        fsm->state = ec_fsm_slave_state_idle;
        return 0;
    }

    if (slave->current_state == EC_SLAVE_STATE_INIT)
    {
        EC_SLAVE_WARN(slave, "放弃EoE请求，从站处于INIT状态。\n");
        request->state = EC_INT_REQUEST_FAILURE;
        wake_up_all(&slave->master->request_queue);
        fsm->state = ec_fsm_slave_state_idle;
        return 0;
    }

    fsm->eoe_request = request;
    request->state = EC_INT_REQUEST_BUSY;

    // 找到待处理的请求，执行它！
    EC_SLAVE_DBG(slave, 1, "处理EoE请求...\n");

    // 开始EoE命令
    fsm->state = ec_fsm_slave_state_eoe_request;
    ec_fsm_eoe_set_ip_param(&fsm->fsm_eoe, slave, request);
    ec_fsm_eoe_exec(&fsm->fsm_eoe, datagram); // 立即执行
    return 1;
}

/*****************************************************************************/

/**
 * @brief 从站状态: EOE_REQUEST（EoE请求）。
 *
 * @param fsm 从站状态机。
 * @param datagram 要使用的数据报。
 *
 * @details 此函数用于处理从站处于EOE_REQUEST状态时的操作。它执行EoE请求的处理，
 * 并根据处理结果更新请求状态。如果EoE执行成功，则将请求状态设置为EC_INT_REQUEST_SUCCESS，
 * 并唤醒主站的请求队列。如果EoE执行失败，则将请求状态设置为EC_INT_REQUEST_FAILURE，
 * 并唤醒主站的请求队列。最后，将EoE请求和从站状态设置为NULL，并将从站状态设置为准备就绪状态。
 */
void ec_fsm_slave_state_eoe_request(
    ec_fsm_slave_t *fsm,    /**< 从站状态机 */
    ec_datagram_t *datagram /**< 要使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;
    ec_eoe_request_t *req = fsm->eoe_request;

    if (ec_fsm_eoe_exec(&fsm->fsm_eoe, datagram))
    {
        return;
    }

    if (ec_fsm_eoe_success(&fsm->fsm_eoe))
    {
        req->state = EC_INT_REQUEST_SUCCESS;
        EC_SLAVE_DBG(slave, 1, "EoE请求处理完成。\n");
    }
    else
    {
        req->state = EC_INT_REQUEST_FAILURE;
        EC_SLAVE_ERR(slave, "处理EoE请求失败。\n");
    }

    wake_up_all(&slave->master->request_queue);
    fsm->eoe_request = NULL;
    fsm->state = ec_fsm_slave_state_ready;
}
#endif

/*****************************************************************************/

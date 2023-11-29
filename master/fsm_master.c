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
 * EtherCAT master state machine.
 */

/*****************************************************************************/

#include "globals.h"
#include "master.h"
#include "mailbox.h"
#include "slave_config.h"
#ifdef EC_EOE
#include "ethernet.h"
#endif

#include "fsm_master.h"
#include "fsm_foe.h"

/*****************************************************************************/

/** Time difference [ns] to tolerate without setting a new system time offset.
 */
#define EC_SYSTEM_TIME_TOLERANCE_NS 1000

/*****************************************************************************/

void ec_fsm_master_state_start(ec_fsm_master_t *);
void ec_fsm_master_state_broadcast(ec_fsm_master_t *);
void ec_fsm_master_state_read_al_status(ec_fsm_master_t *);
#ifdef EC_LOOP_CONTROL
void ec_fsm_master_state_read_dl_status(ec_fsm_master_t *);
void ec_fsm_master_state_open_port(ec_fsm_master_t *);
#endif
void ec_fsm_master_state_dc_read_old_times(ec_fsm_master_t *);
void ec_fsm_master_state_clear_addresses(ec_fsm_master_t *);
#ifdef EC_LOOP_CONTROL
void ec_fsm_master_state_loop_control(ec_fsm_master_t *);
#endif
void ec_fsm_master_state_dc_measure_delays(ec_fsm_master_t *);
void ec_fsm_master_state_scan_slave(ec_fsm_master_t *);
void ec_fsm_master_state_dc_read_offset(ec_fsm_master_t *);
void ec_fsm_master_state_dc_write_offset(ec_fsm_master_t *);
void ec_fsm_master_state_dc_reset_filter(ec_fsm_master_t *);
void ec_fsm_master_state_write_sii(ec_fsm_master_t *);
void ec_fsm_master_state_reboot_slave(ec_fsm_master_t *);

void ec_fsm_master_enter_dc_read_old_times(ec_fsm_master_t *);
void ec_fsm_master_enter_clear_addresses(ec_fsm_master_t *);
void ec_fsm_master_enter_write_system_times(ec_fsm_master_t *);

/*****************************************************************************/

/**
 * @brief     构造函数。
 * @details   这是一个构造函数，用于初始化EtherCAT主站状态机。
 * @param     fsm 主站状态机指针。
 * @param     master EtherCAT主站指针。
 * @param     datagram 用于通信的数据报对象。
 * @retval    无返回值。
 */
void ec_fsm_master_init(
    ec_fsm_master_t *fsm,   /**< 主站状态机指针。 */
    ec_master_t *master,    /**< EtherCAT主站指针。 */
    ec_datagram_t *datagram /**< 用于通信的数据报对象。 */
)
{
    fsm->master = master;
    fsm->datagram = datagram;

    ec_fsm_master_reset(fsm);

    // 初始化子状态机
    ec_fsm_reboot_init(&fsm->fsm_reboot, fsm->datagram);
    ec_fsm_sii_init(&fsm->fsm_sii);
}


/*****************************************************************************/

/**
 * @brief     析构函数。
 * @details   这是一个析构函数，用于清理EtherCAT主站状态机。
 * @param     fsm 主站状态机指针。
 * @retval    无返回值。
 */
void ec_fsm_master_clear(
    ec_fsm_master_t *fsm /**< 主站状态机指针。 */
)
{
    // 清理子状态机
    ec_fsm_reboot_clear(&fsm->fsm_reboot);
    ec_fsm_sii_clear(&fsm->fsm_sii);
}


/*****************************************************************************/

/**
 * @brief     重置状态机。
 * @details   这是一个重置状态机的函数，用于将状态机恢复到初始状态。
 * @param     fsm 主站状态机指针。
 * @retval    无返回值。
 * @作用      将状态机的各个变量重置为初始值。
 */
void ec_fsm_master_reset(
    ec_fsm_master_t *fsm /**< 主站状态机指针。 */
)
{
    ec_device_index_t dev_idx;

    fsm->state = ec_fsm_master_state_start;
    fsm->idle = 0;
    fsm->dev_idx = EC_DEVICE_MAIN;

    for (dev_idx = EC_DEVICE_MAIN;
         dev_idx < ec_master_num_devices(fsm->master); dev_idx++)
    {
        fsm->link_state[dev_idx] = 0;
        fsm->slaves_responding[dev_idx] = 0;
        fsm->slave_states[dev_idx] = EC_SLAVE_STATE_UNKNOWN;
    }

    fsm->rescan_required = 0;
}

/*****************************************************************************/

/**
 * @brief     执行当前状态机的状态。
 * @details   如果状态机的数据报尚未发送或接收，状态机的执行将延迟到下一个周期。
 * @param     fsm 主站状态机指针。
 * @retval    返回值为1，表示状态机已执行；返回值为0，表示状态机未执行。
 * @作用      执行当前状态机的状态，并检查数据报的发送和接收状态。
 */
int ec_fsm_master_exec(
    ec_fsm_master_t *fsm /**< 主站状态机指针。 */
)
{
    if (fsm->datagram->state == EC_DATAGRAM_SENT || fsm->datagram->state == EC_DATAGRAM_QUEUED)
    {
        // 数据报尚未发送或接收。
        return 0;
    }

    fsm->state(fsm);
    return 1;
}

/*****************************************************************************/

/**
 * @brief     判断状态机是否处于空闲阶段。
 * @param     fsm 主站状态机指针。
 * @retval    返回值为1，表示状态机处于空闲阶段；返回值为0，表示状态机不处于空闲阶段。
 * @作用      判断状态机是否处于空闲阶段。
 */
int ec_fsm_master_idle(
    const ec_fsm_master_t *fsm /**< 主站状态机指针。 */
)
{
    return fsm->idle;
}

/*****************************************************************************/

/**
 * @brief     重启主站状态机。
 * @param     fsm 主站状态机指针。
 * @retval    无返回值。
 * @作用      重启主站状态机，并将状态设置为起始状态。
 */
void ec_fsm_master_restart(
    ec_fsm_master_t *fsm /**< 主站状态机指针。 */
)
{
    fsm->dev_idx = EC_DEVICE_MAIN;
    fsm->state = ec_fsm_master_state_start;
    fsm->state(fsm); // 立即执行
}
/******************************************************************************
 * Master state machine
 *****************************************************************************/

/**
 * @brief     主站状态：开始。
 * @功能      开始时获取从站数量和从站状态。
 * @param     fsm 主站状态机指针。
 * @retval    无返回值。
 * @作用      主站状态机的起始状态，用于获取从站数量和从站状态。
 */
void ec_fsm_master_state_start(
    ec_fsm_master_t *fsm /**< 主站状态机指针。 */
)
{
    ec_master_t *master = fsm->master;

    fsm->idle = 1;

    // 检查紧急请求
    if (!list_empty(&master->emerg_reg_requests))
    {
        ec_reg_request_t *request;

        // 获取第一个请求
        request = list_entry(master->emerg_reg_requests.next,
                             ec_reg_request_t, list);
        list_del_init(&request->list); // 出队
        request->state = EC_INT_REQUEST_BUSY;

        if (request->transfer_size > fsm->datagram->mem_size)
        {
            EC_MASTER_ERR(master, "紧急请求数据太大！\n");
            request->state = EC_INT_REQUEST_FAILURE;
            wake_up_all(&master->request_queue);
            fsm->state(fsm); // 继续执行
            return;
        }

        if (request->dir != EC_DIR_OUTPUT)
        {
            EC_MASTER_ERR(master, "紧急请求必须是写请求！\n");
            request->state = EC_INT_REQUEST_FAILURE;
            wake_up_all(&master->request_queue);
            fsm->state(fsm); // 继续执行
            return;
        }

        EC_MASTER_DBG(master, 1, "写入紧急寄存器请求...\n");
        ec_datagram_apwr(fsm->datagram, request->ring_position,
                         request->address, request->transfer_size);
        memcpy(fsm->datagram->data, request->data, request->transfer_size);
        fsm->datagram->device_index = EC_DEVICE_MAIN;
        request->state = EC_INT_REQUEST_SUCCESS;
        wake_up_all(&master->request_queue);
        return;
    }

    // 检查已分离的配置请求
    ec_master_expire_slave_config_requests(fsm->master);

    if (master->reboot)
    {
        // 请求重启所有从站
        master->reboot = 0;
        fsm->idle = 0;
        fsm->state = ec_fsm_master_state_reboot_slave;
        fsm->slave = NULL;
        ec_fsm_reboot_all(&fsm->fsm_reboot, master);
        fsm->state(fsm); // 立即执行
        return;
    }

    ec_datagram_brd(fsm->datagram, 0x0130, 2);
    ec_datagram_zero(fsm->datagram);
    fsm->datagram->device_index = fsm->dev_idx;
    fsm->state = ec_fsm_master_state_broadcast;
}

/*****************************************************************************/

/**
 * @brief     主站状态：广播。
 * @功能      处理广播读取从站数量和从站状态。
 * @param     fsm 主站状态机指针。
 * @retval    无返回值。
 * @作用      主站状态机的广播状态，用于处理广播读取从站数量和从站状态。
 */
void ec_fsm_master_state_broadcast(
    ec_fsm_master_t *fsm /**< 主站状态机指针。 */
)
{
    ec_datagram_t *datagram = fsm->datagram;
    unsigned int i, size;
    ec_slave_t *slave;
    ec_master_t *master = fsm->master;

    // 总线拓扑变化？
    if (datagram->working_counter != fsm->slaves_responding[fsm->dev_idx])
    {
        fsm->rescan_required = 1;
        fsm->slaves_responding[fsm->dev_idx] = datagram->working_counter;
        EC_MASTER_INFO(master, "%u个从站响应在%s设备上。\n",
                       fsm->slaves_responding[fsm->dev_idx],
                       ec_device_names[fsm->dev_idx != 0]);
    }

    if (fsm->link_state[fsm->dev_idx] &&
        !master->devices[fsm->dev_idx].link_state)
    {
        ec_device_index_t dev_idx;

        EC_MASTER_DBG(master, 1, "主站状态机检测到%s设备链路断开。清除从站列表。\n",
                      ec_device_names[fsm->dev_idx != 0]);

        ec_master_slaves_not_available(master);
#ifdef EC_EOE
        ec_master_eoe_stop(master);
        ec_master_clear_eoe_handlers(master, 0);
#endif
        ec_master_clear_slaves(master);
        ec_master_clear_sii_images(master);

        ec_lock_down(&master->config_sem);
        master->config_busy = 0;
        ec_lock_up(&master->config_sem);

        for (dev_idx = EC_DEVICE_MAIN;
             dev_idx < ec_master_num_devices(master); dev_idx++)
        {
            fsm->slave_states[dev_idx] = 0x00;
            fsm->slaves_responding[dev_idx] = 0; /* 重置以在下次链路恢复时触发重新扫描。 */
        }
    }
    fsm->link_state[fsm->dev_idx] = master->devices[fsm->dev_idx].link_state;

    if (datagram->state == EC_DATAGRAM_RECEIVED &&
        fsm->slaves_responding[fsm->dev_idx])
    {
        uint8_t states = EC_READ_U8(datagram->data);
        if (states != fsm->slave_states[fsm->dev_idx])
        {
            // 从站状态发生变化
            char state_str[EC_STATE_STRING_SIZE];
            fsm->slave_states[fsm->dev_idx] = states;
            ec_state_string(states, state_str, 1);
            EC_MASTER_INFO(master, "%s设备上的从站状态：%s。\n",
                           ec_device_names[fsm->dev_idx != 0], state_str);
        }
    }
    else
    {
        fsm->slave_states[fsm->dev_idx] = 0x00;
    }

    fsm->dev_idx++;
    if (fsm->dev_idx < ec_master_num_devices(master))
    {
        // 检查下一个设备上响应的从站数量
        fsm->state = ec_fsm_master_state_start;
        fsm->state(fsm); // 立即执行
        return;
    }

    if (fsm->rescan_required)
    {
        ec_lock_down(&master->scan_sem);
        if (!master->allow_scan)
        {
            ec_lock_up(&master->scan_sem);
        }
        else
        {
            unsigned int count = 0, next_dev_slave, ring_position;
            ec_device_index_t dev_idx;

            master->scan_busy = 1;
            ec_lock_up(&master->scan_sem);

            // 清除所有从站并扫描总线
            fsm->rescan_required = 0;
            fsm->idle = 0;
            fsm->scan_jiffies = jiffies;

            ec_master_slaves_not_available(master);
#ifdef EC_EOE
            ec_master_eoe_stop(master);
            ec_master_clear_eoe_handlers(master, 0);
#endif
            ec_master_clear_slaves(master);
            ec_master_clear_sii_images(master);

            ec_lock_down(&master->config_sem);
            master->config_busy = 0;
            ec_lock_up(&master->config_sem);

            for (dev_idx = EC_DEVICE_MAIN;
                 dev_idx < ec_master_num_devices(master); dev_idx++)
            {
                count += fsm->slaves_responding[dev_idx];
            }

            if (!count)
            {
                // 没有从站存在 -> 结束状态机。
                master->scan_busy = 0;
                wake_up_interruptible(&master->scan_queue);
                ec_fsm_master_restart(fsm);
                return;
            }

            size = sizeof(ec_slave_t) * count;
            if (!(master->slaves =
                      (ec_slave_t *)kmalloc(size, GFP_KERNEL)))
            {
                EC_MASTER_ERR(master, "分配%u字节的从站内存失败！\n",
                              size);
                master->scan_busy = 0;
                wake_up_interruptible(&master->scan_queue);
                ec_fsm_master_restart(fsm);
                return;
            }

            // 初始化从站
            dev_idx = EC_DEVICE_MAIN;
            next_dev_slave = fsm->slaves_responding[dev_idx];
            ring_position = 0;
            for (i = 0; i < count; i++, ring_position++)
            {
                slave = master->slaves + i;
                while (i >= next_dev_slave)
                {
                    dev_idx++;
                    next_dev_slave += fsm->slaves_responding[dev_idx];
                    ring_position = 0;
                }

                ec_slave_init(slave, master, dev_idx, ring_position, i + 1);

                // 在操作阶段不要强制重新配置以避免不必要的过程数据中断
                if (master->phase != EC_OPERATION)
                {
                    slave->force_config = 1;
                }
            }
            master->slave_count = count;
            master->fsm_slave = master->slaves;

            ec_master_slaves_available(master);
            ec_fsm_master_enter_dc_read_old_times(fsm);
            return;
        }
    }

    if (master->slave_count)
    {

        // 应用配置
        if (master->config_changed)
        {
            master->config_changed = 0;
            master->dc_offset_valid = 0;

            EC_MASTER_DBG(master, 1, "配置已更改。\n");

            fsm->slave = master->slaves; // 从第一个从站开始
            ec_fsm_master_enter_write_system_times(fsm);
        }
        else
        {
            // 从第一个从站获取状态
            fsm->slave = master->slaves;
            ec_datagram_fprd(fsm->datagram, fsm->slave->station_address,
                             0x0130, 2);
            ec_datagram_zero(datagram);
            fsm->datagram->device_index = fsm->slave->device_index;
            fsm->retries = EC_FSM_RETRIES;
            fsm->state = ec_fsm_master_state_read_al_status;
        }
    }
    else
    {
        ec_fsm_master_restart(fsm);
    }
}

/*****************************************************************************/

/**
 * @brief     检查是否有待处理的SII写请求并处理其中一个。
 * @功能      该函数用于检查是否存在待处理的SII写请求，并处理其中一个请求。
 * @details   这是一个用于检查是否有待处理的SII写请求，并处理其中一个请求的函数。
 * @param     fsm 主站状态机指针。
 * @retval    非零值，如果处理了一个SII写请求。
 */
int ec_fsm_master_action_process_sii(
    ec_fsm_master_t *fsm /**< 主站状态机。 */
)
{
    ec_master_t *master = fsm->master;
    ec_sii_write_request_t *request;

    // 搜索要处理的第一个请求
    while (1)
    {
        if (list_empty(&master->sii_requests))
            break;

        // 获取第一个请求
        request = list_entry(master->sii_requests.next,
                             ec_sii_write_request_t, list);
        list_del_init(&request->list); // 出队
        request->state = EC_INT_REQUEST_BUSY;

        // 找到待处理的SII写操作。执行它！
        EC_SLAVE_DBG(request->slave, 1, "正在写入SII数据...\n");
        fsm->sii_request = request;
        fsm->sii_index = 0;
        ec_fsm_sii_write(&fsm->fsm_sii, request->slave, request->offset,
                         request->words, EC_FSM_SII_USE_CONFIGURED_ADDRESS);
        fsm->state = ec_fsm_master_state_write_sii;
        fsm->state(fsm); // 立即执行
        return 1;
    }

    return 0;
}

/*****************************************************************************/

/**
 * @brief     主站动作：空闲状态。
 * @功能      执行次要工作。
 * @details   这是一个执行次要工作的函数。
 * @param     fsm 主站状态机指针。
 * @retval    无返回值。
 */
void ec_fsm_master_action_idle(
    ec_fsm_master_t *fsm /**< 主站状态机。 */
)
{
    // 检查是否有待处理的SII写操作。
    if (ec_fsm_master_action_process_sii(fsm))
    {
        return; // 发现SII写请求
    }

    ec_fsm_master_restart(fsm);
}

/*****************************************************************************/

/**
 * @brief     主站动作：获取下一个从站的状态。
 * @功能      获取下一个从站的状态。
 * @details   这是一个用于获取下一个从站状态的函数。
 * @param     fsm 主站状态机指针。
 * @retval    无返回值。
 */
void ec_fsm_master_action_next_slave_state(
    ec_fsm_master_t *fsm /**< 主站状态机。 */
)
{
    ec_master_t *master = fsm->master;

    // 是否还有其他从站需要查询？
    fsm->slave++;
    if (fsm->slave < master->slaves + master->slave_count)
    {
        // 从下一个从站获取状态
        fsm->idle = 1;
        ec_datagram_fprd(fsm->datagram,
                         fsm->slave->station_address, 0x0130, 2);
        ec_datagram_zero(fsm->datagram);
        fsm->datagram->device_index = fsm->slave->device_index;
        fsm->retries = EC_FSM_RETRIES;
        fsm->state = ec_fsm_master_state_read_al_status;
        return;
    }

    // 所有从站已处理完毕
    ec_fsm_master_action_idle(fsm);
}
/*****************************************************************************/

#ifdef EC_LOOP_CONTROL

/**
 * @brief     主站动作：读取当前从站的DL状态。
 * @功能      读取当前从站的DL状态。
 * @details   这是一个用于读取当前从站DL状态的函数。
 * @param     fsm 主站状态机指针。
 * @retval    无返回值。
 */
void ec_fsm_master_action_read_dl_status(
    ec_fsm_master_t *fsm /**< 主站状态机。 */
)
{
    ec_datagram_fprd(fsm->datagram, fsm->slave->station_address, 0x0110, 2);
    ec_datagram_zero(fsm->datagram);
    fsm->datagram->device_index = fsm->slave->device_index;
    fsm->retries = EC_FSM_RETRIES;
    fsm->state = ec_fsm_master_state_read_dl_status;
}

/*****************************************************************************/

/**
 * @brief     主站动作：打开从站端口。
 * @功能      打开从站端口。
 * @details   这是一个用于打开从站端口的函数。
 * @param     fsm 主站状态机指针。
 * @retval    无返回值。
 */
void ec_fsm_master_action_open_port(
    ec_fsm_master_t *fsm /**< 主站状态机。 */
)
{
    EC_SLAVE_INFO(fsm->slave, "正在打开端口。\n");

    ec_datagram_fpwr(fsm->datagram, fsm->slave->station_address, 0x0101, 1);
    EC_WRITE_U8(fsm->datagram->data, 0x54); // 端口0自动，1-3自动关闭
    fsm->datagram->device_index = fsm->slave->device_index;
    fsm->retries = EC_FSM_RETRIES;
    fsm->state = ec_fsm_master_state_open_port;
}

/*****************************************************************************/

/**
 * @brief     主站状态：读取DL状态。
 * @功能      获取从站的DL状态。
 * @details   这是一个用于获取从站DL状态的函数。
 * @param     fsm 主站状态机指针。
 * @retval    无返回值。
 */
void ec_fsm_master_state_read_dl_status(
    ec_fsm_master_t *fsm /**< 主站状态机。 */
)
{
    ec_slave_t *slave = fsm->slave;
    ec_datagram_t *datagram = fsm->datagram;
    unsigned int i;

    if (datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--)
    {
        return;
    }

    if (datagram->state != EC_DATAGRAM_RECEIVED)
    {
        EC_SLAVE_ERR(slave, "无法接收到AL状态数据报文：");
        ec_datagram_print_state(datagram);
        ec_fsm_master_restart(fsm);
        return;
    }

    // 从站是否未响应其站地址？
    if (datagram->working_counter != 1)
    {
        // 下次再试
        ec_fsm_master_action_next_slave_state(fsm);
        return;
    }

    ec_slave_set_dl_status(slave, EC_READ_U16(datagram->data));

    // 处理端口状态机
    for (i = 0; i < EC_MAX_PORTS; i++)
    {
        ec_slave_port_t *port = &slave->ports[i];

        switch (port->state)
        {
        case EC_SLAVE_PORT_DOWN:
            if (port->link.loop_closed)
            {
                if (port->link.link_up)
                {
                    port->link_detection_jiffies = jiffies;
                    port->state = EC_SLAVE_PORT_WAIT;
                }
            }
            else
            { // 环路打开
                port->state = EC_SLAVE_PORT_UP;
            }
            break;
        case EC_SLAVE_PORT_WAIT:
            if (port->link.link_up)
            {
                if (jiffies - port->link_detection_jiffies >
                    HZ * EC_PORT_WAIT_MS / 1000)
                {
                    port->state = EC_SLAVE_PORT_UP;
                    ec_fsm_master_action_open_port(fsm);
                    return;
                }
            }
            else
            { // 链路断开
                port->state = EC_SLAVE_PORT_DOWN;
            }
            break;
        default: // EC_SLAVE_PORT_UP
            if (!port->link.link_up)
            {
                port->state = EC_SLAVE_PORT_DOWN;
            }
            break;
        }
    }

    // 处理下一个从站
    ec_fsm_master_action_next_slave_state(fsm);
}

/*****************************************************************************/

/**
 * @brief     主站状态：打开端口。
 * @功能      打开从站端口。
 * @details   这是一个用于打开从站端口的函数。
 * @param     fsm 主站状态机指针。
 * @retval    无返回值。
 */
void ec_fsm_master_state_open_port(
    ec_fsm_master_t *fsm /**< 主站状态机。 */
)
{
    ec_slave_t *slave = fsm->slave;
    ec_datagram_t *datagram = fsm->datagram;

    if (datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--)
    {
        return;
    }

    if (datagram->state != EC_DATAGRAM_RECEIVED)
    {
        EC_SLAVE_ERR(slave, "无法接收到打开端口数据报文：");
        ec_datagram_print_state(datagram);
        ec_fsm_master_restart(fsm);
        return;
    }

    // 从站是否未响应其站地址？
    if (datagram->working_counter != 1)
    {
        EC_SLAVE_ERR(slave, "未响应打开端口命令！\n");
        return;
    }

    // 处理下一个从站
    ec_fsm_master_action_next_slave_state(fsm);
}

#endif

/*****************************************************************************/

/**
 * @brief     主站动作：配置。
 * @功能      配置主站。
 * @details   这是一个用于配置主站的函数。
 * @param     fsm 主站状态机指针。
 * @retval    无返回值。
 */
void ec_fsm_master_action_configure(
    ec_fsm_master_t *fsm /**< 主站状态机。 */
)
{
    ec_master_t *master = fsm->master;

    if (master->config_changed)
    {
        master->config_changed = 0;
        master->dc_offset_valid = 0;

        // 中止遍历从站，
        // 首先补偿DC系统时间偏移，
        // 然后从从站0开始配置
        EC_MASTER_DBG(master, 1, "配置已更改（中止状态检查）。\n");

        fsm->slave = master->slaves; // 从第一个从站开始
        ec_fsm_master_enter_write_system_times(fsm);
        return;
    }

    // 允许从站开始配置（如果尚未完成）。
    ec_fsm_slave_set_ready(&fsm->slave->fsm);

#ifdef EC_LOOP_CONTROL
    // 读取DL状态
    ec_fsm_master_action_read_dl_status(fsm);
#else
    // 处理下一个从站
    ec_fsm_master_action_next_slave_state(fsm);
#endif
}

/*****************************************************************************/

/**
 * @brief     主站状态：读取AL状态。
 * @功能      获取从站的AL状态。
 * @details   这是一个用于读取从站AL状态的函数。
 * @param     fsm 主站状态机指针。
 * @retval    无返回值。
 */
void ec_fsm_master_state_read_al_status(
    ec_fsm_master_t *fsm /**< 主站状态机。 */
)
{
    ec_slave_t *slave = fsm->slave;
    ec_datagram_t *datagram = fsm->datagram;

    if (datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--)
    {
        return;
    }

    if (datagram->state != EC_DATAGRAM_RECEIVED)
    {
        EC_SLAVE_ERR(slave, "无法接收AL状态数据报：");
        ec_datagram_print_state(datagram);
        ec_fsm_master_restart(fsm);
        return;
    }

    // 从站是否未响应其站地址？
    if (datagram->working_counter != 1)
    {
        if (!slave->error_flag)
        {
            slave->error_flag = 1;
            EC_SLAVE_DBG(slave, 1, "从站未响应状态查询。\n");
        }
        fsm->rescan_required = 1;
        ec_fsm_master_restart(fsm);
        return;
    }

    // 单个从站响应
    ec_slave_set_al_status(slave, EC_READ_U8(datagram->data));

    if (slave->reboot)
    {
        // 请求重新启动该从站
        slave->reboot = 0;
        fsm->idle = 0;
        fsm->state = ec_fsm_master_state_reboot_slave;
        ec_fsm_reboot_single(&fsm->fsm_reboot, slave);
        fsm->state(fsm); // 立即执行
        return;
    }

    if (!slave->error_flag)
    {
        // 检查配置
        ec_fsm_master_action_configure(fsm);
        return;
    }

#ifdef EC_LOOP_CONTROL
    // 读取DL状态
    ec_fsm_master_action_read_dl_status(fsm);
#else
    // 处理下一个从站
    ec_fsm_master_action_next_slave_state(fsm);
#endif
}

/*****************************************************************************/

/**
 * @brief     主站状态：重新启动从站。
 * @功能      重新启动从站。
 * @details   这是一个用于重新启动从站的函数。
 * @param     fsm 主站状态机指针。
 * @retval    无返回值。
 */
void ec_fsm_master_state_reboot_slave(
    ec_fsm_master_t *fsm /**< 主站状态机。 */
)
{
    ec_slave_t *slave = fsm->slave;

    if (ec_fsm_reboot_exec(&fsm->fsm_reboot))
    {
        return;
    }

    if (!ec_fsm_reboot_success(&fsm->fsm_reboot))
    {
        if (slave)
        {
            EC_SLAVE_ERR(slave, "重新启动失败。\n");
        }
        else
        {
            EC_MASTER_ERR(fsm->master, "重新启动失败。\n");
        }
    }

    ec_fsm_master_restart(fsm);
}

/*****************************************************************************/

/**
 * @brief     主站状态：开始读取从站的旧时间戳。
 * @功能      开始读取从站的旧时间戳。
 * @details   这是一个用于开始读取从站旧时间戳的函数。
 * @param     fsm 主站状态机指针。
 * @retval    无返回值。
 */
void ec_fsm_master_enter_dc_read_old_times(
    ec_fsm_master_t *fsm /**< 主站状态机。 */
)
{
    EC_MASTER_DBG(fsm->master, 1, "正在读取旧端口接收时间...\n");

    // 读取DC端口接收时间
    // （站地址尚未分配，因此必须APRD。）
    fsm->slave = fsm->master->slaves;
    ec_datagram_aprd(fsm->datagram, fsm->slave->ring_position, 0x0900, 16);
    ec_datagram_zero(fsm->datagram);
    fsm->datagram->device_index = fsm->slave->device_index;
    fsm->retries = EC_FSM_RETRIES;
    fsm->state = ec_fsm_master_state_dc_read_old_times;
}

/*****************************************************************************/

/**
 * @brief     主站状态：读取旧时间戳。
 * @功能      读取旧时间戳。
 * @details   这是一个用于读取旧时间戳的主站状态函数。
 * @param     fsm 主站状态机指针。
 * @retval    无返回值。
 */
void ec_fsm_master_state_dc_read_old_times(
    ec_fsm_master_t *fsm /**< 主站状态机。 */
)
{
    ec_master_t *master = fsm->master;
    ec_slave_t *slave = fsm->slave;
    ec_datagram_t *datagram = fsm->datagram;
    int i;

    if (datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--)
        return;

    if (datagram->state != EC_DATAGRAM_RECEIVED)
    {
        EC_SLAVE_ERR(slave, "无法接收到DC接收时间数据报文：");
        ec_datagram_print_state(datagram);
        // 即使出现错误也继续执行
    }
    else if (datagram->working_counter != 1)
    {
        EC_SLAVE_WARN(slave, "无法获取DC接收时间：");
        ec_datagram_print_wc_error(datagram);
        // 继续执行；这只是一个警告，因为在这一点上我们不知道从站是否支持这些寄存器
    }

    for (i = 0; i < EC_MAX_PORTS; i++)
    {
        slave->ports[i].receive_time = EC_READ_U32(datagram->data + 4 * i);
    }

    ++fsm->slave;
    if (fsm->slave < master->slaves + master->slave_count)
    {
        // 读取DC端口接收时间
        ec_datagram_aprd(datagram, fsm->slave->ring_position, 0x0900, 16);
        ec_datagram_zero(datagram);
        datagram->device_index = fsm->slave->device_index;
        fsm->retries = EC_FSM_RETRIES;
    }
    else
    {
        /* 从第一个有响应的从站开始；至少有一个从站有响应，否则计数将为零。 */
        fsm->dev_idx = EC_DEVICE_MAIN;
        while (!fsm->slaves_responding[fsm->dev_idx])
        {
            fsm->dev_idx++;
        }

        fsm->slave = master->slaves;
        ec_fsm_master_enter_clear_addresses(fsm);
    }
}

/*****************************************************************************/

/**
 * @brief     主站状态：开始清除从站地址。
 * @功能      开始清除从站地址。
 * @details   这是一个用于开始清除从站地址的函数。
 * @param     fsm 主站状态机指针。
 * @retval    无返回值。
 */
void ec_fsm_master_enter_clear_addresses(
    ec_fsm_master_t *fsm /**< 主站状态机。 */
)
{
    // 广播清除所有站地址
    ec_datagram_bwr(fsm->datagram, 0x0010, 2);
    EC_WRITE_U16(fsm->datagram->data, 0x0000);
    fsm->datagram->device_index = fsm->dev_idx;
    fsm->retries = EC_FSM_RETRIES;
    fsm->state = ec_fsm_master_state_clear_addresses;
}
/*****************************************************************************/

/**
 * @brief     主站状态：开始测量DC延迟。
 * @功能      启动测量DC延迟。
 * @details   这是一个用于开始测量DC延迟的函数。
 * @param     fsm 主站状态机指针。
 * @retval    无返回值。
 */
void ec_fsm_master_enter_dc_measure_delays(
    ec_fsm_master_t *fsm /**< 主站状态机。 */
)
{
    EC_MASTER_DBG(fsm->master, 1, "发送广播写入以测量%s链路上的传输延迟。\n",
                  ec_device_names[fsm->dev_idx != 0]);

    ec_datagram_bwr(fsm->datagram, 0x0900, 1);
    ec_datagram_zero(fsm->datagram);
    fsm->datagram->device_index = fsm->dev_idx;
    fsm->retries = EC_FSM_RETRIES;
    fsm->state = ec_fsm_master_state_dc_measure_delays;
}

/*****************************************************************************/

#ifdef EC_LOOP_CONTROL

/**
 * @brief     主站状态：开始写入环路控制寄存器。
 * @功能      启动写入环路控制寄存器。
 * @details   这是一个用于开始写入环路控制寄存器的函数。
 * @param     fsm 主站状态机指针。
 * @retval    无返回值。
 */
void ec_fsm_master_enter_loop_control(
    ec_fsm_master_t *fsm /**< 主站状态机。 */
)
{
    EC_MASTER_DBG(fsm->master, 1, "在%s链路上广播写入环路控制寄存器。\n",
                  ec_device_names[fsm->dev_idx != 0]);

    ec_datagram_bwr(fsm->datagram, 0x0101, 1);
    EC_WRITE_U8(fsm->datagram->data, 0x54); // 端口0自动，1-3自动关闭
    fsm->datagram->device_index = fsm->dev_idx;
    fsm->retries = EC_FSM_RETRIES;
    fsm->state = ec_fsm_master_state_loop_control;
}
/*****************************************************************************/

/**
 * @brief     主站状态：环路控制。
 * @功能      执行环路控制状态。
 * @details   这是一个执行环路控制状态的函数。
 * @param     fsm 主站状态机指针。
 * @retval    无返回值。
 */
void ec_fsm_master_state_loop_control(
    ec_fsm_master_t *fsm /**< 主站状态机。 */
)
{
    ec_master_t *master = fsm->master;
    ec_datagram_t *datagram = fsm->datagram;

    if (datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--)
    {
        return;
    }

    if (datagram->state != EC_DATAGRAM_RECEIVED)
    {
        EC_MASTER_ERR(master, "在%s链路上接收环路控制数据报失败：",
                      ec_device_names[fsm->dev_idx != 0]);
        ec_datagram_print_state(datagram);
    }

    ec_fsm_master_enter_dc_measure_delays(fsm);
}

#endif

/*****************************************************************************/

/**
 * @brief     主站状态：清除地址。
 * @功能      执行清除地址状态。
 * @details   这是一个执行清除地址状态的函数。
 * @param     fsm 主站状态机指针。
 * @retval    无返回值。
 */
void ec_fsm_master_state_clear_addresses(
    ec_fsm_master_t *fsm /**< 主站状态机。 */
)
{
    ec_master_t *master = fsm->master;
    ec_datagram_t *datagram = fsm->datagram;

    if (datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--)
    {
        return;
    }

    if (datagram->state != EC_DATAGRAM_RECEIVED)
    {
        EC_MASTER_ERR(master, "在%s链路上接收地址清除数据报失败：",
                      ec_device_names[fsm->dev_idx != 0]);
        ec_datagram_print_state(datagram);
        master->scan_busy = 0;
        wake_up_interruptible(&master->scan_queue);
        ec_fsm_master_restart(fsm);
        return;
    }

    if (datagram->working_counter != fsm->slaves_responding[fsm->dev_idx])
    {
        EC_MASTER_WARN(master, "在%s链路上清除站点地址失败：已清除 %u 个站点中的 %u 个",
                       ec_device_names[fsm->dev_idx != 0], datagram->working_counter,
                       fsm->slaves_responding[fsm->dev_idx]);
    }

#ifdef EC_LOOP_CONTROL
    ec_fsm_master_enter_loop_control(fsm);
#else
    ec_fsm_master_enter_dc_measure_delays(fsm);
#endif
}

/*****************************************************************************/

/**
 * @brief     主站状态：测量延迟。
 * @功能      执行测量延迟状态。
 * @details   这是一个执行测量延迟状态的函数。
 * @param     fsm 主站状态机指针。
 * @retval    无返回值。
 */
void ec_fsm_master_state_dc_measure_delays(
    ec_fsm_master_t *fsm /**< 主站状态机。 */
)
{
    ec_master_t *master = fsm->master;
    ec_datagram_t *datagram = fsm->datagram;
    ec_slave_t *slave;

    if (datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--)
    {
        return;
    }

    if (datagram->state != EC_DATAGRAM_RECEIVED)
    {
        EC_MASTER_ERR(master, "在%s链路上接收延迟测量数据报失败：",
                      ec_device_names[fsm->dev_idx != 0]);
        ec_datagram_print_state(datagram);
        master->scan_busy = 0;
        wake_up_interruptible(&master->scan_queue);
        ec_fsm_master_restart(fsm);
        return;
    }

    EC_MASTER_DBG(master, 1, "%u 个站点响应了在%s链路上的延迟测量。\n",
                  datagram->working_counter, ec_device_names[fsm->dev_idx != 0]);

    do
    {
        fsm->dev_idx++;
    } while (fsm->dev_idx < ec_master_num_devices(master) &&
             !fsm->slaves_responding[fsm->dev_idx]);
    if (fsm->dev_idx < ec_master_num_devices(master))
    {
        ec_fsm_master_enter_clear_addresses(fsm);
        return;
    }

    EC_MASTER_INFO(master, "正在扫描总线。\n");

    // 设置站点准备好接收请求（开始扫描）。
    for (slave = master->slaves;
         slave < master->slaves + master->slave_count;
         slave++)
    {
        ec_fsm_slave_set_ready(&slave->fsm);
    }

    fsm->state = ec_fsm_master_state_scan_slave;
    fsm->datagram->state = EC_DATAGRAM_INVALID; // 没有要发送的数据报
    fsm->state(fsm);                            // 立即执行
}

/*****************************************************************************/

/**
 * @brief     主站状态：扫描从站。
 * @功能      等待从站扫描完成。
 * @details   这是一个等待从站扫描完成的函数。
 * @param     fsm 主站状态机指针。
 * @retval    无返回值。
 */
void ec_fsm_master_state_scan_slave(
    ec_fsm_master_t *fsm /**< 主站状态机。 */
)
{
    ec_master_t *master = fsm->master;
    ec_slave_t *slave;

    for (slave = master->slaves;
         slave < master->slaves + master->slave_count;
         slave++)
    {
        if (slave->scan_required && !slave->error_flag)
        {
            // 仍在进行中
            return;
        }
    }

    EC_MASTER_INFO(master, "总线扫描完成，耗时 %lu 毫秒。\n",
                   (jiffies - fsm->scan_jiffies) * 1000 / HZ);

    master->scan_busy = 0;
    wake_up_interruptible(&master->scan_queue);

    // 附加从站配置
    ec_master_attach_slave_configs(master);

    // 设置DC参考从站并计算拓扑和传输延迟
    // 注意：必须在attach_slave_configs之后进行，以便应用程序选择的dc_ref_config返回其从站
    ec_master_calc_dc(master);

#ifdef EC_EOE
    // 检查是否需要启动EoE处理
    ec_master_eoe_start(master);
#endif

    if (master->slave_count)
    {
        master->config_changed = 0;
        master->dc_offset_valid = 0;

        fsm->slave = master->slaves; // 从第一个从站开始
        ec_fsm_master_enter_write_system_times(fsm);
    }
    else
    {
        ec_fsm_master_restart(fsm);
    }
}

/*****************************************************************************/

/**
 * @brief     开始写入DC系统时间。
 * @功能      详细描述该函数的功能，完成了什么任务。
 * @details   这是开始写入DC系统时间的函数。
 * @param     fsm 主站状态机指针。
 * @retval    无返回值。
 */
void ec_fsm_master_enter_write_system_times(
    ec_fsm_master_t *fsm /**< 主站状态机。 */
)
{
    ec_master_t *master = fsm->master;

    if (master->active)
    {

        while (fsm->slave < master->slaves + master->slave_count)
        {
            if (!fsm->slave->base_dc_supported || !fsm->slave->has_dc_system_time)
            {
                fsm->slave++;
                continue;
            }

            EC_SLAVE_DBG(fsm->slave, 1, "检查系统时间偏移。\n");

            // 读取DC系统时间（0x0910，64位）
            //                     间隔（64位）
            //     和时间偏移（0x0920，64位）
            //   和接收延迟（0x0928，32位）
            ec_datagram_fprd(fsm->datagram, fsm->slave->station_address,
                             0x0910, 28);
            ec_datagram_zero(fsm->datagram);
            fsm->datagram->device_index = fsm->slave->device_index;
            fsm->retries = EC_FSM_RETRIES;
            fsm->state = ec_fsm_master_state_dc_read_offset;
            return;
        }
        master->dc_offset_valid = 1;
    }
    else
    {
        EC_MASTER_DBG(master, 1, "到目前为止没有收到app_time。\n");
    }

    // 扫描和设置系统时间完成
    ec_master_request_op(master);
    ec_fsm_master_restart(fsm);
}

/*****************************************************************************/

/**
 * @brief     配置32位时间偏移量。
 * @功能      详细描述该函数的功能，完成了什么任务。
 * @details   这是配置32位时间偏移量的函数。
 * @param     fsm 主站状态机指针。
 * @param     system_time 系统时间寄存器。
 * @param     old_offset 时间偏移寄存器。
 * @param     app_time_sent 通过读取得到的主站应用时间。
 * @retval    新的偏移量。
 */
u64 ec_fsm_master_dc_offset32(
    ec_fsm_master_t *fsm, /**< 主站状态机。 */
    u64 system_time,      /**< 系统时间寄存器。 */
    u64 old_offset,       /**< 时间偏移寄存器。 */
    u64 app_time_sent     /**< 通过读取得到的主站应用时间。 */
)
{
    ec_slave_t *slave = fsm->slave;
    u32 system_time32, old_offset32, new_offset;
    s32 time_diff;

    system_time32 = (u32)system_time;
    old_offset32 = (u32)old_offset;

    time_diff = (u32)app_time_sent - system_time32;

    EC_SLAVE_DBG(slave, 1, "DC 32位系统时间偏移计算："
                           "system_time=%u, app_time=%llu, diff=%i\n",
                 system_time32, app_time_sent, time_diff);

    if (EC_ABS(time_diff) > EC_SYSTEM_TIME_TOLERANCE_NS)
    {
        new_offset = time_diff + old_offset32;
        EC_SLAVE_DBG(slave, 1, "将时间偏移设置为 %u（之前为 %u）\n",
                     new_offset, old_offset32);
        return (u64)new_offset;
    }
    else
    {
        EC_SLAVE_DBG(slave, 1, "不修改时间偏移。\n");
        return old_offset;
    }
}

/*****************************************************************************/

/**
 * @brief     配置64位时间偏移量。
 * @功能      详细描述该函数的功能，完成了什么任务。
 * @details   这是配置64位时间偏移量的函数。
 * @param     fsm 主站状态机指针。
 * @param     system_time 系统时间寄存器。
 * @param     old_offset 时间偏移寄存器。
 * @param     app_time_sent 通过读取得到的主站应用时间。
 * @retval    新的偏移量。
 */
u64 ec_fsm_master_dc_offset64(
    ec_fsm_master_t *fsm, /**< 主站状态机。 */
    u64 system_time,      /**< 系统时间寄存器。 */
    u64 old_offset,       /**< 时间偏移寄存器。 */
    u64 app_time_sent     /**< 通过读取得到的主站应用时间。 */
)
{
    ec_slave_t *slave = fsm->slave;
    u64 new_offset;
    s64 time_diff;

    time_diff = app_time_sent - system_time;

    EC_SLAVE_DBG(slave, 1, "DC 64位系统时间偏移计算："
                           "system_time=%llu, app_time=%llu, diff=%lli\n",
                 system_time, app_time_sent, time_diff);

    if (EC_ABS(time_diff) > EC_SYSTEM_TIME_TOLERANCE_NS)
    {
        new_offset = time_diff + old_offset;
        EC_SLAVE_DBG(slave, 1, "将时间偏移设置为 %llu（之前为 %llu）\n",
                     new_offset, old_offset);
    }
    else
    {
        new_offset = old_offset;
        EC_SLAVE_DBG(slave, 1, "不修改时间偏移。\n");
    }

    return new_offset;
}

/*****************************************************************************/

/**
 * @brief	主状态：DC读取偏移量。
 * @作用	该函数用于读取并设置从站的DC时间偏移量和传输延迟。
 * @param	fsm 主状态机指针。
 * @retval	无。
 */
void ec_fsm_master_state_dc_read_offset(
    ec_fsm_master_t *fsm /**< 主状态机指针。 */
)
{
    ec_datagram_t *datagram = fsm->datagram;
    ec_slave_t *slave = fsm->slave;
    ec_master_t *master = fsm->master;
    u64 system_time, old_offset, new_offset;
    u32 old_delay;

    if (datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--)
        return;

    if (datagram->state != EC_DATAGRAM_RECEIVED)
    {
        EC_SLAVE_ERR(slave, "无法接收到DC时间数据报文：");
        ec_datagram_print_state(datagram);
        fsm->slave++;
        ec_fsm_master_enter_write_system_times(fsm);
        return;
    }

    if (datagram->working_counter != 1)
    {
        EC_SLAVE_WARN(slave, "无法获取DC时间：");
        ec_datagram_print_wc_error(datagram);
        fsm->slave++;
        ec_fsm_master_enter_write_system_times(fsm);
        return;
    }

    if (unlikely(!master->dc_ref_time))
    {
        EC_MASTER_WARN(master, "未收到应用时间，中止DC时间偏移计算。\n");
        // 扫描和设置系统时间完成
        ec_master_request_op(master);
        ec_fsm_master_restart(fsm);
        return;
    }

    system_time = EC_READ_U64(datagram->data);     // 0x0910
    old_offset = EC_READ_U64(datagram->data + 16); // 0x0920
    old_delay = EC_READ_U32(datagram->data + 24);  // 0x0928

    if (slave->base_dc_range == EC_DC_32)
    {
        new_offset = ec_fsm_master_dc_offset32(fsm,
                                               system_time, old_offset, datagram->app_time_sent);
    }
    else
    {
        new_offset = ec_fsm_master_dc_offset64(fsm,
                                               system_time, old_offset, datagram->app_time_sent);
    }

    if (new_offset != old_offset && slave->current_state >= EC_SLAVE_STATE_SAFEOP)
    {
        // 从站已经处于激活状态；改变系统时间偏移量可能会导致问题。
        // 在这种情况下保持偏移量不变，让正常的周期同步过程逐渐调整到正确的时间。
        EC_SLAVE_DBG(slave, 1, "从站正在运行；忽略DC偏移量的更改。\n");
        new_offset = old_offset;
    }

    if (new_offset == old_offset && slave->transmission_delay == old_delay)
    {
        // 偏移量未改变；跳过写操作以避免可能的问题
        fsm->slave++;
        ec_fsm_master_enter_write_system_times(fsm);
        return;
    }

    // 设置DC系统时间偏移量和传输延迟
    ec_datagram_fpwr(datagram, slave->station_address, 0x0920, 12);
    EC_WRITE_U64(datagram->data, new_offset);
    EC_WRITE_U32(datagram->data + 8, slave->transmission_delay);
    fsm->datagram->device_index = slave->device_index;
    fsm->retries = EC_FSM_RETRIES;
    fsm->state = ec_fsm_master_state_dc_write_offset;
}

/*****************************************************************************/

/**
 * @brief	主站状态：DC写入偏移量。
 * @作用	该函数用于设置从站的DC系统时间偏移量。
 * @param	fsm 主站状态机指针。
 * @retval	无。
 */
void ec_fsm_master_state_dc_write_offset(
    ec_fsm_master_t *fsm /**< 主站状态机指针。 */
)
{
    ec_datagram_t *datagram = fsm->datagram;
    ec_slave_t *slave = fsm->slave;

    if (datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--)
        return;

    if (datagram->state != EC_DATAGRAM_RECEIVED)
    {
        EC_SLAVE_ERR(slave,
                     "无法接收到DC系统时间偏移量数据报文：");
        ec_datagram_print_state(datagram);
        fsm->slave++;
        ec_fsm_master_enter_write_system_times(fsm);
        return;
    }

    if (datagram->working_counter != 1)
    {
        EC_SLAVE_ERR(slave, "无法设置DC系统时间偏移量：");
        ec_datagram_print_wc_error(datagram);
        fsm->slave++;
        ec_fsm_master_enter_write_system_times(fsm);
        return;
    }

    // 在更改偏移量后重置DC滤波器
    if (slave->current_state >= EC_SLAVE_STATE_SAFEOP)
    {
        EC_SLAVE_DBG(slave, 1, "从站正在运行；不重置DC滤波器。\n");
        fsm->slave++;
        ec_fsm_master_enter_write_system_times(fsm);
    }
    else
    {
        ec_datagram_fpwr(datagram, slave->station_address, 0x0930, 2);
        EC_WRITE_U16(datagram->data, 0x1000);
        fsm->datagram->device_index = slave->device_index;
        fsm->retries = EC_FSM_RETRIES;
        fsm->state = ec_fsm_master_state_dc_reset_filter;
    }
}


/*****************************************************************************/

/**
 * @brief	主站状态：DC时钟重置滤波器。
 * @作用	执行主站状态机的DC时钟重置滤波器操作。
 * @param	fsm 主站状态机。
 * @retval	无
 */
void ec_fsm_master_state_dc_reset_filter(
    ec_fsm_master_t *fsm /**< 主站状态机。 */
)
{
    ec_datagram_t *datagram = fsm->datagram;
    ec_slave_t *slave = fsm->slave;

    if (datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--)
        return;

    if (datagram->state != EC_DATAGRAM_RECEIVED)
    {
        EC_SLAVE_ERR(slave,
                     "无法接收到DC时钟重置滤波器数据报文：");
        ec_datagram_print_state(datagram);
        fsm->slave++;
        ec_fsm_master_enter_write_system_times(fsm);
        return;
    }

    if (datagram->working_counter != 1)
    {
        EC_SLAVE_ERR(slave, "无法重置DC滤波器：");
        ec_datagram_print_wc_error(datagram);
        fsm->slave++;
        ec_fsm_master_enter_write_system_times(fsm);
        return;
    }

    fsm->slave++;
    ec_fsm_master_enter_write_system_times(fsm);
}

/*****************************************************************************/

/**
 * @brief	主站状态：写入SII（Slave Information Interface）。
 * @作用	执行主站状态机的写入SII操作。
 * @param	fsm 主站状态机。
 * @retval	无
 */
void ec_fsm_master_state_write_sii(
    ec_fsm_master_t *fsm /**< 主站状态机。 */
)
{
    ec_master_t *master = fsm->master;
    ec_sii_write_request_t *request = fsm->sii_request;
    ec_slave_t *slave = request->slave;

    if (ec_fsm_sii_exec(&fsm->fsm_sii, fsm->datagram))
        return;

    if (!ec_fsm_sii_success(&fsm->fsm_sii))
    {
        EC_SLAVE_ERR(slave, "无法写入SII数据。\n");
        request->state = EC_INT_REQUEST_FAILURE;
        wake_up_all(&master->request_queue);
        ec_fsm_master_restart(fsm);
        return;
    }

    fsm->sii_index++;
    if (fsm->sii_index < request->nwords)
    {
        ec_fsm_sii_write(&fsm->fsm_sii, slave,
                         request->offset + fsm->sii_index,
                         request->words + fsm->sii_index,
                         EC_FSM_SII_USE_CONFIGURED_ADDRESS);
        ec_fsm_sii_exec(&fsm->fsm_sii, fsm->datagram); // 立即执行
        return;
    }

    // SII写入完成
    EC_SLAVE_DBG(slave, 1, "已成功写入 %zu 个SII数据字。\n",
                 request->nwords);

    if (request->offset <= 4 && request->offset + request->nwords > 4)
    {
        // 写入了别名
        if (slave->sii_image)
        {
            slave->sii_image->sii.alias = EC_READ_U16(request->words + 4);
            // TODO: 从寄存器0x0012读取别名
            slave->effective_alias = slave->sii_image->sii.alias;
        }
        else
        {
            EC_SLAVE_WARN(slave, "无法更新有效别名。SII数据不可用。\n");
        }
    }
    // TODO: 评估其他SII内容！

    request->state = EC_INT_REQUEST_SUCCESS;
    wake_up_all(&master->request_queue);

    // 检查是否有另一个SII写入请求
    if (ec_fsm_master_action_process_sii(fsm))
        return; // 处理另一个请求

    ec_fsm_master_restart(fsm);
}

/*****************************************************************************/

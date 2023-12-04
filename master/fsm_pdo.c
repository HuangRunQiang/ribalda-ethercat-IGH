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
 * EtherCAT PDO配置状态机。
 */

/*****************************************************************************/

#include "globals.h"
#include "master.h"
#include "mailbox.h"
#include "slave_config.h"

#include "fsm_pdo.h"

/*****************************************************************************/

void ec_fsm_pdo_read_state_start(ec_fsm_pdo_t *, ec_datagram_t *);
void ec_fsm_pdo_read_state_pdo_count(ec_fsm_pdo_t *, ec_datagram_t *);
void ec_fsm_pdo_read_state_pdo(ec_fsm_pdo_t *, ec_datagram_t *);
void ec_fsm_pdo_read_state_pdo_entries(ec_fsm_pdo_t *, ec_datagram_t *);

void ec_fsm_pdo_read_action_next_sync(ec_fsm_pdo_t *, ec_datagram_t *);
void ec_fsm_pdo_read_action_next_pdo(ec_fsm_pdo_t *, ec_datagram_t *);

void ec_fsm_pdo_conf_state_start(ec_fsm_pdo_t *, ec_datagram_t *);
void ec_fsm_pdo_conf_state_read_mapping(ec_fsm_pdo_t *, ec_datagram_t *);
void ec_fsm_pdo_conf_state_mapping(ec_fsm_pdo_t *, ec_datagram_t *);
void ec_fsm_pdo_conf_state_zero_pdo_count(ec_fsm_pdo_t *, ec_datagram_t *);
void ec_fsm_pdo_conf_state_assign_pdo(ec_fsm_pdo_t *, ec_datagram_t *);
void ec_fsm_pdo_conf_state_set_pdo_count(ec_fsm_pdo_t *, ec_datagram_t *);

void ec_fsm_pdo_conf_action_next_sync(ec_fsm_pdo_t *, ec_datagram_t *);
void ec_fsm_pdo_conf_action_pdo_mapping(ec_fsm_pdo_t *, ec_datagram_t *);
void ec_fsm_pdo_conf_action_check_mapping(ec_fsm_pdo_t *, ec_datagram_t *);
void ec_fsm_pdo_conf_action_next_pdo_mapping(ec_fsm_pdo_t *, ec_datagram_t *);
void ec_fsm_pdo_conf_action_check_assignment(ec_fsm_pdo_t *, ec_datagram_t *);
void ec_fsm_pdo_conf_action_assign_pdo(ec_fsm_pdo_t *, ec_datagram_t *);

void ec_fsm_pdo_state_end(ec_fsm_pdo_t *, ec_datagram_t *);
void ec_fsm_pdo_state_error(ec_fsm_pdo_t *, ec_datagram_t *);

/*****************************************************************************/

/**
@brief 构造函数。初始化EtherCAT PDO配置状态机。
@param fsm 指向PDO配置状态机的指针。
@param fsm_coe 指向要使用的CoE状态机的指针。
@return 无。
@details 此函数用于初始化EtherCAT PDO配置状态机。它将设置状态机的成员变量，并调用其他初始化函数来初始化相关的数据结构。

- `fsm`: PDO配置状态机。
- `fsm_coe`: 要使用的CoE状态机。
*/
void ec_fsm_pdo_init(
    ec_fsm_pdo_t *fsm,    /**< PDO配置状态机。 */
    ec_fsm_coe_t *fsm_coe /**< 要使用的CoE状态机 */
)
{
    fsm->fsm_coe = fsm_coe;
    ec_fsm_pdo_entry_init(&fsm->fsm_pdo_entry, fsm_coe);
    ec_pdo_list_init(&fsm->pdos);
    ec_sdo_request_init(&fsm->request);
    ec_pdo_init(&fsm->slave_pdo);
}

/*****************************************************************************/

/**
@brief 析构函数。清理EtherCAT PDO配置状态机的资源。
@param fsm 指向PDO配置状态机的指针。
@return 无。
@details 此函数用于清理EtherCAT PDO配置状态机的资源。它将调用其他清理函数来清理相关的数据结构。

- `fsm`: PDO配置状态机。
*/
void ec_fsm_pdo_clear(
    ec_fsm_pdo_t *fsm /**< PDO配置状态机。 */
)
{
    ec_fsm_pdo_entry_clear(&fsm->fsm_pdo_entry);
    ec_pdo_list_clear(&fsm->pdos);
    ec_sdo_request_clear(&fsm->request);
    ec_pdo_clear(&fsm->slave_pdo);
}

/*****************************************************************************/

/**
@brief 打印当前和期望的PDO分配情况。
@param fsm 指向PDO配置状态机的指针。
@return 无。
@details 此函数用于打印当前和期望的PDO分配情况。

- `fsm`: PDO配置状态机。
*/
void ec_fsm_pdo_print(
    ec_fsm_pdo_t *fsm /**< PDO配置状态机。 */
)
{
    printk(KERN_CONT "当前分配的PDO：");
    ec_pdo_list_print(&fsm->sync->pdos);
    printk(KERN_CONT "。待分配的PDO：");
    ec_pdo_list_print(&fsm->pdos);
    printk(KERN_CONT "\n");
}

/*****************************************************************************/

/**
@brief 开始读取PDO配置。
@param fsm 指向PDO配置状态机的指针。
@param slave 要配置的从站。
@return 无。
@details 此函数用于开始读取PDO配置。它将设置要配置的从站，并将状态设置为开始读取PDO配置。

- `fsm`: PDO配置状态机。
- `slave`: 要配置的从站。
*/
void ec_fsm_pdo_start_reading(
    ec_fsm_pdo_t *fsm, /**< PDO配置状态机。 */
    ec_slave_t *slave  /**< 要配置的从站 */
)
{
    fsm->slave = slave;
    fsm->state = ec_fsm_pdo_read_state_start;
}

/*****************************************************************************/

/**
@brief 开始写入PDO配置。
@param fsm 指向PDO配置状态机的指针。
@param slave 要配置的从站。
@return 无。
@details 此函数用于开始写入PDO配置。它将设置要配置的从站，并将状态设置为开始写入PDO配置。

- `fsm`: PDO配置状态机。
- `slave`: 要配置的从站。
*/
void ec_fsm_pdo_start_configuration(
    ec_fsm_pdo_t *fsm, /**< PDO配置状态机。 */
    ec_slave_t *slave  /**< 要配置的从站 */
)
{
    fsm->slave = slave;
    fsm->state = ec_fsm_pdo_conf_state_start;
}

/*****************************************************************************/

/**
@brief 获取状态机的运行状态。
@return 如果状态机已终止，则返回0；否则返回1。
*/
int ec_fsm_pdo_running(
    const ec_fsm_pdo_t *fsm /**< PDO配置状态机。 */
)
{
    return fsm->state != ec_fsm_pdo_state_end && fsm->state != ec_fsm_pdo_state_error;
}

/*****************************************************************************/

/**
@brief 执行状态机的当前状态。
@param fsm 指向PDO配置状态机的指针。
@param datagram 要使用的数据报文。
@return 如果状态机仍在运行，则返回1；否则返回0。
@details 此函数用于执行状态机的当前状态。如果状态机的数据报文尚未发送或接收，则状态机的执行将延迟到下一个周期。

- `fsm`: PDO配置状态机。
- `datagram`: 要使用的数据报文。
*/
int ec_fsm_pdo_exec(
    ec_fsm_pdo_t *fsm,      /**< PDO配置状态机。 */
    ec_datagram_t *datagram /**< 要使用的数据报文。 */
)
{
    fsm->state(fsm, datagram);

    return ec_fsm_pdo_running(fsm);
}

/*****************************************************************************/

/**
@brief 获取状态机的执行结果。
@return 如果状态机正常终止，则返回1；否则返回0。
*/
int ec_fsm_pdo_success(
    const ec_fsm_pdo_t *fsm /**< PDO配置状态机。 */
)
{
    return fsm->state == ec_fsm_pdo_state_end;
}
/******************************************************************************
* 读取状态函数。
 *****************************************************************************/

/**
@brief 开始读取PDO分配。
@param fsm 指向PDO配置状态机的指针。
@param datagram 要使用的数据报文。
@return 无。
@details 此函数用于开始读取PDO分配。它将设置状态机的同步管理器索引，并调用下一个同步管理器的读取动作函数。

- `fsm`: PDO配置状态机。
- `datagram`: 要使用的数据报文。
*/
void ec_fsm_pdo_read_state_start(
    ec_fsm_pdo_t *fsm,      /**< 有限状态机。 */
    ec_datagram_t *datagram /**< 要使用的数据报文。 */
)
{
    // 读取第一个未保留用于邮箱的同步管理器的PDO分配
    fsm->sync_index = 1; // 下一个是2
    ec_fsm_pdo_read_action_next_sync(fsm, datagram);
}

/*****************************************************************************/

/**
@brief 读取下一个同步管理器的PDO分配。
@param fsm 指向PDO配置状态机的指针。
@param datagram 要使用的数据报文。
@return 无。
@details 此函数用于读取下一个同步管理器的PDO分配。它将逐个检查同步管理器，并执行相应的操作。

- `fsm`: PDO配置状态机。
- `datagram`: 要使用的数据报文。
*/
void ec_fsm_pdo_read_action_next_sync(
    ec_fsm_pdo_t *fsm,      /**< 有限状态机。 */
    ec_datagram_t *datagram /**< 要使用的数据报文。 */
)
{
    ec_slave_t *slave = fsm->slave;

    fsm->sync_index++;

    for (; fsm->sync_index < EC_MAX_SYNC_MANAGERS; fsm->sync_index++)
    {
        if (!(fsm->sync = ec_slave_get_sync(slave, fsm->sync_index)))
            continue;

        EC_SLAVE_DBG(slave, 1, "正在读取第%u个同步管理器的PDO分配。\n",
                     fsm->sync_index);

        ec_pdo_list_clear_pdos(&fsm->pdos);

        ecrt_sdo_request_index(&fsm->request, 0x1C10 + fsm->sync_index, 0);
        ecrt_sdo_request_read(&fsm->request);
        fsm->state = ec_fsm_pdo_read_state_pdo_count;
        ec_fsm_coe_transfer(fsm->fsm_coe, fsm->slave, &fsm->request);
        ec_fsm_coe_exec(fsm->fsm_coe, datagram); // 立即执行
        return;
    }

    EC_SLAVE_DBG(slave, 1, "PDO配置读取完成。\n");

    ec_pdo_list_clear_pdos(&fsm->pdos);
    fsm->state = ec_fsm_pdo_state_end;
}


/*****************************************************************************/

/**
@brief 计算已分配的PDO数量。
@param fsm 有限状态机。
@param datagram 使用的数据报。
@return 无
@details
- 如果执行ec_fsm_coe_exec(fsm->fsm_coe, datagram)失败，则返回。
- 如果ec_fsm_coe_success(fsm->fsm_coe)为假，则输出错误信息，指示无法读取SM%u的已分配PDO数量，并调用ec_fsm_pdo_read_action_next_sync(fsm, datagram)函数。
- 如果fsm->request.data_size的大小不等于sizeof(uint8_t)，则输出错误信息，指示在上传SDO 0x%04X:%02X时返回的数据大小无效，并调用ec_fsm_pdo_read_action_next_sync(fsm, datagram)函数。
- 将fsm->request.data中的值读取为一个字节，并将其赋值给fsm->pdo_count。
- 输出调试信息，指示已分配的PDO数量为%u。
- 将fsm->pdo_pos设置为1。
- 调用ec_fsm_pdo_read_action_next_pdo(fsm, datagram)函数，读取第一个PDO。
*/
void ec_fsm_pdo_read_state_pdo_count(
    ec_fsm_pdo_t *fsm,      /**< 有限状态机。 */
    ec_datagram_t *datagram /**< 使用的数据报。 */
)
{
    if (ec_fsm_coe_exec(fsm->fsm_coe, datagram))
    {
        return;
    }

    if (!ec_fsm_coe_success(fsm->fsm_coe))
    {
        EC_SLAVE_ERR(fsm->slave, "无法读取SM%u的已分配PDO数量。\n",
                     fsm->sync_index);
        ec_fsm_pdo_read_action_next_sync(fsm, datagram);
        return;
    }

    if (fsm->request.data_size != sizeof(uint8_t))
    {
        EC_SLAVE_ERR(fsm->slave, "上传SDO 0x%04X:%02X时返回的数据大小%zu无效。\n",
                     fsm->request.index, fsm->request.subindex,
                     fsm->request.data_size);
        ec_fsm_pdo_read_action_next_sync(fsm, datagram);
        return;
    }
    fsm->pdo_count = EC_READ_U8(fsm->request.data);

    EC_SLAVE_DBG(fsm->slave, 1, "已分配%u个PDO。\n", fsm->pdo_count);

    // 读取第一个PDO
    fsm->pdo_pos = 1;
    ec_fsm_pdo_read_action_next_pdo(fsm, datagram);
}

/*****************************************************************************/

/** Read next PDO.
 */
void ec_fsm_pdo_read_action_next_pdo(
    ec_fsm_pdo_t *fsm,      /**< Finite state machine. */
    ec_datagram_t *datagram /**< Datagram to use. */
)
{
    if (fsm->pdo_pos <= fsm->pdo_count)
    {
        ecrt_sdo_request_index(&fsm->request, 0x1C10 + fsm->sync_index,
                               fsm->pdo_pos);
        ecrt_sdo_request_read(&fsm->request);
        fsm->state = ec_fsm_pdo_read_state_pdo;
        ec_fsm_coe_transfer(fsm->fsm_coe, fsm->slave, &fsm->request);
        ec_fsm_coe_exec(fsm->fsm_coe, datagram); // execute immediately
        return;
    }

    // finished reading PDO configuration

    ec_pdo_list_copy(&fsm->sync->pdos, &fsm->pdos);
    ec_pdo_list_clear_pdos(&fsm->pdos);

    // next sync manager
    ec_fsm_pdo_read_action_next_sync(fsm, datagram);
}

/*****************************************************************************/

/** Fetch PDO information.
 */
void ec_fsm_pdo_read_state_pdo(
    ec_fsm_pdo_t *fsm,      /**< Finite state machine. */
    ec_datagram_t *datagram /**< Datagram to use. */
)
{
    if (ec_fsm_coe_exec(fsm->fsm_coe, datagram))
    {
        return;
    }

    if (!ec_fsm_coe_success(fsm->fsm_coe))
    {
        EC_SLAVE_ERR(fsm->slave, "Failed to read index of"
                                 " assigned PDO %u from SM%u.\n",
                     fsm->pdo_pos, fsm->sync_index);
        ec_fsm_pdo_read_action_next_sync(fsm, datagram);
        return;
    }

    if (fsm->request.data_size != sizeof(uint16_t))
    {
        EC_SLAVE_ERR(fsm->slave, "Invalid data size %zu returned"
                                 " when uploading SDO 0x%04X:%02X.\n",
                     fsm->request.data_size,
                     fsm->request.index, fsm->request.subindex);
        ec_fsm_pdo_read_action_next_sync(fsm, datagram);
        return;
    }

    if (!(fsm->pdo = (ec_pdo_t *)
              kmalloc(sizeof(ec_pdo_t), GFP_KERNEL)))
    {
        EC_SLAVE_ERR(fsm->slave, "Failed to allocate PDO.\n");
        ec_fsm_pdo_read_action_next_sync(fsm, datagram);
        return;
    }

    ec_pdo_init(fsm->pdo);
    fsm->pdo->index = EC_READ_U16(fsm->request.data);
    fsm->pdo->sync_index = fsm->sync_index;

    EC_SLAVE_DBG(fsm->slave, 1, "PDO 0x%04X.\n", fsm->pdo->index);

    list_add_tail(&fsm->pdo->list, &fsm->pdos.list);

    fsm->state = ec_fsm_pdo_read_state_pdo_entries;
    ec_fsm_pdo_entry_start_reading(&fsm->fsm_pdo_entry, fsm->slave, fsm->pdo);
    fsm->state(fsm, datagram); // execute immediately
}

/*****************************************************************************/

/** Fetch PDO information.
 */
void ec_fsm_pdo_read_state_pdo_entries(
    ec_fsm_pdo_t *fsm,      /**< Finite state machine. */
    ec_datagram_t *datagram /**< Datagram to use. */
)
{
    if (ec_fsm_pdo_entry_exec(&fsm->fsm_pdo_entry, datagram))
    {
        return;
    }

    if (!ec_fsm_pdo_entry_success(&fsm->fsm_pdo_entry))
    {
        EC_SLAVE_ERR(fsm->slave, "Failed to read mapped PDO entries"
                                 " for PDO 0x%04X.\n",
                     fsm->pdo->index);
        ec_fsm_pdo_read_action_next_sync(fsm, datagram);
        return;
    }

    // next PDO
    fsm->pdo_pos++;
    ec_fsm_pdo_read_action_next_pdo(fsm, datagram);
}

/******************************************************************************
 * Writing state functions.
 *****************************************************************************/

/** Start PDO configuration.
 */
void ec_fsm_pdo_conf_state_start(
    ec_fsm_pdo_t *fsm,      /**< Finite state machine. */
    ec_datagram_t *datagram /**< Datagram to use. */
)
{
    if (!fsm->slave->config)
    {
        fsm->state = ec_fsm_pdo_state_end;
        return;
    }

    fsm->sync_index = 1; // next is 2
    ec_fsm_pdo_conf_action_next_sync(fsm, datagram);
}

/*****************************************************************************/

/** Assign next PDO.
 *
 * \return Next PDO, or NULL.
 */
ec_pdo_t *ec_fsm_pdo_conf_action_next_pdo(
    const ec_fsm_pdo_t *fsm,     /**< PDO configuration state machine. */
    const struct list_head *list /**< current PDO list item */
)
{
    list = list->next;
    if (list == &fsm->pdos.list)
        return NULL; // no next PDO
    return list_entry(list, ec_pdo_t, list);
}

/*****************************************************************************/

/** Get the next sync manager for a pdo configuration.
 */
void ec_fsm_pdo_conf_action_next_sync(
    ec_fsm_pdo_t *fsm,      /**< Finite state machine. */
    ec_datagram_t *datagram /**< Datagram to use. */
)
{
    fsm->sync_index++;

    for (; fsm->sync_index < EC_MAX_SYNC_MANAGERS; fsm->sync_index++)
    {
        if (!fsm->slave->config)
        {
            // slave configuration removed in the meantime
            fsm->state = ec_fsm_pdo_state_error;
            return;
        }

        if (ec_pdo_list_copy(&fsm->pdos,
                             &fsm->slave->config->sync_configs[fsm->sync_index].pdos))
        {
            fsm->state = ec_fsm_pdo_state_error;
            return;
        }

        if (!(fsm->sync = ec_slave_get_sync(fsm->slave, fsm->sync_index)))
        {
            if (!list_empty(&fsm->pdos.list))
                EC_SLAVE_WARN(fsm->slave, "PDOs configured for SM%u,"
                                          " but slave does not provide the"
                                          " sync manager information!\n",
                              fsm->sync_index);
            continue;
        }

        // get first configured PDO
        if (!(fsm->pdo =
                  ec_fsm_pdo_conf_action_next_pdo(fsm, &fsm->pdos.list)))
        {
            // no pdos configured
            ec_fsm_pdo_conf_action_check_assignment(fsm, datagram);
            return;
        }

        ec_fsm_pdo_conf_action_pdo_mapping(fsm, datagram);
        return;
    }

    fsm->state = ec_fsm_pdo_state_end;
}

/*****************************************************************************/

/** Check if the mapping has to be read, otherwise start to configure it.
 */
void ec_fsm_pdo_conf_action_pdo_mapping(
    ec_fsm_pdo_t *fsm,      /**< Finite state machine. */
    ec_datagram_t *datagram /**< Datagram to use. */
)
{
    const ec_pdo_t *assigned_pdo;

    fsm->slave_pdo.index = fsm->pdo->index;

    if ((assigned_pdo = ec_slave_find_pdo(fsm->slave, fsm->pdo->index)))
    {
        ec_pdo_copy_entries(&fsm->slave_pdo, assigned_pdo);
    }
    else
    { // configured PDO is not assigned and thus unknown
        ec_pdo_clear_entries(&fsm->slave_pdo);
    }

    if (list_empty(&fsm->slave_pdo.entries))
    {
        EC_SLAVE_DBG(fsm->slave, 1, "Reading mapping of PDO 0x%04X.\n",
                     fsm->pdo->index);

        // pdo mapping is unknown; start loading it
        ec_fsm_pdo_entry_start_reading(&fsm->fsm_pdo_entry, fsm->slave,
                                       &fsm->slave_pdo);
        fsm->state = ec_fsm_pdo_conf_state_read_mapping;
        fsm->state(fsm, datagram); // execute immediately
        return;
    }

    // pdo mapping is known, check if it most be re-configured
    ec_fsm_pdo_conf_action_check_mapping(fsm, datagram);
}

/*****************************************************************************/

/** Execute the PDO entry state machine to read the current PDO's mapping.
 */
void ec_fsm_pdo_conf_state_read_mapping(
    ec_fsm_pdo_t *fsm,      /**< Finite state machine. */
    ec_datagram_t *datagram /**< Datagram to use. */
)
{
    if (ec_fsm_pdo_entry_exec(&fsm->fsm_pdo_entry, datagram))
    {
        return;
    }

    if (!ec_fsm_pdo_entry_success(&fsm->fsm_pdo_entry))
        EC_SLAVE_WARN(fsm->slave,
                      "Failed to read PDO entries for PDO 0x%04X.\n",
                      fsm->pdo->index);

    // check if the mapping must be re-configured
    ec_fsm_pdo_conf_action_check_mapping(fsm, datagram);
}

/*****************************************************************************/

/** Check if the mapping has to be re-configured.
 *
 * \todo Display mapping differences.
 */
void ec_fsm_pdo_conf_action_check_mapping(
    ec_fsm_pdo_t *fsm,      /**< Finite state machine. */
    ec_datagram_t *datagram /**< Datagram to use. */
)
{
    if (fsm->slave->sii_image)
    {
        // check, if slave supports PDO configuration
        if ((fsm->slave->sii_image->sii.mailbox_protocols & EC_MBOX_COE) && fsm->slave->sii_image->sii.has_general && fsm->slave->sii_image->sii.coe_details.enable_pdo_configuration)
        {

            // always write PDO mapping
            ec_fsm_pdo_entry_start_configuration(&fsm->fsm_pdo_entry, fsm->slave,
                                                 fsm->pdo, &fsm->slave_pdo);
            fsm->state = ec_fsm_pdo_conf_state_mapping;
            fsm->state(fsm, datagram); // execure immediately
            return;
        }
        else if (!ec_pdo_equal_entries(fsm->pdo, &fsm->slave_pdo))
        {
            EC_SLAVE_WARN(fsm->slave, "Slave does not support"
                                      " changing the PDO mapping!\n");
            EC_SLAVE_WARN(fsm->slave, "");
            printk(KERN_CONT "Currently mapped PDO entries: ");
            ec_pdo_print_entries(&fsm->slave_pdo);
            printk(KERN_CONT ". Entries to map: ");
            ec_pdo_print_entries(fsm->pdo);
            printk(KERN_CONT "\n");
        }
    }
    else
    {
        EC_SLAVE_ERR(fsm->slave, "Slave cannot do PDO mapping."
                                 " SII data not available.\n");
    }

    ec_fsm_pdo_conf_action_next_pdo_mapping(fsm, datagram);
}

/*****************************************************************************/

/** Let the PDO entry state machine configure the current PDO's mapping.
 */
void ec_fsm_pdo_conf_state_mapping(
    ec_fsm_pdo_t *fsm,      /**< Finite state machine. */
    ec_datagram_t *datagram /**< Datagram to use. */
)
{
    if (ec_fsm_pdo_entry_exec(&fsm->fsm_pdo_entry, datagram))
    {
        return;
    }

    if (!ec_fsm_pdo_entry_success(&fsm->fsm_pdo_entry))
        EC_SLAVE_WARN(fsm->slave,
                      "Failed to configure mapping of PDO 0x%04X.\n",
                      fsm->pdo->index);

    ec_fsm_pdo_conf_action_next_pdo_mapping(fsm, datagram);
}

/*****************************************************************************/

/** Check mapping of next PDO, otherwise configure assignment.
 */
void ec_fsm_pdo_conf_action_next_pdo_mapping(
    ec_fsm_pdo_t *fsm,      /**< Finite state machine. */
    ec_datagram_t *datagram /**< Datagram to use. */
)
{
    // get next configured PDO
    if (!(fsm->pdo = ec_fsm_pdo_conf_action_next_pdo(fsm, &fsm->pdo->list)))
    {
        // no more configured pdos
        ec_fsm_pdo_conf_action_check_assignment(fsm, datagram);
        return;
    }

    ec_fsm_pdo_conf_action_pdo_mapping(fsm, datagram);
}

/*****************************************************************************/

/** Check if the PDO assignment of the current SM has to be re-configured.
 */
void ec_fsm_pdo_conf_action_check_assignment(
    ec_fsm_pdo_t *fsm,      /**< Finite state machine. */
    ec_datagram_t *datagram /**< Datagram to use. */
)
{
    if (fsm->slave->sii_image)
    {
        if ((fsm->slave->sii_image->sii.mailbox_protocols & EC_MBOX_COE) && fsm->slave->sii_image->sii.has_general && fsm->slave->sii_image->sii.coe_details.enable_pdo_assign)
        {

            // always write PDO assignment
            if (fsm->slave->master->debug_level)
            {
                EC_SLAVE_DBG(fsm->slave, 1, "Setting PDO assignment of SM%u:\n",
                             fsm->sync_index);
                EC_SLAVE_DBG(fsm->slave, 1, "");
                ec_fsm_pdo_print(fsm);
            }

            if (ec_sdo_request_alloc(&fsm->request, 2))
            {
                fsm->state = ec_fsm_pdo_state_error;
                return;
            }

            // set mapped PDO count to zero
            EC_WRITE_U8(fsm->request.data, 0); // zero PDOs mapped
            fsm->request.data_size = 1;
            ecrt_sdo_request_index(&fsm->request, 0x1C10 + fsm->sync_index, 0);
            ecrt_sdo_request_write(&fsm->request);

            EC_SLAVE_DBG(fsm->slave, 1, "Setting number of assigned"
                                        " PDOs to zero.\n");

            fsm->state = ec_fsm_pdo_conf_state_zero_pdo_count;
            ec_fsm_coe_transfer(fsm->fsm_coe, fsm->slave, &fsm->request);
            ec_fsm_coe_exec(fsm->fsm_coe, datagram); // execute immediately
            return;
        }
        else if (!ec_pdo_list_equal(&fsm->sync->pdos, &fsm->pdos))
        {
            EC_SLAVE_WARN(fsm->slave, "Slave does not support assigning PDOs!\n");
            EC_SLAVE_WARN(fsm->slave, "");
            ec_fsm_pdo_print(fsm);
        }
    }
    else
    {
        EC_SLAVE_ERR(fsm->slave, "Slave cannot do PDO assignment."
                                 " SII data not available.\n");
    }

    ec_fsm_pdo_conf_action_next_sync(fsm, datagram);
}

/*****************************************************************************/

/** Set the number of assigned PDOs to zero.
 */
void ec_fsm_pdo_conf_state_zero_pdo_count(
    ec_fsm_pdo_t *fsm,      /**< Finite state machine. */
    ec_datagram_t *datagram /**< Datagram to use. */
)
{
    if (ec_fsm_coe_exec(fsm->fsm_coe, datagram))
    {
        return;
    }

    if (!ec_fsm_coe_success(fsm->fsm_coe))
    {
        EC_SLAVE_WARN(fsm->slave, "Failed to clear PDO assignment of SM%u.\n",
                      fsm->sync_index);
        EC_SLAVE_WARN(fsm->slave, "");
        ec_fsm_pdo_print(fsm);
        ec_fsm_pdo_conf_action_next_sync(fsm, datagram);
        return;
    }

    // the sync manager's assigned PDOs have been cleared
    ec_pdo_list_clear_pdos(&fsm->sync->pdos);

    // assign all PDOs belonging to the current sync manager

    // find first PDO
    if (!(fsm->pdo = ec_fsm_pdo_conf_action_next_pdo(fsm, &fsm->pdos.list)))
    {
        // check for mapping to be altered
        ec_fsm_pdo_conf_action_next_sync(fsm, datagram);
        return;
    }

    // assign first PDO
    fsm->pdo_pos = 1;
    ec_fsm_pdo_conf_action_assign_pdo(fsm, datagram);
}

/*****************************************************************************/

/** Assign a PDO.
 */
void ec_fsm_pdo_conf_action_assign_pdo(
    ec_fsm_pdo_t *fsm,      /**< Finite state machine. */
    ec_datagram_t *datagram /**< Datagram to use. */
)
{
    EC_WRITE_U16(fsm->request.data, fsm->pdo->index);
    fsm->request.data_size = 2;
    ecrt_sdo_request_index(&fsm->request,
                           0x1C10 + fsm->sync_index, fsm->pdo_pos);
    ecrt_sdo_request_write(&fsm->request);

    EC_SLAVE_DBG(fsm->slave, 1, "Assigning PDO 0x%04X at position %u.\n",
                 fsm->pdo->index, fsm->pdo_pos);

    fsm->state = ec_fsm_pdo_conf_state_assign_pdo;
    ec_fsm_coe_transfer(fsm->fsm_coe, fsm->slave, &fsm->request);
    ec_fsm_coe_exec(fsm->fsm_coe, datagram); // execute immediately
}

/*****************************************************************************/

/** Add a PDO to the sync managers PDO assignment.
 */
void ec_fsm_pdo_conf_state_assign_pdo(
    ec_fsm_pdo_t *fsm,      /**< Finite state machine. */
    ec_datagram_t *datagram /**< Datagram to use. */
)
{
    if (ec_fsm_coe_exec(fsm->fsm_coe, datagram))
    {
        return;
    }

    if (!ec_fsm_coe_success(fsm->fsm_coe))
    {
        EC_SLAVE_WARN(fsm->slave, "Failed to assign PDO 0x%04X at position %u"
                                  " of SM%u.\n",
                      fsm->pdo->index, fsm->pdo_pos, fsm->sync_index);
        EC_SLAVE_WARN(fsm->slave, "");
        ec_fsm_pdo_print(fsm);
        fsm->state = ec_fsm_pdo_state_error;
        return;
    }

    // find next PDO
    if (!(fsm->pdo = ec_fsm_pdo_conf_action_next_pdo(fsm, &fsm->pdo->list)))
    {
        // no more PDOs to assign, set PDO count
        EC_WRITE_U8(fsm->request.data, fsm->pdo_pos);
        fsm->request.data_size = 1;
        ecrt_sdo_request_index(&fsm->request, 0x1C10 + fsm->sync_index, 0);
        ecrt_sdo_request_write(&fsm->request);

        EC_SLAVE_DBG(fsm->slave, 1,
                     "Setting number of assigned PDOs to %u.\n",
                     fsm->pdo_pos);

        fsm->state = ec_fsm_pdo_conf_state_set_pdo_count;
        ec_fsm_coe_transfer(fsm->fsm_coe, fsm->slave, &fsm->request);
        ec_fsm_coe_exec(fsm->fsm_coe, datagram); // execute immediately
        return;
    }

    // add next PDO to assignment
    fsm->pdo_pos++;
    ec_fsm_pdo_conf_action_assign_pdo(fsm, datagram);
}

/*****************************************************************************/

/** Set the number of assigned PDOs.
 */
void ec_fsm_pdo_conf_state_set_pdo_count(
    ec_fsm_pdo_t *fsm,      /**< Finite state machine. */
    ec_datagram_t *datagram /**< Datagram to use. */
)
{
    if (ec_fsm_coe_exec(fsm->fsm_coe, datagram))
    {
        return;
    }

    if (!ec_fsm_coe_success(fsm->fsm_coe))
    {
        EC_SLAVE_WARN(fsm->slave, "Failed to set number of"
                                  " assigned PDOs of SM%u.\n",
                      fsm->sync_index);
        EC_SLAVE_WARN(fsm->slave, "");
        ec_fsm_pdo_print(fsm);
        fsm->state = ec_fsm_pdo_state_error;
        return;
    }

    // PDOs have been configured
    ec_pdo_list_copy(&fsm->sync->pdos, &fsm->pdos);

    EC_SLAVE_DBG(fsm->slave, 1, "Successfully configured"
                                " PDO assignment of SM%u.\n",
                 fsm->sync_index);

    // check if PDO mapping has to be altered
    ec_fsm_pdo_conf_action_next_sync(fsm, datagram);
}

/******************************************************************************
 * Common state functions
 *****************************************************************************/

/** State: ERROR.
 */
void ec_fsm_pdo_state_error(
    ec_fsm_pdo_t *fsm,      /**< Finite state machine. */
    ec_datagram_t *datagram /**< Datagram to use. */
)
{
}

/*****************************************************************************/

/** State: END.
 */
void ec_fsm_pdo_state_end(
    ec_fsm_pdo_t *fsm,      /**< Finite state machine. */
    ec_datagram_t *datagram /**< Datagram to use. */
)
{
}

/*****************************************************************************/

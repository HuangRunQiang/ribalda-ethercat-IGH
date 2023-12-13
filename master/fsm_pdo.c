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
<<<<<<< Updated upstream
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
=======
 * @brief 构造函数。
 * @param fsm PDO配置状态机。
 * @param fsm_coe 使用的CoE状态机。
 * @return 无。
 * @details
 * - 将fsm_coe赋值给fsm的fsm_coe成员。
 * - 调用ec_fsm_pdo_entry_init函数初始化fsm的fsm_pdo_entry成员。
 * - 调用ec_pdo_list_init函数初始化fsm的pdos成员。
 * - 调用ec_sdo_request_init函数初始化fsm的request成员。
 * - 调用ec_pdo_init函数初始化fsm的slave_pdo成员。
 */
void ec_fsm_pdo_init(
    ec_fsm_pdo_t *fsm,    /**< PDO配置状态机 */
    ec_fsm_coe_t *fsm_coe /**< 使用的CoE状态机 */
>>>>>>> Stashed changes
)
{
    fsm->fsm_coe = fsm_coe;
    ec_fsm_pdo_entry_init(&fsm->fsm_pdo_entry, fsm_coe);
    ec_pdo_list_init(&fsm->pdos);
    ec_sdo_request_init(&fsm->request);
    ec_pdo_init(&fsm->slave_pdo);
}

/*****************************************************************************/

<<<<<<< Updated upstream
/**
@brief 析构函数。清理EtherCAT PDO配置状态机的资源。
@param fsm 指向PDO配置状态机的指针。
@return 无。
@details 此函数用于清理EtherCAT PDO配置状态机的资源。它将调用其他清理函数来清理相关的数据结构。

- `fsm`: PDO配置状态机。
*/
void ec_fsm_pdo_clear(
    ec_fsm_pdo_t *fsm /**< PDO配置状态机。 */
=======
/** 析构函数。
 * 
 * @param fsm PDO配置状态机。
 * @return 无。
 * @details
 * - 调用ec_fsm_pdo_entry_clear函数清除fsm的fsm_pdo_entry成员。
 * - 调用ec_pdo_list_clear函数清除fsm的pdos成员。
 * - 调用ec_sdo_request_clear函数清除fsm的request成员。
 * - 调用ec_pdo_clear函数清除fsm的slave_pdo成员。
 */
void ec_fsm_pdo_clear(
    ec_fsm_pdo_t *fsm /**< PDO配置状态机 */
>>>>>>> Stashed changes
)
{
    ec_fsm_pdo_entry_clear(&fsm->fsm_pdo_entry);
    ec_pdo_list_clear(&fsm->pdos);
    ec_sdo_request_clear(&fsm->request);
    ec_pdo_clear(&fsm->slave_pdo);
}

/*****************************************************************************/

<<<<<<< Updated upstream
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
=======
/** 打印当前和期望的PDO分配。
 * 
 * @param fsm PDO配置状态机。
 * @return 无。
 * @details
 * - 输出当前分配的PDO信息。
 * - 输出待分配的PDO信息。
 * - 输出换行。
 */
void ec_fsm_pdo_print(
    ec_fsm_pdo_t *fsm /**< PDO配置状态机 */
)
{
    printk(KERN_CONT "当前分配的PDOs：");
    ec_pdo_list_print(&fsm->sync->pdos);
    printk(KERN_CONT "。待分配的PDOs：");
>>>>>>> Stashed changes
    ec_pdo_list_print(&fsm->pdos);
    printk(KERN_CONT "\n");
}

/*****************************************************************************/

<<<<<<< Updated upstream
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
=======
/** 开始读取PDO配置。
 * 
 * @param fsm PDO配置状态机。
 * @param slave 要配置的从站。
 * @return 无。
 * @details
 * - 将slave赋值给fsm的slave成员。
 * - 将状态设置为ec_fsm_pdo_read_state_start。
 */
void ec_fsm_pdo_start_reading(
    ec_fsm_pdo_t *fsm, /**< PDO配置状态机 */
>>>>>>> Stashed changes
    ec_slave_t *slave  /**< 要配置的从站 */
)
{
    fsm->slave = slave;
    fsm->state = ec_fsm_pdo_read_state_start;
}

/*****************************************************************************/

<<<<<<< Updated upstream
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
=======
/** 开始写入PDO配置。
 * 
 * @param fsm PDO配置状态机。
 * @param slave 要配置的从站。
 * @return 无。
 * @details
 * - 将slave赋值给fsm的slave成员。
 * - 将状态设置为ec_fsm_pdo_conf_state_start。
 */
void ec_fsm_pdo_start_configuration(
    ec_fsm_pdo_t *fsm, /**< PDO配置状态机 */
>>>>>>> Stashed changes
    ec_slave_t *slave  /**< 要配置的从站 */
)
{
    fsm->slave = slave;
    fsm->state = ec_fsm_pdo_conf_state_start;
}

/*****************************************************************************/

<<<<<<< Updated upstream
/**
@brief 获取状态机的运行状态。
@return 如果状态机已终止，则返回0；否则返回1。
*/
int ec_fsm_pdo_running(
    const ec_fsm_pdo_t *fsm /**< PDO配置状态机。 */
=======
/** 获取运行状态。
 * 
 * @param fsm PDO配置状态机。
 * @return 如果状态机已终止，则返回false。
 */
int ec_fsm_pdo_running(
    const ec_fsm_pdo_t *fsm /**< PDO配置状态机 */
>>>>>>> Stashed changes
)
{
    return fsm->state != ec_fsm_pdo_state_end && fsm->state != ec_fsm_pdo_state_error;
}

/*****************************************************************************/

<<<<<<< Updated upstream
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
=======
/** 执行当前状态的状态机。
 * 
 * @param fsm PDO配置状态机。
 * @param datagram 使用的数据报。
 * @return 如果状态机已终止，则返回false。
 * @details
 * - 调用当前状态的状态函数。
 * - 返回状态机的运行状态。
 */
int ec_fsm_pdo_exec(
    ec_fsm_pdo_t *fsm,      /**< PDO配置状态机 */
    ec_datagram_t *datagram /**< 使用的数据报 */
>>>>>>> Stashed changes
)
{
    fsm->state(fsm, datagram);

    return ec_fsm_pdo_running(fsm);
}

/*****************************************************************************/

<<<<<<< Updated upstream
/**
@brief 获取状态机的执行结果。
@return 如果状态机正常终止，则返回1；否则返回0。
*/
int ec_fsm_pdo_success(
    const ec_fsm_pdo_t *fsm /**< PDO配置状态机。 */
=======
/** 获取执行结果。
 * 
 * @param fsm PDO配置状态机。
 * @return 如果状态机正常终止，则返回true。
 */
int ec_fsm_pdo_success(
    const ec_fsm_pdo_t *fsm /**< PDO配置状态机 */
>>>>>>> Stashed changes
)
{
    return fsm->state == ec_fsm_pdo_state_end;
}
/******************************************************************************
<<<<<<< Updated upstream
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
=======
 * 读取状态函数。
 *****************************************************************************/

/** 开始读取PDO分配。
 * 
 * @param fsm 有限状态机。
 * @param datagram 使用的数据报。
 * @return 无。
 * @details
 * - 为第一个未保留用于邮箱的同步管理器读取PDO分配。
 * - 将sync_index设置为1（下一个是2）。
 * - 调用ec_fsm_pdo_read_action_next_sync函数。
 */
void ec_fsm_pdo_read_state_start(
    ec_fsm_pdo_t *fsm,      /**< 有限状态机 */
    ec_datagram_t *datagram /**< 使用的数据报 */
)
{
    // 为第一个未保留用于邮箱的同步管理器读取PDO分配
>>>>>>> Stashed changes
    fsm->sync_index = 1; // 下一个是2
    ec_fsm_pdo_read_action_next_sync(fsm, datagram);
}

/*****************************************************************************/

<<<<<<< Updated upstream
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
=======
/** 读取下一个同步管理器的PDO分配。
 * 
 * @param fsm 有限状态机。
 * @param datagram 使用的数据报。
 * @return 无。
 * @details
 * - 获取从站的指针。
 * - 递增sync_index。
 * - 对于每个sync_index，从fsm->sync中获取同步管理器。
 * - 清除fsm的pdos成员。
 * - 设置请求的索引和子索引。
 * - 将状态设置为ec_fsm_pdo_read_state_pdo_count。
 * - 调用ec_fsm_coe_transfer函数传输请求。
 * - 调用ec_fsm_coe_exec函数执行状态机。
 * - 如果状态机的执行未完成，则立即返回。
 */
void ec_fsm_pdo_read_action_next_sync(
    ec_fsm_pdo_t *fsm,      /**< 有限状态机 */
    ec_datagram_t *datagram /**< 使用的数据报 */
>>>>>>> Stashed changes
)
{
    ec_slave_t *slave = fsm->slave;

    fsm->sync_index++;

    for (; fsm->sync_index < EC_MAX_SYNC_MANAGERS; fsm->sync_index++)
    {
        if (!(fsm->sync = ec_slave_get_sync(slave, fsm->sync_index)))
            continue;

<<<<<<< Updated upstream
        EC_SLAVE_DBG(slave, 1, "正在读取第%u个同步管理器的PDO分配。\n",
=======
        EC_SLAVE_DBG(slave, 1, "正在读取SM%u的PDO分配。\n",
>>>>>>> Stashed changes
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

<<<<<<< Updated upstream
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
=======
/** 计算已分配的PDO数量。
 * 
 * @param fsm 有限状态机。
 * @param datagram 使用的数据报。
 * @return 无。
 * @details
 * - 如果状态机的执行未完成，则立即返回。
 * - 如果读取PDO数量失败，则调用ec_fsm_pdo_read_action_next_sync函数。
 * - 如果请求的数据大小无效，则调用ec_fsm_pdo_read_action_next_sync函数。
 * - 分配PDO内存。
 * - 初始化PDO。
 * - 设置PDO的索引和同步管理器索引。
 * - 将PDO添加到pdos链表中。
 * - 将状态设置为ec_fsm_pdo_read_state_pdo_entries。
 * - 调用ec_fsm_pdo_entry_start_reading函数开始读取PDO条目。
 * - 调用状态函数（立即执行）。
 */
void ec_fsm_pdo_read_state_pdo_count(
    ec_fsm_pdo_t *fsm,      /**< 有限状态机 */
    ec_datagram_t *datagram /**< 使用的数据报 */
>>>>>>> Stashed changes
)
{
    if (ec_fsm_coe_exec(fsm->fsm_coe, datagram))
    {
        return;
    }

    if (!ec_fsm_coe_success(fsm->fsm_coe))
    {
<<<<<<< Updated upstream
        EC_SLAVE_ERR(fsm->slave, "无法读取SM%u的已分配PDO数量。\n",
=======
        EC_SLAVE_ERR(fsm->slave, "读取SM%u的已分配PDO数量失败。\n",
>>>>>>> Stashed changes
                     fsm->sync_index);
        ec_fsm_pdo_read_action_next_sync(fsm, datagram);
        return;
    }

    if (fsm->request.data_size != sizeof(uint8_t))
    {
<<<<<<< Updated upstream
        EC_SLAVE_ERR(fsm->slave, "上传SDO 0x%04X:%02X时返回的数据大小%zu无效。\n",
=======
        EC_SLAVE_ERR(fsm->slave, "上传SDO 0x%04X:%02X时返回的数据大小 %zu 无效。\n",
>>>>>>> Stashed changes
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

/** 读取下一个PDO。
 * 
 * @param fsm 有限状态机。
 * @param datagram 使用的数据报。
 * @return 无。
 * @details
 * - 如果pdo_pos小于等于pdo_count，则继续读取下一个PDO。
 * - 设置请求的索引和子索引。
 * - 将状态设置为ec_fsm_pdo_read_state_pdo。
 * - 调用ec_fsm_coe_transfer函数传输请求。
 * - 调用ec_fsm_coe_exec函数执行状态机。
 * - 如果状态机的执行未完成，则立即返回。
 */
void ec_fsm_pdo_read_action_next_pdo(
    ec_fsm_pdo_t *fsm,      /**< 有限状态机 */
    ec_datagram_t *datagram /**< 使用的数据报 */
)
{
    if (fsm->pdo_pos <= fsm->pdo_count)
    {
        ecrt_sdo_request_index(&fsm->request, 0x1C10 + fsm->sync_index,
                               fsm->pdo_pos);
        ecrt_sdo_request_read(&fsm->request);
        fsm->state = ec_fsm_pdo_read_state_pdo;
        ec_fsm_coe_transfer(fsm->fsm_coe, fsm->slave, &fsm->request);
        ec_fsm_coe_exec(fsm->fsm_coe, datagram); // 立即执行
        return;
    }

    // 完成读取PDO配置

    ec_pdo_list_copy(&fsm->sync->pdos, &fsm->pdos);
    ec_pdo_list_clear_pdos(&fsm->pdos);

    // 下一个同步管理器
    ec_fsm_pdo_read_action_next_sync(fsm, datagram);
}

/*****************************************************************************/

/** 获取PDO信息。
 * 
 * @param fsm 有限状态机。
 * @param datagram 使用的数据报。
 * @return 无。
 * @details
 * - 如果状态机的执行未完成，则立即返回。
 * - 如果读取PDO索引失败，则调用ec_fsm_pdo_read_action_next_sync函数。
 * - 如果请求的数据大小无效，则调用ec_fsm_pdo_read_action_next_sync函数。
 * - 分配PDO内存。
 * - 初始化PDO。
 * - 设置PDO的索引和同步管理器索引。
 * - 将PDO添加到pdos链表中。
 * - 将状态设置为ec_fsm_pdo_read_state_pdo_entries。
 * - 调用ec_fsm_pdo_entry_start_reading函数开始读取PDO条目。
 * - 调用状态函数（立即执行）。
 */
void ec_fsm_pdo_read_state_pdo(
    ec_fsm_pdo_t *fsm,      /**< 有限状态机 */
    ec_datagram_t *datagram /**< 使用的数据报 */
)
{
    if (ec_fsm_coe_exec(fsm->fsm_coe, datagram))
    {
        return;
    }

    if (!ec_fsm_coe_success(fsm->fsm_coe))
    {
        EC_SLAVE_ERR(fsm->slave, "读取SM%u的已分配PDO %u 的索引失败。\n",
                     fsm->sync_index, fsm->pdo_pos);
        ec_fsm_pdo_read_action_next_sync(fsm, datagram);
        return;
    }

    if (fsm->request.data_size != sizeof(uint16_t))
    {
        EC_SLAVE_ERR(fsm->slave, "上传SDO 0x%04X:%02X时返回的数据大小 %zu 无效。\n",
                     fsm->request.index, fsm->request.subindex,
                     fsm->request.data_size);
        ec_fsm_pdo_read_action_next_sync(fsm, datagram);
        return;
    }

    if (!(fsm->pdo = (ec_pdo_t *)
              kmalloc(sizeof(ec_pdo_t), GFP_KERNEL)))
    {
        EC_SLAVE_ERR(fsm->slave, "分配PDO失败。\n");
        ec_fsm_pdo_read_action_next_sync(fsm, datagram);
        return;
    }

    ec_pdo_init(fsm->pdo);
    fsm->pdo->index = EC_READ_U16(fsm->request.data);
    fsm->pdo->sync_index = fsm->sync_index;

    EC_SLAVE_DBG(fsm->slave, 1, "PDO 0x%04X。\n", fsm->pdo->index);

    list_add_tail(&fsm->pdo->list, &fsm->pdos.list);

    fsm->state = ec_fsm_pdo_read_state_pdo_entries;
    ec_fsm_pdo_entry_start_reading(&fsm->fsm_pdo_entry, fsm->slave, fsm->pdo);
    fsm->state(fsm, datagram); // 立即执行
}

/*****************************************************************************/

/** 获取PDO条目信息。
 * 
 * @param fsm 有限状态机。
 * @param datagram 使用的数据报。
 * @return 无。
 * @details
 * - 如果状态机的执行未完成，则立即返回。
 * - 如果读取PDO条目失败，则调用ec_fsm_pdo_read_action_next_sync函数。
 * - 调用状态函数（立即执行）。
 * - 如果读取PDO条目成功，则读取下一个PDO。
 */
void ec_fsm_pdo_read_state_pdo_entries(
    ec_fsm_pdo_t *fsm,      /**< 有限状态机 */
    ec_datagram_t *datagram /**< 使用的数据报 */
)
{
    if (ec_fsm_pdo_entry_exec(&fsm->fsm_pdo_entry, datagram))
    {
        return;
    }

    if (!ec_fsm_pdo_entry_success(&fsm->fsm_pdo_entry))
    {
        EC_SLAVE_ERR(fsm->slave, "读取PDO 0x%04X的映射条目失败。\n",
                     fsm->pdo->index);
        ec_fsm_pdo_read_action_next_sync(fsm, datagram);
        return;
    }

    // 下一个PDO
    fsm->pdo_pos++;
    ec_fsm_pdo_read_action_next_pdo(fsm, datagram);
}
/******************************************************************************
 * 写入状态函数。
 *****************************************************************************/

/** 开始PDO配置。
 * 
 * @param fsm 有限状态机。
 * @param datagram 使用的数据报。
 * @return 无。
 * @details
 * - 如果从站的配置不存在，则将状态设置为ec_fsm_pdo_state_end。
 * - 将sync_index设置为1（下一个是2）。
 * - 调用ec_fsm_pdo_conf_action_next_sync函数。
 */
void ec_fsm_pdo_conf_state_start(
    ec_fsm_pdo_t *fsm,      /**< 有限状态机 */
    ec_datagram_t *datagram /**< 使用的数据报 */
)
{
    if (!fsm->slave->config)
    {
        fsm->state = ec_fsm_pdo_state_end;
        return;
    }

    fsm->sync_index = 1; // 下一个是2
    ec_fsm_pdo_conf_action_next_sync(fsm, datagram);
}

/*****************************************************************************/

/** 分配下一个PDO。
 * 
 * @param fsm PDO配置状态机。
 * @param list 当前PDO列表项。
 * @return 下一个PDO，或NULL。
 * @details
 * - 将list指向下一个节点。
 * - 如果list等于fsm->pdos.list，则返回NULL（没有下一个PDO）。
 * - 返回list节点所对应的ec_pdo_t结构体。
 */
ec_pdo_t *ec_fsm_pdo_conf_action_next_pdo(
    const ec_fsm_pdo_t *fsm,     /**< PDO配置状态机 */
    const struct list_head *list /**< 当前PDO列表项 */
)
{
    list = list->next;
    if (list == &fsm->pdos.list)
        return NULL; // 没有下一个PDO
    return list_entry(list, ec_pdo_t, list);
}

/*****************************************************************************/

/** 获取下一个同步管理器的PDO配置。
 * 
 * @param fsm 有限状态机。
 * @param datagram 使用的数据报。
 * @return 无。
 * @details
 * - 递增sync_index。
 * - 对于每个sync_index，从fsm->slave->config->sync_configs[fsm->sync_index].pdos复制到fsm->pdos。
 * - 如果从站的配置不存在，则将状态设置为ec_fsm_pdo_state_error。
 * - 如果fsm->slave->config->sync_configs[fsm->sync_index]不存在，则继续循环。
 * - 如果fsm->slave->config->sync_configs[fsm->sync_index]存在，但fsm->sync不存在，则打印警告信息。
 * - 如果fsm->pdo为NULL，则调用ec_fsm_pdo_conf_action_check_assignment函数。
 * - 否则，调用ec_fsm_pdo_conf_action_pdo_mapping函数。
 */
void ec_fsm_pdo_conf_action_next_sync(
    ec_fsm_pdo_t *fsm,      /**< 有限状态机 */
    ec_datagram_t *datagram /**< 使用的数据报 */
)
{
    fsm->sync_index++;

    for (; fsm->sync_index < EC_MAX_SYNC_MANAGERS; fsm->sync_index++)
    {
        if (!fsm->slave->config)
        {
            // 在此期间移除了从站配置
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
                EC_SLAVE_WARN(fsm->slave, "PDO已配置为SM%u，但从站未提供同步管理器信息！\n",
                              fsm->sync_index);
            continue;
        }

        // 获取第一个配置的PDO
        if (!(fsm->pdo =
                  ec_fsm_pdo_conf_action_next_pdo(fsm, &fsm->pdos.list)))
        {
            // 没有配置的PDO
            ec_fsm_pdo_conf_action_check_assignment(fsm, datagram);
            return;
        }

        ec_fsm_pdo_conf_action_pdo_mapping(fsm, datagram);
        return;
    }

    fsm->state = ec_fsm_pdo_state_end;
}

/*****************************************************************************/

/** 检查是否需要读取映射，否则开始配置。
 * 
 * @param fsm 有限状态机。
 * @param datagram 使用的数据报。
 * @return 无。
 * @details
 * - 将fsm->pdo的index赋值给fsm->slave_pdo的index。
 * - 如果在从站中找到了已分配的PDO，则将已分配的PDO的条目复制到fsm->slave_pdo中。
 * - 否则，清除fsm->slave_pdo的条目。
 * - 如果fsm->slave_pdo的entries为空，则打印调试信息，开始读取PDO映射。
 * - 如果fsm->slave_pdo的entries不为空，则调用ec_fsm_pdo_conf_action_check_mapping函数。
 */
void ec_fsm_pdo_conf_action_pdo_mapping(
    ec_fsm_pdo_t *fsm,      /**< 有限状态机 */
    ec_datagram_t *datagram /**< 使用的数据报 */
)
{
    const ec_pdo_t *assigned_pdo;

    fsm->slave_pdo.index = fsm->pdo->index;

    if ((assigned_pdo = ec_slave_find_pdo(fsm->slave, fsm->pdo->index)))
    {
        ec_pdo_copy_entries(&fsm->slave_pdo, assigned_pdo);
    }
    else
    { // 配置的PDO未分配，因此未知
        ec_pdo_clear_entries(&fsm->slave_pdo);
    }

    if (list_empty(&fsm->slave_pdo.entries))
    {
        EC_SLAVE_DBG(fsm->slave, 1, "正在读取PDO 0x%04X的映射。\n",
                     fsm->pdo->index);

        // pdo映射未知；开始加载
        ec_fsm_pdo_entry_start_reading(&fsm->fsm_pdo_entry, fsm->slave,
                                       &fsm->slave_pdo);
        fsm->state = ec_fsm_pdo_conf_state_read_mapping;
        fsm->state(fsm, datagram); // 立即执行
        return;
    }

    // pdo映射已知，检查是否需要重新配置
    ec_fsm_pdo_conf_action_check_mapping(fsm, datagram);
}

/*****************************************************************************/

/** 执行PDO条目状态机以读取当前PDO的映射。
 * 
 * @param fsm 有限状态机。
 * @param datagram 使用的数据报。
 * @return 无。
 * @details
 * - 如果状态机的执行未完成，则立即返回。
 * - 如果状态机的执行未成功，则打印警告信息。
 * - 检查是否需要重新配置映射。
 */
void ec_fsm_pdo_conf_state_read_mapping(
    ec_fsm_pdo_t *fsm,      /**< 有限状态机 */
    ec_datagram_t *datagram /**< 使用的数据报 */
)
{
    if (ec_fsm_pdo_entry_exec(&fsm->fsm_pdo_entry, datagram))
    {
        return;
    }

    if (!ec_fsm_pdo_entry_success(&fsm->fsm_pdo_entry))
        EC_SLAVE_WARN(fsm->slave,
                      "读取PDO 0x%04X的条目失败。\n",
                      fsm->pdo->index);

    // 检查是否需要重新配置映射
    ec_fsm_pdo_conf_action_check_mapping(fsm, datagram);
}

/*****************************************************************************/

/** 检查是否需要重新配置映射。
 * 
 * @param fsm 有限状态机。
 * @param datagram 使用的数据报。
 * @return 无。
 * @details
 * - 如果从站的sii_image存在，则进行以下检查：
 *   - 检查从站是否支持PDO配置。
 *   - 如果从站支持PDO配置，则始终写入PDO映射。
 *   - 如果从站不支持更改PDO映射，则打印警告信息。
 * - 如果从站的sii_image不存在，则打印错误信息。
 * - 调用ec_fsm_pdo_conf_action_next_pdo_mapping函数。
 */
void ec_fsm_pdo_conf_action_check_mapping(
    ec_fsm_pdo_t *fsm,      /**< 有限状态机 */
    ec_datagram_t *datagram /**< 使用的数据报 */
)
{
    if (fsm->slave->sii_image)
    {
        // 检查从站是否支持PDO配置
        if ((fsm->slave->sii_image->sii.mailbox_protocols & EC_MBOX_COE) && fsm->slave->sii_image->sii.has_general && fsm->slave->sii_image->sii.coe_details.enable_pdo_configuration)
        {

            // 始终写入PDO映射
            ec_fsm_pdo_entry_start_configuration(&fsm->fsm_pdo_entry, fsm->slave,
                                                 fsm->pdo, &fsm->slave_pdo);
            fsm->state = ec_fsm_pdo_conf_state_mapping;
            fsm->state(fsm, datagram); // 立即执行
            return;
        }
        else if (!ec_pdo_equal_entries(fsm->pdo, &fsm->slave_pdo))
        {
            EC_SLAVE_WARN(fsm->slave, "从站不支持更改PDO映射！\n");
            EC_SLAVE_WARN(fsm->slave, "");
            printk(KERN_CONT "当前映射的PDO条目：");
            ec_pdo_print_entries(&fsm->slave_pdo);
            printk(KERN_CONT "。待映射的条目：");
            ec_pdo_print_entries(fsm->pdo);
            printk(KERN_CONT "\n");
        }
    }
    else
    {
        EC_SLAVE_ERR(fsm->slave, "从站无法进行PDO映射。SII数据不可用。\n");
    }

    ec_fsm_pdo_conf_action_next_pdo_mapping(fsm, datagram);
}

/*****************************************************************************/

/** 让PDO条目状态机配置当前PDO的映射。
 * 
 * @param fsm 有限状态机。
 * @param datagram 使用的数据报。
 * @return 无。
 * @details
 * - 如果状态机的执行未完成，则立即返回。
 * - 如果状态机的执行未成功，则打印警告信息。
 * - 调用ec_fsm_pdo_conf_action_next_pdo_mapping函数。
 */
void ec_fsm_pdo_conf_state_mapping(
    ec_fsm_pdo_t *fsm,      /**< 有限状态机 */
    ec_datagram_t *datagram /**< 使用的数据报 */
)
{
    if (ec_fsm_pdo_entry_exec(&fsm->fsm_pdo_entry, datagram))
    {
        return;
    }

    if (!ec_fsm_pdo_entry_success(&fsm->fsm_pdo_entry))
        EC_SLAVE_WARN(fsm->slave,
                      "配置PDO 0x%04X的映射失败。\n",
                      fsm->pdo->index);

    ec_fsm_pdo_conf_action_next_pdo_mapping(fsm, datagram);
}

/*****************************************************************************/

/** 检查下一个PDO的映射，否则配置分配。
 * 
 * @param fsm 有限状态机。
 * @param datagram 使用的数据报。
 * @return 无。
 * @details
 * - 获取下一个配置的PDO。
 * - 如果fsm->pdo为NULL，则调用ec_fsm_pdo_conf_action_check_assignment函数。
 * - 否则，调用ec_fsm_pdo_conf_action_pdo_mapping函数。
 */
void ec_fsm_pdo_conf_action_next_pdo_mapping(
    ec_fsm_pdo_t *fsm,      /**< 有限状态机 */
    ec_datagram_t *datagram /**< 使用的数据报 */
)
{
    // 获取下一个配置的PDO
    if (!(fsm->pdo = ec_fsm_pdo_conf_action_next_pdo(fsm, &fsm->pdo->list)))
    {
        // 没有更多的配置的pdos
        ec_fsm_pdo_conf_action_check_assignment(fsm, datagram);
        return;
    }

    ec_fsm_pdo_conf_action_pdo_mapping(fsm, datagram);
}

/*****************************************************************************/

/** 检查当前同步管理器的PDO分配是否需要重新配置。
 * 
 * @param fsm PDO配置状态机。
 * @param datagram 使用的数据报。
 * @return 无。
 * @details
 * - 如果从站的sii_image存在，则进行以下检查：
 *   - 检查从站是否支持PDO分配。
 *   - 如果从站支持PDO分配，则始终写入PDO分配。
 *   - 如果从站不支持更改PDO分配，则打印警告信息。
 * - 如果从站的sii_image不存在，则打印错误信息。
 * - 调用ec_fsm_pdo_conf_state_zero_pdo_count函数。
 */
void ec_fsm_pdo_conf_action_check_assignment(
    ec_fsm_pdo_t *fsm,      /**< PDO配置状态机 */
    ec_datagram_t *datagram /**< 使用的数据报 */
)
{
    if (fsm->slave->sii_image)
    {
        // 检查从站是否支持PDO分配
        if ((fsm->slave->sii_image->sii.mailbox_protocols & EC_MBOX_COE) && fsm->slave->sii_image->sii.has_general && fsm->slave->sii_image->sii.coe_details.enable_pdo_assign)
        {

            // 始终写入PDO分配
            if (fsm->slave->master->debug_level)
            {
                EC_SLAVE_DBG(fsm->slave, 1, "正在设置SM%u的PDO分配：\n",
                             fsm->sync_index);
                EC_SLAVE_DBG(fsm->slave, 1, "");
                ec_fsm_pdo_print(fsm);
            }

            if (ec_sdo_request_alloc(&fsm->request, 2))
            {
                fsm->state = ec_fsm_pdo_state_error;
                return;
            }

            // 将映射的PDO数目设置为零
            EC_WRITE_U8(fsm->request.data, 0); // 零个映射的PDO
            fsm->request.data_size = 1;
            ecrt_sdo_request_index(&fsm->request, 0x1C10 + fsm->sync_index, 0);
            ecrt_sdo_request_write(&fsm->request);

            EC_SLAVE_DBG(fsm->slave, 1, "将分配的PDO数目设置为零。\n");

            fsm->state = ec_fsm_pdo_conf_state_zero_pdo_count;
            ec_fsm_coe_transfer(fsm->fsm_coe, fsm->slave, &fsm->request);
            ec_fsm_coe_exec(fsm->fsm_coe, datagram); // 立即执行
            return;
        }
        else if (!ec_pdo_list_equal(&fsm->sync->pdos, &fsm->pdos))
        {
            EC_SLAVE_WARN(fsm->slave, "从站不支持分配PDO！\n");
            EC_SLAVE_WARN(fsm->slave, "");
            ec_fsm_pdo_print(fsm);
        }
    }
    else
    {
        EC_SLAVE_ERR(fsm->slave, "从站无法进行PDO分配。SII数据不可用。\n");
    }

    ec_fsm_pdo_conf_state_zero_pdo_count(fsm, datagram);
}

/*****************************************************************************/

/** 将分配的PDO数目设置为零。
 * 
 * @param fsm PDO配置状态机。
 * @param datagram 使用的数据报。
 * @return 无。
 * @details
 * - 如果状态机的执行未完成，则立即返回。
 * - 如果状态机的执行未成功，则打印警告信息。
 * - 清除同步管理器的已分配PDO列表。
 * - 查找第一个PDO。
 * - 如果fsm->pdo为NULL，则调用ec_fsm_pdo_conf_action_next_sync函数。
 * - 否则，调用ec_fsm_pdo_conf_action_assign_pdo函数。
 */
void ec_fsm_pdo_conf_state_zero_pdo_count(
    ec_fsm_pdo_t *fsm,      /**< PDO配置状态机 */
    ec_datagram_t *datagram /**< 使用的数据报 */
)
{
    if (ec_fsm_coe_exec(fsm->fsm_coe, datagram))
    {
        return;
    }

    if (!ec_fsm_coe_success(fsm->fsm_coe))
    {
        EC_SLAVE_WARN(fsm->slave, "清除SM%u的PDO分配失败。\n",
                      fsm->sync_index);
        EC_SLAVE_WARN(fsm->slave, "");
        ec_fsm_pdo_print(fsm);
        ec_fsm_pdo_conf_action_next_sync(fsm, datagram);
        return;
    }

    // 同步管理器的已分配PDO列表已清除
    ec_pdo_list_clear_pdos(&fsm->sync->pdos);

    // 分配属于当前同步管理器的所有PDO

    // 查找第一个PDO
    if (!(fsm->pdo = ec_fsm_pdo_conf_action_next_pdo(fsm, &fsm->pdos.list)))
    {
        // 检查是否需要更改映射
        ec_fsm_pdo_conf_action_next_sync(fsm, datagram);
        return;
    }

    // 分配第一个PDO
    fsm->pdo_pos = 1;
    ec_fsm_pdo_conf_action_assign_pdo(fsm, datagram);
}

/*****************************************************************************/

/** 分配一个PDO。
 * 
 * @param fsm PDO配置状态机。
 * @param datagram 使用的数据报。
 * @return 无。
 * @details
 * - 将fsm->pdo的index赋值给fsm->request的data。
 * - 将fsm->request的data_size设置为2。
 * - 调用ecrt_sdo_request_index函数设置fsm->request的索引。
 * - 调用ecrt_sdo_request_write函数写入fsm->request。
 * - 打印调试信息，分配PDO的位置。
 * - 将状态设置为ec_fsm_pdo_conf_state_assign_pdo。
 * - 调用ec_fsm_coe_transfer函数传输fsm->fsm_coe的COE通信。
 * - 调用ec_fsm_coe_exec函数执行fsm->fsm_coe的COE通信，立即执行。
 */
void ec_fsm_pdo_conf_action_assign_pdo(
    ec_fsm_pdo_t *fsm,      /**< PDO配置状态机 */
    ec_datagram_t *datagram /**< 使用的数据报 */
)
{
    EC_WRITE_U16(fsm->request.data, fsm->pdo->index);
    fsm->request.data_size = 2;
    ecrt_sdo_request_index(&fsm->request,
                           0x1C10 + fsm->sync_index, fsm->pdo_pos);
    ecrt_sdo_request_write(&fsm->request);

    EC_SLAVE_DBG(fsm->slave, 1, "正在分配PDO 0x%04X到位置 %u。\n",
                 fsm->pdo->index, fsm->pdo_pos);

    fsm->state = ec_fsm_pdo_conf_state_assign_pdo;
    ec_fsm_coe_transfer(fsm->fsm_coe, fsm->slave, &fsm->request);
    ec_fsm_coe_exec(fsm->fsm_coe, datagram); // 立即执行
}

/*****************************************************************************/

/** 添加一个PDO到同步管理器的PDO分配中。
 * 
 * @param fsm PDO配置状态机。
 * @param datagram 使用的数据报。
 * @return 无。
 * @details
 * - 如果状态机的执行未完成，则立即返回。
 * - 如果状态机的执行未成功，则打印警告信息。
 * - 查找下一个PDO。
 * - 如果fsm->pdo为NULL，则调用ec_fsm_pdo_conf_state_set_pdo_count函数。
 * - 否则，调用ec_fsm_pdo_conf_action_assign_pdo函数。
 */
void ec_fsm_pdo_conf_state_assign_pdo(
    ec_fsm_pdo_t *fsm,      /**< PDO配置状态机 */
    ec_datagram_t *datagram /**< 使用的数据报 */
)
{
    if (ec_fsm_coe_exec(fsm->fsm_coe, datagram))
    {
        return;
    }

    if (!ec_fsm_coe_success(fsm->fsm_coe))
    {
        EC_SLAVE_WARN(fsm->slave, "在SM%u的位置 %u 处分配PDO 0x%04X失败。\n",
                      fsm->sync_index, fsm->pdo_pos, fsm->pdo->index);
        EC_SLAVE_WARN(fsm->slave, "");
        ec_fsm_pdo_print(fsm);
        fsm->state = ec_fsm_pdo_state_error;
        return;
    }

    // 查找下一个PDO
    if (!(fsm->pdo = ec_fsm_pdo_conf_action_next_pdo(fsm, &fsm->pdo->list)))
    {
        // 没有更多的PDO需要分配，设置PDO数目
        EC_WRITE_U8(fsm->request.data, fsm->pdo_pos);
        fsm->request.data_size = 1;
        ecrt_sdo_request_index(&fsm->request, 0x1C10 + fsm->sync_index, 0);
        ecrt_sdo_request_write(&fsm->request);

        EC_SLAVE_DBG(fsm->slave, 1,
                     "将分配的PDO数目设置为 %u。\n",
                     fsm->pdo_pos);

        fsm->state = ec_fsm_pdo_conf_state_set_pdo_count;
        ec_fsm_coe_transfer(fsm->fsm_coe, fsm->slave, &fsm->request);
        ec_fsm_coe_exec(fsm->fsm_coe, datagram); // 立即执行
        return;
    }

    // 添加下一个PDO到分配中
    fsm->pdo_pos++;
    ec_fsm_pdo_conf_action_assign_pdo(fsm, datagram);
}

/*****************************************************************************/

/** 设置已分配PDO的数目。
 * 
 * @param fsm PDO配置状态机。
 * @param datagram 使用的数据报。
 * @return 无。
 * @details
 * - 如果状态机的执行未完成，则立即返回。
 * - 如果状态机的执行未成功，则打印警告信息。
 * - 分配的PDO已配置完成。
 * - 将fsm->pdos复制到fsm->sync->pdos。
 * - 打印调试信息，成功配置PDO的分配。
 * - 检查是否需要更改PDO映射。
 * - 调用ec_fsm_pdo_conf_action_next_sync函数。
 */
void ec_fsm_pdo_conf_state_set_pdo_count(
    ec_fsm_pdo_t *fsm,      /**< PDO配置状态机 */
    ec_datagram_t *datagram /**< 使用的数据报 */
)
{
    if (ec_fsm_coe_exec(fsm->fsm_coe, datagram))
    {
        return;
    }

    if (!ec_fsm_coe_success(fsm->fsm_coe))
    {
        EC_SLAVE_WARN(fsm->slave, "设置SM%u的已分配PDO数目失败。\n",
                      fsm->sync_index);
        EC_SLAVE_WARN(fsm->slave, "");
        ec_fsm_pdo_print(fsm);
        fsm->state = ec_fsm_pdo_state_error;
        return;
    }

    // PDO已配置
    ec_pdo_list_copy(&fsm->sync->pdos, &fsm->pdos);

    EC_SLAVE_DBG(fsm->slave, 1, "成功配置SM%u的PDO分配。\n",
                 fsm->sync_index);

    // 检查是否需要更改PDO映射
    ec_fsm_pdo_conf_action_next_sync(fsm, datagram);
}

/******************************************************************************
 * 通用状态函数
 *****************************************************************************/

/** 状态：错误。
 * 
 * @param fsm PDO状态机。
 * @param datagram 使用的数据报。
 * @return 无。
 * @details 该状态函数为空，没有具体逻辑。
 */
void ec_fsm_pdo_state_error(
    ec_fsm_pdo_t *fsm,      /**< PDO状态机 */
    ec_datagram_t *datagram /**< 使用的数据报 */
)
{
}

/*****************************************************************************/

/** 状态：结束。
 * 
 * @param fsm PDO状态机。
 * @param datagram 使用的数据报。
 * @return 无。
 * @details 该状态函数为空，没有具体逻辑。
 */
void ec_fsm_pdo_state_end(
    ec_fsm_pdo_t *fsm,      /**< PDO状态机 */
    ec_datagram_t *datagram /**< 使用的数据报 */
)
{
}

/*****************************************************************************/

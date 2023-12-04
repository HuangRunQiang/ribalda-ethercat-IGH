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
 * EtherCAT PDO 映射状态机。
 */

/*****************************************************************************/

#include "globals.h"
#include "master.h"
#include "mailbox.h"
#include "slave_config.h"

#include "fsm_pdo_entry.h"

/*****************************************************************************/

void ec_fsm_pdo_entry_read_state_start(ec_fsm_pdo_entry_t *, ec_datagram_t *);
void ec_fsm_pdo_entry_read_state_count(ec_fsm_pdo_entry_t *, ec_datagram_t *);
void ec_fsm_pdo_entry_read_state_entry(ec_fsm_pdo_entry_t *, ec_datagram_t *);

void ec_fsm_pdo_entry_read_action_next(ec_fsm_pdo_entry_t *, ec_datagram_t *);

void ec_fsm_pdo_entry_conf_state_start(ec_fsm_pdo_entry_t *, ec_datagram_t *);
void ec_fsm_pdo_entry_conf_state_zero_entry_count(ec_fsm_pdo_entry_t *,
                                                  ec_datagram_t *);
void ec_fsm_pdo_entry_conf_state_map_entry(ec_fsm_pdo_entry_t *,
                                           ec_datagram_t *);
void ec_fsm_pdo_entry_conf_state_set_entry_count(ec_fsm_pdo_entry_t *,
                                                 ec_datagram_t *);

void ec_fsm_pdo_entry_conf_action_map(ec_fsm_pdo_entry_t *, ec_datagram_t *);

void ec_fsm_pdo_entry_state_end(ec_fsm_pdo_entry_t *, ec_datagram_t *);
void ec_fsm_pdo_entry_state_error(ec_fsm_pdo_entry_t *, ec_datagram_t *);

/*****************************************************************************/

/**
 * 构造函数。
 * @param fsm PDO映射状态机。
 * @param fsm_coe 要使用的CoE状态机。
 * @return 无返回值。
 * @details 初始化函数，用于初始化PDO映射状态机和CoE状态机。
 */
void ec_fsm_pdo_entry_init(
    ec_fsm_pdo_entry_t *fsm, /**< PDO映射状态机。 */
    ec_fsm_coe_t *fsm_coe    /**< 要使用的CoE状态机。 */
)
{
    fsm->fsm_coe = fsm_coe;
    ec_sdo_request_init(&fsm->request);
}

/*****************************************************************************/

/**
 * 析构函数。
 * @param fsm PDO映射状态机。
 * @return 无返回值。
 * @details 清理函数，用于清理PDO映射状态机。
 */
void ec_fsm_pdo_entry_clear(
    ec_fsm_pdo_entry_t *fsm /**< PDO映射状态机。 */
)
{
    ec_sdo_request_clear(&fsm->request);
}

/*****************************************************************************/

/**
 * 打印当前和期望的PDO映射。
 * @param fsm PDO映射状态机。
 * @return 无返回值。
 * @details 打印函数，用于打印当前和期望的PDO映射。
 */
void ec_fsm_pdo_entry_print(
    ec_fsm_pdo_entry_t *fsm /**< PDO映射状态机。 */
)
{
    printk(KERN_CONT "当前映射的PDO条目: ");
    ec_pdo_print_entries(fsm->cur_pdo);
    printk(KERN_CONT "。待映射的条目: ");
    ec_pdo_print_entries(fsm->source_pdo);
    printk(KERN_CONT "\n");
}


/*****************************************************************************/

/**
 * 开始读取PDO的条目。
 * @param fsm PDO映射状态机。
 * @param slave 要配置的从站。
 * @param pdo 要读取条目的PDO。
 * @return 无返回值。
 * @details 该函数用于开始读取PDO的条目，清除目标PDO的条目，并设置状态为读取开始状态。
 */
void ec_fsm_pdo_entry_start_reading(
    ec_fsm_pdo_entry_t *fsm, /**< PDO映射状态机。 */
    ec_slave_t *slave,       /**< 要配置的从站。 */
    ec_pdo_t *pdo            /**< 要读取条目的PDO。 */
)
{
    fsm->slave = slave;
    fsm->target_pdo = pdo;

    ec_pdo_clear_entries(fsm->target_pdo);

    fsm->state = ec_fsm_pdo_entry_read_state_start;
}

/*****************************************************************************/

/**
 * 开始PDO映射状态机。
 * @param fsm PDO映射状态机。
 * @param slave 要配置的从站。
 * @param pdo 带有期望条目的PDO。
 * @param cur_pdo 当前PDO映射。
 * @return 无返回值。
 * @details 该函数用于开始PDO映射状态机，设置从站、期望的PDO和当前PDO映射，并根据调试级别打印当前和期望的PDO映射，然后设置状态为配置开始状态。
 */
void ec_fsm_pdo_entry_start_configuration(
    ec_fsm_pdo_entry_t *fsm, /**< PDO映射状态机。 */
    ec_slave_t *slave,       /**< 要配置的从站。 */
    const ec_pdo_t *pdo,     /**< 带有期望条目的PDO。 */
    const ec_pdo_t *cur_pdo  /**< 当前PDO映射。 */
)
{
    fsm->slave = slave;
    fsm->source_pdo = pdo;
    fsm->cur_pdo = cur_pdo;

    if (fsm->slave->master->debug_level)
    {
        EC_SLAVE_DBG(slave, 1, "正在改变PDO 0x%04X的映射。\n",
                     pdo->index);
        EC_SLAVE_DBG(slave, 1, "");
        ec_fsm_pdo_entry_print(fsm);
    }

    fsm->state = ec_fsm_pdo_entry_conf_state_start;
}

/*****************************************************************************/

/** 获取运行状态。
 *
 * \return 如果状态机已终止则返回false。
 */
int ec_fsm_pdo_entry_running(
    const ec_fsm_pdo_entry_t *fsm /**< PDO映射状态机。 */
)
{
    return fsm->state != ec_fsm_pdo_entry_state_end && fsm->state != ec_fsm_pdo_entry_state_error;
}

/*****************************************************************************/

/** 执行当前状态。
 *
 * \return 如果状态机已终止则返回false。
 */
int ec_fsm_pdo_entry_exec(
    ec_fsm_pdo_entry_t *fsm, /**< PDO映射状态机。 */
    ec_datagram_t *datagram  /**< 要使用的数据报。 */
)
{
    fsm->state(fsm, datagram);

    return ec_fsm_pdo_entry_running(fsm);
}


/*****************************************************************************/

/** 获取执行结果。
 *
 * \return 如果状态机正常终止则返回true。
 */
int ec_fsm_pdo_entry_success(
    const ec_fsm_pdo_entry_t *fsm /**< PDO映射状态机。 */
)
{
    return fsm->state == ec_fsm_pdo_entry_state_end;
}


/******************************************************************************
  * 读取状态函数
 *****************************************************************************/

/** 
 * 请求读取映射的PDO条目数量。
 * 
 * @param fsm PDO映射状态机。
 * @param datagram 要使用的数据报。
 */
void ec_fsm_pdo_entry_read_state_start(
    ec_fsm_pdo_entry_t *fsm, /**< PDO映射状态机。 */
    ec_datagram_t *datagram  /**< 要使用的数据报。 */
)
{
    // 设置并执行读取请求
    ecrt_sdo_request_index(&fsm->request, fsm->target_pdo->index, 0);
    ecrt_sdo_request_read(&fsm->request);

    // 设置状态为读取条目数量
    fsm->state = ec_fsm_pdo_entry_read_state_count;
    ec_fsm_coe_transfer(fsm->fsm_coe, fsm->slave, &fsm->request);
    ec_fsm_coe_exec(fsm->fsm_coe, datagram); // 立即执行
}

/*****************************************************************************/

/** 
 * 读取映射的PDO条目数量。
 * 
 * @param fsm 有限状态机。
 * @param datagram 要使用的数据报。
 */
void ec_fsm_pdo_entry_read_state_count(
    ec_fsm_pdo_entry_t *fsm, /**< PDO映射状态机。 */
    ec_datagram_t *datagram  /**< 要使用的数据报。 */
)
{
    // 如果执行失败，则直接返回
    if (ec_fsm_coe_exec(fsm->fsm_coe, datagram))
    {
        return;
    }

    // 如果读取不成功，则记录错误并设置状态为错误
    if (!ec_fsm_coe_success(fsm->fsm_coe))
    {
        EC_SLAVE_ERR(fsm->slave, "读取映射的PDO条目数量失败。\n");
        fsm->state = ec_fsm_pdo_entry_state_error;
        return;
    }

    // 如果数据大小不正确，则记录错误并设置状态为错误
    if (fsm->request.data_size != sizeof(uint8_t))
    {
        EC_SLAVE_ERR(fsm->slave, "上传 SDO 0x%04X:%02X 的数据大小 %zu 无效。\n",
                     fsm->request.index, fsm->request.subindex, fsm->request.data_size);
        fsm->state = ec_fsm_pdo_entry_state_error;
        return;
    }

    // 读取映射的PDO条目数量
    fsm->entry_count = EC_READ_U8(fsm->request.data);

    // 输出日志，记录映射的PDO条目数量
    EC_SLAVE_DBG(fsm->slave, 1, "映射了 %u 个PDO条目。\n", fsm->entry_count);

    // 读取第一个PDO条目
    fsm->entry_pos = 1;
    ec_fsm_pdo_entry_read_action_next(fsm, datagram);
}

/*****************************************************************************/

/**
@brief 读取下一个PDO条目。
@param fsm 有限状态机
@param datagram 要使用的数据报。
@return 无
@details 
- 如果条目位置小于等于条目计数
  - 请求SDO索引(&fsm->request, fsm->target_pdo->index, fsm->entry_pos);
  - 读取SDO请求(&fsm->request);
  - 状态设置为PDO条目读取状态条目;
  - COE传输(fsm->fsm_coe, fsm->slave, &fsm->request);
  - COE执行(fsm->fsm_coe, datagram); // 立即执行
- 完成读取条目。
  - 状态设置为PDO条目结束状态。
*/
void ec_fsm_pdo_entry_read_action_next(
    ec_fsm_pdo_entry_t *fsm, /**< 有限状态机 */
    ec_datagram_t *datagram  /**< 要使用的数据报 */
)
{
    if (fsm->entry_pos <= fsm->entry_count)
    {
        ecrt_sdo_request_index(&fsm->request, fsm->target_pdo->index,
                               fsm->entry_pos);
        ecrt_sdo_request_read(&fsm->request);
        fsm->state = ec_fsm_pdo_entry_read_state_entry;
        ec_fsm_coe_transfer(fsm->fsm_coe, fsm->slave, &fsm->request);
        ec_fsm_coe_exec(fsm->fsm_coe, datagram); // 立即执行
        return;
    }

    // 完成读取条目。
    fsm->state = ec_fsm_pdo_entry_state_end;
}


/*****************************************************************************/

/**
 * @brief 读取PDO条目信息。
 * @param fsm 有限状态机。
 * @param datagram 使用的数据报。
 * @return 无。
 * @details
 * - 调用ec_fsm_coe_exec()执行COE操作，如果返回true，则函数结束。
 * - 如果ec_fsm_coe_success()返回false，则打印错误信息，将状态设置为错误状态，并返回。
 * - 如果fsm->request.data_size不等于sizeof(uint32_t)，则打印错误信息，将状态设置为错误状态。
 * - 否则，分配并初始化一个ec_pdo_entry_t结构体，将pdo_entry_info解析为index、subindex和bit_length。
 * - 如果index和subindex都为0，则设置名称为"Gap"。
 * - 打印PDO条目信息。
 * - 将pdo_entry添加到fsm->target_pdo->entries链表尾部。
 * - 更新entry_pos并调用ec_fsm_pdo_entry_read_action_next()处理下一个PDO条目。
 */
void ec_fsm_pdo_entry_read_state_entry(
    ec_fsm_pdo_entry_t *fsm, /**< 有限状态机 */
    ec_datagram_t *datagram  /**< 使用的数据报 */
)
{
    if (ec_fsm_coe_exec(fsm->fsm_coe, datagram))
    {
        return;
    }

    if (!ec_fsm_coe_success(fsm->fsm_coe))
    {
        EC_SLAVE_ERR(fsm->slave, "读取映射的PDO条目失败。\n");
        fsm->state = ec_fsm_pdo_entry_state_error;
        return;
    }

    if (fsm->request.data_size != sizeof(uint32_t))
    {
        EC_SLAVE_ERR(fsm->slave, "无效的数据大小 %zu，上传SDO 0x%04X:%02X。\n",
                     fsm->request.data_size, fsm->request.index,
                     fsm->request.subindex);
        fsm->state = ec_fsm_pdo_entry_state_error;
    }
    else
    {
        uint32_t pdo_entry_info;
        ec_pdo_entry_t *pdo_entry;

        pdo_entry_info = EC_READ_U32(fsm->request.data);

        if (!(pdo_entry = (ec_pdo_entry_t *)
                  kmalloc(sizeof(ec_pdo_entry_t), GFP_KERNEL)))
        {
            EC_SLAVE_ERR(fsm->slave, "分配PDO条目失败。\n");
            fsm->state = ec_fsm_pdo_entry_state_error;
            return;
        }

        ec_pdo_entry_init(pdo_entry);
        pdo_entry->index = pdo_entry_info >> 16;
        pdo_entry->subindex = (pdo_entry_info >> 8) & 0xFF;
        pdo_entry->bit_length = pdo_entry_info & 0xFF;

        if (!pdo_entry->index && !pdo_entry->subindex)
        {
            if (ec_pdo_entry_set_name(pdo_entry, "Gap"))
            {
                ec_pdo_entry_clear(pdo_entry);
                kfree(pdo_entry);
                fsm->state = ec_fsm_pdo_entry_state_error;
                return;
            }
        }

        EC_SLAVE_DBG(fsm->slave, 1,
                     "PDO条目 0x%04X:%02X，%u位，\"%s\"。\n",
                     pdo_entry->index, pdo_entry->subindex,
                     pdo_entry->bit_length,
                     pdo_entry->name ? pdo_entry->name : "???");

        list_add_tail(&pdo_entry->list, &fsm->target_pdo->entries);

        // 下一个PDO条目
        fsm->entry_pos++;
        ec_fsm_pdo_entry_read_action_next(fsm, datagram);
    }
}

/******************************************************************************
 * 配置 状态函数
 *****************************************************************************/

/**
 * @brief 开始PDO映射。
 * @param fsm PDO映射状态机。
 * @param datagram 用于操作的数据报。
 * @return 无。
 * @details
 * - 分配4个字节的EC SDO请求。
 * - 如果分配成功，则将状态机的状态设置为错误状态，并返回。
 * - 将映射的PDO条目计数设置为零。
 * - 设置请求数据大小为1字节。
 * - 将请求的索引设置为源PDO的索引和0。
 * - 执行写操作。
 * - 设置状态机的状态为零条目计数状态。
 * - 执行COE传输。
 * - 立即执行COE。
 */
void ec_fsm_pdo_entry_conf_state_start(
    ec_fsm_pdo_entry_t *fsm, /**< PDO映射状态机。 */
    ec_datagram_t *datagram  /**< 用于操作的数据报。 */
)
{
    if (ec_sdo_request_alloc(&fsm->request, 4))
    {
        fsm->state = ec_fsm_pdo_entry_state_error;
        return;
    }

    // 将映射的PDO条目计数设置为零
    EC_WRITE_U8(fsm->request.data, 0);
    fsm->request.data_size = 1;
    ecrt_sdo_request_index(&fsm->request, fsm->source_pdo->index, 0);
    ecrt_sdo_request_write(&fsm->request);

    EC_SLAVE_DBG(fsm->slave, 1, "将条目计数设置为零。\n");

    fsm->state = ec_fsm_pdo_entry_conf_state_zero_entry_count;
    ec_fsm_coe_transfer(fsm->fsm_coe, fsm->slave, &fsm->request);
    ec_fsm_coe_exec(fsm->fsm_coe, datagram); // 立即执行
}

/*****************************************************************************/

/**
 * @brief 处理下一个PDO条目。
 * @param fsm PDO映射状态机。
 * @param list 当前条目列表项。
 * @return 下一个PDO条目，或NULL。
 * @details
 * - 将列表项指针指向下一个条目。
 * - 如果列表项指针指向源PDO的最后一个条目，则返回NULL，表示没有下一个条目。
 * - 返回下一个PDO条目。
 */
ec_pdo_entry_t *ec_fsm_pdo_entry_conf_next_entry(
    const ec_fsm_pdo_entry_t *fsm, /**< PDO映射状态机。 */
    const struct list_head *list   /**< 当前条目列表项。 */
)
{
    list = list->next;
    if (list == &fsm->source_pdo->entries)
        return NULL; // 没有下一个条目
    return list_entry(list, ec_pdo_entry_t, list);
}

/*****************************************************************************/

/**
 * @brief 将映射的条目数设置为零。
 * @param fsm PDO映射状态机。
 * @param datagram 用于操作的数据报。
 * @return 无。
 * @details
 * - 执行COE传输。
 * - 如果COE执行失败，则打印警告信息，设置状态机的状态为错误状态，并返回。
 * - 查找第一个条目。
 * - 如果找不到条目，则打印调试信息，设置状态机的状态为结束状态，并返回。
 * - 添加第一个条目。
 * - 设置条目位置为1。
 * - 执行映射动作。
 */
void ec_fsm_pdo_entry_conf_state_zero_entry_count(
    ec_fsm_pdo_entry_t *fsm, /**< PDO映射状态机。 */
    ec_datagram_t *datagram  /**< 用于操作的数据报。 */
)
{
    if (ec_fsm_coe_exec(fsm->fsm_coe, datagram))
    {
        return;
    }

    if (!ec_fsm_coe_success(fsm->fsm_coe))
    {
        EC_SLAVE_WARN(fsm->slave, "清除PDO映射失败。\n");
        EC_SLAVE_WARN(fsm->slave, "");
        ec_fsm_pdo_entry_print(fsm);
        fsm->state = ec_fsm_pdo_entry_state_error;
        return;
    }

    // 查找第一个条目
    if (!(fsm->entry = ec_fsm_pdo_entry_conf_next_entry(
              fsm, &fsm->source_pdo->entries)))
    {

        EC_SLAVE_DBG(fsm->slave, 1, "没有要映射的条目。\n");

        fsm->state = ec_fsm_pdo_entry_state_end; // 完成
        return;
    }

    // 添加第一个条目
    fsm->entry_pos = 1;
    ec_fsm_pdo_entry_conf_action_map(fsm, datagram);
}

/*****************************************************************************/

/**
 * @brief 开始添加PDO条目。
 * 
 * @param fsm PDO映射状态机。
 * @param datagram 用于操作的数据报。
 * @return 无。
 * 
 * @details
 * - 打印调试信息，指示正在映射的PDO条目的索引、子索引、位长度和位置。
 * - 将索引、子索引和位长度组合为一个32位的值。
 * - 将值写入请求数据中，并设置数据大小为4字节。
 * - 设置请求的索引为源PDO的索引，子索引为条目位置。
 * - 执行SDO写操作。
 * - 设置状态机的状态为映射条目状态。
 * - 启动COE传输，执行请求。
 */
void ec_fsm_pdo_entry_conf_action_map(
    ec_fsm_pdo_entry_t *fsm, /**< PDO映射状态机。 */
    ec_datagram_t *datagram  /**< 用于操作的数据报。 */
)
{
    uint32_t value;

    EC_SLAVE_DBG(fsm->slave, 1, "Mapping PDO entry 0x%04X:%02X (%u bit)"
                                " at position %u.\n",
                 fsm->entry->index, fsm->entry->subindex,
                 fsm->entry->bit_length, fsm->entry_pos);

    value = fsm->entry->index << 16 | fsm->entry->subindex << 8 | fsm->entry->bit_length;
    EC_WRITE_U32(fsm->request.data, value);
    fsm->request.data_size = 4;
    ecrt_sdo_request_index(&fsm->request, fsm->source_pdo->index,
                           fsm->entry_pos);
    ecrt_sdo_request_write(&fsm->request);

    fsm->state = ec_fsm_pdo_entry_conf_state_map_entry;
    ec_fsm_coe_transfer(fsm->fsm_coe, fsm->slave, &fsm->request);
    ec_fsm_coe_exec(fsm->fsm_coe, datagram); // 立即执行
}

/*****************************************************************************/

/**
 * @brief 添加PDO条目。
 * 
 * @param fsm PDO映射状态机。
 * @param datagram 用于操作的数据报。
 * @return 无。
 * 
 * @details
 * - 检查是否执行了COE传输操作，如果是则返回。
 * - 检查COE操作是否成功，如果失败则打印警告信息，并设置状态机状态为错误状态。
 * - 如果没有更多条目可添加，则写入条目计数。
 * - 设置请求的索引为源PDO的索引，子索引为0。
 * - 执行SDO写操作。
 * - 打印调试信息，指示成功配置PDO的映射。
 * - 设置状态机的状态为结束状态。
 */
void ec_fsm_pdo_entry_conf_state_map_entry(
    ec_fsm_pdo_entry_t *fsm, /**< PDO映射状态机。 */
    ec_datagram_t *datagram  /**< 用于操作的数据报。 */
)
{
    if (ec_fsm_coe_exec(fsm->fsm_coe, datagram))
    {
        return;
    }

    if (!ec_fsm_coe_success(fsm->fsm_coe))
    {
        EC_SLAVE_WARN(fsm->slave, "Failed to map PDO entry"
                                  " 0x%04X:%02X (%u bit) to position %u.\n",
                      fsm->entry->index, fsm->entry->subindex,
                      fsm->entry->bit_length, fsm->entry_pos);
        EC_SLAVE_WARN(fsm->slave, "");
        ec_fsm_pdo_entry_print(fsm);
        fsm->state = ec_fsm_pdo_entry_state_error;
        return;
    }

    // 查找下一个条目
    if (!(fsm->entry = ec_fsm_pdo_entry_conf_next_entry(
              fsm, &fsm->entry->list)))
    {

        // 没有更多条目可添加。写入条目计数。
        EC_WRITE_U8(fsm->request.data, fsm->entry_pos);
        fsm->request.data_size = 1;
        ecrt_sdo_request_index(&fsm->request, fsm->source_pdo->index, 0);
        ecrt_sdo_request_write(&fsm->request);

        EC_SLAVE_DBG(fsm->slave, 1, "Setting number of PDO entries to %u.\n",
                     fsm->entry_pos);

        fsm->state = ec_fsm_pdo_entry_conf_state_set_entry_count;
        ec_fsm_coe_transfer(fsm->fsm_coe, fsm->slave, &fsm->request);
        ec_fsm_coe_exec(fsm->fsm_coe, datagram); // 立即执行
        return;
    }

    // 添加下一个条目
    fsm->entry_pos++;
    ec_fsm_pdo_entry_conf_action_map(fsm, datagram);
}

/*****************************************************************************/

/**
 * @brief 设置条目数量。
 * 
 * @param fsm PDO映射状态机。
 * @param datagram 用于操作的数据报。
 * @return 无。
 * 
 * @details
 * - 检查是否执行了COE传输操作，如果是则返回。
 * - 检查COE操作是否成功，如果失败则打印警告信息，并设置状态机状态为错误状态。
 * - 打印调试信息，指示成功配置PDO的映射。
 * - 设置状态机的状态为结束状态。
 */
void ec_fsm_pdo_entry_conf_state_set_entry_count(
    ec_fsm_pdo_entry_t *fsm, /**< PDO映射状态机。 */
    ec_datagram_t *datagram  /**< 用于操作的数据报。 */
)
{
    if (ec_fsm_coe_exec(fsm->fsm_coe, datagram))
    {
        return;
    }

    if (!ec_fsm_coe_success(fsm->fsm_coe))
    {
        EC_SLAVE_WARN(fsm->slave, "Failed to set number of entries.\n");
        EC_SLAVE_WARN(fsm->slave, "");
        ec_fsm_pdo_entry_print(fsm);
        fsm->state = ec_fsm_pdo_entry_state_error;
        return;
    }

    EC_SLAVE_DBG(fsm->slave, 1, "Successfully configured mapping for PDO 0x%04X.\n",
                 fsm->source_pdo->index);

    fsm->state = ec_fsm_pdo_entry_state_end; // 完成
}

/******************************************************************************
 * 通用状态函数
 *****************************************************************************/

/**
@brief 将PDO映射状态机设置为错误状态。
@param fsm PDO映射状态机。
@param datagram 要使用的数据报。
@return 无返回值。
@details
- 设置状态为错误状态。
*/

void ec_fsm_pdo_entry_state_error(
    ec_fsm_pdo_entry_t *fsm, /**< PDO映射状态机。 */
    ec_datagram_t *datagram  /**< 要使用的数据报。 */
)
{
}

/*****************************************************************************/

/**
@brief 将PDO映射状态机设置为结束状态。
@param fsm PDO映射状态机。
@param datagram 要使用的数据报。
@return 无返回值。
@details
- 设置状态为结束状态。
*/

void ec_fsm_pdo_entry_state_end(
    ec_fsm_pdo_entry_t *fsm, /**< PDO映射状态机。 */
    ec_datagram_t *datagram  /**< 要使用的数据报。 */
)
{
}

/*****************************************************************************/

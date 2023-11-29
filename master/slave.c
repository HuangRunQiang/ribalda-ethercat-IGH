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

/**
   \file
   EtherCAT slave methods.
*/

/*****************************************************************************/

#include <linux/module.h>
#include <linux/delay.h>

#include "globals.h"
#include "datagram.h"
#include "master.h"
#include "slave_config.h"

#include "slave.h"

/*****************************************************************************/

extern const ec_code_msg_t al_status_messages[];

/*****************************************************************************/

char *ec_slave_sii_string(ec_slave_t *, unsigned int);

/*****************************************************************************/
/**

@brief 从站构造函数。

@param slave EtherCAT从站对象指针。

@param master EtherCAT主站对象指针。

@param dev_idx 设备索引。

@param ring_position 环位置。

@param station_address 配置的站地址。
*/
void ec_slave_init(
ec_slave_t *slave, /< EtherCAT从站 */
ec_master_t *master, /< EtherCAT主站 */
ec_device_index_t dev_idx, /< 设备索引 */
uint16_t ring_position, /< 环位置 */
uint16_t station_address /**< 配置的站地址 */
)
{
unsigned int i;

slave->master = master;
slave->device_index = dev_idx;

slave->ring_position = ring_position;
slave->station_address = station_address;
slave->effective_alias = 0x0000;
#ifdef EC_SII_CACHE
slave->effective_vendor_id = 0x00000000;
slave->effective_product_code = 0x00000000;
slave->effective_revision_number = 0x00000000;
slave->effective_serial_number = 0x00000000;
#endif
slave->config = NULL;
slave->requested_state = EC_SLAVE_STATE_PREOP;
slave->current_state = EC_SLAVE_STATE_UNKNOWN;
slave->last_al_error = 0;
slave->error_flag = 0;
slave->force_config = 0;
slave->reboot = 0;
slave->configured_rx_mailbox_offset = 0x0000;
slave->configured_rx_mailbox_size = 0x0000;
slave->configured_tx_mailbox_offset = 0x0000;
slave->configured_tx_mailbox_size = 0x0000;

slave->base_type = 0;
slave->base_revision = 0;
slave->base_build = 0;
slave->base_fmmu_count = 0;
slave->base_sync_count = 0;

for (i = 0; i < EC_MAX_PORTS; i++)
{
slave->ports[i].desc = EC_PORT_NOT_IMPLEMENTED;

 slave->ports[i].link.link_up = 0;
 slave->ports[i].link.loop_closed = 1;
 slave->ports[i].link.signal_detected = 0;
 slave->ports[i].link.bypassed = 0;

 slave->ports[i].receive_time = 0U;

 slave->ports[i].next_slave = NULL;
 slave->ports[i].delay_to_next_dc = 0U;
#ifdef EC_LOOP_CONTROL
slave->ports[i].state = EC_SLAVE_PORT_DOWN;
slave->ports[i].link_detection_jiffies = 0;
#endif
}
slave->upstream_port = 0;

slave->base_fmmu_bit_operation = 0;
slave->base_dc_supported = 0;
slave->base_dc_range = EC_DC_32;
slave->has_dc_system_time = 0;
slave->transmission_delay = 0U;

slave->vendor_words = NULL;
slave->sii_image = NULL;

INIT_LIST_HEAD(&slave->sdo_dictionary);

slave->scan_required = 1;
slave->sdo_dictionary_fetched = 0;
slave->jiffies_preop = 0;

INIT_LIST_HEAD(&slave->sdo_requests);
INIT_LIST_HEAD(&slave->reg_requests);
INIT_LIST_HEAD(&slave->foe_requests);
INIT_LIST_HEAD(&slave->soe_requests);
INIT_LIST_HEAD(&slave->eoe_requests);
INIT_LIST_HEAD(&slave->mbg_requests);
INIT_LIST_HEAD(&slave->dict_requests);

// 创建状态机对象
ec_fsm_slave_init(&slave->fsm, slave);

slave->read_mbox_busy = 0;
rt_mutex_init(&slave->mbox_sem);
#ifdef EC_EOE
ec_mbox_data_init(&slave->mbox_eoe_frag_data);
ec_mbox_data_init(&slave->mbox_eoe_init_data);
#endif
ec_mbox_data_init(&slave->mbox_coe_data);
ec_mbox_data_init(&slave->mbox_foe_data);
ec_mbox_data_init(&slave->mbox_soe_data);
ec_mbox_data_init(&slave->mbox_voe_data);
ec_mbox_data_init(&slave->mbox_mbg_data);

slave->valid_mbox_data = 0;
}

/**
 * @brief 初始化SII图像
 * @param sii_image SII图像
 */
void ec_slave_sii_image_init(ec_sii_image_t *sii_image)
{
    unsigned int i;

    sii_image->words = NULL;
    sii_image->nwords = 0;

    sii_image->sii.alias = 0x0000;
    sii_image->sii.vendor_id = 0x00000000;
    sii_image->sii.product_code = 0x00000000;
    sii_image->sii.revision_number = 0x00000000;
    sii_image->sii.serial_number = 0x00000000;
    sii_image->sii.boot_rx_mailbox_offset = 0x0000;
    sii_image->sii.boot_rx_mailbox_size = 0x0000;
    sii_image->sii.boot_tx_mailbox_offset = 0x0000;
    sii_image->sii.boot_tx_mailbox_size = 0x0000;
    sii_image->sii.std_rx_mailbox_offset = 0x0000;
    sii_image->sii.std_rx_mailbox_size = 0x0000;
    sii_image->sii.std_tx_mailbox_offset = 0x0000;
    sii_image->sii.std_tx_mailbox_size = 0x0000;
    sii_image->sii.mailbox_protocols = 0;
    sii_image->sii.strings = NULL;
    sii_image->sii.string_count = 0;

    sii_image->sii.has_general = 0;
    sii_image->sii.group = NULL;
    sii_image->sii.image = NULL;
    sii_image->sii.order = NULL;
    sii_image->sii.name = NULL;
    memset(&sii_image->sii.coe_details, 0x00, sizeof(ec_sii_coe_details_t));
    memset(&sii_image->sii.general_flags, 0x00, sizeof(ec_sii_general_flags_t));
    sii_image->sii.current_on_ebus = 0;

    sii_image->sii.syncs = NULL;
    sii_image->sii.sync_count = 0;

    INIT_LIST_HEAD(&sii_image->sii.pdos);

    for (i = 0; i < EC_MAX_PORTS; i++)
    {
        sii_image->sii.physical_layer[i] = 0xFF;
    }
}

/*****************************************************************************/

/**
 * @brief 清除邮箱锁定状态
 * @param slave EtherCAT从站
 */
void ec_read_mbox_lock_clear(ec_slave_t *slave)
{
    rt_mutex_lock(&slave->mbox_sem);
    slave->read_mbox_busy = 0;
    rt_mutex_unlock(&slave->mbox_sem);
}

/*****************************************************************************/

/**
 * @brief 返回当前邮箱锁定状态并在未锁定时锁定它
 * @param slave EtherCAT从站
 * @return 当前邮箱锁定状态
 */
int ec_read_mbox_locked(ec_slave_t *slave)
{
    int rc;

    rt_mutex_lock(&slave->mbox_sem);
    rc = slave->read_mbox_busy;
    if (!slave->read_mbox_busy)
    {
        slave->read_mbox_busy = 1;
    }
    rt_mutex_unlock(&slave->mbox_sem);
    return rc;
}


/*****************************************************************************/

/**
 * @brief 从站析构函数
 * 清除并释放从站对象。中止所有挂起的请求，并释放相关的资源，包括SDO、邮箱响应数据等。
 *
 * @param slave EtherCAT从站
 */
void ec_slave_clear(ec_slave_t *slave)
{
    ec_sdo_t *sdo, *next_sdo;

    // 中止所有挂起的请求

    while (!list_empty(&slave->sdo_requests))
    {
        ec_sdo_request_t *request =
            list_entry(slave->sdo_requests.next, ec_sdo_request_t, list);
        list_del_init(&request->list); // 出队
        EC_SLAVE_WARN(slave, "丢弃SDO请求，从站即将被删除。\n");
        request->state = EC_INT_REQUEST_FAILURE;
    }

    while (!list_empty(&slave->reg_requests))
    {
        ec_reg_request_t *reg =
            list_entry(slave->reg_requests.next, ec_reg_request_t, list);
        list_del_init(&reg->list); // 出队
        EC_SLAVE_WARN(slave, "丢弃寄存器请求，从站即将被删除。\n");
        reg->state = EC_INT_REQUEST_FAILURE;
    }

    while (!list_empty(&slave->foe_requests))
    {
        ec_foe_request_t *request =
            list_entry(slave->foe_requests.next, ec_foe_request_t, list);
        list_del_init(&request->list); // 出队
        EC_SLAVE_WARN(slave, "丢弃FoE请求，从站即将被删除。\n");
        request->state = EC_INT_REQUEST_FAILURE;
    }

    while (!list_empty(&slave->soe_requests))
    {
        ec_soe_request_t *request =
            list_entry(slave->soe_requests.next, ec_soe_request_t, list);
        list_del_init(&request->list); // 出队
        EC_SLAVE_WARN(slave, "丢弃SoE请求，从站即将被删除。\n");
        request->state = EC_INT_REQUEST_FAILURE;
    }

    while (!list_empty(&slave->eoe_requests))
    {
        ec_eoe_request_t *request =
            list_entry(slave->eoe_requests.next, ec_eoe_request_t, list);
        list_del_init(&request->list); // 出队
        EC_SLAVE_WARN(slave, "丢弃EoE请求，从站即将被删除。\n");
        request->state = EC_INT_REQUEST_FAILURE;
    }

    while (!list_empty(&slave->mbg_requests))
    {
        ec_mbg_request_t *request =
            list_entry(slave->mbg_requests.next, ec_mbg_request_t, list);
        list_del_init(&request->list); // 出队
        EC_SLAVE_WARN(slave, "丢弃MBox Gateway请求，从站即将被删除。\n");
        request->state = EC_INT_REQUEST_FAILURE;
    }

    while (!list_empty(&slave->dict_requests))
    {
        ec_dict_request_t *request =
            list_entry(slave->dict_requests.next, ec_dict_request_t, list);
        list_del_init(&request->list); // 出队
        EC_SLAVE_WARN(slave, "丢弃字典请求，从站即将被删除。\n");
        request->state = EC_INT_REQUEST_FAILURE;
    }

    wake_up_all(&slave->master->request_queue);

    if (slave->config)
    {
        ec_slave_config_detach(slave->config);
    }

    // 释放所有SDO

    list_for_each_entry_safe(sdo, next_sdo, &slave->sdo_dictionary, list)
    {
        list_del(&sdo->list);
        ec_sdo_clear(sdo);
        kfree(sdo);
    }

    if (slave->vendor_words)
    {
        kfree(slave->vendor_words);
        slave->vendor_words = NULL;
    }

    // 释放邮箱响应数据
#ifdef EC_EOE
    ec_mbox_data_clear(&slave->mbox_eoe_frag_data);
    ec_mbox_data_clear(&slave->mbox_eoe_init_data);
#endif
    ec_mbox_data_clear(&slave->mbox_coe_data);
    ec_mbox_data_clear(&slave->mbox_foe_data);
    ec_mbox_data_clear(&slave->mbox_soe_data);
    ec_mbox_data_clear(&slave->mbox_voe_data);
    ec_mbox_data_clear(&slave->mbox_mbg_data);

    ec_fsm_slave_clear(&slave->fsm);
}


/*****************************************************************************/

/**
 * @brief 清除同步管理器数组。
 * 清除同步管理器数组，释放相关资源。
 *
 * @param slave EtherCAT从站
 */
void ec_slave_clear_sync_managers(ec_slave_t *slave)
{
    unsigned int i;

    if (slave->sii_image && slave->sii_image->sii.syncs)
    {
        for (i = 0; i < slave->sii_image->sii.sync_count; i++)
        {
            ec_sync_clear(&slave->sii_image->sii.syncs[i]);
        }
        kfree(slave->sii_image->sii.syncs);
        slave->sii_image->sii.syncs = NULL;
    }
}

/*****************************************************************************/

/**
 * @brief 设置从站的数据链路状态。
 * 设置从站的数据链路状态，包括链接状态、回环状态和信号检测状态。
 *
 * @param slave      EtherCAT从站
 * @param new_state  寄存器0x0110-0x0111的内容
 */
void ec_slave_set_dl_status(ec_slave_t *slave, uint16_t new_state)
{
    unsigned int i;
    uint8_t state;

    for (i = 0; i < EC_MAX_PORTS; i++)
    {
        // 链接状态
        state = new_state & (1 << (4 + i)) ? 1 : 0;
        if (slave->ports[i].link.link_up != state)
        {
            EC_SLAVE_DBG(slave, 1, "端口 %u 链接状态改变为 %s。\n",
                         i, state ? "上线" : "下线");
            slave->ports[i].link.link_up = state;
        }

        // 回环状态
        state = new_state & (1 << (8 + i * 2)) ? 1 : 0;
        if (slave->ports[i].link.loop_closed != state)
        {
            EC_SLAVE_DBG(slave, 1, "端口 %u 回环状态改变为 %s。\n",
                         i, state ? "闭合" : "打开");
            slave->ports[i].link.loop_closed = state;
        }

        // 信号检测状态
        state = new_state & (1 << (9 + i * 2)) ? 1 : 0;
        if (slave->ports[i].link.signal_detected != state)
        {
            EC_SLAVE_DBG(slave, 1, "端口 %u 信号状态改变为 %s。\n",
                         i, state ? "有信号" : "无信号");
            slave->ports[i].link.signal_detected = state;
        }
    }
}


/*****************************************************************************/

/**
 * @brief 设置从站的应用状态。
 * 设置从站的应用状态。
 *
 * @param slave      EtherCAT从站
 * @param new_state  新的应用状态
 */
void ec_slave_set_al_status(ec_slave_t *slave, ec_slave_state_t new_state)
{
    if (new_state != slave->current_state)
    {
        if (slave->master->debug_level)
        {
            char old_state[EC_STATE_STRING_SIZE];
            char cur_state[EC_STATE_STRING_SIZE];
            ec_state_string(slave->current_state, old_state, 0);
            ec_state_string(new_state, cur_state, 0);
            EC_SLAVE_DBG(slave, 0, "%s -> %s。\n", old_state, cur_state);
        }
        slave->current_state = new_state;
    }
}

/*****************************************************************************/

/**
 * @brief 请求从站状态并重置错误标志。
 * 请求从站状态并重置错误标志。
 *
 * @param slave  EtherCAT从站
 * @param state  新的状态
 */
void ec_slave_request_state(ec_slave_t *slave, ec_slave_state_t state)
{
    slave->requested_state = state;
    slave->error_flag = 0;
}

/*****************************************************************************/

/**
 * @brief 请求从站重启（某些从站可能会忽略该请求）。
 * 请求从站重启（某些从站可能会忽略该请求）。
 *
 * @param slave  EtherCAT从站
 */
void ec_slave_request_reboot(ec_slave_t *slave)
{
    slave->reboot = 1;
}

/*****************************************************************************/

/**
 * 从STRING类别中获取数据。
 * \todo 范围检查
 * 成功返回0，否则返回 < 0。
 *
 * @param slave      EtherCAT从站
 * @param data       类别数据
 * @param data_size  字节数
 * @return           成功返回0，否则返回 < 0
 */
int ec_slave_fetch_sii_strings(ec_slave_t *slave, const uint8_t *data, size_t data_size)
{
    int i, err;
    size_t size;
    off_t offset;

    if (!slave->sii_image)
    {
        EC_SLAVE_ERR(slave, "SII数据未附加！\n");
        return -EINVAL;
    }

    slave->sii_image->sii.string_count = data[0];

    if (slave->sii_image->sii.string_count)
    {
        if (!(slave->sii_image->sii.strings =
                  kmalloc(sizeof(char *) * slave->sii_image->sii.string_count, GFP_KERNEL)))
        {
            EC_SLAVE_ERR(slave, "无法分配字符串数组内存。\n");
            err = -ENOMEM;
            goto out_zero;
        }

        offset = 1;
        for (i = 0; i < slave->sii_image->sii.string_count; i++)
        {
            size = data[offset];
            // 一次性分配字符串结构和数据的内存
            if (!(slave->sii_image->sii.strings[i] =
                      kmalloc(sizeof(char) * size + 1, GFP_KERNEL)))
            {
                EC_SLAVE_ERR(slave, "无法分配字符串内存。\n");
                err = -ENOMEM;
                goto out_free;
            }
            memcpy(slave->sii_image->sii.strings[i], data + offset + 1, size);
            slave->sii_image->sii.strings[i][size] = 0x00; // 追加二进制零
            offset += 1 + size;
        }
    }

    return 0;

out_free:
    for (i--; i >= 0; i--)
        kfree(slave->sii_image->sii.strings[i]);
    kfree(slave->sii_image->sii.strings);
    slave->sii_image->sii.strings = NULL;
out_zero:
    slave->sii_image->sii.string_count = 0;
    return err;
}


/*****************************************************************************/

/**
 * 从GENERAL类别中获取数据。
 * 成功返回0，否则返回 < 0。
 *
 * @param slave      EtherCAT从站
 * @param data       类别数据
 * @param data_size  字节数
 * @return           成功返回0，否则返回 < 0
 */
int ec_slave_fetch_sii_general(ec_slave_t *slave, const uint8_t *data, size_t data_size)
{
    unsigned int i;
    uint8_t flags;

    if (data_size != 32)
    {
        EC_SLAVE_ERR(slave, "GENERAL类别的大小错误（%zu/32）。\n", data_size);
        return -EINVAL;
    }

    if (!slave->sii_image)
    {
        EC_SLAVE_ERR(slave, "SII数据未附加！\n");
        return -EINVAL;
    }

    slave->sii_image->sii.group = ec_slave_sii_string(slave, data[0]);
    slave->sii_image->sii.image = ec_slave_sii_string(slave, data[1]);
    slave->sii_image->sii.order = ec_slave_sii_string(slave, data[2]);
    slave->sii_image->sii.name = ec_slave_sii_string(slave, data[3]);

    for (i = 0; i < 4; i++)
        slave->sii_image->sii.physical_layer[i] =
            (data[4] & (0x03 << (i * 2))) >> (i * 2);

    // 读取CoE详细信息
    flags = EC_READ_U8(data + 5);
    slave->sii_image->sii.coe_details.enable_sdo = (flags >> 0) & 0x01;
    slave->sii_image->sii.coe_details.enable_sdo_info = (flags >> 1) & 0x01;
    slave->sii_image->sii.coe_details.enable_pdo_assign = (flags >> 2) & 0x01;
    slave->sii_image->sii.coe_details.enable_pdo_configuration = (flags >> 3) & 0x01;
    slave->sii_image->sii.coe_details.enable_upload_at_startup = (flags >> 4) & 0x01;
    slave->sii_image->sii.coe_details.enable_sdo_complete_access = (flags >> 5) & 0x01;

    // 读取通用标志
    flags = EC_READ_U8(data + 0x000B);
    slave->sii_image->sii.general_flags.enable_safeop = (flags >> 0) & 0x01;
    slave->sii_image->sii.general_flags.enable_not_lrw = (flags >> 1) & 0x01;

    slave->sii_image->sii.current_on_ebus = EC_READ_S16(data + 0x0C);
    slave->sii_image->sii.has_general = 1;
    return 0;
}

/*****************************************************************************/

/**
 * 从SYNC MANAGER类别中获取数据。
 *
 * 将类别中描述的同步管理器追加到现有的同步管理器中。
 *
 * 成功返回0，否则返回 < 0。
 *
 * @param slave      EtherCAT从站
 * @param data       类别数据
 * @param data_size  字节数
 * @return           成功返回0，否则返回 < 0
 */
int ec_slave_fetch_sii_syncs(ec_slave_t *slave, const uint8_t *data, size_t data_size)
{
    unsigned int i, count, total_count;
    ec_sync_t *sync;
    size_t memsize;
    ec_sync_t *syncs;
    uint8_t index;

    // 一个同步管理器结构体长度为4个字
    if (data_size % 8)
    {
        EC_SLAVE_ERR(slave, "无效的SII同步管理器类别大小 %zu。\n", data_size);
        return -EINVAL;
    }

    if (!slave->sii_image)
    {
        EC_SLAVE_ERR(slave, "SII数据未附加！\n");
        return -EINVAL;
    }

    count = data_size / 8;

    if (count)
    {
        total_count = count + slave->sii_image->sii.sync_count;
        if (total_count > EC_MAX_SYNC_MANAGERS)
        {
            EC_SLAVE_ERR(slave, "超过最大同步管理器数量！\n");
            return -EOVERFLOW;
        }
        memsize = sizeof(ec_sync_t) * total_count;
        if (!(syncs = kmalloc(memsize, GFP_KERNEL)))
        {
            EC_SLAVE_ERR(slave, "无法为同步管理器分配 %zu 字节内存。\n", memsize);
            return -ENOMEM;
        }

        for (i = 0; i < slave->sii_image->sii.sync_count; i++)
            ec_sync_init_copy(syncs + i, slave->sii_image->sii.syncs + i);

        // 初始化新的同步管理器
        for (i = 0; i < count; i++, data += 8)
        {
            index = i + slave->sii_image->sii.sync_count;
            sync = &syncs[index];

            ec_sync_init(sync, slave);
            sync->physical_start_address = EC_READ_U16(data);
            sync->default_length = EC_READ_U16(data + 2);
            sync->control_register = EC_READ_U8(data + 4);
            sync->enable = EC_READ_U8(data + 6);
        }

        if (slave->sii_image->sii.syncs)
            kfree(slave->sii_image->sii.syncs);
        slave->sii_image->sii.syncs = syncs;
        slave->sii_image->sii.sync_count = total_count;
    }

    return 0;
}


/*****************************************************************************/

/**
 * 从[RT]xPDO类别中获取数据。
 * 成功返回0，否则返回 < 0。
 *
 * @param slave      EtherCAT从站
 * @param data       类别数据
 * @param data_size  字节数
 * @param dir        PDO方向
 * @return           成功返回0，否则返回 < 0
 */
int ec_slave_fetch_sii_pdos(ec_slave_t *slave, const uint8_t *data, size_t data_size, ec_direction_t dir)
{
    int ret;
    ec_pdo_t *pdo;
    ec_pdo_entry_t *entry;
    unsigned int entry_count, i;

    if (!slave->sii_image)
    {
        EC_SLAVE_ERR(slave, "SII数据未附加！\n");
        return -EINVAL;
    }

    while (data_size >= 8)
    {
        if (!(pdo = kmalloc(sizeof(ec_pdo_t), GFP_KERNEL)))
        {
            EC_SLAVE_ERR(slave, "无法分配PDO内存。\n");
            return -ENOMEM;
        }

        ec_pdo_init(pdo);
        pdo->index = EC_READ_U16(data);
        entry_count = EC_READ_U8(data + 2);
        pdo->sync_index = EC_READ_U8(data + 3);
        ret = ec_pdo_set_name(pdo,
                              ec_slave_sii_string(slave, EC_READ_U8(data + 5)));
        if (ret)
        {
            ec_pdo_clear(pdo);
            kfree(pdo);
            return ret;
        }
        list_add_tail(&pdo->list, &slave->sii_image->sii.pdos);

        data_size -= 8;
        data += 8;

        for (i = 0; i < entry_count; i++)
        {
            if (!(entry = kmalloc(sizeof(ec_pdo_entry_t), GFP_KERNEL)))
            {
                EC_SLAVE_ERR(slave, "无法分配PDO条目内存。\n");
                return -ENOMEM;
            }

            ec_pdo_entry_init(entry);
            entry->index = EC_READ_U16(data);
            entry->subindex = EC_READ_U8(data + 2);
            ret = ec_pdo_entry_set_name(entry,
                                        ec_slave_sii_string(slave, EC_READ_U8(data + 3)));
            if (ret)
            {
                ec_pdo_entry_clear(entry);
                kfree(entry);
                return ret;
            }
            entry->bit_length = EC_READ_U8(data + 5);
            list_add_tail(&entry->list, &pdo->entries);

            data_size -= 8;
            data += 8;
        }

        // 如果同步管理器索引为正数，则默认映射PDO
        if (pdo->sync_index >= 0)
        {
            ec_sync_t *sync;

            if (!(sync = ec_slave_get_sync(slave, pdo->sync_index)))
            {
                EC_SLAVE_ERR(slave, "PDO 0x%04X 的SM索引 %i 无效。",
                             pdo->index, pdo->sync_index);
                return -ENOENT;
            }

            ret = ec_pdo_list_add_pdo_copy(&sync->pdos, pdo);
            if (ret)
                return ret;
        }
    }

    return 0;
}


/*****************************************************************************/

/**
 * 在字符串列表中搜索索引。
 * 成功返回0，否则返回 < 0。
 *
 * @param slave  EtherCAT从站
 * @param index  字符串索引
 * @return       成功返回字符串指针，否则返回NULL
 */
char *ec_slave_sii_string(ec_slave_t *slave, unsigned int index)
{
    if (!index--)
        return NULL;

    if (!slave->sii_image)
    {
        EC_SLAVE_ERR(slave, "SII数据未附加！\n");
        return NULL;
    }

    if (index >= slave->sii_image->sii.string_count)
    {
        EC_SLAVE_DBG(slave, 1, "未找到字符串 %u。\n", index);
        return NULL;
    }

    return slave->sii_image->sii.strings[index];
}

/*****************************************************************************/

/**
 * 根据索引获取同步管理器。
 *
 * @param slave       EtherCAT从站
 * @param sync_index  同步管理器索引
 * @return            同步管理器指针，如果不存在则返回NULL
 */
ec_sync_t *ec_slave_get_sync(ec_slave_t *slave, uint8_t sync_index)
{
    if (!slave->sii_image)
    {
        EC_SLAVE_ERR(slave, "SII数据未附加！\n");
        return NULL;
    }

    if (sync_index < slave->sii_image->sii.sync_count)
    {
        return &slave->sii_image->sii.syncs[sync_index];
    }
    else
    {
        return NULL;
    }
}

/*****************************************************************************/

/**
 * 计算字典中的SDO和条目总数。
 *
 * @param slave         EtherCAT从站
 * @param sdo_count     SDO数量
 * @param entry_count   条目总数
 */
void ec_slave_sdo_dict_info(const ec_slave_t *slave, unsigned int *sdo_count, unsigned int *entry_count)
{
    unsigned int sdos = 0, entries = 0;
    ec_sdo_t *sdo;
    ec_sdo_entry_t *entry;

    list_for_each_entry(sdo, &slave->sdo_dictionary, list)
    {
        sdos++;
        list_for_each_entry(entry, &sdo->entries, list)
        {
            entries++;
        }
    }

    *sdo_count = sdos;
    *entry_count = entries;
}

/*****************************************************************************/

/**
 * 从字典中获取一个SDO。
 * 返回所需的SDO，如果不存在则返回NULL。
 *
 * @param slave  EtherCAT从站
 * @param index  SDO索引
 * @return       SDO指针，如果不存在则返回NULL
 */
ec_sdo_t *ec_slave_get_sdo(ec_slave_t *slave, uint16_t index)
{
    ec_sdo_t *sdo;

    list_for_each_entry(sdo, &slave->sdo_dictionary, list)
    {
        if (sdo->index != index)
            continue;
        return sdo;
    }

    return NULL;
}


/*****************************************************************************/
/**
 * 从字典中获取一个SDO。
 * 常量版本。
 * 返回所需的SDO，如果不存在则返回NULL。
 *
 * @param slave  EtherCAT从站
 * @param index  SDO索引
 * @return       SDO指针，如果不存在则返回NULL
 */
const ec_sdo_t *ec_slave_get_sdo_const(const ec_slave_t *slave, uint16_t index)
{
    const ec_sdo_t *sdo;

    list_for_each_entry(sdo, &slave->sdo_dictionary, list)
    {
        if (sdo->index != index)
            continue;
        return sdo;
    }

    return NULL;
}

/*****************************************************************************/

/**
 * 根据列表中的位置获取一个SDO。
 * 返回所需的SDO，如果不存在则返回NULL。
 *
 * @param slave         EtherCAT从站
 * @param sdo_position  SDO列表位置
 * @return              SDO指针，如果不存在则返回NULL
 */
const ec_sdo_t *ec_slave_get_sdo_by_pos_const(const ec_slave_t *slave, uint16_t sdo_position)
{
    const ec_sdo_t *sdo;

    list_for_each_entry(sdo, &slave->sdo_dictionary, list)
    {
        if (sdo_position--)
            continue;
        return sdo;
    }

    return NULL;
}

/*****************************************************************************/

/**
 * 获取字典中的SDO数量。
 * 返回SDO数量。
 *
 * @param slave  EtherCAT从站
 * @return       SDO数量
 */
uint16_t ec_slave_sdo_count(const ec_slave_t *slave)
{
    const ec_sdo_t *sdo;
    uint16_t count = 0;

    list_for_each_entry(sdo, &slave->sdo_dictionary, list)
    {
        count++;
    }

    return count;
}

/*****************************************************************************/

/**
 * 查找映射的PDO。
 * 返回所需的PDO对象，如果不存在则返回NULL。
 *
 * @param slave  从站
 * @param index  要查找的PDO索引
 * @return       PDO指针，如果不存在则返回NULL
 */
const ec_pdo_t *ec_slave_find_pdo(const ec_slave_t *slave, uint16_t index)
{
    unsigned int i;
    const ec_sync_t *sync;
    const ec_pdo_t *pdo;

    if (!slave->sii_image)
    {
        EC_SLAVE_ERR(slave, "SII数据未附加！\n");
        return NULL;
    }

    for (i = 0; i < slave->sii_image->sii.sync_count; i++)
    {
        sync = &slave->sii_image->sii.syncs[i];

        if (!(pdo = ec_pdo_list_find_pdo_const(&sync->pdos, index)))
            continue;

        return pdo;
    }

    return NULL;
}

/*****************************************************************************/

/**
 * 查找PDO和其条目的名称。
 *
 * @param slave  从站
 * @param pdo    PDO对象
 */
void ec_slave_find_names_for_pdo(ec_slave_t *slave, ec_pdo_t *pdo)
{
    const ec_sdo_t *sdo;
    ec_pdo_entry_t *pdo_entry;
    const ec_sdo_entry_t *sdo_entry;

    list_for_each_entry(sdo, &slave->sdo_dictionary, list)
    {
        if (sdo->index == pdo->index)
        {
            ec_pdo_set_name(pdo, sdo->name);
        }
        else
        {
            list_for_each_entry(pdo_entry, &pdo->entries, list)
            {
                if (sdo->index == pdo_entry->index)
                {
                    sdo_entry = ec_sdo_get_entry_const(sdo, pdo_entry->subindex);
                    if (sdo_entry)
                    {
                        ec_pdo_entry_set_name(pdo_entry, sdo_entry->description);
                    }
                }
            }
        }
    }
}

/*****************************************************************************/

/**
 * 附加PDO名称。
 *
 * @param slave  从站
 */
void ec_slave_attach_pdo_names(ec_slave_t *slave)
{
    unsigned int i;
    ec_sync_t *sync;
    ec_pdo_t *pdo;

    if (!slave->sii_image)
    {
        EC_SLAVE_ERR(slave, "SII数据未附加！\n");
        return;
    }

    for (i = 0; i < slave->sii_image->sii.sync_count; i++)
    {
        sync = slave->sii_image->sii.syncs + i;
        list_for_each_entry(pdo, &sync->pdos.list, list)
        {
            ec_slave_find_names_for_pdo(slave, pdo);
        }
    }
}

/*****************************************************************************/

/**
 * 返回给定端口的前一个连接端口。
 *
 * @param slave       EtherCAT从站
 * @param port_index  端口索引
 * @return            端口索引
 */
unsigned int ec_slave_get_previous_port(ec_slave_t *slave, unsigned int port_index)
{
    static const unsigned int prev_table[EC_MAX_PORTS] = {
        2, 3, 1, 0};

    if (port_index >= EC_MAX_PORTS)
    {
        EC_SLAVE_WARN(slave, "%s(port_index=%u): 无效的端口索引！\n",
                      __func__, port_index);
    }

    do
    {
        port_index = prev_table[port_index];
        if (slave->ports[port_index].next_slave)
        {
            return port_index;
        }
    } while (port_index != slave->upstream_port);

    return slave->upstream_port;
}

/*****************************************************************************/

/**
 * 返回给定端口的前一个连接且未旁路的端口。
 *
 * @param slave       EtherCAT从站
 * @param port_index  端口索引
 * @return            端口索引
 */
unsigned int ec_slave_get_previous_normal_port(ec_slave_t *slave, unsigned int port_index)
{
    static const unsigned int prev_table[EC_MAX_PORTS] = {
        2, 3, 1, 0};

    if (port_index >= EC_MAX_PORTS)
    {
        EC_SLAVE_WARN(slave, "%s(port_index=%u): 无效的端口索引！\n",
                      __func__, port_index);
    }

    do
    {
        port_index = prev_table[port_index];
        if (!slave->ports[port_index].link.bypassed &&
            slave->ports[port_index].next_slave)
        {
            return port_index;
        }
    } while (port_index != slave->upstream_port);

    return slave->upstream_port;
}

/*****************************************************************************/

/**
 * 返回给定端口的下一个连接端口。
 *
 * @param slave       EtherCAT从站
 * @param port_index  端口索引
 * @return            端口索引
 */
unsigned int ec_slave_get_next_port(ec_slave_t *slave, unsigned int port_index)
{
    static const unsigned int next_table[EC_MAX_PORTS] = {
        3, 2, 0, 1};

    if (port_index >= EC_MAX_PORTS)
    {
        EC_SLAVE_WARN(slave, "%s(port_index=%u): 无效的端口索引！\n",
                      __func__, port_index);
    }

    do
    {
        port_index = next_table[port_index];
        if (slave->ports[port_index].next_slave)
        {
            return port_index;
        }
    } while (port_index != slave->upstream_port);

    return slave->upstream_port;
}

/*****************************************************************************/

/**
 * 计算0-3号端口中哪个端口似乎是上游端口。
 *
 * @param slave  EtherCAT从站
 */
void ec_slave_calc_upstream_port(ec_slave_t *slave)
{
    int i, replace;

    // 最初假设为端口0（正常连接顺序）
    slave->upstream_port = 0;
    replace = slave->ports[0].link.loop_closed || slave->ports[0].link.bypassed;

    if (!slave->base_dc_supported)
    {
        // 对于非DC从站，我们无法做出更好的判断；假设我们是正确的
        EC_SLAVE_DBG(slave, 1, "不支持DC；假设上游端口为0。\n");
        return;
    }

    // 任何开放且未旁路的端口，如果它的接收时间较低，则是上游端口的更好候选项
    for (i = 1; i < EC_MAX_PORTS; ++i)
    {
        if (!slave->ports[i].link.loop_closed &&
            !slave->ports[i].link.bypassed)
        {
            int32_t diff = slave->ports[i].receive_time -
                           slave->ports[slave->upstream_port].receive_time;
            if (diff < 0 || replace)
            {
                slave->upstream_port = i;
                replace = 0;
            }
        }
    }
    EC_SLAVE_DBG(slave, 1, "上游端口 = %u\n", slave->upstream_port);
}


/*****************************************************************************/
/**
 * 计算连接的1-3号端口的往返时间总和。
 *
 * @param slave  EtherCAT从站
 * @return       往返时间总和（单位：ns）
 */
uint32_t ec_slave_calc_rtt_sum(ec_slave_t *slave)
{
    uint32_t rtt_sum = 0, rtt;
    unsigned int port_index = ec_slave_get_next_port(slave, slave->upstream_port);

    while (port_index != slave->upstream_port)
    {
        unsigned int prev_index = ec_slave_get_previous_normal_port(slave, port_index);

        if (!slave->ports[port_index].link.bypassed)
        {
            rtt = slave->ports[port_index].receive_time - slave->ports[prev_index].receive_time;
            rtt_sum += rtt;
        }
        port_index = ec_slave_get_next_port(slave, port_index);
    }

    return rtt_sum;
}

/*****************************************************************************/

/**
 * 查找下一个支持DC延迟测量的从站。
 *
 * @param slave  EtherCAT从站
 * @return       下一个支持DC的从站，如果不存在则返回NULL
 */
ec_slave_t *ec_slave_find_next_dc_slave(ec_slave_t *slave)
{
    unsigned int port_index;
    ec_slave_t *dc_slave = NULL;

    if (slave->base_dc_supported)
    {
        dc_slave = slave;
    }
    else
    {
        port_index = ec_slave_get_next_port(slave, slave->upstream_port);

        while (port_index != slave->upstream_port)
        {
            ec_slave_t *next = slave->ports[port_index].next_slave;

            if (next)
            {
                dc_slave = ec_slave_find_next_dc_slave(next);

                if (dc_slave)
                {
                    break;
                }
            }
            port_index = ec_slave_get_next_port(slave, port_index);
        }
    }

    return dc_slave;
}

/*****************************************************************************/

/**
 * 计算端口传输延迟。
 *
 * @param slave  EtherCAT从站
 */
void ec_slave_calc_port_delays(ec_slave_t *slave)
{
    unsigned int port_index;
    ec_slave_t *next_slave, *next_dc;
    uint32_t rtt, next_rtt_sum;

    if (!slave->base_dc_supported)
        return;

    port_index = ec_slave_get_next_port(slave, slave->upstream_port);

    while (port_index != slave->upstream_port)
    {
        next_slave = slave->ports[port_index].next_slave;
        next_dc = ec_slave_find_next_dc_slave(next_slave);

        if (next_dc)
        {
            unsigned int prev_port = ec_slave_get_previous_normal_port(slave, port_index);

            if (!slave->ports[port_index].link.bypassed)
            {
                rtt = slave->ports[port_index].receive_time - slave->ports[prev_port].receive_time;
            }
            else
            {
                rtt = 0; // FIXME
            }
            next_rtt_sum = ec_slave_calc_rtt_sum(next_dc);

            slave->ports[port_index].delay_to_next_dc = (rtt - next_rtt_sum) / 2; // FIXME
            next_dc->ports[next_dc->upstream_port].delay_to_next_dc = (rtt - next_rtt_sum) / 2;

#if 0
            EC_SLAVE_DBG(slave, 1, "delay %u:%u rtt=%u"
                    " next_rtt_sum=%u delay=%u\n",
                    slave->ring_position, port_index, rtt, next_rtt_sum,
                    slave->ports[port_index].delay_to_next_dc);
#endif
        }

        port_index = ec_slave_get_next_port(slave, port_index);
    }
}

/*****************************************************************************/

/**
 * 递归计算传输延迟。
 *
 * @param slave  当前从站
 * @param delay  延迟总和
 */
void ec_slave_calc_transmission_delays_rec(ec_slave_t *slave, uint32_t *delay)
{
    unsigned int i;
    ec_slave_t *next_dc;

    EC_SLAVE_DBG(slave, 1, "%s(delay = %u ns)\n", __func__, *delay);

    slave->transmission_delay = *delay;

    i = ec_slave_get_next_port(slave, slave->upstream_port);

    while (i != slave->upstream_port)
    {
        ec_slave_port_t *port = &slave->ports[i];
        next_dc = ec_slave_find_next_dc_slave(port->next_slave);
        if (next_dc)
        {
            *delay = *delay + port->delay_to_next_dc;
#if 0
            EC_SLAVE_DBG(slave, 1, "%u:%u %u\n",
                    slave->ring_position, i, *delay);
#endif
            ec_slave_calc_transmission_delays_rec(next_dc, delay);
        }

        i = ec_slave_get_next_port(slave, i);
    }

    *delay = *delay + slave->ports[slave->upstream_port].delay_to_next_dc;
}

/*****************************************************************************/

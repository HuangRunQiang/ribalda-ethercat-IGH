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
   EtherCAT datagram pair methods.
*/

/*****************************************************************************/

#include <linux/slab.h>

#include "master.h"
#include "datagram_pair.h"

/*****************************************************************************/

/**
 * @brief Datagram pair constructor.
 *
 * @param pair 数据报对。
 * @param domain 父域。
 * @param logical_offset 逻辑偏移量。
 * @param data 数据指针。
 * @param data_size 数据大小。
 * @param used 输入/输出使用计数。
 * @return 成功返回0，否则返回负数错误代码。
 *
 * @details 此函数用于构造数据报对。
 * 它初始化数据报对的成员，并根据输入的参数设置数据报的相关属性。
 * 如果构造成功，则返回0；否则返回错误代码。
 */
int ec_datagram_pair_init(
    ec_datagram_pair_t *pair, /**< 数据报对。 */
    ec_domain_t *domain,      /**< 父域。 */
    uint32_t logical_offset,  /**< 逻辑偏移量。 */
    uint8_t *data,            /**< 数据指针。 */
    size_t data_size,         /**< 数据大小。 */
    const unsigned int used[] /**< 输入/输出使用计数。 */
)
{
    ec_device_index_t dev_idx;
    int ret;

    INIT_LIST_HEAD(&pair->list);
    pair->domain = domain;

    for (dev_idx = EC_DEVICE_MAIN;
         dev_idx < ec_master_num_devices(domain->master); dev_idx++)
    {
        ec_datagram_init(&pair->datagrams[dev_idx]);
        snprintf(pair->datagrams[dev_idx].name,
                 EC_DATAGRAM_NAME_SIZE, "domain%u-%u-%s", domain->index,
                 logical_offset, ec_device_names[dev_idx != 0]);
        pair->datagrams[dev_idx].device_index = dev_idx;
    }

    pair->expected_working_counter = 0U;

    for (dev_idx = EC_DEVICE_BACKUP;
         dev_idx < ec_master_num_devices(domain->master); dev_idx++)
    {
        /* 备份数据报有自己的内存 */
        ret = ec_datagram_prealloc(&pair->datagrams[dev_idx], data_size);
        if (ret)
        {
            goto out_datagrams;
        }
    }

#if EC_MAX_NUM_DEVICES > 1
    if (!(pair->send_buffer = kmalloc(data_size, GFP_KERNEL)))
    {
        EC_MASTER_ERR(domain->master,
                      "Failed to allocate domain send buffer!\n");
        ret = -ENOMEM;
        goto out_datagrams;
    }
#endif

    /* 下面的ec_datagram_lxx()调用不会失败，因为数据报要么有外部内存，要么是预分配的。 */

    if (used[EC_DIR_OUTPUT] && used[EC_DIR_INPUT])
    { // 输入和输出
        ec_datagram_lrw_ext(&pair->datagrams[EC_DEVICE_MAIN],
                            logical_offset, data_size, data);

        for (dev_idx = EC_DEVICE_BACKUP;
             dev_idx < ec_master_num_devices(domain->master); dev_idx++)
        {
            ec_datagram_lrw(&pair->datagrams[dev_idx],
                            logical_offset, data_size);
        }

        // 如果使用了LRW，输出FMMU的工作计数器增加2，而输入FMMU增加1。
        pair->expected_working_counter =
            used[EC_DIR_OUTPUT] * 2 + used[EC_DIR_INPUT];
    }
    else if (used[EC_DIR_OUTPUT])
    { // 仅输出
        ec_datagram_lwr_ext(&pair->datagrams[EC_DEVICE_MAIN],
                            logical_offset, data_size, data);
        for (dev_idx = EC_DEVICE_BACKUP;
             dev_idx < ec_master_num_devices(domain->master); dev_idx++)
        {
            ec_datagram_lwr(&pair->datagrams[dev_idx],
                            logical_offset, data_size);
        }

        pair->expected_working_counter = used[EC_DIR_OUTPUT];
    }
    else
    { // 仅输入（或无）
        ec_datagram_lrd_ext(&pair->datagrams[EC_DEVICE_MAIN],
                            logical_offset, data_size, data);
        for (dev_idx = EC_DEVICE_BACKUP;
             dev_idx < ec_master_num_devices(domain->master); dev_idx++)
        {
            ec_datagram_lrd(&pair->datagrams[dev_idx], logical_offset,
                            data_size);
        }

        pair->expected_working_counter = used[EC_DIR_INPUT];
    }

    for (dev_idx = EC_DEVICE_MAIN;
         dev_idx < ec_master_num_devices(domain->master); dev_idx++)
    {
        ec_datagram_zero(&pair->datagrams[dev_idx]);
    }

    return 0;

out_datagrams:
    for (dev_idx = EC_DEVICE_MAIN;
         dev_idx < ec_master_num_devices(domain->master); dev_idx++)
    {
        ec_datagram_clear(&pair->datagrams[dev_idx]);
    }

    return ret;
}

/*****************************************************************************/

/**
 * @brief Datagram pair destructor.
 *
 * @param pair 数据报对。
 *
 * @details 此函数用于销毁数据报对。
 * 它释放数据报对的资源，包括数据报的内存和发送缓冲区。
 * 注意：此函数不会释放父域的资源。
 */
void ec_datagram_pair_clear(
    ec_datagram_pair_t *pair /**< 数据报对。 */
)
{
    unsigned int dev_idx;

    for (dev_idx = EC_DEVICE_MAIN;
         dev_idx < ec_master_num_devices(pair->domain->master);
         dev_idx++)
    {
        ec_datagram_clear(&pair->datagrams[dev_idx]);
    }

#if EC_MAX_NUM_DEVICES > 1
    if (pair->send_buffer)
    {
        kfree(pair->send_buffer);
    }
#endif
}

/*****************************************************************************/

/**
 * @brief 处理接收到的数据。
 *
 * @param pair 数据报对。
 * @param wc_sum 工作计数器总和数组。
 * @return 所有设备的工作计数器总和。
 *
 * @details 此函数用于处理接收到的数据报。
 * 它遍历数据报对中的所有数据报，统计工作计数器总和，并更新wc_sum数组。
 * 返回所有设备的工作计数器总和。
 */
uint16_t ec_datagram_pair_process(
    ec_datagram_pair_t *pair, /**< 数据报对。 */
    uint16_t wc_sum[]         /**< 工作计数器总和数组。 */
)
{
    unsigned int dev_idx;
    uint16_t pair_wc = 0;

    for (dev_idx = 0; dev_idx < ec_master_num_devices(pair->domain->master);
         dev_idx++)
    {
        ec_datagram_t *datagram = &pair->datagrams[dev_idx];

#ifdef EC_RT_SYSLOG
        ec_datagram_output_stats(datagram);
#endif

        if (datagram->state == EC_DATAGRAM_RECEIVED)
        {
            pair_wc += datagram->working_counter;
            wc_sum[dev_idx] += datagram->working_counter;
        }
    }

    return pair_wc;
}

/*****************************************************************************/

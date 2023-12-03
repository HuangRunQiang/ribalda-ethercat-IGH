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
   EtherCAT domain methods.
*/

/*****************************************************************************/

#include <linux/module.h>

#include "globals.h"
#include "master.h"
#include "slave_config.h"

#include "domain.h"
#include "datagram_pair.h"

/** 冗余函数的额外调试输出。
 */
#define DEBUG_REDUNDANCY 0

#ifndef list_next_entry
#define list_next_entry(pos, member) \
    list_entry((pos)->member.next, typeof(*(pos)), member)
#endif

/*****************************************************************************/

void ec_domain_clear_data(ec_domain_t *);

/*****************************************************************************/

/**
 * @brief EtherCAT域初始化函数
 * @param domain EtherCAT域
 * @param master 父主站
 * @param index 索引
 * @return 无返回值
 * @details 初始化给定的EtherCAT域。
 */
void ec_domain_init(
    ec_domain_t *domain, /**< EtherCAT域。 */
    ec_master_t *master, /**< 父主站。 */
    unsigned int index   /**< 索引。 */
)
{
    unsigned int dev_idx;

    domain->master = master;
    domain->index = index;
    INIT_LIST_HEAD(&domain->fmmu_configs);
    domain->data_size = 0;
    domain->data = NULL;
    domain->data_origin = EC_ORIG_INTERNAL;
    domain->logical_base_address = 0x00000000;
    INIT_LIST_HEAD(&domain->datagram_pairs);
    for (dev_idx = EC_DEVICE_MAIN; dev_idx < ec_master_num_devices(master);
         dev_idx++)
    {
        domain->working_counter[dev_idx] = 0x0000;
    }
    domain->expected_working_counter = 0x0000;
    domain->working_counter_changes = 0;
    domain->redundancy_active = 0;
    domain->notify_jiffies = 0;

    /* Used by ec_domain_add_fmmu_config */
    memset(domain->offset_used, 0, sizeof(domain->offset_used));
    domain->sc_in_work = 0;
}

/*****************************************************************************/

/**
 * @brief 域析构函数
 * @param domain EtherCAT域
 * @return 无返回值
 * @details 释放域中的数据报对并清除域的数据
 */
void ec_domain_clear(ec_domain_t *domain /**< EtherCAT域 */)
{
    ec_datagram_pair_t *datagram_pair, *next_pair;

    // 遍历链表，释放并清除数据报对
    list_for_each_entry_safe(datagram_pair, next_pair,
                             &domain->datagram_pairs, list)
    {
        ec_datagram_pair_clear(datagram_pair);
        kfree(datagram_pair);
    }

    // 清除域的数据
    ec_domain_clear_data(domain);
}

/*****************************************************************************/

/**
 * @brief 域析构函数
 * @param domain EtherCAT域
 * @return 无返回值
 * @details 释放域中的数据报对并清除域的数据
 */
void ec_domain_clear(ec_domain_t *domain /**< EtherCAT域 */)
{
    ec_datagram_pair_t *datagram_pair, *next_pair;

    // 遍历链表，释放并清除数据报对
    list_for_each_entry_safe(datagram_pair, next_pair,
                             &domain->datagram_pairs, list)
    {
        ec_datagram_pair_clear(datagram_pair);
        kfree(datagram_pair);
    }

    // 清除域的数据
    ec_domain_clear_data(domain);
}

/*****************************************************************************/

/**
 * @brief 释放内部分配的内存
 * @param domain EtherCAT域
 * @return 无返回值
 * @details 释放域内部分配的内存空间
 */
void ec_domain_clear_data(
    ec_domain_t *domain /**< EtherCAT域。 */
)
{
    if (domain->data_origin == EC_ORIG_INTERNAL && domain->data)
    {
        kfree(domain->data);
    }

    domain->data = NULL;
    domain->data_origin = EC_ORIG_INTERNAL;
}

/*****************************************************************************/

/**
 * @brief 向域中添加FMMU配置
 * @param domain EtherCAT域
 * @param fmmu FMMU配置
 * @return 无返回值
 * @details 将FMMU配置添加到域中，并根据配置信息分配偏移量和空间大小
 */
void ec_domain_add_fmmu_config(
    ec_domain_t *domain,   /**< EtherCAT域。 */
    ec_fmmu_config_t *fmmu /**< FMMU配置。 */
)
{
    const ec_slave_config_t *sc;
    uint32_t logical_domain_offset;
    unsigned fmmu_data_size;

    fmmu->domain = domain;
    sc = fmmu->sc;

    fmmu_data_size = ec_pdo_list_total_size(
        &sc->sync_configs[fmmu->sync_index].pdos);

    if (sc->allow_overlapping_pdos && (sc == domain->sc_in_work))
    {
        // 如果允许重叠的PDO，并且已经为该从站分配了一个FMMU，通过方向分配后续的FMMU偏移量
        logical_domain_offset = domain->offset_used[fmmu->dir];
    }
    else
    {
        // 否则，将分配到该域上已分配的所有FMMU的最大范围。
        logical_domain_offset = max(domain->offset_used[EC_DIR_INPUT],
                                    domain->offset_used[EC_DIR_OUTPUT]);
        // 将空闲偏移量重新基于当前位置
        domain->offset_used[EC_DIR_INPUT] = logical_domain_offset;
        domain->offset_used[EC_DIR_OUTPUT] = logical_domain_offset;
    }
    domain->sc_in_work = sc;

    // 消耗该FMMU方向的偏移空间
    domain->offset_used[fmmu->dir] += fmmu_data_size;

    ec_fmmu_set_domain_offset_size(fmmu, logical_domain_offset, fmmu_data_size);

    list_add_tail(&fmmu->list, &domain->fmmu_configs);

    // 根据FMMU数据的最大范围确定域的大小
    domain->data_size = max(domain->offset_used[EC_DIR_INPUT],
                            domain->offset_used[EC_DIR_OUTPUT]);

    EC_MASTER_DBG(domain->master, 1, "Domain %u:"
                                     " Added %u bytes at %u.\n",
                  domain->index, fmmu->data_size, logical_domain_offset);
}

/*****************************************************************************/

/**
 * @brief 分配一个域数据报对并将其添加到列表中。
 *
 * 数据报的类型和期望的工作计数器由共享数据报的输入和输出FMMU的数量决定。
 *
 * @param domain EtherCAT域。
 * @param logical_offset 逻辑偏移量。
 * @param data_size 数据大小。
 * @param data 进程数据。
 * @param used 输入和输出的从站配置计数器。
 * @return 0表示成功，<0表示错误代码。
 * @details 该函数分配一个域数据报对并将其添加到列表中。数据报的类型和期望的工作计数器由共享数据报的输入和输出FMMU的数量决定。
 */
int ec_domain_add_datagram_pair(
    ec_domain_t *domain,      /**< EtherCAT域。 */
    uint32_t logical_offset,  /**< 逻辑偏移量。 */
    size_t data_size,         /**< 数据大小。 */
    uint8_t *data,            /**< 进程数据。 */
    const unsigned int used[] /**< 输入和输出的从站配置计数器。 */
)
{
    ec_datagram_pair_t *datagram_pair;
    int ret;

    if (!(datagram_pair = kmalloc(sizeof(ec_datagram_pair_t), GFP_KERNEL)))
    {
        EC_MASTER_ERR(domain->master,
                      "无法分配域数据报对！\n");
        return -ENOMEM;
    }

    ret = ec_datagram_pair_init(datagram_pair, domain, logical_offset, data,
                                data_size, used);
    if (ret)
    {
        kfree(datagram_pair);
        return ret;
    }

    domain->expected_working_counter +=
        datagram_pair->expected_working_counter;

    EC_MASTER_DBG(domain->master, 1,
                  "正在添加期望工作计数器为%u的数据报对。\n",
                  datagram_pair->expected_working_counter);

    list_add_tail(&datagram_pair->list, &domain->datagram_pairs);
    return 0;
}

/*****************************************************************************/

/**
 * @brief 域完成辅助函数。
 *
 * 检测从站配置是否已经计入数据报的期望工作计数器计算。
 *
 * 遍历当前数据报的所有FMMU配置列表，并在当前数据报之前结束。
 *
 * @param cur_fmmu 当前FMMU及其方向。
 * @param first_fmmu 数据报的第一个FMMU配置。
 * @return 非零表示从站配置已经计入计数。
 * @details 该函数用于检测从站配置是否已经计入数据报的期望工作计数器计算。它遍历当前数据报的所有FMMU配置列表，并在当前数据报之前结束。
 */
static int shall_count(
    const ec_fmmu_config_t *cur_fmmu,  /**< 当前FMMU及其方向。 */
    const ec_fmmu_config_t *first_fmmu /**< 数据报的第一个FMMU配置。 */
)
{
    for (; first_fmmu != cur_fmmu;
         first_fmmu = list_entry(first_fmmu->list.next,
                                 ec_fmmu_config_t, list))
    {

        if (first_fmmu->sc == cur_fmmu->sc && first_fmmu->dir == cur_fmmu->dir)
        {
            return 0; // 已经计入计数
        }
    }

    return 1;
}

/*****************************************************************************/

/** 域完成辅助函数。
 *
 * 已确定数据报的边界。扫描数据报的FMMU边界以获取工作计数器，并创建数据报对。
 *
 * @param domain 父域。
 * @param datagram_begin_offset 数据报的逻辑起始偏移量。
 * @param datagram_end_offset 逻辑结束偏移量（最后一个字节之后）。
 * @param datagram_first_fmmu 数据报中的起始FMMU。
 * @param datagram_end_fmmu 结束（最后一个）FMMU。
 * @return 非零表示域插入错误。
 * @details 该函数用于已确定数据报的边界。它会扫描数据报的FMMU边界以获取工作计数器，并创建数据报对。
 */
static int emplace_datagram(ec_domain_t *domain,
                            uint32_t datagram_begin_offset,
                            uint32_t datagram_end_offset,
                            const ec_fmmu_config_t *datagram_first_fmmu,
                            const ec_fmmu_config_t *datagram_end_fmmu)
{
    unsigned int datagram_used[EC_DIR_COUNT];
    const ec_fmmu_config_t *curr_fmmu;
    size_t data_size;

    data_size = datagram_end_offset - datagram_begin_offset;

    memset(datagram_used, 0, sizeof(datagram_used));
    for (curr_fmmu = datagram_first_fmmu;
         &curr_fmmu->list != &datagram_end_fmmu->list;
         curr_fmmu = list_next_entry(curr_fmmu, list))
    {
        if (shall_count(curr_fmmu, datagram_first_fmmu))
        {
            datagram_used[curr_fmmu->dir]++;
        }
    }

    return ec_domain_add_datagram_pair(domain,
                                       domain->logical_base_address + datagram_begin_offset,
                                       data_size,
                                       domain->data + datagram_begin_offset,
                                       datagram_used);
}

/*****************************************************************************/

/**
 * @brief 完成域。
 *
 * 这将分配必要的数据报并将正确的逻辑地址写入每个配置的FMMU。
 *
 * @param domain EtherCAT域。
 * @param base_address 逻辑基地址。
 * @return 成功返回0，否则返回负数错误代码。
 * @details 该函数完成域的配置。它会分配必要的数据报并将正确的逻辑地址写入每个配置的FMMU。
 */
int ec_domain_finish(
    ec_domain_t *domain,  /**< EtherCAT域。 */
    uint32_t base_address /**< 逻辑基地址。 */
)
{
    uint32_t datagram_offset = 0;
    unsigned int datagram_count = 0;
    ec_fmmu_config_t *fmmu;
    const ec_fmmu_config_t *datagram_first_fmmu = NULL;
    const ec_fmmu_config_t *valid_fmmu = NULL;
    unsigned candidate_start = 0;
    unsigned valid_start = 0;
    const ec_datagram_pair_t *datagram_pair;
    int ret;

    domain->logical_base_address = base_address;

    if (domain->data_size && domain->data_origin == EC_ORIG_INTERNAL)
    {
        if (!(domain->data =
                  (uint8_t *)kmalloc(domain->data_size, GFP_KERNEL)))
        {
            EC_MASTER_ERR(domain->master, "无法为域%u分配%zu字节的内部内存！\n",
                          domain->index, domain->data_size);
            return -ENOMEM;
        }
    }

    // 遍历所有域FMMU
    // - 修正逻辑基地址
    // - 设置承载进程数据的数据报
    // - 计算数据报的期望工作计数器
    if (!list_empty(&domain->fmmu_configs))
    {
        datagram_first_fmmu =
            list_entry(domain->fmmu_configs.next, ec_fmmu_config_t, list);
    }

    list_for_each_entry(fmmu, &domain->fmmu_configs, list)
    {
        if (fmmu->data_size > EC_MAX_DATA_SIZE)
        {
            EC_MASTER_ERR(domain->master,
                          "FMMU大小%u字节超过最大数据大小%u字节",
                          fmmu->data_size, EC_MAX_DATA_SIZE);
            return -EINVAL;
        }
        if (fmmu->logical_domain_offset >= candidate_start)
        {
            // 由于FMMU偏移量单调递增，并且候选起始偏移量从未被否定，现在它将永远不会被否定，
            // 因为没有未来的FMMU可以跨越它。
            // 在此点之前的所有FMMU都被批准为下一个数据报
            valid_fmmu = fmmu;
            valid_start = candidate_start;
            if (fmmu->logical_domain_offset + fmmu->data_size - datagram_offset > EC_MAX_DATA_SIZE)
            {
                // 新的候选起始偏移量超过了数据报大小，因此我们使用最后一个已知的有效候选者创建数据报
                ret = emplace_datagram(domain, datagram_offset, valid_start,
                                       datagram_first_fmmu, valid_fmmu);
                if (ret < 0)
                    return ret;

                datagram_offset = valid_start;
                datagram_count++;
                datagram_first_fmmu = fmmu;
            }
        }
        if (fmmu->logical_domain_offset + fmmu->data_size > candidate_start)
        {
            candidate_start = fmmu->logical_domain_offset + fmmu->data_size;
        }
    }

    /* 分配最后一个数据报对，如果还有剩余数据（即使进程数据适合单个数据报） */
    if (domain->data_size > datagram_offset)
    {
        ret = emplace_datagram(domain, datagram_offset, domain->data_size,
                               datagram_first_fmmu, fmmu);
        if (ret < 0)
            return ret;
        datagram_count++;
    }

    EC_MASTER_INFO(domain->master, "域%u：逻辑地址0x%08x，%zu字节，期望工作计数器%u。\n",
                   domain->index,
                   domain->logical_base_address, domain->data_size,
                   domain->expected_working_counter);

    list_for_each_entry(datagram_pair, &domain->datagram_pairs, list)
    {
        const ec_datagram_t *datagram =
            &datagram_pair->datagrams[EC_DEVICE_MAIN];
        EC_MASTER_INFO(domain->master, "  数据报%s：逻辑偏移量0x%08x，%zu字节，类型%s，位于%p。\n",
                       datagram->name,
                       EC_READ_U32(datagram->address), datagram->data_size,
                       ec_datagram_type_string(datagram), datagram);
    }

    return 0;
}

/*****************************************************************************/

/**
 * @brief 获取域的FMMU配置数量。
 *
 * @param domain EtherCAT域。
 * @return FMMU配置数量。
 * @details 该函数返回域中FMMU配置的数量。
 */
unsigned int ec_domain_fmmu_count(const ec_domain_t *domain)
{
    const ec_fmmu_config_t *fmmu;
    unsigned int num = 0;

    list_for_each_entry(fmmu, &domain->fmmu_configs, list)
    {
        num++;
    }

    return num;
}

/*****************************************************************************/

/**
 * @brief 通过列表中的位置获取特定的FMMU配置。
 *
 * @param domain EtherCAT域。
 * @param pos 列表位置。
 * @return 位置为pos的FMMU，如果不存在则返回NULL。
 * @details 该函数通过列表中的位置返回特定的FMMU配置。
 */
const ec_fmmu_config_t *ec_domain_find_fmmu(
    const ec_domain_t *domain, /**< EtherCAT域。 */
    unsigned int pos           /**< 列表位置。 */
)
{
    const ec_fmmu_config_t *fmmu;

    list_for_each_entry(fmmu, &domain->fmmu_configs, list)
    {
        if (pos--)
            continue;
        return fmmu;
    }

    return NULL;
}

/*****************************************************************************/

#if EC_MAX_NUM_DEVICES > 1

/**
 * @brief 数据变化处理。
 * @param send_buffer 发送缓冲区。
 * @param datagram 数据报。
 * @param offset 偏移量。
 * @param size 大小。
 * @return 如果接收到的数据与发送的数据不一致，返回1；否则返回0。
 * @details 该函数用于处理接收到的数据是否发生变化。
 */
int data_changed(
    uint8_t *send_buffer,          /**< 发送缓冲区。 */
    const ec_datagram_t *datagram, /**< 数据报。 */
    size_t offset,                 /**< 偏移量。 */
    size_t size                    /**< 大小。 */
)
{
    uint8_t *sent = send_buffer + offset;
    uint8_t *recv = datagram->data + offset;
    size_t i;

    for (i = 0; i < size; i++)
    {
        if (recv[i] != sent[i])
        {
            return 1;
        }
    }

    return 0;
}

#endif

/******************************************************************************
 *  Application interface
 *****************************************************************************/

/**
 * @brief 将接收到的数据与发送的数据进行比较，判断是否发生变化。
 * @param send_buffer 发送缓冲区。
 * @param datagram 数据报。
 * @param offset 偏移量。
 * @param size 大小。
 * @return 如果接收到的数据与发送的数据不一致，返回1；否则返回0。
 * @details 该函数用于处理接收到的数据是否发生变化。它通过逐个比较接收缓冲区和发送缓冲区中的数据，
 * 如果存在不一致的数据，则返回1，否则返回0。
 */
int data_changed(
    uint8_t *send_buffer,          /**< 发送缓冲区。 */
    const ec_datagram_t *datagram, /**< 数据报。 */
    size_t offset,                 /**< 偏移量。 */
    size_t size                    /**< 大小。 */
)
{
    uint8_t *sent = send_buffer + offset;
    uint8_t *recv = datagram->data + offset;
    size_t i;

    for (i = 0; i < size; i++)
    {
        if (recv[i] != sent[i])
        {
            return 1;
        }
    }

    return 0;
}

/*****************************************************************************/

/**
 * @brief 将PDO条目注册列表注册到指定的域中。
 * @param domain EtherCAT域。
 * @param regs PDO条目注册数组。
 * @return 成功返回0，否则返回负数错误代码。
 * @details 该函数将PDO条目注册列表注册到指定的域中。它遍历注册数组中的每个条目，
 * 为每个条目创建从站配置对象，并将其注册为域的PDO条目。同时，它将条目的偏移量保存到指定的偏移量变量中。
 */
int ecrt_domain_reg_pdo_entry_list(ec_domain_t *domain,
                                   const ec_pdo_entry_reg_t *regs)
{
    const ec_pdo_entry_reg_t *reg;
    ec_slave_config_t *sc;
    int ret;

    EC_MASTER_DBG(domain->master, 1, "ecrt_domain_reg_pdo_entry_list("
                                     "domain = 0x%p, regs = 0x%p)\n",
                  domain, regs);

    for (reg = regs; reg->index; reg++)
    {
        sc = ecrt_master_slave_config_err(domain->master, reg->alias,
                                          reg->position, reg->vendor_id, reg->product_code);
        if (IS_ERR(sc))
            return PTR_ERR(sc);

        ret = ecrt_slave_config_reg_pdo_entry(sc, reg->index,
                                              reg->subindex, domain, reg->bit_position);
        if (ret < 0)
            return ret;

        *reg->offset = ret;
    }

    return 0;
}

/*****************************************************************************/

/**
 * @brief 获取域的大小。
 * @param domain EtherCAT域。
 * @return 域的大小。
 * @details 该函数返回指定域的数据大小。
 */
size_t ecrt_domain_size(const ec_domain_t *domain)
{
    return domain->data_size;
}

/*****************************************************************************/

/**
 * @brief 设置域的外部内存。
 * @param domain EtherCAT域。
 * @param mem 外部内存指针。
 * @details 该函数设置指定域的外部内存。注意：在调用此函数之前，请确保已经锁定主站。
 */
void ecrt_domain_external_memory(ec_domain_t *domain, uint8_t *mem)
{
    EC_MASTER_DBG(domain->master, 1, "ecrt_domain_external_memory("
                                     "domain = 0x%p, mem = 0x%p)\n",
                  domain, mem);

    ec_lock_down(&domain->master->master_sem);

    ec_domain_clear_data(domain);

    domain->data = mem;
    domain->data_origin = EC_ORIG_EXTERNAL;

    ec_lock_up(&domain->master->master_sem);
}

/*****************************************************************************/

/**
 * @brief 获取域的数据指针。
 * @param domain EtherCAT域。
 * @return 域的数据指针。
 */
uint8_t *ecrt_domain_data(ec_domain_t *domain)
{
    return domain->data;
}

/*****************************************************************************/

/**
 * @brief 处理域中的数据。
 * @param domain EtherCAT域。
 * @details 该函数用于处理域中的数据。它遍历域中的每个数据报对，对每个数据报对进行处理。
 * 在处理过程中，它会计算工作计数器的总和，并根据特定条件执行一些操作，如数据冗余处理、
 * 工作计数器变化通知等。
 */
void ecrt_domain_process(ec_domain_t *domain)
{
    uint16_t wc_sum[EC_MAX_NUM_DEVICES] = {}, wc_total;
    ec_datagram_pair_t *pair;
#if EC_MAX_NUM_DEVICES > 1
    uint16_t datagram_pair_wc, redundant_wc;
    unsigned int datagram_offset;
    ec_fmmu_config_t *fmmu = list_first_entry(&domain->fmmu_configs,
                                              ec_fmmu_config_t, list);
    unsigned int redundancy;
#endif
    unsigned int dev_idx;
#ifdef EC_RT_SYSLOG
    unsigned int wc_change;
#endif

#if DEBUG_REDUNDANCY
    EC_MASTER_DBG(domain->master, 1, "域 %u 处理\n", domain->index);
#endif

    // 遍历每个数据报对
    list_for_each_entry(pair, &domain->datagram_pairs, list)
    {
#if EC_MAX_NUM_DEVICES > 1
        // 处理数据报对并计算工作计数器和
        datagram_pair_wc = ec_datagram_pair_process(pair, wc_sum);
#else
        ec_datagram_pair_process(pair, wc_sum);
#endif

#if EC_MAX_NUM_DEVICES > 1
        if (ec_master_num_devices(domain->master) > 1)
        {
            ec_datagram_t *main_datagram = &pair->datagrams[EC_DEVICE_MAIN];
            uint32_t logical_datagram_address =
                EC_READ_U32(main_datagram->address);
            size_t datagram_size = main_datagram->data_size;

#if DEBUG_REDUNDANCY
            EC_MASTER_DBG(domain->master, 1, "数据报 %s 逻辑地址=%u\n",
                          main_datagram->name, logical_datagram_address);
#endif

            /* 冗余性：遍历FMMU配置以检测数据变化。 */
            list_for_each_entry_from(fmmu, &domain->fmmu_configs, list)
            {
                ec_datagram_t *backup_datagram =
                    &pair->datagrams[EC_DEVICE_BACKUP];

                if (fmmu->dir != EC_DIR_INPUT)
                {
                    continue;
                }

                if (fmmu->logical_domain_offset >= datagram_size)
                {
                    // fmmu数据包含在下一个数据报对中
                    break;
                }

                datagram_offset = fmmu->logical_domain_offset;

#if DEBUG_REDUNDANCY
                EC_MASTER_DBG(domain->master, 1,
                              "输入FMMU 逻辑偏移=%u 大小=%u 偏移=%u\n",
                              fmmu->logical_domain_offset, fmmu->data_size,
                              datagram_offset);
                if (domain->master->debug_level > 0)
                {
                    ec_print_data(pair->send_buffer + datagram_offset,
                                  fmmu->data_size);
                    ec_print_data(main_datagram->data + datagram_offset,
                                  fmmu->data_size);
                    ec_print_data(backup_datagram->data + datagram_offset,
                                  fmmu->data_size);
                }
#endif

                if (data_changed(pair->send_buffer, main_datagram,
                                 datagram_offset, fmmu->data_size))
                {
                    /* 主链路数据变化：不需要复制 */
#if DEBUG_REDUNDANCY
                    EC_MASTER_DBG(domain->master, 1, "主链路数据变化\n");
#endif
                }
                else if (data_changed(pair->send_buffer, backup_datagram,
                                      datagram_offset, fmmu->data_size))
                {
                    /* 备用链路数据变化：复制到主内存 */
#if DEBUG_REDUNDANCY
                    EC_MASTER_DBG(domain->master, 1, "备用链路数据变化\n");
#endif
                    memcpy(main_datagram->data + datagram_offset,
                           backup_datagram->data + datagram_offset,
                           fmmu->data_size);
                }
                else if (datagram_pair_wc ==
                         pair->expected_working_counter)
                {
                    /* 没有变化，但工作计数器完成：使用主数据 */
#if DEBUG_REDUNDANCY
                    EC_MASTER_DBG(domain->master, 1,
                                  "没有变化但完成\n");
#endif
                }
                else
                {
                    /* 没有变化且工作计数器不完整：将工作计数器标记为零，以避免数据相关的工作计数器闪烁。 */
                    datagram_pair_wc = 0;
#if DEBUG_REDUNDANCY
                    EC_MASTER_DBG(domain->master, 1,
                                  "没有变化且不完整\n");
#endif
                }
            }
        }
#endif // EC_MAX_NUM_DEVICES > 1
    }

#if EC_MAX_NUM_DEVICES > 1
    redundant_wc = 0;
    for (dev_idx = EC_DEVICE_BACKUP;
         dev_idx < ec_master_num_devices(domain->master); dev_idx++)
    {
        redundant_wc += wc_sum[dev_idx];
    }

    redundancy = redundant_wc > 0;
    if (redundancy != domain->redundancy_active)
    {
#ifdef EC_RT_SYSLOG
        if (redundancy)
        {
            EC_MASTER_WARN(domain->master,
                           "域 %u：冗余链路正在使用！\n",
                           domain->index);
        }
        else
        {
            EC_MASTER_INFO(domain->master,
                           "域 %u：冗余链路再次未使用。\n",
                           domain->index);
        }
#endif
        domain->redundancy_active = redundancy;
    }
#else
    domain->redundancy_active = 0;
#endif

#ifdef EC_RT_SYSLOG
    wc_change = 0;
#endif
    wc_total = 0;
    for (dev_idx = EC_DEVICE_MAIN;
         dev_idx < ec_master_num_devices(domain->master); dev_idx++)
    {
        if (wc_sum[dev_idx] != domain->working_counter[dev_idx])
        {
#ifdef EC_RT_SYSLOG
            wc_change = 1;
#endif
            domain->working_counter[dev_idx] = wc_sum[dev_idx];
        }
        wc_total += wc_sum[dev_idx];
    }

#ifdef EC_RT_SYSLOG
    if (wc_change)
    {
        domain->working_counter_changes++;
    }

    if (domain->working_counter_changes &&
        jiffies - domain->notify_jiffies > HZ)
    {
        domain->notify_jiffies = jiffies;
        if (domain->working_counter_changes == 1)
        {
            EC_MASTER_INFO(domain->master, "域 %u：工作计数器"
                                           " 变为 %u/%u",
                           domain->index,
                           wc_total, domain->expected_working_counter);
        }
        else
        {
            EC_MASTER_INFO(domain->master, "域 %u：%u个工作计数器"
                                           " 变化 - 现在 %u/%u",
                           domain->index,
                           domain->working_counter_changes,
                           wc_total, domain->expected_working_counter);
        }
#if EC_MAX_NUM_DEVICES > 1
        if (ec_master_num_devices(domain->master) > 1)
        {
            printk(KERN_CONT " (");
            for (dev_idx = EC_DEVICE_MAIN;
                 dev_idx < ec_master_num_devices(domain->master);
                 dev_idx++)
            {
                printk(KERN_CONT "%u", domain->working_counter[dev_idx]);
                if (dev_idx + 1 < ec_master_num_devices(domain->master))
                {
                    printk(KERN_CONT "+");
                }
            }
            printk(KERN_CONT ")");
        }
#endif
        printk(KERN_CONT ".\n");

        domain->working_counter_changes = 0;
    }
#endif
}


/*****************************************************************************/

/**
 * @brief 将域中的数据报加入主站的发送队列。
 * @param domain EtherCAT域。
 * @details 该函数将域中的数据报加入主站的发送队列。它遍历域中的每个数据报对，
 * 将主数据报加入发送队列，并将主数据报的数据复制到备份数据报中，然后将备份数据报加入发送队列。
 */
void ecrt_domain_queue(ec_domain_t *domain)
{
    ec_datagram_pair_t *datagram_pair;
    ec_device_index_t dev_idx;

    list_for_each_entry(datagram_pair, &domain->datagram_pairs, list)
    {

#if EC_MAX_NUM_DEVICES > 1
        /* 将主数据复制到发送缓冲区 */
        memcpy(datagram_pair->send_buffer,
               datagram_pair->datagrams[EC_DEVICE_MAIN].data,
               datagram_pair->datagrams[EC_DEVICE_MAIN].data_size);
#endif
        ec_master_queue_datagram(domain->master,
                                 &datagram_pair->datagrams[EC_DEVICE_MAIN]);

        /* 将主数据复制到备份数据报 */
        for (dev_idx = EC_DEVICE_BACKUP;
             dev_idx < ec_master_num_devices(domain->master); dev_idx++)
        {
            memcpy(datagram_pair->datagrams[dev_idx].data,
                   datagram_pair->datagrams[EC_DEVICE_MAIN].data,
                   datagram_pair->datagrams[EC_DEVICE_MAIN].data_size);
            ec_master_queue_datagram(domain->master,
                                     &datagram_pair->datagrams[dev_idx]);
        }
    }
}
/*****************************************************************************/

/**
 * @brief 获取域的状态。
 * @param domain EtherCAT域。
 * @param state 域状态结构体指针。
 * @details 该函数获取指定域的状态信息。它计算工作计数器的总和，并根据工作计数器的值判断
 * 域的工作计数器状态和冗余状态，并将这些信息保存在状态结构体中。
 */
void ecrt_domain_state(const ec_domain_t *domain, ec_domain_state_t *state)
{
    unsigned int dev_idx;
    uint16_t wc = 0;

    for (dev_idx = EC_DEVICE_MAIN;
         dev_idx < ec_master_num_devices(domain->master); dev_idx++)
    {
        wc += domain->working_counter[dev_idx];
    }

    state->working_counter = wc;

    if (wc)
    {
        if (wc == domain->expected_working_counter)
        {
            state->wc_state = EC_WC_COMPLETE;
        }
        else
        {
            state->wc_state = EC_WC_INCOMPLETE;
        }
    }
    else
    {
        state->wc_state = EC_WC_ZERO;
    }

    state->redundancy_active = domain->redundancy_active;
}

/*****************************************************************************/

/** \cond */

EXPORT_SYMBOL(ecrt_domain_reg_pdo_entry_list);
EXPORT_SYMBOL(ecrt_domain_size);
EXPORT_SYMBOL(ecrt_domain_external_memory);
EXPORT_SYMBOL(ecrt_domain_data);
EXPORT_SYMBOL(ecrt_domain_process);
EXPORT_SYMBOL(ecrt_domain_queue);
EXPORT_SYMBOL(ecrt_domain_state);

/** \endcond */

/*****************************************************************************/

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
   EtherCAT domain structure.
*/

/*****************************************************************************/

#ifndef __EC_DOMAIN_H__
#define __EC_DOMAIN_H__

#include <linux/list.h>

#include "globals.h"
#include "datagram.h"
#include "master.h"
#include "fmmu_config.h"

/*****************************************************************************/

/** EtherCAT 域。
 *
 * 处理特定组从站的过程数据和因此需要的数据报。
 */
struct ec_domain
{
    struct list_head list; /**< 列表项。 */
    ec_master_t *master;   /**< 拥有该域的 EtherCAT 主站。 */
    unsigned int index;    /**< 索引（仅为一个数字）。 */

    struct list_head fmmu_configs;                /**< 包含的 FMMU 配置。 */
    size_t data_size;                             /**< 过程数据的大小。 */
    uint8_t *data;                                /**< 用于过程数据的内存。 */
    ec_origin_t data_origin;                      /**< \a data 内存的来源。 */
    uint32_t logical_base_address;                /**< 过程数据的逻辑偏移地址。 */
    struct list_head datagram_pairs;              /**< 过程数据交换的数据报对（主/备份）。 */
    uint16_t working_counter[EC_MAX_NUM_DEVICES]; /**< 上次工作计数器的值。 */
    uint16_t expected_working_counter;            /**< 预期的工作计数器。 */
    unsigned int working_counter_changes;         /**< 自上次通知以来的工作计数器变化。 */
    unsigned int redundancy_active;               /**< 如果使用冗余，则为非零值。 */
    unsigned long notify_jiffies;                 /**< 上次通知的时间。 */
    uint32_t offset_used[EC_DIR_COUNT];           /**< 下一个可用的域偏移量
                                                     （按方向划分的 PDO）。 */
    const ec_slave_config_t *sc_in_work;          /**< 正在该域中被激活注册的 slave_config
                                                     （即 ecrt_slave_config_reg_pdo_entry()）。 */
};


/*****************************************************************************/

void ec_domain_init(ec_domain_t *, ec_master_t *, unsigned int);
void ec_domain_clear(ec_domain_t *);

void ec_domain_add_fmmu_config(ec_domain_t *, ec_fmmu_config_t *);
int ec_domain_finish(ec_domain_t *, uint32_t);

unsigned int ec_domain_fmmu_count(const ec_domain_t *);
const ec_fmmu_config_t *ec_domain_find_fmmu(const ec_domain_t *, unsigned int);

/*****************************************************************************/

#endif

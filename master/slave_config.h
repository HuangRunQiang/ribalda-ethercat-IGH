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
   EtherCAT slave configuration structure.
*/

/*****************************************************************************/

#ifndef __EC_SLAVE_CONFIG_H__
#define __EC_SLAVE_CONFIG_H__

#include <linux/list.h>

#include "globals.h"
#include "slave.h"
#include "sync_config.h"
#include "fmmu_config.h"
#include "coe_emerg_ring.h"

/*****************************************************************************/

/** 用于将配置特定信息打印到系统日志的便捷宏。
 *
 * 这将使用带有前缀的格式为 \a fmt 的消息打印日志，
 * 前缀格式为 "EtherCAT <INDEX> <ALIAS>:<POSITION>: "，
 * 其中 INDEX 是主站索引，ALIAS 和 POSITION 标识配置。
 *
 * \param sc EtherCAT 从对象配置
 * \param fmt 格式字符串（类似于 printf()）
 * \param args 参数（可选）
 */
#define EC_CONFIG_INFO(sc, fmt, args...)                               \
        printk(KERN_INFO "EtherCAT %u %u:%u: " fmt, sc->master->index, \
               sc->alias, sc->position, ##args)

/** 用于将配置特定错误打印到系统日志的便捷宏。
 *
 * 这将使用带有前缀的格式为 \a fmt 的消息打印日志，
 * 前缀格式为 "EtherCAT ERROR %u %u:%u: "，
 * 其中 INDEX 是主站索引，ALIAS 和 POSITION 标识配置。
 *
 * \param sc EtherCAT 从对象配置
 * \param fmt 格式字符串（类似于 printf()）
 * \param args 参数（可选）
 */
#define EC_CONFIG_ERR(sc, fmt, args...)                                     \
        printk(KERN_ERR "EtherCAT ERROR %u %u:%u: " fmt, sc->master->index, \
               sc->alias, sc->position, ##args)

/** 用于将配置特定警告打印到系统日志的便捷宏。
 *
 * 这将使用带有前缀的格式为 \a fmt 的消息打印日志，
 * 前缀格式为 "EtherCAT WARNING %u %u:%u: "，
 * 其中 INDEX 是主站索引，ALIAS 和 POSITION 标识配置。
 *
 * \param sc EtherCAT 从对象配置
 * \param fmt 格式字符串（类似于 printf()）
 * \param args 参数（可选）
 */
#define EC_CONFIG_WARN(sc, fmt, args...)                       \
        printk(KERN_WARNING "EtherCAT WARNING %u %u:%u: " fmt, \
               sc->master->index, sc->alias, sc->position, ##args)

/** 用于将配置特定调试消息打印到系统日志的便捷宏。
 *
 * 这将使用带有前缀的格式为 \a fmt 的消息打印日志，
 * 前缀格式为 "EtherCAT DEBUG %u %u:%u: "，
 * 其中 INDEX 是主站索引，ALIAS 和 POSITION 标识配置。
 *
 * \param sc EtherCAT 从对象配置
 * \param level 调试级别。输出前主站的调试级别必须 >= \a level。
 * \param fmt 格式字符串（类似于 printf()）
 * \param args 参数（可选）
 */
#define EC_CONFIG_DBG(sc, level, fmt, args...)                                      \
        do                                                                          \
        {                                                                           \
                if (sc->master->debug_level >= level)                               \
                {                                                                   \
                        printk(KERN_DEBUG "EtherCAT DEBUG %u %u:%u: " fmt,          \
                               sc->master->index, sc->alias, sc->position, ##args); \
                }                                                                   \
        } while (0)
        
/*****************************************************************************/

/** EtherCAT 从对象配置。
 */
struct ec_slave_config
{
        struct list_head list; /**< 列表项。 */
        ec_master_t *master;   /**< 拥有该从对象配置的主站。 */

        uint16_t alias;        /**< 从对象别名。 */
        uint16_t position;     /**< 别名之后的索引。如果别名为零，则为环形位置。 */
        uint32_t vendor_id;    /**< 从对象厂商ID。 */
        uint32_t product_code; /**< 从对象产品代码。 */

        uint16_t watchdog_divider;   /**< 看门狗分频器，以40ns间隔的数量表示（参见规范寄存器0x0400）。 */
        uint16_t watchdog_intervals; /**< 过程数据看门狗间隔（参见规范寄存器0x0420）。 */

        uint8_t allow_overlapping_pdos; /**< 允许输入PDO使用与输出PDO相同的帧空间。 */
        ec_slave_t *slave;              /**< 从对象指针。如果从对象离线，则为 \a NULL。 */

        ec_sync_config_t sync_configs[EC_MAX_SYNC_MANAGERS]; /**< 同步管理器配置。 */
        ec_fmmu_config_t fmmu_configs[EC_MAX_FMMUS];         /**< FMMU配置。 */
        uint8_t used_fmmus;                                  /**< 使用的FMMU数量。 */
        uint16_t dc_assign_activate;                         /**< 厂商特定的AssignActivate字。 */
        ec_sync_signal_t dc_sync[EC_SYNC_SIGNAL_COUNT];      /**< DC同步信号。 */

        struct list_head sdo_configs;  /**< SDO配置列表。 */
        struct list_head sdo_requests; /**< SDO请求列表。 */
        struct list_head foe_requests; /**< FoE请求列表。 */
        struct list_head voe_handlers; /**< VoE处理程序列表。 */
        struct list_head reg_requests; /**< 寄存器请求列表。 */
        struct list_head soe_configs;  /**< SoE配置列表。 */

        ec_coe_emerg_ring_t emerg_ring; /**< CoE紧急环形缓冲区。 */
};

/*****************************************************************************/

void ec_slave_config_init(ec_slave_config_t *, ec_master_t *, uint16_t,
                          uint16_t, uint32_t, uint32_t);
void ec_slave_config_clear(ec_slave_config_t *);

int ec_slave_config_attach(ec_slave_config_t *);
void ec_slave_config_detach(ec_slave_config_t *);

void ec_slave_config_load_default_sync_config(ec_slave_config_t *);

unsigned int ec_slave_config_sdo_count(const ec_slave_config_t *);
const ec_sdo_request_t *ec_slave_config_get_sdo_by_pos_const(
    const ec_slave_config_t *, unsigned int);
unsigned int ec_slave_config_idn_count(const ec_slave_config_t *);
const ec_soe_request_t *ec_slave_config_get_idn_by_pos_const(
    const ec_slave_config_t *, unsigned int);
ec_sdo_request_t *ec_slave_config_find_sdo_request(ec_slave_config_t *,
                                                   unsigned int);
ec_foe_request_t *ec_slave_config_find_foe_request(ec_slave_config_t *,
                                                   unsigned int);
ec_reg_request_t *ec_slave_config_find_reg_request(ec_slave_config_t *,
                                                   unsigned int);
ec_voe_handler_t *ec_slave_config_find_voe_handler(ec_slave_config_t *,
                                                   unsigned int);
void ec_slave_config_expire_disconnected_requests(ec_slave_config_t *);

ec_sdo_request_t *ecrt_slave_config_create_sdo_request_err(
    ec_slave_config_t *, uint16_t, uint8_t, uint8_t, size_t);
ec_foe_request_t *ecrt_slave_config_create_foe_request_err(
    ec_slave_config_t *, size_t);
ec_voe_handler_t *ecrt_slave_config_create_voe_handler_err(
    ec_slave_config_t *, size_t);
ec_reg_request_t *ecrt_slave_config_create_reg_request_err(
    ec_slave_config_t *, size_t);

/*****************************************************************************/

#endif

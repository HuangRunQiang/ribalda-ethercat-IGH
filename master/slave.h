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
   EtherCAT slave structure.
*/

/*****************************************************************************/

#ifndef __EC_SLAVE_H__
#define __EC_SLAVE_H__

#include <linux/list.h>
#include <linux/kobject.h>
#include <linux/rtmutex.h>

#include "globals.h"
#include "datagram.h"
#include "pdo.h"
#include "sync.h"
#include "sdo.h"
#include "fsm_slave.h"

/*****************************************************************************/

/** 方便打印从站特定信息到系统日志的宏。
 *
 * 这将打印\a fmt中的消息，前缀为
 * "EtherCAT <INDEX>-<DEV>-<POSITION>: "，其中INDEX是主站索引，
 * DEV是设备，POSITION是从站的环形位置。
 *
 * \param slave EtherCAT从站
 * \param fmt 格式字符串（如printf()中的）
 * \param args 参数（可选）
 */
#define EC_SLAVE_INFO(slave, fmt, args...)                            \
    printk(KERN_INFO "EtherCAT %u-%s-%u: " fmt, slave->master->index, \
           ec_device_names[slave->device_index != 0], slave->ring_position, ##args)

/** 方便打印从站特定错误到系统日志的宏。
 *
 * 这将打印\a fmt中的消息，前缀为
 * "EtherCAT <INDEX>-<DEV>-<POSITION>: "，其中INDEX是主站索引，
 * DEV是设备，POSITION是从站的环形位置。
 *
 * \param slave EtherCAT从站
 * \param fmt 格式字符串（如printf()中的）
 * \param args 参数（可选）
 */
#define EC_SLAVE_ERR(slave, fmt, args...)                                  \
    printk(KERN_ERR "EtherCAT ERROR %u-%s-%u: " fmt, slave->master->index, \
           ec_device_names[slave->device_index != 0], slave->ring_position, ##args)

/** 方便打印从站特定警告到系统日志的宏。
 *
 * 这将打印\a fmt中的消息，前缀为
 * "EtherCAT <INDEX>-<DEV>-<POSITION>: "，其中INDEX是主站索引，
 * DEV是设备，POSITION是从站的环形位置。
 *
 * \param slave EtherCAT从站
 * \param fmt 格式字符串（如printf()中的）
 * \param args 参数（可选）
 */
#define EC_SLAVE_WARN(slave, fmt, args...)                                  \
    printk(KERN_WARNING "EtherCAT WARNING %u-%s-%u: " fmt,                  \
           slave->master->index, ec_device_names[slave->device_index != 0], \
           slave->ring_position, ##args)

/** 方便打印从站特定调试信息到系统日志的宏。
 *
 * 这将打印\a fmt中的消息，前缀为
 * "EtherCAT <INDEX>-<DEV>-<POSITION>: "，其中INDEX是主站索引，
 * DEV是设备，POSITION是从站的环形位置。
 *
 * \param slave EtherCAT从站
 * \param level 调试级别。主站的调试级别必须>= \a level才能输出。
 * \param fmt 格式字符串（如printf()中的）
 * \param args 参数（可选）
 */
#define EC_SLAVE_DBG(slave, level, fmt, args...)                                    \
    do                                                                              \
    {                                                                               \
        if (slave->master->debug_level >= level)                                    \
        {                                                                           \
            printk(KERN_DEBUG "EtherCAT DEBUG %u-%s-%u: " fmt,                      \
                   slave->master->index, ec_device_names[slave->device_index != 0], \
                   slave->ring_position, ##args);                                   \
        }                                                                           \
    } while (0)

/*****************************************************************************/

#ifdef EC_LOOP_CONTROL

/** 从站端口状态。
 */
typedef enum
{
    EC_SLAVE_PORT_DOWN,
    EC_SLAVE_PORT_WAIT,
    EC_SLAVE_PORT_UP
} ec_slave_port_state_t;

/** 从检测到链接到打开端口的等待时间[ms]。
 */
#define EC_PORT_WAIT_MS 2000

#endif

/*****************************************************************************/

/** 从站端口。
 */
typedef struct
{
    ec_slave_port_desc_t desc; /**< 端口描述符。 */
    ec_slave_port_link_t link; /**< 端口连接状态。 */
    ec_slave_t *next_slave;    /**< 连接的从站。 */
    uint32_t receive_time;     /**< 端口接收时间，用于延迟测量。 */
    uint32_t delay_to_next_dc; /**< 到此端口后面的下一个支持DC的从站的延迟[ns]。 */
#ifdef EC_LOOP_CONTROL
    ec_slave_port_state_t state;          /**< 用于环控制的端口状态。 */
    unsigned long link_detection_jiffies; /**< 链接检测的时间。 */
#endif
} ec_slave_port_t;

/*****************************************************************************/

/** 提取的从站信息接口数据。
 */
typedef struct
{
    // 非类别数据
    uint16_t alias;                  /**< 配置的站别名。 */
    uint32_t vendor_id;              /**< 厂商ID。 */
    uint32_t product_code;           /**< 厂商特定的产品代码。 */
    uint32_t revision_number;        /**< 修订号。 */
    uint32_t serial_number;          /**< 序列号。 */
    uint16_t boot_rx_mailbox_offset; /**< Bootstrap接收邮箱地址。 */
    uint16_t boot_rx_mailbox_size;   /**< Bootstrap接收邮箱大小。 */
    uint16_t boot_tx_mailbox_offset; /**< Bootstrap发送邮箱地址。 */
    uint16_t boot_tx_mailbox_size;   /**< Bootstrap发送邮箱大小。 */
    uint16_t std_rx_mailbox_offset;  /**< 标准接收邮箱地址。 */
    uint16_t std_rx_mailbox_size;    /**< 标准接收邮箱大小。 */
    uint16_t std_tx_mailbox_offset;  /**< 标准发送邮箱地址。 */
    uint16_t std_tx_mailbox_size;    /**< 标准发送邮箱大小。 */
    uint16_t mailbox_protocols;      /**< 支持的邮箱协议。 */

    // 字符串
    char **strings;            /**< SII类别中的字符串。 */
    unsigned int string_count; /**< SII字符串的数量。 */

    // 通用
    unsigned int has_general;             /**< 通用类别存在。 */
    char *group;                          /**< 组名。 */
    char *image;                          /**< 图像名。 */
    char *order;                          /**< 订单号。 */
    char *name;                           /**< 从站名。 */
    uint8_t physical_layer[EC_MAX_PORTS]; /**< 端口媒体。 */
    ec_sii_coe_details_t coe_details;     /**< CoE详细标志。 */
    ec_sii_general_flags_t general_flags; /**< 通用标志。 */
    int16_t current_on_ebus;              /**< 电流消耗量，单位mA。 */

    // SyncM
    ec_sync_t *syncs;        /**< SYNC MANAGER类别。 */
    unsigned int sync_count; /**< 同步管理器的数量。 */

    // [RT]XPDO
    struct list_head pdos; /**< SII [RT]XPDO类别。 */
} ec_sii_t;

/*****************************************************************************/

/** 完整的从站信息接口数据图像。
 */
typedef struct
{
    struct list_head list; /**< 列表项。 */

    uint16_t *words;
    size_t nwords; /**< SII内容的大小，单位为字。 */

    ec_sii_t sii; /**< 提取的SII数据。 */
} ec_sii_image_t;

/*****************************************************************************/

/** EtherCAT从站。
 */
struct ec_slave
{
    ec_master_t *master;            /**< 拥有该从站的主站。 */
    ec_device_index_t device_index; /**< 从站响应的设备的索引。 */

    // 地址
    uint16_t ring_position;   /**< 环形位置。 */
    uint16_t station_address; /**< 配置的站地址。 */
    uint16_t effective_alias; /**< 有效的别名地址。 */
    // 标识
#ifdef EC_SII_CACHE
    uint32_t effective_vendor_id;       /**< 有效的厂商ID。 */
    uint32_t effective_product_code;    /**< 有效的产品代码。 */
    uint32_t effective_revision_number; /**< 有效的修订号。 */
    uint32_t effective_serial_number;   /**< 有效的序列号。 */
#endif
    ec_slave_port_t ports[EC_MAX_PORTS]; /**< 端口。 */
    uint8_t upstream_port;               /**< 面向主站的端口的索引。 */

    // 配置
    ec_slave_config_t *config;             /**< 当前配置。 */
    ec_slave_state_t requested_state;      /**< 请求的应用状态。 */
    ec_slave_state_t current_state;        /**< 当前应用状态。 */
    uint16_t last_al_error;                /**< 最后的AL状态错误代码 */
    unsigned int error_flag;               /**< 在出错后停止处理。 */
    unsigned int force_config;             /**< 强制（重新）配置。 */
    unsigned int reboot;                   /**< 请求重启 */
    uint16_t configured_rx_mailbox_offset; /**< 配置的接收邮箱偏移。 */
    uint16_t configured_rx_mailbox_size;   /**< 配置的接收邮箱大小。 */
    uint16_t configured_tx_mailbox_offset; /**< 配置的发送邮箱偏移。 */
    uint16_t configured_tx_mailbox_size;   /**< 配置的发送邮箱大小。 */

    // 基本数据
    uint8_t base_type;                 /**< 从站类型。 */
    uint8_t base_revision;             /**< 修订。 */
    uint16_t base_build;               /**< 构建号。 */
    uint8_t base_fmmu_count;           /**< 支持的FMMU数量。 */
    uint8_t base_sync_count;           /**< 支持的同步管理器数量。 */
    uint8_t base_fmmu_bit_operation;   /**< 支持FMMU位操作。 */
    uint8_t base_dc_supported;         /**< 支持分布式时钟。 */
    ec_slave_dc_range_t base_dc_range; /**< DC范围。 */
    uint8_t has_dc_system_time;        /**< 从站支持DC系统时间寄存器。否则它只能用于
                                         延迟测量。 */
    uint32_t transmission_delay;       /**< DC系统时间传输延迟
                                         （相对于参考时钟的偏移）。 */

    // 从站信息接口
    uint16_t *vendor_words;    /**< SII图像的前16个字。 */
    ec_sii_image_t *sii_image; /**< 当前的完整SII图像。 */

    struct list_head sdo_dictionary; /**< SDO字典列表 */
    uint8_t scan_required;           /**< 需要扫描。 */
    uint8_t sdo_dictionary_fetched;  /**< 字典已被获取。 */
    unsigned long jiffies_preop;     /**< 从站进入PREOP的时间。 */

    struct list_head sdo_requests;  /**< SDO访问请求。 */
    struct list_head reg_requests;  /**< 寄存器访问请求。 */
    struct list_head foe_requests;  /**< FoE请求。 */
    struct list_head soe_requests;  /**< SoE请求。 */
    struct list_head eoe_requests;  /**< EoE设置IP参数请求。 */
    struct list_head mbg_requests;  /**< EoE设置IP参数请求。 */
    struct list_head dict_requests; /**< 字典读取请求。 */

    ec_fsm_slave_t fsm; /**< 从站状态机。 */

    uint8_t read_mbox_busy;   /**< 在邮箱读取请求期间设置的标志。 */
    struct rt_mutex mbox_sem; /**< 保护check_mbox变量的信号量。 */

#ifdef EC_EOE
    ec_mbox_data_t mbox_eoe_frag_data; /**< EoE接收的邮箱数据，帧片段类型。 */
    ec_mbox_data_t mbox_eoe_init_data; /**< EoE接收的邮箱数据，eoe初始化响应类型。 */
#endif
    ec_mbox_data_t mbox_coe_data; /**< CoE接收的邮箱数据。 */
    ec_mbox_data_t mbox_foe_data; /**< FoE接收的邮箱数据。 */
    ec_mbox_data_t mbox_soe_data; /**< SoE接收的邮箱数据。 */
    ec_mbox_data_t mbox_voe_data; /**< VoE接收的邮箱数据。 */
    ec_mbox_data_t mbox_mbg_data; /**< MBox Gateway接收的邮箱数据。 */

    uint8_t valid_mbox_data; /**< 接收的邮箱数据是有效的。 */
};

/*****************************************************************************/

// 从站的构造/析构
void ec_slave_init(ec_slave_t *, ec_master_t *, ec_device_index_t,
                   uint16_t, uint16_t); // 初始化EtherCAT从站对象

void ec_slave_sii_image_init(ec_sii_image_t *); // 初始化SII（从站信息接口）图像

void ec_slave_clear(ec_slave_t *); // 清除EtherCAT从站对象

void ec_slave_clear_sync_managers(ec_slave_t *); // 清除从站的同步管理器

void ec_slave_request_state(ec_slave_t *, ec_slave_state_t); // 请求从站状态改变
void ec_slave_set_dl_status(ec_slave_t *, uint16_t); // 设置从站的数据链路状态
void ec_slave_set_al_status(ec_slave_t *, ec_slave_state_t); // 设置从站的应用层状态
void ec_slave_request_reboot(ec_slave_t *); // 请求从站重启

// SII类别
int ec_slave_fetch_sii_strings(ec_slave_t *, const uint8_t *, size_t); // 获取SII字符串
int ec_slave_fetch_sii_general(ec_slave_t *, const uint8_t *, size_t); // 获取SII通用信息
int ec_slave_fetch_sii_syncs(ec_slave_t *, const uint8_t *, size_t); // 获取SII同步信息
int ec_slave_fetch_sii_pdos(ec_slave_t *, const uint8_t *, size_t,
                            ec_direction_t); // 获取SII PDOs

// 杂项。
ec_sync_t *ec_slave_get_sync(ec_slave_t *, uint8_t); // 获取从站的同步管理器

void ec_slave_sdo_dict_info(const ec_slave_t *,
                            unsigned int *, unsigned int *); // 获取从站SDO字典信息
ec_sdo_t *ec_slave_get_sdo(ec_slave_t *, uint16_t); // 获取从站的SDO
const ec_sdo_t *ec_slave_get_sdo_const(const ec_slave_t *, uint16_t); // 获取从站的SDO（const版本）
const ec_sdo_t *ec_slave_get_sdo_by_pos_const(const ec_slave_t *, uint16_t); // 根据位置获取从站的SDO（const版本）
uint16_t ec_slave_sdo_count(const ec_slave_t *); // 计算从站的SDO数量
const ec_pdo_t *ec_slave_find_pdo(const ec_slave_t *, uint16_t); // 查找从站的PDO
void ec_slave_attach_pdo_names(ec_slave_t *); // 为从站的PDO附加名称

void ec_slave_calc_upstream_port(ec_slave_t *); // 计算从站的上游端口
void ec_slave_calc_port_delays(ec_slave_t *); // 计算从站的端口延迟
void ec_slave_calc_transmission_delays_rec(ec_slave_t *, uint32_t *); // 递归计算从站的传输延迟

void ec_read_mbox_lock_clear(ec_slave_t *); // 清除从站的读邮箱锁
int ec_read_mbox_locked(ec_slave_t *); // 检查从站的读邮箱是否被锁定

#endif


/*****************************************************************************/

#endif

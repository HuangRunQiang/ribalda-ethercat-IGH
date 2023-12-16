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
 * \file
 * 全局定义和宏。
 */

/*****************************************************************************/

#ifndef __EC_MASTER_GLOBALS_H__
#define __EC_MASTER_GLOBALS_H__

#include "../globals.h"
#include "../include/ecrt.h"

/******************************************************************************
 * EtherCAT主站
 *****************************************************************************/

/** 数据报超时时间，单位为微秒。 */
#define EC_IO_TIMEOUT 500

/** SDO注入超时时间，单位为微秒。 */
#define EC_SDO_INJECTION_TIMEOUT 10000

/** 发送一个字节所需的时间，单位为纳秒。
 *
 * t_ns = 1 / (100 MBit/s / 8 bit/byte) = 80 ns/byte
 */
#define EC_BYTE_TRANSMISSION_TIME_NS 80

/** 在数据报超时时的状态机重试次数。 */
#define EC_FSM_RETRIES 3

/** 如果设置了此宏，则在从站扫描期间跳过获取SDO字典。 */
#define EC_SKIP_SDO_DICT 1

/** 用于ec_state_string()的缓冲区的最小大小。 */
#define EC_STATE_STRING_SIZE 32

/** SII的最大大小（以字为单位），以避免无限读取。 */
#define EC_MAX_SII_SIZE 4096

/** 维护的统计速率间隔数。 */
#define EC_RATE_COUNT 3

/******************************************************************************
 * EtherCAT协议
 *****************************************************************************/

/** EtherCAT帧头的大小。 */
#define EC_FRAME_HEADER_SIZE 2

/** EtherCAT数据报头的大小。 */
#define EC_DATAGRAM_HEADER_SIZE 10

/** EtherCAT数据报尾的大小。 */
#define EC_DATAGRAM_FOOTER_SIZE 2

/** EtherCAT地址字段的大小。 */
#define EC_ADDR_LEN 4

/** 单个数据报在帧中的最大数据大小。 */
#ifdef DEBUG_DATAGRAM_OVERFLOW
// 定义一个小数据报，用于在测试ec_domain_finish()时容易溢出
#define EC_MAX_DATA_SIZE (128)
#else
#define EC_MAX_DATA_SIZE (ETH_DATA_LEN - EC_FRAME_HEADER_SIZE - EC_DATAGRAM_HEADER_SIZE - EC_DATAGRAM_FOOTER_SIZE)
#endif // DEBUG_DATAGRAM_OVERFLOW

/** 邮箱头的大小。 */
#define EC_MBOX_HEADER_SIZE 6

/** CoE头的大小。 */
#define EC_COE_HEADER_SIZE 2

/** 邮箱网关，邮箱头从站地址偏移量。 */
#define EC_MBG_SLAVE_ADDR_OFFSET 1000

/** 第一个SII类别的字偏移量。 */
#define EC_FIRST_SII_CATEGORY_OFFSET 0x40

/** SII别名的字偏移量。 */
#define EC_ALIAS_SII_OFFSET 0x04

/** SII供应商ID的字偏移量。 */
#define EC_VENDOR_SII_OFFSET 0x08

/** SII产品编号的字偏移量。 */
#define EC_PRODUCT_SII_OFFSET 0x0A

/** SII修订号的字偏移量。 */
#define EC_REVISION_SII_OFFSET 0x0C

/** SII序列号的字偏移量。 */
#define EC_SERIAL_SII_OFFSET 0x0E

/** 同步管理器配置页面的大小。 */
#define EC_SYNC_PAGE_SIZE 8

/** 每个从站的最大FMMU数量。 */
#define EC_MAX_FMMUS 16

/** FMMU配置页面的大小。 */
#define EC_FMMU_PAGE_SIZE 16

/** DC同步信号的数量。 */
#define EC_SYNC_SIGNAL_COUNT 2

/** 数据报描述字符串的大小。
 *
 * 这也用作EoE设备名称的最大长度。
 **/
#define EC_DATAGRAM_NAME_SIZE 20

/** 最大主机名大小。
 *
 * 用于EoE设置IP参数请求中。
 */
#define EC_MAX_HOSTNAME_SIZE 32

/** 从站状态的掩码。
 *
 * 将此掩码应用于从站状态字节，以获取没有错误标志的从站状态。
 */
#define EC_SLAVE_STATE_MASK 0x0F

/**
 * @brief EtherCAT从站的状态。
 */
typedef enum
{
    EC_SLAVE_STATE_UNKNOWN = 0x00,
    /**< 未知状态 */
    EC_SLAVE_STATE_INIT = 0x01,
    /**< INIT状态（没有邮箱通信，没有IO） */
    EC_SLAVE_STATE_PREOP = 0x02,
    /**< PREOP状态（邮箱通信，没有IO） */
    EC_SLAVE_STATE_BOOT = 0x03,
    /**< 引导状态（邮箱通信，固件更新） */
    EC_SLAVE_STATE_SAFEOP = 0x04,
    /**< SAFEOP（邮箱通信和输入更新） */
    EC_SLAVE_STATE_OP = 0x08,
    /**< OP（邮箱通信和输入/输出更新） */
    EC_SLAVE_STATE_ACK_ERR = 0x10
    /**< 确认/错误位（没有实际状态） */
} ec_slave_state_t;

/** 支持的邮箱协议。
 *
 * 不要与邮箱头中定义的邮箱类型字段混淆，该字段在master/mailbox.h中定义。
 */
enum
{
    EC_MBOX_AOE = 0x01, /**< ADS over EtherCAT */
    EC_MBOX_EOE = 0x02, /**< Ethernet over EtherCAT */
    EC_MBOX_COE = 0x04, /**< CANopen over EtherCAT */
    EC_MBOX_FOE = 0x08, /**< File-Access over EtherCAT */
    EC_MBOX_SOE = 0x10, /**< Servo-Profile over EtherCAT */
    EC_MBOX_VOE = 0x20  /**< Vendor specific */
};

/** CANopen over EtherCAT的从站信息接口详细标志。
 */
typedef struct
{
    uint8_t enable_sdo : 1;                 /**< 启用SDO访问。 */
    uint8_t enable_sdo_info : 1;            /**< 可用SDO信息服务。 */
    uint8_t enable_pdo_assign : 1;          /**< 可配置PDO映射。 */
    uint8_t enable_pdo_configuration : 1;   /**< 可进行PDO配置。 */
    uint8_t enable_upload_at_startup : 1;   /**< ?. */
    uint8_t enable_sdo_complete_access : 1; /**< 可进行完全访问。 */
} ec_sii_coe_details_t;

/** 通用标志的从站信息接口。
 */
typedef struct
{
    uint8_t enable_safeop : 1;  /**< ?. */
    uint8_t enable_not_lrw : 1; /**< 从站不支持LRW。 */
} ec_sii_general_flags_t;

/** EtherCAT从站的分布式时钟范围。
 */
typedef enum
{
    EC_DC_32, /**< 32位。 */
    EC_DC_64  /*< 用于系统时间、系统时间偏移量和
                端口0接收时间的64位。 */
} ec_slave_dc_range_t;

/** EtherCAT从站同步信号配置。
 */
typedef struct
{
    uint32_t cycle_time; /**< 周期时间[ns]。 */
    int32_t shift_time;  /**< 偏移时间[ns]。 */
} ec_sync_signal_t;

/** SDO条目的访问状态。
 *
 * 访问权限是根据AL状态进行管理的。
 */
enum
{
    EC_SDO_ENTRY_ACCESS_PREOP,  /**< PREOP状态下的访问权限。 */
    EC_SDO_ENTRY_ACCESS_SAFEOP, /**< SAFEOP状态下的访问权限。 */
    EC_SDO_ENTRY_ACCESS_OP,     /**< OP状态下的访问权限。 */
    EC_SDO_ENTRY_ACCESS_COUNT   /**< 状态数量。 */
};

/** 主设备和备份设备。
 */
typedef enum
{
    EC_DEVICE_MAIN,  /**< 主设备。 */
    EC_DEVICE_BACKUP /**< 备份设备 */
} ec_device_index_t;

extern const char *ec_device_names[2]; // 仅主设备和备份设备！

/*****************************************************************************/

/** 打印EtherCAT特定信息到syslog的便捷宏。
 *
 * 这将使用带有前缀“EtherCAT：”的\a fmt打印消息。
 *
 * \param fmt 格式字符串（与printf()中的格式相同）
 * \param args 参数（可选）
 */
#define EC_INFO(fmt, args...) \
    printk(KERN_INFO "EtherCAT： " fmt, ##args)

/** 打印EtherCAT特定错误到syslog的便捷宏。
 *
 * 这将使用带有前缀“EtherCAT ERROR：”的\a fmt打印消息。
 *
 * \param fmt 格式字符串（与printf()中的格式相同）
 * \param args 参数（可选）
 */
#define EC_ERR(fmt, args...) \
    printk(KERN_ERR "EtherCAT ERROR： " fmt, ##args)

/** 打印EtherCAT特定警告到syslog的便捷宏。
 *
 * 这将使用带有前缀“EtherCAT WARNING：”的\a fmt打印消息。
 *
 * \param fmt 格式字符串（与printf()中的格式相同）
 * \param args 参数（可选）
 */
#define EC_WARN(fmt, args...) \
    printk(KERN_WARNING "EtherCAT WARNING： " fmt, ##args)

/** 打印EtherCAT调试消息到syslog的便捷宏。
 *
 * 这将使用带有前缀“EtherCAT DEBUG：”的\a fmt打印消息。
 *
 * \param fmt 格式字符串（与printf()中的格式相同）
 * \param args 参数（可选）
 */
#define EC_DBG(fmt, args...) \
    printk(KERN_DEBUG "EtherCAT DEBUG： " fmt, ##args)

/*****************************************************************************/

/**
 * @brief 绝对值。
 */
#define EC_ABS(X) ((X) >= 0 ? (X) : -(X))

/*****************************************************************************/

extern char *ec_master_version_str; // EtherCAT主版本字符串

/*****************************************************************************/

unsigned int ec_master_count(void); // 获取EtherCAT主站数量
void ec_print_data(const uint8_t *, size_t); // 打印数据
void ec_print_data_diff(const uint8_t *, const uint8_t *, size_t); // 打印数据差异
size_t ec_state_string(uint8_t, char *, uint8_t); // 获取状态字符串
size_t ec_mac_print(const uint8_t *, char *); // 打印MAC地址
int ec_mac_is_zero(const uint8_t *); // 检查MAC地址是否为零

ec_master_t *ecrt_request_master_err(unsigned int); // 请求EtherCAT主站

/*****************************************************************************/

/** Code/Message pair.
 *
 * 某些EtherCAT数据报支持读取状态码以显示特定消息。此类型允许将代码映射到消息字符串。
 */
typedef struct
{
    uint32_t code;       /**< 代码。 */
    const char *message; /**< 与 \a code 相关的消息。 */
} ec_code_msg_t;

/*****************************************************************************/

/** 通用请求状态。
 *
 * \attention 如果要更改此状态，请确保调整master/sdo_request.c中的 \a state_table。
 */
typedef enum
{
    EC_INT_REQUEST_INIT,    /**< 初始化。 */
    EC_INT_REQUEST_QUEUED,  /**< 已入队。 */
    EC_INT_REQUEST_BUSY,    /**< 忙碌。 */
    EC_INT_REQUEST_SUCCESS, /**< 成功。 */
    EC_INT_REQUEST_FAILURE  /**< 失败。 */
} ec_internal_request_state_t;

/*****************************************************************************/

extern const ec_request_state_t ec_request_state_translation_table[];

/*****************************************************************************/

/** 原始类型。
 */
typedef enum
{
    EC_ORIG_INTERNAL, /**< 内部。 */
    EC_ORIG_EXTERNAL  /**< 外部。 */
} ec_origin_t;
/*****************************************************************************/

typedef struct ec_slave ec_slave_t; /**< \see ec_slave. */

/*****************************************************************************/

#endif

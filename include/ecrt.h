/******************************************************************************
 *
 *  $Id$
 *
 *  Copyright (C) 2006-2012  Florian Pose, Ingenieurgemeinschaft IgH
 *
 *  This file is part of the IgH EtherCAT master userspace library.
 *
 *  The IgH EtherCAT master userspace library is free software; you can
 *  redistribute it and/or modify it under the terms of the GNU Lesser General
 *  Public License as published by the Free Software Foundation; version 2.1
 *  of the License.
 *
 *  The IgH EtherCAT master userspace library is distributed in the hope that
 *  it will be useful, but WITHOUT ANY WARRANTY; without even the implied
 *  warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with the IgH EtherCAT master userspace library. If not, see
 *  <http://www.gnu.org/licenses/>.
 *
 *  ---
 *
 *  The license mentioned above concerns the source code only. Using the
 *  EtherCAT technology and brand is only permitted in compliance with the
 *  industrial property and similar rights of Beckhoff Automation GmbH.
 *
 *****************************************************************************/

/** \file
 *
 * EtherCAT主站应用程序接口。
 *
 * \defgroup ApplicationInterface EtherCAT应用程序接口
 *
 * 用于实时应用程序的EtherCAT接口。该接口专为希望使用EtherCAT的实时模块而设计。它提供了请求主站、映射过程数据、通过CoE与从站通信以及配置和激活总线等功能的函数。
 *
 * 版本1.5.2中的变化：
 *
 * - 在ec_domain_state_t中添加了redundancy_active标志。
 * -
添加了ecrt_master_link_state()方法和ec_master_link_state_t，用于查询冗余链路的状态。
 * - 添加了EC_HAVE_REDUNDANCY定义，用于检查接口是否包含冗余功能。
 * - 添加了ecrt_sdo_request_index()，用于在处理程序创建后更改SDO索引和子索引。
 * -
添加了检索CoE紧急消息的接口，例如ecrt_slave_config_emerg_size()、ecrt_slave_config_emerg_pop()、ecrt_slave_config_emerg_clear()、ecrt_slave_config_emerg_overruns()以及EC_HAVE_EMERGENCY和EC_COE_EMERGENCY_MSG_SIZE的定义。
 * -
添加了直接EtherCAT寄存器访问的接口：添加了数据类型ec_reg_request_t和方法ecrt_slave_config_create_reg_request()、ecrt_reg_request_data()、ecrt_reg_request_state()、ecrt_reg_request_write()、ecrt_reg_request_read()，以及特性标志EC_HAVE_REG_ACCESS。
 * -
添加了选择参考时钟的方法ecrt_master_select_reference_clock()和特性标志EC_HAVE_SELECT_REF_CLOCK，以检查该方法是否可用。
 * -
添加了获取参考时钟时间的方法ecrt_master_reference_clock_time()和特性标志EC_HAVE_REF_CLOCK_TIME，以实现将主时钟与参考时钟同步的可能性。
 * -
将ecrt_slave_config_dc()中的移位时间数据类型更改为int32_t，以正确显示负移位时间。
 * -
添加了ecrt_slave_config_reg_pdo_entry_pos()和特性标志EC_HAVE_REG_BY_POS，用于通过映射中的位置注册具有非唯一索引的PDO条目。
 *
 * 版本1.5中的变化：
 *
 * -
添加了分布式时钟功能和相应的方法ecrt_slave_config_dc()，用于配置循环操作的从站，以及ecrt_master_application_time()、ecrt_master_sync_reference_clock()和ecrt_master_sync_slave_clocks()用于偏移和漂移补偿。可以使用EC_TIMEVAL2NANO()宏进行时代时间转换，同时可以使用ecrt_master_sync_monitor_queue()和ecrt_master_sync_monitor_process()方法来监视同步性。
 * -
改进了回调机制。ecrt_master_callbacks()现在接受两个回调函数用于发送和接收数据报文。使用ecrt_master_send_ext()来执行非应用数据报文的发送。
 * -
添加了看门狗配置（方法ecrt_slave_config_watchdog()、#ec_watchdog_mode_t、ec_sync_info_t中的\a
watchdog_mode参数以及ecrt_slave_config_sync_manager()）。
 * -
添加了ecrt_slave_config_complete_sdo()方法，用于在配置期间通过CompleteAccess下载SDO。
 * - 添加了ecrt_master_deactivate()方法，用于移除总线配置。
 * - 为用户空间添加了ecrt_open_master()和ecrt_master_reserve()的分离。
 * -
添加了总线信息接口（方法ecrt_master()、ecrt_master_get_slave()、ecrt_master_get_sync_manager()、ecrt_master_get_pdo()和ecrt_master_get_pdo_entry()），用于获取当前连接的从站和提供的PDO条目的信息。
 * -
添加了ecrt_master_sdo_download()、ecrt_master_sdo_download_complete()和ecrt_master_sdo_upload()方法，允许应用程序在激活主站之前传输SDO。
 * -
更改了ecrt_slave_config_reg_pdo_entry()和ecrt_slave_config_sdo*()的负返回值的含义。
 * -
实现了基于EtherCAT邮箱的供应商特定协议。请参阅ecrt_slave_config_create_voe_handler()。
 * -
将ec_sdo_request_state_t重命名为#ec_request_state_t，因为它也被VoE处理程序使用。
 * - 从ecrt_sdo_request_state()的参数中删除了'const'，因为用户

/*****************************************************************************/

#ifndef __ECRT_H__
#define __ECRT_H__

#ifdef __KERNEL__
#include <asm/byteorder.h>
#include <linux/time.h>
#include <linux/types.h>
#else
#include <stdint.h>
#include <stdlib.h>   // for size_t
#include <sys/time.h> // for struct timeval
#endif

/******************************************************************************
 * Global definitions
 *****************************************************************************/

/** EtherCAT实时接口的主要版本号。
 */
#define ECRT_VER_MAJOR 1

/** EtherCAT实时接口的次要版本号。
 */
#define ECRT_VER_MINOR 5

/** EtherCAT实时接口的修订版本号。
 */
#define ECRT_VER_PATCH 10

/** EtherCAT实时接口版本号生成器。
 */
#define ECRT_VERSION(a, b, c) (((a) << 16) + ((b) << 8) + (c))

/** EtherCAT实时接口版本号。
 */
#define ECRT_VERSION_MAGIC \
    ECRT_VERSION(ECRT_VER_MAJOR, ECRT_VER_MINOR, ECRT_VER_PATCH)

/******************************************************************************
 * 功能标志
 *****************************************************************************/

/** 定义，如果冗余功能可用。
 *
 * 例如，如果ec_domain_state_t中的\a
 * redundancy_active标志和ecrt_master_link_state()方法可用。
 */
#define EC_HAVE_REDUNDANCY

/** 定义，如果CoE紧急环功能可用。
 *
 * 例如，如果ecrt_slave_config_emerg_*()方法可用。
 */
#define EC_HAVE_EMERGENCY

/** 定义，如果寄存器访问接口可用。
 *
 * 例如，如果ecrt_slave_config_create_reg_request()、ecrt_reg_request_data()、ecrt_reg_request_state()、ecrt_reg_request_write()和ecrt_reg_request_read()方法可用。
 */
#define EC_HAVE_REG_ACCESS

/** 定义，如果方法ecrt_master_select_reference_clock()可用。
 */
#define EC_HAVE_SELECT_REF_CLOCK

/** 定义，如果方法ecrt_master_reference_clock_time()可用。
 */
#define EC_HAVE_REF_CLOCK_TIME

/** 定义，如果方法ecrt_slave_config_reg_pdo_entry_pos()可用。
 */
#define EC_HAVE_REG_BY_POS

/** 定义，如果方法ecrt_master_sync_reference_clock_to()可用。
 */
#define EC_HAVE_SYNC_TO

/*****************************************************************************/

/** 列表结束标记。
 *
 * 可以与ecrt_slave_config_pdos()一起使用。
 */
#define EC_END ~0U

/** 每个从站的最大同步管理器数量。
 */
#define EC_MAX_SYNC_MANAGERS 16

/** 最大字符串长度。
 *
 * 在ec_slave_info_t中使用。
 */
#define EC_MAX_STRING_LENGTH 64

/** 最大从站端口数量。 */
#define EC_MAX_PORTS 4

/** timeval到纳秒的转换。
 *
 * 此宏将Unix纪元时间转换为EtherCAT DC时间。
 *
 * \see void ecrt_master_application_time()
 *
 * \param TV 包含纪元时间的struct timeval。
 */
#define EC_TIMEVAL2NANO(TV) \
    (((TV).tv_sec - 946684800ULL) * 1000000000ULL + (TV).tv_usec * 1000ULL)

/** CoE紧急消息的字节大小。
 *
 * \see ecrt_slave_config_emerg_pop()。
 */
#define EC_COE_EMERGENCY_MSG_SIZE 8

/******************************************************************************
 * Data types
 *****************************************************************************/

struct ec_master;
typedef struct ec_master ec_master_t; /**< \see ec_master */

struct ec_slave_config;
typedef struct ec_slave_config ec_slave_config_t; /**< \see ec_slave_config */

struct ec_domain;
typedef struct ec_domain ec_domain_t; /**< \see ec_domain */

struct ec_sdo_request;
typedef struct ec_sdo_request ec_sdo_request_t; /**< \see ec_sdo_request. */

struct ec_foe_request;
typedef struct ec_foe_request ec_foe_request_t; /**< \see ec_foe_request. */

struct ec_voe_handler;
typedef struct ec_voe_handler ec_voe_handler_t; /**< \see ec_voe_handler. */

struct ec_reg_request;
typedef struct ec_reg_request ec_reg_request_t; /**< \see ec_reg_request. */

/*****************************************************************************/

/**
 * @brief       主站状态。
 *
 * @details     用于ecrt_master_state()的输出参数。
 *
 * @see         ecrt_master_state()。
 */
typedef struct
{
    unsigned int slaves_responding; /**< 所有以太网设备上响应的从站总数。 */
    unsigned int
        al_states : 4;          /**< 所有从站的应用层状态。
                                状态以低4位编码。
                                如果某一位被设置，表示总线上至少有一个从站处于相应的状态：
                                - 位 0: \a INIT
                                - 位 1: \a PREOP
                                - 位 2: \a SAFEOP
                                - 位 3: \a OP */
    unsigned int link_up : 1;   /**< \a true，如果至少有一个以太网链接正常。 */
    unsigned int scan_busy : 1; /**< \a true，如果从站重新扫描正在进行中。 */
} ec_master_state_t;

/*****************************************************************************/

/**
 * @brief       冗余链路状态。
 *
 * @details     用于ecrt_master_link_state()的输出参数。
 *
 * @see         ecrt_master_link_state()。
 */
typedef struct
{
    unsigned int slaves_responding; /**< 在给定链路上响应的从站总数。 */
    unsigned int
        al_states : 4;        /**< 给定链路上从站的应用层状态。
                                状态以低4位编码。
                                如果某一位被设置，表示总线上至少有一个从站处于相应的状态：
                                - 位 0: \a INIT
                                - 位 1: \a PREOP
                                - 位 2: \a SAFEOP
                                - 位 3: \a OP */
    unsigned int link_up : 1; /**< \a true，如果给定的以太网链路正常。 */
} ec_master_link_state_t;

/*****************************************************************************/

/**
 * @brief       从站配置状态。
 *
 * @details     用于ecrt_slave_config_state()的输出参数。
 *
 * @see         ecrt_slave_config_state()。
 */
typedef struct
{
    unsigned int online : 1;      /**< 从站在线。 */
    unsigned int operational : 1; /**< 使用指定配置将从站引入\a OP状态。 */
    unsigned int al_state : 4;    /**< 从站的应用层状态。
                                    - 1: \a INIT
                                    - 2: \a PREOP
                                    - 4: \a SAFEOP
                                    - 8: \a OP
   
                                    注意，每个状态都以不同的位编码！ */
    unsigned int error_flag : 1;  /**< 从站存在不可恢复的错误。 */
    unsigned int ready : 1;       /**< 从站准备好接收外部请求。 */
    uint16_t position;            /**< 从站在环中的偏移量。 */
} ec_slave_config_state_t;

/*****************************************************************************/

/** 主站信息。
 *
 * @details     用于ecrt_master()的输出参数。
 *
 * @see         ecrt_master()。
 */
typedef struct
{
    unsigned int slave_count; /**< 总线上的从站数量。 */
    unsigned int link_up : 1; /**< \a true，如果网络链接正常。 */
    uint8_t scan_busy;        /**< \a true，在主站扫描总线时。 */
    uint64_t app_time;        /**< 应用时间。 */
} ec_master_info_t;

/*****************************************************************************/
/** EtherCAT从站端口描述符。
 */
typedef enum
{
    EC_PORT_NOT_IMPLEMENTED, /**< 未实现的端口。 */
    EC_PORT_NOT_CONFIGURED,  /**< 未配置的端口。 */
    EC_PORT_EBUS,            /**< 端口是E-Bus。 */
    EC_PORT_MII              /**< 端口是MII。 */
} ec_slave_port_desc_t;

/*****************************************************************************/

/** EtherCAT从站端口信息。
 */
typedef struct
{
    uint8_t link_up;         /**< 检测到链接。 */
    uint8_t loop_closed;     /**< 闭环。 */
    uint8_t signal_detected; /**< 在RX端口上检测到的信号。 */
    uint8_t bypassed;        /**< 数据包绕过此端口（例如冗余）。 */
} ec_slave_port_link_t;

/*****************************************************************************/

/** 从站信息。
 *
 * 作为ecrt_master_get_slave()的输出参数使用。
 *
 * \see ecrt_master_get_slave()。
 */
typedef struct
{
    uint16_t position;        /**< 从站在环中的偏移量。 */
    uint32_t vendor_id;       /**< 存储在从站上的供应商ID。 */
    uint32_t product_code;    /**< 存储在从站上的产品代码。 */
    uint32_t revision_number; /**< 存储在从站上的版本号。 */
    uint32_t serial_number;   /**< 存储在从站上的序列号。 */
    uint16_t alias;           /**< 如果不等于0，则为从站的别名。 */
    int16_t current_on_ebus;  /**< 在毫安时使用的电流。 */
    struct
    {
        ec_slave_port_desc_t desc;   /**< 物理端口类型。 */
        ec_slave_port_link_t link;   /**< 端口链接状态。 */
        uint32_t receive_time;       /**< DC传输延迟测量的接收时间。 */
        uint16_t next_slave;         /**< 在该端口上的下一个DC从站的环位置。 */
        uint32_t delay_to_next_dc;   /**< 到下一个DC从站的延迟[ns]。 */
    } ports[EC_MAX_PORTS];           /**< 端口信息。 */
    uint8_t upstream_port;           /**< 上游（面向主站）端口的索引。 */
    uint8_t al_state;                /**< 从站的当前状态。 */
    uint8_t error_flag;              /**< 该从站的错误标志。 */
    uint8_t scan_required;           /**< 从站正在被扫描。 */
    uint8_t ready;                   /**< 从站已准备好接收外部请求。 */
    uint8_t sync_count;              /**< 同步管理器的数量。 */
    uint16_t sdo_count;              /**< SDO的数量。 */
    char name[EC_MAX_STRING_LENGTH]; /**< 从站的名称。 */
} ec_slave_info_t;

/*****************************************************************************/

/**
 * \brief 域工作计数器解释
 *
 * 用于ec_domain_state_t中。
 */
typedef enum
{
    EC_WC_ZERO = 0,   /**< 未交换任何已注册的过程数据。 */
    EC_WC_INCOMPLETE, /**< 部分已注册的过程数据已交换。 */
    EC_WC_COMPLETE    /**< 所有已注册的过程数据已交换。 */
} ec_wc_state_t;

/*****************************************************************************/

/** 域状态。
 *
 * 该结构体用于描述域（Domain）的状态，用作ecrt_domain_state()函数的输出参数。
 */
typedef struct
{
    unsigned int working_counter;   /**< 上一次工作计数器的值。 */
    ec_wc_state_t wc_state;         /**< 工作计数器的解释。 */
    unsigned int redundancy_active; /**< 冗余链路是否正在使用中的标志。 */
} ec_domain_state_t;

/*****************************************************************************/

/** PDO分配函数的方向类型。
 */
typedef enum
{
    EC_DIR_INVALID, /**< 无效的方向。不要使用该值。 */
    EC_DIR_OUTPUT,  /**< 主站写入的值。 */
    EC_DIR_INPUT,   /**< 主站读取的值。 */
    EC_DIR_BOTH,    /**< 主站读写的值。 */
    EC_DIR_COUNT    /**< 方向数量。仅供内部使用。 */
} ec_direction_t;

/*****************************************************************************/

/** 同步管理器配置的看门狗模式。
 *
 * 用于指定是否启用同步管理器的看门狗。
 */
typedef enum
{
    EC_WD_DEFAULT, /**< 使用同步管理器的默认设置。 */
    EC_WD_ENABLE,  /**< 启用看门狗。 */
    EC_WD_DISABLE, /**< 禁用看门狗。 */
} ec_watchdog_mode_t;
/*****************************************************************************/

/** PDO条目配置信息。
 *
 * 这是ec_pdo_info_t中\a entries字段的数据类型。
 *
 * \see ecrt_slave_config_pdos()。
 */
typedef struct
{
    uint16_t index;     /**< PDO条目索引。 */
    uint8_t subindex;   /**< PDO条目子索引。 */
    uint8_t bit_length; /**< PDO条目的位大小。 */
} ec_pdo_entry_info_t;

/*****************************************************************************/

/** PDO配置信息。
 *
 * 这是ec_sync_info_t中\a pdos字段的数据类型。
 *
 * \see ecrt_slave_config_pdos()。
 */
typedef struct
{
    uint16_t index;         /**< PDO索引。 */
    unsigned int n_entries; /**< \a entries中要映射的PDO条目数量。
                              如果为零，表示将使用默认映射（只能在总线配置时存在从设备时才能使用）。
                            */
    ec_pdo_entry_info_t
        *entries; /**< 要映射的PDO条目数组。
                    可以是\a NULL，或者必须包含至少\a n_entries个值。 */
} ec_pdo_info_t;

/*****************************************************************************/

/** 同步管理器配置信息。
 *
 * 这可以用于配置多个同步管理器，包括PDO分配和PDO映射。它作为ecrt_slave_config_pdos()中的输入参数类型使用。
 */
typedef struct
{
    uint8_t
        index;                        /**<
                                         同步管理器索引。必须小于#EC_MAX_SYNC_MANAGERS才能成为有效的同步管理器，但也可以是\a
                                         0xff，表示列表的结尾。 */
    ec_direction_t dir;               /**< 同步管理器方向。 */
    unsigned int n_pdos;              /**< \a pdos中的PDO数量。 */
    ec_pdo_info_t *pdos;              /**< 要分配的PDO数组。它必须包含至少\a n_pdos个PDO。 */
    ec_watchdog_mode_t watchdog_mode; /**< 看门狗模式。 */
} ec_sync_info_t;

/*****************************************************************************/

/** 用于PDO条目批量注册的列表记录类型。
 *
 * 此类型用于ecrt_domain_reg_pdo_entry_list()的数组参数。
 */
typedef struct
{
    uint16_t alias;             /**< 从站别名地址。 */
    uint16_t position;          /**< 从站位置。 */
    uint32_t vendor_id;         /**< 从站厂商ID。 */
    uint32_t product_code;      /**< 从站产品代码。 */
    uint16_t index;             /**< PDO条目索引。 */
    uint8_t subindex;           /**< PDO条目子索引。 */
    unsigned int *offset;       /**<
                                   指向变量的指针，用于存储PDO条目在过程数据中的（字节）偏移量。
                                 */
    unsigned int *bit_position; /**< 指向变量的指针，用于存储位（0-7）在\a
                                   offset内的位置。如果为NULL，则在PDO条目不字节对齐时引发错误。
                                 */
} ec_pdo_entry_reg_t;

/*****************************************************************************/

/** 请求状态。
 *
 * 用作ecrt_sdo_request_state()和ecrt_voe_handler_state()的返回类型。
 */
typedef enum
{
    EC_REQUEST_UNUSED,  /**< 未请求。 */
    EC_REQUEST_BUSY,    /**< 请求正在处理中。 */
    EC_REQUEST_SUCCESS, /**< 请求已成功处理。 */
    EC_REQUEST_ERROR,   /**< 请求处理失败。 */
} ec_request_state_t;

/*****************************************************************************/

/** FoE错误枚举类型。
 */
typedef enum
{
    FOE_BUSY = 0,                /**< 忙碌。 */
    FOE_READY = 1,               /**< 就绪。 */
    FOE_IDLE = 2,                /**< 空闲。 */
    FOE_WC_ERROR = 3,            /**< 工作计数器错误。 */
    FOE_RECEIVE_ERROR = 4,       /**< 接收错误。 */
    FOE_PROT_ERROR = 5,          /**< 协议错误。 */
    FOE_NODATA_ERROR = 6,        /**< 无数据错误。 */
    FOE_PACKETNO_ERROR = 7,      /**< 数据包编号错误。 */
    FOE_OPCODE_ERROR = 8,        /**< 操作码错误。 */
    FOE_TIMEOUT_ERROR = 9,       /**< 超时错误。 */
    FOE_SEND_RX_DATA_ERROR = 10, /**< 发送接收到的数据错误。 */
    FOE_RX_DATA_ACK_ERROR = 11,  /**< 确认接收到的数据错误。 */
    FOE_ACK_ERROR = 12,          /**< 确认错误。 */
    FOE_MBOX_FETCH_ERROR = 13,   /**< 从邮箱获取数据错误。 */
    FOE_READ_NODATA_ERROR = 14,  /**< 读取时无数据错误。 */
    FOE_MBOX_PROT_ERROR = 15,    /**< 邮箱协议错误。 */
    FOE_READ_OVER_ERROR = 16,    /**< 读取缓冲区溢出错误。 */
} ec_foe_error_t;

/*****************************************************************************/

/** Application-layer state.
 */
typedef enum
{
    EC_AL_STATE_INIT = 1,   /**< Init. */
    EC_AL_STATE_PREOP = 2,  /**< Pre-operational. */
    EC_AL_STATE_SAFEOP = 4, /**< Safe-operational. */
    EC_AL_STATE_OP = 8,     /**< Operational. */
} ec_al_state_t;

/******************************************************************************
 * Global functions
 *****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

    /** 返回实时接口的版本魔术数。
     *
     * \return EtherCAT主站编译时的ECRT_VERSION_MAGIC()的值。
     */
    unsigned int ecrt_version_magic(void);

    /**
     * @brief		请求一个用于实时操作的EtherCAT主站。
     * @details
     * 在应用程序可以访问EtherCAT主站之前，必须先为其保留独占使用权。
     *
     * 在用户空间，这是一个方便的函数，用于调用ecrt_open_master()和ecrt_master_reserve()。
     *
     * 这个函数必须是应用程序调用的第一个函数，用于使用EtherCAT。该函数以主站的索引作为参数。
     * 第一个主站的索引为0，第n个主站的索引为n -
     * 1。在加载主站模块时需要指定主站的数量。
     *
     * @param[in]	master_index 请求的主站索引。
     * @return		指向已保留主站的指针，否则返回 \a NULL。
     */
    ec_master_t *
    ecrt_request_master(unsigned int master_index /**< 请求的主站索引。 */
    );

#ifndef __KERNEL__

    /**
     * @brief		打开一个用于用户空间访问的EtherCAT主站。
     *
     * 这个函数必须是应用程序调用的第一个函数，用于使用EtherCAT。该函数以主站的索引作为参数。
     * 第一个主站的索引为0，第n个主站的索引为n -
     * 1。在加载主站模块时需要指定主站的数量。
     *
     * 为了方便起见，可以使用函数ecrt_request_master()。
     *
     * @param[in]	master_index 请求的主站索引。
     * @return		指向已打开主站的指针，否则返回 \a NULL。
     */
    ec_master_t *
    ecrt_open_master(unsigned int master_index /**< 请求的主站索引。 */
    );

#endif // #ifndef __KERNEL__

    /**
     * @brief		释放已请求的EtherCAT主站。
     *
     * 在使用完毕后，必须释放主站以便其他应用程序可以使用。
     *
     * 该方法释放所有创建的数据结构。不应在实时环境中调用。
     *
     * 如果主站已激活，则内部会调用ecrt_master_deactivate()。
     *
     * @param[in]	master EtherCAT主站。
     */
    void ecrt_release_master(ec_master_t *master /**< EtherCAT主站 */
    );

    /******************************************************************************
     * Master methods
     *****************************************************************************/

#ifndef __KERNEL__

    /**
     * @brief		为实时操作保留一个EtherCAT主站。
     *
     * 在应用程序可以在主站上使用PDO/域注册函数或SDO请求函数之前，必须为其保留一个主站以供独占使用。
     *
     * @param[in]	master EtherCAT主站。
     * @return		成功返回0，否则返回 < 0。
     */
    int ecrt_master_reserve(ec_master_t *master /**< EtherCAT主站 */
    );

#endif // #ifndef __KERNEL__

#ifdef __KERNEL__

    /**
     * @brief		设置锁定回调函数。
     *
     * 对于并发主站访问，即如果除应用程序之外的其他实例要在总线上发送和接收数据报，应用程序必须提供回调机制。
     * 该方法接受两个函数指针作为参数。只有在设置了回调函数后，才能进行异步主站访问（如EoE处理）。
     *
     * 发送回调函数（\a send_cb）的任务是判断总线当前是否可访问，并决定是否调用ecrt_master_send_ext()方法。
     *
     * 接收回调函数（\a receive_cb）的任务是判断是否允许调用ecrt_master_receive()并相应地执行它。
     *
     * \attention	该方法必须在调用ecrt_master_activate()之前调用。
     *
     * @param[in]	master		EtherCAT主站。
     * @param[in]	send_cb		数据报发送回调函数。
     * @param[in]	receive_cb	接收回调函数。
     * @param[in]	cb_data		传递给回调函数的任意指针。
     */
    void ecrt_master_callbacks(
        ec_master_t *master,        /**< EtherCAT主站 */
        void (*send_cb)(void *),    /**< 数据报发送回调函数 */
        void (*receive_cb)(void *), /**< 接收回调函数 */
        void *cb_data               /**< 传递给回调函数的任意指针 */
    );

#endif /* __KERNEL__ */

    /**
     * @brief		创建一个新的过程数据域。
     * @details		用于过程数据交换，至少需要一个过程数据域。
     *              该方法创建一个新的过程数据域并返回指向新域对象的指针。
     *              可以使用该对象来注册PDO并在周期性操作中进行交换。
     *              该方法分配内存，应在非实时上下文中在调用ecrt_master_activate()之前调用。
     * @param[in]	master	EtherCAT主站。
     * @return		成功时返回指向新域的指针，否则返回NULL。
     */
    ec_domain_t *ecrt_master_create_domain(
        ec_master_t *master /**< EtherCAT主站 */
    );

    /**
     * @brief		设置域的过程数据内存。
     * @details		在所有PDO条目注册完成并激活主站之前调用此方法。
     *              如果需要在激活主站之前访问域内存，请调用此方法。
     * @param[in]	master	EtherCAT主站。
     * @return		成功返回0，否则返回非零值。
     */
    int ecrt_master_setup_domain_memory(
        ec_master_t *master /**< EtherCAT主站 */
    );
    /**
     * @brief 获取从站配置。
     *
     * 为给定的别名和位置元组创建从站配置对象并返回。
     * 如果已存在具有相同别名和位置的配置，则将重新使用它。
     * 在后一种情况下，将与存储的厂商ID和产品代码进行比较。
     * 如果不匹配，则会引发错误消息并返回NULL。
     *
     * 从站使用别名和位置参数进行寻址。
     * - 如果别名为零，则位置被解释为所需从站的环位置。
     * - 如果别名非零，则它与具有给定别名的从站匹配。
     * 在这种情况下，位置被解释为从别名从站开始的环偏移量，
     * 因此，位置为零表示别名从站本身，正值表示别名从站后面的第n个从站。
     *
     * 如果在总线配置期间找到具有给定地址的从站，
     * 则将其厂商ID和产品代码与给定值进行匹配。
     * 如果不匹配，则不配置该从站并引发错误消息。
     *
     * 如果在总线配置期间不同的从站配置指向同一个从站，
     * 则会发出警告，并且只应用第一个配置。
     *
     * 该方法分配内存，并应在非实时上下文中在调用ecrt_master_activate()之前调用。
     *
     * @param master EtherCAT主站
     * @param alias 从站别名
     * @param position 从站位置
     * @param vendor_id 期望的厂商ID
     * @param product_code 期望的产品代码
     * @retval >0 指向从站配置结构的指针
     * @retval NULL 错误情况下返回NULL
     */
    ec_slave_config_t *ecrt_master_slave_config(
        ec_master_t *master,
        uint16_t alias,
        uint16_t position,
        uint32_t vendor_id,
        uint32_t product_code);

    /**
     * @brief 选择分布式时钟的参考时钟。
     *
     * 如果对于某个主站未调用此方法，或者从站配置指针为NULL，
     * 则具有DC功能的第一个从站将提供参考时钟。
     *
     * @param master EtherCAT主站
     * @param sc 用作参考从站的从站配置（或NULL）
     * @return 成功时返回0，否则返回负错误代码
     */
    int ecrt_master_select_reference_clock(
        ec_master_t *master,  /**< EtherCAT主站 */
        ec_slave_config_t *sc /**< 用作参考从站的从站配置（或NULL） */
    );
    /**
     * @brief 获取主站信息。
     *
     * 此函数不会在堆上分配内存。
     *
     * @attention 对该结构的指针必须指向一个有效的变量。
     *
     * @param master EtherCAT主站
     * @param master_info 将输出信息的结构体
     * @return 成功时返回0，否则返回<0
     */
    int ecrt_master(
        ec_master_t *master,          /**< EtherCAT主站 */
        ec_master_info_t *master_info /**< 将输出信息的结构体 */
    );
    /**
     * @brief 获取从站信息。
     *
     * 尝试找到给定环位置的从站。获取到的信息存储在一个结构体中。
     * 此函数不会在堆上分配内存。
     *
     * @attention 对该结构的指针必须指向一个有效的变量。
     *
     * @param master EtherCAT主站
     * @param slave_position 从站位置
     * @param slave_info 将输出信息的结构体
     * @return 成功时返回0，否则返回<0
     */
    int ecrt_master_get_slave(
        ec_master_t *master,        /**< EtherCAT主站 */
        uint16_t slave_position,    /**< 从站位置 */
        ec_slave_info_t *slave_info /**< 将输出信息的结构体 */
    );

#ifndef __KERNEL__

    /**
     * @brief 返回从站同步管理器的建议配置。
     *
     * 使用给定的ec_sync_info_t结构填充同步管理器的属性。
     * 返回值的 \a pdos 字段为空。使用ecrt_master_get_pdo()来获取PDO信息。
     *
     * @param master EtherCAT主站
     * @param slave_position 从站位置
     * @param sync_index 同步管理器索引。必须小于 #EC_MAX_SYNC_MANAGERS。
     * @param sync 指向输出结构的指针
     * @return 成功时返回0，否则返回非零值
     */
    int ecrt_master_get_sync_manager(
        ec_master_t *master,     /**< EtherCAT主站 */
        uint16_t slave_position, /**< 从站位置 */
        uint8_t sync_index,      /**< 同步管理器索引。必须小于 #EC_MAX_SYNC_MANAGERS。 */
        ec_sync_info_t *sync     /**< 指向输出结构的指针 */
    );

    /**
     * @brief 返回当前分配的PDO的信息。
     *
     * 使用给定的ec_pdo_info_t结构填充当前分配的PDO的属性，该PDO属于给定的同步管理器。
     * 返回值的 \a entries 字段为空。使用ecrt_master_get_pdo_entry()来获取PDO条目信息。
     *
     * @param master EtherCAT主站
     * @param slave_position 从站位置
     * @param sync_index 同步管理器索引。必须小于 #EC_MAX_SYNC_MANAGERS。
     * @param pos 基于零的PDO位置。
     * @param pdo 指向输出结构的指针
     * @return 成功时返回0，否则返回非零值
     */
    int ecrt_master_get_pdo(
        ec_master_t *master,     /**< EtherCAT主站 */
        uint16_t slave_position, /**< 从站位置 */
        uint8_t sync_index,      /**< 同步管理器索引。必须小于 #EC_MAX_SYNC_MANAGERS。 */
        uint16_t pos,            /**< 基于零的PDO位置。 */
        ec_pdo_info_t *pdo       /**< 指向输出结构的指针 */
    );

    /**
     * @brief 返回当前映射的PDO条目的信息。
     *
     * 使用给定的ec_pdo_entry_info_t结构填充当前映射的PDO条目的属性，该条目属于给定的PDO。
     *
     * @param master EtherCAT主站
     * @param slave_position 从站位置
     * @param sync_index 同步管理器索引。必须小于 #EC_MAX_SYNC_MANAGERS。
     * @param pdo_pos 基于零的PDO位置。
     * @param entry_pos 基于零的PDO条目位置。
     * @param entry 指向输出结构的指针
     * @return 成功时返回0，否则返回非零值
     */
    int ecrt_master_get_pdo_entry(
        ec_master_t *master,       /**< EtherCAT主站 */
        uint16_t slave_position,   /**< 从站位置 */
        uint8_t sync_index,        /**< 同步管理器索引。必须小于 #EC_MAX_SYNC_MANAGERS。 */
        uint16_t pdo_pos,          /**< 基于零的PDO位置。 */
        uint16_t entry_pos,        /**< 基于零的PDO条目位置。 */
        ec_pdo_entry_info_t *entry /**< 指向输出结构的指针 */
    );

#endif /* #ifndef __KERNEL__ */

    /**
     * @brief 执行SDO下载请求，向从站写入数据。
     *
     * 此请求由主站状态机处理。此方法会阻塞，直到请求被处理完毕，并且不能在实时上下文中调用。
     *
     * @param master EtherCAT主站
     * @param slave_position 从站位置
     * @param index SDO的索引
     * @param subindex SDO的子索引
     * @param data 要下载的数据缓冲区
     * @param data_size 数据缓冲区的大小
     * @param abort_code SDO下载的中止代码
     * @return 成功时返回0，否则返回负数错误代码
     */
    int ecrt_master_sdo_download(
        ec_master_t *master,     /**< EtherCAT主站 */
        uint16_t slave_position, /**< 从站位置 */
        uint16_t index,          /**< SDO的索引 */
        uint8_t subindex,        /**< SDO的子索引 */
        const uint8_t *data,     /**< 要下载的数据缓冲区 */
        size_t data_size,        /**< 数据缓冲区的大小 */
        uint32_t *abort_code     /**< SDO下载的中止代码 */
    );

    /**
     * @brief 执行SDO下载请求，通过完全访问方式向从站写入数据。
     *
     * 此请求由主站状态机处理。此方法会阻塞，直到请求被处理完毕，并且不能在实时上下文中调用。
     *
     * @param master EtherCAT主站
     * @param slave_position 从站位置
     * @param index SDO的索引
     * @param data 要下载的数据缓冲区
     * @param data_size 数据缓冲区的大小
     * @param abort_code SDO下载的中止代码
     * @return 成功时返回0，否则返回负数错误代码
     */
    int ecrt_master_sdo_download_complete(
        ec_master_t *master,     /**< EtherCAT主站 */
        uint16_t slave_position, /**< 从站位置 */
        uint16_t index,          /**< SDO的索引 */
        const uint8_t *data,     /**< 要下载的数据缓冲区 */
        size_t data_size,        /**< 数据缓冲区的大小 */
        uint32_t *abort_code     /**< SDO下载的中止代码 */
    );

    /**
     * @brief 执行SDO上传请求，从从站读取数据。
     *
     * 此请求由主站状态机处理。此方法会阻塞，直到请求被处理完毕，并且不能在实时上下文中调用。
     *
     * @param master EtherCAT主站
     * @param slave_position 从站位置
     * @param index SDO的索引
     * @param subindex SDO的子索引
     * @param target 用于上传的目标缓冲区
     * @param target_size 目标缓冲区的大小
     * @param result_size 上传的数据大小
     * @param abort_code SDO上传的中止代码
     * @return 成功时返回0，否则返回负数错误代码
     */
    int ecrt_master_sdo_upload(
        ec_master_t *master,     /**< EtherCAT主站 */
        uint16_t slave_position, /**< 从站位置 */
        uint16_t index,          /**< SDO的索引 */
        uint8_t subindex,        /**< SDO的子索引 */
        uint8_t *target,         /**< 用于上传的目标缓冲区 */
        size_t target_size,      /**< 目标缓冲区的大小 */
        size_t *result_size,     /**< 上传的数据大小 */
        uint32_t *abort_code     /**< SDO上传的中止代码 */
    );

    /**
     * @brief 执行SDO上传请求，通过完全访问方式从从站读取数据。
     *
     * 此请求由主站状态机处理。此方法会阻塞，直到请求被处理完毕，并且不能在实时上下文中调用。
     *
     * @param master EtherCAT主站
     * @param slave_position 从站位置
     * @param index SDO的索引
     * @param target 用于上传的目标缓冲区
     * @param target_size 目标缓冲区的大小
     * @param result_size 上传的数据大小
     * @param abort_code SDO上传的中止代码
     * @return 成功时返回0，否则返回负数错误代码
     */
    int ecrt_master_sdo_upload_complete(
        ec_master_t *master,     /**< EtherCAT主站 */
        uint16_t slave_position, /**< 从站位置 */
        uint16_t index,          /**< SDO的索引 */
        uint8_t *target,         /**< 用于上传的目标缓冲区 */
        size_t target_size,      /**< 目标缓冲区的大小 */
        size_t *result_size,     /**< 上传的数据大小 */
        uint32_t *abort_code     /**< SDO上传的中止代码 */
    );

    /**
     * @brief 执行SoE写入请求。
     *
     * 开始写入一个IDN并阻塞，直到请求被处理完毕或发生错误。
     *
     * @param master EtherCAT主站
     * @param slave_position 从站位置
     * @param drive_no 驱动器编号
     * @param idn SoE IDN（参见ecrt_slave_config_idn()）
     * @param data 要写入的数据的指针
     * @param data_size 要写入的数据的大小
     * @param error_code 可以存储SoE错误代码的变量的指针
     * @return 成功时返回0，否则返回负数错误代码
     */
    int ecrt_master_write_idn(
        ec_master_t *master,     /**< EtherCAT主站 */
        uint16_t slave_position, /**< 从站位置 */
        uint8_t drive_no,        /**< 驱动器编号 */
        uint16_t idn,            /**< SoE IDN（参见ecrt_slave_config_idn()） */
        uint8_t *data,           /**< 要写入的数据的指针 */
        size_t data_size,        /**< 要写入的数据的大小 */
        uint16_t *error_code     /**< 可以存储SoE错误代码的变量的指针 */
    );

    /**
     * @brief 执行SoE读取请求。
     *
     * 开始读取一个IDN并阻塞，直到请求被处理完毕或发生错误。
     *
     * @param master EtherCAT主站
     * @param slave_position 从站位置
     * @param drive_no 驱动器编号
     * @param idn SoE IDN（参见ecrt_slave_config_idn()）
     * @param target 用于存储读取数据的内存指针
     * @param target_size 内存指针target指向的大小
     * @param result_size 接收到的数据的实际大小
     * @param error_code 可以存储SoE错误代码的变量的指针
     * @return 成功时返回0，否则返回负数错误代码
     */
    int ecrt_master_read_idn(
        ec_master_t *master,     /**< EtherCAT主站 */
        uint16_t slave_position, /**< 从站位置 */
        uint8_t drive_no,        /**< 驱动器编号 */
        uint16_t idn,            /**< SoE IDN（参见ecrt_slave_config_idn()） */
        uint8_t *target,         /**< 用于存储读取数据的内存指针 */
        size_t target_size,      /**< 内存指针target指向的大小 */
        size_t *result_size,     /**< 接收到的数据的实际大小 */
        uint16_t *error_code     /**< 可以存储SoE错误代码的变量的指针 */
    );

    /**
     * @brief 完成配置阶段并准备进行周期性操作。
     *
     * 此函数告知主站配置阶段已经完成，并且实时操作将开始。该函数为域分配内部内存，并为域成员计算逻辑FMMU地址。它告诉主站状态机现在应用总线配置。
     *
     * 注意：在调用此函数之后，实时应用程序负责循环调用ecrt_master_send()和ecrt_master_receive()以确保总线通信。在调用此函数之前，主线程负责此操作，因此不能调用这些函数！该方法分配内存，不应在实时上下文中调用。
     *
     * @return 成功时返回0，否则返回负数错误代码
     */
    int ecrt_master_activate(ec_master_t *master /**< EtherCAT主站 */
    );

    /**
     * @brief 取消激活从站分布式时钟并将从站发送到PREOP状态。
     *
     * 这可以在ecrt_master_deactivate之前调用，以避免从站出现同步错误。
     *
     * 此方法应在实时上下文中调用。
     *
     * 注意：EoE从站不会更改为PREOP状态。
     */
    void ecrt_master_deactivate_slaves(ec_master_t *master /**< EtherCAT主站 */
    );

    /**
     * @brief 取消激活主站。
     *
     * 删除总线配置。由ecrt_master_create_domain()、ecrt_master_slave_config()、ecrt_domain_data()、ecrt_slave_config_create_sdo_request()和ecrt_slave_config_create_voe_handler()创建的所有对象都将被释放，因此指向它们的指针将变为无效。
     *
     * 此方法不应在实时上下文中调用。
     */
    void ecrt_master_deactivate(ec_master_t *master /**< EtherCAT主站 */
    );

    /**
     * @brief 设置ecrt_master_send()调用之间的间隔。
     *
     * 此信息帮助主站决定主站状态机可以向帧附加多少数据。当主站配置为--enable-hrtimers时，用于计算主线程的调度。
     *
     * @param master EtherCAT主站
     * @param send_interval 发送间隔，单位为微秒
     * @return 成功时返回0，否则返回负数错误代码
     */
    int ecrt_master_set_send_interval(
        ec_master_t *master, /**< EtherCAT主站 */
        size_t send_interval /**< 发送间隔，单位为微秒 */
    );

    /**
     * @brief 发送队列中的所有数据报。
     *
     * 此方法将所有已排队等待发送的数据报放入帧中，并将其传递给以太网设备进行发送。
     *
     * 在ecrt_master_activate()返回后，应用程序需要周期性调用此方法。
     *
     * 返回发送的字节数。
     */
    size_t ecrt_master_send(ec_master_t *master /**< EtherCAT主站 */
    );

    /**
     * @brief 从硬件获取接收到的帧并处理数据报。
     *
     * 通过调用中断服务例程查询网络设备以获取接收到的帧。提取接收到的数据报并将结果分派给队列中的数据报对象。接收到的数据报和超时的数据报将被标记并出队。
     *
     * 在ecrt_master_activate()返回后，实时应用程序需要周期性调用此方法。
     */
    void ecrt_master_receive(ec_master_t *master /**< EtherCAT主站 */
    );

    /**
     * @brief 发送非应用数据报。
     *
     * 必须在通过ecrt_master_callbacks()传递的发送回调函数中调用此方法，以允许发送非应用数据报。
     *
     * 返回发送的字节数。
     */
    size_t ecrt_master_send_ext(ec_master_t *master /**< EtherCAT主站 */
    );

#if !defined(__KERNEL__) && defined(EC_RTDM) && (EC_EOE)

    /**
     * @brief 检查是否存在打开的EOE处理程序。
     *
     * 由用户空间代码用于处理EOE处理程序。
     *
     * @return 如果存在打开的EOE处理程序，则返回1；如果不存在，则返回0；否则返回负数错误代码。
     */
    int ecrt_master_eoe_is_open(ec_master_t *master /**< EtherCAT主站 */
    );

/** return flag from ecrt_master_eoe_process() to indicate there is
 * something to send.  if this flag is set call ecrt_master_send_ext()
 */
#define EOE_STH_TO_SEND 1

    /** return flag from ecrt_master_eoe_process() to indicate there is
     * something still pending.  if this flag is set yield the process
     * before starting the cycle again quickly, else sleep for a short time
     * (e.g. 1ms)
     */

#define EOE_STH_PENDING 2

    /**
     * @brief     检查是否有任何EOE处理程序处于打开状态。
     * @details   用于用户空间代码处理EOE处理程序。
     * @param     master EtherCAT主站。
     * @retval    1表示有待发送的内容 +
     *            2表示某个EOE处理程序仍有待处理的内容。
     */
    int ecrt_master_eoe_process(ec_master_t *master /**< EtherCAT主站。 */
    );

#endif /* !defined(__KERNEL__) && defined(EC_RTDM) && (EC_EOE) */

#ifdef EC_EOE

    /**
     * @brief     添加一个EOE网络接口。
     * @details   此函数用于向EtherCAT主站添加一个EOE网络接口。
     * @param     master EtherCAT主站。
     * @param     alias 从站别名。
     * @param     posn 从站位置。
     * @retval    成功返回0，否则返回负错误代码。
     */
    int ecrt_master_eoe_addif(ec_master_t *master, /**< EtherCAT主站。 */
                              uint16_t alias,      /**< 从站别名。 */
                              uint16_t posn        /**< 从站位置。 */
    );

    /**
     * @brief     删除一个EOE网络接口。
     * @details   此函数用于从EtherCAT主站删除一个EOE网络接口。
     * @param     master EtherCAT主站。
     * @param     alias 从站别名。
     * @param     posn 从站位置。
     * @retval    成功返回0，否则返回负错误代码。
     */
    int ecrt_master_eoe_delif(ec_master_t *master, /**< EtherCAT主站。 */
                              uint16_t alias,      /**< 从站别名。 */
                              uint16_t posn        /**< 从站位置。 */
    );

#endif /* EC_EOE */

    /**
     * @brief     读取当前主站状态。
     * @details   将主站状态信息存储在给定的 \a state 结构中。
     *
     * 该方法返回全局状态。对于冗余总线拓扑中的链路特定状态，请使用 ecrt_master_link_state() 方法。
     *
     * @param     master EtherCAT主站。
     * @param     state 用于存储信息的结构体。
     */
    void ecrt_master_state(
        const ec_master_t *master, /**< EtherCAT主站。 */
        ec_master_state_t *state   /**< 用于存储信息的结构体。 */
    );

    /**
     * @brief     读取冗余链路的当前状态。
     * @details   将链路状态信息存储在给定的 \a state 结构中。
     *
     * @param     master EtherCAT主站。
     * @param     dev_idx 设备索引（0 = 主设备，1 = 第一个备份设备，...）。
     * @param     state 用于存储信息的结构体。
     * @retval    成功返回0，否则返回负错误代码。
     */
    int ecrt_master_link_state(
        const ec_master_t *master,    /**< EtherCAT主站。 */
        unsigned int dev_idx,         /**< 设备索引。 */
        ec_master_link_state_t *state /**< 用于存储信息的结构体。 */
    );

    /**
     * @brief     设置应用程序时间。
     * @details   当使用分布式时钟操作从站时，主站需要知道应用程序的时间。时间不会由主站自行递增，因此必须周期性调用此方法。
     *
     * @attention 传递给此方法的时间用于计算从站的SYNC0/1中断的相位。它应该在实时周期的相同点不断调用。因此，建议在计算开始时调用它，以避免由于执行时间的变化而产生偏差。
     *
     * 时间在设置从站的<tt>System Time Offset</tt>和<tt>Cyclic Operation Start Time</tt>寄存器以及通过 ecrt_master_sync_reference_clock() 将DC参考时钟与应用程序时间同步时使用。
     *
     * 时间定义为从2000-01-01 00:00开始的纳秒数。可以使用 EC_TIMEVAL2NANO() 宏将纪元时间转换为纳秒数，但这并不是必需的，因为绝对值并不重要。
     *
     * @param     master EtherCAT主站。
     * @param     app_time 应用程序时间。
     */
    void ecrt_master_application_time(ec_master_t *master, /**< EtherCAT主站。 */
                                      uint64_t app_time    /**< 应用程序时间。 */
    );

    /**
     * @brief     将DC参考时钟漂移补偿数据报文加入发送队列。
     * @details   参考时钟将与最后一次调用 ecrt_master_application_time() 提供的应用程序时间同步。
     *
     * @param     master EtherCAT主站。
     */
    void ecrt_master_sync_reference_clock(
        ec_master_t *master /**< EtherCAT主站。 */
    );

    /**
     * @brief     将DC参考时钟漂移补偿数据报文加入发送队列。
     * @details   参考时钟将与 sync_time 参数中传递的时间同步。
     *
     * @param     master EtherCAT主站。
     * @param     sync_time 将参考时钟同步到此时间。
     */
    void ecrt_master_sync_reference_clock_to(
        ec_master_t *master, /**< EtherCAT主站。 */
        uint64_t sync_time   /**< 同步参考时钟的时间。 */
    );

    /**
     * @brief     将DC时钟漂移补偿数据报文加入发送队列。
     * @details   所有从站时钟与参考时钟同步。
     *
     * @param     master EtherCAT主站。
     */
    void
    ecrt_master_sync_slave_clocks(ec_master_t *master /**< EtherCAT主站。 */
    );

    /**
     * @brief     获取参考时钟系统时间的低32位。
     * @details   此方法可用于将主站与参考时钟同步。
     *
     * 参考时钟系统时间通过 ecrt_master_sync_slave_clocks() 方法查询，该方法读取参考时钟的系统时间并将其写入从站时钟（因此请确保周期性调用以获取有效数据）。
     *
     * @attention 返回的时间是参考时钟的系统时间减去参考时钟的传输延迟。
     *
     * @param     master EtherCAT主站。
     * @param     time 用于存储查询的系统时间的指针。
     * @retval    0 成功，系统时间已写入 \a time。
     * @retval    -ENXIO 未找到参考时钟。
     * @retval    -EIO 未接收到从站同步数据报文。
     */
    int ecrt_master_reference_clock_time(
        ec_master_t *master, /**< EtherCAT主站。 */
        uint32_t *time       /**< 用于存储查询的系统时间的指针。 */
    );

    /**
     * @brief     将64位DC参考从站时钟时间值数据报文加入发送队列。
     * @details   该数据报文读取DC参考从站的64位DC时间戳（寄存器 \a 0x0910:0x0917）。结果可使用 ecrt_master_64bit_reference_clock_time() 方法进行检查。
     *
     * @param     master EtherCAT主站。
     */
    void ecrt_master_64bit_reference_clock_time_queue(
        ec_master_t *master /**< EtherCAT主站。 */
    );

    /**
     * @brief     获取64位DC参考从站时钟时间。
     * @details   在调用此方法之前，必须在周期中调用 ecrt_master_64bit_reference_clock_time_queue()。
     *
     * @attention 返回的时间是参考时钟的系统时间减去参考时钟的传输延迟。
     *
     * @param     master EtherCAT主站。
     * @param     time 用于存储查询的时间的指针。
     * @retval    0 成功，系统时间已写入 \a time。
     * @retval    -ENXIO 未找到参考时钟。
     * @retval    -EIO 未接收到从站同步数据报文。
     */
    int ecrt_master_64bit_reference_clock_time(
        ec_master_t *master, /**< EtherCAT主站。 */
        uint64_t *time       /**< 用于存储查询的时间的指针。 */
    );

    /**
     * @brief     将DC同步监控数据报文加入发送队列。
     * @details   该数据报文广播读取所有的“System time difference”寄存器（\a 0x092c），以获取DC同步的上限估计。结果可使用 ecrt_master_sync_monitor_process() 方法进行检查。
     *
     * @param     master EtherCAT主站。
     */
    void
    ecrt_master_sync_monitor_queue(ec_master_t *master /**< EtherCAT主站。 */
    );

    /**
     * @brief     处理DC同步监控数据报文。
     * @details   如果之前使用 ecrt_master_sync_monitor_queue() 发送了同步监控数据报文，则可以使用此方法查询结果。
     *
     * @param     master EtherCAT主站。
     * @return    最大时间差的上限估计（以纳秒为单位）。
     */
    uint32_t ecrt_master_sync_monitor_process(
        ec_master_t *master /**< EtherCAT主站。 */
    );

    /**
     * @brief     选择由应用程序还是主站处理从站请求。
     * @details   如果 rt_slave_requests 为 \a True，则从站请求将由应用程序的实时上下文通过调用 ecrt_master_exec_requests() 处理，否则主站将在其操作线程中处理它们。
     *
     * @param     master EtherCAT主站。
     * @param     rt_slave_requests 如果为 \a True，则从站请求将由应用程序的实时上下文处理。
     * @return    成功返回0，否则返回负错误代码。
     */
    int ecrt_master_rt_slave_requests(
        ec_master_t *master,           /**< EtherCAT主站。 */
        unsigned int rt_slave_requests /**< 如果为 \a True，则从站请求将由应用程序的实时上下文处理。 */
    );

    /**
     * @brief     显式调用以处理从站请求。
     * @details   如果在调用 ecrt_master_rt_slave_requests() 时将 rt_slave_requests 设置为 true，则应用程序的实时上下文需要按周期调用此函数来处理从站请求。如果 rt_slave_requests 为 \a False（默认值），则从站请求将在主站内部处理，此调用将被忽略。
     *
     * @param     master EtherCAT主站。
     */
    void
    ecrt_master_exec_slave_requests(ec_master_t *master /**< EtherCAT主站。 */
    );

    /**
     * @brief     重新尝试配置从站。
     * @details   通过此方法，应用程序可以告知主站将所有从站置于OP状态。通常情况下，这是不必要的，因为主站会自动执行此操作。但对于可以在运行时由供应商重新配置的特殊从站，这可能是有用的。
     *
     * @param     master EtherCAT主站。
     */
    void ecrt_master_reset(ec_master_t *master /**< EtherCAT主站。 */
    );

    /******************************************************************************
     * Slave configuration methods
     *****************************************************************************/

    /**
     * @brief     配置同步管理器。
     * @details   设置同步管理器的方向。这将覆盖SII中默认控制寄存器的方向位。
     *
     * 此方法必须在非实时上下文中在调用 ecrt_master_activate() 之前调用。
     *
     * @param     sc 从站配置。
     * @param     sync_index 同步管理器索引。必须小于 #EC_MAX_SYNC_MANAGERS。
     * @param     direction 输入/输出。
     * @param     watchdog_mode 看门狗模式。
     * @return    成功返回零，否则返回非零值。
     */
    int ecrt_slave_config_sync_manager(
        ec_slave_config_t *sc,           /**< 从站配置。 */
        uint8_t sync_index,              /**< 同步管理器索引。必须小于 #EC_MAX_SYNC_MANAGERS。 */
        ec_direction_t direction,        /**< 输入/输出。 */
        ec_watchdog_mode_t watchdog_mode /**< 看门狗模式。 */
    );

    /**
     * @brief     配置从站的看门狗定时器。
     * @details   此方法必须在非实时上下文中在调用 ecrt_master_activate() 之前调用。
     *
     * @param     sc 从站配置。
     * @param     watchdog_divider 40 ns间隔数。用作所有从站看门狗的基本单位。如果设置为零，则不写入该值，使用默认值。
     * @param     watchdog_intervals 进程数据看门狗的基本间隔数。如果设置为零，则不写入该值，使用默认值。
     */
    void ecrt_slave_config_watchdog(
        ec_slave_config_t *sc,      /**< 从站配置。 */
        uint16_t watchdog_divider,  /**< 40 ns间隔数。用作所有从站看门狗的基本单位。如果设置为零，则不写入该值，使用默认值。 */
        uint16_t watchdog_intervals /**< 进程数据看门狗的基本间隔数。如果设置为零，则不写入该值，使用默认值。 */
    );

    /**
     * @brief     配置从站是否允许重叠的PDO。
     * @details   允许重叠的PDO使输入在帧中使用与输出相同的空间。这减少了帧长度。
     *
     * @param     sc 从站配置。
     * @param     allow_overlapping_pdos 允许重叠的PDO。
     */
    void ecrt_slave_config_overlapping_pdos(
        ec_slave_config_t *sc,         /**< 从站配置。 */
        uint8_t allow_overlapping_pdos /**< 允许重叠的PDO。 */
    );

    /**
     * @brief     将PDO添加到同步管理器的PDO分配中。
     * @details   此方法必须在非实时上下文中在调用 ecrt_master_activate() 之前调用。
     *
     * @param     sc 从站配置。
     * @param     sync_index 同步管理器索引。必须小于 #EC_MAX_SYNC_MANAGERS。
     * @param     index 要分配的PDO的索引。
     * @return    成功返回零，否则返回非零值。
     */
    int ecrt_slave_config_pdo_assign_add(
        ec_slave_config_t *sc, /**< 从站配置。 */
        uint8_t sync_index,    /**< 同步管理器索引。必须小于 #EC_MAX_SYNC_MANAGERS。 */
        uint16_t index         /**< 要分配的PDO的索引。 */
    );

    /**
     * @brief     清除同步管理器的PDO分配。
     * @details   在通过 ecrt_slave_config_pdo_assign_add() 分配PDO之前，可以调用此函数来清除同步管理器的默认分配。
     *
     * 此方法必须在非实时上下文中在调用 ecrt_master_activate() 之前调用。
     *
     * @param     sc 从站配置。
     * @param     sync_index 同步管理器索引。必须小于 #EC_MAX_SYNC_MANAGERS。
     */
    void ecrt_slave_config_pdo_assign_clear(
        ec_slave_config_t *sc, /**< 从站配置。 */
        uint8_t sync_index     /**< 同步管理器索引。必须小于 #EC_MAX_SYNC_MANAGERS。 */
    );

    /**
     * @brief     将PDO条目添加到给定PDO的映射中。
     * @details   此方法必须在非实时上下文中在调用 ecrt_master_activate() 之前调用。
     *
     * @param     sc 从站配置。
     * @param     pdo_index PDO的索引。
     * @param     entry_index 要添加到PDO映射的PDO条目的索引。
     * @param     entry_subindex 要添加到PDO映射的PDO条目的子索引。
     * @param     entry_bit_length PDO条目的位大小。
     * @return    成功返回零，否则返回非零值。
     */
    int ecrt_slave_config_pdo_mapping_add(
        ec_slave_config_t *sc,   /**< 从站配置。 */
        uint16_t pdo_index,      /**< PDO的索引。 */
        uint16_t entry_index,    /**< 要添加到PDO映射的PDO条目的索引。 */
        uint8_t entry_subindex,  /**< 要添加到PDO映射的PDO条目的子索引。 */
        uint8_t entry_bit_length /**< PDO条目的位大小。 */
    );

    /**
     * @brief     清除给定PDO的映射。
     * @details   在通过 ecrt_slave_config_pdo_mapping_add() 进行PDO条目映射之前，可以调用此函数来清除默认映射。
     *
     * 此方法必须在非实时上下文中在调用 ecrt_master_activate() 之前调用。
     *
     * @param     sc 从站配置。
     * @param     pdo_index PDO的索引。
     */
    void ecrt_slave_config_pdo_mapping_clear(
        ec_slave_config_t *sc, /**< 从站配置。 */
        uint16_t pdo_index     /**< PDO的索引。 */
    );

    /**
     * @brief     指定完整的PDO配置。
     * @details   此函数是用于方便的函数包装，包括ecrt_slave_config_sync_manager()、
     *            ecrt_slave_config_pdo_assign_clear()、ecrt_slave_config_pdo_assign_add()、
     *            ecrt_slave_config_pdo_mapping_clear() 和 ecrt_slave_config_pdo_mapping_add()，
     *            更适合于自动生成代码。
     *
     * 下面的示例展示了如何指定完整的配置，包括PDO映射。有了这些信息，即使从站在配置时不在场，主站也能够保留完整的过程数据：
     *
     * \code
     * ec_pdo_entry_info_t el3162_channel1[] = {
     *     {0x3101, 1,  8}, // 状态
     *     {0x3101, 2, 16}  // 值
     * };
     *
     * ec_pdo_entry_info_t el3162_channel2[] = {
     *     {0x3102, 1,  8}, // 状态
     *     {0x3102, 2, 16}  // 值
     * };
     *
     * ec_pdo_info_t el3162_pdos[] = {
     *     {0x1A00, 2, el3162_channel1},
     *     {0x1A01, 2, el3162_channel2}
     * };
     *
     * ec_sync_info_t el3162_syncs[] = {
     *     {2, EC_DIR_OUTPUT},
     *     {3, EC_DIR_INPUT, 2, el3162_pdos},
     *     {0xff}
     * };
     *
     * if (ecrt_slave_config_pdos(sc_ana_in, EC_END, el3162_syncs)) {
     *     // 处理错误
     * }
     * \endcode
     *
     * 下一个示例展示了如何仅配置PDO分配。每个分配的PDO的条目取自PDO的默认映射。
     * 请注意，如果PDO配置保持为空且从站处于离线状态，PDO条目注册将失败。
     *
     * \code
     * ec_pdo_info_t pdos[] = {
     *     {0x1600}, // 通道1
     *     {0x1601}  // 通道2
     * };
     *
     * ec_sync_info_t syncs[] = {
     *     {3, EC_DIR_INPUT, 2, pdos},
     * };
     *
     * if (ecrt_slave_config_pdos(slave_config_ana_in, 1, syncs)) {
     *     // 处理错误
     * }
     * \endcode
     *
     * 如果满足以下条件之一，将停止处理 \a syncs：
     * - 已处理的项目数达到 \a n_syncs，或者
     * - ec_sync_info_t 项的 \a index 成员为 0xff。在这种情况下，\a n_syncs 应设置为大于列表项数的数字；
     *   推荐使用 EC_END。
     *
     * 此方法必须在非实时上下文中在调用 ecrt_master_activate() 之前调用。
     *
     * @param     sc 从站配置。
     * @param     n_syncs 同步管理器配置的数量。
     * @param     syncs 同步管理器配置的数组。
     * @return    成功返回零，否则返回非零值。
     */
    int ecrt_slave_config_pdos(
        ec_slave_config_t *sc,       /**< 从站配置。 */
        unsigned int n_syncs,        /**< 同步管理器配置的数量。 */
        const ec_sync_info_t syncs[] /**< 同步管理器配置的数组。 */
    );

    /**
     * @brief     注册一个用于在域中进行过程数据交换的PDO条目。
     * @details   在分配的PDO中搜索给定的PDO条目。如果给定的条目没有映射，将引发错误。
     *            否则，提供相应的同步管理器和FMMU配置以进行从站配置，并将相应的同步管理器的分配的
     *            PDO附加到给定的域中（如果尚未完成）。返回请求的PDO条目数据在域的过程数据中的偏移量。
     *            可选地，可以通过 \a bit_position 输出参数检索PDO条目的位位置（0-7）。
     *            如果该指针为 \a NULL，则在PDO条目不字节对齐的情况下引发错误。
     *
     *            此方法必须在非实时上下文中在调用 ecrt_master_activate() 之前调用。
     *
     * @param     sc 从站配置。
     * @param     entry_index 要注册的PDO条目的索引。
     * @param     entry_subindex 要注册的PDO条目的子索引。
     * @param     domain 域。
     * @param     bit_position 可选地址，如果需要位寻址。
     * @retval    >=0 成功：PDO条目的过程数据的偏移量。
     * @retval    <0 错误代码。
     */
    int ecrt_slave_config_reg_pdo_entry(
        ec_slave_config_t *sc,     /**< 从站配置。 */
        uint16_t entry_index,      /**< 要注册的PDO条目的索引。 */
        uint8_t entry_subindex,    /**< 要注册的PDO条目的子索引。 */
        ec_domain_t *domain,       /**< 域。 */
        unsigned int *bit_position /**< 可选地址，如果需要位寻址。 */
    );

    /**
     * @brief     使用位置注册PDO条目。
     * @details   类似于ecrt_slave_config_reg_pdo_entry()，但不使用PDO索引，而是使用PDO映射中的偏移量，
     *            因为PDO条目索引在从站的PDO映射中可能不是唯一的。如果给定的位置超出范围，将引发错误。
     *
     *            此方法必须在非实时上下文中在调用 ecrt_master_activate() 之前调用。
     *
     * @param     sc 从站配置。
     * @param     sync_index 同步管理器索引。
     * @param     pdo_pos PDO在SM中的位置。
     * @param     entry_pos 条目在PDO中的位置。
     * @param     domain 域。
     * @param     bit_position 可选地址，如果需要位寻址。
     * @retval    >=0 成功：PDO条目的过程数据的偏移量。
     * @retval    <0 错误代码。
     */
    int ecrt_slave_config_reg_pdo_entry_pos(
        ec_slave_config_t *sc,     /**< 从站配置。 */
        uint8_t sync_index,        /**< 同步管理器索引。 */
        unsigned int pdo_pos,      /**< PDO在SM中的位置。 */
        unsigned int entry_pos,    /**< 条目在PDO中的位置。 */
        ec_domain_t *domain,       /**< 域。 */
        unsigned int *bit_position /**< 可选地址，如果需要位寻址。 */
    );

    /**
     * @brief     配置分布式时钟。
     * @details   设置AssignActivate字和同步信号的周期和偏移时间。
     *
     * AssignActivate字是供应商特定的，可以从XML设备描述文件中获取（Device -> Dc -> AssignActivate）。
     * 如果从站不使用分布式时钟，则将其设置为零（默认值）。
     *
     * 此方法必须在非实时上下文中在调用ecrt_master_activate()之前调用。
     *
     * \attention 忽略 \a sync1_shift 时间。
     *
     * @param     sc 从站配置。
     * @param     assign_activate AssignActivate字。
     * @param     sync0_cycle SYNC0周期时间[纳秒]。
     * @param     sync0_shift SYNC0偏移时间[纳秒]。
     * @param     sync1_cycle SYNC1周期时间[纳秒]。
     * @param     sync1_shift SYNC1偏移时间[纳秒]。
     * @return    无返回值。
     */
    void ecrt_slave_config_dc(
        ec_slave_config_t *sc,    /**< 从站配置。 */
        uint16_t assign_activate, /**< AssignActivate字。 */
        uint32_t sync0_cycle,     /**< SYNC0周期时间[纳秒]。 */
        int32_t sync0_shift,      /**< SYNC0偏移时间[纳秒]。 */
        uint32_t sync1_cycle,     /**< SYNC1周期时间[纳秒]。 */
        int32_t sync1_shift       /**< SYNC1偏移时间[纳秒]。 */
    );

    /**
     * @brief     添加SDO配置。
     * @details   SDO配置存储在从站配置对象中，并在每次从主站配置从站时下载到从站。
     *            这通常在主站激活时进行一次，但可以在之后重复执行，例如在从站的电源供应失败后。
     *
     * \attention 不应使用此函数配置PDO分配（\p 0x1C10 - \p 0x1C2F）和PDO映射（\p 0x1600 - \p 0x17FF 和 \p 0x1A00 - \p 0x1BFF），
     *            因为它们是主站完成的从站配置的一部分。请改用 ecrt_slave_config_pdos() 和相关函数。
     *
     * 此函数用于添加通用的SDO配置。请注意，此函数不执行任何字节序修正。
     * 如果需要特定数据类型的函数（自动修正字节序），请参考 ecrt_slave_config_sdo8()、ecrt_slave_config_sdo16() 和 ecrt_slave_config_sdo32()。
     *
     * 此方法必须在非实时上下文中在调用 ecrt_master_activate() 之前调用。
     *
     * @param     sc 从站配置。
     * @param     index 要配置的SDO的索引。
     * @param     subindex 要配置的SDO的子索引。
     * @param     data 指向数据的指针。
     * @param     size \a data 的大小。
     * @return    成功返回 0，错误返回负数的错误代码。
     */
    int ecrt_slave_config_sdo(
        ec_slave_config_t *sc, /**< 从站配置。 */
        uint16_t index,        /**< 要配置的SDO的索引。 */
        uint8_t subindex,      /**< 要配置的SDO的子索引。 */
        const uint8_t *data,   /**< 指向数据的指针。 */
        size_t size            /**< \a data 的大小。 */
    );

    /**
     * @brief     添加8位SDO的配置值。
     *
     * 此方法必须在非实时上下文中在调用ecrt_master_activate()之前调用。
     *
     * \see ecrt_slave_config_sdo()。
     *
     * \retval  0 成功。
     * \retval <0 错误代码。
     */
    int ecrt_slave_config_sdo8(
        ec_slave_config_t *sc, /**< 从站配置 */
        uint16_t sdo_index,    /**< 要配置的SDO的索引。 */
        uint8_t sdo_subindex,  /**< 要配置的SDO的子索引。 */
        uint8_t value          /**< 要设置的值。 */
    );

    /**
     * @brief     添加16位SDO的配置值。
     *
     * 此方法必须在非实时上下文中在调用ecrt_master_activate()之前调用。
     *
     * \see ecrt_slave_config_sdo()。
     *
     * \retval  0 成功。
     * \retval <0 错误代码。
     */
    int ecrt_slave_config_sdo16(
        ec_slave_config_t *sc, /**< 从站配置 */
        uint16_t sdo_index,    /**< 要配置的SDO的索引。 */
        uint8_t sdo_subindex,  /**< 要配置的SDO的子索引。 */
        uint16_t value         /**< 要设置的值。 */
    );

    /**
     * @brief     添加32位SDO的配置值。
     *
     * 此方法必须在非实时上下文中在调用ecrt_master_activate()之前调用。
     *
     * \see ecrt_slave_config_sdo()。
     *
     * \retval  0 成功。
     * \retval <0 错误代码。
     */
    int ecrt_slave_config_sdo32(
        ec_slave_config_t *sc, /**< 从站配置 */
        uint16_t sdo_index,    /**< 要配置的SDO的索引。 */
        uint8_t sdo_subindex,  /**< 要配置的SDO的子索引。 */
        uint32_t value         /**< 要设置的值。 */
    );

    /**
     * @brief     添加完整SDO的配置数据。
     *
     * SDO数据通过CompleteAccess传输。必须包含第一个子索引（0）的数据。
     *
     * 此方法必须在非实时上下文中在调用ecrt_master_activate()之前调用。
     *
     * \see ecrt_slave_config_sdo()。
     *
     * \retval  0 成功。
     * \retval <0 错误代码。
     */
    int ecrt_slave_config_complete_sdo(
        ec_slave_config_t *sc, /**< 从站配置。 */
        uint16_t index,        /**< 要配置的SDO的索引。 */
        const uint8_t *data,   /**< 指向数据的指针。 */
        size_t size            /**< \a data 的大小。 */
    );

    /**
     * @brief     设置CoE紧急环形缓冲区的大小。
     *
     * 初始大小为零，因此所有消息都将被丢弃。即使在主站激活后也可以调用此方法，但它将清除环形缓冲区！
     *
     * 此方法必须在非实时上下文中在调用ecrt_master_activate()之前调用。
     *
     * \return 成功返回0，否则返回负数的错误代码。
     */
    int ecrt_slave_config_emerg_size(
        ec_slave_config_t *sc, /**< 从站配置。 */
        size_t elements        /**< CoE紧急环形缓冲区的记录数。 */
    );

    /**
     * @brief     从CoE紧急环形缓冲区中读取并移除一条记录。
     *
     * 一条记录由8个字节组成：
     *
     * 字节 0-1：错误码（小端序）
     * 字节   2：错误寄存器
     * 字节 3-7：数据
     *
     * \return 成功返回0（弹出记录），否则返回负数的错误代码（例如 -ENOENT，如果环形缓冲区为空）。
     */
    int ecrt_slave_config_emerg_pop(
        ec_slave_config_t *sc, /**< 从站配置。 */
        uint8_t *target        /**< 目标内存的指针（至少EC_COE_EMERGENCY_MSG_SIZE字节）。 */
    );

    /**
     * @brief     清除CoE紧急环形缓冲区和溢出计数器。
     *
     * \return 成功返回0，否则返回负数的错误代码。
     */
    int ecrt_slave_config_emerg_clear(
        ec_slave_config_t *sc /**< 从站配置。 */
    );

    /**
     * @brief     读取CoE紧急溢出次数。
     *
     * 当CoE紧急消息无法存储在环形缓冲区中并且必须被丢弃时，溢出计数器将递增。调用ecrt_slave_config_emerg_clear()以重置计数器。
     *
     * \return 上次清除以来的溢出次数，否则返回负数的错误代码。
     */
    int ecrt_slave_config_emerg_overruns(
        ec_slave_config_t *sc /**< 从站配置。 */
    );

    /**
     * @brief     创建一个SDO请求，在实时操作期间交换SDO。
     *
     * 创建的SDO请求对象在主站释放时会自动释放。
     *
     * 此方法必须在非实时上下文中在调用ecrt_master_activate()之前调用。
     *
     * \return 新的SDO请求，或者在错误时返回NULL。
     */
    ec_sdo_request_t *ecrt_slave_config_create_sdo_request(
        ec_slave_config_t *sc, /**< 从站配置。 */
        uint16_t index,        /**< SDO索引。 */
        uint8_t subindex,      /**< SDO子索引。 */
        size_t size            /**< 要预留的数据大小。 */
    );

    /**
     * @brief     创建一个使用完全访问的SDO请求，在实时操作期间交换SDO。
     *
     * 创建的SDO请求对象在主站释放时会自动释放。
     *
     * 此方法必须在非实时上下文中在调用ecrt_master_activate()之前调用。
     *
     * \return 新的SDO请求，或者在错误时返回NULL。
     */
    ec_sdo_request_t *ecrt_slave_config_create_sdo_request_complete(
        ec_slave_config_t *sc, /**< 从站配置。 */
        uint16_t index,        /**< SDO索引。 */
        size_t size            /**< 要预留的数据大小。 */
    );

    /**
     * @brief     创建一个FoE请求，在实时操作期间交换文件。
     *
     * 创建的FoE请求对象在主站释放时会自动释放。
     *
     * 此方法必须在非实时上下文中在调用ecrt_master_activate()之前调用。
     *
     * \return 新的FoE请求，或者在错误时返回NULL。
     */
    ec_foe_request_t *ecrt_slave_config_create_foe_request(
        ec_slave_config_t *sc, /**< 从站配置。 */
        size_t size            /**< 要预留的数据大小。 */
    );

    /**
     * @brief     创建一个VoE处理器，在实时操作期间交换供应商特定数据。
     *
     * 每个从站配置的VoE处理器数量没有限制，但通常创建一个用于发送和一个用于接收，如果两者可以同时进行。
     *
     * 创建的VoE处理器对象在主站释放时会自动释放。
     *
     * 此方法必须在非实时上下文中在调用ecrt_master_activate()之前调用。
     *
     * \return 新的VoE处理器，或者在错误时返回NULL。
     */
    ec_voe_handler_t *ecrt_slave_config_create_voe_handler(
        ec_slave_config_t *sc, /**< 从站配置。 */
        size_t size            /**< 要预留的数据大小。 */
    );

    /**
     * @brief     创建一个寄存器请求，在实时操作期间交换EtherCAT寄存器内容。
     *
     * 此接口不应用于接管主站功能，而是用于调试和监视的目的。
     *
     * 创建的寄存器请求对象在主站释放时会自动释放。
     *
     * 此方法必须在非实时上下文中在调用ecrt_master_activate()之前调用。
     *
     * \return 新的寄存器请求，或者在错误时返回NULL。
     */
    ec_reg_request_t *ecrt_slave_config_create_reg_request(
        ec_slave_config_t *sc, /**< 从站配置。 */
        size_t size            /**< 要预留的数据大小。 */
    );

    /**
     * @brief     输出从站配置的状态。
     *
     * 将状态信息存储在给定的\a state结构中。状态信息由主站状态机更新，因此可能需要几个周期才会更改。
     *
     * \attention 如果要实时监视过程数据交换的状态，应使用ecrt_domain_state()。
     */
    void ecrt_slave_config_state(
        const ec_slave_config_t *sc,   /**< 从站配置。 */
        ec_slave_config_state_t *state /**< 要写入的状态对象。 */
    );

    /**
     * @brief     添加SoE IDN配置。
     *
     * Sercos-over-EtherCAT（SoE）IDN的配置存储在从站配置对象中，并在每次从主站配置从站时写入从站。
     * 这通常在主站激活时进行一次，但可以在之后重复执行，例如在从站的电源供应失败后。
     *
     * \a idn参数可以分为几个部分：
     *  - 位 15：标准数据（0）或产品数据（1）
     *  - 位 14 - 12：参数集（0 - 7）
     *  - 位 11 - 0：数据块号（0 - 4095）
     *
     * 请注意，此函数不执行任何字节序修正。多字节数据必须以EtherCAT字节序（小端序）传递。
     *
     * 此方法必须在非实时上下文中在调用ecrt_master_activate()之前调用。
     *
     * \retval  0 成功。
     * \retval <0 错误代码。
     */
    int ecrt_slave_config_idn(
        ec_slave_config_t *sc, /**< 从站配置。 */
        uint8_t drive_no,      /**< 驱动器编号。 */
        uint16_t idn,          /**< SoE IDN。 */
        ec_al_state_t state,   /**< 要写入IDN的AL状态（PREOP或SAFEOP）。 */
        const uint8_t *data,   /**< 指向数据的指针。 */
        size_t size            /**< \a data 的大小。 */
    );

    /******************************************************************************
     * Domain methods
     *****************************************************************************/

    /**
     * @brief     注册一组PDO条目到域中。
     *
     * 此方法必须在非实时上下文中在调用ecrt_master_activate()之前调用。
     *
     * \see ecrt_slave_config_reg_pdo_entry()
     *
     * \attention 注册数组必须以一个空结构或者一个\index字段设置为零的结构结束！
     * \return 成功返回0，否则返回非零值。
     */
    int ecrt_domain_reg_pdo_entry_list(
        ec_domain_t *domain,                     /**< 域。 */
        const ec_pdo_entry_reg_t *pdo_entry_regs /**< PDO注册数组。 */
    );

    /** 返回域的当前过程数据大小。
     *
     * \return 过程数据镜像的大小，或者负数的错误代码。
     */
    size_t ecrt_domain_size(const ec_domain_t *domain /**< 域。 */
    );

#ifdef __KERNEL__

    /**
     * @brief 提供外部内存以存储域的过程数据。
     *
     * 在所有PDO条目都注册完成并激活主站之前调用此函数。
     *
     * 分配的内存大小必须至少为ecrt_domain_size()，在所有PDO条目都注册完成后。
     *
     * 此方法必须在非实时上下文中，在激活主站之前调用。
     *
     * @param domain 域。
     * @param memory 存储过程数据的内存地址。
     */
    void ecrt_domain_external_memory(ec_domain_t *domain, /**< 域。 */
                                     uint8_t *memory      /**< 存储过程数据的内存地址。 */
    );

#endif /* __KERNEL__ */

    /**
     * @brief       返回域的进程数据。
     * @details     - 在内核上下文中：如果使用ecrt_domain_external_memory()提供了外部内存，
     *                  返回的指针将包含该内存的地址。否则，它将指向内部分配的内存。
     *                  在后一种情况下，此方法在ecrt_master_activate()之前可能无法调用。
     *              - 在用户空间上下文中：必须在ecrt_master_activate()之后调用此方法以获取映射的域进程数据内存。
     * @param       domain 域。
     * @retval      指向进程数据内存的指针。
     */
    uint8_t *ecrt_domain_data(ec_domain_t *domain /**< 域。 */
    );

    /**
     * @brief       确定域数据报的状态。
     * @details     评估接收到的数据报的工作计数器，并在必要时输出统计信息。
     *              在调用ecrt_master_receive()之后，应调用此函数来接收域数据报，
     *              以使ecrt_domain_state()返回最后一次进程数据交换的结果。
     * @param       domain 域。
     */
    void ecrt_domain_process(ec_domain_t *domain /**< 域。 */
    );

    /**
     * @brief       （重新）将所有域数据报排队到主站数据报队列中。
     * @details     在下一次调用ecrt_master_send()时，调用此函数标记域数据报进行交换。
     * @param       domain 域。
     */
    void ecrt_domain_queue(ec_domain_t *domain /**< 域。 */
    );

    /**
     * @brief       读取域的状态。
     * @details     将域的状态存储在给定的状态结构中。
     *              使用此方法可以实时监视进程数据交换。
     * @param       domain 域。
     * @param       state 存储信息的状态对象的指针。
     */
    void ecrt_domain_state(const ec_domain_t *domain, /**< 域。 */
                           ec_domain_state_t *state   /**< 指向状态对象的指针，用于存储信息。 */
    );

    /*****************************************************************************
     * SDO request methods.
     ****************************************************************************/

    /**
     * @brief       设置SDO索引和子索引，并准备进行非完全访问。
     * @details     即使请求是为完全访问创建的，此操作也是有效的。
     * @attention   如果在ecrt_sdo_request_state()返回EC_REQUEST_BUSY时更改SDO索引和/或子索引，
     *              可能会导致意外结果。
     * @param       req SDO请求。
     * @param       index SDO索引。
     * @param       subindex SDO子索引。
     */
    void ecrt_sdo_request_index(ec_sdo_request_t *req, /**< SDO请求。 */
                                uint16_t index,        /**< SDO索引。 */
                                uint8_t subindex       /**< SDO子索引。 */
    );

    /**
     * @brief       设置SDO索引并准备进行完全访问。
     * @details     即使请求未为完全访问创建，此操作也是有效的。
     * @attention   如果在ecrt_sdo_request_state()返回EC_REQUEST_BUSY时更改SDO索引，
     *              可能会导致意外结果。
     * @param       req SDO请求。
     * @param       index SDO索引。
     */
    void ecrt_sdo_request_index_complete(ec_sdo_request_t *req, /**< SDO请求。 */
                                         uint16_t index         /**< SDO索引。 */
    );

    /**
     * @brief       设置SDO请求的超时时间。
     * @details     如果请求在指定的时间内无法处理，将标记为失败。
     *              超时时间将永久存储在请求对象中，并在下一次调用此方法时生效。
     * @param       req SDO请求。
     * @param       timeout 超时时间（以毫秒为单位）。
     *              零表示无超时。
     */
    void ecrt_sdo_request_timeout(ec_sdo_request_t *req, /**< SDO请求。 */
                                  uint32_t timeout       /**< 超时时间（以毫秒为单位）。
                                                           零表示无超时。 */
    );

    /**
     * @brief       访问SDO请求的数据。
     * @details     此函数返回指向请求的内部SDO数据内存的指针。
     *              - 在读取操作成功后，可以使用EC_READ_*()宏来评估整数数据。例如：
     *                \code uint16_t value = EC_READ_U16(ecrt_sdo_request_data(sdo))); \endcode
     *              - 如果要触发写操作，则必须将数据写入内部内存。
     *                如果要写入整数数据，请使用EC_WRITE_*()宏。
     *                确保数据适合内存中。内存大小是ecrt_slave_config_create_sdo_request()的参数。
     *                \code EC_WRITE_U16(ecrt_sdo_request_data(sdo), 0xFFFF); \endcode
     * @attention   在读取操作期间，返回值可能无效，因为如果读取的SDO数据不适合内部SDO数据内存，
     *              则可能会重新分配内部SDO数据内存。
     * @return      指向内部SDO数据内存的指针。
     */
    uint8_t *ecrt_sdo_request_data(ec_sdo_request_t *req /**< SDO请求。 */
    );

    /**
     * @brief       返回当前SDO数据的大小。
     * @details     在创建SDO请求时，数据大小设置为保留内存的大小。
     *              在读取操作后，大小设置为读取数据的大小。
     *              在其他情况下，大小不会被修改。
     * @param       req SDO请求。
     * @return      SDO数据大小（以字节为单位）。
     */
    size_t ecrt_sdo_request_data_size(const ec_sdo_request_t *req /**< SDO请求。 */
    );

/**
 * @brief       获取SDO请求的当前状态。
 * @details     这是一个简要描述。
 * @param       req SDO请求。
 * @retval      请求状态。
 */
#ifdef __KERNEL__
    ec_request_state_t
    ecrt_sdo_request_state(const ec_sdo_request_t *req /**< SDO请求。 */
    );
#else
ec_request_state_t
ecrt_sdo_request_state(ec_sdo_request_t *req /**< SDO请求。 */
);
#endif

   /**
 * @brief       安排一个SDO写操作。
 * @details     这是一个简要描述。
 * @param       req SDO请求。
 * @retval      无返回值。
 *
 * \attention   在ecrt_sdo_request_state()返回EC_REQUEST_BUSY时，不可调用此方法。
 */
void ecrt_sdo_request_write(ec_sdo_request_t *req /**< SDO请求。 */
);

/** 
 * @brief       安排一个SDO写操作。
 * @details     这是一个详细描述。
 * @param       req SDO请求。
 * @param       size 要写入的数据大小。
 * @retval      无返回值。
 *
 * \attention   在ecrt_sdo_request_state()返回EC_REQUEST_BUSY时，不可调用此方法。
 * \attention   大小必须小于或等于创建请求时指定的大小。
 */
void ecrt_sdo_request_write_with_size(ec_sdo_request_t *req, /**< SDO请求。 */
                                      size_t size            /**< 要写入的数据大小。 */
);

/** 
 * @brief       安排一个SDO读操作。
 * @details     这是一个详细描述。
 * @param       req SDO请求。
 * @retval      无返回值。
 *
 * \attention   在ecrt_sdo_request_state()返回EC_REQUEST_BUSY时，不可调用此方法。
 * \attention   在调用此函数后，ecrt_sdo_request_data()的返回值在ecrt_sdo_request_state()返回EC_REQUEST_BUSY时应被视为无效。
 */
void ecrt_sdo_request_read(ec_sdo_request_t *req /**< SDO请求。 */
);

    /*****************************************************************************
     * FoE request methods.
     ****************************************************************************/

  /**
 * @brief       选择下一个FoE操作要使用的文件名。
 * @details     这是一个简要描述。
 * @param       req FoE请求。
 * @param       file_name 文件名。
 * @param       password 密码。
 * @retval      无返回值。
 */
void ecrt_foe_request_file(ec_foe_request_t *req, /**< FoE请求。 */
                           const char *file_name, /**< 文件名。 */
                           uint32_t password      /**< 密码。 */
);

/**
 * @brief       设置FoE请求的超时时间。
 * @details     这是一个详细描述。
 * @param       req FoE请求。
 * @param       timeout 超时时间（毫秒）。
 *              如果请求在指定的时间内无法处理，将被标记为失败。
 *              超时时间会永久存储在请求对象中，有效直到下一次调用此方法。
 * @retval      无返回值。
 */
void ecrt_foe_request_timeout(ec_foe_request_t *req, /**< FoE请求。 */
                              uint32_t timeout       /**< 超时时间（毫秒）。
                                                       0表示无超时。 */
);

/**
 * @brief       访问FoE请求的数据。
 * @details     这个函数返回指向请求的内部数据内存的指针。
 * @attention   在读操作成功后，可以从此缓冲区读取数据，直到ecrt_foe_request_data_size()。
 *              如果要触发写操作，则必须将数据写入内部内存。
 *              确保数据适合内存。内存大小是ecrt_slave_config_create_foe_request()的一个参数。
 * @return      指向内部文件数据内存的指针。
 */
uint8_t *ecrt_foe_request_data(ec_foe_request_t *req /**< FoE请求。 */
);

/**
 * @brief       返回当前FoE数据的大小。
 * @details     当创建FoE请求时，数据大小设置为保留内存的大小。
 *              在读操作完成后，大小设置为读取数据的大小。
 *              在写操作开始后，大小设置为要写入的数据的大小。
 * @return      FoE数据大小（字节）。
 */
size_t ecrt_foe_request_data_size(const ec_foe_request_t *req /**< FoE请求。 */
);

/**
 * @brief       获取FoE请求的当前状态。
 * @return      请求状态。
 */
#ifdef __KERNEL__
    ec_request_state_t
    ecrt_foe_request_state(const ec_foe_request_t *req /**< FoE请求。 */
    );
#else
ec_request_state_t
ecrt_foe_request_state(ec_foe_request_t *req /**< FoE请求。 */
);
#endif

/**
 * @brief       获取FoE请求的结果。
 * @details     这是一个详细描述。
 * @attention   在ecrt_foe_request_state()返回EC_REQUEST_BUSY时，不可调用此方法。
 * @return      FoE传输结果。
 */
ec_foe_error_t
ecrt_foe_request_result(const ec_foe_request_t *req /**< FoE请求。 */
);

/**
 * @brief       从FoE请求中获取FoE错误码。
 * @details     这个值只在ecrt_foe_request_result()返回FOE_OPCODE_ERROR时有效。
 * @return      FoE错误码。
 *              如果返回值为零，则表示错误是接收到一个意外的操作码；
 *              如果返回值非零，则该值是从FoE ERROR操作码中报告的从站代码。
 */
uint32_t
ecrt_foe_request_error_code(const ec_foe_request_t *req /**< FoE请求。 */
);

/**
 * @brief       返回当前@EC_REQUEST_BUSY传输的进度。
 * @attention   必须在调用ecrt_foe_request_state()之后调用。
 * @return      进度（字节）。
 */
size_t
ecrt_foe_request_progress(const ec_foe_request_t *req /**< FoE请求。 */
);

/**
 * @brief       安排一个FoE写操作。
 * @details     这是一个详细描述。
 * @attention   在ecrt_foe_request_state()返回EC_REQUEST_BUSY时，不可调用此方法。
 * @attention   大小必须小于或等于创建请求时指定的大小。
 * @param       req FoE请求。
 * @param       size 要写入的数据大小。
 * @retval      无返回值。
 */
void ecrt_foe_request_write(ec_foe_request_t *req, /**< FoE请求。 */
                            size_t size            /**< 要写入的数据大小。 */
);

/**
 * @brief       安排一个FoE读操作。
 * @details     这是一个详细描述。
 * @attention   在ecrt_foe_request_state()返回EC_REQUEST_BUSY时，不可调用此方法。
 * @attention   在调用此函数后，ecrt_foe_request_data()的返回值在ecrt_foe_request_state()返回EC_REQUEST_BUSY时应被视为无效。
 * @param       req FoE请求。
 * @retval      无返回值。
 */
void ecrt_foe_request_read(ec_foe_request_t *req /**< FoE请求。 */
);

    /*****************************************************************************
     * VoE handler methods.
     ****************************************************************************/

  /**
 * @brief       设置未来发送操作的VoE头部。
 * @details     一个VoE消息应包含一个4字节的供应商ID，后跟一个2字节的供应商类型作为头部。
 *              这些数字可以通过此函数设置。这些值是有效的，并将用于未来的发送操作，直到下一次调用此方法。
 * @param       voe VoE处理器。
 * @param       vendor_id 供应商ID。
 * @param       vendor_type 供应商特定类型。
 * @retval      无返回值。
 */
void ecrt_voe_handler_send_header(
    ec_voe_handler_t *voe, /**< VoE处理器。 */
    uint32_t vendor_id,    /**< 供应商ID。 */
    uint16_t vendor_type   /**< 供应商特定类型。 */
);

/**
 * @brief       读取接收到的VoE消息的头部数据。
 * @details     此方法可用于在读操作成功后获取接收到的VoE头部信息。
 *              头部信息存储在由指针参数给出的内存中。
 * @param       voe VoE处理器。
 * @param       vendor_id 供应商ID。
 * @param       vendor_type 供应商特定类型。
 * @retval      无返回值。
 */
void ecrt_voe_handler_received_header(
    const ec_voe_handler_t *voe, /**< VoE处理器。 */
    uint32_t *vendor_id,         /**< 供应商ID。 */
    uint16_t *vendor_type        /**< 供应商特定类型。 */
);

/**
 * @brief       访问VoE处理器的数据。
 * @details     此函数返回指向VoE处理器内部内存的指针，该内存指向VoE头部后的实际VoE数据（参见ecrt_voe_handler_send_header()）。
 * @attention   在读操作成功后，内存中包含接收到的数据。接收到的数据的大小可以通过ecrt_voe_handler_data_size()确定。
 *              在触发写操作之前，数据必须写入内部内存。确保数据适合内存。保留内存大小是ecrt_slave_config_create_voe_handler()的一个参数。
 * @return      指向内部内存的指针。
 */
uint8_t *ecrt_voe_handler_data(ec_voe_handler_t *voe /**< VoE处理器。 */
);

/**
 * @brief       返回当前数据的大小。
 * @details     数据大小是VoE数据的大小，不包括头部（参见ecrt_voe_handler_send_header()）。
 *              当创建VoE处理器时，数据大小设置为保留内存的大小。
 *              在写操作时，数据大小设置为要写入的字节数。在读操作后，大小设置为读取数据的大小。
 *              在其他情况下，大小不会被修改。
 * @return      数据大小（字节）。
 */
size_t
ecrt_voe_handler_data_size(const ec_voe_handler_t *voe /**< VoE处理器。 */
);

/**
 * @brief       启动一个VoE写操作。
 * @details     在调用此函数后，必须在每个总线周期中调用ecrt_voe_handler_execute()方法，
 *              只要它返回EC_REQUEST_BUSY。当处理器忙碌时，不可启动其他操作。
 * @param       voe VoE处理器。
 * @param       size 要写入的字节数（不包括VoE头部）。
 * @retval      无返回值。
 */
void ecrt_voe_handler_write(
    ec_voe_handler_t *voe, /**< VoE处理器。 */
    size_t size            /**< 要写入的字节数（不包括VoE头部）。 */
);

/**
 * @brief       启动一个VoE读操作。
 * @details     在调用此函数后，必须在每个总线周期中调用ecrt_voe_handler_execute()方法，
 *              只要它返回EC_REQUEST_BUSY。当处理器忙碌时，不可启动其他操作。
 *              状态机查询从站的发送邮箱以获取要发送到主站的新数据。
 *              如果在EC_VOE_RESPONSE_TIMEOUT（在master/voe_handler.c中定义）内没有数据出现，则操作失败。
 *              成功时，可以通过ecrt_voe_handler_data_size()确定读取数据的大小，
 *              而接收到的数据的VoE头部可以通过ecrt_voe_handler_received_header()检索。
 * @param       voe VoE处理器。
 * @retval      无返回值。
 */
void ecrt_voe_handler_read(ec_voe_handler_t *voe /**< VoE处理器。 */
);

/**
 * @brief       启动一个VoE读操作，而无需查询同步管理器状态。
 * @details     在调用此函数后，必须在每个总线周期中调用ecrt_voe_handler_execute()方法，
 *              只要它返回EC_REQUEST_BUSY。当处理器忙碌时，不可启动其他操作。
 *              状态机通过发送一个空邮箱查询从站。从站将其数据填充到主站的邮箱中。
 *              如果在EC_VOE_RESPONSE_TIMEOUT（在master/voe_handler.c中定义）内没有数据出现，则操作失败。
 *              成功时，可以通过ecrt_voe_handler_data_size()确定读取数据的大小，
 *              而接收到的数据的VoE头部可以通过ecrt_voe_handler_received_header()检索。
 * @param       voe VoE处理器。
 * @retval      无返回值。
 */
void ecrt_voe_handler_read_nosync(ec_voe_handler_t *voe /**< VoE处理器。 */
);

/**
 * @brief       执行处理器。
 * @details     此方法执行VoE处理器。在每个总线周期中必须调用它，只要它返回EC_REQUEST_BUSY。
 * @return      处理器状态。
 */
ec_request_state_t
ecrt_voe_handler_execute(ec_voe_handler_t *voe /**< VoE处理器。 */
);

    /*****************************************************************************
     * Register request methods.
     ****************************************************************************/

    /**
 * @brief       访问寄存器请求的数据。
 * @details     此函数返回指向请求的内部内存的指针。
 * @attention   - 在读操作成功后，可以使用EC_READ_*()宏来评估整数数据，如常规操作。
 *                示例：
 *                \code
 *                uint16_t value = EC_READ_U16(ecrt_reg_request_data(reg_request));
 *                \endcode
 *              - 如果要触发写操作，数据必须写入内部内存。如果要写入整数数据，请使用EC_WRITE_*()宏。
 *                确保数据适合内存。内存大小是ecrt_slave_config_create_reg_request()的一个参数。
 *                \code
 *                EC_WRITE_U16(ecrt_reg_request_data(reg_request), 0xFFFF);
 *                \endcode
 * @return      指向内部内存的指针。
 */
uint8_t *
ecrt_reg_request_data(ec_reg_request_t *req /**< 寄存器请求。 */
);

/** 获取寄存器请求的当前状态。
 *
 * @return      请求状态。
 */
#ifdef __KERNEL__
ec_request_state_t
ecrt_reg_request_state(const ec_reg_request_t *req /**< 寄存器请求。 */
);
#else
ec_request_state_t
ecrt_reg_request_state(ec_reg_request_t *req /**< 寄存器请求。 */
);
#endif

/** 安排一个寄存器写操作。
 *
 * @attention   当ecrt_reg_request_state()返回EC_REQUEST_BUSY时，不可调用此方法。
 *
 * @attention   \a size参数将被截断为请求创建时给定的大小。
 */
void ecrt_reg_request_write(ec_reg_request_t *req, /**< 寄存器请求。 */
                            uint16_t address,      /**< 寄存器地址。 */
                            size_t size            /**< 要写入的大小。 */
);

/** 安排一个寄存器读操作。
 *
 * @attention   当ecrt_reg_request_state()返回EC_REQUEST_BUSY时，不可调用此方法。
 *
 * @attention   \a size参数将被截断为请求创建时给定的大小。
 */
void ecrt_reg_request_read(ec_reg_request_t *req, /**< 寄存器请求。 */
                           uint16_t address,      /**< 寄存器地址。 */
                           size_t size            /**< 要读取的大小。 */
);

/******************************************************************************
 * Bitwise read/write macros
 *****************************************************************************/
/** 读取EtherCAT数据字节的特定位。
 *
 * \param DATA EtherCAT数据指针
 * \param POS 位位置
 */
#define EC_READ_BIT(DATA, POS) ((*((uint8_t *)(DATA)) >> (POS)) & 0x01)

/** 写入EtherCAT数据字节的特定位。
 *
 * \param DATA EtherCAT数据指针
 * \param POS 位位置
 * \param VAL 新的位值
 */
#define EC_WRITE_BIT(DATA, POS, VAL)               \
    do                                             \
    {                                              \
        if (VAL)                                   \
            *((uint8_t *)(DATA)) |= (1 << (POS));  \
        else                                       \
            *((uint8_t *)(DATA)) &= ~(1 << (POS)); \
    } while (0)

   /******************************************************************************
 * 用户空间的字节交换函数
 *****************************************************************************/

#ifndef __KERNEL__

#if __BYTE_ORDER == __LITTLE_ENDIAN

#define le16_to_cpu(x) x
#define le32_to_cpu(x) x
#define le64_to_cpu(x) x

#define cpu_to_le16(x) x
#define cpu_to_le32(x) x
#define cpu_to_le64(x) x

#elif __BYTE_ORDER == __BIG_ENDIAN

#define swap16(x) \
    ((uint16_t)((((uint16_t)(x) & 0x00ffU) << 8) | (((uint16_t)(x) & 0xff00U) >> 8)))
#define swap32(x) \
    ((uint32_t)((((uint32_t)(x) & 0x000000ffUL) << 24) | (((uint32_t)(x) & 0x0000ff00UL) << 8) | (((uint32_t)(x) & 0x00ff0000UL) >> 8) | (((uint32_t)(x) & 0xff000000UL) >> 24)))
#define swap64(x) \
    ((uint64_t)((((uint64_t)(x) & 0x00000000000000ffULL) << 56) | (((uint64_t)(x) & 0x000000000000ff00ULL) << 40) | (((uint64_t)(x) & 0x0000000000ff0000ULL) << 24) | (((uint64_t)(x) & 0x00000000ff000000ULL) << 8) | (((uint64_t)(x) & 0x000000ff00000000ULL) >> 8) | (((uint64_t)(x) & 0x0000ff0000000000ULL) >> 24) | (((uint64_t)(x) & 0x00ff000000000000ULL) >> 40) | (((uint64_t)(x) & 0xff00000000000000ULL) >> 56)))

#define le16_to_cpu(x) swap16(x)
#define le32_to_cpu(x) swap32(x)
#define le64_to_cpu(x) swap64(x)

#define cpu_to_le16(x) swap16(x)
#define cpu_to_le32(x) swap32(x)
#define cpu_to_le64(x) swap64(x)

#endif

#define le16_to_cpup(x) le16_to_cpu(*((uint16_t *)(x)))
#define le32_to_cpup(x) le32_to_cpu(*((uint32_t *)(x)))
#define le64_to_cpup(x) le64_to_cpu(*((uint64_t *)(x)))

#endif /* ifndef __KERNEL__ */

/******************************************************************************
 * 读取宏
 *****************************************************************************/

/** 从EtherCAT数据中读取一个8位无符号值。
 *
 * \return EtherCAT数据值
 */
#define EC_READ_U8(DATA) ((uint8_t) * ((uint8_t *)(DATA)))

/** 从EtherCAT数据中读取一个8位有符号值。
 *
 * \param DATA EtherCAT数据指针
 * \return EtherCAT数据值
 */
#define EC_READ_S8(DATA) ((int8_t) * ((uint8_t *)(DATA)))

/** 从EtherCAT数据中读取一个16位无符号值。
 *
 * \param DATA EtherCAT数据指针
 * \return EtherCAT数据值
 */
#define EC_READ_U16(DATA) ((uint16_t)le16_to_cpup((void *)(DATA)))

/** 从EtherCAT数据中读取一个16位有符号值。
 *
 * \param DATA EtherCAT数据指针
 * \return EtherCAT数据值
 */
#define EC_READ_S16(DATA) ((int16_t)le16_to_cpup((void *)(DATA)))

/** 从EtherCAT数据中读取一个32位无符号值。
 *
 * \param DATA EtherCAT数据指针
 * \return EtherCAT数据值
 */
#define EC_READ_U32(DATA) ((uint32_t)le32_to_cpup((void *)(DATA)))

/** 从EtherCAT数据中读取一个32位有符号值。
 *
 * \param DATA EtherCAT数据指针
 * \return EtherCAT数据值
 */
#define EC_READ_S32(DATA) ((int32_t)le32_to_cpup((void *)(DATA)))

/** 从EtherCAT数据中读取一个64位无符号值。
 *
 * \param DATA EtherCAT数据指针
 * \return EtherCAT数据值
 */
#define EC_READ_U64(DATA) ((uint64_t)le64_to_cpup((void *)(DATA)))

/** 从EtherCAT数据中读取一个64位有符号值。
 *
 * \param DATA EtherCAT数据指针
 * \return EtherCAT数据值
 */
#define EC_READ_S64(DATA) ((int64_t)le64_to_cpup((void *)(DATA)))
/******************************************************************************
 * 浮点数读取函数和宏（仅限用户空间）
 *****************************************************************************/

#ifndef __KERNEL__

/** 从EtherCAT数据中读取32位浮点数值。
 *
 * \param data EtherCAT数据指针
 * \return EtherCAT数据值
 */
float ecrt_read_real(const void *data);

/** 从EtherCAT数据中读取32位浮点数值。
 *
 * \param DATA EtherCAT数据指针
 * \return EtherCAT数据值
 */
#define EC_READ_REAL(DATA) ecrt_read_real(DATA)

/** 从EtherCAT数据中读取64位浮点数值。
 *
 * \param data EtherCAT数据指针
 * \return EtherCAT数据值
 */
double ecrt_read_lreal(const void *data);

/** 从EtherCAT数据中读取64位浮点数值。
 *
 * \param DATA EtherCAT数据指针
 * \return EtherCAT数据值
 */
#define EC_READ_LREAL(DATA) ecrt_read_lreal(DATA)

#endif // ifndef __KERNEL__


/******************************************************************************
 * 写入宏
 *****************************************************************************/

/** 将一个8位无符号值写入EtherCAT数据。
 *
 * \param DATA EtherCAT数据指针
 * \param VAL 新值
 */
#define EC_WRITE_U8(DATA, VAL)                   \
    do                                           \
    {                                            \
        *((uint8_t *)(DATA)) = ((uint8_t)(VAL)); \
    } while (0)

/** 将一个8位有符号值写入EtherCAT数据。
 *
 * \param DATA EtherCAT数据指针
 * \param VAL 新值
 */
#define EC_WRITE_S8(DATA, VAL) EC_WRITE_U8(DATA, VAL)

/** 将一个16位无符号值写入EtherCAT数据。
 *
 * \param DATA EtherCAT数据指针
 * \param VAL 新值
 */
#define EC_WRITE_U16(DATA, VAL)                               \
    do                                                        \
    {                                                         \
        *((uint16_t *)(DATA)) = cpu_to_le16((uint16_t)(VAL)); \
    } while (0)

/** 将一个16位有符号值写入EtherCAT数据。
 *
 * \param DATA EtherCAT数据指针
 * \param VAL 新值
 */
#define EC_WRITE_S16(DATA, VAL) EC_WRITE_U16(DATA, VAL)

/** 将一个32位无符号值写入EtherCAT数据。
 *
 * \param DATA EtherCAT数据指针
 * \param VAL 新值
 */
#define EC_WRITE_U32(DATA, VAL)                               \
    do                                                        \
    {                                                         \
        *((uint32_t *)(DATA)) = cpu_to_le32((uint32_t)(VAL)); \
    } while (0)

/** 将一个32位有符号值写入EtherCAT数据。
 *
 * \param DATA EtherCAT数据指针
 * \param VAL 新值
 */
#define EC_WRITE_S32(DATA, VAL) EC_WRITE_U32(DATA, VAL)

/** 将一个64位无符号值写入EtherCAT数据。
 *
 * \param DATA EtherCAT数据指针
 * \param VAL 新值
 */
#define EC_WRITE_U64(DATA, VAL)                               \
    do                                                        \
    {                                                         \
        *((uint64_t *)(DATA)) = cpu_to_le64((uint64_t)(VAL)); \
    } while (0)

/** 将一个64位有符号值写入EtherCAT数据。
 *
 * \param DATA EtherCAT数据指针
 * \param VAL 新值
 */
#define EC_WRITE_S64(DATA, VAL) EC_WRITE_U64(DATA, VAL)


  /******************************************************************************
 * 浮点数写入函数和宏（仅限用户空间）
 *****************************************************************************/

#ifndef __KERNEL__

/** 将一个32位浮点数值写入EtherCAT数据。
 *
 * \param data EtherCAT数据指针
 * \param value 新值
 */
void ecrt_write_real(void *data, float value);

/** 将一个32位浮点数值写入EtherCAT数据。
 *
 * \param DATA EtherCAT数据指针
 * \param VAL 新值
 */
#define EC_WRITE_REAL(DATA, VAL) ecrt_write_real(DATA, VAL)

/** 将一个64位浮点数值写入EtherCAT数据。
 *
 * \param data EtherCAT数据指针
 * \param value 新值
 */
void ecrt_write_lreal(void *data, double value);

/** 将一个64位浮点数值写入EtherCAT数据。
 *
 * \param DATA EtherCAT数据指针
 * \param VAL 新值
 */
#define EC_WRITE_LREAL(DATA, VAL) ecrt_write_lreal(DATA, VAL)

#endif // ifndef __KERNEL__

/** 调度一个寄存器读写操作。
 *
 * \attention 在ecrt_reg_request_state()返回EC_REQUEST_BUSY时，不能调用此方法。
 *
 * \attention \a size参数会被截断为请求创建时给定的大小。
 */
void
ecrt_reg_request_readwrite(ec_reg_request_t *req, /**< 寄存器请求。 */
                           uint16_t address,      /**< 寄存器地址。 */
                           size_t size            /**< 读写大小。 */
);


    /*****************************************************************************/

#ifdef __cplusplus
}
#endif

/*****************************************************************************/

/** @} */

#endif

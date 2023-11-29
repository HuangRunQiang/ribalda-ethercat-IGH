/******************************************************************************
 *
 *  $Id$
 *
 *  版权所有 (C) 2006-2012 Florian Pose, Ingenieurgemeinschaft IgH
 *
 *  本文件是IgH EtherCAT主站的一部分。
 *
 *  IgH EtherCAT主站是自由软件；您可以在自由软件基金会发布的GNU通用公共许可证第2版下重新分发和修改它。
 *
 *  IgH EtherCAT主站希望它是有用的，但不提供任何担保，也不包含任何隐含的担保，
 *  包括适销性或特定用途的适用性。有关更多详细信息，请参阅GNU通用公共许可证。
 *
 *  您应该随着IgH EtherCAT主站一起收到GNU通用公共许可证的副本；如果没有，
 *  请写信给Free Software Foundation, Inc.，51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA。
 *
 *  ---
 *
 *  上述许可证仅涉及源代码。在遵守Beckhoff Automation GmbH的工业产权和类似权利的前提下，
 *  才允许使用EtherCAT技术和品牌。
 *
 *****************************************************************************/

/**
   \file
   EtherCAT主站结构体。
*/

/*****************************************************************************/

#ifndef __EC_MASTER_H__
#define __EC_MASTER_H__

#include <linux/version.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/wait.h>
#include <linux/kthread.h>

#include "device.h"
#include "domain.h"
#include "ethernet.h"
#include "fsm_master.h"
#include "locks.h"
#include "cdev.h"

#ifdef EC_RTDM
#include "rtdm.h"
#endif

/*****************************************************************************/

/** 方便的宏，用于将主站特定的信息打印到syslog。
 *
 * 这将使用带有前缀“EtherCAT <INDEX>：”的格式打印消息，其中INDEX是主站的索引。
 *
 * \param master EtherCAT主站
 * \param fmt 格式字符串（类似于printf（）中的格式）
 * \param args 参数（可选）
 */
#define EC_MASTER_INFO(master, fmt, args...) \
    printk(KERN_INFO "EtherCAT %u: " fmt, master->index, ##args)

/** 方便的宏，用于将主站特定的错误打印到syslog。
 *
 * 这将使用带有前缀“EtherCAT <INDEX>：”的格式打印消息，其中INDEX是主站的索引。
 *
 * \param master EtherCAT主站
 * \param fmt 格式字符串（类似于printf（）中的格式）
 * \param args 参数（可选）
 */
#define EC_MASTER_ERR(master, fmt, args...) \
    printk(KERN_ERR "EtherCAT ERROR %u: " fmt, master->index, ##args)

/** 方便的宏，用于将主站特定的警告打印到syslog。
 *
 * 这将使用带有前缀“EtherCAT <INDEX>：”的格式打印消息，其中INDEX是主站的索引。
 *
 * \param master EtherCAT主站
 * \param fmt 格式字符串（类似于printf（）中的格式）
 * \param args 参数（可选）
 */
#define EC_MASTER_WARN(master, fmt, args...) \
    printk(KERN_WARNING "EtherCAT WARNING %u: " fmt, master->index, ##args)

/** 方便的宏，用于将主站特定的调试消息打印到syslog。
 *
 * 这将使用带有前缀“EtherCAT <INDEX>：”的格式打印消息，其中INDEX是主站的索引。
 *
 * \param master EtherCAT主站
 * \param level 调试级别。主站的调试级别必须>= level才能输出。
 * \param fmt 格式字符串（类似于printf（）中的格式）
 * \param args 参数（可选）
 */
#define EC_MASTER_DBG(master, level, fmt, args...)       \
    do                                                   \
    {                                                    \
        if (master->debug_level >= level)                \
        {                                                \
            printk(KERN_DEBUG "EtherCAT DEBUG %u: " fmt, \
                   master->index, ##args);               \
        }                                                \
    } while (0)

/** 外部数据报文环的大小。
 *
 * 外部数据报文环用于从站FSM。
 */
#define EC_EXT_RING_SIZE 32

/** 从ecrt_master_eoe_process()返回的标志，表示有待发送的内容。
 * 如果设置了此标志，请调用ecrt_master_send_ext()。
 */
#define EOE_STH_TO_SEND 1

/** 从ecrt_master_eoe_process()返回的标志，表示仍有待处理的内容。
 * 如果设置了此标志，请在重新开始循环之前快速进行一次循环，否则休眠一段时间（例如1ms）。
 */

#define EOE_STH_PENDING 2

/*****************************************************************************/

/** EtherCAT主站阶段。
 */
typedef enum
{
    EC_ORPHANED, /**< 孤立阶段。主站没有连接的以太网设备。 */
    EC_IDLE,     /**< 空闲阶段。已连接以太网设备，但主站尚未使用。 */
    EC_OPERATION /**< 运行阶段。实时应用程序请求了主站。 */
} ec_master_phase_t;

/*****************************************************************************/

/** 循环统计信息。
 */
typedef struct
{
    unsigned int timeouts;        /**< 数据报文超时次数 */
    unsigned int corrupted;       /**< 损坏的帧数 */
    unsigned int unmatched;       /**< 未匹配的数据报文（已接收但不再排队） */
    unsigned long output_jiffies; /**< 上次输出的时间 */
} ec_stats_t;

/*****************************************************************************/

/** 设备统计信息。
 */
typedef struct
{
    u64 tx_count;                      /**< 发送的帧数。 */
    u64 last_tx_count;                 /**< 上一个统计周期中发送的帧数。 */
    u64 rx_count;                      /**< 接收的帧数。 */
    u64 last_rx_count;                 /**< 上一个统计周期中接收的帧数。 */
    u64 tx_bytes;                      /**< 发送的字节数。 */
    u64 last_tx_bytes;                 /**< 上一个统计周期中发送的字节数。 */
    u64 rx_bytes;                      /**< 接收的字节数。 */
    u64 last_rx_bytes;                 /**< 上一个统计周期中接收的字节数。 */
    u64 last_loss;                     /**< 上一个统计周期中的发送/接收差异。 */
    s32 tx_frame_rates[EC_RATE_COUNT]; /**< 不同统计周期内的帧速率（以帧/秒为单位）。 */
    s32 rx_frame_rates[EC_RATE_COUNT]; /**< 不同统计周期内的帧速率（以帧/秒为单位）。 */
    s32 tx_byte_rates[EC_RATE_COUNT];  /**< 不同统计周期内的字节速率（以字节/秒为单位）。 */
    s32 rx_byte_rates[EC_RATE_COUNT];  /**< 不同统计周期内的字节速率（以字节/秒为单位）。 */
    s32 loss_rates[EC_RATE_COUNT];     /**< 不同统计周期内的帧丢失率。 */
    unsigned long jiffies;             /**< 上一个统计周期的jiffies。 */
} ec_device_stats_t;

/*****************************************************************************/

#if EC_MAX_NUM_DEVICES < 1
#error Invalid number of devices
#endif

/*****************************************************************************/

/** EtherCAT主站。
 *
 * 管理从站、域和IO。
 */
struct ec_master
{
    unsigned int index;    /**< 索引。 */
    unsigned int reserved; /**< \a True，如果主站正在使用中。 */

    ec_cdev_t cdev; /**< 主站字符设备。 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 26)
    struct device *class_device; /**< 主站类设备。 */
#else
    struct class_device *class_device; /**< 主站类设备。 */
#endif

#ifdef EC_RTDM
    ec_rtdm_dev_t rtdm_dev; /**< RTDM设备。 */
#endif

    ec_lock_t master_sem; /**< 主站信号量。 */

    ec_device_t devices[EC_MAX_NUM_DEVICES]; /**< EtherCAT设备。 */
    const uint8_t *macs[EC_MAX_NUM_DEVICES]; /**< 设备MAC地址。 */
#if EC_MAX_NUM_DEVICES > 1
    unsigned int num_devices; /**< 设备数量。始终通过ec_master_num_devices()访问，因为它可能被优化！ */
#endif
    ec_lock_t device_sem;           /**< 设备信号量。 */
    ec_device_stats_t device_stats; /**< 设备统计信息。 */

    ec_fsm_master_t fsm;            /**< 主站状态机。 */
    ec_datagram_t fsm_datagram;     /**< 用于状态机的数据报文。 */
    ec_master_phase_t phase;        /**< 主站阶段。 */
    unsigned int active;            /**< 主站已激活。 */
    unsigned int config_changed;    /**< 配置已更改。 */
    unsigned int injection_seq_fsm; /**< FSM端的数据报文注入序列号。 */
    unsigned int injection_seq_rt;  /**< 实时端的数据报文注入序列号。 */

    ec_slave_t *slaves;       /**< 总线上的从站数组。 */
    unsigned int slave_count; /**< 总线上的从站数量。 */

    /* 应用程序应用的配置。 */
    struct list_head configs; /**< 从站配置列表。 */
    struct list_head domains; /**< 域列表。 */

    /* 在总线扫描期间应用的配置。 */
    struct list_head sii_images; /**< 从站SII映像列表。 */

    u64 app_time;                     /**< 上次ecrt_master_sync()调用的时间。 */
    u64 dc_ref_time;                  /**< DC启动时间的公共参考时间戳。 */
    u8 dc_offset_valid;               /**< DC从站具有有效的系统时间偏移量 */
    ec_datagram_t ref_sync_datagram;  /**< 用于将参考时钟与主时钟同步的数据报文。 */
    ec_datagram_t sync_datagram;      /**< 用于DC漂移补偿的数据报文。 */
    ec_datagram_t sync64_datagram;    /**< 用于检索64位参考从站系统时钟时间的数据报文。 */
    ec_datagram_t sync_mon_datagram;  /**< 用于DC同步监控的数据报文。 */
    ec_slave_config_t *dc_ref_config; /**< 应用程序选择的DC参考时钟从站配置。 */
    ec_slave_t *dc_ref_clock;         /**< DC参考时钟从站。 */

    unsigned int reboot;          /**< 请求重启。 */
    unsigned int scan_busy;       /**< 当前扫描状态。 */
    unsigned int allow_scan;      /**< \a True，如果允许从站扫描。 */
    ec_lock_t scan_sem;           /**< 保护\a scan_busy变量和\a allow_scan标志的信号量。 */
    wait_queue_head_t scan_queue; /**< 等待从站扫描的进程队列。 */

    unsigned int config_busy;       /**< 从站配置状态。 */
    ec_lock_t config_sem;           /**< 保护\a config_busy变量和\a allow_config标志的信号量。 */
    wait_queue_head_t config_queue; /**< 等待从站配置的进程队列。 */

    struct list_head datagram_queue; /**< 数据报文队列。 */
    uint8_t datagram_index;          /**< 当前数据报文索引。 */

    struct list_head ext_datagram_queue; /**< 非应用程序数据报文队列。 */
    ec_lock_t ext_queue_sem;             /**< 保护\a ext_datagram_queue的信号量。 */

    ec_datagram_t ext_datagram_ring[EC_EXT_RING_SIZE]; /**< 外部数据报文环。 */
    unsigned int ext_ring_idx_rt;                      /**< 实时端的外部数据报文环索引。 */
    unsigned int ext_ring_idx_fsm;                     /**< FSM端的外部数据报文环索引。 */
    unsigned int send_interval;                        /**< 两次调用ecrt_master_send()之间的间隔。 */
    size_t max_queue_size;                             /**< 数据报文队列的最大大小 */
    unsigned int rt_slave_requests;                    /**< 如果\a True，则从站请求将由应用程序的实时上下文中的ecrt_master_exec_requests()调用处理。 */
    unsigned int rt_slaves_available;                  /**< 如果\a True，则从站请求可以由应用程序的实时上下文中的ecrt_master_exec_requests()调用处理。
                                                         否则，主站当前正在配置从站 */
    ec_slave_t *fsm_slave;                             /**< 下一个用于FSM执行的从站。 */
    struct list_head fsm_exec_list;                    /**< 从站FSM执行列表。 */
    unsigned int fsm_exec_count;                       /**< 执行列表中的条目数。 */

    unsigned int debug_level; /**< 主站调试级别。 */
    ec_stats_t stats;         /**< 循环统计信息。 */

    void *pcap_data;      /**< pcap调试输出内存指针 */
    void *pcap_curr_data; /**< pcap调试输出当前内存指针 */

    struct task_struct *thread; /**< 主站线程。 */

#ifdef EC_EOE
    struct task_struct *eoe_thread; /**< EoE线程。 */
    struct list_head eoe_handlers;  /**< Ethernet over EtherCAT处理程序。 */
#endif

    ec_lock_t io_sem; /**< 在\a IDLE阶段使用的信号量。 */

    void (*send_cb)(void *);        /**< 当前发送数据报文的回调函数。 */
    void (*receive_cb)(void *);     /**< 当前接收数据报文的回调函数。 */
    void *cb_data;                  /**< 当前回调函数的数据。 */
    void (*app_send_cb)(void *);    /**< 应用程序的发送数据报文回调函数。 */
    void (*app_receive_cb)(void *); /**< 应用程序的接收数据报文回调函数。 */
    void *app_cb_data;              /**< 应用程序回调函数的数据。 */

    struct list_head sii_requests;       /**< SII写入请求列表。 */
    struct list_head emerg_reg_requests; /**< 紧急寄存器访问请求列表。 */

    wait_queue_head_t request_queue; /**< 用户空间外部请求的等待队列。 */
};

/*****************************************************************************/

// 静态函数
void ec_master_init_static(void);

// 主站的创建/删除
int ec_master_init(ec_master_t *, unsigned int, const uint8_t *,
                   const uint8_t *, dev_t, struct class *, unsigned int);
void ec_master_clear(ec_master_t *);

void ec_sii_image_clear(ec_sii_image_t *);

/** 以太网设备数量。
 */
#if EC_MAX_NUM_DEVICES > 1
#define ec_master_num_devices(MASTER) ((MASTER)->num_devices)
#else
#define ec_master_num_devices(MASTER) 1
#endif

// 阶段转换
int ec_master_enter_idle_phase(ec_master_t *);
void ec_master_leave_idle_phase(ec_master_t *);
int ec_master_enter_operation_phase(ec_master_t *);
void ec_master_leave_operation_phase(ec_master_t *);

#ifdef EC_EOE
// EoE
void ec_master_eoe_start(ec_master_t *);
void ec_master_eoe_stop(ec_master_t *);
#endif

// 数据报文IO
void ec_master_receive_datagrams(ec_master_t *, ec_device_t *,
                                 const uint8_t *, size_t);
void ec_master_queue_datagram(ec_master_t *, ec_datagram_t *);
void ec_master_queue_datagram_ext(ec_master_t *, ec_datagram_t *);

// 其他
void ec_master_set_send_interval(ec_master_t *, unsigned int);
void ec_master_attach_slave_configs(ec_master_t *);
void ec_master_expire_slave_config_requests(ec_master_t *);
ec_slave_t *ec_master_find_slave(ec_master_t *, uint16_t, uint16_t);
const ec_slave_t *ec_master_find_slave_const(const ec_master_t *, uint16_t,
                                             uint16_t);
void ec_master_output_stats(ec_master_t *);
#ifdef EC_EOE
void ec_master_clear_eoe_handlers(ec_master_t *, unsigned int);
#endif
void ec_master_slaves_not_available(ec_master_t *);
void ec_master_slaves_available(ec_master_t *);
void ec_master_clear_slaves(ec_master_t *);
void ec_master_clear_sii_images(ec_master_t *);
void ec_master_reboot_slaves(ec_master_t *);

unsigned int ec_master_config_count(const ec_master_t *);
ec_slave_config_t *ec_master_get_config(
    const ec_master_t *, unsigned int);
const ec_slave_config_t *ec_master_get_config_const(
    const ec_master_t *, unsigned int);
unsigned int ec_master_domain_count(const ec_master_t *);
ec_domain_t *ec_master_find_domain(ec_master_t *, unsigned int);
const ec_domain_t *ec_master_find_domain_const(const ec_master_t *,
                                               unsigned int);
#ifdef EC_EOE
uint16_t ec_master_eoe_handler_count(const ec_master_t *);
const ec_eoe_t *ec_master_get_eoe_handler_const(const ec_master_t *, uint16_t);
#ifdef EC_RTDM
int ec_master_eoe_is_open(ec_master_t *);
int ec_master_eoe_process(ec_master_t *);
#endif
#endif

int ec_master_mbox_gateway(ec_master_t *master, uint8_t *data,
                           size_t *data_size, size_t buff_size);

int ec_master_debug_level(ec_master_t *, unsigned int);

ec_domain_t *ecrt_master_create_domain_err(ec_master_t *);
ec_slave_config_t *ecrt_master_slave_config_err(ec_master_t *, uint16_t,
                                                uint16_t, uint32_t, uint32_t);

void ec_master_calc_dc(ec_master_t *);
void ec_master_request_op(ec_master_t *);

void ec_master_internal_send_cb(void *);
void ec_master_internal_receive_cb(void *);

int ec_master_dict_upload(ec_master_t *, uint16_t);

extern const unsigned int rate_intervals[EC_RATE_COUNT]; // 见master.c

#ifdef EC_EOE
#define MAX_EOE 32                    /**< 可以在启动时定义的EOE接口的最大数量。 */
extern char *eoe_interfaces[MAX_EOE]; // 见module.c
extern unsigned int eoe_count;        // 见module.c
extern bool eoe_autocreate;           // 见module.c
#endif
extern unsigned long pcap_size; // 见module.c

/*****************************************************************************/

#endif

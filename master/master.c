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
 *  vim: expandtab
 *
 *****************************************************************************/

/**
   \file
   EtherCAT master methods.
*/

/*****************************************************************************/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/version.h>
#include <linux/hrtimer.h>
#include <linux/vmalloc.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 11, 0)
#include <linux/sched/types.h>  // struct sched_param
#include <linux/sched/signal.h> // signal_pending
#endif

#include "globals.h"
#include "slave.h"
#include "slave_config.h"
#include "device.h"
#include "datagram.h"
#include "mailbox.h"
#ifdef EC_EOE
#include "ethernet.h"
#endif
#include "master.h"
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 11, 0)
#include <linux/sched/signal.h>
#include <uapi/linux/sched/types.h>
#endif

/*****************************************************************************/

/** 将此值设置为1以启用外部数据报注入调试。
 */
#define DEBUG_INJECT 0

/** 总是输出损坏的帧。
 */
#define FORCE_OUTPUT_CORRUPTED 0

#ifdef EC_HAVE_CYCLES

/** 帧超时时间（以cycles为单位）。
 */
static cycles_t timeout_cycles;

/** 外部数据报注入超时时间（以cycles为单位）。
 */
static cycles_t ext_injection_timeout_cycles;

#else

/** 帧超时时间（以jiffies为单位）。
 */
static unsigned long timeout_jiffies;

/** 外部数据报注入超时时间（以jiffies为单位）。
 */
static unsigned long ext_injection_timeout_jiffies;

#endif

/** 统计信息的时间间隔列表（以秒为单位）。
 */
const unsigned int rate_intervals[] = {
    1, 10, 60};

/*****************************************************************************/

/** 清除主站的从站配置。
 */
void ec_master_clear_slave_configs(ec_master_t *);

/** 清除主站的域。
 */
void ec_master_clear_domains(ec_master_t *);

/** 主站的空闲线程。
 */
static int ec_master_idle_thread(void *);

/** 主站的操作线程。
 */
static int ec_master_operation_thread(void *);

#ifdef EC_EOE
/** 主站的EOE线程。
 */
static int ec_master_eoe_thread(void *);
#endif

/** 查找主站的DC参考时钟。
 */
void ec_master_find_dc_ref_clock(ec_master_t *);

/** 清除主站的设备统计信息。
 */
void ec_master_clear_device_stats(ec_master_t *);

/** 更新主站的设备统计信息。
 */
void ec_master_update_device_stats(ec_master_t *);


/*****************************************************************************/

/**
 * @brief	静态变量初始化函数
 * @作用	初始化静态变量
 */
void ec_master_init_static(void)
{
#ifdef EC_HAVE_CYCLES
    timeout_cycles = (cycles_t)EC_IO_TIMEOUT /* us */ * (cpu_khz / 1000);
    ext_injection_timeout_cycles =
        (cycles_t)EC_SDO_INJECTION_TIMEOUT /* us */ * (cpu_khz / 1000);
#else
    // 两次时间测量之间可能会经过一个jiffy
    timeout_jiffies = max(EC_IO_TIMEOUT * HZ / 1000000, 1);
    ext_injection_timeout_jiffies =
        max(EC_SDO_INJECTION_TIMEOUT * HZ / 1000000, 1);
#endif
}


/*****************************************************************************/

/**
 * @brief	主站初始化函数
 * @作用	初始化EtherCAT主站
 * @param	master			EtherCAT主站对象
 * @param	index			主站索引
 * @param	main_mac		主设备的MAC地址
 * @param	backup_mac		备份设备的MAC地址
 * @param	device_number	字符设备编号
 * @param	class			设备类
 * @param	debug_level		调试级别（模块参数）
 * @retval	0：成功，< 0：失败
 */
int ec_master_init(ec_master_t *master,
                   unsigned int index,
                   const uint8_t *main_mac,
                   const uint8_t *backup_mac,
                   dev_t device_number,
                   struct class *class,
                   unsigned int debug_level)
{
    int ret;
    unsigned int dev_idx, i;

    master->index = index;
    master->reserved = 0;

    ec_lock_init(&master->master_sem);

    for (dev_idx = EC_DEVICE_MAIN; dev_idx < EC_MAX_NUM_DEVICES; dev_idx++)
    {
        master->macs[dev_idx] = NULL;
    }

    master->macs[EC_DEVICE_MAIN] = main_mac;

#if EC_MAX_NUM_DEVICES > 1
    master->macs[EC_DEVICE_BACKUP] = backup_mac;
    master->num_devices = 1 + !ec_mac_is_zero(backup_mac);
#else
    if (!ec_mac_is_zero(backup_mac))
    {
        EC_MASTER_WARN(master, "Ignoring backup MAC address!");
    }
#endif

    ec_master_clear_device_stats(master);

    ec_lock_init(&master->device_sem);

    master->phase = EC_ORPHANED;
    master->active = 0;
    master->config_changed = 0;
    master->injection_seq_fsm = 0;
    master->injection_seq_rt = 0;
    master->reboot = 0;

    master->slaves = NULL;
    master->slave_count = 0;

    INIT_LIST_HEAD(&master->configs);
    INIT_LIST_HEAD(&master->domains);
    INIT_LIST_HEAD(&master->sii_images);

    master->app_time = 0ULL;
    master->dc_ref_time = 0ULL;
    master->dc_offset_valid = 0;

    master->scan_busy = 0;
    master->allow_scan = 1;
    ec_lock_init(&master->scan_sem);
    init_waitqueue_head(&master->scan_queue);

    master->config_busy = 0;
    ec_lock_init(&master->config_sem);
    init_waitqueue_head(&master->config_queue);

    INIT_LIST_HEAD(&master->datagram_queue);
    master->datagram_index = 0;

    INIT_LIST_HEAD(&master->ext_datagram_queue);
    ec_lock_init(&master->ext_queue_sem);

    master->ext_ring_idx_rt = 0;
    master->ext_ring_idx_fsm = 0;
    master->rt_slave_requests = 0;
    master->rt_slaves_available = 0;

    // 初始化外部数据报环
    for (i = 0; i < EC_EXT_RING_SIZE; i++)
    {
        ec_datagram_t *datagram = &master->ext_datagram_ring[i];
        ec_datagram_init(datagram);
        snprintf(datagram->name, EC_DATAGRAM_NAME_SIZE, "ext-%u", i);
    }

    // 设置IDLE阶段的发送间隔
    ec_master_set_send_interval(master, 1000000 / HZ);

    master->fsm_slave = NULL;
    INIT_LIST_HEAD(&master->fsm_exec_list);
    master->fsm_exec_count = 0U;

    master->debug_level = debug_level;
    master->stats.timeouts = 0;
    master->stats.corrupted = 0;
    master->stats.unmatched = 0;
    master->stats.output_jiffies = 0;

    // 设置pcap调试
    if (pcap_size > 0)
    {
        master->pcap_data = vmalloc(pcap_size);
    }
    else
    {
        master->pcap_data = NULL;
    }
    master->pcap_curr_data = master->pcap_data;

    master->thread = NULL;

#ifdef EC_EOE
    master->eoe_thread = NULL;
    INIT_LIST_HEAD(&master->eoe_handlers);
#endif

    ec_lock_init(&master->io_sem);
    master->send_cb = NULL;
    master->receive_cb = NULL;
    master->cb_data = NULL;
    master->app_send_cb = NULL;
    master->app_receive_cb = NULL;
    master->app_cb_data = NULL;

    INIT_LIST_HEAD(&master->sii_requests);
    INIT_LIST_HEAD(&master->emerg_reg_requests);

    init_waitqueue_head(&master->request_queue);

    // 初始化设备
    for (dev_idx = EC_DEVICE_MAIN; dev_idx < ec_master_num_devices(master);
         dev_idx++)
    {
        ret = ec_device_init(&master->devices[dev_idx], master);
        if (ret < 0)
        {
            goto out_clear_devices;
        }
    }

    // 初始化状态机数据报
    ec_datagram_init(&master->fsm_datagram);
    snprintf(master->fsm_datagram.name, EC_DATAGRAM_NAME_SIZE, "master-fsm");
    ret = ec_datagram_prealloc(&master->fsm_datagram, EC_MAX_DATA_SIZE);
    if (ret < 0)
    {
        ec_datagram_clear(&master->fsm_datagram);
        EC_MASTER_ERR(master, "Failed to allocate FSM datagram.\n");
        goto out_clear_devices;
    }

    // 创建状态机对象
    ec_fsm_master_init(&master->fsm, master, &master->fsm_datagram);

    // 分配外部数据报环
    for (i = 0; i < EC_EXT_RING_SIZE; i++)
    {
        ec_datagram_t *datagram = &master->ext_datagram_ring[i];
        ret = ec_datagram_prealloc(datagram, EC_MAX_DATA_SIZE);
        if (ret)
        {
            EC_MASTER_ERR(master, "Failed to allocate external"
                                  " datagram %u.\n",
                          i);
            goto out_clear_ext_datagrams;
        }
    }

    // 初始化参考同步数据报
    ec_datagram_init(&master->ref_sync_datagram);
    snprintf(master->ref_sync_datagram.name, EC_DATAGRAM_NAME_SIZE,
             "refsync");
    ret = ec_datagram_prealloc(&master->ref_sync_datagram, 4);
    if (ret < 0)
    {
        ec_datagram_clear(&master->ref_sync_datagram);
        EC_MASTER_ERR(master, "Failed to allocate reference"
                              " synchronisation datagram.\n");
        goto out_clear_ext_datagrams;
    }

    // 初始化同步数据报
    ec_datagram_init(&master->sync_datagram);
    snprintf(master->sync_datagram.name, EC_DATAGRAM_NAME_SIZE, "sync");
    ret = ec_datagram_prealloc(&master->sync_datagram, 4);
    if (ret < 0)
    {
        ec_datagram_clear(&master->sync_datagram);
        EC_MASTER_ERR(master, "Failed to allocate"
                              " synchronisation datagram.\n");
        goto out_clear_ref_sync;
    }

    // 初始化64位同步数据报
    ec_datagram_init(&master->sync64_datagram);
    snprintf(master->sync64_datagram.name, EC_DATAGRAM_NAME_SIZE, "sync64");
    ret = ec_datagram_prealloc(&master->sync64_datagram, 8);
    if (ret < 0)
    {
        ec_datagram_clear(&master->sync_datagram);
        EC_MASTER_ERR(master, "Failed to allocate 64bit ref slave"
                              " system clock datagram.\n");
        goto out_clear_sync;
    }

    // 初始化同步监控数据报
    ec_datagram_init(&master->sync_mon_datagram);
    snprintf(master->sync_mon_datagram.name, EC_DATAGRAM_NAME_SIZE,
             "syncmon");
    ret = ec_datagram_brd(&master->sync_mon_datagram, 0x092c, 4);
    if (ret < 0)
    {
        ec_datagram_clear(&master->sync_mon_datagram);
        EC_MASTER_ERR(master, "Failed to allocate sync"
                              " monitoring datagram.\n");
        goto out_clear_sync64;
    }

    master->dc_ref_config = NULL;
    master->dc_ref_clock = NULL;

    // 初始化字符设备
    ret = ec_cdev_init(&master->cdev, master, device_number);
    if (ret)
        goto out_clear_sync_mon;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)
    master->class_device = device_create(class, NULL,
                                         MKDEV(MAJOR(device_number), master->index), NULL,
                                         "EtherCAT%u", master->index);
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 26)
    master->class_device = device_create(class, NULL,
                                         MKDEV(MAJOR(device_number), master->index),
                                         "EtherCAT%u", master->index);
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 15)
    master->class_device = class_device_create(class, NULL,
                                               MKDEV(MAJOR(device_number), master->index), NULL,
                                               "EtherCAT%u", master->index);
#else
    master->class_device = class_device_create(class,
                                               MKDEV(MAJOR(device_number), master->index), NULL,
                                               "EtherCAT%u", master->index);
#endif
    if (IS_ERR(master->class_device))
    {
        EC_MASTER_ERR(master, "Failed to create class device!\n");
        ret = PTR_ERR(master->class_device);
        goto out_clear_cdev;
    }

#ifdef EC_RTDM
    // 初始化RTDM设备
    ret = ec_rtdm_dev_init(&master->rtdm_dev, master);
    if (ret)
    {
        goto out_unregister_class_device;
    }
#endif

    return 0;

#ifdef EC_RTDM
out_unregister_class_device:
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 26)
    device_unregister(master->class_device);
#else
    class_device_unregister(master->class_device);
#endif
#endif
out_clear_cdev:
    ec_cdev_clear(&master->cdev);
out_clear_sync_mon:
    ec_datagram_clear(&master->sync_mon_datagram);
out_clear_sync64:
    ec_datagram_clear(&master->sync64_datagram);
out_clear_sync:
    ec_datagram_clear(&master->sync_datagram);
out_clear_ref_sync:
    ec_datagram_clear(&master->ref_sync_datagram);
out_clear_ext_datagrams:
    for (i = 0; i < EC_EXT_RING_SIZE; i++)
    {
        ec_datagram_clear(&master->ext_datagram_ring[i]);
    }
    ec_fsm_master_clear(&master->fsm);
    ec_datagram_clear(&master->fsm_datagram);
out_clear_devices:
    for (; dev_idx > 0; dev_idx--)
    {
        ec_device_clear(&master->devices[dev_idx - 1]);
    }
    return ret;
}

/*****************************************************************************/

/**
 * @brief	析构函数
 * @作用	清理EtherCAT主站对象
 * @param	master	EtherCAT主站对象
 */
void ec_master_clear(
    ec_master_t *master /**< EtherCAT主站 */
)
{
    unsigned int dev_idx, i;

#ifdef EC_RTDM
    ec_rtdm_dev_clear(&master->rtdm_dev);
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 26)
    device_unregister(master->class_device);
#else
    class_device_unregister(master->class_device);
#endif

    ec_cdev_clear(&master->cdev);

    ec_master_slaves_not_available(master);
#ifdef EC_EOE
    // 释放所有EoE处理程序
    ec_master_clear_eoe_handlers(master, 1);
#endif
    ec_master_clear_domains(master);
    ec_master_clear_slave_configs(master);
    ec_master_clear_slaves(master);
    ec_master_clear_sii_images(master);

    ec_datagram_clear(&master->sync_mon_datagram);
    ec_datagram_clear(&master->sync64_datagram);
    ec_datagram_clear(&master->sync_datagram);
    ec_datagram_clear(&master->ref_sync_datagram);

    for (i = 0; i < EC_EXT_RING_SIZE; i++)
    {
        ec_datagram_clear(&master->ext_datagram_ring[i]);
    }

    ec_fsm_master_clear(&master->fsm);
    ec_datagram_clear(&master->fsm_datagram);

    for (dev_idx = EC_DEVICE_MAIN; dev_idx < ec_master_num_devices(master);
         dev_idx++)
    {
        ec_device_clear(&master->devices[dev_idx]);
    }

    if (master->pcap_data)
    {
        vfree(master->pcap_data);
        master->pcap_data = NULL;
        master->pcap_curr_data = NULL;
    }
}

/*****************************************************************************/

#ifdef EC_EOE
/**
 * @brief	清除并释放自动创建的EoE处理程序
 * @作用	清除手动创建的EoE处理程序的从站引用
 * @param	master	EtherCAT主站对象
 * @param	free_all	释放自动和手动创建的EoE处理程序
 */
void ec_master_clear_eoe_handlers(
    ec_master_t *master, /**< EtherCAT主站 */
    unsigned int free_all /**< 释放自动和手动EoE处理程序 */
)
{
    ec_eoe_t *eoe, *next;

    list_for_each_entry_safe(eoe, next, &master->eoe_handlers, list)
    {
        if (free_all || eoe->auto_created)
        {
            // 释放所有或者自动创建的EoE处理程序：清除并释放
            list_del(&eoe->list);
            ec_eoe_clear(eoe);
            kfree(eoe);
        }
        else
        {
            // 手动创建的EoE处理程序：清除从站引用
            ec_eoe_clear_slave(eoe);
        }
    }
}
#endif

/*****************************************************************************/

/**
 * @brief	清除所有从站配置
 * @param	master	EtherCAT主站对象
 */
void ec_master_clear_slave_configs(ec_master_t *master)
{
    ec_slave_config_t *sc, *next;

    master->dc_ref_config = NULL;

    list_for_each_entry_safe(sc, next, &master->configs, list)
    {
        list_del(&sc->list);
        ec_slave_config_clear(sc);
        kfree(sc);
    }
}

/*****************************************************************************/


/**
 * @brief	清除SII数据
 * @param	sii_image	SII 映像对象
 */
void ec_sii_image_clear(ec_sii_image_t *sii_image)
{
    unsigned int i;
    ec_pdo_t *pdo, *next_pdo;

    // 释放所有同步管理器
    if (sii_image->sii.syncs)
    {
        for (i = 0; i < sii_image->sii.sync_count; i++)
        {
            ec_sync_clear(&sii_image->sii.syncs[i]);
        }
        kfree(sii_image->sii.syncs);
        sii_image->sii.syncs = NULL;
    }

    // 释放所有字符串
    if (sii_image->sii.strings)
    {
        for (i = 0; i < sii_image->sii.string_count; i++)
            kfree(sii_image->sii.strings[i]);
        kfree(sii_image->sii.strings);
    }

    // 释放所有SII PDOs
    list_for_each_entry_safe(pdo, next_pdo, &sii_image->sii.pdos, list)
    {
        list_del(&pdo->list);
        ec_pdo_clear(pdo);
        kfree(pdo);
    }

    if (sii_image->words)
    {
        kfree(sii_image->words);
    }
}


/*****************************************************************************/

/**
 * @brief 清除总线扫描期间应用的SII数据。
 * 
 * @param master EtherCAT主站。
 */
void ec_master_clear_sii_images(
    ec_master_t *master /**< EtherCAT主站。 */
)
{
    ec_sii_image_t *sii_image, *next;

    list_for_each_entry_safe(sii_image, next, &master->sii_images, list)
    {
#ifdef EC_SII_CACHE
        if ((master->phase != EC_OPERATION) ||
            ((sii_image->sii.serial_number == 0) && (sii_image->sii.alias == 0)))
#endif
        {
            list_del(&sii_image->list);
            ec_sii_image_clear(sii_image);
            kfree(sii_image);
        }
    }
}


/*****************************************************************************/

/**
 * @brief 设置标志，表示从站不可用于从站请求处理。
 *
 * 从主站fsm中调用，该函数在master_sem锁内处理。
 *
 * @param master EtherCAT主站。
 */
void ec_master_slaves_not_available(ec_master_t *master)
{
    master->rt_slaves_available = 0;
}


/*****************************************************************************/

/**
 * @brief 设置标志，表示从站现在可用于从站请求处理。
 *
 * 从主站fsm中调用，该函数在master_sem锁内处理。
 *
 * @param master EtherCAT主站。
 */
void ec_master_slaves_available(ec_master_t *master)
{
    master->rt_slaves_available = 1;
}

/*****************************************************************************/

/**
 * @brief 清除所有从站。
 *
 * 该函数的作用是清除主站中的所有从站。
 *
 * @param master EtherCAT主站。
 * @retval 无
 */
void ec_master_clear_slaves(ec_master_t *master)
{
    ec_slave_t *slave;

    master->dc_ref_clock = NULL;

    // 外部请求已过时，因此我们唤醒待处理的等待者并将其从列表中移除。

    while (!list_empty(&master->sii_requests))
    {
        ec_sii_write_request_t *request =
            list_entry(master->sii_requests.next,
                       ec_sii_write_request_t, list);
        list_del_init(&request->list); // 出队
        EC_MASTER_WARN(master, "丢弃SII请求，从站 %s-%u 即将被删除。\n",
                       ec_device_names[request->slave->device_index != 0],
                       request->slave->ring_position);
        request->state = EC_INT_REQUEST_FAILURE;
        wake_up_all(&master->request_queue);
    }

    master->fsm_slave = NULL;
    INIT_LIST_HEAD(&master->fsm_exec_list);
    master->fsm_exec_count = 0;

    for (slave = master->slaves;
         slave < master->slaves + master->slave_count;
         slave++)
    {
        ec_slave_clear(slave);
    }

    if (master->slaves)
    {
        kfree(master->slaves);
        master->slaves = NULL;
    }

    master->slave_count = 0;
}


/*****************************************************************************/

/**
 * @brief 清除所有域。
 *
 * 该函数的作用是清除主站中的所有域。
 *
 * @param master EtherCAT主站。
 * @retval 无
 */
void ec_master_clear_domains(ec_master_t *master)
{
    ec_domain_t *domain, *next;

    list_for_each_entry_safe(domain, next, &master->domains, list)
    {
        list_del(&domain->list);
        ec_domain_clear(domain);
        kfree(domain);
    }
}


/*****************************************************************************/

/**
 * @brief 清除应用程序应用的配置。
 * @作用：清除应用程序应用的配置。
 * @param master EtherCAT主站。
 * @retval 无。
 */
void ec_master_clear_config(
    ec_master_t *master /**< EtherCAT主站。 */
)
{
    ec_lock_down(&master->master_sem);
    ec_master_clear_domains(master);
    ec_master_clear_slave_configs(master);
    ec_lock_up(&master->master_sem);
}

/*****************************************************************************/

/**
@brief 内部发送回调函数。
@作用 用于发送EtherCAT主站的内部回调函数。
@param cb_data 回调数据。
@retval 无
*/
void ec_master_internal_send_cb(
void *cb_data /**< 回调数据。 */
)
{
    ec_master_t *master = (ec_master_t *)cb_data;
    ec_lock_down(&master->io_sem);
    ecrt_master_send_ext(master);
    ec_lock_up(&master->io_sem);
}


/*****************************************************************************/

/**
 * @brief 内部接收回调函数。
 *
 * 此函数用作EtherCAT主站的内部接收回调函数。
 *
 * @param cb_data 回调数据。
 * @retval 无。
 */
void ec_master_internal_receive_cb(
    void *cb_data /**< 回调数据。 */
)
{
    ec_master_t *master = (ec_master_t *)cb_data;
    ec_lock_down(&master->io_sem);
    ecrt_master_receive(master);
    ec_lock_up(&master->io_sem);
}
*/
/*****************************************************************************/

/**
 * @brief 启动主站线程。
 *
 * 此函数用于启动EtherCAT主站的线程。
 *
 * @param master EtherCAT主站。
 * @param thread_func 要启动的线程函数。
 * @param name 线程名称。
 * @retval 0 成功。
 * @retval <0 错误代码。
 */
int ec_master_thread_start(
    ec_master_t *master,        /**< EtherCAT主站 */
    int (*thread_func)(void *), /**< 要启动的线程函数 */
    const char *name            /**< 线程名称 */
)
{
    EC_MASTER_INFO(master, "启动%s线程。\n", name);
    master->thread = kthread_run(thread_func, master, name);
    if (IS_ERR(master->thread))
    {
        int err = (int)PTR_ERR(master->thread);
        EC_MASTER_ERR(master, "无法启动主站线程（错误代码 %i）！\n", err);
        master->thread = NULL;
        return err;
    }

    return 0;
}
*/

/*****************************************************************************/

/**
 * @brief 停止主站线程。
 *
 * 此函数用于停止EtherCAT主站的线程。
 *
 * @param master EtherCAT主站。
 * @retval 无。
 */
void ec_master_thread_stop(
    ec_master_t *master /**< EtherCAT主站 */
)
{
    unsigned long sleep_jiffies;

    if (!master->thread)
    {
        EC_MASTER_WARN(master, "%s(): 已经停止！\n", __func__);
        return;
    }

    EC_MASTER_DBG(master, 1, "停止主站线程。\n");

    kthread_stop(master->thread);
    master->thread = NULL;
    EC_MASTER_INFO(master, "主站线程已退出。\n");

    if (master->fsm_datagram.state != EC_DATAGRAM_SENT)
    {
        return;
    }

    // 等待FSM数据报
    sleep_jiffies = max(HZ / 100, 1); // 10毫秒，至少1个jiffy
    schedule_timeout(sleep_jiffies);
}
*/

/*****************************************************************************/

/**
 * @brief 从ORPHANED（孤立）状态转换到IDLE（空闲）阶段的函数。
 *
 * 此函数用于将EtherCAT主站从ORPHANED状态转换到IDLE状态。
 *
 * @param master EtherCAT主站。
 * @return 成功返回零，否则返回负的错误代码。
 */
int ec_master_enter_idle_phase(
    ec_master_t *master /**< EtherCAT主站 */
)
{
    int ret;
    ec_device_index_t dev_idx;
#ifdef EC_EOE
    int i;
    int master_index = 0;
    uint16_t alias, ring_position = 0;
#endif

    EC_MASTER_DBG(master, 1, "ORPHANED -> IDLE.\n");

    master->send_cb = ec_master_internal_send_cb;
    master->receive_cb = ec_master_internal_receive_cb;
    master->cb_data = master;

    master->phase = EC_IDLE;

    // 重置响应从站的数量以触发扫描
    for (dev_idx = EC_DEVICE_MAIN; dev_idx < ec_master_num_devices(master);
         dev_idx++)
    {
        master->fsm.slaves_responding[dev_idx] = 0;
    }

#ifdef EC_EOE
    // 在启动时为该主站创建EOE接口
    // 注意：需要主站的主设备进行配置以初始化EOE的MAC地址
    for (i = 0; i < eoe_count; i++)
    {
        ret = ec_eoe_parse(eoe_interfaces[i], &master_index, &alias,
                           &ring_position);

        if ((ret == 0) && (master_index == master->index))
        {
            EC_MASTER_INFO(master, "为主站 %d 添加EOE接口 \"%s\"，别名 %u，环位置 %u。\n",
                           master_index, eoe_interfaces[i], alias, ring_position);
            ecrt_master_eoe_addif(master, alias, ring_position);
        }
    }
#endif

    ret = ec_master_thread_start(master, ec_master_idle_thread,
                                 "EtherCAT-IDLE");
    if (ret)
        master->phase = EC_ORPHANED;

    return ret;
}

/*****************************************************************************/

/**
 * @brief 从IDLE（空闲）状态转换到ORPHANED（孤立）阶段的函数。
 *
 * 此函数用于将EtherCAT主站从IDLE状态转换到ORPHANED状态。
 *
 * @param master EtherCAT主站。
 * @return 无。
 */
void ec_master_leave_idle_phase(ec_master_t *master /**< EtherCAT主站 */)
{
    EC_MASTER_DBG(master, 1, "IDLE -> ORPHANED.\n");

    master->phase = EC_ORPHANED;

    ec_master_slaves_not_available(master);
#ifdef EC_EOE
    ec_master_eoe_stop(master);
#endif
    ec_master_thread_stop(master);

    ec_lock_down(&master->master_sem);
    ec_master_clear_slaves(master);
    ec_master_clear_sii_images(master);
    ec_lock_up(&master->master_sem);

    ec_fsm_master_reset(&master->fsm);
}

/*****************************************************************************/

/**
 * @brief 从IDLE（空闲）状态转换到OPERATION（运行）阶段的函数。
 *
 * 此函数用于将EtherCAT主站从IDLE状态转换到OPERATION状态。
 *
 * @param master EtherCAT主站。
 * @return 成功返回零，否则返回负的错误代码。
 */
int ec_master_enter_operation_phase(
    ec_master_t *master /**< EtherCAT主站 */
)
{
    int ret = 0;
    ec_slave_t *slave;

    EC_MASTER_DBG(master, 1, "IDLE -> OPERATION.\n");

    ec_lock_down(&master->config_sem);
    if (master->config_busy)
    {
        ec_lock_up(&master->config_sem);

        // 等待从站配置完成
        ret = wait_event_interruptible(master->config_queue,
                                       !master->config_busy);
        if (ret)
        {
            EC_MASTER_INFO(master, "从站配置被信号中断，正在完成配置。\n");
            goto out_return;
        }

        EC_MASTER_DBG(master, 1, "等待待处理的从站配置返回。\n");
    }
    else
    {
        ec_lock_up(&master->config_sem);
    }

    ec_lock_down(&master->scan_sem);
    master->allow_scan = 0; // 'lock' the slave list
    if (!master->scan_busy)
    {
        ec_lock_up(&master->scan_sem);
    }
    else
    {
        ec_lock_up(&master->scan_sem);

        // 等待从站扫描完成
        ret = wait_event_interruptible(master->scan_queue,
                                       !master->scan_busy);
        if (ret)
        {
            EC_MASTER_INFO(master, "等待从站扫描被信号中断。\n");
            goto out_allow;
        }

        EC_MASTER_DBG(master, 1, "等待待处理的从站扫描返回。\n");
    }

    // 设置所有从站的状态为PREOP
    for (slave = master->slaves;
         slave < master->slaves + master->slave_count;
         slave++)
    {
        ec_slave_request_state(slave, EC_SLAVE_STATE_PREOP);
    }

    master->phase = EC_OPERATION;
    master->app_send_cb = NULL;
    master->app_receive_cb = NULL;
    master->app_cb_data = NULL;
    return ret;

out_allow:
    master->allow_scan = 1;
out_return:
    return ret;
}

/*****************************************************************************/

/**
 * @brief 从OPERATION（运行）状态转换到IDLE（空闲）阶段的函数。
 *
 * 此函数用于将EtherCAT主站从OPERATION状态转换到IDLE状态。
 *
 * @param master EtherCAT主站。
 * @return 无。
 */
void ec_master_leave_operation_phase(
    ec_master_t *master /**< EtherCAT主站 */
)
{
    if (master->active)
    {
        ecrt_master_deactivate(master); // 同时清除配置
    }
    else
    {
        ec_master_clear_config(master);
    }

    /* 重新允许扫描以进行IDLE阶段。 */
    master->allow_scan = 1;

    EC_MASTER_DBG(master, 1, "OPERATION -> IDLE.\n");

    master->phase = EC_IDLE;
}

/*****************************************************************************/

/**
 * @brief 注入适合于数据报队列的外部数据报的函数。
 *
 * 此函数用于注入适合于数据报队列的外部数据报。
 *
 * @param master EtherCAT主站。
 * @return 无。
 */
void ec_master_inject_external_datagrams(
    ec_master_t *master /**< EtherCAT主站 */
)
{
    ec_datagram_t *datagram;
    size_t queue_size = 0, new_queue_size = 0;
#if DEBUG_INJECT
    unsigned int datagram_count = 0;
#endif

    if (master->ext_ring_idx_rt == master->ext_ring_idx_fsm)
    {
        // 没有需要注入的数据报
        return;
    }

    list_for_each_entry(datagram, &master->datagram_queue, queue)
    {
        if (datagram->state == EC_DATAGRAM_QUEUED)
        {
            queue_size += datagram->data_size;
        }
    }

#if DEBUG_INJECT
    EC_MASTER_DBG(master, 1, "正在注入数据报，queue_size=%zu\n",
                  queue_size);
#endif

    while (master->ext_ring_idx_rt != master->ext_ring_idx_fsm)
    {
        datagram = &master->ext_datagram_ring[master->ext_ring_idx_rt];

        if (datagram->state != EC_DATAGRAM_INIT)
        {
            // 跳过数据报
            master->ext_ring_idx_rt =
                (master->ext_ring_idx_rt + 1) % EC_EXT_RING_SIZE;
            continue;
        }

        new_queue_size = queue_size + datagram->data_size;
        if (new_queue_size <= master->max_queue_size)
        {
#if DEBUG_INJECT
            EC_MASTER_DBG(master, 1, "正在注入数据报 %s，大小为%zu，queue_size=%zu\n",
                          datagram->name,
                          datagram->data_size, new_queue_size);
            datagram_count++;
#endif
#ifdef EC_HAVE_CYCLES
            datagram->cycles_sent = 0;
#endif
            datagram->jiffies_sent = 0;
            ec_master_queue_datagram(master, datagram);
            queue_size = new_queue_size;
        }
        else if (datagram->data_size > master->max_queue_size)
        {
            datagram->state = EC_DATAGRAM_ERROR;
            EC_MASTER_ERR(master, "外部数据报 %s 太大，大小为%zu，最大队列大小为%zu\n",
                          datagram->name, datagram->data_size,
                          master->max_queue_size);
        }
        else
        { // 数据报无法在当前周期内容纳
#ifdef EC_HAVE_CYCLES
            cycles_t cycles_now = get_cycles();

            if (cycles_now - datagram->cycles_sent > ext_injection_timeout_cycles)
#else
            if (jiffies - datagram->jiffies_sent > ext_injection_timeout_jiffies)
#endif
            {
#if defined EC_RT_SYSLOG || DEBUG_INJECT
                unsigned int time_us;
#endif

                datagram->state = EC_DATAGRAM_ERROR;

#if defined EC_RT_SYSLOG || DEBUG_INJECT
#ifdef EC_HAVE_CYCLES
                time_us = (unsigned int)((cycles_now - datagram->cycles_sent) * 1000LL) / cpu_khz;
#else
                time_us = (unsigned int)((jiffies - datagram->jiffies_sent) * 1000000 / HZ);
#endif
                EC_MASTER_ERR(master, "超时 %u 微秒：注入外部数据报 %s，大小为%zu，最大队列大小为%zu\n",
                              time_us, datagram->name,
                              datagram->data_size, master->max_queue_size);
#endif
            }
            else
            {
#if DEBUG_INJECT
                EC_MASTER_DBG(master, 1, "延迟注入外部数据报 %s，大小为%u，queue_size=%u\n",
                              datagram->name, datagram->data_size, queue_size);
#endif
                break;
            }
        }

        master->ext_ring_idx_rt =
            (master->ext_ring_idx_rt + 1) % EC_EXT_RING_SIZE;
    }

#if DEBUG_INJECT
    EC_MASTER_DBG(master, 1, "已注入 %u 个数据报。\n", datagram_count);
#endif
}

/*****************************************************************************/

/**
 * @brief 设置ecrt_master_send调用之间的预期间隔，并计算最大队列中的数据量。
 *
 * 此函数用于设置ecrt_master_send调用之间的预期间隔，并计算最大队列中的数据量。
 *
 * @param master EtherCAT主站。
 * @param send_interval 发送间隔。
 * @return 无。
 */
void ec_master_set_send_interval(
    ec_master_t *master,       /**< EtherCAT主站 */
    unsigned int send_interval /**< 发送间隔 */
)
{
    master->send_interval = send_interval;
    master->max_queue_size =
        (send_interval * 1000) / EC_BYTE_TRANSMISSION_TIME_NS;
    master->max_queue_size -= master->max_queue_size / 10;
}

/*****************************************************************************/

/**
 * @brief 请求重启此主站上的所有从站（如果支持）。
 *
 * 此函数用于请求重启此主站上的所有从站（如果支持）。
 *
 * @param master EtherCAT主站。
 * @return 无。
 */
void ec_master_reboot_slaves(
    ec_master_t *master /**< EtherCAT主站 */
)
{
    master->reboot = 1;
}

/*****************************************************************************/

/**
 * @brief 在外部数据报环中搜索空闲的数据报。
 *
 * 此函数用于在外部数据报环中搜索空闲的数据报。
 *
 * @param master EtherCAT主站。
 * @return 下一个空闲的数据报，如果没有则返回NULL。
 */
ec_datagram_t *ec_master_get_external_datagram(
    ec_master_t *master /**< EtherCAT主站 */
)
{
    if ((master->ext_ring_idx_fsm + 1) % EC_EXT_RING_SIZE !=
        master->ext_ring_idx_rt)
    {
        ec_datagram_t *datagram =
            &master->ext_datagram_ring[master->ext_ring_idx_fsm];
        /* 记录ec_master_inject_external_datagrams的排队时间 */
#ifdef EC_HAVE_CYCLES
        datagram->cycles_sent = get_cycles();
#endif
        datagram->jiffies_sent = jiffies;

        return datagram;
    }
    else
    {
        return NULL;
    }
}

/*****************************************************************************/

/**
 * @brief 将数据报放入数据报队列。
 *
 * 此函数用于将数据报放入数据报队列。
 *
 * @param master EtherCAT主站。
 * @param datagram 数据报。
 * @return 无。
 */
void ec_master_queue_datagram(
    ec_master_t *master,    /**< EtherCAT主站 */
    ec_datagram_t *datagram /**< 数据报 */
)
{
    ec_datagram_t *queued_datagram;

    /* 可能会出现在队列中重置状态为EC_DATAGRAM_INIT的数据报，然后使用此方法将其排队。
     * 在这种情况下，状态已经重置为EC_DATAGRAM_INIT。检查数据报是否已排队以避免重复排队（这将导致无限循环！）。
     * 将状态重新设置为EC_DATAGRAM_QUEUED，可能导致数据报不匹配。 */
    list_for_each_entry(queued_datagram, &master->datagram_queue, queue)
    {
        if (queued_datagram == datagram)
        {
            datagram->skip_count++;
#ifdef EC_RT_SYSLOG
            EC_MASTER_DBG(master, 1,
                          "数据报 %p 已排队（跳过）。\n", datagram);
#endif
            datagram->state = EC_DATAGRAM_QUEUED;
            return;
        }
    }

    if (datagram->state != EC_DATAGRAM_INVALID)
    {
        list_add_tail(&datagram->queue, &master->datagram_queue);
        datagram->state = EC_DATAGRAM_QUEUED;
    }
}

/*****************************************************************************/

/**
 * @brief 将数据报放入非应用数据报队列。
 *
 * 此函数用于将数据报放入非应用数据报队列。
 *
 * @param master EtherCAT主站。
 * @param datagram 数据报。
 * @return 无。
 */
void ec_master_queue_datagram_ext(
    ec_master_t *master,    /**< EtherCAT主站 */
    ec_datagram_t *datagram /**< 数据报 */
)
{
    ec_lock_down(&master->ext_queue_sem);
    list_add_tail(&datagram->queue, &master->ext_datagram_queue);
    ec_lock_up(&master->ext_queue_sem);
}

/*****************************************************************************/

static int index_in_use(ec_master_t *master, uint8_t index)
{
    ec_datagram_t *datagram;
    list_for_each_entry(datagram, &master->datagram_queue, queue) if (datagram->state == EC_DATAGRAM_SENT && datagram->index == index) return 1;
    return 0;
}

/** 发送队列中的数据报给特定的设备。
 *
 * @param master EtherCAT主站。
 * @param device_index 设备索引。
 * @return 发送的字节数。
 */
size_t ec_master_send_datagrams(
    ec_master_t *master,           /**< EtherCAT主站 */
    ec_device_index_t device_index /**< 设备索引 */
)
{
    ec_datagram_t *datagram, *next;
    size_t datagram_size;
    uint8_t *frame_data, *cur_data = NULL;
    void *follows_word;
#ifdef EC_HAVE_CYCLES
    cycles_t cycles_start, cycles_sent, cycles_end;
#endif
    unsigned long jiffies_sent;
    unsigned int frame_count, more_datagrams_waiting;
    struct list_head sent_datagrams;
    size_t sent_bytes = 0;
    uint8_t last_index;

#ifdef EC_HAVE_CYCLES
    cycles_start = get_cycles();
#endif
    frame_count = 0;
    INIT_LIST_HEAD(&sent_datagrams);

    EC_MASTER_DBG(master, 2, "%s(device_index = %u)\n",
                  __func__, device_index);

    do
    {
        frame_data = NULL;
        follows_word = NULL;
        more_datagrams_waiting = 0;

        // 填充当前帧的数据报
        list_for_each_entry(datagram, &master->datagram_queue, queue)
        {
            if (datagram->state != EC_DATAGRAM_QUEUED ||
                datagram->device_index != device_index)
            {
                continue;
            }

            if (!frame_data)
            {
                // 获取传输套接字缓冲区的指针
                frame_data =
                    ec_device_tx_data(&master->devices[device_index]);
                cur_data = frame_data + EC_FRAME_HEADER_SIZE;
            }

            // 当前数据报是否适合帧中？
            datagram_size = EC_DATAGRAM_HEADER_SIZE + datagram->data_size + EC_DATAGRAM_FOOTER_SIZE;
            if (cur_data - frame_data + datagram_size > ETH_DATA_LEN)
            {
                more_datagrams_waiting = 1;
                break;
            }

            // 不要重用待处理的数据报的索引，以避免在ec_master_receive_datagrams()中混淆
            last_index = master->datagram_index;
            while (index_in_use(master, master->datagram_index))
            {
                if (++master->datagram_index == last_index)
                {
                    EC_MASTER_ERR(master, "没有空闲的数据报索引，发送延迟\n");
                    goto break_send;
                }
            }
            datagram->index = master->datagram_index++;

            list_add_tail(&datagram->sent, &sent_datagrams);

            EC_MASTER_DBG(master, 2, "添加数据报 0x%02X\n",
                          datagram->index);

            // 在上一个数据报中设置“数据报后续”标志
            if (follows_word)
            {
                EC_WRITE_U16(follows_word,
                             EC_READ_U16(follows_word) | 0x8000);
            }

            // EtherCAT数据报头
            EC_WRITE_U8(cur_data, datagram->type);
            EC_WRITE_U8(cur_data + 1, datagram->index);
            memcpy(cur_data + 2, datagram->address, EC_ADDR_LEN);
            EC_WRITE_U16(cur_data + 6, datagram->data_size & 0x7FF);
            EC_WRITE_U16(cur_data + 8, 0x0000);
            follows_word = cur_data + 6;
            cur_data += EC_DATAGRAM_HEADER_SIZE;

            // EtherCAT数据报数据
            memcpy(cur_data, datagram->data, datagram->data_size);
            cur_data += datagram->data_size;

            // EtherCAT数据报尾部
            EC_WRITE_U16(cur_data, 0x0000); // 重置工作计数器
            cur_data += EC_DATAGRAM_FOOTER_SIZE;
        }

    break_send:
        if (list_empty(&sent_datagrams))
        {
            EC_MASTER_DBG(master, 2, "没有要发送的数据。\n");
            break;
        }

        // EtherCAT帧头
        EC_WRITE_U16(frame_data, ((cur_data - frame_data - EC_FRAME_HEADER_SIZE) & 0x7FF) | 0x1000);

        // 填充帧
        while (cur_data - frame_data < ETH_ZLEN - ETH_HLEN)
            EC_WRITE_U8(cur_data++, 0x00);

        EC_MASTER_DBG(master, 2, "帧大小：%zu\n", cur_data - frame_data);

        // 发送帧
        ec_device_send(&master->devices[device_index],
                       cur_data - frame_data);
        /* 前导码和帧间隙 */
        sent_bytes += ETH_HLEN + cur_data - frame_data + ETH_FCS_LEN + 20;
#ifdef EC_HAVE_CYCLES
        cycles_sent = get_cycles();
#endif
        jiffies_sent = jiffies;

        // 设置数据报的状态和发送时间戳
        list_for_each_entry_safe(datagram, next, &sent_datagrams, sent)
        {
            datagram->state = EC_DATAGRAM_SENT;
#ifdef EC_HAVE_CYCLES
            datagram->cycles_sent = cycles_sent;
#endif
            datagram->jiffies_sent = jiffies_sent;
            datagram->app_time_sent = master->app_time;
            list_del_init(&datagram->sent); // 清空已发送数据报的列表
        }

        frame_count++;
    } while (more_datagrams_waiting && frame_count < EC_TX_RING_SIZE);

#ifdef EC_HAVE_CYCLES
    if (unlikely(master->debug_level > 1))
    {
        cycles_end = get_cycles();
        EC_MASTER_DBG(master, 0, "%s()"
                                 " 在%u微秒内发送了%u帧。\n",
                      __func__, frame_count,
                      (unsigned int)(cycles_end - cycles_start) * 1000 / cpu_khz);
    }
#endif
    return sent_bytes;
}

/*****************************************************************************/

/** 处理接收到的帧。
 *
 * 网络驱动程序会为每个接收到的帧调用此函数。
 *
 * @param master EtherCAT主站。
 * @param device EtherCAT设备。
 * @param frame_data 帧数据。
 * @param size 接收到的数据大小。
 * @return 无。
 */
void ec_master_receive_datagrams(
    ec_master_t *master,       /**< EtherCAT主站 */
    ec_device_t *device,       /**< EtherCAT设备 */
    const uint8_t *frame_data, /**< 帧数据 */
    size_t size                /**< 接收到的数据大小 */
)
{
    size_t frame_size, data_size;
    uint8_t datagram_type, datagram_index, datagram_mbox_prot;
#ifdef EC_EOE
    uint8_t eoe_type;
#endif
    unsigned int cmd_follows, datagram_slave_addr, datagram_offset_addr, datagram_wc, matched;
    const uint8_t *cur_data;
    ec_datagram_t *datagram;
    ec_slave_t *slave;

    if (unlikely(size < EC_FRAME_HEADER_SIZE))
    {
        if (master->debug_level || FORCE_OUTPUT_CORRUPTED)
        {
            EC_MASTER_DBG(master, 0, "在%s上接收到损坏的帧（大小%zu < %u字节）：\n",
                          device->dev->name, size, EC_FRAME_HEADER_SIZE);
            ec_print_data(frame_data, size);
        }
        master->stats.corrupted++;
#ifdef EC_RT_SYSLOG
        ec_master_output_stats(master);
#endif
        return;
    }

    cur_data = frame_data;

    // 检查整个帧的长度
    frame_size = EC_READ_U16(cur_data) & 0x07FF;
    cur_data += EC_FRAME_HEADER_SIZE;

    if (unlikely(frame_size > size))
    {
        if (master->debug_level || FORCE_OUTPUT_CORRUPTED)
        {
            EC_MASTER_DBG(master, 0, "在%s上接收到损坏的帧（无效的帧大小%zu，接收到的大小%zu）：\n",
                          device->dev->name,
                          frame_size, size);
            ec_print_data(frame_data, size);
        }
        master->stats.corrupted++;
#ifdef EC_RT_SYSLOG
        ec_master_output_stats(master);
#endif
        return;
    }

    cmd_follows = 1;
    while (cmd_follows)
    {
        // 处理数据报头
        datagram_type = EC_READ_U8(cur_data);
        datagram_index = EC_READ_U8(cur_data + 1);
        datagram_slave_addr = EC_READ_U16(cur_data + 2);
        datagram_offset_addr = EC_READ_U16(cur_data + 4);
        data_size = EC_READ_U16(cur_data + 6) & 0x07FF;
        cmd_follows = EC_READ_U16(cur_data + 6) & 0x8000;
        cur_data += EC_DATAGRAM_HEADER_SIZE;

        if (unlikely(cur_data - frame_data + data_size + EC_DATAGRAM_FOOTER_SIZE > size))
        {
            if (master->debug_level || FORCE_OUTPUT_CORRUPTED)
            {
                EC_MASTER_DBG(master, 0, "在%s上接收到损坏的帧（无效的数据大小%zu）：\n",
                              device->dev->name, data_size);
                ec_print_data(frame_data, size);
            }
            master->stats.corrupted++;
#ifdef EC_RT_SYSLOG
            ec_master_output_stats(master);
#endif
            return;
        }

        // 在队列中搜索匹配的数据报
        matched = 0;
        list_for_each_entry(datagram, &master->datagram_queue, queue)
        {
            if (datagram->index == datagram_index && datagram->state == EC_DATAGRAM_SENT && datagram->type == datagram_type && datagram->data_size == data_size)
            {
                matched = 1;
                break;
            }
        }

        // 没有找到匹配的数据报
        if (!matched)
        {
            master->stats.unmatched++;
#ifdef EC_RT_SYSLOG
            ec_master_output_stats(master);
#endif

            if (unlikely(master->debug_level > 0))
            {
                EC_MASTER_DBG(master, 0, "未匹配的数据报：\n");
                ec_print_data(cur_data - EC_DATAGRAM_HEADER_SIZE,
                              EC_DATAGRAM_HEADER_SIZE + data_size + EC_DATAGRAM_FOOTER_SIZE);
#ifdef EC_DEBUG_RING
                ec_device_debug_ring_print(&master->devices[EC_DEVICE_MAIN]);
#endif
            }

            cur_data += data_size + EC_DATAGRAM_FOOTER_SIZE;
            continue;
        }

        if (datagram->type != EC_DATAGRAM_APWR &&
            datagram->type != EC_DATAGRAM_FPWR &&
            datagram->type != EC_DATAGRAM_BWR &&
            datagram->type != EC_DATAGRAM_LWR)
        {

            // 使用物理从站地址读取的常规邮箱调度程序
            if (datagram->type == EC_DATAGRAM_FPRD)
            {
                datagram_wc = EC_READ_U16(cur_data + data_size);
                if (datagram_wc)
                {
                    if (master->slaves != NULL)
                    {
                        for (slave = master->slaves; slave < master->slaves + master->slave_count; slave++)
                        {
                            if (slave->station_address == datagram_slave_addr)
                            {
                                break;
                            }
                        }
                        if (slave->station_address == datagram_slave_addr)
                        {
                            if (slave->configured_tx_mailbox_offset != 0)
                            {
                                if (datagram_offset_addr == slave->configured_tx_mailbox_offset)
                                {
                                    if (slave->valid_mbox_data)
                                    {
                                        // 检查邮箱头的从站地址是否为从站位置上方的MBox Gateway地址偏移量，并且是有效的MBox Gateway地址
                                        // 注意：数据报的站地址为从站位置+1
                                        // 注意：EL6614 EoE模块不会填充EoE响应中的邮箱头地址值。其他模块/协议可能也会这样做。
                                        if (unlikely(
                                                (EC_READ_U16(cur_data + 2) == datagram_slave_addr + EC_MBG_SLAVE_ADDR_OFFSET - 1) &&
                                                (EC_READ_U16(cur_data + 2) >= EC_MBG_SLAVE_ADDR_OFFSET)))
                                        {
                                            // EtherCAT邮箱网关响应
                                            if ((slave->mbox_mbg_data.data) && (data_size <= slave->mbox_mbg_data.data_size))
                                            {
                                                memcpy(slave->mbox_mbg_data.data, cur_data, data_size);
                                                slave->mbox_mbg_data.payload_size = data_size;
                                            }
                                        }
                                        else
                                        {
                                            datagram_mbox_prot = EC_READ_U8(cur_data + 5) & 0x0F;
                                            switch (datagram_mbox_prot)
                                            {
#ifdef EC_EOE
                                            case EC_MBOX_TYPE_EOE:
                                                // 检查EOE类型并存储在正确的处理程序的邮箱数据缓存中
                                                eoe_type = EC_READ_U8(cur_data + 6) & 0x0F;

                                                switch (eoe_type)
                                                {

                                                case EC_EOE_TYPE_FRAME_FRAG:
                                                    // EoE帧片段处理程序
                                                    if ((slave->mbox_eoe_frag_data.data) && (data_size <= slave->mbox_eoe_frag_data.data_size))
                                                    {
                                                        memcpy(slave->mbox_eoe_frag_data.data, cur_data, data_size);
                                                        slave->mbox_eoe_frag_data.payload_size = data_size;
                                                    }
                                                    break;
                                                case EC_EOE_TYPE_INIT_RES:
                                                    // EoE初始化/设置IP响应处理程序
                                                    if ((slave->mbox_eoe_init_data.data) && (data_size <= slave->mbox_eoe_init_data.data_size))
                                                    {
                                                        memcpy(slave->mbox_eoe_init_data.data, cur_data, data_size);
                                                        slave->mbox_eoe_init_data.payload_size = data_size;
                                                    }
                                                    break;
                                                default:
                                                    EC_MASTER_DBG(master, 1, "从从站接收到未处理的EOE协议类型：%u 协议：%u 类型：%x\n",
                                                                  datagram_slave_addr, datagram_mbox_prot, eoe_type);
                                                    // 将接收到的数据复制到数据报内存中。
                                                    memcpy(datagram->data, cur_data, data_size);
                                                    break;
                                                }
                                                break;
#endif
                                            case EC_MBOX_TYPE_COE:
                                                if ((slave->mbox_coe_data.data) && (data_size <= slave->mbox_coe_data.data_size))
                                                {
                                                    memcpy(slave->mbox_coe_data.data, cur_data, data_size);
                                                    slave->mbox_coe_data.payload_size = data_size;
                                                }
                                                break;
                                            case EC_MBOX_TYPE_FOE:
                                                if ((slave->mbox_foe_data.data) && (data_size <= slave->mbox_foe_data.data_size))
                                                {
                                                    memcpy(slave->mbox_foe_data.data, cur_data, data_size);
                                                    slave->mbox_foe_data.payload_size = data_size;
                                                }
                                                break;
                                            case EC_MBOX_TYPE_SOE:
                                                if ((slave->mbox_soe_data.data) && (data_size <= slave->mbox_soe_data.data_size))
                                                {
                                                    memcpy(slave->mbox_soe_data.data, cur_data, data_size);
                                                    slave->mbox_soe_data.payload_size = data_size;
                                                }
                                                break;
                                            case EC_MBOX_TYPE_VOE:
                                                if ((slave->mbox_voe_data.data) && (data_size <= slave->mbox_voe_data.data_size))
                                                {
                                                    memcpy(slave->mbox_voe_data.data, cur_data, data_size);
                                                    slave->mbox_voe_data.payload_size = data_size;
                                                }
                                                break;
                                            default:
                                                EC_MASTER_DBG(master, 1, "从从站接收到未知的邮箱协议：从站：%u 协议：%u\n", datagram_slave_addr, datagram_mbox_prot);
                                                // 将接收到的数据复制到数据报内存中。
                                                memcpy(datagram->data, cur_data, data_size);
                                                break;
                                            }
                                        }
                                    }
                                    else
                                    {
                                        // 将接收到的数据复制到数据报内存中。
                                        memcpy(datagram->data, cur_data, data_size);
                                    }
                                }
                                else
                                {
                                    // 将接收到的数据复制到数据报内存中。
                                    memcpy(datagram->data, cur_data, data_size);
                                }
                            }
                            else
                            {
                                EC_MASTER_DBG(master, 1, "没有匹配的从站与数据报从站地址匹配：%u\n", datagram_slave_addr);
                            }
                        }
                        else
                        {
                            EC_MASTER_DBG(master, 1, "没有配置的从站！\n");
                            // 将接收到的数据复制到数据报内存中。
                            memcpy(datagram->data, cur_data, data_size);
                        }
                    }
                    else
                    {
                        EC_MASTER_DBG(master, 1, "没有配置的从站！\n");
                        // 将接收到的数据复制到数据报内存中。
                        memcpy(datagram->data, cur_data, data_size);
                    }
                }
                else
                {
                    // 将接收到的数据复制到数据报内存中。
                    memcpy(datagram->data, cur_data, data_size);
                }
            }
            else
            {
                // 将接收到的数据复制到数据报内存中。
                memcpy(datagram->data, cur_data, data_size);
            }
        }
        cur_data += data_size;

        // 设置数据报的工作计数器
        datagram->working_counter = EC_READ_U16(cur_data);
        cur_data += EC_DATAGRAM_FOOTER_SIZE;

#ifdef EC_HAVE_CYCLES
        datagram->cycles_received =
            master->devices[EC_DEVICE_MAIN].cycles_poll;
#endif
        datagram->jiffies_received =
            master->devices[EC_DEVICE_MAIN].jiffies_poll;

        barrier(); /* 重排序可能导致竞争条件 */

        // 出队接收到的数据报
        datagram->state = EC_DATAGRAM_RECEIVED;
        list_del_init(&datagram->queue);
    }
}

/*****************************************************************************/

/**
 * @brief 输出主站的统计信息。
 *
 * 此函数在需要时输出统计数据，但不超过每秒一次。
 *
 * @param master EtherCAT主站。
 * @return 无。
 */
void ec_master_output_stats(
    ec_master_t *master /**< EtherCAT主站 */
)
{
    if (unlikely(jiffies - master->stats.output_jiffies >= HZ))
    {
        if (!master->scan_busy || (master->debug_level > 0))
        {
            master->stats.output_jiffies = jiffies;
            if (master->stats.timeouts)
            {
                EC_MASTER_WARN(master, "%u个数据报超时！\n",
                               master->stats.timeouts);
                master->stats.timeouts = 0;
            }
            if (master->stats.corrupted)
            {
                EC_MASTER_WARN(master, "%u个帧损坏！\n",
                               master->stats.corrupted);
                master->stats.corrupted = 0;
            }
            if (master->stats.unmatched)
            {
                EC_MASTER_WARN(master, "%u个数据报无法匹配！\n",
                               master->stats.unmatched);
                master->stats.unmatched = 0;
            }
        }
    }
}

/*****************************************************************************/

/**
 * @brief 清除常见设备统计信息。
 *
 * 此函数用于清除常见设备统计信息。
 *
 * @param master EtherCAT主站。
 * @return 无。
 */
void ec_master_clear_device_stats(
    ec_master_t *master /**< EtherCAT主站 */
)
{
    unsigned int i;

    // 清零帧统计信息
    master->device_stats.tx_count = 0;
    master->device_stats.last_tx_count = 0;
    master->device_stats.rx_count = 0;
    master->device_stats.last_rx_count = 0;
    master->device_stats.tx_bytes = 0;
    master->device_stats.last_tx_bytes = 0;
    master->device_stats.rx_bytes = 0;
    master->device_stats.last_rx_bytes = 0;
    master->device_stats.last_loss = 0;

    for (i = 0; i < EC_RATE_COUNT; i++)
    {
        master->device_stats.tx_frame_rates[i] = 0;
        master->device_stats.rx_frame_rates[i] = 0;
        master->device_stats.tx_byte_rates[i] = 0;
        master->device_stats.rx_byte_rates[i] = 0;
        master->device_stats.loss_rates[i] = 0;
    }

    master->device_stats.jiffies = 0;
}

/*****************************************************************************/

/**
 * @brief 更新常见设备统计信息。
 *
 * 此函数用于更新常见设备的统计信息。
 *
 * @param master EtherCAT主站。
 * @return 无。
 */
void ec_master_update_device_stats(
    ec_master_t *master /**< EtherCAT主站 */
)
{
    ec_device_stats_t *s = &master->device_stats;
    s32 tx_frame_rate, rx_frame_rate, tx_byte_rate, rx_byte_rate, loss_rate;
    u64 loss;
    unsigned int i, dev_idx;

    // 帧统计信息
    if (likely(jiffies - s->jiffies < HZ))
    {
        return;
    }

    tx_frame_rate = (s->tx_count - s->last_tx_count) * 1000;
    rx_frame_rate = (s->rx_count - s->last_rx_count) * 1000;
    tx_byte_rate = s->tx_bytes - s->last_tx_bytes;
    rx_byte_rate = s->rx_bytes - s->last_rx_bytes;
    loss = s->tx_count - s->rx_count;
    loss_rate = (loss - s->last_loss) * 1000;

    /* 低通滤波器：
     *      Y_n = y_(n - 1) + T / tau * (x - y_(n - 1))   | T = 1
     *   -> Y_n += (x - y_(n - 1)) / tau
     */
    for (i = 0; i < EC_RATE_COUNT; i++)
    {
        s32 n = rate_intervals[i];
        s->tx_frame_rates[i] += (tx_frame_rate - s->tx_frame_rates[i]) / n;
        s->rx_frame_rates[i] += (rx_frame_rate - s->rx_frame_rates[i]) / n;
        s->tx_byte_rates[i] += (tx_byte_rate - s->tx_byte_rates[i]) / n;
        s->rx_byte_rates[i] += (rx_byte_rate - s->rx_byte_rates[i]) / n;
        s->loss_rates[i] += (loss_rate - s->loss_rates[i]) / n;
    }

    s->last_tx_count = s->tx_count;
    s->last_rx_count = s->rx_count;
    s->last_tx_bytes = s->tx_bytes;
    s->last_rx_bytes = s->rx_bytes;
    s->last_loss = loss;

    for (dev_idx = EC_DEVICE_MAIN; dev_idx < ec_master_num_devices(master);
         dev_idx++)
    {
        ec_device_update_stats(&master->devices[dev_idx]);
    }

    s->jiffies = jiffies;
}

/*****************************************************************************/

#ifdef EC_USE_HRTIMER

/*
 * 休眠相关函数：
 */
static enum hrtimer_restart ec_master_nanosleep_wakeup(struct hrtimer *timer)
{
    struct hrtimer_sleeper *t =
        container_of(timer, struct hrtimer_sleeper, timer);
    struct task_struct *task = t->task;

    t->task = NULL;
    if (task)
        wake_up_process(task);

    return HRTIMER_NORESTART;
}

/*****************************************************************************/

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 28)

/* 兼容新的hrtimer接口 */
/**
 * @brief   获取hrtimer的到期时间。
 * @param   timer   指向hrtimer结构的指针。
 * @retval  hrtimer的到期时间。
 */
static inline ktime_t hrtimer_get_expires(const struct hrtimer *timer)
{
    return timer->expires;
}

/*****************************************************************************/

/**
 * @brief   设置hrtimer的到期时间。
 * @param   timer   指向hrtimer结构的指针。
 * @param   time    新的到期时间。
 * @retval  None.
 */
static inline void hrtimer_set_expires(struct hrtimer *timer, ktime_t time)
{
    timer->expires = time;
}

#endif

/*****************************************************************************/

/**
 * @brief   使用高分辨率定时器睡眠指定的纳秒数。
 * @param   nsecs   要睡眠的纳秒数。
 * @retval  None.
 */
void ec_master_nanosleep(const unsigned long nsecs)
{
    struct hrtimer_sleeper t;
    enum hrtimer_mode mode = HRTIMER_MODE_REL;

    hrtimer_init(&t.timer, CLOCK_MONOTONIC, mode);
    t.timer.function = ec_master_nanosleep_wakeup;
    t.task = current;
#ifdef CONFIG_HIGH_RES_TIMERS
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2, 6, 24)
    t.timer.cb_mode = HRTIMER_CB_IRQSAFE_NO_RESTART;
#elif LINUX_VERSION_CODE <= KERNEL_VERSION(2, 6, 26)
    t.timer.cb_mode = HRTIMER_CB_IRQSAFE_NO_SOFTIRQ;
#elif LINUX_VERSION_CODE <= KERNEL_VERSION(2, 6, 28)
    t.timer.cb_mode = HRTIMER_CB_IRQSAFE_UNLOCKED;
#endif
#endif
    hrtimer_set_expires(&t.timer, ktime_set(0, nsecs));

    do
    {
        set_current_state(TASK_INTERRUPTIBLE);
        hrtimer_start(&t.timer, hrtimer_get_expires(&t.timer), mode);

        if (likely(t.task))
            schedule();

        hrtimer_cancel(&t.timer);
        mode = HRTIMER_MODE_ABS;

    } while (t.task && !signal_pending(current));
}

#endif // EC_USE_HRTIMER

/*****************************************************************************/

/* 优先级更改的兼容性 */
/**
 * @brief   设置进程的普通优先级。
 * @param   p       指向进程结构的指针。
 * @param   nice    优先级值。
 * @retval  None.
 */
static inline void set_normal_priority(struct task_struct *p, int nice)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 9, 0)
    sched_set_normal(p, nice);
#else
    struct sched_param param = {.sched_priority = 0};
    sched_setscheduler(p, SCHED_NORMAL, &param);
    set_user_nice(p, nice);
#endif
}

/*****************************************************************************/

/**
 * @brief 执行从站有限状态机（FSM）。
 *
 * 该函数用于执行从站的有限状态机（FSM）。
 *
 * @param master EtherCAT主站。
 * 
 * @retval 无。
 */
void ec_master_exec_slave_fsms(
    ec_master_t *master /**< EtherCAT主站。 */
)
{
    ec_datagram_t *datagram;
    ec_fsm_slave_t *fsm, *next;
    unsigned int count = 0;

    list_for_each_entry_safe(fsm, next, &master->fsm_exec_list, list)
    {
        if (!fsm->datagram)
        {
            EC_MASTER_WARN(master, "从站 %s-%u 的FSM没有数据报文。这是一个错误！\n",
                           ec_device_names[fsm->slave->device_index != 0],
                           fsm->slave->ring_position);
            list_del_init(&fsm->list);
            master->fsm_exec_count--;
            return;
        }

        if (fsm->datagram->state == EC_DATAGRAM_INIT ||
            fsm->datagram->state == EC_DATAGRAM_QUEUED ||
            fsm->datagram->state == EC_DATAGRAM_SENT)
        {
            // 上一个数据报文尚未发送或接收。
            // 等待下一次线程执行。
            return;
        }

        datagram = ec_master_get_external_datagram(master);
        if (!datagram)
        {
            // 当前没有可用的数据报文。
            EC_MASTER_WARN(master, "在执行从站FSM时没有可用的数据报文。这是一个错误！\n");
            continue;
        }

#if DEBUG_INJECT
        EC_MASTER_DBG(master, 1, "执行从站 %s-%u 的FSM。\n",
                      ec_device_names[fsm->slave->device_index != 0],
                      fsm->slave->ring_position);
#endif
        if (ec_fsm_slave_exec(fsm, datagram))
        {
            if (datagram->state != EC_DATAGRAM_INVALID)
            {
                // FSM消耗了数据报文
#if DEBUG_INJECT
                EC_MASTER_DBG(master, 1, "FSM消耗了数据报文 %s\n",
                              datagram->name);
#endif
                master->ext_ring_idx_fsm =
                    (master->ext_ring_idx_fsm + 1) % EC_EXT_RING_SIZE;
            }
        }
        else
        {
            // FSM完成
            list_del_init(&fsm->list);
            master->fsm_exec_count--;
#if DEBUG_INJECT
            EC_MASTER_DBG(master, 1, "FSM完成。剩余 %u 个。\n",
                          master->fsm_exec_count);
#endif
        }
    }

    while (master->fsm_exec_count < EC_EXT_RING_SIZE / 2 && count < master->slave_count)
    {

        if (ec_fsm_slave_is_ready(&master->fsm_slave->fsm))
        {
            datagram = ec_master_get_external_datagram(master);

            if (ec_fsm_slave_exec(&master->fsm_slave->fsm, datagram))
            {
                if (datagram->state != EC_DATAGRAM_INVALID)
                {
                    master->ext_ring_idx_fsm =
                        (master->ext_ring_idx_fsm + 1) % EC_EXT_RING_SIZE;
                }
                list_add_tail(&master->fsm_slave->fsm.list,
                              &master->fsm_exec_list);
                master->fsm_exec_count++;
#if DEBUG_INJECT
                EC_MASTER_DBG(master, 1, "新的从站 %s-%u 的FSM消耗了数据报文 %s，当前列表中有 %u 个FSM。\n",
                              ec_device_names[master->fsm_slave->device_index != 0],
                              master->fsm_slave->ring_position, datagram->name,
                              master->fsm_exec_count);
#endif
            }
        }

        master->fsm_slave++;
        if (master->fsm_slave >= master->slaves + master->slave_count)
        {
            master->fsm_slave = master->slaves;
        }
        count++;
    }
}

/*****************************************************************************/

/**
 * @brief	主站内核线程函数，用于IDLE阶段。
 * @作用：执行IDLE阶段的功能。
 * @param	priv_data	指向ec_master_t结构体的指针，包含主站的信息和状态。
 * @retval	返回执行结果的整数值。
 */
static int ec_master_idle_thread(void *priv_data)
{
    ec_master_t *master = (ec_master_t *)priv_data;
    int fsm_exec;
    size_t sent_bytes;

    // 设置IDLE阶段的发送间隔
    ec_master_set_send_interval(master, 1000000 / HZ);

    EC_MASTER_DBG(master, 1, "IDLE线程运行中，发送间隔 = %u 微秒，最大数据大小 = %zu\n",
                  master->send_interval,
                  master->max_queue_size);

    while (!kthread_should_stop())
    {
        // 输出数据报文统计信息
        ec_datagram_output_stats(&master->fsm_datagram);

        // 接收数据
        ec_lock_down(&master->io_sem);
        ecrt_master_receive(master);
        ec_lock_up(&master->io_sem);

        // 执行主站和从站状态机
        if (ec_lock_down_interruptible(&master->master_sem))
        {
            break;
        }

        fsm_exec = ec_fsm_master_exec(&master->fsm);

        // IDLE线程仍负责调用从站请求
        ec_master_exec_slave_fsms(master);

        ec_lock_up(&master->master_sem);

        // 队列并发送数据
        ec_lock_down(&master->io_sem);
        if (fsm_exec)
        {
            ec_master_queue_datagram(master, &master->fsm_datagram);
        }
        sent_bytes = ecrt_master_send(master);
        ec_lock_up(&master->io_sem);

        if (ec_fsm_master_idle(&master->fsm))
        {
#ifdef EC_USE_HRTIMER
            ec_master_nanosleep(master->send_interval * 1000);
#else
            set_current_state(TASK_INTERRUPTIBLE);
            schedule_timeout(1);
#endif
        }
        else
        {
#ifdef EC_USE_HRTIMER
            ec_master_nanosleep(
                sent_bytes * EC_BYTE_TRANSMISSION_TIME_NS * 6 / 5);
#else
            schedule();
#endif
        }
    }

    EC_MASTER_DBG(master, 1, "主站IDLE线程退出...\n");

    return 0;
}

/*****************************************************************************/

/**
 * @brief	主站内核线程函数，用于操作阶段。
 * @作用：执行操作阶段的功能。
 * @param	priv_data	指向ec_master_t结构体的指针，包含主站的信息和状态。
 * @retval	返回执行结果的整数值。
 */
static int ec_master_operation_thread(void *priv_data)
{
    ec_master_t *master = (ec_master_t *)priv_data;

    EC_MASTER_DBG(master, 1, "操作线程运行中，fsm间隔 = %u 微秒，最大数据大小 = %zu\n",
                  master->send_interval, master->max_queue_size);

    while (!kthread_should_stop())
    {
        // 输出数据报文统计信息
        ec_datagram_output_stats(&master->fsm_datagram);

        if (master->injection_seq_rt == master->injection_seq_fsm)
        {
            // 输出统计信息
            ec_master_output_stats(master);

            // 执行主站和从站状态机
            if (ec_lock_down_interruptible(&master->master_sem))
            {
                break;
            }

            if (ec_fsm_master_exec(&master->fsm))
            {
                // 注入数据报文（让RT线程将其加入队列，参见ecrt_master_send()）
                master->injection_seq_fsm++;
            }

            // 如果rt_slave_requests为true且从站可用，这将由应用程序显式调用ecrt_master_exec_slave_request()处理
            if (!master->rt_slave_requests || !master->rt_slaves_available)
            {
                ec_master_exec_slave_fsms(master);
            }

            ec_lock_up(&master->master_sem);
        }

#ifdef EC_USE_HRTIMER
        // 操作线程不应该比发送RT线程更快
        ec_master_nanosleep(master->send_interval * 1000);
#else
        if (ec_fsm_master_idle(&master->fsm))
        {
            set_current_state(TASK_INTERRUPTIBLE);
            schedule_timeout(1);
        }
        else
        {
            schedule();
        }
#endif
    }

    EC_MASTER_DBG(master, 1, "主站操作线程退出...\n");
    return 0;
}

/*****************************************************************************/

#ifdef EC_EOE
/**

@brief 启动以太网通过EtherCAT的处理。

@作用：在需要时启动以太网通过EtherCAT的处理。

@param master EtherCAT主站

@retval 无
*/
void ec_master_eoe_start(ec_master_t *master /**< EtherCAT主站 */)
{
if (master->eoe_thread)
{
EC_MASTER_WARN(master, "EoE已经在运行！\n");
return;
}

if (list_empty(&master->eoe_handlers))
{
return;
}

if (!master->send_cb || !master->receive_cb)
{
EC_MASTER_WARN(master, "EoE需要外部处理！\n");
return;
}

EC_MASTER_INFO(master, "启动EoE线程。\n");

master->eoe_thread = kthread_run(ec_master_eoe_thread, master,
"EtherCAT-EoE");
if (IS_ERR(master->eoe_thread))
{
int err = (int)PTR_ERR(master->eoe_thread);
EC_MASTER_ERR(master, "无法启动EoE线程（错误 %i）！\n",
err);
master->eoe_thread = NULL;
return;
}

set_normal_priority(master->eoe_thread, 0);
}

/*****************************************************************************/

/**
 * @brief 停止以太网通过EtherCAT的处理。
 * @作用：停止以太网通过EtherCAT的处理。
 * @param master EtherCAT主站
 * @retval 无
 */
void ec_master_eoe_stop(ec_master_t *master /**< EtherCAT主站 */)
{
    if (master->eoe_thread)
    {
        EC_MASTER_INFO(master, "停止EoE线程。\n");

        kthread_stop(master->eoe_thread);
        master->eoe_thread = NULL;
        EC_MASTER_INFO(master, "EoE线程已退出。\n");
    }
}

/*****************************************************************************/

#ifdef EC_RTDM

/**
 * @brief 检查是否有任何EOE处理程序处于打开状态。
 * @作用：检查是否有任何EOE处理程序处于打开状态。
 * @param master EtherCAT主站
 * @retval 如果有任何EOE处理程序处于打开状态，则返回1；如果没有，则返回0；否则返回负错误代码。
 */
int ec_master_eoe_is_open(ec_master_t *master /**< EtherCAT主站 */)
{
    ec_eoe_t *eoe;

    // 检查EOE是否已被主站处理，并且我们当前可以处理EOE
    if ((master->phase != EC_OPERATION) || master->eoe_thread ||
        !master->rt_slaves_available)
    {
        // 协议不可用
        return -ENOPROTOOPT;
    }

    ec_lock_down(&master->master_sem);
    list_for_each_entry(eoe, &master->eoe_handlers, list)
    {
        if (ec_eoe_is_open(eoe))
        {
            ec_lock_up(&master->master_sem);
            return 1;
        }
    }
    ec_lock_up(&master->master_sem);

    return 0;
}

/*****************************************************************************/

/**
 * @brief 检查是否有任何EOE处理程序处于打开状态。
 * @param master EtherCAT主站
 * @return 1：如果有要发送的内容；2：如果某个EOE处理程序仍有待处理的内容
 */
int ec_master_eoe_process(ec_master_t *master /**< EtherCAT主站 */)
{
    ec_eoe_t *eoe;
    int sth_to_send = 0;
    int sth_pending = 0;

    // 检查是否已经有主站正在处理EOE
    if (master->eoe_thread)
    {
        return 0;
    }

    // 实际的EOE处理
    ec_lock_down(&master->master_sem);
    list_for_each_entry(eoe, &master->eoe_handlers, list)
    {
        if (eoe->slave &&
            ((eoe->slave->current_state == EC_SLAVE_STATE_PREOP) ||
             (eoe->slave->current_state == EC_SLAVE_STATE_SAFEOP) ||
             (eoe->slave->current_state == EC_SLAVE_STATE_OP)))
        {
            ec_eoe_run(eoe);
            if (eoe->queue_datagram)
            {
                sth_to_send = EOE_STH_TO_SEND;
            }
            if (!ec_eoe_is_idle(eoe))
            {
                sth_pending = EOE_STH_PENDING;
            }
        }
    }

    if (sth_to_send)
    {
        list_for_each_entry(eoe, &master->eoe_handlers, list)
        {
            ec_eoe_queue(eoe);
        }
    }
    ec_lock_up(&master->master_sem);

    return sth_to_send + sth_pending;
}


#endif

/*****************************************************************************/


/** 执行以太网通过EtherCAT的处理。
 * @param priv_data 私有数据，指向EtherCAT主站
 * @return 0
 */
static int ec_master_eoe_thread(void *priv_data)
{
    ec_master_t *master = (ec_master_t *)priv_data;
    ec_eoe_t *eoe;
    unsigned int none_open, sth_to_send, all_idle;

    EC_MASTER_DBG(master, 1, "EoE线程正在运行。\n");

    while (!kthread_should_stop())
    {
        none_open = 1;
        all_idle = 1;

        ec_lock_down(&master->master_sem);
        list_for_each_entry(eoe, &master->eoe_handlers, list)
        {
            if (ec_eoe_is_open(eoe))
            {
                none_open = 0;
                break;
            }
        }
        ec_lock_up(&master->master_sem);

        if (none_open)
        {
            goto schedule;
        }

        // 接收数据报文
        master->receive_cb(master->cb_data);

        // 实际的EoE处理
        ec_lock_down(&master->master_sem);
        sth_to_send = 0;
        list_for_each_entry(eoe, &master->eoe_handlers, list)
        {
            if (eoe->slave &&
                ((eoe->slave->current_state == EC_SLAVE_STATE_PREOP) ||
                 (eoe->slave->current_state == EC_SLAVE_STATE_SAFEOP) ||
                 (eoe->slave->current_state == EC_SLAVE_STATE_OP)))
            {
                ec_eoe_run(eoe);
                if (eoe->queue_datagram)
                {
                    sth_to_send = 1;
                }
                if (!ec_eoe_is_idle(eoe))
                {
                    all_idle = 0;
                }
            }
        }
        ec_lock_up(&master->master_sem);

        if (sth_to_send)
        {
            ec_lock_down(&master->master_sem);
            list_for_each_entry(eoe, &master->eoe_handlers, list)
            {
                ec_eoe_queue(eoe);
            }
            ec_lock_up(&master->master_sem);

            // 尝试发送数据报文
            master->send_cb(master->cb_data);
        }

    schedule:
        if (all_idle)
        {
            set_current_state(TASK_INTERRUPTIBLE);
            schedule_timeout(1);
        }
        else
        {
            schedule();
        }
    }

    EC_MASTER_DBG(master, 1, "EoE线程退出...\n");
    return 0;
}

#endif

/*****************************************************************************/

/** 将从站配置附加到从站。
 * @param master EtherCAT主站。
 */
void ec_master_attach_slave_configs(
    ec_master_t *master /**< EtherCAT主站。 */
)
{
    ec_slave_config_t *sc;

    list_for_each_entry(sc, &master->configs, list)
    {
        ec_slave_config_attach(sc);
    }
}

/*****************************************************************************/

/** 中止未附加从站的从站配置的活动请求。
 * @param master EtherCAT主站。
 */
void ec_master_expire_slave_config_requests(
    ec_master_t *master /**< EtherCAT主站。 */
)
{
    ec_slave_config_t *sc;

    list_for_each_entry(sc, &master->configs, list)
    {
        ec_slave_config_expire_disconnected_requests(sc);
    }
}


/*****************************************************************************/

/**
 * @brief 通用的ec_master_find_slave()和ec_master_find_slave_const()的实现。
 * @param alias 从站别名
 * @param position 从站位置
 * @return 搜索结果，如果找不到则返回NULL。
 */
#define EC_FIND_SLAVE                                            \
    do                                                           \
    {                                                            \
        if (alias)                                               \
        {                                                        \
            for (; slave < master->slaves + master->slave_count; \
                 slave++)                                        \
            {                                                    \
                if (slave->effective_alias == alias)             \
                    break;                                       \
            }                                                    \
            if (slave == master->slaves + master->slave_count)   \
                return NULL;                                     \
        }                                                        \
                                                                 \
        slave += position;                                       \
        if (slave < master->slaves + master->slave_count)        \
        {                                                        \
            return slave;                                        \
        }                                                        \
        else                                                     \
        {                                                        \
            return NULL;                                         \
        }                                                        \
    } while (0)

/**
 * @brief 根据别名和位置在总线上查找从站。
 * @param master EtherCAT主站
 * @param alias 从站别名
 * @param position 从站位置
 * @return 搜索结果，如果找不到则返回NULL。
 */
ec_slave_t *ec_master_find_slave(
    ec_master_t *master, /**< EtherCAT主站 */
    uint16_t alias,      /**< 从站别名 */
    uint16_t position    /**< 从站位置 */
)
{
    ec_slave_t *slave = master->slaves;
    EC_FIND_SLAVE;
}

/**
 * @brief 根据别名和位置在总线上查找从站。
 * @param master EtherCAT主站
 * @param alias 从站别名
 * @param position 从站位置
 * @return 搜索结果，如果找不到则返回NULL。
 */
const ec_slave_t *ec_master_find_slave_const(
    const ec_master_t *master, /**< EtherCAT主站 */
    uint16_t alias,            /**< 从站别名 */
    uint16_t position          /**< 从站位置 */
)
{
    const ec_slave_t *slave = master->slaves;
    EC_FIND_SLAVE;
}


/*****************************************************************************/

/**
 * @brief 获取应用程序提供的从站配置数量。
 * @param master EtherCAT主站。
 * @return 配置数量。
 */
unsigned int ec_master_config_count(
    const ec_master_t *master /**< EtherCAT主站 */
)
{
    const ec_slave_config_t *sc;
    unsigned int count = 0;

    list_for_each_entry(sc, &master->configs, list)
    {
        count++;
    }

    return count;
}

/*****************************************************************************/

/**
 * @brief 通用的ec_master_get_config()和ec_master_get_config_const()的实现。
 * @param pos 列表位置。
 * @return 从站配置或NULL。
 */
#define EC_FIND_CONFIG                                  \
    do                                                  \
    {                                                   \
        list_for_each_entry(sc, &master->configs, list) \
        {                                               \
            if (pos--)                                  \
                continue;                               \
            return sc;                                  \
        }                                               \
        return NULL;                                    \
    } while (0)

/**
 * @brief 根据列表中的位置获取从站配置。
 * @param master EtherCAT主站。
 * @param pos 列表位置。
 * @return 从站配置或NULL。
 */
ec_slave_config_t *ec_master_get_config(
    const ec_master_t *master, /**< EtherCAT主站 */
    unsigned int pos           /**< 列表位置 */
)
{
    ec_slave_config_t *sc;
    EC_FIND_CONFIG;
}

/**
 * @brief 根据列表中的位置获取从站配置。
 * @param master EtherCAT主站。
 * @param pos 列表位置。
 * @return 从站配置或NULL。
 */
const ec_slave_config_t *ec_master_get_config_const(
    const ec_master_t *master, /**< EtherCAT主站 */
    unsigned int pos           /**< 列表位置 */
)
{
    const ec_slave_config_t *sc;
    EC_FIND_CONFIG;
}


/*****************************************************************************/

/**
 * @brief 获取域的数量。
 * @param master EtherCAT主站。
 * @return 域的数量。
 */
unsigned int ec_master_domain_count(
    const ec_master_t *master /**< EtherCAT主站 */
)
{
    const ec_domain_t *domain;
    unsigned int count = 0;

    list_for_each_entry(domain, &master->domains, list)
    {
        count++;
    }

    return count;
}

/*****************************************************************************/

/**
 * @brief 通用的ec_master_find_domain()和ec_master_find_domain_const()的实现。
 * @param index 域索引。
 * @return 域指针，如果找不到则返回NULL。
 */
#define EC_FIND_DOMAIN                                      \
    do                                                      \
    {                                                       \
        list_for_each_entry(domain, &master->domains, list) \
        {                                                   \
            if (index--)                                    \
                continue;                                   \
            return domain;                                  \
        }                                                   \
                                                            \
        return NULL;                                        \
    } while (0)

/**
 * @brief 根据列表中的位置获取域。
 * @param master EtherCAT主站。
 * @param index 域索引。
 * @return 域指针，如果找不到则返回NULL。
 */
ec_domain_t *ec_master_find_domain(
    ec_master_t *master, /**< EtherCAT主站 */
    unsigned int index   /**< 域索引 */
)
{
    ec_domain_t *domain;
    EC_FIND_DOMAIN;
}

/**
 * @brief 根据列表中的位置获取域。
 * @param master EtherCAT主站。
 * @param index 域索引。
 * @return 域指针，如果找不到则返回NULL。
 */
const ec_domain_t *ec_master_find_domain_const(
    const ec_master_t *master, /**< EtherCAT主站 */
    unsigned int index         /**< 域索引 */
)
{
    const ec_domain_t *domain;
    EC_FIND_DOMAIN;
}


/*****************************************************************************/

#ifdef EC_EOE

/**
 * @brief 获取EoE处理程序的数量。
 * @param master EtherCAT主站。
 * @return EoE处理程序的数量。
 */
uint16_t ec_master_eoe_handler_count(
    const ec_master_t *master /**< EtherCAT主站 */
)
{
    const ec_eoe_t *eoe;
    unsigned int count = 0;

    list_for_each_entry(eoe, &master->eoe_handlers, list)
    {
        count++;
    }

    return count;
}

/*****************************************************************************/

/**
 * @brief 根据列表中的位置获取EoE处理程序。
 * @param master EtherCAT主站。
 * @param index EoE处理程序索引。
 * @return EoE处理程序指针，如果找不到则返回NULL。
 */
const ec_eoe_t *ec_master_get_eoe_handler_const(
    const ec_master_t *master, /**< EtherCAT主站 */
    uint16_t index             /**< EoE处理程序索引 */
)
{
    const ec_eoe_t *eoe;

    list_for_each_entry(eoe, &master->eoe_handlers, list)
    {
        if (index--)
            continue;
        return eoe;
    }

    return NULL;
}

#endif

/*****************************************************************************/

/**
 * @brief 设置调试级别。
 * @param master EtherCAT主站。
 * @param level 调试级别。可以是0、1或2。
 * @retval 0 成功。
 * @retval -EINVAL 无效的调试级别。
 */
int ec_master_debug_level(
    ec_master_t *master, /**< EtherCAT主站 */
    unsigned int level   /**< 调试级别 */
)
{
    if (level > 2)
    {
        EC_MASTER_ERR(master, "无效的调试级别 %u！\n", level);
        return -EINVAL;
    }

    if (level != master->debug_level)
    {
        master->debug_level = level;
        EC_MASTER_INFO(master, "主站调试级别设置为 %u。\n",
                       master->debug_level);
    }

    return 0;
}
/*****************************************************************************/

/**
 * @brief 查找DC参考时钟。
 * @param master EtherCAT主站。
 */
void ec_master_find_dc_ref_clock(
    ec_master_t *master /**< EtherCAT主站 */
)
{
    ec_slave_t *slave, *ref = NULL;

    if (master->dc_ref_config)
    {
        // 检查应用程序选择的参考时钟
        slave = master->dc_ref_config->slave;

        if (slave)
        {
            if (slave->base_dc_supported && slave->has_dc_system_time)
            {
                ref = slave;
                EC_MASTER_INFO(master, "使用从站 %s-%u 作为应用程序选择的"
                                       " DC参考时钟。\n",
                               ec_device_names[slave->device_index != 0],
                               ref->ring_position);
            }
            else
            {
                EC_MASTER_WARN(master, "应用程序选择的从站 %s-%u 无法"
                                       " 作为DC参考时钟！",
                               ec_device_names[slave->device_index != 0],
                               slave->ring_position);
            }
        }
        else
        {
            EC_MASTER_WARN(master, "应用程序选择的DC参考时钟配置 (%u-%u) 未连接从站！\n",
                           master->dc_ref_config->alias,
                           master->dc_ref_config->position);
        }
    }

    if (!ref)
    {
        // 使用第一个支持DC的从站作为参考时钟
        for (slave = master->slaves;
             slave < master->slaves + master->slave_count;
             slave++)
        {
            if (slave->base_dc_supported && slave->has_dc_system_time)
            {
                ref = slave;
                break;
            }
        }
    }

    master->dc_ref_clock = ref;

    if (ref)
    {
        EC_MASTER_INFO(master, "使用从站 %s-%u 作为DC参考时钟。\n",
                       ec_device_names[ref->device_index != 0], ref->ring_position);
    }
    else
    {
        EC_MASTER_INFO(master, "未找到DC参考时钟。\n");
    }

    // 这些调用总是成功的，因为数据报已经预分配。
    ec_datagram_fpwr(&master->ref_sync_datagram,
                     ref ? ref->station_address : 0xffff, 0x0910, 4);
    ec_datagram_frmw(&master->sync_datagram,
                     ref ? ref->station_address : 0xffff, 0x0910, 4);
    ec_datagram_fprd(&master->sync64_datagram,
                     ref ? ref->station_address : 0xffff, 0x0910, 8);
}


/*****************************************************************************/

/**
 * @brief 计算总线拓扑结构的递归函数。
 * @param master EtherCAT主站。
 * @param upstream_slave 上游端口的从站。
 * @param slave_position 从站位置。
 * @return 成功返回0，否则返回负数错误代码。
 */
int ec_master_calc_topology_rec(
    ec_master_t *master,         /**< EtherCAT主站 */
    ec_slave_t *upstream_slave,  /**< 上游端口的从站 */
    unsigned int *slave_position /**< 从站位置 */
)
{
    ec_slave_t *slave = master->slaves + *slave_position;
    unsigned int port_index;
    int ret;

    static const unsigned int next_table[EC_MAX_PORTS] = {
        3, 2, 0, 1};

    slave->ports[slave->upstream_port].next_slave = upstream_slave;

    port_index = next_table[slave->upstream_port];
    while (port_index != slave->upstream_port)
    {
        if (!slave->ports[port_index].link.loop_closed)
        {
            *slave_position = *slave_position + 1;
            if (*slave_position < master->slave_count)
            {
                slave->ports[port_index].next_slave =
                    master->slaves + *slave_position;
                ret = ec_master_calc_topology_rec(master,
                                                  slave, slave_position);
                if (ret)
                {
                    return ret;
                }
            }
            else
            {
                return -1;
            }
        }

        port_index = next_table[port_index];
    }

    return 0;
}


/*****************************************************************************/

/**
 * @brief 计算总线拓扑结构。
 * @param master 主站。
 */
void ec_master_calc_topology(
    ec_master_t *master /**< EtherCAT主站 */
)
{
    unsigned int slave_position = 0;
    ec_slave_t *slave;

    if (master->slave_count == 0)
        return;

    for (slave = master->slaves;
         slave < master->slaves + master->slave_count;
         slave++)
    {
        ec_slave_calc_upstream_port(slave);
    }

    if (ec_master_calc_topology_rec(master, NULL, &slave_position))
        EC_MASTER_ERR(master, "Failed to calculate bus topology.\n");
}

/*****************************************************************************/

/**
 * @brief 计算总线传输延迟。
 * @param master 主站。
 */
void ec_master_calc_transmission_delays(
    ec_master_t *master /**< EtherCAT主站 */
)
{
    ec_slave_t *slave;

    for (slave = master->slaves;
         slave < master->slaves + master->slave_count;
         slave++)
    {
        ec_slave_calc_port_delays(slave);
    }

    if (master->dc_ref_clock)
    {
        uint32_t delay = 0;
        ec_slave_calc_transmission_delays_rec(master->dc_ref_clock, &delay);
    }
}


/*****************************************************************************/


/**
 * @brief 分布式时钟计算。
 * @param master 主站。
 */
void ec_master_calc_dc(
    ec_master_t *master /**< EtherCAT主站 */
)
{
    // 查找DC参考时钟
    ec_master_find_dc_ref_clock(master);

    // 计算总线拓扑结构
    ec_master_calc_topology(master);

    // 计算总线传输延迟
    ec_master_calc_transmission_delays(master);
}

/*****************************************************************************/

/**
 * @brief 为配置的从站请求OP状态。
 * @param master 主站。
 */
void ec_master_request_op(
    ec_master_t *master /**< EtherCAT主站 */
)
{
    unsigned int i;
    ec_slave_t *slave;

    if (!master->active)
        return;

    EC_MASTER_DBG(master, 1, "请求OP状态...\n");

    // 为所有配置的从站请求OP状态
    for (i = 0; i < master->slave_count; i++)
    {
        slave = master->slaves + i;
        if (slave->config)
        {
            ec_slave_request_state(slave, EC_SLAVE_STATE_OP);
        }
    }

#ifdef EC_REFCLKOP
    // 总是将DC参考时钟设置为OP状态
    if (master->dc_ref_clock)
    {
        ec_slave_request_state(master->dc_ref_clock, EC_SLAVE_STATE_OP);
    }
#endif
}
```

/*****************************************************************************/

/**
 * @brief 上传从站字典到EtherCAT主站。
 *
 * 该函数将从站字典上传到EtherCAT主站。
 *
 * @param master EtherCAT主站对象指针。
 * @param slave_position 从站位置。
 * @return 成功返回0，失败返回负数错误码。
 */
int ec_master_dict_upload(ec_master_t *master, uint16_t slave_position)
{
    ec_dict_request_t request;
    ec_slave_t *slave;
    int ret = 0;

    EC_MASTER_DBG(master, 1, "%s(master = 0x%p, slave_position = %u\n",
                  __func__, master, slave_position);

    ec_dict_request_init(&request);
    ec_dict_request_read(&request);

    if (ec_lock_down_interruptible(&master->master_sem))
    {
        return -EINTR;
    }

    if (!(slave = ec_master_find_slave(master, 0, slave_position)))
    {
        ec_lock_up(&master->master_sem);
        EC_MASTER_ERR(master, "从站 %u 不存在！\n", slave_position);
        return -EINVAL;
    }

    EC_SLAVE_DBG(slave, 1, "调度字典上传请求。\n");

    // 调度请求。
    list_add_tail(&request.list, &slave->dict_requests);

    ec_lock_up(&master->master_sem);

    // 等待FSM处理
    if (wait_event_interruptible(master->request_queue,
                                 request.state != EC_INT_REQUEST_QUEUED))
    {
        // 被信号中断
        ec_lock_down(&master->master_sem);
        if (request.state == EC_INT_REQUEST_QUEUED)
        {
            list_del(&request.list);
            ec_lock_up(&master->master_sem);
            return -EINTR;
        }
        // 请求已在处理中：无法中断。
        ec_lock_up(&master->master_sem);
    }

    // 等待主站FSM处理完成
    wait_event(master->request_queue, request.state != EC_INT_REQUEST_BUSY);

    if (request.state != EC_INT_REQUEST_SUCCESS)
    {
        ret = -EIO;
    }
    return ret;
}


/******************************************************************************
 *  Application interface
 *****************************************************************************/

/**
 * @brief 创建EtherCAT主站域（Domain）。
 *
 * 该函数创建一个新的EtherCAT主站域（Domain）。
 *
 * @param master EtherCAT主站对象指针。
 * @return 成功返回新的域（Domain）指针，失败返回错误指针（ERR_PTR()）。
 */
ec_domain_t *ecrt_master_create_domain_err(
    ec_master_t *master /**< 主站 */
)
{
    ec_domain_t *domain, *last_domain;
    unsigned int index;

    EC_MASTER_DBG(master, 1, "ecrt_master_create_domain(master = 0x%p)\n",
                  master);

    if (!(domain =
              (ec_domain_t *)kmalloc(sizeof(ec_domain_t), GFP_KERNEL)))
    {
        EC_MASTER_ERR(master, "分配域（Domain）内存时发生错误！\n");
        return ERR_PTR(-ENOMEM);
    }

    ec_lock_down(&master->master_sem);

    if (list_empty(&master->domains))
    {
        index = 0;
    }
    else
    {
        last_domain = list_entry(master->domains.prev, ec_domain_t, list);
        index = last_domain->index + 1;
    }

    ec_domain_init(domain, master, index);
    list_add_tail(&domain->list, &master->domains);

    ec_lock_up(&master->master_sem);

    EC_MASTER_DBG(master, 1, "创建域（Domain）%u。\n", domain->index);

    return domain;
}


/*****************************************************************************/

/**
 * @brief 创建EtherCAT主站域（Domain）。
 *
 * 该函数创建一个新的EtherCAT主站域（Domain）。
 *
 * @param master EtherCAT主站对象指针。
 * @return 成功返回新的域（Domain）指针，失败返回NULL。
 */
ec_domain_t *ecrt_master_create_domain(
    ec_master_t *master /**< 主站 */
)
{
    ec_domain_t *d = ecrt_master_create_domain_err(master);
    return IS_ERR(d) ? NULL : d;
}

/*****************************************************************************/

/**
 * @brief 设置EtherCAT主站域（Domain）的内存。
 *
 * 目前不支持此功能。
 *
 * @param master EtherCAT主站对象指针。
 * @return 返回错误码 -ENOMEM。
 */
int ecrt_master_setup_domain_memory(ec_master_t *master)
{
    // 目前不支持
    return -ENOMEM; // FIXME
}


/*****************************************************************************/

/**
 * @brief 激活EtherCAT主站。
 *
 * 该函数激活EtherCAT主站，使其进入操作状态。
 *
 * @param master EtherCAT主站对象指针。
 * @return 成功返回0，失败返回错误码。
 */
int ecrt_master_activate(ec_master_t *master)
{
    uint32_t domain_offset;
    ec_domain_t *domain;
    int ret;
#ifdef EC_EOE
    int eoe_was_running;
#endif

    EC_MASTER_DBG(master, 1, "ecrt_master_activate(master = 0x%p)\n", master);

    if (master->active)
    {
        EC_MASTER_WARN(master, "%s: 主站已经激活！\n", __func__);
        return 0;
    }

    ec_lock_down(&master->master_sem);

    // 完成所有域（Domain）
    domain_offset = 0;
    list_for_each_entry(domain, &master->domains, list)
    {
        ret = ec_domain_finish(domain, domain_offset);
        if (ret < 0)
        {
            ec_lock_up(&master->master_sem);
            EC_MASTER_ERR(master, "无法完成域（Domain）0x%p！\n", domain);
            return ret;
        }
        domain_offset += domain->data_size;
    }

    ec_lock_up(&master->master_sem);

    // 重新启动EoE进程和主线程，并使用新的锁定机制

    ec_master_thread_stop(master);
#ifdef EC_EOE
    eoe_was_running = (master->eoe_thread != NULL);
    ec_master_eoe_stop(master);
#endif

    EC_MASTER_DBG(master, 1, "FSM数据报文为 %p。\n", &master->fsm_datagram);

    master->injection_seq_fsm = 0;
    master->injection_seq_rt = 0;

    master->send_cb = master->app_send_cb;
    master->receive_cb = master->app_receive_cb;
    master->cb_data = master->app_cb_data;

#ifdef EC_EOE
    if (eoe_was_running)
    {
        ec_master_eoe_start(master);
    }
#endif
    ret = ec_master_thread_start(master, ec_master_operation_thread,
                                 "EtherCAT-OP");
    if (ret < 0)
    {
        EC_MASTER_ERR(master, "无法启动主线程！\n");
        return ret;
    }

    /* 允许在拓扑变化后进行扫描。 */
    master->allow_scan = 1;

    master->active = 1;

    // 通知状态机，现在应用配置
    master->config_changed = 1;
    master->dc_offset_valid = 0;

    return 0;
}

/*****************************************************************************/

/**
 * @brief 停用所有从站。
 *
 * 该函数停用所有从站，将其状态设置为PREOP，并进行重新配置。
 *
 * @param master EtherCAT主站对象指针。
 */
void ecrt_master_deactivate_slaves(ec_master_t *master)
{
    ec_slave_t *slave;
    ec_slave_config_t *sc, *next;

    EC_MASTER_DBG(master, 1, "%s(master = 0x%p)\n", __func__, master);

    if (!master->active)
    {
        EC_MASTER_WARN(master, "%s: 主站未激活。\n", __func__);
        return;
    }

    // 清除所有从站的DC设置
    list_for_each_entry_safe(sc, next, &master->configs, list)
    {
        if (sc->dc_assign_activate)
        {
            ecrt_slave_config_dc(sc, 0x0000, 0, 0, 0, 0);
        }
    }

    for (slave = master->slaves;
         slave < master->slaves + master->slave_count;
         slave++)
    {

        // 设置所有从站的状态为PREOP
        ec_slave_request_state(slave, EC_SLAVE_STATE_PREOP);

        // 标记为需要重新配置，因为主站在两个连续操作阶段之间可能没有重新配置的机会。
        slave->force_config = 1;
    }
}

/*****************************************************************************/

/**
 * @brief 停用EtherCAT主站。
 *
 * 该函数停用EtherCAT主站，将其状态设置为非激活状态，并进行必要的清理和重置。
 *
 * @param master EtherCAT主站对象指针。
 */
void ecrt_master_deactivate(ec_master_t *master)
{
    ec_slave_t *slave;

    EC_MASTER_DBG(master, 1, "%s(master = 0x%p)\n", __func__, master);

    if (!master->active)
    {
        EC_MASTER_WARN(master, "%s: 主站未激活。\n", __func__);
        ec_master_clear_config(master);
        return;
    }

    ec_master_thread_stop(master);
#ifdef EC_EOE
    ec_master_eoe_stop(master);
#endif

    master->send_cb = ec_master_internal_send_cb;
    master->receive_cb = ec_master_internal_receive_cb;
    master->cb_data = master;

    ec_master_clear_config(master);

    for (slave = master->slaves;
         slave < master->slaves + master->slave_count;
         slave++)
    {

        // 设置所有从站的状态为PREOP
        ec_slave_request_state(slave, EC_SLAVE_STATE_PREOP);

        // 清除read_mbox_busy标志，以防止从站的CoE状态机中断
        ec_read_mbox_lock_clear(slave);

        // 标记为需要重新配置，因为主站在两个连续操作阶段之间可能没有重新配置的机会。
        slave->force_config = 1;
    }

    master->app_time = 0ULL;
    master->dc_ref_time = 0ULL;
    master->dc_offset_valid = 0;

    /* 禁止扫描，以达到与主站请求后（在调用ec_master_enter_operation_phase()之后）相同的状态。 */
    master->allow_scan = 0;

    master->active = 0;

#ifdef EC_EOE
    ec_master_eoe_start(master);
#endif
    if (ec_master_thread_start(master, ec_master_idle_thread,
                               "EtherCAT-IDLE"))
    {
        EC_MASTER_WARN(master, "无法重新启动主站线程！\n");
    }
}


/*****************************************************************************/

/**
 * @brief 发送EtherCAT主站的数据报文。
 *
 * 该函数发送EtherCAT主站的数据报文，并返回发送的字节数。
 *
 * @param master EtherCAT主站对象指针。
 * @return 返回发送的字节数。
 */
size_t ecrt_master_send(ec_master_t *master)
{
    ec_datagram_t *datagram, *n;
    ec_device_index_t dev_idx;
    size_t sent_bytes = 0;

    if (master->injection_seq_rt != master->injection_seq_fsm)
    {
        // 注入主站FSM生成的数据报文
        ec_master_queue_datagram(master, &master->fsm_datagram);
        master->injection_seq_rt = master->injection_seq_fsm;
    }

    ec_master_inject_external_datagrams(master);

    for (dev_idx = EC_DEVICE_MAIN; dev_idx < ec_master_num_devices(master);
         dev_idx++)
    {
        if (unlikely(!master->devices[dev_idx].link_state))
        {
            // 链路断开，无法发送数据报文
            list_for_each_entry_safe(datagram, n,
                                     &master->datagram_queue, queue)
            {
                if (datagram->device_index == dev_idx)
                {
                    datagram->state = EC_DATAGRAM_ERROR;
                    list_del_init(&datagram->queue);
                }
            }

            if (!master->devices[dev_idx].dev)
            {
                continue;
            }

            // 查询链路状态
            ec_device_poll(&master->devices[dev_idx]);

            // 清除帧统计信息
            ec_device_clear_stats(&master->devices[dev_idx]);
            continue;
        }

        // 发送帧
        sent_bytes = max(sent_bytes,
                         ec_master_send_datagrams(master, dev_idx));
    }

    return sent_bytes;
}

/*****************************************************************************/

/**
 * @brief 接收EtherCAT主站的数据报文。
 *
 * 该函数接收EtherCAT主站的数据报文，并进行必要的处理。
 *
 * @param master EtherCAT主站对象指针。
 */
void ecrt_master_receive(ec_master_t *master)
{
    unsigned int dev_idx;
    ec_datagram_t *datagram, *next;

    // 接收数据报文
    for (dev_idx = EC_DEVICE_MAIN; dev_idx < ec_master_num_devices(master);
         dev_idx++)
    {
        ec_device_poll(&master->devices[dev_idx]);
    }
    ec_master_update_device_stats(master);

    // 移除所有超时的数据报文
    list_for_each_entry_safe(datagram, next, &master->datagram_queue, queue)
    {
        if (datagram->state != EC_DATAGRAM_SENT)
            continue;

#ifdef EC_HAVE_CYCLES
        if (master->devices[EC_DEVICE_MAIN].cycles_poll -
                datagram->cycles_sent >
            timeout_cycles)
        {
#else
        if (master->devices[EC_DEVICE_MAIN].jiffies_poll -
                datagram->jiffies_sent >
            timeout_jiffies)
        {
#endif
            list_del_init(&datagram->queue);
            datagram->state = EC_DATAGRAM_TIMED_OUT;
            master->stats.timeouts++;

#ifdef EC_RT_SYSLOG
            ec_master_output_stats(master);

            if (unlikely(master->debug_level > 0))
            {
                unsigned int time_us;
#ifdef EC_HAVE_CYCLES
                time_us = (unsigned int)(master->devices[EC_DEVICE_MAIN].cycles_poll -
                                         datagram->cycles_sent) *
                          1000 / cpu_khz;
#else
                time_us = (unsigned int)((master->devices[EC_DEVICE_MAIN].jiffies_poll -
                                          datagram->jiffies_sent) *
                                         1000000 / HZ);
#endif
                EC_MASTER_DBG(master, 0, "超时的数据报文 %p，索引 %02X，等待时间 %u 微秒。\n",
                              datagram, datagram->index, time_us);
            }
#endif /* RT_SYSLOG */
        }
    }
}

/*****************************************************************************/

/**
 * @brief 发送扩展的EtherCAT主站数据报文。
 *
 * 该函数发送扩展的EtherCAT主站数据报文，并返回发送的字节数。
 *
 * @param master EtherCAT主站对象指针。
 * @return 返回发送的字节数。
 */
size_t ecrt_master_send_ext(ec_master_t *master)
{
    ec_datagram_t *datagram, *next;

    ec_lock_down(&master->ext_queue_sem);

    list_for_each_entry_safe(datagram, next, &master->ext_datagram_queue,
                             queue)
    {
        list_del(&datagram->queue);
        ec_master_queue_datagram(master, datagram);
    }
    ec_lock_up(&master->ext_queue_sem);

    return ecrt_master_send(master);
}

/*****************************************************************************/

/**
 * @brief ecrt_master_slave_config_err()函数的作用与ecrt_master_slave_config()相同，但返回值为ERR_PTR()。
 *
 * @param master EtherCAT主站对象指针。
 * @param alias 从站的别名。
 * @param position 从站的位置。
 * @param vendor_id 从站的厂商ID。
 * @param product_code 从站的产品代码。
 * @return 返回指向ec_slave_config_t结构体的指针，表示从站的配置；如果出现错误，则返回ERR_PTR()。
 */
ec_slave_config_t *ecrt_master_slave_config_err(ec_master_t *master,
                                                uint16_t alias, uint16_t position, uint32_t vendor_id,
                                                uint32_t product_code)
{
    ec_slave_config_t *sc;
    unsigned int found = 0;

    EC_MASTER_DBG(master, 1, "ecrt_master_slave_config(master = 0x%p,"
                             " alias = %u, position = %u, vendor_id = 0x%08x,"
                             " product_code = 0x%08x)\n",
                  master, alias, position, vendor_id, product_code);

    list_for_each_entry(sc, &master->configs, list)
    {
        if (sc->alias == alias && sc->position == position)
        {
            found = 1;
            break;
        }
    }

    if (found)
    { // 已存在相同alias/position的配置
        if (sc->vendor_id != vendor_id || sc->product_code != product_code)
        {
            EC_MASTER_ERR(master, "从站类型不匹配。之前已配置为0x%08X/0x%08X。现在配置为0x%08X/0x%08X。\n",
                          sc->vendor_id, sc->product_code,
                          vendor_id, product_code);
            return ERR_PTR(-ENOENT);
        }
    }
    else
    {
        EC_MASTER_DBG(master, 1, "为 %u:%u，0x%08X/0x%08X 创建从站配置。\n",
                      alias, position, vendor_id, product_code);

        if (!(sc = (ec_slave_config_t *)kmalloc(sizeof(ec_slave_config_t),
                                                GFP_KERNEL)))
        {
            EC_MASTER_ERR(master, "无法为从站配置分配内存。\n");
            return ERR_PTR(-ENOMEM);
        }

        ec_slave_config_init(sc, master,
                             alias, position, vendor_id, product_code);

        ec_lock_down(&master->master_sem);

        // 尝试找到指定的从站
        ec_slave_config_attach(sc);
        ec_slave_config_load_default_sync_config(sc);
        list_add_tail(&sc->list, &master->configs);

        ec_lock_up(&master->master_sem);
    }

    return sc;
}



/*****************************************************************************/

/**
 * @brief 根据别名、位置、厂商ID和产品代码配置EtherCAT从站。
 *
 * @param master EtherCAT主站对象指针。
 * @param alias 从站的别名。
 * @param position 从站的位置。
 * @param vendor_id 从站的厂商ID。
 * @param product_code 从站的产品代码。
 * @return 返回指向ec_slave_config_t结构体的指针，表示从站的配置；如果出现错误，则返回NULL。
 */
ec_slave_config_t *ecrt_master_slave_config(ec_master_t *master,
                                            uint16_t alias, uint16_t position, uint32_t vendor_id,
                                            uint32_t product_code)
{
    ec_slave_config_t *sc = ecrt_master_slave_config_err(master, alias,
                                                         position, vendor_id, product_code);
    return IS_ERR(sc) ? NULL : sc;
}

/*****************************************************************************/

/**
 * @brief 选择EtherCAT主站的参考时钟配置。
 *
 * @param master EtherCAT主站对象指针。
 * @param sc 从站配置对象指针。
 * @return 返回0表示成功，返回负值表示错误。
 */
int ecrt_master_select_reference_clock(ec_master_t *master,
                                       ec_slave_config_t *sc)
{
    master->dc_ref_config = sc;

    if (master->dc_ref_config)
    {
        EC_MASTER_INFO(master, "应用程序选择了由应用程序设置的DC参考时钟配置（%u-%u）。\n",
                       master->dc_ref_config->alias,
                       master->dc_ref_config->position);
    }
    else
    {
        EC_MASTER_INFO(master, "应用程序清除了由应用程序设置的DC参考时钟配置。\n");
    }

    // 更新DC数据报文
    ec_master_find_dc_ref_clock(master);

    return 0;
}

/*****************************************************************************/

/**
 * @brief 获取EtherCAT主站的信息。
 *
 * @param master EtherCAT主站对象指针。
 * @param master_info 用于存储主站信息的结构体指针。
 * @return 返回0表示成功，返回负值表示错误。
 */
int ecrt_master(ec_master_t *master, ec_master_info_t *master_info)
{
    EC_MASTER_DBG(master, 1, "ecrt_master(master = 0x%p,"
                             " master_info = 0x%p)\n",
                  master, master_info);

    master_info->slave_count = master->slave_count;
    master_info->link_up = master->devices[EC_DEVICE_MAIN].link_state;
    master_info->scan_busy = master->scan_busy;
    master_info->app_time = master->app_time;
    return 0;
}

/*****************************************************************************/

/**
 * @brief 获取EtherCAT主站中指定从站的信息。
 *
 * @param master EtherCAT主站对象指针。
 * @param slave_position 从站的位置。
 * @param slave_info 用于存储从站信息的结构体指针。
 * @return 返回0表示成功，返回负值表示错误。
 */
int ecrt_master_get_slave(ec_master_t *master, uint16_t slave_position,
                          ec_slave_info_t *slave_info)
{
    const ec_slave_t *slave;
    unsigned int i;
    int ret = 0;

    if (ec_lock_down_interruptible(&master->master_sem))
    {
        return -EINTR;
    }

    slave = ec_master_find_slave_const(master, 0, slave_position);

    if (slave == NULL)
    {
        ret = -ENOENT;
        goto out_get_slave;
    }

    if (slave->sii_image == NULL)
    {
        EC_MASTER_WARN(master, "无法访问从站位置 %s-%u 的SII数据",
                       ec_device_names[slave->device_index != 0], slave->ring_position);
        ret = -ENOENT;
        goto out_get_slave;
    }

    slave_info->position = slave->ring_position;
    slave_info->vendor_id = slave->sii_image->sii.vendor_id;
    slave_info->product_code = slave->sii_image->sii.product_code;
    slave_info->revision_number = slave->sii_image->sii.revision_number;
    slave_info->serial_number = slave->sii_image->sii.serial_number;
    slave_info->alias = slave->effective_alias;
    slave_info->current_on_ebus = slave->sii_image->sii.current_on_ebus;

    for (i = 0; i < EC_MAX_PORTS; i++)
    {
        slave_info->ports[i].desc = slave->ports[i].desc;
        slave_info->ports[i].link.link_up = slave->ports[i].link.link_up;
        slave_info->ports[i].link.loop_closed =
            slave->ports[i].link.loop_closed;
        slave_info->ports[i].link.signal_detected =
            slave->ports[i].link.signal_detected;
        slave_info->ports[i].receive_time = slave->ports[i].receive_time;
        if (slave->ports[i].next_slave)
        {
            slave_info->ports[i].next_slave =
                slave->ports[i].next_slave->ring_position;
        }
        else
        {
            slave_info->ports[i].next_slave = 0xffff;
        }
        slave_info->ports[i].delay_to_next_dc =
            slave->ports[i].delay_to_next_dc;
    }
    slave_info->upstream_port = slave->upstream_port;

    slave_info->al_state = slave->current_state;
    slave_info->error_flag = slave->error_flag;
    slave_info->scan_required = slave->scan_required;
    slave_info->ready = ec_fsm_slave_is_ready(&slave->fsm);
    slave_info->sync_count = slave->sii_image->sii.sync_count;
    slave_info->sdo_count = ec_slave_sdo_count(slave);
    if (slave->sii_image->sii.name)
    {
        strncpy(slave_info->name, slave->sii_image->sii.name, EC_MAX_STRING_LENGTH);
    }
    else
    {
        slave_info->name[0] = 0;
    }

out_get_slave:
    ec_lock_up(&master->master_sem);

    return ret;
}


/*****************************************************************************/

/**
 * @brief 设置EtherCAT主站的回调函数。
 *
 * @param master EtherCAT主站对象指针。
 * @param send_cb 发送回调函数指针。
 * @param receive_cb 接收回调函数指针。
 * @param cb_data 回调函数的用户数据指针。
 */
void ecrt_master_callbacks(ec_master_t *master,
                           void (*send_cb)(void *), void (*receive_cb)(void *), void *cb_data)
{
    EC_MASTER_DBG(master, 1, "ecrt_master_callbacks(master = 0x%p,"
                             " send_cb = 0x%p, receive_cb = 0x%p, cb_data = 0x%p)\n",
                  master, send_cb, receive_cb, cb_data);

    master->app_send_cb = send_cb;
    master->app_receive_cb = receive_cb;
    master->app_cb_data = cb_data;
}

/*****************************************************************************/

/**
 * @brief 获取EtherCAT主站的状态。
 *
 * @param master EtherCAT主站对象指针。
 * @param state 用于存储主站状态的结构体指针。
 */
void ecrt_master_state(const ec_master_t *master, ec_master_state_t *state)
{
    ec_device_index_t dev_idx;

    state->slaves_responding = 0U;
    state->al_states = 0;
    state->link_up = 0U;
    state->scan_busy = master->scan_busy ? 1U : 0U;

    for (dev_idx = EC_DEVICE_MAIN; dev_idx < ec_master_num_devices(master);
         dev_idx++)
    {
        /* 所有链路上响应的从站数量之和 */
        state->slaves_responding += master->fsm.slaves_responding[dev_idx];

        /* 所有链路上的从站状态的二进制或结果 */
        state->al_states |= master->fsm.slave_states[dev_idx];

        /* 如果至少有一个设备具有链路，则设置链路状态为真 */
        state->link_up |= master->devices[dev_idx].link_state;
    }
}

/*****************************************************************************/

/**
 * @brief 获取EtherCAT主站指定设备索引的链路状态。
 *
 * @param master EtherCAT主站对象指针。
 * @param dev_idx 设备索引。
 * @param state 用于存储链路状态的结构体指针。
 * @return 返回0表示成功，返回负值表示错误。
 */
int ecrt_master_link_state(const ec_master_t *master, unsigned int dev_idx,
                           ec_master_link_state_t *state)
{
    if (dev_idx >= ec_master_num_devices(master))
    {
        return -EINVAL;
    }

    state->slaves_responding = master->fsm.slaves_responding[dev_idx];
    state->al_states = master->fsm.slave_states[dev_idx];
    state->link_up = master->devices[dev_idx].link_state;

    return 0;
}


/*****************************************************************************/

/**
 * @brief 设置EtherCAT主站的应用时间。
 *
 * @param master EtherCAT主站对象指针。
 * @param app_time 应用时间。
 */
void ecrt_master_application_time(ec_master_t *master, uint64_t app_time)
{
    master->app_time = app_time;

    if (unlikely(!master->dc_ref_time))
    {
        master->dc_ref_time = app_time;
    }
}

/*****************************************************************************/

/**
 * @brief 获取EtherCAT主站参考时钟时间。
 *
 * @param master EtherCAT主站对象指针。
 * @param time 用于存储参考时钟时间的指针。
 * @return 返回0表示成功，返回负值表示错误。
 */
int ecrt_master_reference_clock_time(ec_master_t *master, uint32_t *time)
{
    if (!master->dc_ref_clock)
    {
        return -ENXIO;
    }

    if (master->sync_datagram.state != EC_DATAGRAM_RECEIVED)
    {
        return -EIO;
    }

    if (!master->dc_offset_valid)
    {
        return -EAGAIN;
    }

    // 获取返回的数据报时间，排除传输延迟。
    *time = EC_READ_U32(master->sync_datagram.data) -
            master->dc_ref_clock->transmission_delay;

    return 0;
}

/*****************************************************************************/

/**
 * @brief 同步EtherCAT主站的参考时钟。
 *
 * @param master EtherCAT主站对象指针。
 */
void ecrt_master_sync_reference_clock(ec_master_t *master)
{
    if (master->dc_ref_clock && master->dc_offset_valid)
    {
        EC_WRITE_U32(master->ref_sync_datagram.data, master->app_time);
        ec_master_queue_datagram(master, &master->ref_sync_datagram);
    }
}

/*****************************************************************************/

/**
 * @brief 将EtherCAT主站的参考时钟同步到指定时间。
 *
 * @param master EtherCAT主站对象指针。
 * @param sync_time 同步时间。
 */
void ecrt_master_sync_reference_clock_to(
    ec_master_t *master,
    uint64_t sync_time)
{
    if (master->dc_ref_clock)
    {
        EC_WRITE_U32(master->ref_sync_datagram.data, sync_time);
        ec_master_queue_datagram(master, &master->ref_sync_datagram);
    }
}

/*****************************************************************************/

/**
 * @brief 同步EtherCAT主站的从站时钟。
 *
 * @param master EtherCAT主站对象指针。
 */
void ecrt_master_sync_slave_clocks(ec_master_t *master)
{
    if (master->dc_ref_clock && master->dc_offset_valid)
    {
        ec_datagram_zero(&master->sync_datagram);
        ec_master_queue_datagram(master, &master->sync_datagram);
    }
}


/*****************************************************************************/

/**
 * @brief 将EtherCAT主站的64位参考时钟时间加入队列。
 *
 * @param master EtherCAT主站对象指针。
 */
void ecrt_master_64bit_reference_clock_time_queue(ec_master_t *master)
{
    if (master->dc_ref_clock && master->dc_offset_valid)
    {
        ec_datagram_zero(&master->sync64_datagram);
        ec_master_queue_datagram(master, &master->sync64_datagram);
    }
}

/*****************************************************************************/

/**
 * @brief 获取EtherCAT主站64位参考时钟时间。
 *
 * @param master EtherCAT主站对象指针。
 * @param time 用于存储64位参考时钟时间的指针。
 * @return 返回0表示成功，返回负值表示错误。
 */
int ecrt_master_64bit_reference_clock_time(ec_master_t *master, uint64_t *time)
{
    if (!master->dc_ref_clock)
    {
        return -ENXIO;
    }

    if (master->sync64_datagram.state != EC_DATAGRAM_RECEIVED)
    {
        return -EIO;
    }

    if (!master->dc_offset_valid)
    {
        return -EAGAIN;
    }

    // 获取返回的数据报时间，排除传输延迟。
    *time = EC_READ_U64(master->sync64_datagram.data) -
            master->dc_ref_clock->transmission_delay;

    return 0;
}

/*****************************************************************************/

/**
 * @brief 同步EtherCAT主站的监视器。
 *
 * @param master EtherCAT主站对象指针。
 */
void ecrt_master_sync_monitor_queue(ec_master_t *master)
{
    ec_datagram_zero(&master->sync_mon_datagram);
    ec_master_queue_datagram(master, &master->sync_mon_datagram);
}

/*****************************************************************************/

/**
 * @brief 处理EtherCAT主站的监视器同步。
 *
 * @param master EtherCAT主站对象指针。
 * @return 返回监视器同步的处理结果。
 */
uint32_t ecrt_master_sync_monitor_process(ec_master_t *master)
{
    if (master->sync_mon_datagram.state == EC_DATAGRAM_RECEIVED)
    {
        return EC_READ_U32(master->sync_mon_datagram.data) & 0x7fffffff;
    }
    else
    {
        return 0xffffffff;
    }
}


/*****************************************************************************/

/**
 * @brief 下载数据到EtherCAT主站的SDO对象。
 *
 * @param master EtherCAT主站对象指针。
 * @param slave_position 从站位置。
 * @param index SDO索引。
 * @param subindex SDO子索引。
 * @param data 数据指针。
 * @param data_size 数据大小。
 * @param abort_code 用于存储中止代码的指针。
 * @return 返回0表示成功，返回负值表示错误。
 */
int ecrt_master_sdo_download(ec_master_t *master, uint16_t slave_position,
                             uint16_t index, uint8_t subindex, const uint8_t *data,
                             size_t data_size, uint32_t *abort_code)
{
    ec_sdo_request_t request;
    ec_slave_t *slave;
    int ret;

    EC_MASTER_DBG(master, 1, "%s(master = 0x%p,"
                             " slave_position = %u, index = 0x%04X, subindex = 0x%02X,"
                             " data = 0x%p, data_size = %zu, abort_code = 0x%p)\n",
                  __func__, master, slave_position, index, subindex,
                  data, data_size, abort_code);

    if (!data_size)
    {
        EC_MASTER_ERR(master, "数据大小为零！\n");
        return -EINVAL;
    }

    ec_sdo_request_init(&request);
    ecrt_sdo_request_index(&request, index, subindex);
    ret = ec_sdo_request_alloc(&request, data_size);
    if (ret)
    {
        ec_sdo_request_clear(&request);
        return ret;
    }

    memcpy(request.data, data, data_size);
    request.data_size = data_size;
    ecrt_sdo_request_write(&request);

    if (ec_lock_down_interruptible(&master->master_sem))
    {
        ec_sdo_request_clear(&request);
        return -EINTR;
    }

    if (!(slave = ec_master_find_slave(master, 0, slave_position)))
    {
        ec_lock_up(&master->master_sem);
        EC_MASTER_ERR(master, "从站 %u 不存在！\n", slave_position);
        ec_sdo_request_clear(&request);
        return -EINVAL;
    }

    EC_SLAVE_DBG(slave, 1, "调度SDO下载请求。\n");

    // 调度请求。
    list_add_tail(&request.list, &slave->sdo_requests);

    ec_lock_up(&master->master_sem);

    // 等待FSM处理
    if (wait_event_interruptible(master->request_queue,
                                 request.state != EC_INT_REQUEST_QUEUED))
    {
        // 被信号中断
        ec_lock_down(&master->master_sem);
        if (request.state == EC_INT_REQUEST_QUEUED)
        {
            list_del(&request.list);
            ec_lock_up(&master->master_sem);
            ec_sdo_request_clear(&request);
            return -EINTR;
        }
        // 请求已在处理中：无法中断。
        ec_lock_up(&master->master_sem);
    }

    // 等待主站FSM处理完成
    wait_event(master->request_queue, request.state != EC_INT_REQUEST_BUSY);

    *abort_code = request.abort_code;

    if (request.state == EC_INT_REQUEST_SUCCESS)
    {
        ret = 0;
    }
    else if (request.errno)
    {
        ret = -request.errno;
    }
    else
    {
        ret = -EIO;
    }

    ec_sdo_request_clear(&request);
    return ret;
}

/*****************************************************************************/

/**
 * @brief 使用完整访问方式将数据下载到EtherCAT主站的SDO对象。
 *
 * @param master EtherCAT主站对象指针。
 * @param slave_position 从站位置。
 * @param index SDO索引。
 * @param data 数据指针。
 * @param data_size 数据大小。
 * @param abort_code 用于存储中止代码的指针。
 * @return 返回0表示成功，返回负值表示错误。
 */
int ecrt_master_sdo_download_complete(ec_master_t *master,
                                      uint16_t slave_position, uint16_t index, const uint8_t *data,
                                      size_t data_size, uint32_t *abort_code)
{
    ec_sdo_request_t request;
    ec_slave_t *slave;
    int ret;

    EC_MASTER_DBG(master, 1, "%s(master = 0x%p,"
                             " slave_position = %u, index = 0x%04X,"
                             " data = 0x%p, data_size = %zu, abort_code = 0x%p)\n",
                  __func__, master, slave_position, index, data, data_size,
                  abort_code);

    if (!data_size)
    {
        EC_MASTER_ERR(master, "数据大小为零！\n");
        return -EINVAL;
    }

    ec_sdo_request_init(&request);
    ecrt_sdo_request_index_complete(&request, index);
    ret = ec_sdo_request_alloc(&request, data_size);
    if (ret)
    {
        ec_sdo_request_clear(&request);
        return ret;
    }

    memcpy(request.data, data, data_size);
    request.data_size = data_size;
    ecrt_sdo_request_write(&request);

    if (ec_lock_down_interruptible(&master->master_sem))
    {
        ec_sdo_request_clear(&request);
        return -EINTR;
    }

    if (!(slave = ec_master_find_slave(master, 0, slave_position)))
    {
        ec_lock_up(&master->master_sem);
        EC_MASTER_ERR(master, "从站 %u 不存在！\n", slave_position);
        ec_sdo_request_clear(&request);
        return -EINVAL;
    }

    EC_SLAVE_DBG(slave, 1, "调度SDO下载请求 (完整访问)。\n");

    // 调度请求。
    list_add_tail(&request.list, &slave->sdo_requests);

    ec_lock_up(&master->master_sem);

    // 等待FSM处理
    if (wait_event_interruptible(master->request_queue,
                                 request.state != EC_INT_REQUEST_QUEUED))
    {
        // 被信号中断
        ec_lock_down(&master->master_sem);
        if (request.state == EC_INT_REQUEST_QUEUED)
        {
            list_del(&request.list);
            ec_lock_up(&master->master_sem);
            ec_sdo_request_clear(&request);
            return -EINTR;
        }
        // 请求已在处理中：无法中断。
        ec_lock_up(&master->master_sem);
    }

    // 等待主站FSM处理完成
    wait_event(master->request_queue, request.state != EC_INT_REQUEST_BUSY);

    *abort_code = request.abort_code;

    if (request.state == EC_INT_REQUEST_SUCCESS)
    {
        ret = 0;
    }
    else if (request.errno)
    {
        ret = -request.errno;
    }
    else
    {
        ret = -EIO;
    }

    ec_sdo_request_clear(&request);
    return ret;
}

/*****************************************************************************/

/**
 * @brief 从EtherCAT主站的SDO对象上传数据。
 *
 * @param master EtherCAT主站对象指针。
 * @param slave_position 从站位置。
 * @param index SDO索引。
 * @param subindex SDO子索引。
 * @param target 目标缓冲区指针。
 * @param target_size 目标缓冲区大小。
 * @param result_size 用于存储结果大小的指针。
 * @param abort_code 用于存储中止代码的指针。
 * @return 返回0表示成功，返回负值表示错误。
 */
int ecrt_master_sdo_upload(ec_master_t *master, uint16_t slave_position,
                           uint16_t index, uint8_t subindex, uint8_t *target,
                           size_t target_size, size_t *result_size, uint32_t *abort_code)
{
    ec_sdo_request_t request;
    ec_slave_t *slave;
    int ret = 0;

    EC_MASTER_DBG(master, 1, "%s(master = 0x%p,"
                             " slave_position = %u, index = 0x%04X, subindex = 0x%02X,"
                             " target = 0x%p, target_size = %zu, result_size = 0x%p,"
                             " abort_code = 0x%p)\n",
                  __func__, master, slave_position, index, subindex,
                  target, target_size, result_size, abort_code);

    ec_sdo_request_init(&request);
    ecrt_sdo_request_index(&request, index, subindex);
    ecrt_sdo_request_read(&request);

    if (ec_lock_down_interruptible(&master->master_sem))
    {
        ec_sdo_request_clear(&request);
        return -EINTR;
    }

    if (!(slave = ec_master_find_slave(master, 0, slave_position)))
    {
        ec_lock_up(&master->master_sem);
        ec_sdo_request_clear(&request);
        EC_MASTER_ERR(master, "从站 %u 不存在！\n", slave_position);
        return -EINVAL;
    }

    EC_SLAVE_DBG(slave, 1, "调度SDO上传请求。\n");

    // 调度请求。
    list_add_tail(&request.list, &slave->sdo_requests);

    ec_lock_up(&master->master_sem);

    // 等待FSM处理
    if (wait_event_interruptible(master->request_queue,
                                 request.state != EC_INT_REQUEST_QUEUED))
    {
        // 被信号中断
        ec_lock_down(&master->master_sem);
        if (request.state == EC_INT_REQUEST_QUEUED)
        {
            list_del(&request.list);
            ec_lock_up(&master->master_sem);
            ec_sdo_request_clear(&request);
            return -EINTR;
        }
        // 请求已在处理中：无法中断。
        ec_lock_up(&master->master_sem);
    }

    // 等待主站FSM处理完成
    wait_event(master->request_queue, request.state != EC_INT_REQUEST_BUSY);

    *abort_code = request.abort_code;

    if (request.state != EC_INT_REQUEST_SUCCESS)
    {
        *result_size = 0;
        if (request.errno)
        {
            ret = -request.errno;
        }
        else
        {
            ret = -EIO;
        }
    }
    else
    {
        if (request.data_size > target_size)
        {
            EC_SLAVE_ERR(slave, "%s(): 缓冲区太小。\n", __func__);
            ret = -EOVERFLOW;
        }
        else
        {
            memcpy(target, request.data, request.data_size);
            *result_size = request.data_size;
            ret = 0;
        }
    }

    ec_sdo_request_clear(&request);
    return ret;
}


/*****************************************************************************/

/**
 * @brief 使用完整访问方式从EtherCAT主站的SDO对象上传数据。
 *
 * @param master EtherCAT主站对象指针。
 * @param slave_position 从站位置。
 * @param index SDO索引。
 * @param target 目标缓冲区指针。
 * @param target_size 目标缓冲区大小。
 * @param result_size 用于存储结果大小的指针。
 * @param abort_code 用于存储中止代码的指针。
 * @return 返回0表示成功，返回负值表示错误。
 */
int ecrt_master_sdo_upload_complete(ec_master_t *master, uint16_t slave_position,
                                    uint16_t index, uint8_t *target,
                                    size_t target_size, size_t *result_size, uint32_t *abort_code)
{
    ec_sdo_request_t request;
    ec_slave_t *slave;
    int ret = 0;

    EC_MASTER_DBG(master, 1, "%s(master = 0x%p,"
                             " slave_position = %u, index = 0x%04X,"
                             " target = 0x%p, target_size = %zu, result_size = 0x%p,"
                             " abort_code = 0x%p)\n",
                  __func__, master, slave_position, index,
                  target, target_size, result_size, abort_code);

    ec_sdo_request_init(&request);
    ecrt_sdo_request_index_complete(&request, index);
    ecrt_sdo_request_read(&request);

    if (ec_lock_down_interruptible(&master->master_sem))
    {
        ec_sdo_request_clear(&request);
        return -EINTR;
    }

    if (!(slave = ec_master_find_slave(master, 0, slave_position)))
    {
        ec_lock_up(&master->master_sem);
        ec_sdo_request_clear(&request);
        EC_MASTER_ERR(master, "从站 %u 不存在！\n", slave_position);
        return -EINVAL;
    }

    EC_SLAVE_DBG(slave, 1, "调度SDO上传请求 (完整访问)。\n");

    // 调度请求。
    list_add_tail(&request.list, &slave->sdo_requests);

    ec_lock_up(&master->master_sem);

    // 等待FSM处理
    if (wait_event_interruptible(master->request_queue,
                                 request.state != EC_INT_REQUEST_QUEUED))
    {
        // 被信号中断
        ec_lock_down(&master->master_sem);
        if (request.state == EC_INT_REQUEST_QUEUED)
        {
            list_del(&request.list);
            ec_lock_up(&master->master_sem);
            ec_sdo_request_clear(&request);
            return -EINTR;
        }
        // 请求已在处理中：无法中断。
        ec_lock_up(&master->master_sem);
    }

    // 等待主站FSM处理完成
    wait_event(master->request_queue, request.state != EC_INT_REQUEST_BUSY);

    *abort_code = request.abort_code;

    if (request.state != EC_INT_REQUEST_SUCCESS)
    {
        *result_size = 0;
        if (request.errno)
        {
            ret = -request.errno;
        }
        else
        {
            ret = -EIO;
        }
    }
    else
    {
        if (request.data_size > target_size)
        {
            EC_MASTER_ERR(master, "缓冲区太小。\n");
            ret = -EOVERFLOW;
        }
        else
        {
            memcpy(target, request.data, request.data_size);
            *result_size = request.data_size;
            ret = 0;
        }
    }

    ec_sdo_request_clear(&request);
    return ret;
}

/*****************************************************************************/

/**
 * @brief 向EtherCAT主站写入IDN数据。
 *
 * @param master EtherCAT主站对象指针。
 * @param slave_position 从站位置。
 * @param drive_no 驱动器编号。
 * @param idn IDN值。
 * @param data 数据指针。
 * @param data_size 数据大小。
 * @param error_code 用于存储错误代码的指针。
 * @return 返回0表示成功，返回负值表示错误。
 */
int ecrt_master_write_idn(ec_master_t *master, uint16_t slave_position,
                          uint8_t drive_no, uint16_t idn, uint8_t *data, size_t data_size,
                          uint16_t *error_code)
{
    ec_soe_request_t request;
    ec_slave_t *slave;
    int ret;

    if (drive_no > 7)
    {
        EC_MASTER_ERR(master, "无效的驱动器编号！\n");
        return -EINVAL;
    }

    ec_soe_request_init(&request);
    ec_soe_request_set_drive_no(&request, drive_no);
    ec_soe_request_set_idn(&request, idn);

    ret = ec_soe_request_alloc(&request, data_size);
    if (ret)
    {
        ec_soe_request_clear(&request);
        return ret;
    }

    memcpy(request.data, data, data_size);
    request.data_size = data_size;
    ec_soe_request_write(&request);

    if (ec_lock_down_interruptible(&master->master_sem))
    {
        ec_soe_request_clear(&request);
        return -EINTR;
    }

    if (!(slave = ec_master_find_slave(master, 0, slave_position)))
    {
        ec_lock_up(&master->master_sem);
        EC_MASTER_ERR(master, "从站 %u 不存在！\n",
                      slave_position);
        ec_soe_request_clear(&request);
        return -EINVAL;
    }

    EC_SLAVE_DBG(slave, 1, "调度SoE写入请求。\n");

    // 调度SoE写入请求。
    list_add_tail(&request.list, &slave->soe_requests);

    ec_lock_up(&master->master_sem);

    // 等待FSM处理
    if (wait_event_interruptible(master->request_queue,
                                 request.state != EC_INT_REQUEST_QUEUED))
    {
        // 被信号中断
        ec_lock_down(&master->master_sem);
        if (request.state == EC_INT_REQUEST_QUEUED)
        {
            // 中止请求
            list_del(&request.list);
            ec_lock_up(&master->master_sem);
            ec_soe_request_clear(&request);
            return -EINTR;
        }
        ec_lock_up(&master->master_sem);
    }

    // 等待主站FSM处理完成
    wait_event(master->request_queue, request.state != EC_INT_REQUEST_BUSY);

    if (error_code)
    {
        *error_code = request.error_code;
    }
    ret = request.state == EC_INT_REQUEST_SUCCESS ? 0 : -EIO;
    ec_soe_request_clear(&request);

    return ret;
}

/*****************************************************************************/

/**
 * @brief 从EtherCAT主站读取IDN数据。
 *
 * @param master EtherCAT主站对象指针。
 * @param slave_position 从站位置。
 * @param drive_no 驱动器编号。
 * @param idn IDN值。
 * @param target 目标缓冲区指针。
 * @param target_size 目标缓冲区大小。
 * @param result_size 用于存储结果大小的指针。
 * @param error_code 用于存储错误代码的指针。
 * @return 返回0表示成功，返回负值表示错误。
 */
int ecrt_master_read_idn(ec_master_t *master, uint16_t slave_position,
                         uint8_t drive_no, uint16_t idn, uint8_t *target, size_t target_size,
                         size_t *result_size, uint16_t *error_code)
{
    ec_soe_request_t request;
    ec_slave_t *slave;
    int ret;

    if (drive_no > 7)
    {
        EC_MASTER_ERR(master, "无效的驱动器编号！\n");
        return -EINVAL;
    }

    ec_soe_request_init(&request);
    ec_soe_request_set_drive_no(&request, drive_no);
    ec_soe_request_set_idn(&request, idn);
    ec_soe_request_read(&request);

    if (ec_lock_down_interruptible(&master->master_sem))
    {
        ec_soe_request_clear(&request);
        return -EINTR;
    }

    if (!(slave = ec_master_find_slave(master, 0, slave_position)))
    {
        ec_lock_up(&master->master_sem);
        ec_soe_request_clear(&request);
        EC_MASTER_ERR(master, "从站 %u 不存在！\n", slave_position);
        return -EINVAL;
    }

    EC_SLAVE_DBG(slave, 1, "调度SoE读取请求。\n");

    // 调度请求。
    list_add_tail(&request.list, &slave->soe_requests);

    ec_lock_up(&master->master_sem);

    // 等待FSM处理
    if (wait_event_interruptible(master->request_queue,
                                 request.state != EC_INT_REQUEST_QUEUED))
    {
        // 被信号中断
        ec_lock_down(&master->master_sem);
        if (request.state == EC_INT_REQUEST_QUEUED)
        {
            list_del(&request.list);
            ec_lock_up(&master->master_sem);
            ec_soe_request_clear(&request);
            return -EINTR;
        }
        // 请求已在处理中：无法中断。
        ec_lock_up(&master->master_sem);
    }

    // 等待主站FSM处理完成
    wait_event(master->request_queue, request.state != EC_INT_REQUEST_BUSY);

    if (error_code)
    {
        *error_code = request.error_code;
    }

    if (request.state != EC_INT_REQUEST_SUCCESS)
    {
        if (result_size)
        {
            *result_size = 0;
        }
        ret = -EIO;
    }
    else
    { // 成功
        if (request.data_size > target_size)
        {
            EC_SLAVE_ERR(slave, "%s(): 缓冲区太小。\n", __func__);
            ret = -EOVERFLOW;
        }
        else
        { // 数据适应缓冲区
            if (result_size)
            {
                *result_size = request.data_size;
            }
            memcpy(target, request.data, request.data_size);
            ret = 0;
        }
    }

    ec_soe_request_clear(&request);
    return ret;
}

/*****************************************************************************/

/**
 * @brief 设置EtherCAT主站的实时从站请求处理方式。
 *
 * @param master EtherCAT主站对象指针。
 * @param rt_slave_requests 实时从站请求处理方式标志。
 * @return 返回0表示成功。
 */
int ecrt_master_rt_slave_requests(ec_master_t *master,
                                  unsigned int rt_slave_requests)
{
    // 设置标志，确定是主站还是外部应用程序处理从站请求
    master->rt_slave_requests = rt_slave_requests;

    if (master->rt_slave_requests)
    {
        EC_MASTER_INFO(master, "应用程序选择由应用程序处理从站请求。\n");
    }
    else
    {
        EC_MASTER_INFO(master, "应用程序选择由主站处理从站请求。\n");
    }

    return 0;
}


/*****************************************************************************/

/**
 * @brief 执行从站请求。
 *
 * @param master EtherCAT主站对象指针。
 */
void ecrt_master_exec_slave_requests(ec_master_t *master)
{
    // 执行从站状态机
    if (ec_lock_down_interruptible(&master->master_sem))
    {
        return;
    }

    // 如果主站处于操作状态且设置为由应用程序处理从站请求，则执行从站状态机
    if (master->rt_slave_requests && master->rt_slaves_available &&
        (master->phase == EC_OPERATION))
    {
        ec_master_exec_slave_fsms(master);
    }

    ec_lock_up(&master->master_sem);
}

/*****************************************************************************/

#ifdef EC_EOE

/**
 * @brief 向EtherCAT主站添加EoE接口。
 *
 * @param master EtherCAT主站对象指针。
 * @param alias EoE别名。
 * @param posn EoE位置。
 * @return 返回0表示成功，返回负值表示错误。
 */
int ecrt_master_eoe_addif(ec_master_t *master,
                          uint16_t alias, uint16_t posn)
{
    ec_eoe_t *eoe;
    char name[EC_DATAGRAM_NAME_SIZE];
    int res;

    // 检查名称是否已存在
    if (alias)
    {
        snprintf(name, EC_DATAGRAM_NAME_SIZE, "eoe%ua%u", master->index, alias);
    }
    else
    {
        snprintf(name, EC_DATAGRAM_NAME_SIZE, "eoe%us%u", master->index, posn);
    }

    ec_lock_down(&master->master_sem);
    list_for_each_entry(eoe, &master->eoe_handlers, list)
    {
        if ((eoe->slave == NULL) &&
            (strncmp(name, ec_eoe_name(eoe), EC_DATAGRAM_NAME_SIZE) == 0))
        {
            ec_lock_up(&master->master_sem);
            return -EADDRINUSE;
        }
    }

    // 未找到，创建新的EoE接口
    if (!(eoe = kmalloc(sizeof(ec_eoe_t), GFP_KERNEL)))
    {
        EC_MASTER_ERR(master, "分配EoE处理程序内存失败！\n");
        ec_lock_up(&master->master_sem);
        return -EFAULT;
    }

    if ((res = ec_eoe_init(master, eoe, alias, posn)))
    {
        EC_MASTER_ERR(master, "初始化EoE处理程序失败！\n");
        kfree(eoe);
        ec_lock_up(&master->master_sem);
        return res;
    }

    list_add_tail(&eoe->list, &master->eoe_handlers);
    ec_lock_up(&master->master_sem);

    return 0;
}

/*****************************************************************************/

/**
 * @brief 从EtherCAT主站删除EoE接口。
 *
 * @param master EtherCAT主站对象指针。
 * @param alias EoE别名。
 * @param posn EoE位置。
 * @return 返回0表示成功，返回负值表示错误。
 */
int ecrt_master_eoe_delif(ec_master_t *master,
                          uint16_t alias, uint16_t posn)
{
    ec_eoe_t *eoe;
    char name[EC_DATAGRAM_NAME_SIZE];

    if (alias)
    {
        snprintf(name, EC_DATAGRAM_NAME_SIZE, "eoe%ua%u", master->index, alias);
    }
    else
    {
        snprintf(name, EC_DATAGRAM_NAME_SIZE, "eoe%us%u", master->index, posn);
    }

    ec_lock_down(&master->master_sem);
    list_for_each_entry(eoe, &master->eoe_handlers, list)
    {
        if (strncmp(name, ec_eoe_name(eoe), EC_DATAGRAM_NAME_SIZE) == 0)
        {
            list_del(&eoe->list);
            ec_eoe_clear(eoe);
            kfree(eoe);
            ec_lock_up(&master->master_sem);
            return 0;
        }
    }
    ec_lock_up(&master->master_sem);

    return -EFAULT;
}

#endif


/*****************************************************************************/

/**
 * @brief 获取EtherCAT主站的对象字典。
 *
 * @param master EtherCAT主站对象指针。
 * @param data 数据指针。
 * @param data_size 用于存储数据大小的指针。
 * @param buff_size 缓冲区大小。
 * @return 返回0表示成功，返回负值表示错误。
 */
int ec_master_obj_dict(ec_master_t *master, uint8_t *data,
                       size_t *data_size, size_t buff_size)
{
    uint8_t sdo_req_cmd;
    uint8_t sdo_resp_cmd;
    uint16_t sdo_index;
    uint8_t sdo_sub_index;
    uint16_t slave_posn;
    ec_slave_t *slave;
    char value[32];
    size_t value_size;
    size_t total_value_size;
    int i;
    uint8_t link_status;
    uint32_t offset;
    uint32_t abort_code;
    uint8_t resp_error = 0;

    EC_MASTER_DBG(master, 1, "MBox Gateway请求主站信息。\n");

    // 检查邮箱头类型是否为CoE
    if ((*data_size < EC_MBOX_HEADER_SIZE) ||
        ((EC_READ_U16(data + 5) & 0x0F) != EC_MBOX_TYPE_COE))
    {
        EC_MASTER_ERR(master, "主站不支持所请求的邮箱类型！\n");
        return -EPROTONOSUPPORT;
    }

    // 确保CoE头服务是SDO请求
    offset = EC_MBOX_HEADER_SIZE;
    if ((*data_size < EC_MBOX_HEADER_SIZE + EC_COE_HEADER_SIZE + 4) ||
        (EC_READ_U16(data + offset) >> 12 != 0x2))
    {
        EC_MASTER_ERR(master, "主站仅支持SDO请求！\n");
        return -EINVAL;
    }

    // 获取SDO命令、索引和子索引
    offset = EC_MBOX_HEADER_SIZE + EC_COE_HEADER_SIZE;
    sdo_req_cmd = EC_READ_U8(data + offset) >> 5;
    sdo_index = EC_READ_U16(data + offset + 1);
    sdo_sub_index = EC_READ_U8(data + offset + 3);

    // 获取主站锁
    if (ec_lock_down_interruptible(&master->master_sem))
    {
        return -EINTR;
    }

    // 处理SDO请求
    // 参见ETG.5001.3，了解所需的对象字典条目支持
    if ((sdo_index >= 0x8000) && (sdo_index < 0x8000 + master->slave_count))
    {
        // 只读命令
        if (sdo_req_cmd != 0x02)
        {
            ec_lock_up(&master->master_sem);
            EC_MASTER_ERR(master, "主站，不支持SDO命令 %hhu 在"
                                  " 0x%04X:%02X 上！\n",
                          sdo_req_cmd, sdo_index, sdo_sub_index);
            return -EPROTONOSUPPORT;
        }

        // 计算从站位置（从站从位置0开始）
        slave_posn = sdo_index - 0x8000;

        // 获取从站信息
        if (!(slave = ec_master_find_slave(master, 0, slave_posn)))
        {
            ec_lock_up(&master->master_sem);
            EC_MASTER_ERR(master, "从站 %u 不存在！\n", slave_posn);
            return -EINVAL;
        }

        switch (sdo_sub_index)
        {
        case 0:
        {
            // 对象的长度（uint8）
            // 注意：这可能需要为35（8和33之间有一个间隔）
            value_size = sizeof(uint8_t);
            EC_WRITE_U8(value, 35);
        }
        break;
        case 1:
        {
            // 从站索引（uint16）
            // 从站位置 + MBG从站地址偏移量
            value_size = sizeof(uint16_t);
            EC_WRITE_U16(value, slave_posn + EC_MBG_SLAVE_ADDR_OFFSET);
        }
        break;
        case 2:
        {
            // 从站类型（visiblestring[16]）
            value_size = 16;
            memset(value, 0x00, value_size);
            if (slave->sii_image)
            {
                snprintf(value, value_size, "%s", slave->sii_image->sii.order);
            }
        }
        break;
        case 3:
        {
            // 从站名称（visiblestring[32]）
            value_size = 32;
            memset(value, 0x00, value_size);
            if (slave->sii_image)
            {
                snprintf(value, value_size, "%s", slave->sii_image->sii.name);
            }
        }
        break;
        case 4:
        {
            // 设备类型（uint32）（模块化设备配置文件）
            // 需要ESI文件提供此信息，或支持带有SDO的邮箱的从站（读取0x1000:00）
            if (!(slave->sii_image) ||
                !(slave->sii_image->sii.mailbox_protocols & EC_MBOX_COE) ||
                (ecrt_master_sdo_upload(master, slave_posn, 0x1000, 0x00,
                                        value, sizeof(uint32_t), &value_size, &abort_code) != 0))
            {
                // 默认返回0
                value_size = sizeof(uint32_t);
                EC_WRITE_U32(value, 0x00000000);
            }
        }
        break;
        case 5:
        {
            // 厂商ID（uint32）
            value_size = sizeof(uint32_t);
            EC_WRITE_U32(value, slave->config->vendor_id);
        }
        break;
        case 6:
        {
            // 产品代码（uint32）
            value_size = sizeof(uint32_t);
            EC_WRITE_U32(value, slave->config->product_code);
        }
        break;
        case 7:
        {
            // 版本号（uint32）
            value_size = sizeof(uint32_t);
            if (slave->sii_image)
            {
                EC_WRITE_U32(value, slave->sii_image->sii.revision_number);
            }
            else
            {
                EC_WRITE_U32(value, 0x00000000);
            }
        }
        break;
        case 8:
        {
            // 序列号（uint32）
            value_size = sizeof(uint32_t);
            if (slave->sii_image)
            {
                EC_WRITE_U32(value, slave->sii_image->sii.serial_number);
            }
            else
            {
                EC_WRITE_U32(value, 0x00000000);
            }
        }
        break;
        case 9 ... 32:
        {
            // 无法读取或存储数据
            resp_error = 1;
            value_size = sizeof(uint32_t);
            EC_WRITE_U32(value, 0x08000020);
        }
        break;
        case 33:
        {
            // 邮箱输出大小（uint16）
            value_size = sizeof(uint16_t);
            if (slave->sii_image)
            {
                EC_WRITE_U16(value, slave->sii_image->sii.std_rx_mailbox_size);
            }
            else
            {
                EC_WRITE_U16(value, 0x0000);
            }
        }
        break;
        case 34:
        {
            // 邮箱输入大小（uint16）
            value_size = sizeof(uint16_t);
            if (slave->sii_image)
            {
                EC_WRITE_U16(value, slave->sii_image->sii.std_tx_mailbox_size);
            }
            else
            {
                EC_WRITE_U16(value, 0x0000);
            }
        }
        break;
        case 35:
        {
            // 连接状态（uint8），寄存器0x0110:0x0111的位4..7
            link_status = 0;
            for (i = 0; i < EC_MAX_PORTS; i++)
            {
                if (slave->ports[i].link.link_up)
                {
                    link_status += 1 << (4 + i);
                }
            }
            value_size = sizeof(uint8_t);
            EC_WRITE_U8(value, link_status);
        }
        break;
        default:
        {
            // 子索引不存在错误
            resp_error = 1;
            value_size = sizeof(uint32_t);
            EC_WRITE_U32(value, 0x06090011);
        }
        break;
        }
    }
    else if ((sdo_index >= 0xA000) && (sdo_index < 0xA000 + master->slave_count))
    {
        // 注意：这应该是可选的，但是TwinSAFE_Loader.exe似乎
        //   希望有它

        // 计算从站位置（从站从位置0开始）
        slave_posn = sdo_index - 0xA000;

        switch (sdo_sub_index)
        {
        case 0:
        {
            // 只读命令
            if (sdo_req_cmd != 0x02)
            {
                ec_lock_up(&master->master_sem);
                EC_MASTER_ERR(master, "主站，不支持SDO命令 %hhu 在"
                                      " 0x%04X:%02X 上！\n",
                              sdo_req_cmd, sdo_index, sdo_sub_index);
                return -EPROTONOSUPPORT;
            }

            // 对象的长度（uint8）
            value_size = sizeof(uint8_t);
            EC_WRITE_U8(value, 2);
        }
        break;
        case 1:
        {
            // AL状态，寄存器0x130-0x131（uint16）
            // 当前状态

            // 只读命令
            if (sdo_req_cmd != 0x02)
            {
                ec_lock_up(&master->master_sem);
                EC_MASTER_ERR(master, "主站，不支持SDO命令 %hhu 在"
                                      " 0x%04X:%02X 上！\n",
                              sdo_req_cmd, sdo_index, sdo_sub_index);
                return -EPROTONOSUPPORT;
            }

            // 获取从站信息
            if (!(slave = ec_master_find_slave(master, 0, slave_posn)))
            {
                ec_lock_up(&master->master_sem);
                EC_MASTER_ERR(master, "从站 %u 不存在！\n", slave_posn);
                return -EINVAL;
            }

            // 返回缓存的从站当前状态
            // 注意：主站仅存储第一个状态字节
            // 而AL状态寄存器是两个字节（第二个字节保留）
            value_size = sizeof(uint16_t);
            EC_WRITE_U16(value, slave->current_state);
        }
        break;
        case 2:
        {
            // AL控制，寄存器0x120-0x121（uint16）
            // 请求的状态

            // 读写命令
            if ((sdo_req_cmd != 0x02) && (sdo_req_cmd != 0x00))
            {
                ec_lock_up(&master->master_sem);
                EC_MASTER_ERR(master, "主站，不支持SDO命令 %hhu 在"
                                      " 0x%04X:%02X 上！\n",
                              sdo_req_cmd, sdo_index, sdo_sub_index);
                return -EPROTONOSUPPORT;
            }

            // 获取从站信息
            if (!(slave = ec_master_find_slave(master, 0, slave_posn)))
            {
                ec_lock_up(&master->master_sem);
                EC_MASTER_ERR(master, "从站 %u 不存在！\n", slave_posn);
                return -EINVAL;
            }

            if (sdo_req_cmd == 0x02)
            {
                // 读取
                // 返回缓存的从站请求的状态
                // 注意：主站仅存储第一个状态字节
                // 而AL控制寄存器是两个字节（第二个字节保留）
                value_size = sizeof(uint16_t);
                EC_WRITE_U16(value, slave->requested_state);
            }
            else
            {
                // 写入（sdo_req_cmd == 0x00）
                uint8_t *write_data = data + offset + 4;
                size_t write_size;
                uint8_t size_specified = EC_READ_U8(data + offset) & 0x01;
                if (size_specified)
                {
                    write_size = 4 - ((EC_READ_U8(data + offset) & 0x0C) >> 2);
                }
                else
                {
                    write_size = 4;
                }

                // 检查写入大小
                if (write_size != 2)
                {
                    ec_lock_up(&master->master_sem);
                    EC_MASTER_ERR(master, "主站，SDO写入数据大小不符合预期"
                                          " %zu（期望 %u）在 0x%04X:%02X 上！\n",
                                  write_size, 2, sdo_index, sdo_sub_index);
                    return -EPROTONOSUPPORT;
                }

                // 请求从站的AL状态
                ec_slave_request_state(slave, EC_READ_U16(write_data));

                // 设置空白下载响应
                value_size = sizeof(uint32_t);
                memset(value, 0x00, 4);
            }
        }
        break;
        default:
        {
            // 子索引不存在错误
            resp_error = 1;
            value_size = sizeof(uint32_t);
            EC_WRITE_U32(value, 0x06090011);
        }
        break;
        }
    }
    else if ((sdo_index >= 0xF020) && (sdo_index < 0xF030))
    {
        // 只读命令
        if (sdo_req_cmd != 0x02)
        {
            ec_lock_up(&master->master_sem);
            EC_MASTER_ERR(master, "主站，不支持SDO命令 %hhu 在"
                                  " 0x%04X:%02X 上！\n",
                          sdo_req_cmd, sdo_index, sdo_sub_index);
            return -EPROTONOSUPPORT;
        }

        if (sdo_sub_index == 0)
        {
            uint64_t index = master->slave_count;
            uint32_t remainder;

            // 对象的长度（uint8）
            value_size = sizeof(uint8_t);

            // 从从站计算索引和余数
            remainder = do_div(index, 255);

            if (sdo_index - 0xF020 < index)
            {
                EC_WRITE_U8(value, 255);
            }
            else if ((sdo_index - 0xF020 == index) && (remainder > 0))
            {
                EC_WRITE_U8(value, remainder);
            }
            else
            {
                EC_WRITE_U8(value, 0);
            }
        }
        else
        {
            // 计算从站位置
            slave_posn = ((sdo_index - 0xF020) << 8) + sdo_sub_index - (sdo_index - 0xF020 + 1);

            if (slave_posn < master->slave_count)
            {
                // 从站索引（uint16）
                // 从站位置 + MBG从站地址偏移量
                value_size = sizeof(uint16_t);
                EC_WRITE_U16(value, slave_posn + EC_MBG_SLAVE_ADDR_OFFSET);
            }
            else
            {
                // 对象未找到错误
                resp_error = 1;
                value_size = sizeof(uint32_t);
                EC_WRITE_U32(value, 0x06020000);
            }
        }
    }
    else if (sdo_index == 0xF000)
    {
        // 只读命令
        if (sdo_req_cmd != 0x02)
        {
            ec_lock_up(&master->master_sem);
            EC_MASTER_ERR(master, "主站，不支持SDO命令 %hhu 在"
                                  " 0x%04X:%02X 上！\n",
                          sdo_req_cmd, sdo_index, sdo_sub_index);
            return -EPROTONOSUPPORT;
        }

        // 模块化设备配置文件
        switch (sdo_sub_index)
        {
        case 0:
        {
            // 对象的长度（uint8）
            value_size = sizeof(uint8_t);
            EC_WRITE_U8(value, 4);
        }
        break;
        case 1:
        {
            // 模块索引距离（uint16）
            value_size = sizeof(uint16_t);
            EC_WRITE_U16(value, 0x0001);
        }
        break;
        case 2:
        {
            // 最大模块数（uint16）
            // 网关信息限制
            value_size = sizeof(uint16_t);
            EC_WRITE_U16(value, 4080);
        }
        break;
        case 3:
        {
            // 通用配置（uint32）
            value_size = sizeof(uint32_t);
            EC_WRITE_U32(value, 0x000000FF);
        }
        break;
        case 4:
        {
            // 通用信息（uint32）
            value_size = sizeof(uint32_t);
            EC_WRITE_U32(value, 0x00000000);
        }
        break;
        default:
        {
            // 子索引不存在错误
            resp_error = 1;
            value_size = sizeof(uint32_t);
            EC_WRITE_U32(value, 0x06090011);
        }
        break;
        }
    }
    else
    {
        // 对象未找到错误
        resp_error = 1;
        value_size = sizeof(uint32_t);
        EC_WRITE_U32(value, 0x06020000);
    }

    ec_lock_up(&master->master_sem);

    // 是否需要为完整大小预留空间？
    if ((value_size > 0) && (value_size <= 4))
    {
        total_value_size = value_size;
    }
    else
    {
        total_value_size = value_size + sizeof(uint32_t);
    }

    // 设置回复
    if (EC_MBOX_HEADER_SIZE + EC_COE_HEADER_SIZE + 4 + total_value_size > buff_size)
    {
        EC_MASTER_ERR(master, "缓冲区太小。\n");
        return -EOVERFLOW;
    }
    else
    {
        // 更新data_size
        *data_size = EC_MBOX_HEADER_SIZE + EC_COE_HEADER_SIZE + 4 + total_value_size;

        // 更新Mailbox Header Length（从CoE Header开始的长度）
        EC_WRITE_U16(data, *data_size - EC_MBOX_HEADER_SIZE);

        // 更新CoE服务响应或错误时的SDO命令说明符
        if (resp_error)
        {
            // 不满意，返回中止SDO传输请求
            offset = EC_MBOX_HEADER_SIZE + EC_COE_HEADER_SIZE;
            EC_WRITE_U8(data + offset, 0x04 << 5);

            // 设置中止值
            memcpy(data + offset + 4, value, value_size);
        }
        else
        {
            // 满意，返回服务代码3（SDO响应）
            offset = EC_MBOX_HEADER_SIZE;
            EC_WRITE_U16(data + offset, 0x03 << 12);

            // 设置SDO命令说明符
            if (sdo_req_cmd == 0x02)
            {
                // 上传响应
                sdo_resp_cmd = 0x02;
            }
            else
            {
                // 下载响应
                sdo_resp_cmd = 0x01;
            }

            // 设置值大小
            offset = EC_MBOX_HEADER_SIZE + EC_COE_HEADER_SIZE;
            if ((value_size > 0) && (value_size <= 4))
            {
                // 上传响应，指定了大小
                // bit 0      1 = 指定了大小
                // bit 1      1 = expedited
                // bit 2..3   4 - size
                // bit 5..7   命令说明符
                EC_WRITE_U8(data + offset, (sdo_resp_cmd << 5) +
                                               ((4 - value_size) << 2) + 0x02 + 0x01);

                // 设置数据偏移量
                offset += 4;
            }
            else
            {
                // 上传响应，指定了大小
                EC_WRITE_U8(data + offset, (sdo_resp_cmd << 5) + 0x01);

                // 设置值大小
                offset += 4;
                EC_WRITE_U32(data + offset, value_size);

                // 设置值的偏移量
                offset += sizeof(uint32_t);
            }

            // 设置值
            memcpy(data + offset, value, value_size);
        }
    }

    return 0;
}


/*****************************************************************************/

/**
 * @brief 执行EtherCAT主站的邮箱网关请求。
 *
 * @param master EtherCAT主站对象指针。
 * @param data 数据指针。
 * @param data_size 用于存储数据大小的指针。
 * @param buff_size 缓冲区大小。
 * @return 返回0表示成功，返回负值表示错误。
 */
int ec_master_mbox_gateway(ec_master_t *master, uint8_t *data,
                           size_t *data_size, size_t buff_size)
{
    ec_mbg_request_t request;
    uint16_t slave_posn;
    ec_slave_t *slave;
    int ret = 0;

    // 获取从站地址
    slave_posn = EC_READ_U16(data + 2);

    // 检查从站地址是否为零（主站对象字典请求）
    if (slave_posn == 0)
    {
        // 请求主站信息
        ret = ec_master_obj_dict(master, data, data_size, buff_size);
    }
    else if (slave_posn >= EC_MBG_SLAVE_ADDR_OFFSET)
    {
        // 计算从站位置地址
        slave_posn -= EC_MBG_SLAVE_ADDR_OFFSET;

        // 将请求传递给从站
        ec_mbg_request_init(&request);
        ret = ec_mbg_request_copy_data(&request, data, *data_size);
        *data_size = 0;
        if (ret)
        {
            ec_mbg_request_clear(&request);
            return ret;
        }
        ec_mbg_request_run(&request);

        if (ec_lock_down_interruptible(&master->master_sem))
        {
            ec_mbg_request_clear(&request);
            return -EINTR;
        }

        // 检查有效的从站请求
        if (!(slave = ec_master_find_slave(master, 0, slave_posn)))
        {
            ec_lock_up(&master->master_sem);
            ec_mbg_request_clear(&request);
            EC_MASTER_ERR(master, "从站 %u 不存在！\n", slave_posn);
            return -EINVAL;
        }

        EC_SLAVE_DBG(slave, 1, "调度邮箱网关请求。\n");

        // 调度请求
        list_add_tail(&request.list, &slave->mbg_requests);

        ec_lock_up(&master->master_sem);

        // 等待FSM处理
        if (wait_event_interruptible(master->request_queue,
                                     request.state != EC_INT_REQUEST_QUEUED))
        {
            // 被信号中断
            ec_lock_down(&master->master_sem);
            if (request.state == EC_INT_REQUEST_QUEUED)
            {
                list_del(&request.list);
                ec_lock_up(&master->master_sem);
                ec_mbg_request_clear(&request);
                return -EINTR;
            }
            // 请求已在处理中：不可中断
            ec_lock_up(&master->master_sem);
        }

        // 等待主站FSM完成处理
        wait_event(master->request_queue, request.state != EC_INT_REQUEST_BUSY);

        if (request.state != EC_INT_REQUEST_SUCCESS)
        {
            if (request.error_code)
            {
                ret = -request.error_code;
            }
            else
            {
                ret = -EIO;
            }
        }
        else
        {
            if (request.data_size > buff_size)
            {
                EC_MASTER_ERR(master, "缓冲区太小。\n");
                ret = -EOVERFLOW;
            }
            else
            {
                memcpy(data, request.data, request.data_size);
                *data_size = request.data_size;
                ret = 0;
            }
        }

        ec_mbg_request_clear(&request);
    }
    else
    {
        EC_MASTER_ERR(master, "邮箱网关：无效的从站偏移地址 %u！\n", slave_posn);
        return -EINVAL;
    }

    return ret;
}

/*****************************************************************************/

/**
 * @brief 重置EtherCAT主站。
 *
 * @param master EtherCAT主站对象指针。
 */
void ecrt_master_reset(ec_master_t *master)
{
    ec_slave_config_t *sc;

    list_for_each_entry(sc, &master->configs, list)
    {
        if (sc->slave)
        {
            ec_slave_request_state(sc->slave, EC_SLAVE_STATE_OP);
        }
    }
}


/*****************************************************************************/

/** \cond */

EXPORT_SYMBOL(ecrt_master_create_domain);
EXPORT_SYMBOL(ecrt_master_setup_domain_memory);
EXPORT_SYMBOL(ecrt_master_activate);
EXPORT_SYMBOL(ecrt_master_deactivate_slaves);
EXPORT_SYMBOL(ecrt_master_deactivate);
EXPORT_SYMBOL(ecrt_master_send);
EXPORT_SYMBOL(ecrt_master_send_ext);
EXPORT_SYMBOL(ecrt_master_receive);
EXPORT_SYMBOL(ecrt_master_callbacks);
EXPORT_SYMBOL(ecrt_master);
EXPORT_SYMBOL(ecrt_master_get_slave);
EXPORT_SYMBOL(ecrt_master_slave_config);
EXPORT_SYMBOL(ecrt_master_select_reference_clock);
EXPORT_SYMBOL(ecrt_master_state);
EXPORT_SYMBOL(ecrt_master_link_state);
EXPORT_SYMBOL(ecrt_master_application_time);
EXPORT_SYMBOL(ecrt_master_sync_reference_clock);
EXPORT_SYMBOL(ecrt_master_sync_reference_clock_to);
EXPORT_SYMBOL(ecrt_master_sync_slave_clocks);
EXPORT_SYMBOL(ecrt_master_reference_clock_time);
EXPORT_SYMBOL(ecrt_master_64bit_reference_clock_time_queue);
EXPORT_SYMBOL(ecrt_master_64bit_reference_clock_time);
EXPORT_SYMBOL(ecrt_master_sync_monitor_queue);
EXPORT_SYMBOL(ecrt_master_sync_monitor_process);
EXPORT_SYMBOL(ecrt_master_sdo_download);
EXPORT_SYMBOL(ecrt_master_sdo_download_complete);
EXPORT_SYMBOL(ecrt_master_sdo_upload);
EXPORT_SYMBOL(ecrt_master_sdo_upload_complete);
EXPORT_SYMBOL(ecrt_master_write_idn);
EXPORT_SYMBOL(ecrt_master_read_idn);
EXPORT_SYMBOL(ecrt_master_rt_slave_requests);
EXPORT_SYMBOL(ecrt_master_exec_slave_requests);
#ifdef EC_EOE
EXPORT_SYMBOL(ecrt_master_eoe_addif);
EXPORT_SYMBOL(ecrt_master_eoe_delif);
#endif
EXPORT_SYMBOL(ecrt_master_reset);

/** \endcond */

/*****************************************************************************/

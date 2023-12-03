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
   Ethernet over EtherCAT (EoE)
*/

/*****************************************************************************/

#ifndef __EC_ETHERNET_H__
#define __EC_ETHERNET_H__

#include <linux/list.h>
#include <linux/netdevice.h>

#include "globals.h"
#include "locks.h"
#include "slave.h"
#include "datagram.h"

/*****************************************************************************/

/** EoE帧类型 */
enum
{
    EC_EOE_TYPE_FRAME_FRAG = 0x00,    /** EoE帧分片发送和接收. */
    EC_EOE_TYPE_TIMESTAMP_RES = 0x01, /** EoE时间戳响应. */
    EC_EOE_TYPE_INIT_REQ = 0x02,      /** EoE初始化，设置IP参数请求. */
    EC_EOE_TYPE_INIT_RES = 0x03,      /** EoE初始化，设置IP参数响应. */
    EC_EOE_TYPE_MACFILTER_REQ = 0x04, /** EoE设置MAC地址过滤器请求. */
    EC_EOE_TYPE_MACFILTER_RES = 0x05, /** EoE设置MAC地址过滤器响应. */
};

/*****************************************************************************/

typedef struct ec_eoe ec_eoe_t; /**< \see ec_eoe */

/**
   以太网通过EtherCAT（EoE）处理器。
   主站为每个支持EoE协议的从站创建一个此类对象。
*/

struct ec_eoe
{
    struct list_head list;         /**< 链表项 */
    ec_master_t *master;           /**< 指向对应主站的指针 */
    ec_slave_t *slave;             /**< 指向对应从站的指针 */
    ec_datagram_t datagram;        /**< 数据报 */
    unsigned int queue_datagram;   /**< 数据报已准备排队 */
    void (*state)(ec_eoe_t *);     /**< 状态机的状态函数 */
    struct net_device *dev;        /**< 虚拟以太网设备的net_device */
    struct net_device_stats stats; /**< 设备统计信息 */
    unsigned int opened;           /**< net_device已打开 */
    unsigned long rate_jiffies;    /**< 上次速率输出的时间 */
    unsigned int have_mbox_lock;   /**< 我们是否有邮箱锁的标志 */
    unsigned int auto_created;     /**< 自动创建的标志 */

    struct sk_buff *rx_skb;       /**< 当前接收的套接字缓冲区 */
    off_t rx_skb_offset;          /**< 套接字缓冲区中的当前写指针 */
    size_t rx_skb_size;           /**< 分配的套接字缓冲区内存大小 */
    uint8_t rx_expected_fragment; /**< 下一个期望的分片编号 */
    uint32_t rx_counter;          /**< 上一秒接收的八位组数 */
    uint32_t rx_rate;             /**< 接收速率（bps） */
    unsigned int rx_idle;         /**< 空闲标志 */

    struct sk_buff **tx_ring;      /**< 用于发送帧的环形缓冲区 */
    unsigned int tx_ring_count;    /**< 传输环形缓冲区计数 */
    unsigned int tx_ring_size;     /**< 传输环形缓冲区大小 */
    unsigned int tx_next_to_use;   /**< 添加到环形缓冲区的帧的索引 */
    unsigned int tx_next_to_clean; /**< 从环形缓冲区使用的帧的索引 */
    unsigned int tx_queue_active;  /**< 内核netif队列已启动 */
    struct sk_buff *tx_skb;        /**< 当前TX帧 */
    uint8_t tx_frame_number;       /**< 已传输帧的编号 */
    uint8_t tx_fragment_number;    /**< 分片编号 */
    size_t tx_offset;              /**< 已发送的八位组数 */
    uint32_t tx_counter;           /**< 上一秒传输的八位组数 */
    uint32_t tx_rate;              /**< 传输速率（bps） */
    unsigned int tx_idle;          /**< 空闲标志 */

    unsigned int tries; /**< 尝试次数 */
};


/*****************************************************************************/

int ec_eoe_parse(const char *, int *, uint16_t *, uint16_t *);

int ec_eoe_init(ec_master_t *, ec_eoe_t *, uint16_t /*alias*/, uint16_t /*posn*/);
int ec_eoe_auto_init(ec_eoe_t *, ec_slave_t *);
void ec_eoe_link_slave(ec_eoe_t *, ec_slave_t *);
void ec_eoe_clear_slave(ec_eoe_t *);
void ec_eoe_clear(ec_eoe_t *);
void ec_eoe_run(ec_eoe_t *);
void ec_eoe_queue(ec_eoe_t *);
int ec_eoe_is_open(const ec_eoe_t *);
int ec_eoe_is_idle(const ec_eoe_t *);
char *ec_eoe_name(const ec_eoe_t *);
unsigned int ec_eoe_tx_queued_frames(const ec_eoe_t *);

/*****************************************************************************/

#endif

/*****************************************************************************/

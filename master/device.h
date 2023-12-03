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
   EtherCAT device structure.
*/

/*****************************************************************************/

#ifndef __EC_DEVICE_H__
#define __EC_DEVICE_H__

#include <linux/interrupt.h>
#include <linux/version.h>

#include "../devices/ecdev.h"
#include "globals.h"


/**
 * 传输环的大小。
 * 该内存环用于传输帧。必须使用不同的内存区域，否则网络设备的 DMA 可能会发送相同的数据两次，如果调用两次。
 */
#define EC_TX_RING_SIZE 0x10

#ifdef EC_DEBUG_IF
#include "debug.h"
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0)
struct timeval
{
    __kernel_old_time_t tv_sec;   /* 秒 */
    __kernel_suseconds_t tv_usec; /* 微秒 */
};
#endif

#ifdef EC_DEBUG_RING
#define EC_DEBUG_RING_SIZE 10

typedef enum
{
    TX,
    RX
} ec_debug_frame_dir_t;

typedef struct
{
    ec_debug_frame_dir_t dir;
    struct timeval t;
    uint8_t data[EC_MAX_DATA_SIZE];
    unsigned int data_size;
} ec_debug_frame_t;

#endif

/*****************************************************************************/

/**
   EtherCAT 设备。
   一个 EtherCAT 设备是一个网络接口卡，由 EtherCAT 主控制器拥有，用于发送和接收 EtherCAT 帧。
*/

struct ec_device
{
    ec_master_t *master;                     /**< EtherCAT 主控制器 */
    struct net_device *dev;                  /**< 指向分配的 net_device 的指针 */
    ec_pollfunc_t poll;                      /**< 指向设备的轮询函数的指针 */
    struct module *module;                   /**< 指向拥有该设备的模块的指针 */
    uint8_t open;                            /**< 如果 net_device 已经打开，则为 true */
    uint8_t link_state;                      /**< 设备连接状态 */
    struct sk_buff *tx_skb[EC_TX_RING_SIZE]; /**< 传输 skb 环 */
    unsigned int tx_ring_index;              /**< 用于传输的最后一个环条目 */
#ifdef EC_HAVE_CYCLES
    cycles_t cycles_poll; /**< 上次轮询的周期数 */
#endif
#if defined(EC_DEBUG_RING) || !defined(EC_RTDM)
    struct timeval timeval_poll;
#endif
    unsigned long jiffies_poll; /**< 上次轮询的 jiffies */

    // 帧统计
    u64 tx_count;                      /**< 发送的帧数 */
    u64 last_tx_count;                 /**< 上次统计周期发送的帧数 */
    u64 rx_count;                      /**< 接收的帧数 */
    u64 last_rx_count;                 /**< 上次统计周期接收的帧数 */
    u64 tx_bytes;                      /**< 发送的字节数 */
    u64 last_tx_bytes;                 /**< 上次统计周期发送的字节数 */
    u64 rx_bytes;                      /**< 接收的字节数 */
    u64 last_rx_bytes;                 /**< 上次统计周期接收的字节数 */
    u64 tx_errors;                     /**< 传输错误数 */
    s32 tx_frame_rates[EC_RATE_COUNT]; /**< 不同统计周期内的帧传输速率（每秒帧数） */
    s32 rx_frame_rates[EC_RATE_COUNT]; /**< 不同统计周期内的帧接收速率（每秒帧数） */
    s32 tx_byte_rates[EC_RATE_COUNT];  /**< 不同统计周期内的字节传输速率（每秒字节数） */
    s32 rx_byte_rates[EC_RATE_COUNT];  /**< 不同统计周期内的字节接收速率（每秒字节数） */

#ifdef EC_DEBUG_IF
    ec_debug_t dbg; /**< 调试设备 */
#endif
#ifdef EC_DEBUG_RING
    ec_debug_frame_t debug_frames[EC_DEBUG_RING_SIZE];
    unsigned int debug_frame_index;
    unsigned int debug_frame_count;
#endif
};

/*****************************************************************************/

/**
   pcap 全局标头
*/

typedef struct
{
    u32 magic_number;  /* 魔术数 */
    u16 version_major; /* 主版本号 */
    u16 version_minor; /* 次版本号 */
    s32 thiszone;      /* GMT 到本地时间的修正 */
    u32 sigfigs;       /* 时间戳的精度 */
    u32 snaplen;       /* 文件中捕获的数据包的最大长度（以八位字节为单位） */
    u32 network;       /* 数据链路类型 */
} pcap_hdr_t;

/*****************************************************************************/

/**
   pcap 数据包标头
*/

typedef struct
{
    u32 ts_sec;   /* 时间戳秒数 */
    u32 ts_usec;  /* 时间戳微秒数 */
    u32 incl_len; /* 文件中保存的数据包的八位字节数 */
    u32 orig_len; /* 数据包的实际长度 */
} pcaprec_hdr_t;

/*****************************************************************************/

int ec_device_init(ec_device_t *, ec_master_t *);
void ec_device_clear(ec_device_t *);

void ec_device_attach(ec_device_t *, struct net_device *, ec_pollfunc_t,
                      struct module *);
void ec_device_detach(ec_device_t *);

int ec_device_open(ec_device_t *);
int ec_device_close(ec_device_t *);

void ec_device_poll(ec_device_t *);
uint8_t *ec_device_tx_data(ec_device_t *);
void ec_device_send(ec_device_t *, size_t);
void ec_device_clear_stats(ec_device_t *);
void ec_device_update_stats(ec_device_t *);

#ifdef EC_DEBUG_RING
void ec_device_debug_ring_append(ec_device_t *, ec_debug_frame_dir_t,
                                 const void *, size_t);
void ec_device_debug_ring_print(const ec_device_t *);
#endif

/*****************************************************************************/

#endif


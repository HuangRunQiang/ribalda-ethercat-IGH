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
   EtherCAT datagram structure.
*/

/*****************************************************************************/

#ifndef __EC_DATAGRAM_H__
#define __EC_DATAGRAM_H__

#include <linux/list.h>
#include <linux/time.h>
#include <linux/timex.h>

#include "globals.h"

/*****************************************************************************/

/** EtherCAT数据报类型 */
typedef enum
{
    EC_DATAGRAM_NONE = 0x00, /**< 无效 */
    EC_DATAGRAM_APRD = 0x01, /**< 自动递增物理读取 */
    EC_DATAGRAM_APWR = 0x02, /**< 自动递增物理写入 */
    EC_DATAGRAM_APRW = 0x03, /**< 自动递增物理读写 */
    EC_DATAGRAM_FPRD = 0x04, /**< 配置地址物理读取 */
    EC_DATAGRAM_FPWR = 0x05, /**< 配置地址物理写入 */
    EC_DATAGRAM_FPRW = 0x06, /**< 配置地址物理读写 */
    EC_DATAGRAM_BRD = 0x07,  /**< 广播读取 */
    EC_DATAGRAM_BWR = 0x08,  /**< 广播写入 */
    EC_DATAGRAM_BRW = 0x09,  /**< 广播读写 */
    EC_DATAGRAM_LRD = 0x0A,  /**< 逻辑读取 */
    EC_DATAGRAM_LWR = 0x0B,  /**< 逻辑写入 */
    EC_DATAGRAM_LRW = 0x0C,  /**< 逻辑读写 */
    EC_DATAGRAM_ARMW = 0x0D, /**< 自动递增物理读取多次写入 */
    EC_DATAGRAM_FRMW = 0x0E, /**< 配置地址物理读取多次写入 */
} ec_datagram_type_t;

/*****************************************************************************/

/** EtherCAT数据报状态 */
typedef enum
{
    EC_DATAGRAM_INIT,      /**< 新数据报的初始状态 */
    EC_DATAGRAM_QUEUED,    /**< 已排队等待发送 */
    EC_DATAGRAM_SENT,      /**< 已发送（仍在队列中） */
    EC_DATAGRAM_RECEIVED,  /**< 已接收（已出队列） */
    EC_DATAGRAM_TIMED_OUT, /**< 超时（已出队列） */
    EC_DATAGRAM_ERROR,     /**< 发送/接收时出错（已出队列） */
    EC_DATAGRAM_INVALID    /**< 未使用，不应排队（已出队列） */
} ec_datagram_state_t;

/*****************************************************************************/

/** EtherCAT数据报 */
typedef struct
{
    struct list_head queue;         /**< 主数据报队列项 */
    struct list_head sent;          /**< 已发送数据报的主列表项 */
    ec_device_index_t device_index; /**< 发送/已发送数据报的设备 */
    ec_datagram_type_t type;        /**< 数据报类型（APRD，BWR等） */
    uint8_t address[EC_ADDR_LEN];   /**< 接收方地址 */
    uint8_t *data;                  /**< 数据报有效载荷 */
    ec_origin_t data_origin;        /**< 数据的来源 */
    size_t mem_size;                /**< 数据报数据内存大小 */
    size_t data_size;               /**< 数据报数据的大小 */
    uint8_t index;                  /**< 索引（由主控制器设置） */
    uint16_t working_counter;       /**< 工作计数器 */
    ec_datagram_state_t state;      /**< 状态 */
#ifdef EC_HAVE_CYCLES
    cycles_t cycles_sent; /**< 数据报发送时间 */
#endif
    unsigned long jiffies_sent; /**< 数据报发送的jiffies时间 */
    uint64_t app_time_sent;     /**< 数据报发送的应用时间 */
#ifdef EC_HAVE_CYCLES
    cycles_t cycles_received; /**< 数据报接收时间 */
#endif
    unsigned long jiffies_received;     /**< 数据报接收的jiffies时间 */
    unsigned int skip_count;            /**< 尚未接收时的重新排队次数 */
    unsigned long stats_output_jiffies; /**< 上次统计输出时间 */
    char name[EC_DATAGRAM_NAME_SIZE];   /**< 数据报描述 */
} ec_datagram_t;

/*****************************************************************************/

/** EtherCAT邮箱响应数据 */
typedef struct
{
    uint8_t *data;       /**< 邮箱响应数据 */
    size_t data_size;    /**< 邮箱响应数据缓冲区的大小 */
    size_t payload_size; /**< 邮箱响应有效载荷数据的大小 */
} ec_mbox_data_t;

/*****************************************************************************/

void ec_datagram_init(ec_datagram_t *);
void ec_datagram_clear(ec_datagram_t *);
void ec_datagram_unqueue(ec_datagram_t *);
int ec_datagram_prealloc(ec_datagram_t *, size_t);
void ec_datagram_zero(ec_datagram_t *);
int ec_datagram_repeat(ec_datagram_t *, const ec_datagram_t *);

int ec_datagram_aprd(ec_datagram_t *, uint16_t, uint16_t, size_t);
int ec_datagram_apwr(ec_datagram_t *, uint16_t, uint16_t, size_t);
int ec_datagram_aprw(ec_datagram_t *, uint16_t, uint16_t, size_t);
int ec_datagram_armw(ec_datagram_t *, uint16_t, uint16_t, size_t);
int ec_datagram_fprd(ec_datagram_t *, uint16_t, uint16_t, size_t);
int ec_datagram_fpwr(ec_datagram_t *, uint16_t, uint16_t, size_t);
int ec_datagram_fprw(ec_datagram_t *, uint16_t, uint16_t, size_t);
int ec_datagram_frmw(ec_datagram_t *, uint16_t, uint16_t, size_t);
int ec_datagram_brd(ec_datagram_t *, uint16_t, size_t);
int ec_datagram_bwr(ec_datagram_t *, uint16_t, size_t);
int ec_datagram_brw(ec_datagram_t *, uint16_t, size_t);
int ec_datagram_lrd(ec_datagram_t *, uint32_t, size_t);
int ec_datagram_lwr(ec_datagram_t *, uint32_t, size_t);
int ec_datagram_lrw(ec_datagram_t *, uint32_t, size_t);
int ec_datagram_lrd_ext(ec_datagram_t *, uint32_t, size_t, uint8_t *);
int ec_datagram_lwr_ext(ec_datagram_t *, uint32_t, size_t, uint8_t *);
int ec_datagram_lrw_ext(ec_datagram_t *, uint32_t, size_t, uint8_t *);

void ec_datagram_print_state(const ec_datagram_t *);
void ec_datagram_print_wc_error(const ec_datagram_t *);
void ec_datagram_output_stats(ec_datagram_t *);
const char *ec_datagram_type_string(const ec_datagram_t *);

void ec_mbox_data_init(ec_mbox_data_t *);
void ec_mbox_data_clear(ec_mbox_data_t *);
void ec_mbox_prot_data_prealloc(ec_slave_t *, uint16_t, size_t);


/*****************************************************************************/

#endif

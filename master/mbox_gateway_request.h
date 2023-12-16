/******************************************************************************
 *
 *  $Id$
 *
 *  Copyright (C) 2019  Florian Pose, Ingenieurgemeinschaft IgH
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
   EtherCAT邮箱网关请求结构体。
*/

/*****************************************************************************/

#ifndef __EC_MBG_REQUEST_H__
#define __EC_MBG_REQUEST_H__

#include <linux/list.h>

#include "globals.h"

/*****************************************************************************/

/** EtherCAT邮箱网关请求。
 */
typedef struct
{
    struct list_head list;             /**< 链表项。 */
    uint8_t *data;                     /**< 指向MBox请求数据的指针。 */
    size_t mem_size;                   /**< MBox请求数据内存的大小。 */
    size_t data_size;                  /**< MBox请求数据的大小。 */
    uint32_t response_timeout;         /**< 如果从站不响应，传输重试的最大时间（以毫秒为单位）。 */
    ec_internal_request_state_t state; /**< 请求状态。 */
    unsigned long jiffies_sent;        /**< 发送上传/下载请求的时刻。 */
    uint16_t error_code;               /**< MBox网关错误代码。 */
    uint8_t mbox_type;                 /**< 缓存的MBox类型。 */
} ec_mbg_request_t;
/*****************************************************************************/

void ec_mbg_request_init(ec_mbg_request_t *);
void ec_mbg_request_clear(ec_mbg_request_t *);

int ec_mbg_request_alloc(ec_mbg_request_t *, size_t);
int ec_mbg_request_copy_data(ec_mbg_request_t *, const uint8_t *, size_t);
void ec_mbg_request_run(ec_mbg_request_t *);

/*****************************************************************************/

#endif

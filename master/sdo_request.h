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
   EtherCAT CANopen SDO request structure.
*/

/*****************************************************************************/

#ifndef __EC_SDO_REQUEST_H__
#define __EC_SDO_REQUEST_H__

#include <linux/list.h>

#include "globals.h"

/*****************************************************************************/

/** CANopen SDO 请求。
 */
struct ec_sdo_request
{
    struct list_head list;             /**< 链表项。 */
    uint16_t index;                    /**< SDO 索引。 */
    uint8_t subindex;                  /**< SDO 子索引。 */
    uint8_t *data;                     /**< 指向 SDO 数据的指针。 */
    size_t mem_size;                   /**< SDO 数据内存的大小。 */
    size_t data_size;                  /**< SDO 数据的大小。 */
    uint8_t complete_access;           /**< 是否完全传输 SDO。 */
    uint32_t issue_timeout;            /**< 请求处理的最大时间（以毫秒为单位）。 */
    uint32_t response_timeout;         /**< 如果从站没有响应，重试传输的最大时间（以毫秒为单位）。 */
    ec_direction_t dir;                /**< 方向。EC_DIR_OUTPUT 表示向从站下载，EC_DIR_INPUT 表示从从站上传。 */
    ec_internal_request_state_t state; /**< SDO 请求的状态。 */
    unsigned long jiffies_start;       /**< 发起请求时的 jiffies 值。 */
    unsigned long jiffies_sent;        /**< 上传/下载请求发送时的 jiffies 值。 */
    int errno;                         /**< 错误编号。 */
    uint32_t abort_code;               /**< SDO 请求的中止码。成功时为零。 */
};

/*****************************************************************************/

void ec_sdo_request_init(ec_sdo_request_t *);
void ec_sdo_request_clear(ec_sdo_request_t *);

int ec_sdo_request_copy(ec_sdo_request_t *, const ec_sdo_request_t *);
int ec_sdo_request_alloc(ec_sdo_request_t *, size_t);
int ec_sdo_request_copy_data(ec_sdo_request_t *, const uint8_t *, size_t);
int ec_sdo_request_timed_out(const ec_sdo_request_t *);

/*****************************************************************************/

#endif

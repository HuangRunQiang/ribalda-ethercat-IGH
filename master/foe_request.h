/******************************************************************************
 *
 *  $Id$
 *
 *  Copyright (C) 2008  Olav Zarges, imc Messsysteme GmbH
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
   EtherCAT FoE请求结构体。
*/

/*****************************************************************************/

#ifndef __EC_FOE_REQUEST_H__
#define __EC_FOE_REQUEST_H__

#include <linux/list.h>

#include "../include/ecrt.h"

#include "globals.h"

/*****************************************************************************/

/** FoE请求。
 */
struct ec_foe_request
{
    struct list_head list; /**< 列表项。 */
    uint8_t *buffer;       /**< 指向FoE数据的指针。 */
    size_t buffer_size;    /**< FoE数据内存的大小。 */
    size_t data_size;      /**< FoE数据的大小。 */
    size_t progress;       /**< BUSY请求的当前位置。 */

    uint32_t issue_timeout;            /**< 处理请求可能花费的最长时间（以毫秒为单位）。 */
    uint32_t response_timeout;         /**< 如果从站没有响应，重试传输的最长时间（以毫秒为单位）。 */
    ec_direction_t dir;                /**< 方向。EC_DIR_OUTPUT表示向从站下载，EC_DIR_INPUT表示从从站上传。 */
    ec_internal_request_state_t state; /**< FoE请求状态。 */
    unsigned long jiffies_start;       /**< 发出请求时的节拍数。 */
    unsigned long jiffies_sent;        /**< 上传/下载请求发送时的节拍数。 */
    uint32_t password;                 /**< FoE密码。 */
    ec_foe_error_t result;             /**< FoE请求中止代码。成功时为零。 */
    uint32_t error_code;               /**< 来自FoE错误请求的错误代码。 */
    uint8_t file_name[255];            /**< FoE文件名。 */
};


/*****************************************************************************/

void ec_foe_request_init(ec_foe_request_t *);
void ec_foe_request_clear(ec_foe_request_t *);

int ec_foe_request_alloc(ec_foe_request_t *, size_t);
int ec_foe_request_copy_data(ec_foe_request_t *, const uint8_t *, size_t);
int ec_foe_request_timed_out(const ec_foe_request_t *);

/*****************************************************************************/

#endif

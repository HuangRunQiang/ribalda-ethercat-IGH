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
   Vendor specific over EtherCAT protocol handler.
*/

/*****************************************************************************/

#ifndef __EC_VOE_HANDLER_H__
#define __EC_VOE_HANDLER_H__

#include <linux/list.h>

#include "globals.h"
#include "datagram.h"

/*****************************************************************************/

/** 厂商特定的EtherCAT处理器。
 */
struct ec_voe_handler
{
    struct list_head list;                     /**< 列表项。*/
    ec_slave_config_t *config;                 /**< 父从站配置。*/
    ec_datagram_t datagram;                    /**< 状态机数据报。*/
    uint32_t vendor_id;                        /**< 头部的厂商ID。*/
    uint16_t vendor_type;                      /**< 头部的厂商类型。*/
    size_t data_size;                          /**< VoE数据的大小。*/
    ec_direction_t dir;                        /**< 方向。EC_DIR_OUTPUT表示写入从站，EC_DIR_INPUT表示从从站读取。*/
    void (*state)(ec_voe_handler_t *);         /**< 状态函数。*/
    ec_internal_request_state_t request_state; /**< 处理器状态。*/
    unsigned int retries;                      /**< 数据报超时时的重试次数。*/
    unsigned long jiffies_start;               /**< 用于计算超时的时间戳。*/
};
/*****************************************************************************/

int ec_voe_handler_init(ec_voe_handler_t *, ec_slave_config_t *, size_t);
void ec_voe_handler_clear(ec_voe_handler_t *);
size_t ec_voe_handler_mem_size(const ec_voe_handler_t *);

/*****************************************************************************/

#endif

/******************************************************************************
 *
 *  $Id$
 *
 *  Copyright (C) 2006-2014  Florian Pose, Ingenieurgemeinschaft IgH
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
   EtherCAT EoE request structure.
*/

/*****************************************************************************/

#ifndef __EC_EOE_REQUEST_H__
#define __EC_EOE_REQUEST_H__

#include <linux/list.h>
#include <linux/etherdevice.h> // ETH_ALEN

#include "globals.h"

/*****************************************************************************/

/** EOE设置IP参数请求结构体 */
typedef struct
{
    struct list_head list;             /**< 链表项 */
    ec_internal_request_state_t state; /**< 请求状态 */
    unsigned long jiffies_sent;        /**< 发送请求时的时间戳 */

    uint8_t mac_address_included; /**< MAC地址是否包含 */
    uint8_t ip_address_included; /**< IP地址是否包含 */
    uint8_t subnet_mask_included; /**< 子网掩码是否包含 */
    uint8_t gateway_included; /**< 网关是否包含 */
    uint8_t dns_included; /**< DNS是否包含 */
    uint8_t name_included; /**< 名称是否包含 */

    unsigned char mac_address[ETH_ALEN]; /**< MAC地址 */
    uint32_t ip_address; /**< IP地址 */
    uint32_t subnet_mask; /**< 子网掩码 */
    uint32_t gateway; /**< 网关 */
    uint32_t dns; /**< DNS */
    char name[EC_MAX_HOSTNAME_SIZE]; /**< 名称 */

    uint16_t result; /**< 结果 */
} ec_eoe_request_t;


/*****************************************************************************/

void ec_eoe_request_init(ec_eoe_request_t *);

/*****************************************************************************/

#endif

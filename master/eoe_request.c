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

/** \file
 * Ethernet-over-EtherCAT request functions.
 */

/*****************************************************************************/

#include <linux/module.h>
#include <linux/jiffies.h>
#include <linux/slab.h>

#include "eoe_request.h"

/*****************************************************************************/

/**
 * @brief 初始化EoE请求。
 * @param req EoE请求。
 * @details 该函数用于初始化EoE请求。它会初始化请求的各个字段，并将状态设置为初始状态。
 */
void ec_eoe_request_init(
    ec_eoe_request_t *req /**< EoE请求。 */
)
{
    INIT_LIST_HEAD(&req->list);
    req->state = EC_INT_REQUEST_INIT;
    req->jiffies_sent = 0U;

    req->mac_address_included = 0;
    req->ip_address_included = 0;
    req->subnet_mask_included = 0;
    req->gateway_included = 0;
    req->dns_included = 0;
    req->name_included = 0;

    memset(req->mac_address, 0x00, ETH_ALEN);
    req->ip_address = 0;
    req->subnet_mask = 0;
    req->gateway = 0;
    req->dns = 0;
    req->name[0] = 0x00;

    req->result = 0x0000;
}

/*****************************************************************************/

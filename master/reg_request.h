/******************************************************************************
 *
 *  $Id$
 *
 *  Copyright (C) 2012  Florian Pose, Ingenieurgemeinschaft IgH
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
   EtherCAT register request structure.
*/

/*****************************************************************************/

#ifndef __EC_REG_REQUEST_H__
#define __EC_REG_REQUEST_H__

#include <linux/list.h>

#include "globals.h"

/*****************************************************************************/

/** 寄存器请求。
 */
struct ec_reg_request
{
    struct list_head list;             /**< 列表项。 */
    size_t mem_size;                   /**< 数据内存大小。 */
    uint8_t *data;                     /**< 数据内存指针。 */
    ec_direction_t dir;                /**< 方向。EC_DIR_OUTPUT 表示向从设备写入数据，EC_DIR_INPUT 表示从从设备读取数据。 */
    uint16_t address;                  /**< 寄存器地址。 */
    size_t transfer_size;              /**< 传输数据的大小。 */
    ec_internal_request_state_t state; /**< 请求状态。 */
    uint16_t ring_position;            /**< 紧急请求的环位置。 */
};

/*****************************************************************************/

int ec_reg_request_init(ec_reg_request_t *, size_t);
void ec_reg_request_clear(ec_reg_request_t *);

/*****************************************************************************/

#endif

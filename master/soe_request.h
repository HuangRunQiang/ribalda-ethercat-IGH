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
   EtherCAT SoE request structure.
*/

/*****************************************************************************/

#ifndef __EC_SOE_REQUEST_H__
#define __EC_SOE_REQUEST_H__

#include <linux/list.h>

#include "globals.h"

/*****************************************************************************/

/** Sercos-over-EtherCAT请求。
 */
typedef struct
{
    struct list_head list;             /**< 列表项。 */
    uint8_t drive_no;                  /**< 驱动号。 */
    uint16_t idn;                      /**< Sercos ID号。 */
    ec_al_state_t al_state;            /**< AL状态（仅对IDN配置有效）。 */
    uint8_t *data;                     /**< 指向SDO数据的指针。 */
    size_t mem_size;                   /**< SDO数据内存的大小。 */
    size_t data_size;                  /**< SDO数据的大小。 */
    ec_direction_t dir;                /**< 方向。EC_DIR_OUTPUT表示向从站写入，EC_DIR_INPUT表示从从站读取。 */
    ec_internal_request_state_t state; /**< 请求状态。 */
    unsigned long jiffies_sent;        /**< 发送上传/下载请求时的jiffies。 */
    uint16_t error_code;               /**< SoE错误代码。 */
} ec_soe_request_t;
/*****************************************************************************/

void ec_soe_request_init(ec_soe_request_t *);
void ec_soe_request_clear(ec_soe_request_t *);

int ec_soe_request_copy(ec_soe_request_t *, const ec_soe_request_t *);
void ec_soe_request_set_drive_no(ec_soe_request_t *, uint8_t);
void ec_soe_request_set_idn(ec_soe_request_t *, uint16_t);
int ec_soe_request_alloc(ec_soe_request_t *, size_t);
int ec_soe_request_copy_data(ec_soe_request_t *, const uint8_t *, size_t);
int ec_soe_request_append_data(ec_soe_request_t *, const uint8_t *, size_t);
void ec_soe_request_read(ec_soe_request_t *);
void ec_soe_request_write(ec_soe_request_t *);

/*****************************************************************************/

#endif

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
   EtherCAT CoE紧急环形缓冲区结构。
*/

/*****************************************************************************/

#ifndef __EC_COE_EMERG_RING_H__
#define __EC_COE_EMERG_RING_H__

#include "globals.h"

/*****************************************************************************/

/** EtherCAT CoE紧急消息记录。
*/
typedef struct
{
u8 data[EC_COE_EMERGENCY_MSG_SIZE]; /**< 消息数据。 */
} ec_coe_emerg_msg_t;

/*****************************************************************************/

/** EtherCAT CoE紧急环形缓冲区。
*/
typedef struct
{
ec_slave_config_t *sc; /**< 拥有该环形缓冲区的从设备配置。 */

ec_coe_emerg_msg_t *msgs; /**< 消息环形缓冲区。 */
size_t size;              /**< 环形缓冲区大小。 */

unsigned int read_index;  /**< 读索引。 */
unsigned int write_index; /**< 写索引。 */
unsigned int overruns;    /**< 上次重置以来的溢出次数。 */

} ec_coe_emerg_ring_t;

/*****************************************************************************/

void ec_coe_emerg_ring_init(ec_coe_emerg_ring_t *, ec_slave_config_t *);
void ec_coe_emerg_ring_clear(ec_coe_emerg_ring_t *);

int ec_coe_emerg_ring_size(ec_coe_emerg_ring_t *, size_t);
void ec_coe_emerg_ring_push(ec_coe_emerg_ring_t *, const u8 *);
int ec_coe_emerg_ring_pop(ec_coe_emerg_ring_t *, u8 *);
int ec_coe_emerg_ring_clear_ring(ec_coe_emerg_ring_t *);
int ec_coe_emerg_ring_overruns(ec_coe_emerg_ring_t *);

/*****************************************************************************/

#endif

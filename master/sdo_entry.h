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
   EtherCAT CANopen SDO entry structure.
*/

/*****************************************************************************/

#ifndef __EC_SDO_ENTRY_H__
#define __EC_SDO_ENTRY_H__

#include <linux/list.h>
#include <linux/kobject.h>

#include "globals.h"

/*****************************************************************************/

struct ec_sdo;
typedef struct ec_sdo ec_sdo_t; /**< \see ec_sdo. */

/*****************************************************************************/

/** CANopen SDO 条目。
 */
typedef struct
{
    struct list_head list;                           /**< 链表项。 */
    ec_sdo_t *sdo;                                   /**< 父级 SDO。 */
    uint8_t subindex;                                /**< 子索引。 */
    uint16_t data_type;                              /**< 数据类型。 */
    uint16_t bit_length;                             /**< 数据位长度。 */
    uint8_t read_access[EC_SDO_ENTRY_ACCESS_COUNT];  /**< 读访问权限。 */
    uint8_t write_access[EC_SDO_ENTRY_ACCESS_COUNT]; /**< 写访问权限。 */
    char *description;                               /**< 描述。 */
} ec_sdo_entry_t;

/*****************************************************************************/

void ec_sdo_entry_init(ec_sdo_entry_t *, ec_sdo_t *, uint8_t);
void ec_sdo_entry_clear(ec_sdo_entry_t *);

/*****************************************************************************/

#endif

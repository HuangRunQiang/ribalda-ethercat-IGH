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
   CANopen over EtherCAT SDO entry functions.
*/

/*****************************************************************************/

#include <linux/slab.h>

#include "sdo_entry.h"

/*****************************************************************************/

/**
 * @brief 初始化 SDO 条目。
 * @param entry SDO 条目的指针。
 * @param sdo 父级 SDO 的指针。
 * @param subindex 子索引。
 * @details
 * - 将父级 SDO 的指针赋值给 SDO 条目。
 * - 设置条目的子索引、数据类型和位长度。
 * - 初始化条目的读访问权限和写访问权限。
 * - 将描述设置为 NULL。
 */
void ec_sdo_entry_init(
    ec_sdo_entry_t *entry, /**< SDO 条目。 */
    ec_sdo_t *sdo,         /**< 父级 SDO。 */
    uint8_t subindex       /**< 子索引。 */
)
{
        entry->sdo = sdo;
        entry->subindex = subindex;
        entry->data_type = 0x0000;
        entry->bit_length = 0;
        entry->read_access[EC_SDO_ENTRY_ACCESS_PREOP] = 0;
        entry->read_access[EC_SDO_ENTRY_ACCESS_SAFEOP] = 0;
        entry->read_access[EC_SDO_ENTRY_ACCESS_OP] = 0;
        entry->write_access[EC_SDO_ENTRY_ACCESS_PREOP] = 0;
        entry->write_access[EC_SDO_ENTRY_ACCESS_SAFEOP] = 0;
        entry->write_access[EC_SDO_ENTRY_ACCESS_OP] = 0;
        entry->description = NULL;
}

/*****************************************************************************/

/**
 * @brief 清除 SDO 条目。
 * @param entry SDO 条目的指针。
 * @details
 * - 如果描述不为空，释放描述的内存。
 */
void ec_sdo_entry_clear(
    ec_sdo_entry_t *entry /**< SDO 条目。 */
)
{
        if (entry->description)
                kfree(entry->description);
}

/*****************************************************************************/

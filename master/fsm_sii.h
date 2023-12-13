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
   EtherCAT从站信息接口FSM结构。

*/

/*****************************************************************************/

#ifndef __EC_FSM_SII_H__
#define __EC_FSM_SII_H__

#include "globals.h"
#include "datagram.h"
#include "slave.h"

/*****************************************************************************/

/** SII访问寻址模式。
 */
typedef enum
{
    EC_FSM_SII_USE_INCREMENT_ADDRESS, /**< 使用自动增量寻址。 */
    EC_FSM_SII_USE_CONFIGURED_ADDRESS /**< 使用配置的地址。 */
} ec_fsm_sii_addressing_t;

/*****************************************************************************/

typedef struct ec_fsm_sii ec_fsm_sii_t; /**< \see ec_fsm_sii */

/**
   从站信息接口FSM。
*/

struct ec_fsm_sii
{
    ec_slave_t *slave;       /**< 运行FSM的从站 */
    ec_datagram_t *datagram; /**< 状态机中使用的数据报 */
    unsigned int retries;    /**< 数据报超时时的重试次数 */

    void (*state)(ec_fsm_sii_t *, ec_datagram_t *); /**< SII状态函数 */
    uint16_t word_offset;                           /**< 输入：SII中的字偏移量 */
    ec_fsm_sii_addressing_t mode;                   /**< 通过APRD或NPRD进行读取 */
    uint8_t value[4];                               /**< 原始SII值（32位） */
    unsigned long jiffies_start;                    /**< 开始时间戳。 */
    uint8_t check_once_more;                        /**< 超时后再尝试一次 */
    uint8_t eeprom_load_retry;                      /**< 等待EEPROM加载 */
};

/*****************************************************************************/

void ec_fsm_sii_init(ec_fsm_sii_t *);
void ec_fsm_sii_clear(ec_fsm_sii_t *);

void ec_fsm_sii_read(ec_fsm_sii_t *, ec_slave_t *,
                     uint16_t, ec_fsm_sii_addressing_t);
void ec_fsm_sii_write(ec_fsm_sii_t *, ec_slave_t *, uint16_t,
                      const uint16_t *, ec_fsm_sii_addressing_t);

int ec_fsm_sii_exec(ec_fsm_sii_t *, ec_datagram_t *);
int ec_fsm_sii_success(ec_fsm_sii_t *);

/*****************************************************************************/

#endif

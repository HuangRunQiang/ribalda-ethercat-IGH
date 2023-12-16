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
   EtherCAT从站扫描状态机。
*/

/*****************************************************************************/

#ifndef __EC_FSM_SLAVE_SCAN_H__
#define __EC_FSM_SLAVE_SCAN_H__

#include "globals.h"
#include "datagram.h"
#include "slave.h"
#include "fsm_sii.h"
#include "fsm_change.h"
#include "fsm_coe.h"
#include "fsm_pdo.h"

/*****************************************************************************/

/** \see ec_fsm_slave_scan */
typedef struct ec_fsm_slave_scan ec_fsm_slave_scan_t;

/** 用于扫描EtherCAT从站的有限状态机。
 */
struct ec_fsm_slave_scan
{
    ec_slave_t *slave;                       /**< 运行FSM的从站。 */
    ec_datagram_t *datagram;                 /**< 在状态机中使用的数据报。 */
    ec_fsm_slave_config_t *fsm_slave_config; /**< 使用的从站配置状态机。 */
    ec_fsm_pdo_t *fsm_pdo;                   /**< 使用的PDO配置状态机。 */
    unsigned int retries;                    /**< 数据报超时的重试次数。 */
    unsigned int scan_retries;               /**< 扫描读取错误的重试次数。 */
    unsigned long scan_jiffies_start;        /**< 扫描重试的起始时间戳。 */

    void (*state)(ec_fsm_slave_scan_t *, ec_datagram_t *); /**< 状态函数。 */
    uint16_t sii_offset;                                   /**< SII偏移量（以字为单位）。 */

    ec_fsm_sii_t fsm_sii; /**< SII状态机。 */

#ifdef EC_SII_OVERRIDE
    const struct firmware *sii_firmware;
#endif
};

/*****************************************************************************/

void ec_fsm_slave_scan_init(ec_fsm_slave_scan_t *, ec_slave_t *,
                            ec_fsm_slave_config_t *, ec_fsm_pdo_t *);
void ec_fsm_slave_scan_clear(ec_fsm_slave_scan_t *);

void ec_fsm_slave_scan_start(ec_fsm_slave_scan_t *);

int ec_fsm_slave_scan_exec(ec_fsm_slave_scan_t *, ec_datagram_t *);
int ec_fsm_slave_scan_success(const ec_fsm_slave_scan_t *);

/*****************************************************************************/

#endif

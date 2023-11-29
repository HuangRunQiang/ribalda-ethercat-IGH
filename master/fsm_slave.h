/******************************************************************************
 *
 *  $Id$
 *
 *  Copyright (C) 2006-2012  Florian Pose, Ingenieurgemeinschaft IgH
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
   EtherCAT slave request state machine.
*/

/*****************************************************************************/

#ifndef __EC_FSM_SLAVE_H__
#define __EC_FSM_SLAVE_H__

#include "globals.h"
#include "datagram.h"
#include "sdo_request.h"
#include "reg_request.h"
#include "eoe_request.h"
#include "mbox_gateway_request.h"
#include "dict_request.h"
#include "fsm_coe.h"
#include "fsm_foe.h"
#include "fsm_soe.h"
#ifdef EC_EOE
#include "fsm_eoe.h"
#endif
#include "fsm_mbox_gateway.h"
#include "fsm_slave_config.h"
#include "fsm_slave_scan.h"

/*****************************************************************************/

typedef struct ec_fsm_slave ec_fsm_slave_t; /**< \see ec_fsm_slave */

/** EtherCAT从站的有限状态机。
 */
struct ec_fsm_slave
{
    ec_slave_t *slave;                  /**< 运行FSM的从站 */
    struct list_head list;              /**< 用于执行列表。 */
    ec_dict_request_t int_dict_request; /**< 内部字典请求。 */

    void (*state)(ec_fsm_slave_t *, ec_datagram_t *); /**< 状态函数。 */
    ec_datagram_t *datagram;                          /**< 之前状态的数据报。 */
    ec_sdo_request_t *sdo_request;                    /**< 要处理的SDO请求。 */
    ec_reg_request_t *reg_request;                    /**< 要处理的寄存器请求。 */
    ec_foe_request_t *foe_request;                    /**< 要处理的FoE请求。 */
    off_t foe_index;                                  /**< FoE写请求数据的索引。 */
    ec_soe_request_t *soe_request;                    /**< 要处理的SoE请求。 */
#ifdef EC_EOE
    ec_eoe_request_t *eoe_request; /**< 要处理的EoE请求。 */
#endif
    ec_mbg_request_t *mbg_request;   /**< 要处理的MBox Gateway请求。 */
    ec_dict_request_t *dict_request; /**< 要处理的字典请求。 */

    ec_fsm_coe_t fsm_coe; /**< CoE状态机。 */
    ec_fsm_foe_t fsm_foe; /**< FoE状态机。 */
    ec_fsm_soe_t fsm_soe; /**< SoE状态机。 */
#ifdef EC_EOE
    ec_fsm_eoe_t fsm_eoe; /**< EoE状态机。 */
#endif
    ec_fsm_mbg_t fsm_mbg;                   /**< MBox Gateway状态机。 */
    ec_fsm_pdo_t fsm_pdo;                   /**< PDO配置状态机。 */
    ec_fsm_change_t fsm_change;             /**< 状态改变状态机 */
    ec_fsm_slave_scan_t fsm_slave_scan;     /**< 从站扫描状态机 */
    ec_fsm_slave_config_t fsm_slave_config; /**< 从站配置状态机。 */
};

/*****************************************************************************/

void ec_fsm_slave_init(ec_fsm_slave_t *, ec_slave_t *); // 初始化从站的有限状态机
void ec_fsm_slave_clear(ec_fsm_slave_t *); // 清除从站的有限状态机

int ec_fsm_slave_exec(ec_fsm_slave_t *, ec_datagram_t *); // 执行从站的有限状态机
void ec_fsm_slave_set_ready(ec_fsm_slave_t *); // 设置从站的有限状态机为准备就绪状态
int ec_fsm_slave_set_unready(ec_fsm_slave_t *); // 设置从站的有限状态机为未准备就绪状态
int ec_fsm_slave_is_ready(const ec_fsm_slave_t *); // 检查从站的有限状态机是否准备就绪
#endif


/*****************************************************************************/

#endif // __EC_FSM_SLAVE_H__

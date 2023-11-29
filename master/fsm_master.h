/******************************************************************************
 *
 *  $Id$
 *
 *  版权所有 (C) 2006-2012 Florian Pose, Ingenieurgemeinschaft IgH
 *
 *  本文件是IgH EtherCAT主站的一部分。
 *
 *  IgH EtherCAT主站是自由软件；您可以在自由软件基金会发布的GNU通用公共许可证第2版下重新分发和修改它。
 *
 *  IgH EtherCAT主站希望它是有用的，但不提供任何担保，也不包含任何隐含的担保，
 *  包括适销性或特定用途的适用性。有关更多详细信息，请参阅GNU通用公共许可证。
 *
 *  您应该随着IgH EtherCAT主站一起收到GNU通用公共许可证的副本；如果没有，
 *  请写信给Free Software Foundation, Inc.，51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA。
 *
 *  ---
 *
 *  上述许可证仅涉及源代码。在遵守Beckhoff Automation GmbH的工业产权和类似权利的前提下，
 *  才允许使用EtherCAT技术和品牌。
 *
 *****************************************************************************/

/**
   \file
   EtherCAT主站状态机。
*/

/*****************************************************************************/

#ifndef __EC_FSM_MASTER_H__
#define __EC_FSM_MASTER_H__

#include "globals.h"
#include "datagram.h"
#include "foe_request.h"
#include "sdo_request.h"
#include "soe_request.h"
#include "mbox_gateway_request.h"
#include "fsm_reboot.h"

/*****************************************************************************/

/** SII写入请求。
 */
typedef struct
{
    struct list_head list;             /**< 链表头。 */
    ec_slave_t *slave;                 /**< EtherCAT从站。 */
    uint16_t offset;                   /**< SII字偏移量。 */
    size_t nwords;                     /**< 字数量。 */
    const uint16_t *words;             /**< 数据字指针。 */
    ec_internal_request_state_t state; /**< 请求状态。 */
} ec_sii_write_request_t;

/*****************************************************************************/

typedef struct ec_fsm_master ec_fsm_master_t; /**< \see ec_fsm_master */

/** EtherCAT主站的有限状态机。
 */
struct ec_fsm_master
{
    ec_master_t *master;     /**< 运行FSM的主站 */
    ec_datagram_t *datagram; /**< 状态机中使用的数据报文 */
    unsigned int retries;    /**< 数据报文超时的重试次数。 */

    void (*state)(ec_fsm_master_t *);                   /**< 主站状态函数 */
    ec_device_index_t dev_idx;                          /**< 当前设备索引（用于扫描等操作）。*/
    int idle;                                           /**< 状态机处于空闲阶段 */
    unsigned long scan_jiffies;                         /**< 从站扫描开始时间 */
    uint8_t link_state[EC_MAX_NUM_DEVICES];             /**< 每个设备的最后链路状态。 */
    unsigned int slaves_responding[EC_MAX_NUM_DEVICES]; /**< 每个设备的响应从站数量。 */
    unsigned int rescan_required;                       /**< 需要重新扫描总线。 */
    ec_slave_state_t slave_states[EC_MAX_NUM_DEVICES];  /**< 每个设备响应从站的AL状态。 */
    ec_slave_t *slave;                                  /**< 当前从站 */
    ec_sii_write_request_t *sii_request;                /**< SII写入请求 */
    off_t sii_index;                                    /**< SII写入请求数据的索引 */

    ec_fsm_reboot_t fsm_reboot; /**< 从站重启状态机 */
    ec_fsm_sii_t fsm_sii;       /**< SII状态机 */
};

/*****************************************************************************/

void ec_fsm_master_init(ec_fsm_master_t *, ec_master_t *, ec_datagram_t *);
void ec_fsm_master_clear(ec_fsm_master_t *);

void ec_fsm_master_reset(ec_fsm_master_t *);

int ec_fsm_master_exec(ec_fsm_master_t *);
int ec_fsm_master_idle(const ec_fsm_master_t *);

/*****************************************************************************/

#endif

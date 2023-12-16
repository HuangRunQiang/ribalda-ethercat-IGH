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
 *  vim: expandtab
 *
 *****************************************************************************/

/**
   \file
   EtherCAT slave configuration methods.
*/

/*****************************************************************************/

#include <linux/module.h>
#include <linux/slab.h>

#include "globals.h"
#include "master.h"
#include "voe_handler.h"

#include "slave_config.h"

/*****************************************************************************/

/**
 * @brief 从配置中获取地址的从站对象。
 * @param sc 从站配置的指针。
 * @return 返回从站对象的指针。
 *         如果未找到匹配的从站对象，则返回 NULL。
 */
ec_slave_t *ec_master_find_slave(
    ec_master_t *master, /**< EtherCAT 主站。 */
    uint16_t alias,      /**< 从站别名。 */
    uint16_t position    /**< 从站位置。 */
)
{
    ec_slave_t *slave;

    list_for_each_entry(slave, &master->slaves, list)
    {
        if (slave->alias == alias && slave->ring_position == position)
            return slave;
    }

    return NULL;
}

/*****************************************************************************/

/**
 * @brief 从站配置构造函数。
 * @param sc 从站配置的指针。
 * @param master EtherCAT 主站。
 * @param alias 从站别名。
 * @param position 从站位置。
 * @param vendor_id 期望的厂商 ID。
 * @param product_code 期望的产品码。
 * @details
 * - 初始化从站配置对象的成员变量。
 * - 设置从站配置的主站、别名、位置、厂商 ID 和产品码。
 * - 设置看门狗分频器为默认值。
 * - 设置允许重叠 PDO 为默认值。
 * - 设置看门狗间隔为默认值。
 * - 初始化从站配置的 SDO 配置链表、SDO 请求链表、FoE 请求链表、寄存器请求链表、VoE 处理器链表和 SoE 配置链表。
 * - 初始化紧急消息环。
 */
void ec_slave_config_init(
    ec_slave_config_t *sc, /**< 从站配置。 */
    ec_master_t *master,   /**< EtherCAT 主站。 */
    uint16_t alias,        /**< 从站别名。 */
    uint16_t position,     /**< 从站位置。 */
    uint32_t vendor_id,    /**< 期望的厂商 ID。 */
    uint32_t product_code  /**< 期望的产品码。 */
)
{
    unsigned int i;

    sc->master = master;

    sc->alias = alias;
    sc->position = position;
    sc->vendor_id = vendor_id;
    sc->product_code = product_code;
    sc->watchdog_divider = 0;       // 使用默认值
    sc->allow_overlapping_pdos = 0; // 默认不允许
    sc->watchdog_intervals = 0;     // 使用默认值

    sc->slave = NULL;

    for (i = 0; i < EC_MAX_SYNC_MANAGERS; i++)
        ec_sync_config_init(&sc->sync_configs[i]);

    sc->used_fmmus = 0;
    sc->dc_assign_activate = 0x0000;
    sc->dc_sync[0].cycle_time = 0U;
    sc->dc_sync[1].cycle_time = 0;
    sc->dc_sync[0].shift_time = 0U;
    sc->dc_sync[1].shift_time = 0;

    INIT_LIST_HEAD(&sc->sdo_configs);
    INIT_LIST_HEAD(&sc->sdo_requests);
    INIT_LIST_HEAD(&sc->foe_requests);
    INIT_LIST_HEAD(&sc->reg_requests);
    INIT_LIST_HEAD(&sc->voe_handlers);
    INIT_LIST_HEAD(&sc->soe_configs);

    ec_coe_emerg_ring_init(&sc->emerg_ring, sc);
}

/*****************************************************************************/

/**
 * @brief 从站配置析构函数。
 * @param sc 从站配置的指针。
 * @details
 * - 清除并释放从站配置对象。
 * - 断开从站配置与从站的关联。
 * - 释放所有同步管理器的内存。
 * - 释放所有 SDO 配置的内存。
 * - 释放所有 SDO 请求的内存。
 * - 释放所有 FoE 请求的内存。
 * - 释放所有寄存器请求的内存。
 * - 释放所有 VoE 处理器的内存。
 * - 释放所有 SoE 配置的内存。
 * - 清除紧急消息环。
 */
void ec_slave_config_clear(
    ec_slave_config_t *sc /**< 从站配置。 */
)
{
    unsigned int i;
    ec_sdo_request_t *req, *next_req;
    ec_foe_request_t *foe, *next_foe;
    ec_voe_handler_t *voe, *next_voe;
    ec_reg_request_t *reg, *next_reg;
    ec_soe_request_t *soe, *next_soe;

    ec_slave_config_detach(sc);

    // 释放所有同步管理器
    for (i = 0; i < EC_MAX_SYNC_MANAGERS; i++)
        ec_sync_config_clear(&sc->sync_configs[i]);

    // 释放所有 SDO 配置
    list_for_each_entry_safe(req, next_req, &sc->sdo_configs, list)
    {
        list_del(&req->list);
        ec_sdo_request_clear(req);
        kfree(req);
    }

    // 释放所有 SDO 请求
    list_for_each_entry_safe(req, next_req, &sc->sdo_requests, list)
    {
        list_del(&req->list);
        ec_sdo_request_clear(req);
        kfree(req);
    }

    // 释放所有 FoE 请求
    list_for_each_entry_safe(foe, next_foe, &sc->foe_requests, list)
    {
        list_del(&foe->list);
        ec_foe_request_clear(foe);
        kfree(foe);
    }

    // 释放所有寄存器请求
    list_for_each_entry_safe(reg, next_reg, &sc->reg_requests, list)
    {
        list_del(&reg->list);
        ec_reg_request_clear(reg);
        kfree(reg);
    }

    // 释放所有 VoE 处理器
    list_for_each_entry_safe(voe, next_voe, &sc->voe_handlers, list)
    {
        list_del(&voe->list);
        ec_voe_handler_clear(voe);
        kfree(voe);
    }

    // 释放所有 SoE 配置
    list_for_each_entry_safe(soe, next_soe, &sc->soe_configs, list)
    {
        list_del(&soe->list);
        ec_soe_request_clear(soe);
        kfree(soe);
    }

    ec_coe_emerg_ring_clear(&sc->emerg_ring);
}

/*****************************************************************************/

/**
 * @brief 准备 FMMU 配置。
 * @param sc 从站配置的指针。
 * @param domain 域。
 * @param sync_index 同步管理器索引。
 * @param dir PDO 方向。
 * @return 成功时返回逻辑偏移字节地址，失败时返回错误码。
 * @details
 * - 将 FMMU 配置数据保存在从站配置结构中，并在配置期间写入从站。
 * - FMMU 配置以覆盖相应同步管理器的完整数据范围的方式进行。
 * - 为每个域配置单独的 FMMU。
 * - 如果 FMMU 配置已准备好，则函数不执行任何操作并返回成功。
 */
int ec_slave_config_prepare_fmmu(
    ec_slave_config_t *sc, /**< 从站配置。 */
    ec_domain_t *domain,   /**< 域。 */
    uint8_t sync_index,    /**< 同步管理器索引。 */
    ec_direction_t dir     /**< PDO 方向。 */
)
{
    unsigned int i;
    ec_fmmu_config_t *fmmu;

    // FMMU 配置已准备好？
    for (i = 0; i < sc->used_fmmus; i++)
    {
        fmmu = &sc->fmmu_configs[i];
        if (fmmu->domain == domain && fmmu->sync_index == sync_index)
            return fmmu->logical_domain_offset;
    }

    if (sc->used_fmmus == EC_MAX_FMMUS)
    {
        EC_CONFIG_ERR(sc, "FMMU 数量已达到上限！\n");
        return -EOVERFLOW;
    }

    fmmu = &sc->fmmu_configs[sc->used_fmmus];

    ec_lock_down(&sc->master->master_sem);
    ec_fmmu_config_init(fmmu, sc, domain, sync_index, dir);

#if 0 // TODO overlapping PDOs
    // Overlapping PDO Support from 4751747d4e6d
    // FIXME FIXME FIXME FIXME FIXME FIXME FIXME FIXME FIXME
    // parent code does not call ec_fmmu_config_domain
    // FIXME FIXME FIXME FIXME FIXME FIXME FIXME FIXME FIXME
    fmmu_logical_start_address = domain->tx_size;
    tx_size = fmmu->data_size;

    // FIXME is it enough to take only the *previous* FMMU into account?

    // FIXME Need to qualify allow_overlapping_pdos with slave->sii.general_flags.enable_not_lrw

    if (sc->allow_overlapping_pdos && sc->used_fmmus > 0) {
        prev_fmmu = &sc->fmmu_configs[sc->used_fmmus - 1];
        if (fmmu->dir != prev_fmmu->dir && prev_fmmu->tx_size != 0) {
            // prev fmmu has opposite direction
            // and is not already paired with prev-prev fmmu
            old_prev_tx_size = prev_fmmu->tx_size;
            prev_fmmu->tx_size = max(fmmu->data_size, prev_fmmu->data_size);
            domain->tx_size += prev_fmmu->tx_size - old_prev_tx_size;
            tx_size = 0;
            fmmu_logical_start_address = prev_fmmu->logical_domain_offset;
        }
    }

    ec_fmmu_config_domain(fmmu, domain, fmmu_logical_start_address, tx_size);
    // Overlapping PDO Support from 4751747d4e6d
#endif

    sc->used_fmmus++;
    ec_lock_up(&sc->master->master_sem);

    return fmmu->logical_domain_offset;
}

/*****************************************************************************/

/**
 * @brief 将配置附加到从站对象。
 * @param sc 从站配置的指针。
 * @return 成功时返回 0。
 *         失败时返回错误码。
 */
int ec_slave_config_attach(
    ec_slave_config_t *sc /**< 从站配置。 */
)
{
    ec_slave_t *slave;

    if (sc->slave)
        return 0; // 已经附加

    if (!(slave = ec_master_find_slave(
              sc->master, sc->alias, sc->position)))
    {
        EC_CONFIG_DBG(sc, 1, "未找到配置的从站。\n");
        return -ENOENT;
    }

    if (slave->config)
    {
        EC_CONFIG_DBG(sc, 1, "无法附加配置。从站 %s-%u"
                             " 已经有一个配置！\n",
                      ec_device_names[slave->device_index != 0], slave->ring_position);
        return -EEXIST;
    }

    if (!slave->sii_image)
    {
        EC_CONFIG_DBG(sc, 1, "从站无法访问其 SII 数据！\n");
        return -EAGAIN;
    }

    if (
#ifdef EC_IDENT_WILDCARDS
        sc->vendor_id != 0xffffffff &&
#endif
        slave->sii_image->sii.vendor_id != sc->vendor_id)
    {
        EC_CONFIG_DBG(sc, 1, "从站 %s-%u 的厂商 ID (0x%08X)"
                             " 与配置的厂商 ID (0x%08X) 不匹配。\n",
                      ec_device_names[slave->device_index != 0], slave->ring_position,
                      slave->sii_image->sii.vendor_id, sc->vendor_id);
        return -EINVAL;
    }

    if (
#ifdef EC_IDENT_WILDCARDS
        sc->product_code != 0xffffffff &&
#endif
        slave->sii_image->sii.product_code != sc->product_code)
    {
        EC_CONFIG_DBG(sc, 1, "从站 %s-%u 的产品码 (0x%08X)"
                             " 与配置的产品码 (0x%08X) 不匹配。\n",
                      ec_device_names[slave->device_index != 0], slave->ring_position,
                      slave->sii_image->sii.product_code, sc->product_code);
        return -EINVAL;
    }

    // 附加从站
    slave->config = sc;
    sc->slave = slave;

    EC_CONFIG_DBG(sc, 1, "附加从站 %s-%u。\n",
                  ec_device_names[slave->device_index != 0], slave->ring_position);
    return 0;
}

/*****************************************************************************/

/**
 * @brief 从站配置解除关联。
 * @param sc 从站配置的指针。
 */
void ec_slave_config_detach(
    ec_slave_config_t *sc /**< 从站配置。 */
)
{
    if (sc->slave)
    {
        ec_reg_request_t *reg;

        sc->slave->config = NULL;

        // 使正在处理的寄存器请求无效
        list_for_each_entry(reg, &sc->reg_requests, list)
        {
            if (sc->slave->fsm.reg_request == reg)
            {
                sc->slave->fsm.reg_request = NULL;
                EC_SLAVE_WARN(sc->slave, "中止寄存器请求，从站正在解除关联。\n");
                reg->state = EC_INT_REQUEST_FAILURE;
                wake_up_all(&sc->slave->master->request_queue);
                break;
            }
        }

        sc->slave = NULL;
    }
}

/*****************************************************************************/

/**
 * @brief 加载从对象的默认PDO分配。
 * @param sc 从对象的配置。
 * @return 无。
 * @details 
 * - 遍历从对象的同步管理器。
 * - 获取同步管理器的方向和PDO列表。
 */
void ec_slave_config_load_default_sync_config(ec_slave_config_t *sc)
{
    uint8_t sync_index;
    ec_sync_config_t *sync_config;
    const ec_sync_t *sync;

    if (!sc->slave)
        return;

    for (sync_index = 0; sync_index < EC_MAX_SYNC_MANAGERS; sync_index++)
    {
        sync_config = &sc->sync_configs[sync_index];
        if ((sync = ec_slave_get_sync(sc->slave, sync_index)))
        {
            sync_config->dir = ec_sync_default_direction(sync);
            if (sync_config->dir == EC_DIR_INVALID)
                EC_SLAVE_WARN(sc->slave,
                              "SM%u的方向字段无效！\n", sync_index);
            ec_pdo_list_copy(&sync_config->pdos, &sync->pdos);
        }
    }
}

/*****************************************************************************/

/**
 * @brief 加载从对象的默认PDO映射。
 * @param sc 从对象的配置。
 * @param pdo PDO对象。
 * @return 无。
 * @details 
 * - 遍历从对象的SII数据中的同步管理器。
 * - 在同步管理器中查找与给定PDO对象相匹配的PDO。
 * - 如果找到匹配的PDO：
 *   - 如果PDO有名称，则使用该名称。
 *   - 复制默认PDO的映射到给定的PDO对象。
 */
void ec_slave_config_load_default_mapping(
    const ec_slave_config_t *sc,
    ec_pdo_t *pdo)
{
    unsigned int i;
    const ec_sync_t *sync;
    const ec_pdo_t *default_pdo;

    if (!sc->slave)
        return;

    EC_CONFIG_DBG(sc, 1, "加载PDO 0x%04X的默认映射。\n",
                  pdo->index);

    if (!sc->slave->sii_image)
    {
        EC_CONFIG_DBG(sc, 1, "从对象无法访问其SII数据！\n");
        return;
    }

    // 在任何同步管理器中查找PDO（稍后可能会重新分配）
    for (i = 0; i < sc->slave->sii_image->sii.sync_count; i++)
    {
        sync = &sc->slave->sii_image->sii.syncs[i];

        list_for_each_entry(default_pdo, &sync->pdos.list, list)
        {
            if (default_pdo->index != pdo->index)
                continue;

            if (default_pdo->name)
            {
                EC_CONFIG_DBG(sc, 1, "找到PDO名称为“%s”。\n",
                              default_pdo->name);

                // 使用已分配的PDO名称
                ec_pdo_set_name(pdo, default_pdo->name);
            }

            // 复制条目（即默认PDO映射）
            if (ec_pdo_copy_entries(pdo, default_pdo))
                return;

            if (sc->master->debug_level)
            {
                const ec_pdo_entry_t *entry;
                list_for_each_entry(entry, &pdo->entries, list)
                {
                    EC_CONFIG_DBG(sc, 1, "条目0x%04X:%02X。\n",
                                  entry->index, entry->subindex);
                }
            }

            return;
        }
    }

    EC_CONFIG_DBG(sc, 1, "未找到默认映射。\n");
}

/*****************************************************************************/

/**
 * @brief 获取SDO配置的数量。
 * @param sc 从对象的配置。
 * @return SDO配置的数量。
 */
unsigned int ec_slave_config_sdo_count(
    const ec_slave_config_t *sc /**< 从对象的配置。 */
)
{
    const ec_sdo_request_t *req;
    unsigned int count = 0;

    list_for_each_entry(req, &sc->sdo_configs, list)
    {
        count++;
    }

    return count;
}

/*****************************************************************************/

/**
 * @brief 通过在列表中的位置查找SDO配置。
 * @param sc 从对象的配置。
 * @param pos 列表中的位置。
 * @return 搜索结果，或者NULL。
 */
const ec_sdo_request_t *ec_slave_config_get_sdo_by_pos_const(
    const ec_slave_config_t *sc, /**< 从对象的配置。 */
    unsigned int pos             /**< 列表中的位置。 */
)
{
    const ec_sdo_request_t *req;

    list_for_each_entry(req, &sc->sdo_configs, list)
    {
        if (pos--)
            continue;
        return req;
    }

    return NULL;
}

/*****************************************************************************/

/**
 * @brief 获取IDN配置的数量。
 * @param sc 从对象的配置。
 * @return IDN配置的数量。
 */
unsigned int ec_slave_config_idn_count(
    const ec_slave_config_t *sc /**< 从对象的配置。 */
)
{
    const ec_soe_request_t *req;
    unsigned int count = 0;

    list_for_each_entry(req, &sc->soe_configs, list)
    {
        count++;
    }

    return count;
}

/*****************************************************************************/

/**
 * @brief 通过在列表中的位置查找IDN配置。
 * @param sc 从对象的配置。
 * @param pos 列表中的位置。
 * @return 搜索结果，或者NULL。
 */
const ec_soe_request_t *ec_slave_config_get_idn_by_pos_const(
    const ec_slave_config_t *sc, /**< 从对象的配置。 */
    unsigned int pos             /**< 列表中的位置。 */
)
{
    const ec_soe_request_t *req;

    list_for_each_entry(req, &sc->soe_configs, list)
    {
        if (pos--)
            continue;
        return req;
    }

    return NULL;
}

/*****************************************************************************/

/**
 * @brief 通过在列表中的位置查找CoE处理程序。
 * @param sc 从对象的配置。
 * @param pos 列表中的位置。
 * @return 搜索结果，或者NULL。
 */
ec_sdo_request_t *ec_slave_config_find_sdo_request(
    ec_slave_config_t *sc, /**< 从对象的配置。 */
    unsigned int pos       /**< 列表中的位置。 */
)
{
    ec_sdo_request_t *req;

    list_for_each_entry(req, &sc->sdo_requests, list)
    {
        if (pos--)
            continue;
        return req;
    }

    return NULL;
}

/*****************************************************************************/

/**
 * @brief 通过在列表中的位置查找FoE处理程序。
 * @param sc 从对象的配置。
 * @param pos 列表中的位置。
 * @return 搜索结果，或者NULL。
 */
ec_foe_request_t *ec_slave_config_find_foe_request(
    ec_slave_config_t *sc, /**< 从对象的配置。 */
    unsigned int pos       /**< 列表中的位置。 */
)
{
    ec_foe_request_t *req;

    list_for_each_entry(req, &sc->foe_requests, list)
    {
        if (pos--)
            continue;
        return req;
    }

    return NULL;
}

/*****************************************************************************/

/**
 * @brief 通过在列表中的位置查找寄存器处理程序。
 * @param sc 从对象的配置。
 * @param pos 列表中的位置。
 * @return 搜索结果，或者NULL。
 */
ec_reg_request_t *ec_slave_config_find_reg_request(
    ec_slave_config_t *sc, /**< 从对象的配置。 */
    unsigned int pos       /**< 列表中的位置。 */
)
{
    ec_reg_request_t *reg;

    list_for_each_entry(reg, &sc->reg_requests, list)
    {
        if (pos--)
            continue;
        return reg;
    }

    return NULL;
}

/*****************************************************************************/

/**
 * @brief 通过在列表中的位置查找VoE处理程序。
 * @param sc 从对象的配置。
 * @param pos 列表中的位置。
 * @return 搜索结果，或者NULL。
 */
ec_voe_handler_t *ec_slave_config_find_voe_handler(
    ec_slave_config_t *sc, /**< 从对象的配置。 */
    unsigned int pos       /**< 列表中的位置。 */
)
{
    ec_voe_handler_t *voe;

    list_for_each_entry(voe, &sc->voe_handlers, list)
    {
        if (pos--)
            continue;
        return voe;
    }

    return NULL;
}

/*****************************************************************************/

/**
 * @brief 在已分离的从对象上过期任何已启动的请求。
 * @param sc 从对象的配置。
 * @return 无。
 * @details 
 * - 遍历从对象的SDO请求列表。
 * - 对于已排队或忙碌的SDO请求，将其状态设置为失败。
 * - 遍历从对象的FoE请求列表。
 * - 对于已排队或忙碌的FoE请求，将其状态设置为失败。
 * - 遍历从对象的寄存器请求列表。
 * - 对于已排队或忙碌的寄存器请求，将其状态设置为失败。
 */
void ec_slave_config_expire_disconnected_requests(
    ec_slave_config_t *sc /**< 从对象的配置。 */
)
{
    ec_sdo_request_t *sdo_req;
    ec_foe_request_t *foe_req;
    ec_reg_request_t *reg_req;

    if (sc->slave)
    {
        return;
    }

    list_for_each_entry(sdo_req, &sc->sdo_requests, list)
    {
        if (sdo_req->state == EC_INT_REQUEST_QUEUED ||
            sdo_req->state == EC_INT_REQUEST_BUSY)
        {
            EC_CONFIG_DBG(sc, 1, "中止SDO请求；没有连接的从对象。\n");
            sdo_req->state = EC_INT_REQUEST_FAILURE;
        }
    }

    list_for_each_entry(foe_req, &sc->foe_requests, list)
    {
        if (foe_req->state == EC_INT_REQUEST_QUEUED ||
            foe_req->state == EC_INT_REQUEST_BUSY)
        {
            EC_CONFIG_DBG(sc, 1, "中止FoE请求；没有连接的从对象。\n");
            foe_req->state = EC_INT_REQUEST_FAILURE;
        }
    }

    list_for_each_entry(reg_req, &sc->reg_requests, list)
    {
        if (reg_req->state == EC_INT_REQUEST_QUEUED ||
            reg_req->state == EC_INT_REQUEST_BUSY)
        {
            EC_CONFIG_DBG(sc, 1, "中止寄存器请求；没有连接的从对象。\n");
            reg_req->state = EC_INT_REQUEST_FAILURE;
        }
    }
}

/******************************************************************************
 *  Application interface
 *****************************************************************************/

/**
 * @brief 配置从对象的同步管理器。
 * @param sc 从对象的配置。
 * @param sync_index 同步管理器的索引。
 * @param dir 方向。
 * @param watchdog_mode 看门狗模式。
 * @return 成功返回0，否则返回错误代码。
 * @details 
 * - 检查同步管理器索引是否有效。
 * - 检查方向是否有效。
 * - 配置同步管理器的方向和看门狗模式。
 */
int ecrt_slave_config_sync_manager(ec_slave_config_t *sc, uint8_t sync_index,
                                   ec_direction_t dir, ec_watchdog_mode_t watchdog_mode)
{
    ec_sync_config_t *sync_config;

    EC_CONFIG_DBG(sc, 1, "ecrt_slave_config_sync_manager(sc = 0x%p,"
                         " sync_index = %u, dir = %i, watchdog_mode = %i)\n",
                  sc, sync_index, dir, watchdog_mode);

    if (sync_index >= EC_MAX_SYNC_MANAGERS)
    {
        EC_CONFIG_ERR(sc, "无效的同步管理器索引 %u！\n", sync_index);
        return -ENOENT;
    }

    if (dir != EC_DIR_OUTPUT && dir != EC_DIR_INPUT)
    {
        EC_CONFIG_ERR(sc, "无效的方向 %u！\n", (unsigned int)dir);
        return -EINVAL;
    }

    sync_config = &sc->sync_configs[sync_index];
    sync_config->dir = dir;
    sync_config->watchdog_mode = watchdog_mode;
    return 0;
}

/*****************************************************************************/

/**
 * @brief 配置从对象的看门狗。
 * @param sc 从对象的配置。
 * @param divider 分频器。
 * @param intervals 间隔。
 * @return 无。
 * @details 
 * - 配置从对象的看门狗分频器和间隔。
 */
void ecrt_slave_config_watchdog(ec_slave_config_t *sc,
                                uint16_t divider, uint16_t intervals)
{
    EC_CONFIG_DBG(sc, 1, "%s(sc = 0x%p, divider = %u, intervals = %u)\n",
                  __func__, sc, divider, intervals);

    sc->watchdog_divider = divider;
    sc->watchdog_intervals = intervals;
}

/*****************************************************************************/

/**
 * @brief 配置从对象的重叠PDO。
 * @param sc 从对象的配置。
 * @param allow_overlapping_pdos 允许重叠的PDO。
 * @return 无。
 * @details 
 * - 配置从对象是否允许重叠的PDO。
 */
void ecrt_slave_config_overlapping_pdos(ec_slave_config_t *sc,
                                        uint8_t allow_overlapping_pdos)
{
    EC_CONFIG_DBG(sc, 1, "%s(sc = 0x%p, allow_overlapping_pdos = %u)\n",
                  __func__, sc, allow_overlapping_pdos);

    sc->allow_overlapping_pdos = allow_overlapping_pdos;
}

/*****************************************************************************/

/**
 * @brief 添加从对象的PDO分配。
 * @param sc 从对象的配置。
 * @param sync_index 同步管理器的索引。
 * @param pdo_index PDO的索引。
 * @return 成功返回0，否则返回错误代码。
 * @details 
 * - 检查同步管理器索引是否有效。
 * - 锁定主站信号量。
 * - 添加PDO到同步管理器的PDO列表。
 * - 设置PDO的同步管理器索引。
 * - 加载默认的PDO映射。
 * - 解锁主站信号量。
 */
int ecrt_slave_config_pdo_assign_add(ec_slave_config_t *sc,
                                     uint8_t sync_index, uint16_t pdo_index)
{
    ec_pdo_t *pdo;

    EC_CONFIG_DBG(sc, 1, "%s(sc = 0x%p, sync_index = %u, "
                         "pdo_index = 0x%04X)\n",
                  __func__, sc, sync_index, pdo_index);

    if (sync_index >= EC_MAX_SYNC_MANAGERS)
    {
        EC_CONFIG_ERR(sc, "无效的同步管理器索引 %u！\n", sync_index);
        return -EINVAL;
    }

    ec_lock_down(&sc->master->master_sem);

    pdo = ec_pdo_list_add_pdo(&sc->sync_configs[sync_index].pdos, pdo_index);
    if (IS_ERR(pdo))
    {
        ec_lock_up(&sc->master->master_sem);
        return PTR_ERR(pdo);
    }
    pdo->sync_index = sync_index;

    ec_slave_config_load_default_mapping(sc, pdo);

    ec_lock_up(&sc->master->master_sem);
    return 0;
}

/*****************************************************************************/

/**
 * @brief 清除从对象的PDO分配。
 * @param sc 从对象的配置。
 * @param sync_index 同步管理器的索引。
 * @return 无。
 * @details 
 * - 检查同步管理器索引是否有效。
 * - 锁定主站信号量。
 * - 清除同步管理器的PDO列表。
 * - 解锁主站信号量。
 */
void ecrt_slave_config_pdo_assign_clear(ec_slave_config_t *sc,
                                        uint8_t sync_index)
{
    EC_CONFIG_DBG(sc, 1, "%s(sc = 0x%p, sync_index = %u)\n",
                  __func__, sc, sync_index);

    if (sync_index >= EC_MAX_SYNC_MANAGERS)
    {
        EC_CONFIG_ERR(sc, "无效的同步管理器索引 %u！\n", sync_index);
        return;
    }

    ec_lock_down(&sc->master->master_sem);
    ec_pdo_list_clear_pdos(&sc->sync_configs[sync_index].pdos);
    ec_lock_up(&sc->master->master_sem);
}

/*****************************************************************************/

/**
 * @brief 添加从对象的PDO映射。
 * @param sc 从对象的配置。
 * @param pdo_index PDO的索引。
 * @param entry_index 条目的索引。
 * @param entry_subindex 条目的子索引。
 * @param entry_bit_length 条目的位长度。
 * @return 成功返回0，否则返回错误代码。
 * @details 
 * - 查找与给定PDO索引相匹配的PDO。
 * - 锁定主站信号量。
 * - 添加条目到PDO的条目列表。
 * - 解锁主站信号量。
 */
int ecrt_slave_config_pdo_mapping_add(ec_slave_config_t *sc,
                                      uint16_t pdo_index, uint16_t entry_index, uint8_t entry_subindex,
                                      uint8_t entry_bit_length)
{
    uint8_t sync_index;
    ec_pdo_t *pdo = NULL;
    ec_pdo_entry_t *entry;
    int retval = 0;

    EC_CONFIG_DBG(sc, 1, "%s(sc = 0x%p, "
                         "pdo_index = 0x%04X, entry_index = 0x%04X, "
                         "entry_subindex = 0x%02X, entry_bit_length = %u)\n",
                  __func__, sc, pdo_index, entry_index, entry_subindex,
                  entry_bit_length);

    for (sync_index = 0; sync_index < EC_MAX_SYNC_MANAGERS; sync_index++)
        if ((pdo = ec_pdo_list_find_pdo(
                 &sc->sync_configs[sync_index].pdos, pdo_index)))
            break;

    if (pdo)
    {
        ec_lock_down(&sc->master->master_sem);
        entry = ec_pdo_add_entry(pdo, entry_index, entry_subindex,
                                 entry_bit_length);
        ec_lock_up(&sc->master->master_sem);
        if (IS_ERR(entry))
            retval = PTR_ERR(entry);
    }
    else
    {
        EC_CONFIG_ERR(sc, "PDO 0x%04X未分配。\n", pdo_index);
        retval = -ENOENT;
    }

    return retval;
}

/*****************************************************************************/

/**
 * @brief 清除从对象的PDO映射。
 * @param sc 从对象的配置。
 * @param pdo_index PDO的索引。
 * @return 无。
 * @details 
 * - 查找与给定PDO索引相匹配的PDO。
 * - 锁定主站信号量。
 * - 清除PDO的条目列表。
 * - 解锁主站信号量。
 */
void ecrt_slave_config_pdo_mapping_clear(ec_slave_config_t *sc,
                                         uint16_t pdo_index)
{
    uint8_t sync_index;
    ec_pdo_t *pdo = NULL;

    EC_CONFIG_DBG(sc, 1, "%s(sc = 0x%p, pdo_index = 0x%04X)\n",
                  __func__, sc, pdo_index);

    for (sync_index = 0; sync_index < EC_MAX_SYNC_MANAGERS; sync_index++)
        if ((pdo = ec_pdo_list_find_pdo(
                 &sc->sync_configs[sync_index].pdos, pdo_index)))
            break;

    if (pdo)
    {
        ec_lock_down(&sc->master->master_sem);
        ec_pdo_clear_entries(pdo);
        ec_lock_up(&sc->master->master_sem);
    }
    else
    {
        EC_CONFIG_WARN(sc, "PDO 0x%04X未分配。\n", pdo_index);
    }
}

/*****************************************************************************/

/**
 * @brief 配置从对象的PDO。
 * @param sc 从对象的配置。
 * @param n_syncs 同步管理器的数量。
 * @param syncs 同步管理器的信息。
 * @return 成功返回0，否则返回错误代码。
 * @details 
 * - 遍历同步管理器信息：
 *   - 检查同步管理器索引是否有效。
 *   - 配置同步管理器。
 *   - 清除同步管理器的PDO分配。
 *   - 如果有PDO信息：
 *     - 遍历PDO信息：
 *       - 添加PDO到同步管理器的PDO列表。
 *       - 如果有条目信息：
 *         - 清除PDO的条目列表。
 *         - 遍历条目信息：
 *           - 添加条目到PDO的条目列表。
 */
int ecrt_slave_config_pdos(ec_slave_config_t *sc,
                           unsigned int n_syncs, const ec_sync_info_t syncs[])
{
    int ret;
    unsigned int i, j, k;
    const ec_sync_info_t *sync_info;
    const ec_pdo_info_t *pdo_info;
    const ec_pdo_entry_info_t *entry_info;

    EC_CONFIG_DBG(sc, 1, "%s(sc = 0x%p, n_syncs = %u, syncs = 0x%p)\n",
                  __func__, sc, n_syncs, syncs);

    if (!syncs)
        return 0;

    for (i = 0; i < n_syncs; i++)
    {
        sync_info = &syncs[i];

        if (sync_info->index == (uint8_t)EC_END)
            break;

        if (sync_info->index >= EC_MAX_SYNC_MANAGERS)
        {
            EC_CONFIG_ERR(sc, "无效的同步管理器索引 %u！\n",
                          sync_info->index);
            return -ENOENT;
        }

        ret = ecrt_slave_config_sync_manager(sc, sync_info->index,
                                             sync_info->dir, sync_info->watchdog_mode);
        if (ret)
            return ret;

        ecrt_slave_config_pdo_assign_clear(sc, sync_info->index);

        if (sync_info->n_pdos && sync_info->pdos)
        {

            for (j = 0; j < sync_info->n_pdos; j++)
            {
                pdo_info = &sync_info->pdos[j];

                ret = ecrt_slave_config_pdo_assign_add(
                    sc, sync_info->index, pdo_info->index);
                if (ret)
                    return ret;

                if (pdo_info->n_entries && pdo_info->entries)
                {
                    ecrt_slave_config_pdo_mapping_clear(sc, pdo_info->index);

                    for (k = 0; k < pdo_info->n_entries; k++)
                    {
                        entry_info = &pdo_info->entries[k];

                        ret = ecrt_slave_config_pdo_mapping_add(sc,
                                                                pdo_info->index, entry_info->index,
                                                                entry_info->subindex,
                                                                entry_info->bit_length);
                        if (ret)
                            return ret;
                    }
                }
            }
        }
    }

    return 0;
}

/*****************************************************************************/

/**
 * @brief 配置从对象的寄存器PDO条目。
 * @param sc 从对象的配置。
 * @param index 寄存器的索引。
 * @param subindex 寄存器的子索引。
 * @param domain 领域。
 * @param bit_position 位位置。
 * @return 成功返回寄存器的偏移量，否则返回错误代码。
 * @details 
 * - 遍历同步管理器的PDO：
 *   - 遍历PDO的条目：
 *     - 如果条目的索引和子索引匹配：
 *       - 计算位偏移量。
 *       - 如果有位位置指针，则设置位位置。
 *       - 准备FMMU并返回同步偏移量和位偏移量。
 */
int ecrt_slave_config_reg_pdo_entry(
    ec_slave_config_t *sc,
    uint16_t index,
    uint8_t subindex,
    ec_domain_t *domain,
    unsigned int *bit_position)
{
    uint8_t sync_index;
    const ec_sync_config_t *sync_config;
    unsigned int bit_offset, bit_pos;
    ec_pdo_t *pdo;
    ec_pdo_entry_t *entry;
    int sync_offset;

    EC_CONFIG_DBG(sc, 1, "%s(sc = 0x%p, index = 0x%04X, "
                         "subindex = 0x%02X, domain = 0x%p, bit_position = 0x%p)\n",
                  __func__, sc, index, subindex, domain, bit_position);

    for (sync_index = 0; sync_index < EC_MAX_SYNC_MANAGERS; sync_index++)
    {
        sync_config = &sc->sync_configs[sync_index];
        bit_offset = 0;

        list_for_each_entry(pdo, &sync_config->pdos.list, list)
        {
            list_for_each_entry(entry, &pdo->entries, list)
            {
                if (entry->index != index || entry->subindex != subindex)
                {
                    bit_offset += entry->bit_length;
                }
                else
                {
                    bit_pos = bit_offset % 8;
                    if (bit_position)
                    {
                        *bit_position = bit_pos;
                    }
                    else if (bit_pos)
                    {
                        EC_CONFIG_ERR(sc, "PDO条目0x%04X:%02X不字节对齐。\n",
                                      index, subindex);
                        return -EFAULT;
                    }

                    sync_offset = ec_slave_config_prepare_fmmu(
                        sc, domain, sync_index, sync_config->dir);
                    if (sync_offset < 0)
                        return sync_offset;

                    return sync_offset + bit_offset / 8;
                }
            }
        }
    }

    EC_CONFIG_ERR(sc, "PDO条目0x%04X:%02X未映射。\n",
                  index, subindex);
    return -ENOENT;
}

/*****************************************************************************/

/**
 * @brief 配置从对象的寄存器PDO条目位置。
 * @param sc 从对象的配置。
 * @param sync_index 同步管理器的索引。
 * @param pdo_pos PDO的位置。
 * @param entry_pos 条目的位置。
 * @param domain 领域。
 * @param bit_position 位位置。
 * @return 成功返回寄存器的偏移量，否则返回错误代码。
 * @details 
 * - 检查同步管理器索引是否有效。
 * - 遍历同步管理器的PDO和条目：
 *   - 如果位置不匹配，计算位偏移量。
 *   - 如果位置匹配，计算位偏移量和同步偏移量。
 *   - 如果有位位置指针，则设置位位置。
 *   - 准备FMMU并返回同步偏移量和位偏移量。
 */
int ecrt_slave_config_reg_pdo_entry_pos(
    ec_slave_config_t *sc,
    uint8_t sync_index,
    unsigned int pdo_pos,
    unsigned int entry_pos,
    ec_domain_t *domain,
    unsigned int *bit_position)
{
    const ec_sync_config_t *sync_config;
    unsigned int bit_offset, pp, ep;
    ec_pdo_t *pdo;
    ec_pdo_entry_t *entry;

    EC_CONFIG_DBG(sc, 1, "%s(sc = 0x%p, sync_index = %u, pdo_pos = %u,"
                         " entry_pos = %u, domain = 0x%p, bit_position = 0x%p)\n",
                  __func__, sc, sync_index, pdo_pos, entry_pos,
                  domain, bit_position);

    if (sync_index >= EC_MAX_SYNC_MANAGERS)
    {
        EC_CONFIG_ERR(sc, "无效的同步管理器位置 %u！\n", sync_index);
        return -EINVAL;
    }

    sync_config = &sc->sync_configs[sync_index];
    bit_offset = 0;
    pp = 0;

    list_for_each_entry(pdo, &sync_config->pdos.list, list)
    {
        ep = 0;
        list_for_each_entry(entry, &pdo->entries, list)
        {
            if (pp != pdo_pos || ep != entry_pos)
            {
                bit_offset += entry->bit_length;
            }
            else
            {
                unsigned int bit_pos = bit_offset % 8;
                int sync_offset;

                if (bit_position)
                {
                    *bit_position = bit_pos;
                }
                else if (bit_pos)
                {
                    EC_CONFIG_ERR(sc, "PDO条目0x%04X:%02X不字节对齐。\n",
                                      pdo->index, entry->subindex);
                    return -EFAULT;
                }

                sync_offset = ec_slave_config_prepare_fmmu(
                    sc, domain, sync_index, sync_config->dir);
                if (sync_offset < 0)
                    return sync_offset;

                return sync_offset + bit_offset / 8;
            }
            ep++;
        }
        pp++;
    }

    EC_CONFIG_ERR(sc, "PDO条目规范 %u/%u/%u 超出范围。\n",
                  sync_index, pdo_pos, entry_pos);
    return -ENOENT;
}

/*****************************************************************************/

/**
 * @brief 配置从对象的DC。
 * @param sc 从对象的配置。
 * @param assign_activate 分配激活。
 * @param sync0_cycle_time 同步0的周期时间。
 * @param sync0_shift_time 同步0的偏移时间。
 * @param sync1_cycle_time 同步1的周期时间。
 * @param sync1_shift_time 同步1的偏移时间。
 * @return 无。
 * @details 
 * - 配置从对象的DC分配激活、同步0的周期时间和偏移时间、同步1的周期时间和偏移时间。
 */
void ecrt_slave_config_dc(ec_slave_config_t *sc, uint16_t assign_activate,
                          uint32_t sync0_cycle_time, int32_t sync0_shift_time,
                          uint32_t sync1_cycle_time, int32_t sync1_shift_time)
{
    EC_CONFIG_DBG(sc, 1, "%s(sc = 0x%p, assign_activate = 0x%04X,"
                         " sync0_cycle = %u, sync0_shift = %i,"
                         " sync1_cycle = %u, sync1_shift = %i\n",
                  __func__, sc, assign_activate, sync0_cycle_time, sync0_shift_time,
                  sync1_cycle_time, sync1_shift_time);

    sc->dc_assign_activate = assign_activate;
    sc->dc_sync[0].cycle_time = sync0_cycle_time;
    sc->dc_sync[0].shift_time = sync0_shift_time;
    if (sync0_cycle_time > 0)
    {
        sc->dc_sync[1].shift_time = (sync1_cycle_time + sync1_shift_time) %
                                    sync0_cycle_time;

        if ((sync1_cycle_time + sync1_shift_time) < sc->dc_sync[1].shift_time)
        {
            EC_CONFIG_ERR(sc, "从配置DC结果为负的同步1周期。重置为零周期和偏移时间\n");

            sc->dc_sync[1].cycle_time = 0;
            sc->dc_sync[1].shift_time = 0;
        }
        else
        {
            sc->dc_sync[1].cycle_time = (sync1_cycle_time + sync1_shift_time) -
                                        sc->dc_sync[1].shift_time;
        }
    }
    else
    {
        sc->dc_sync[1].cycle_time = 0;
        sc->dc_sync[1].shift_time = 0;
    }
}

/*****************************************************************************/

/**
 * @brief 配置从对象的SDO。
 * @param sc 从对象的配置。
 * @param index SDO的索引。
 * @param subindex SDO的子索引。
 * @param data 数据。
 * @param size 大小。
 * @return 成功返回0，否则返回错误代码。
 * @details 
 * - 检查从对象是否支持CoE。
 * - 分配SDO请求内存。
 * - 初始化SDO请求。
 * - 设置SDO请求的索引和数据。
 * - 锁定主站信号量。
 * - 添加SDO请求到从对象的SDO配置列表。
 * - 解锁主站信号量。
 */
int ecrt_slave_config_sdo(ec_slave_config_t *sc, uint16_t index,
                          uint8_t subindex, const uint8_t *data, size_t size)
{
    ec_slave_t *slave = sc->slave;
    ec_sdo_request_t *req;
    int ret;

    EC_CONFIG_DBG(sc, 1, "%s(sc = 0x%p, index = 0x%04X, "
                         "subindex = 0x%02X, data = 0x%p, size = %zu)\n",
                  __func__, sc, index, subindex, data, size);

    if (slave && slave->sii_image && !(slave->sii_image->sii.mailbox_protocols & EC_MBOX_COE))
    {
        EC_CONFIG_WARN(sc, "连接的从设备不支持CoE！\n");
    }

    if (!(req = (ec_sdo_request_t *)
              kmalloc(sizeof(ec_sdo_request_t), GFP_KERNEL)))
    {
        EC_CONFIG_ERR(sc, "分配SDO配置的内存失败！\n");
        return -ENOMEM;
    }

    ec_sdo_request_init(req);
    ecrt_sdo_request_index(req, index, subindex);

    ret = ec_sdo_request_copy_data(req, data, size);
    if (ret < 0)
    {
        ec_sdo_request_clear(req);
        kfree(req);
        return ret;
    }

    ec_lock_down(&sc->master->master_sem);
    list_add_tail(&req->list, &sc->sdo_configs);
    ec_lock_up(&sc->master->master_sem);
    return 0;
}

/*****************************************************************************/

/**
 * @brief 配置从对象的SDO8。
 * @param sc 从对象的配置。
 * @param index SDO的索引。
 * @param subindex SDO的子索引。
 * @param value 值。
 * @return 成功返回0，否则返回错误代码。
 * @details 
 * - 调用ecrt_slave_config_sdo()，传递一个字节的数据。
 */
int ecrt_slave_config_sdo8(ec_slave_config_t *sc, uint16_t index,
                           uint8_t subindex, uint8_t value)
{
    uint8_t data[1];

    EC_CONFIG_DBG(sc, 1, "%s(sc = 0x%p, index = 0x%04X, "
                         "subindex = 0x%02X, value = %u)\n",
                  __func__, sc, index, subindex, (unsigned int)value);

    EC_WRITE_U8(data, value);
    return ecrt_slave_config_sdo(sc, index, subindex, data, 1);
}

/*****************************************************************************/

/**
 * @brief 配置从对象的SDO16。
 * @param sc 从对象的配置。
 * @param index SDO的索引。
 * @param subindex SDO的子索引。
 * @param value 值。
 * @return 成功返回0，否则返回错误代码。
 * @details 
 * - 调用ecrt_slave_config_sdo()，传递两个字节的数据。
 */
int ecrt_slave_config_sdo16(ec_slave_config_t *sc, uint16_t index,
                            uint8_t subindex, uint16_t value)
{
    uint8_t data[2];

    EC_CONFIG_DBG(sc, 1, "%s(sc = 0x%p, index = 0x%04X, "
                         "subindex = 0x%02X, value = %u)\n",
                  __func__, sc, index, subindex, value);

    EC_WRITE_U16(data, value);
    return ecrt_slave_config_sdo(sc, index, subindex, data, 2);
}

/*****************************************************************************/

/**
 * @brief 配置从对象的SDO32。
 * @param sc 从对象的配置。
 * @param index SDO的索引。
 * @param subindex SDO的子索引。
 * @param value 值。
 * @return 成功返回0，否则返回错误代码。
 * @details 
 * - 调用ecrt_slave_config_sdo()，传递四个字节的数据。
 */
int ecrt_slave_config_sdo32(ec_slave_config_t *sc, uint16_t index,
                            uint8_t subindex, uint32_t value)
{
    uint8_t data[4];

    EC_CONFIG_DBG(sc, 1, "%s(sc = 0x%p, index = 0x%04X, "
                         "subindex = 0x%02X, value = %u)\n",
                  __func__, sc, index, subindex, value);

    EC_WRITE_U32(data, value);
    return ecrt_slave_config_sdo(sc, index, subindex, data, 4);
}

/*****************************************************************************/

/**
 * @brief 配置从对象的完整SDO。
 * @param sc 从对象的配置。
 * @param index SDO的索引。
 * @param data 数据。
 * @param size 大小。
 * @return 成功返回0，否则返回错误代码。
 * @details 
 * - 检查从对象是否支持CoE。
 * - 分配SDO请求内存。
 * - 初始化SDO请求。
 * - 设置SDO请求的索引和数据。
 * - 锁定主站信号量。
 * - 添加SDO请求到从对象的SDO配置列表。
 * - 解锁主站信号量。
 */
int ecrt_slave_config_complete_sdo(ec_slave_config_t *sc, uint16_t index,
                                   const uint8_t *data, size_t size)
{
    ec_slave_t *slave = sc->slave;
    ec_sdo_request_t *req;
    int ret;

    EC_CONFIG_DBG(sc, 1, "%s(sc = 0x%p, index = 0x%04X, "
                         "data = 0x%p, size = %zu)\n",
                  __func__, sc, index, data, size);

    if (slave && !(slave->sii_image->sii.mailbox_protocols & EC_MBOX_COE))
    {
        EC_CONFIG_WARN(sc, "连接的从设备不支持CoE！\n");
    }

    if (!(req = (ec_sdo_request_t *)
              kmalloc(sizeof(ec_sdo_request_t), GFP_KERNEL)))
    {
        EC_CONFIG_ERR(sc, "分配SDO配置的内存失败！\n");
        return -ENOMEM;
    }

    ec_sdo_request_init(req);
    ecrt_sdo_request_index_complete(req, index);

    ret = ec_sdo_request_copy_data(req, data, size);
    if (ret < 0)
    {
        ec_sdo_request_clear(req);
        kfree(req);
        return ret;
    }

    ec_lock_down(&sc->master->master_sem);
    list_add_tail(&req->list, &sc->sdo_configs);
    ec_lock_up(&sc->master->master_sem);
    return 0;
}

/*****************************************************************************/

/**
 * @brief 配置从对象的紧急大小。
 * @param sc 从对象的配置。
 * @param elements 元素数量。
 * @return 成功返回0，否则返回错误代码。
 * @details 
 * - 调用ec_coe_emerg_ring_size()，设置紧急环的大小。
 */
int ecrt_slave_config_emerg_size(ec_slave_config_t *sc, size_t elements)
{
    return ec_coe_emerg_ring_size(&sc->emerg_ring, elements);
}

/*****************************************************************************/

/**
 * @brief 弹出从对象的紧急数据。
 * @param sc 从对象的配置。
 * @param target 目标指针。
 * @return 成功返回0，否则返回错误代码。
 * @details 
 * - 调用ec_coe_emerg_ring_pop()，弹出紧急环的数据。
 */
int ecrt_slave_config_emerg_pop(ec_slave_config_t *sc, uint8_t *target)
{
    return ec_coe_emerg_ring_pop(&sc->emerg_ring, target);
}

/*****************************************************************************/

/**
 * @brief 清除从对象的紧急环。
 * @param sc 从对象的配置。
 * @return 成功返回0，否则返回错误代码。
 * @details 
 * - 调用ec_coe_emerg_ring_clear_ring()，清除紧急环。
 */
int ecrt_slave_config_emerg_clear(ec_slave_config_t *sc)
{
    return ec_coe_emerg_ring_clear_ring(&sc->emerg_ring);
}

/*****************************************************************************/

/**
 * @brief 获取从对象的紧急环溢出次数。
 * @param sc 从对象的配置。
 * @return 紧急环溢出次数。
 * @details 
 * - 调用ec_coe_emerg_ring_overruns()，获取紧急环溢出次数。
 */
int ecrt_slave_config_emerg_overruns(ec_slave_config_t *sc)
{
    return ec_coe_emerg_ring_overruns(&sc->emerg_ring);
}

/*****************************************************************************/

/**
 * @brief 创建从对象的SDO请求（带错误指针返回值）。
 * @param sc 从对象的配置。
 * @param index SDO的索引。
 * @param subindex SDO的子索引。
 * @param complete 是否为完整SDO。
 * @param size 数据大小。
 * @return SDO请求指针。
 * @details 
 * - 分配SDO请求内存。
 * - 初始化SDO请求。
 * - 设置SDO请求的索引。
 * - 锁定主站信号量。
 * - 添加SDO请求到从对象的SDO请求列表。
 * - 解锁主站信号量。
 */
ec_sdo_request_t *ecrt_slave_config_create_sdo_request_err(
    ec_slave_config_t *sc, uint16_t index, uint8_t subindex, uint8_t complete, size_t size)
{
    ec_sdo_request_t *req;
    int ret;

    EC_CONFIG_DBG(sc, 1, "%s(sc = 0x%p, "
                         "index = 0x%04X, subindex = 0x%02X, complete = %u, size = %zu)\n",
                  __func__, sc, index, subindex, complete, size);

    if (!(req = (ec_sdo_request_t *)
              kmalloc(sizeof(ec_sdo_request_t), GFP_KERNEL)))
    {
        EC_CONFIG_ERR(sc, "分配SDO请求的内存失败！\n");
        return ERR_PTR(-ENOMEM);
    }

    ec_sdo_request_init(req);
    if (complete)
    {
        ecrt_sdo_request_index_complete(req, index);
    }
    else
    {
        ecrt_sdo_request_index(req, index, subindex);
    }
    ret = ec_sdo_request_alloc(req, size);
    if (ret < 0)
    {
        ec_sdo_request_clear(req);
        kfree(req);
        return ERR_PTR(ret);
    }

    // 准备可选写入的数据
    memset(req->data, 0x00, size);
    req->data_size = size;

    ec_lock_down(&sc->master->master_sem);
    list_add_tail(&req->list, &sc->sdo_requests);
    ec_lock_up(&sc->master->master_sem);

    return req;
}

/*****************************************************************************/

/**
 * @brief 创建从对象的SDO请求。
 * @param sc 从对象的配置。
 * @param index SDO的索引。
 * @param subindex SDO的子索引。
 * @param size 数据大小。
 * @return SDO请求指针。
 * @details 
 * - 调用ecrt_slave_config_create_sdo_request_err()，传递complete为0。
 */
ec_sdo_request_t *ecrt_slave_config_create_sdo_request(
    ec_slave_config_t *sc, uint16_t index, uint8_t subindex, size_t size)
{
    ec_sdo_request_t *s = ecrt_slave_config_create_sdo_request_err(sc, index,
                                                                   subindex, 0, size);
    return IS_ERR(s) ? NULL : s;
}

/*****************************************************************************/

/**
 * @brief 创建从对象的完整SDO请求。
 * @param sc 从对象的配置。
 * @param index SDO的索引。
 * @param size 数据大小。
 * @return SDO请求指针。
 * @details 
 * - 调用ecrt_slave_config_create_sdo_request_err()，传递complete为1。
 */
ec_sdo_request_t *ecrt_slave_config_create_sdo_request_complete(
    ec_slave_config_t *sc, uint16_t index, size_t size)
{
    ec_sdo_request_t *s = ecrt_slave_config_create_sdo_request_err(sc, index,
                                                                   0, 1, size);
    return IS_ERR(s) ? NULL : s;
}

/*****************************************************************************/

/**
 * @brief 创建从对象的FoE请求（带错误指针返回值）。
 * @param sc 从对象的配置。
 * @param size 数据大小。
 * @return FoE请求指针。
 * @details 
 * - 分配FoE请求内存。
 * - 初始化FoE请求。
 * - 分配FoE请求的数据内存。
 * - 锁定主站信号量。
 * - 添加FoE请求到从对象的FoE请求列表。
 * - 解锁主站信号量。
 */
ec_foe_request_t *ecrt_slave_config_create_foe_request_err(
    ec_slave_config_t *sc, size_t size)
{
    ec_foe_request_t *req;
    int ret;

    EC_CONFIG_DBG(sc, 1, "%s(sc = 0x%p, size = %zu)\n",
                  __func__, sc, size);

    if (!(req = (ec_foe_request_t *)
              kmalloc(sizeof(ec_foe_request_t), GFP_KERNEL)))
    {
        EC_CONFIG_ERR(sc, "分配FoE请求的内存失败！\n");
        return ERR_PTR(-ENOMEM);
    }

    ec_foe_request_init(req);

    ret = ec_foe_request_alloc(req, size);
    if (ret < 0)
    {
        ec_foe_request_clear(req);
        kfree(req);
        EC_CONFIG_ERR(sc, "分配FoE请求数据的内存失败（大小=%zu）！\n",
                      size);
        return ERR_PTR(ret);
    }

    // 准备可选写入的数据
    memset(req->buffer, 0x00, size);
    req->data_size = size;

    ec_lock_down(&sc->master->master_sem);
    list_add_tail(&req->list, &sc->foe_requests);
    ec_lock_up(&sc->master->master_sem);

    return req;
}

/*****************************************************************************/

/**
 * @brief 创建从对象的FoE请求。
 * @param sc 从对象的配置。
 * @param size 数据大小。
 * @return FoE请求指针。
 * @details 
 * - 调用ecrt_slave_config_create_foe_request_err()，返回NULL或指针。
 */
ec_foe_request_t *ecrt_slave_config_create_foe_request(
    ec_slave_config_t *sc, size_t size)
{
    ec_foe_request_t *s = ecrt_slave_config_create_foe_request_err(sc, size);
    return IS_ERR(s) ? NULL : s;
}

/*****************************************************************************/

/**
 * @brief 创建从对象的寄存器请求（带错误指针返回值）。
 * @param sc 从对象的配置。
 * @param size 数据大小。
 * @return 寄存器请求指针。
 * @details 
 * - 分配寄存器请求内存。
 * - 初始化寄存器请求。
 * - 锁定主站信号量。
 * - 添加寄存器请求到从对象的寄存器请求列表。
 * - 解锁主站信号量。
 */
ec_reg_request_t *ecrt_slave_config_create_reg_request_err(
    ec_slave_config_t *sc, size_t size)
{
    ec_reg_request_t *reg;
    int ret;

    EC_CONFIG_DBG(sc, 1, "%s(sc = 0x%p, size = %zu)\n",
                  __func__, sc, size);

    if (!(reg = (ec_reg_request_t *)
              kmalloc(sizeof(ec_reg_request_t), GFP_KERNEL)))
    {
        EC_CONFIG_ERR(sc, "分配寄存器请求的内存失败！\n");
        return ERR_PTR(-ENOMEM);
    }

    ret = ec_reg_request_init(reg, size);
    if (ret)
    {
        kfree(reg);
        return ERR_PTR(ret);
    }

    ec_lock_down(&sc->master->master_sem);
    list_add_tail(&reg->list, &sc->reg_requests);
    ec_lock_up(&sc->master->master_sem);

    return reg;
}

/*****************************************************************************/

/**
 * @brief 创建从对象的寄存器请求。
 * @param sc 从对象的配置。
 * @param size 数据大小。
 * @return 寄存器请求指针。
 * @details 
 * - 调用ecrt_slave_config_create_reg_request_err()，返回NULL或指针。
 */
ec_reg_request_t *ecrt_slave_config_create_reg_request(
    ec_slave_config_t *sc, size_t size)
{
    ec_reg_request_t *reg =
        ecrt_slave_config_create_reg_request_err(sc, size);
    return IS_ERR(reg) ? NULL : reg;
}

/*****************************************************************************/

/**
 * @brief 创建从对象的VoE处理程序（带错误指针返回值）。
 * @param sc 从对象的配置。
 * @param size 数据大小。
 * @return VoE处理程序指针。
 * @details 
 * - 分配VoE处理程序内存。
 * - 初始化VoE处理程序。
 * - 锁定主站信号量。
 * - 添加VoE处理程序到从对象的VoE处理程序列表。
 * - 解锁主站信号量。
 */
ec_voe_handler_t *ecrt_slave_config_create_voe_handler_err(
    ec_slave_config_t *sc, size_t size)
{
    ec_voe_handler_t *voe;
    int ret;

    EC_CONFIG_DBG(sc, 1, "%s(sc = 0x%p, size = %zu)\n", __func__, sc, size);

    if (!(voe = (ec_voe_handler_t *)
              kmalloc(sizeof(ec_voe_handler_t), GFP_KERNEL)))
    {
        EC_CONFIG_ERR(sc, "分配VoE处理程序的内存失败！\n");
        return ERR_PTR(-ENOMEM);
    }

    ret = ec_voe_handler_init(voe, sc, size);
    if (ret < 0)
    {
        kfree(voe);
        return ERR_PTR(ret);
    }

    ec_lock_down(&sc->master->master_sem);
    list_add_tail(&voe->list, &sc->voe_handlers);
    ec_lock_up(&sc->master->master_sem);

    return voe;
}

/*****************************************************************************/

/**
 * @brief 创建从对象的VoE处理程序。
 * @param sc 从对象的配置。
 * @param size 数据大小。
 * @return VoE处理程序指针。
 * @details 
 * - 调用ecrt_slave_config_create_voe_handler_err()，返回NULL或指针。
 */
ec_voe_handler_t *ecrt_slave_config_create_voe_handler(
    ec_slave_config_t *sc, size_t size)
{
    ec_voe_handler_t *voe = ecrt_slave_config_create_voe_handler_err(sc,
                                                                     size);
    return IS_ERR(voe) ? NULL : voe;
}

/*****************************************************************************/

/**
 * @brief 获取从对象的状态。
 * @param sc 从对象的配置。
 * @param state 状态结构体。
 * @return 无。
 * @details 
 * - 设置状态结构体的在线状态。
 * - 如果在线，设置状态结构体的操作状态、AL状态、错误标志、准备状态和位置。
 * - 如果不在线，设置状态结构体的操作状态、AL状态、错误标志、准备状态和位置（-1）。
 */
void ecrt_slave_config_state(const ec_slave_config_t *sc,
                             ec_slave_config_state_t *state)
{
    state->online = sc->slave ? 1 : 0;
    if (state->online)
    {
        state->operational =
            sc->slave->current_state == EC_SLAVE_STATE_OP && !sc->slave->force_config;
        state->al_state = sc->slave->current_state;
        state->error_flag = sc->slave->error_flag ? 1 : 0;
        state->ready = ec_fsm_slave_is_ready(&sc->slave->fsm) ? 1 : 0;
        state->position = sc->slave->ring_position;
    }
    else
    {
        state->operational = 0;
        state->al_state = EC_SLAVE_STATE_UNKNOWN;
        state->error_flag = 0;
        state->ready = 0;
        state->position = (uint16_t)-1;
    }
}

/*****************************************************************************/

/**
 * @brief 为从对象配置IDN。
 * @param sc 从对象的配置。
 * @param drive_no 驱动号。
 * @param idn IDN。
 * @param state AL状态。
 * @param data 数据。
 * @param size 数据大小。
 * @return 成功返回0，否则返回错误代码。
 * @details 
 * - 检查驱动号是否有效。
 * - 检查AL状态是否为PREOP或SAFEOP。
 * - 检查从对象是否支持SoE。
 * - 分配SoE请求内存。
 * - 初始化SoE请求。
 * - 设置SoE请求的驱动号、IDN和AL状态。
 * - 复制数据到SoE请求。
 * - 锁定主站信号量。
 * - 添加SoE请求到从对象的SoE配置列表。
 * - 解锁主站信号量。
 */
int ecrt_slave_config_idn(ec_slave_config_t *sc, uint8_t drive_no,
                          uint16_t idn, ec_al_state_t state, const uint8_t *data,
                          size_t size)
{
    ec_slave_t *slave = sc->slave;
    ec_soe_request_t *req;
    int ret;

    EC_CONFIG_DBG(sc, 1, "%s(sc = 0x%p, drive_no = %u, idn = 0x%04X, "
                         "state = %u, data = 0x%p, size = %zu)\n",
                  __func__, sc, drive_no, idn, state, data, size);

    if (drive_no > 7)
    {
        EC_CONFIG_ERR(sc, "无效的驱动号 %u！\n",
                      (unsigned int)drive_no);
        return -EINVAL;
    }

    if (state != EC_AL_STATE_PREOP && state != EC_AL_STATE_SAFEOP)
    {
        EC_CONFIG_ERR(sc, "IDN配置的AL状态必须为PREOP或SAFEOP！\n");
        return -EINVAL;
    }

    if (slave && slave->sii_image && !(slave->sii_image->sii.mailbox_protocols & EC_MBOX_SOE))
    {
        EC_CONFIG_WARN(sc, "连接的从设备不支持SoE！\n");
    }

    if (!(req = (ec_soe_request_t *)
              kmalloc(sizeof(ec_soe_request_t), GFP_KERNEL)))
    {
        EC_CONFIG_ERR(sc, "分配IDN配置的内存失败！\n");
        return -ENOMEM;
    }

    ec_soe_request_init(req);
    ec_soe_request_set_drive_no(req, drive_no);
    ec_soe_request_set_idn(req, idn);
    req->al_state = state;

    ret = ec_soe_request_copy_data(req, data, size);
    if (ret < 0)
    {
        ec_soe_request_clear(req);
        kfree(req);
        return ret;
    }

    ec_lock_down(&sc->master->master_sem);
    list_add_tail(&req->list, &sc->soe_configs);
    ec_lock_up(&sc->master->master_sem);
    return 0;
}

/*****************************************************************************/

/** \cond */

EXPORT_SYMBOL(ecrt_slave_config_sync_manager);
EXPORT_SYMBOL(ecrt_slave_config_watchdog);
EXPORT_SYMBOL(ecrt_slave_config_pdo_assign_add);
EXPORT_SYMBOL(ecrt_slave_config_pdo_assign_clear);
EXPORT_SYMBOL(ecrt_slave_config_pdo_mapping_add);
EXPORT_SYMBOL(ecrt_slave_config_pdo_mapping_clear);
EXPORT_SYMBOL(ecrt_slave_config_pdos);
EXPORT_SYMBOL(ecrt_slave_config_reg_pdo_entry);
EXPORT_SYMBOL(ecrt_slave_config_dc);
EXPORT_SYMBOL(ecrt_slave_config_sdo);
EXPORT_SYMBOL(ecrt_slave_config_sdo8);
EXPORT_SYMBOL(ecrt_slave_config_sdo16);
EXPORT_SYMBOL(ecrt_slave_config_sdo32);
EXPORT_SYMBOL(ecrt_slave_config_complete_sdo);
EXPORT_SYMBOL(ecrt_slave_config_emerg_size);
EXPORT_SYMBOL(ecrt_slave_config_emerg_pop);
EXPORT_SYMBOL(ecrt_slave_config_emerg_clear);
EXPORT_SYMBOL(ecrt_slave_config_emerg_overruns);
EXPORT_SYMBOL(ecrt_slave_config_create_sdo_request);
EXPORT_SYMBOL(ecrt_slave_config_create_sdo_request_complete);
EXPORT_SYMBOL(ecrt_slave_config_create_foe_request);
EXPORT_SYMBOL(ecrt_slave_config_create_voe_handler);
EXPORT_SYMBOL(ecrt_slave_config_create_reg_request);
EXPORT_SYMBOL(ecrt_slave_config_state);
EXPORT_SYMBOL(ecrt_slave_config_idn);

/** \endcond */

/*****************************************************************************/

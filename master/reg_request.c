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

/** \file
 * Register request functions.
 */

/*****************************************************************************/

#include <linux/module.h>
#include <linux/jiffies.h>
#include <linux/slab.h>

#include "reg_request.h"

/*****************************************************************************/

/**
 * @brief 注册请求构造函数。
 * @param reg 注册请求。
 * @param size 内存大小。
 * @return 成功返回零，否则返回负错误代码。
 * @details 初始化给定的注册请求，并分配指定大小的内存。
 */
int ec_reg_request_init(
    ec_reg_request_t *reg, /**< 注册请求。 */
    size_t size            /**< 内存大小。 */
)
{
    if (!(reg->data = (uint8_t *)kmalloc(size, GFP_KERNEL)))
    {
        EC_ERR("无法分配%zu字节的寄存器内存。\n", size);
        return -ENOMEM;
    }

    INIT_LIST_HEAD(&reg->list);
    reg->mem_size = size;
    memset(reg->data, 0x00, size);
    reg->dir = EC_DIR_INVALID;
    reg->address = 0;
    reg->transfer_size = 0;
    reg->state = EC_INT_REQUEST_INIT;
    reg->ring_position = 0;
    return 0;
}

/*****************************************************************************/

/**
 * @brief 注册请求析构函数。
 * @param reg 注册请求。
 * @return 无返回值。
 * @details 清除给定的注册请求，并释放相关的内存。
 */
void ec_reg_request_clear(
    ec_reg_request_t *reg /**< 注册请求。 */
)
{
    if (reg->data)
    {
        kfree(reg->data);
    }
}

/*****************************************************************************
 * Application interface.
 ****************************************************************************/

/**
 * @brief 获取寄存器请求的数据指针。
 * @param reg 寄存器请求。
 * @return 数据指针。
 * @details 返回寄存器请求中的数据指针。
 */
uint8_t *ecrt_reg_request_data(ec_reg_request_t *reg)
{
    return reg->data;
}

/*****************************************************************************/

/**
 * @brief 获取寄存器请求的状态。
 * @param reg 寄存器请求。
 * @return 寄存器请求的状态。
 * @details 返回寄存器请求的状态，使用状态转换表进行转换。
 */
ec_request_state_t ecrt_reg_request_state(const ec_reg_request_t *reg)
{
    return ec_request_state_translation_table[reg->state];
}

/*****************************************************************************/

/**
 * @brief 设置寄存器请求的写操作。
 * @param reg 寄存器请求。
 * @param address 地址。
 * @param size 大小。
 * @return 无返回值。
 * @details 设置寄存器请求为写操作，并指定地址和传输大小。
 */
void ecrt_reg_request_write(ec_reg_request_t *reg, uint16_t address,
                            size_t size)
{
    reg->dir = EC_DIR_OUTPUT;
    reg->address = address;
    reg->transfer_size = min(size, reg->mem_size);
    reg->state = EC_INT_REQUEST_QUEUED;
}

/*****************************************************************************/

/**
 * @brief 设置寄存器请求的读操作。
 * @param reg 寄存器请求。
 * @param address 地址。
 * @param size 大小。
 * @return 无返回值。
 * @details 设置寄存器请求为读操作，并指定地址和传输大小。
 */
void ecrt_reg_request_read(ec_reg_request_t *reg, uint16_t address,
                           size_t size)
{
    reg->dir = EC_DIR_INPUT;
    reg->address = address;
    reg->transfer_size = min(size, reg->mem_size);
    reg->state = EC_INT_REQUEST_QUEUED;
}

/*****************************************************************************/

/**
 * @brief 设置寄存器请求的读写操作。
 * @param reg 寄存器请求。
 * @param address 地址。
 * @param size 大小。
 * @return 无返回值。
 * @details 设置寄存器请求为读写操作，并指定地址和传输大小。
 */
void ecrt_reg_request_readwrite(ec_reg_request_t *reg, uint16_t address,
                                size_t size)
{
    reg->dir = EC_DIR_BOTH;
    reg->address = address;
    reg->transfer_size = min(size, reg->mem_size);
    reg->state = EC_INT_REQUEST_QUEUED;
}

/*****************************************************************************/

/** \cond */

EXPORT_SYMBOL(ecrt_reg_request_data);
EXPORT_SYMBOL(ecrt_reg_request_state);
EXPORT_SYMBOL(ecrt_reg_request_write);
EXPORT_SYMBOL(ecrt_reg_request_read);
EXPORT_SYMBOL(ecrt_reg_request_readwrite);

/** \endcond */

/*****************************************************************************/

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
   Methods of an EtherCAT datagram.
*/

/*****************************************************************************/

#include <linux/slab.h>

#include "datagram.h"
#include "master.h"

/*****************************************************************************/

/** \cond */

#define EC_FUNC_HEADER                               \
    ret = ec_datagram_prealloc(datagram, data_size); \
    if (unlikely(ret))                               \
        return ret;                                  \
    datagram->index = 0;                             \
    datagram->working_counter = 0;                   \
    datagram->state = EC_DATAGRAM_INIT;

#define EC_FUNC_FOOTER               \
    datagram->data_size = data_size; \
    return 0;

/** \endcond */

/*****************************************************************************/

/** 用于ec_datagram_type_string()的数据报类型字符串数组。
 *
 * \attention 这是按照ec_datagram_type_t的索引进行排序的。
 */
static const char *type_strings[] = {
    "?",
    "APRD",
    "APWR",
    "APRW",
    "FPRD",
    "FPWR",
    "FPRW",
    "BRD",
    "BWR",
    "BRW",
    "LRD",
    "LWR",
    "LRW",
    "ARMW",
    "FRMW"};


/*****************************************************************************/

/**
 * @brief 构造函数。
 *
 * @param datagram EtherCAT数据报文。
 *
 * @details 初始化EtherCAT数据报文的各个成员变量。
 */
void ec_datagram_init(ec_datagram_t *datagram /**< EtherCAT数据报文。 */)
{
    INIT_LIST_HEAD(&datagram->queue); // 标记为未排队
    datagram->device_index = EC_DEVICE_MAIN;
    datagram->type = EC_DATAGRAM_NONE;
    memset(datagram->address, 0x00, EC_ADDR_LEN);
    datagram->data = NULL;
    datagram->data_origin = EC_ORIG_INTERNAL;
    datagram->mem_size = 0;
    datagram->data_size = 0;
    datagram->index = 0x00;
    datagram->working_counter = 0x0000;
    datagram->state = EC_DATAGRAM_INIT;
#ifdef EC_HAVE_CYCLES
    datagram->cycles_sent = 0;
#endif
    datagram->jiffies_sent = 0;
    datagram->app_time_sent = 0;
#ifdef EC_HAVE_CYCLES
    datagram->cycles_received = 0;
#endif
    datagram->jiffies_received = 0;
    datagram->skip_count = 0;
    datagram->stats_output_jiffies = 0;
    memset(datagram->name, 0x00, EC_DATAGRAM_NAME_SIZE);
}

/*****************************************************************************/

/**
 * @brief 析构函数。
 *
 * @param datagram EtherCAT数据报文。
 *
 * @details 清理EtherCAT数据报文的资源，包括从队列中移除和释放内部分配的数据内存。
 */
void ec_datagram_clear(ec_datagram_t *datagram /**< EtherCAT数据报文。 */)
{
    ec_datagram_unqueue(datagram);

    if (datagram->data_origin == EC_ORIG_INTERNAL && datagram->data)
    {
        kfree(datagram->data);
        datagram->data = NULL;
    }
}

/*****************************************************************************/

/**
 * @brief 从队列中移除数据报文。
 *
 * @param datagram EtherCAT数据报文。
 *
 * @details 如果数据报文在队列中，将其从队列中移除。
 */
void ec_datagram_unqueue(ec_datagram_t *datagram /**< EtherCAT数据报文。 */)
{
    if (!list_empty(&datagram->queue))
    {
        list_del_init(&datagram->queue);
    }
}

/*****************************************************************************/

/**
 * @brief 分配内部负载内存。
 *
 * @param datagram EtherCAT数据报文。
 * @param size     新的负载大小（字节）。
 *
 * @return 成功返回0，否则返回-ENOMEM。
 *
 * @details 如果已分配的内存大小已经大于请求的大小，则不进行任何操作。
 *          如果提供了外部负载内存，则不进行范围检查。
 */
int ec_datagram_prealloc(
    ec_datagram_t *datagram, /**< EtherCAT数据报文。 */
    size_t size              /**< 新的负载大小（字节）。 */
)
{
    if (datagram->data_origin == EC_ORIG_EXTERNAL || size <= datagram->mem_size)
        return 0;

    if (datagram->data)
    {
        kfree(datagram->data);
        datagram->data = NULL;
        datagram->mem_size = 0;
    }

    if (!(datagram->data = kmalloc(size, GFP_KERNEL)))
    {
        EC_ERR("Failed to allocate %zu bytes of datagram memory!\n", size);
        return -ENOMEM;
    }

    datagram->mem_size = size;
    return 0;
}

/*****************************************************************************/

/**
 * @brief 将数据报文负载内存填充为零。
 *
 * @param datagram EtherCAT数据报文。
 *
 * @details 将数据报文的负载内存填充为零。
 */
void ec_datagram_zero(ec_datagram_t *datagram /**< EtherCAT数据报文。 */)
{
    memset(datagram->data, 0x00, datagram->data_size);
}

/*****************************************************************************/

/**
 * @brief 复制先前构造的数据报文以便重复发送。
 *
 * @param datagram 目标EtherCAT数据报文。
 * @param source   要复制的EtherCAT数据报文。
 *
 * @return ec_datagram_prealloc()的返回值。
 *
 * @details 复制源EtherCAT数据报文的内容到目标EtherCAT数据报文，用于重复发送。
 */
int ec_datagram_repeat(
    ec_datagram_t *datagram, /**< 目标EtherCAT数据报文。 */
    const ec_datagram_t *source /**< 要复制的EtherCAT数据报文。 */)
{
    int ret;
    size_t data_size = source->data_size;
    EC_FUNC_HEADER;
    if (datagram != source)
    {
        datagram->type = source->type;
        memcpy(datagram->address, source->address, sizeof(datagram->address));
        memcpy(datagram->data, source->data, data_size);
    }
    EC_FUNC_FOOTER;
}

/*****************************************************************************/

/**
 * @brief 初始化EtherCAT APRD数据报文。
 *
 * @param datagram      EtherCAT数据报文。
 * @param ring_position 自动递增地址。
 * @param mem_address   物理内存地址。
 * @param data_size     要读取的字节数。
 *
 * @return ec_datagram_prealloc()的返回值。
 *
 * @details 初始化EtherCAT APRD（Auto-increment Physical Read）数据报文。
 */
int ec_datagram_aprd(
    ec_datagram_t *datagram, /**< EtherCAT数据报文。 */
    uint16_t ring_position,  /**< 自动递增地址。 */
    uint16_t mem_address,    /**< 物理内存地址。 */
    size_t data_size         /**< 要读取的字节数。 */
)
{
    int ret;
    EC_FUNC_HEADER;
    datagram->type = EC_DATAGRAM_APRD;
    EC_WRITE_S16(datagram->address, (int16_t)ring_position * (-1));
    EC_WRITE_U16(datagram->address + 2, mem_address);
    EC_FUNC_FOOTER;
}

/*****************************************************************************/

/**
 * @brief 初始化EtherCAT APWR数据报文。
 *
 * @param datagram      EtherCAT数据报文。
 * @param ring_position 自动递增地址。
 * @param mem_address   物理内存地址。
 * @param data_size     要写入的字节数。
 *
 * @return ec_datagram_prealloc()的返回值。
 *
 * @details 初始化EtherCAT APWR（Auto-increment Physical Write）数据报文。
 */
int ec_datagram_apwr(
    ec_datagram_t *datagram, /**< EtherCAT数据报文。 */
    uint16_t ring_position,  /**< 自动递增地址。 */
    uint16_t mem_address,    /**< 物理内存地址。 */
    size_t data_size         /**< 要写入的字节数。 */
)
{
    int ret;
    EC_FUNC_HEADER;
    datagram->type = EC_DATAGRAM_APWR;
    EC_WRITE_S16(datagram->address, (int16_t)ring_position * (-1));
    EC_WRITE_U16(datagram->address + 2, mem_address);
    EC_FUNC_FOOTER;
}

/*****************************************************************************/

/**
 * @brief 初始化EtherCAT APRW数据报文。
 *
 * @param datagram      EtherCAT数据报文。
 * @param ring_position 自动递增地址。
 * @param mem_address   物理内存地址。
 * @param data_size     要写入的字节数。
 *
 * @return ec_datagram_prealloc()的返回值。
 *
 * @details 初始化EtherCAT APRW（Auto-increment Physical Read/Write）数据报文。
 */
int ec_datagram_aprw(
    ec_datagram_t *datagram, /**< EtherCAT数据报文。 */
    uint16_t ring_position,  /**< 自动递增地址。 */
    uint16_t mem_address,    /**< 物理内存地址。 */
    size_t data_size         /**< 要写入的字节数。 */
)
{
    int ret;
    EC_FUNC_HEADER;
    datagram->type = EC_DATAGRAM_APRW;
    EC_WRITE_S16(datagram->address, (int16_t)ring_position * (-1));
    EC_WRITE_U16(datagram->address + 2, mem_address);
    EC_FUNC_FOOTER;
}

/*****************************************************************************/

/**
 * @brief 初始化EtherCAT ARMW数据报文。
 *
 * @param datagram      EtherCAT数据报文。
 * @param ring_position 自动递增地址。
 * @param mem_address   物理内存地址。
 * @param data_size     要读取的字节数。
 *
 * @return ec_datagram_prealloc()的返回值。
 *
 * @details 初始化EtherCAT ARMW（Auto-increment Read/Modify/Write）数据报文。
 */
int ec_datagram_armw(
    ec_datagram_t *datagram, /**< EtherCAT数据报文。 */
    uint16_t ring_position,  /**< 自动递增地址。 */
    uint16_t mem_address,    /**< 物理内存地址。 */
    size_t data_size         /**< 要读取的字节数。 */
)
{
    int ret;
    EC_FUNC_HEADER;
    datagram->type = EC_DATAGRAM_ARMW;
    EC_WRITE_S16(datagram->address, (int16_t)ring_position * (-1));
    EC_WRITE_U16(datagram->address + 2, mem_address);
    EC_FUNC_FOOTER;
}

/*****************************************************************************/

/**
 * @brief 初始化EtherCAT FPRD数据报文。
 *
 * @param datagram            EtherCAT数据报文。
 * @param configured_address  配置的站地址。
 * @param mem_address         物理内存地址。
 * @param data_size           要读取的字节数。
 *
 * @return ec_datagram_prealloc()的返回值。
 *
 * @details 初始化EtherCAT FPRD（Fixed Physical Read）数据报文。
 */
int ec_datagram_fprd(
    ec_datagram_t *datagram,     /**< EtherCAT数据报文。 */
    uint16_t configured_address, /**< 配置的站地址。 */
    uint16_t mem_address,        /**< 物理内存地址。 */
    size_t data_size             /**< 要读取的字节数。 */
)
{
    int ret;

    if (unlikely(configured_address == 0x0000))
        EC_WARN("使用配置的站地址0x0000！\n");

    EC_FUNC_HEADER;
    datagram->type = EC_DATAGRAM_FPRD;
    EC_WRITE_U16(datagram->address, configured_address);
    EC_WRITE_U16(datagram->address + 2, mem_address);
    EC_FUNC_FOOTER;
}

/*****************************************************************************/

/**
 * @brief 初始化EtherCAT FPWR数据报文。
 *
 * @param datagram            EtherCAT数据报文。
 * @param configured_address  配置的站地址。
 * @param mem_address         物理内存地址。
 * @param data_size           要写入的字节数。
 *
 * @return ec_datagram_prealloc()的返回值。
 *
 * @details 初始化EtherCAT FPWR（Fixed Physical Write）数据报文。
 */
int ec_datagram_fpwr(
    ec_datagram_t *datagram,     /**< EtherCAT数据报文。 */
    uint16_t configured_address, /**< 配置的站地址。 */
    uint16_t mem_address,        /**< 物理内存地址。 */
    size_t data_size             /**< 要写入的字节数。 */
)
{
    int ret;

    if (unlikely(configured_address == 0x0000))
        EC_WARN("使用配置的站地址0x0000！\n");

    EC_FUNC_HEADER;
    datagram->type = EC_DATAGRAM_FPWR;
    EC_WRITE_U16(datagram->address, configured_address);
    EC_WRITE_U16(datagram->address + 2, mem_address);
    EC_FUNC_FOOTER;
}

/*****************************************************************************/

/**
 * @brief 初始化EtherCAT FPRW数据报文。
 *
 * @param datagram            EtherCAT数据报文。
 * @param configured_address  配置的站地址。
 * @param mem_address         物理内存地址。
 * @param data_size           要写入的字节数。
 *
 * @return ec_datagram_prealloc()的返回值。
 *
 * @details 初始化EtherCAT FPRW（Fixed Physical Read/Write）数据报文。
 */
int ec_datagram_fprw(
    ec_datagram_t *datagram,     /**< EtherCAT数据报文。 */
    uint16_t configured_address, /**< 配置的站地址。 */
    uint16_t mem_address,        /**< 物理内存地址。 */
    size_t data_size             /**< 要写入的字节数。 */
)
{
    int ret;

    if (unlikely(configured_address == 0x0000))
        EC_WARN("使用配置的站地址0x0000！\n");

    EC_FUNC_HEADER;
    datagram->type = EC_DATAGRAM_FPRW;
    EC_WRITE_U16(datagram->address, configured_address);
    EC_WRITE_U16(datagram->address + 2, mem_address);
    EC_FUNC_FOOTER;
}

/*****************************************************************************/

/**
 * @brief 初始化EtherCAT FRMW数据报文。
 *
 * @param datagram            EtherCAT数据报文。
 * @param configured_address  配置的站地址。
 * @param mem_address         物理内存地址。
 * @param data_size           要写入的字节数。
 *
 * @return ec_datagram_prealloc()的返回值。
 *
 * @details 初始化EtherCAT FRMW（Fixed Register Modify/Write）数据报文。
 */
int ec_datagram_frmw(
    ec_datagram_t *datagram,     /**< EtherCAT数据报文。 */
    uint16_t configured_address, /**< 配置的站地址。 */
    uint16_t mem_address,        /**< 物理内存地址。 */
    size_t data_size             /**< 要写入的字节数。 */
)
{
    int ret;

    if (unlikely(configured_address == 0x0000))
        EC_WARN("使用配置的站地址0x0000！\n");

    EC_FUNC_HEADER;
    datagram->type = EC_DATAGRAM_FRMW;
    EC_WRITE_U16(datagram->address, configured_address);
    EC_WRITE_U16(datagram->address + 2, mem_address);
    EC_FUNC_FOOTER;
}

/*****************************************************************************/

/**
 * @brief 初始化EtherCAT BRD数据报文。
 *
 * @param datagram      EtherCAT数据报文。
 * @param mem_address   物理内存地址。
 * @param data_size     要读取的字节数。
 *
 * @return ec_datagram_prealloc()的返回值。
 *
 * @details 初始化EtherCAT BRD（Broadcast Read）数据报文。
 */
int ec_datagram_brd(
    ec_datagram_t *datagram, /**< EtherCAT数据报文。 */
    uint16_t mem_address,    /**< 物理内存地址。 */
    size_t data_size         /**< 要读取的字节数。 */
)
{
    int ret;
    EC_FUNC_HEADER;
    datagram->type = EC_DATAGRAM_BRD;
    EC_WRITE_U16(datagram->address, 0x0000);
    EC_WRITE_U16(datagram->address + 2, mem_address);
    EC_FUNC_FOOTER;
}

/*****************************************************************************/

/**
 * @brief 初始化EtherCAT BWR数据报文。
 *
 * @param datagram      EtherCAT数据报文。
 * @param mem_address   物理内存地址。
 * @param data_size     要写入的字节数。
 *
 * @return ec_datagram_prealloc()的返回值。
 *
 * @details 初始化EtherCAT BWR（Broadcast Write）数据报文。
 */
int ec_datagram_bwr(
    ec_datagram_t *datagram, /**< EtherCAT数据报文。 */
    uint16_t mem_address,    /**< 物理内存地址。 */
    size_t data_size         /**< 要写入的字节数。 */
)
{
    int ret;
    EC_FUNC_HEADER;
    datagram->type = EC_DATAGRAM_BWR;
    EC_WRITE_U16(datagram->address, 0x0000);
    EC_WRITE_U16(datagram->address + 2, mem_address);
    EC_FUNC_FOOTER;
}

/*****************************************************************************/

/**
 * @brief 初始化EtherCAT BRW数据报文。
 *
 * @param datagram      EtherCAT数据报文。
 * @param mem_address   物理内存地址。
 * @param data_size     要写入的字节数。
 *
 * @return ec_datagram_prealloc()的返回值。
 *
 * @details 初始化EtherCAT BRW（Broadcast Read/Write）数据报文。
 */
int ec_datagram_brw(
    ec_datagram_t *datagram, /**< EtherCAT数据报文。 */
    uint16_t mem_address,    /**< 物理内存地址。 */
    size_t data_size         /**< 要写入的字节数。 */
)
{
    int ret;
    EC_FUNC_HEADER;
    datagram->type = EC_DATAGRAM_BRW;
    EC_WRITE_U16(datagram->address, 0x0000);
    EC_WRITE_U16(datagram->address + 2, mem_address);
    EC_FUNC_FOOTER;
}

/*****************************************************************************/

/**
 * @brief 初始化EtherCAT LRD数据报文。
 *
 * @param datagram      EtherCAT数据报文。
 * @param offset        逻辑地址。
 * @param data_size     要读取/写入的字节数。
 *
 * @return ec_datagram_prealloc()的返回值。
 *
 * @details 初始化EtherCAT LRD（Logical Read）数据报文。
 */
int ec_datagram_lrd(
    ec_datagram_t *datagram, /**< EtherCAT数据报文。 */
    uint32_t offset,         /**< 逻辑地址。 */
    size_t data_size         /**< 要读取/写入的字节数。 */
)
{
    int ret;
    EC_FUNC_HEADER;
    datagram->type = EC_DATAGRAM_LRD;
    EC_WRITE_U32(datagram->address, offset);
    EC_FUNC_FOOTER;
}

/*****************************************************************************/

/**
 * @brief 初始化EtherCAT LWR数据报文。
 *
 * @param datagram      EtherCAT数据报文。
 * @param offset        逻辑地址。
 * @param data_size     要读取/写入的字节数。
 *
 * @return ec_datagram_prealloc()的返回值。
 *
 * @details 初始化EtherCAT LWR（Logical Write）数据报文。
 */
int ec_datagram_lwr(
    ec_datagram_t *datagram, /**< EtherCAT数据报文。 */
    uint32_t offset,         /**< 逻辑地址。 */
    size_t data_size         /**< 要读取/写入的字节数。 */
)
{
    int ret;
    EC_FUNC_HEADER;
    datagram->type = EC_DATAGRAM_LWR;
    EC_WRITE_U32(datagram->address, offset);
    EC_FUNC_FOOTER;
}

/*****************************************************************************/

/**
 * @brief 初始化EtherCAT LRW数据报文。
 *
 * @param datagram      EtherCAT数据报文。
 * @param offset        逻辑地址。
 * @param data_size     要读取/写入的字节数。
 *
 * @return ec_datagram_prealloc()的返回值。
 *
 * @details 初始化EtherCAT LRW（Logical Read/Write）数据报文。
 */
int ec_datagram_lrw(
    ec_datagram_t *datagram, /**< EtherCAT数据报文。 */
    uint32_t offset,         /**< 逻辑地址。 */
    size_t data_size         /**< 要读取/写入的字节数。 */
)
{
    int ret;
    EC_FUNC_HEADER;
    datagram->type = EC_DATAGRAM_LRW;
    EC_WRITE_U32(datagram->address, offset);
    EC_FUNC_FOOTER;
}

/*****************************************************************************/

/**
 * @brief 使用外部内存初始化EtherCAT LRD数据报文。
 *
 * @param datagram          EtherCAT数据报文。
 * @param offset            逻辑地址。
 * @param data_size         要读取/写入的字节数。
 * @param external_memory   要使用的内存指针。
 *
 * @return ec_datagram_prealloc()的返回值。
 *
 * @details 使用外部内存初始化EtherCAT LRD（Logical Read）数据报文。
 *          假设外部内存至少有 \a data_size 字节大小。
 */
int ec_datagram_lrd_ext(
    ec_datagram_t *datagram, /**< EtherCAT数据报文。 */
    uint32_t offset,         /**< 逻辑地址。 */
    size_t data_size,        /**< 要读取/写入的字节数。 */
    uint8_t *external_memory /**< 要使用的内存指针。 */
)
{
    int ret;
    datagram->data = external_memory;
    datagram->data_origin = EC_ORIG_EXTERNAL;
    EC_FUNC_HEADER;
    datagram->type = EC_DATAGRAM_LRD;
    EC_WRITE_U32(datagram->address, offset);
    EC_FUNC_FOOTER;
}

/*****************************************************************************/

/**
 * @brief 使用外部内存初始化EtherCAT LWR数据报文。
 *
 * @param datagram          EtherCAT数据报文。
 * @param offset            逻辑地址。
 * @param data_size         要读取/写入的字节数。
 * @param external_memory   要使用的内存指针。
 *
 * @return ec_datagram_prealloc()的返回值。
 *
 * @details 使用外部内存初始化EtherCAT LWR（Logical Write）数据报文。
 *          假设外部内存至少有 \a data_size 字节大小。
 */
int ec_datagram_lwr_ext(
    ec_datagram_t *datagram, /**< EtherCAT数据报文。 */
    uint32_t offset,         /**< 逻辑地址。 */
    size_t data_size,        /**< 要读取/写入的字节数。 */
    uint8_t *external_memory /**< 要使用的内存指针。 */
)
{
    int ret;
    datagram->data = external_memory;
    datagram->data_origin = EC_ORIG_EXTERNAL;
    EC_FUNC_HEADER;
    datagram->type = EC_DATAGRAM_LWR;
    EC_WRITE_U32(datagram->address, offset);
    EC_FUNC_FOOTER;
}

/*****************************************************************************/

/**
 * @brief 使用外部内存初始化EtherCAT LRW数据报文。
 *
 * @param datagram          EtherCAT数据报文。
 * @param offset            逻辑地址。
 * @param data_size         要读取/写入的字节数。
 * @param external_memory   要使用的内存指针。
 *
 * @return ec_datagram_prealloc()的返回值。
 *
 * @details 使用外部内存初始化EtherCAT LRW（Logical Read/Write）数据报文。
 *          假设外部内存至少有 \a data_size 字节大小。
 */
int ec_datagram_lrw_ext(
    ec_datagram_t *datagram, /**< EtherCAT数据报文。 */
    uint32_t offset,         /**< 逻辑地址。 */
    size_t data_size,        /**< 要读取/写入的字节数。 */
    uint8_t *external_memory /**< 要使用的内存指针。 */
)
{
    int ret;
    datagram->data = external_memory;
    datagram->data_origin = EC_ORIG_EXTERNAL;
    EC_FUNC_HEADER;
    datagram->type = EC_DATAGRAM_LRW;
    EC_WRITE_U32(datagram->address, offset);
    EC_FUNC_FOOTER;
}

/*****************************************************************************/

/**
 * @brief 打印数据报文的状态。
 *
 * @param datagram EtherCAT数据报文
 * 
 * @details 输出文本消息。
 */
void ec_datagram_print_state(
    const ec_datagram_t *datagram /**< EtherCAT数据报文 */
)
{
    printk(KERN_CONT "数据报文 ");
    switch (datagram->state)
    {
    case EC_DATAGRAM_INIT:
        printk(KERN_CONT "已初始化");
        break;
    case EC_DATAGRAM_QUEUED:
        printk(KERN_CONT "已入队");
        break;
    case EC_DATAGRAM_SENT:
        printk(KERN_CONT "已发送");
        break;
    case EC_DATAGRAM_RECEIVED:
        printk(KERN_CONT "已接收");
        break;
    case EC_DATAGRAM_TIMED_OUT:
        printk(KERN_CONT "超时");
        break;
    case EC_DATAGRAM_ERROR:
        printk(KERN_CONT "错误");
        break;
    case EC_DATAGRAM_INVALID:
        printk(KERN_CONT "无效");
        break;
    default:
        printk(KERN_CONT "???");
    }

    printk(KERN_CONT "。\n");
}

/*****************************************************************************/

/**
 * @brief 评估单播数据报文的工作计数器。
 *
 * @param datagram EtherCAT数据报文
 * 
 * @details 输出错误消息。
 */
void ec_datagram_print_wc_error(
    const ec_datagram_t *datagram /**< EtherCAT数据报文 */
)
{
    if (datagram->working_counter == 0)
        printk(KERN_CONT "无响应。");
    else if (datagram->working_counter > 1)
        printk(KERN_CONT "有%u个从站响应！", datagram->working_counter);
    else
        printk(KERN_CONT "成功。");
    printk(KERN_CONT "\n");
}

/*****************************************************************************/

/**
 * @brief 每秒最多输出一次数据报文统计信息。
 *
 * @param datagram EtherCAT数据报文
 * 
 * @details 如果距上次输出超过一秒，则输出数据报文的统计信息。
 */
void ec_datagram_output_stats(
    ec_datagram_t *datagram)
{
    if (jiffies - datagram->stats_output_jiffies > HZ)
    {
        datagram->stats_output_jiffies = jiffies;

        if (unlikely(datagram->skip_count))
        {
            EC_WARN("数据报文 %p (%s) 已被跳过 %u 次。\n",
                    datagram, datagram->name,
                    datagram->skip_count);
            datagram->skip_count = 0;
        }
    }
}

/*****************************************************************************/

/**
 * @brief 返回描述数据报文类型的字符串。
 *
 * @param datagram EtherCAT数据报文
 * 
 * @return 包含所需字符串的静态内存的指针。
 */
const char *ec_datagram_type_string(
    const ec_datagram_t *datagram /**< EtherCAT数据报文。 */
)
{
    return type_strings[datagram->type];
}

/*****************************************************************************/

/**
 * @brief 初始化邮箱响应数据。
 *
 * @param mbox_data 邮箱响应数据。
 * 
 * @details 初始化邮箱响应数据结构。
 */
void ec_mbox_data_init(
    ec_mbox_data_t *mbox_data /**< 邮箱响应数据。 */
)
{
    mbox_data->data = NULL;
    mbox_data->data_size = 0;
    mbox_data->payload_size = 0;
}

/*****************************************************************************/

/**
 * @brief 释放邮箱响应数据的内部内存。
 *
 * @param mbox_data 邮箱响应数据。
 * 
 * @details 释放邮箱响应数据结构中的内部内存。
 */
void ec_mbox_data_clear(
    ec_mbox_data_t *mbox_data /**< 邮箱响应数据。 */
)
{
    if (mbox_data->data)
    {
        kfree(mbox_data->data);
        mbox_data->data = NULL;
        mbox_data->data_size = 0;
    }
}

/*****************************************************************************/

/**
 * @brief 为邮箱响应数据分配内部内存。
 *
 * @param mbox_data 邮箱响应数据。
 * @param size 邮箱大小（字节）。
 * @return 如果成功则为0，否则为 \a -ENOMEM。
 * 
 * @details 为邮箱响应数据结构分配指定大小的内部内存。
 */
int ec_mbox_data_prealloc(
    ec_mbox_data_t *mbox_data, /**< 邮箱响应数据。 */
    size_t size                /**< 邮箱大小（字节）。 */
)
{
    if (mbox_data->data)
    {
        kfree(mbox_data->data);
        mbox_data->data = NULL;
        mbox_data->data_size = 0;
    }

    if (!(mbox_data->data = kmalloc(size, GFP_KERNEL)))
    {
        EC_ERR("分配 %zu 字节的邮箱数据内存失败！\n", size);
        return -ENOMEM;
    }
    mbox_data->data_size = size;
    return 0;
}

/*****************************************************************************/

/**
 * @brief 为所有支持的邮箱协议分配邮箱响应数据的内部内存。
 *
 * @param slave EtherCAT从站。
 * @param protocols 支持的协议。
 * @param size 邮箱大小（字节）。
 * 
 * @details 根据支持的协议为从站的邮箱响应数据结构分配指定大小的内部内存。
 */
void ec_mbox_prot_data_prealloc(
    ec_slave_t *slave,  /**< EtherCAT从站。 */
    uint16_t protocols, /**< 支持的协议。 */
    size_t size         /**< 邮箱大小（字节）。 */
)
{
    if ((size > 0) && (size <= EC_MAX_DATA_SIZE))
    {
#ifdef EC_EOE
        if (protocols & EC_MBOX_EOE)
        {
            ec_mbox_data_prealloc(&slave->mbox_eoe_frag_data, size);
            ec_mbox_data_prealloc(&slave->mbox_eoe_init_data, size);
        }
        else
        {
            ec_mbox_data_clear(&slave->mbox_eoe_frag_data);
            ec_mbox_data_clear(&slave->mbox_eoe_init_data);
        }
#endif
        if (protocols & EC_MBOX_COE)
        {
            ec_mbox_data_prealloc(&slave->mbox_coe_data, size);
        }
        else
        {
            ec_mbox_data_clear(&slave->mbox_coe_data);
        }
        if (protocols & EC_MBOX_FOE)
        {
            ec_mbox_data_prealloc(&slave->mbox_foe_data, size);
        }
        else
        {
            ec_mbox_data_clear(&slave->mbox_foe_data);
        }
        if (protocols & EC_MBOX_SOE)
        {
            ec_mbox_data_prealloc(&slave->mbox_soe_data, size);
        }
        else
        {
            ec_mbox_data_clear(&slave->mbox_soe_data);
        }
        if (protocols & EC_MBOX_VOE)
        {
            ec_mbox_data_prealloc(&slave->mbox_voe_data, size);
        }
        else
        {
            ec_mbox_data_clear(&slave->mbox_voe_data);
        }

        // 如果从站支持任何协议，则分配邮箱网关
        if (protocols)
        {
            ec_mbox_data_prealloc(&slave->mbox_mbg_data, size);
        }
        else
        {
            ec_mbox_data_clear(&slave->mbox_mbg_data);
        }
    }
}

/*****************************************************************************/

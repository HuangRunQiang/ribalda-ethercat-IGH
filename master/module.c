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

/** \file
 * EtherCAT主站驱动模块。
 */

/*****************************************************************************/

#include <linux/module.h>
#include <linux/device.h>
#include <linux/err.h>

#include "globals.h"
#include "master.h"
#include "device.h"

/*****************************************************************************/

#define MAX_MASTERS 32 /**< 最大主站数量。 */

/*****************************************************************************/

int __init ec_init_module(void);
void __exit ec_cleanup_module(void);

static int ec_mac_parse(uint8_t *, const char *, int);

/*****************************************************************************/

static char *main_devices[MAX_MASTERS];   /**< 主设备参数。 */
static unsigned int master_count;         /**< 主站数量。 */
static char *backup_devices[MAX_MASTERS]; /**< 备份设备参数。 */
static unsigned int backup_count;         /**< 备份设备数量。 */
#ifdef EC_EOE
char *eoe_interfaces[MAX_EOE]; /**< EOE接口参数。 */
unsigned int eoe_count;        /**< EOE接口数量。 */
bool eoe_autocreate = 1;       /**< EOE接口自动创建模式。 */
#endif
static unsigned int debug_level; /**< 调试级别参数。 */
unsigned long pcap_size;         /**< Pcap缓冲区大小（字节）。 */

static ec_master_t *masters; /**< 主站数组。 */
static ec_lock_t master_sem; /**< 主站信号量。 */

dev_t device_number; /**< 主站cdev的设备号。 */
struct class *class; /**< 设备类。 */

static uint8_t macs[MAX_MASTERS][2][ETH_ALEN]; /**< MAC地址。 */

char *ec_master_version_str = EC_MASTER_VERSION; /**< 版本字符串。 */

/*****************************************************************************/

/** \cond */

MODULE_AUTHOR("Florian Pose <fp@igh-essen.com>");
MODULE_DESCRIPTION("EtherCAT主站驱动模块");
MODULE_LICENSE("GPL");
MODULE_VERSION(EC_MASTER_VERSION);

module_param_array(main_devices, charp, &master_count, S_IRUGO);
MODULE_PARM_DESC(main_devices, "主设备的MAC地址");
module_param_array(backup_devices, charp, &backup_count, S_IRUGO);
MODULE_PARM_DESC(backup_devices, "备份设备的MAC地址");
#ifdef EC_EOE
module_param_array(eoe_interfaces, charp, &eoe_count, S_IRUGO);
MODULE_PARM_DESC(eoe_interfaces, "EOE接口");
module_param_named(eoe_autocreate, eoe_autocreate, bool, S_IRUGO);
MODULE_PARM_DESC(eoe_autocreate, "EOE自动创建模式");
#endif
module_param_named(debug_level, debug_level, uint, S_IRUGO);
MODULE_PARM_DESC(debug_level, "调试级别");
module_param_named(pcap_size, pcap_size, ulong, S_IRUGO);
MODULE_PARM_DESC(pcap_size, "Pcap缓冲区大小");

/** \endcond */

/*****************************************************************************/

/**
@brief 模块初始化。
@return 成功返回0，否则返回小于0的值。
@details 初始化 \a master_count 个主站。

- 打印主站驱动版本信息。
- 初始化主站的信号量。
- 如果主站数量不为0：
  - 获取设备号。
  - 创建设备类。
- 清零MAC地址数组。
- 处理MAC地址参数：
  - 解析主设备的MAC地址。
  - 如果备份设备数量不为0，解析备份设备的MAC地址。
- 初始化静态主站变量。
- 如果主站数量不为0：
  - 分配内存空间用于存储主站实例。
- 初始化每个主站：
  - 初始化主站实例。
- 打印等待设备的主站数量。

*/
int __init ec_init_module(void)
{
    int i, ret = 0;

    EC_INFO("主站驱动程序 %s\n", EC_MASTER_VERSION);

    ec_lock_init(&master_sem);

    if (master_count)
    {
        if (alloc_chrdev_region(&device_number,
                                0, master_count, "EtherCAT"))
        {
            EC_ERR("无法获取设备号！\n");
            ret = -EBUSY;
            goto out_return;
        }
    }

    class = class_create(THIS_MODULE, "EtherCAT");
    if (IS_ERR(class))
    {
        EC_ERR("无法创建设备类。\n");
        ret = PTR_ERR(class);
        goto out_cdev;
    }

    // 清零MAC地址
    memset(macs, 0x00, sizeof(uint8_t) * MAX_MASTERS * 2 * ETH_ALEN);

    // 处理MAC参数
    for (i = 0; i < master_count; i++)
    {
        ret = ec_mac_parse(macs[i][0], main_devices[i], 0);
        if (ret)
            goto out_class;

        if (i < backup_count)
        {
            ret = ec_mac_parse(macs[i][1], backup_devices[i], 1);
            if (ret)
                goto out_class;
        }
    }

    // 初始化静态主站变量
    ec_master_init_static();

    if (master_count)
    {
        if (!(masters = kmalloc(sizeof(ec_master_t) * master_count,
                                GFP_KERNEL)))
        {
            EC_ERR("无法为EtherCAT主站分配内存。\n");
            ret = -ENOMEM;
            goto out_class;
        }
    }

    for (i = 0; i < master_count; i++)
    {
        ret = ec_master_init(&masters[i], i, macs[i][0], macs[i][1],
                             device_number, class, debug_level);
        if (ret)
            goto out_free_masters;
    }

    EC_INFO("%u 个主站正在等待设备。\n",
            master_count, (master_count == 1 ? "" : "s"));
    return ret;

out_free_masters:
    for (i--; i >= 0; i--)
        ec_master_clear(&masters[i]);
    kfree(masters);
out_class:
    class_destroy(class);
out_cdev:
    if (master_count)
        unregister_chrdev_region(device_number, master_count);
out_return:
    return ret;
}

/*****************************************************************************/

/**
@brief 模块清理。
@details 清除所有主站实例。

- 清除所有主站实例。
- 如果主站数量不为0，释放主站实例的内存空间。
- 销毁设备类。
- 如果主站数量不为0，注销设备号。
- 打印主模块清理完成的信息。
*/
void __exit ec_cleanup_module(void)
{
    unsigned int i;

    for (i = 0; i < master_count; i++)
    {
        ec_master_clear(&masters[i]);
    }

    if (master_count)
        kfree(masters);

    class_destroy(class);

    if (master_count)
        unregister_chrdev_region(device_number, master_count);

    EC_INFO("主模块已清理。\n");
}

/*****************************************************************************/

/**
@brief 获取主站数量。
@return 主站数量。
*/
unsigned int ec_master_count(void)
{
    return master_count;
}
/*****************************************************************************
 * MAC address functions
 ****************************************************************************/

/**
 * @brief 判断两个MAC地址是否相等。
 *
 * 该函数用于比较两个MAC地址是否相等。
 *
 * @param mac1 第一个MAC地址。
 * @param mac2 第二个MAC地址。
 * @return 如果两个MAC地址相等则返回true，否则返回false。
 */
int ec_mac_equal(
    const uint8_t *mac1, /**< 第一个MAC地址。 */
    const uint8_t *mac2  /**< 第二个MAC地址。 */
)
{
    unsigned int i;

    for (i = 0; i < ETH_ALEN; i++)
        if (mac1[i] != mac2[i])
            return 0;

    return 1;
}

/*****************************************************************************/

/** MAC地址字符串的最大长度。
 */
#define EC_MAX_MAC_STRING_SIZE (3 * ETH_ALEN)

/** 将MAC地址打印到缓冲区。
 *
 * 该函数将给定的MAC地址打印成字符串形式，并存储到目标缓冲区中。
 * 目标缓冲区的大小必须至少为EC_MAX_MAC_STRING_SIZE。
 *
 * @param mac MAC地址。
 * @param buffer 目标缓冲区。
 * @return 写入的字节数。
 */
size_t ec_mac_print(
    const uint8_t *mac, /**< MAC地址 */
    char *buffer        /**< 目标缓冲区。 */
)
{
    size_t off = 0;
    unsigned int i;

    for (i = 0; i < ETH_ALEN; i++)
    {
        off += sprintf(buffer + off, "%02X", mac[i]);
        if (i < ETH_ALEN - 1)
            off += sprintf(buffer + off, ":");
    }

    return off;
}

/*****************************************************************************/

/**
 * @brief 判断MAC地址是否全为零。
 *
 * 该函数用于判断给定的MAC地址是否全为零。
 *
 * @param mac MAC地址。
 * @return 如果MAC地址全为零则返回true，否则返回false。
 */
int ec_mac_is_zero(
    const uint8_t *mac /**< MAC地址。 */
)
{
    unsigned int i;

    for (i = 0; i < ETH_ALEN; i++)
        if (mac[i])
            return 0;

    return 1;
}

/*****************************************************************************/

/**
 * @brief 判断给定的MAC地址是否为广播地址。
 *
 * 该函数用于判断给定的MAC地址是否为广播地址。
 *
 * @param mac MAC地址。
 * @return 如果给定的MAC地址是广播地址则返回true，否则返回false。
 */
int ec_mac_is_broadcast(
    const uint8_t *mac /**< MAC地址。 */
)
{
    unsigned int i;

    for (i = 0; i < ETH_ALEN; i++)
        if (mac[i] != 0xff)
            return 0;

    return 1;
}

/*****************************************************************************/

/** 从字符串解析MAC地址。
 *
 * 该函数用于从字符串中解析出MAC地址。
 * 解析的MAC地址必须符合正则表达式"([0-9a-fA-F]{2}:){5}[0-9a-fA-F]{2}"。
 *
 * @param mac MAC地址存储的目标缓冲区。
 * @param src 包含MAC地址的字符串。
 * @param allow_empty 是否允许空的MAC地址。
 * @return 如果解析成功则返回0，否则返回负数。
 */
static int ec_mac_parse(uint8_t *mac, const char *src, int allow_empty)
{
    unsigned int i, value;
    const char *orig = src;
    char *rem;

    if (!strlen(src))
    {
        if (allow_empty)
        {
            return 0;
        }
        else
        {
            EC_ERR("MAC地址不能为空。\n");
            return -EINVAL;
        }
    }

    for (i = 0; i < ETH_ALEN; i++)
    {
        value = simple_strtoul(src, &rem, 16);
        if (rem != src + 2 || value > 0xFF || (i < ETH_ALEN - 1 && *rem != ':'))
        {
            EC_ERR("无效的MAC地址 \"%s\"。\n", orig);
            return -EINVAL;
        }
        mac[i] = value;
        if (i < ETH_ALEN - 1)
        {
            src = rem + 1; // 跳过冒号
        }
    }

    return 0;
}

/*****************************************************************************/

/** 输出用于调试目的的数据帧内容。
 *
 * 如果数据块的大小大于256字节，则只显示前128字节和后128字节。
 *
 * @param data 指向数据的指针。
 * @param size 要输出的字节数。
 */
void ec_print_data(const uint8_t *data, /**< 指向数据的指针 */
                   size_t size          /**< 要输出的字节数 */
)
{
    unsigned int i;

    EC_DBG("");
    for (i = 0; i < size; i++)
    {
        printk(KERN_CONT "%02X ", data[i]);

        if ((i + 1) % 16 == 0 && i < size - 1)
        {
            printk(KERN_CONT "\n");
            EC_DBG("");
        }

        if (i + 1 == 128 && size > 256)
        {
            printk(KERN_CONT "丢弃 %zu 字节\n", size - 128 - i);
            i = size - 128;
            EC_DBG("");
        }
    }
    printk(KERN_CONT "\n");
}

/*****************************************************************************/

/** 输出用于调试目的的数据帧内容和差异。
 *
 * @param d1 第一个数据。
 * @param d2 第二个数据。
 * @param size 要输出的字节数。
 */
void ec_print_data_diff(const uint8_t *d1, /**< 第一个数据 */
                        const uint8_t *d2, /**< 第二个数据 */
                        size_t size        /** 要输出的字节数 */
)
{
    unsigned int i;

    EC_DBG("");
    for (i = 0; i < size; i++)
    {
        if (d1[i] == d2[i])
            printk(KERN_CONT ".. ");
        else
            printk(KERN_CONT "%02X ", d2[i]);
        if ((i + 1) % 16 == 0)
        {
            printk(KERN_CONT "\n");
            EC_DBG("");
        }
    }
    printk(KERN_CONT "\n");
}

/*****************************************************************************/

/** 以明文形式打印从站状态。
 *
 * @param states 从站状态。
 * @param buffer 目标缓冲区（至少EC_STATE_STRING_SIZE字节）。
 * @param multi 显示多状态掩码。
 * @return 创建的字符串的大小。
 */
size_t ec_state_string(uint8_t states, /**< 从站状态 */
                       char *buffer,   /**< 目标缓冲区
                                         （至少EC_STATE_STRING_SIZE字节） */
                       uint8_t multi   /**< 显示多状态掩码 */
)
{
    off_t off = 0;
    unsigned int first = 1;

    if (!states)
    {
        off += sprintf(buffer + off, "(未知)");
        return off;
    }

    if (multi)
    { // 多个从站
        if (states & EC_SLAVE_STATE_INIT)
        {
            off += sprintf(buffer + off, "INIT");
            first = 0;
        }
        if (states & EC_SLAVE_STATE_PREOP)
        {
            if (!first)
                off += sprintf(buffer + off, "，");
            off += sprintf(buffer + off, "PREOP");
            first = 0;
        }
        if (states & EC_SLAVE_STATE_SAFEOP)
        {
            if (!first)
                off += sprintf(buffer + off, "，");
            off += sprintf(buffer + off, "SAFEOP");
            first = 0;
        }
        if (states & EC_SLAVE_STATE_OP)
        {
            if (!first)
                off += sprintf(buffer + off, "，");
            off += sprintf(buffer + off, "OP");
        }
    }
    else
    { // 单个从站
        if ((states & EC_SLAVE_STATE_MASK) == EC_SLAVE_STATE_INIT)
        {
            off += sprintf(buffer + off, "INIT");
        }
        else if ((states & EC_SLAVE_STATE_MASK) == EC_SLAVE_STATE_PREOP)
        {
            off += sprintf(buffer + off, "PREOP");
        }
        else if ((states & EC_SLAVE_STATE_MASK) == EC_SLAVE_STATE_BOOT)
        {
            off += sprintf(buffer + off, "BOOT");
        }
        else if ((states & EC_SLAVE_STATE_MASK) == EC_SLAVE_STATE_SAFEOP)
        {
            off += sprintf(buffer + off, "SAFEOP");
        }
        else if ((states & EC_SLAVE_STATE_MASK) == EC_SLAVE_STATE_OP)
        {
            off += sprintf(buffer + off, "OP");
        }
        else
        {
            off += sprintf(buffer + off, "(无效)");
        }
        first = 0;
    }

    if (states & EC_SLAVE_STATE_ACK_ERR)
    {
        if (!first)
            off += sprintf(buffer + off, " + ");
        off += sprintf(buffer + off, "错误");
    }

    return off;
}

/******************************************************************************
 *  Device interface
 *****************************************************************************/

/**
 * @brief 提供一个EtherCAT设备给特定的主站。
 *
 * 主站决定是否要将该设备用于EtherCAT操作。
 * 重要的是，所提供的net_device不能被内核IP栈使用。
 * 如果主站接受了该设备的提供，将返回新创建的EtherCAT设备的地址，否则返回NULL。
 *
 * @param net_dev 要提供的net_device
 * @param poll 设备的轮询函数
 * @param module 指向模块的指针
 * @return 如果接受了提供，则返回设备的指针；如果拒绝了提供，则返回NULL。
 * @ingroup DeviceInterface
 */
ec_device_t *ecdev_offer(
    struct net_device *net_dev, /**< 要提供的net_device */
    ec_pollfunc_t poll,         /**< 设备的轮询函数 */
    struct module *module       /**< 指向模块的指针 */
)
{
    ec_master_t *master;
    char str[EC_MAX_MAC_STRING_SIZE];
    unsigned int i, dev_idx;

    for (i = 0; i < master_count; i++)
    {
        master = &masters[i];
        ec_mac_print(net_dev->dev_addr, str);

        if (ec_lock_down_interruptible(&master->device_sem))
        {
            EC_MASTER_WARN(master, "%s() 被中断！\n", __func__);
            return NULL;
        }

        for (dev_idx = EC_DEVICE_MAIN;
             dev_idx < ec_master_num_devices(master); dev_idx++)
        {
            if (!master->devices[dev_idx].dev && (ec_mac_equal(master->macs[dev_idx], net_dev->dev_addr) || ec_mac_is_broadcast(master->macs[dev_idx])))
            {

                EC_INFO("接受 %s 作为主站 %u 的%s设备。\n",
                        str, ec_device_names[dev_idx != 0], master->index);

                ec_device_attach(&master->devices[dev_idx],
                                 net_dev, poll, module);
                ec_lock_up(&master->device_sem);

                snprintf(net_dev->name, IFNAMSIZ, "ec%c%u",
                         ec_device_names[dev_idx != 0][0], master->index);

                return &master->devices[dev_idx]; // 提供被接受
            }
        }

        ec_lock_up(&master->device_sem);

        EC_MASTER_DBG(master, 1, "主站拒绝设备 %s。\n", str);
    }

    return NULL; // 提供被拒绝
}

/******************************************************************************
 * Application interface
 *****************************************************************************/

/**
@brief 请求一个主站。
@param master_index 主站索引。
@return 请求的主站。
@details 
- 请求一个主站，与ecrt_request_master()相同，但返回值为ERR_PTR()。
- 如果主站索引无效，返回错误指针，并打印错误信息。
- 如果主站已被占用，返回错误指针，并打印错误信息。
- 如果无法获取设备模块，返回错误指针，并打印错误信息。
- 如果无法进入操作阶段，返回错误指针，并打印错误信息。
- 成功请求主站，返回主站指针。
*/

ec_master_t *ecrt_request_master_err(
    unsigned int master_index /**< 主站索引。 */
)
{
    ec_master_t *master, *errptr = NULL;
    unsigned int dev_idx = EC_DEVICE_MAIN;

    EC_INFO("正在请求主站 %u...\n", master_index);

    if (master_index >= master_count)
    {
        EC_ERR("无效的主站索引 %u。\n", master_index);
        errptr = ERR_PTR(-EINVAL);
        goto out_return;
    }
    master = &masters[master_index];

    if (ec_lock_down_interruptible(&master_sem))
    {
        errptr = ERR_PTR(-EINTR);
        goto out_return;
    }

    if (master->reserved)
    {
        ec_lock_up(&master_sem);
        EC_MASTER_ERR(master, "主站已被占用！\n");
        errptr = ERR_PTR(-EBUSY);
        goto out_return;
    }
    master->reserved = 1;
    ec_lock_up(&master_sem);

    if (ec_lock_down_interruptible(&master->device_sem))
    {
        errptr = ERR_PTR(-EINTR);
        goto out_release;
    }

    if (master->phase != EC_IDLE)
    {
        ec_lock_up(&master->device_sem);
        EC_MASTER_ERR(master, "主站仍在等待设备！\n");
        errptr = ERR_PTR(-ENODEV);
        goto out_release;
    }

    for (; dev_idx < ec_master_num_devices(master); dev_idx++)
    {
        ec_device_t *device = &master->devices[dev_idx];
        if (!try_module_get(device->module))
        {
            ec_lock_up(&master->device_sem);
            EC_MASTER_ERR(master, "设备模块正在卸载！\n");
            errptr = ERR_PTR(-ENODEV);
            goto out_module_put;
        }
    }

    ec_lock_up(&master->device_sem);

    if (ec_master_enter_operation_phase(master))
    {
        EC_MASTER_ERR(master, "进入操作阶段失败！\n");
        errptr = ERR_PTR(-EIO);
        goto out_module_put;
    }

    EC_INFO("成功请求主站 %u。\n", master_index);
    return master;

out_module_put:
    for (; dev_idx > 0; dev_idx--)
    {
        ec_device_t *device = &master->devices[dev_idx - 1];
        module_put(device->module);
    }
out_release:
    master->reserved = 0;
out_return:
    return errptr;
}

/*****************************************************************************/

/**
 * @brief 请求EtherCAT主站
 * @param master_index 主站索引
 * @return 返回请求到的EtherCAT主站指针，如果请求失败则返回NULL
 * @details 请求指定索引的EtherCAT主站，并返回主站指针。如果请求失败，返回NULL。
 */
ec_master_t *ecrt_request_master(unsigned int master_index)
{
    ec_master_t *master = ecrt_request_master_err(master_index);
    return IS_ERR(master) ? NULL : master;
}

/*****************************************************************************/

/**
 * @brief 释放EtherCAT主站
 * @param master EtherCAT主站指针
 * @details 释放指定的EtherCAT主站资源。如果主站未被请求，则打印警告信息并返回。
 *          离开操作阶段，释放主站下的设备模块，将主站的reserved标志置为0。
 */
void ecrt_release_master(ec_master_t *master)
{
    unsigned int dev_idx;

    EC_MASTER_INFO(master, "释放主站...\n");

    if (!master->reserved)
    {
        EC_MASTER_WARN(master, "%s(): 主站未被请求！\n",
                       __func__);
        return;
    }

    ec_master_leave_operation_phase(master);

    for (dev_idx = EC_DEVICE_MAIN; dev_idx < ec_master_num_devices(master);
         dev_idx++)
    {
        module_put(master->devices[dev_idx].module);
    }

    master->reserved = 0;

    EC_MASTER_INFO(master, "已释放。\n");
}

/*****************************************************************************/

/**
 * @brief 获取EtherCAT库的版本号
 * @return 返回EtherCAT库的版本号
 */
unsigned int ecrt_version_magic(void)
{
    return ECRT_VERSION_MAGIC;
}

/*****************************************************************************/

/**
 * @brief 全局请求状态类型翻译表
 *
 * 将内部请求状态翻译为外部状态。
 */
const ec_request_state_t ec_request_state_translation_table[] = {
    EC_REQUEST_UNUSED,  // EC_INT_REQUEST_INIT，未使用
    EC_REQUEST_BUSY,    // EC_INT_REQUEST_QUEUED，请求已排队
    EC_REQUEST_BUSY,    // EC_INT_REQUEST_BUSY，请求忙碌中
    EC_REQUEST_SUCCESS, // EC_INT_REQUEST_SUCCESS，请求成功
    EC_REQUEST_ERROR    // EC_INT_REQUEST_FAILURE，请求失败
};

/*****************************************************************************/

/** \cond */

module_init(ec_init_module);
module_exit(ec_cleanup_module);

EXPORT_SYMBOL(ecdev_offer);

EXPORT_SYMBOL(ecrt_request_master);
EXPORT_SYMBOL(ecrt_release_master);
EXPORT_SYMBOL(ecrt_version_magic);

/** \endcond */

/*****************************************************************************/

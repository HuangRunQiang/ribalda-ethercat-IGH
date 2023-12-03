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
   Ethernet over EtherCAT (EoE).
*/

/*****************************************************************************/

#include <linux/version.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>

#include "globals.h"
#include "master.h"
#include "slave.h"
#include "mailbox.h"
#include "ethernet.h"

/*****************************************************************************/

/** 定义EoE处理的调试级别。
 *
 * 0 = 无调试消息。
 * 1 = 输出警告。
 * 2 = 输出动作。
 * 3 = 输出动作和帧数据。
 */
#define EOE_DEBUG_LEVEL 1

/** EoE发送环的大小。
 */
#define EC_EOE_TX_RING_SIZE 100

/** 尝试次数。
 */
#define EC_EOE_TRIES 100

/*****************************************************************************/

void ec_eoe_flush(ec_eoe_t *);
static unsigned int eoe_tx_unused_frames(ec_eoe_t *);

// 状态函数
void ec_eoe_state_rx_start(ec_eoe_t *);
void ec_eoe_state_rx_check(ec_eoe_t *);
void ec_eoe_state_rx_fetch(ec_eoe_t *);
void ec_eoe_state_rx_fetch_data(ec_eoe_t *);
void ec_eoe_state_tx_start(ec_eoe_t *);
void ec_eoe_state_tx_sent(ec_eoe_t *);

// net_device函数
int ec_eoedev_open(struct net_device *);
int ec_eoedev_stop(struct net_device *);
int ec_eoedev_tx(struct sk_buff *, struct net_device *);
struct net_device_stats *ec_eoedev_stats(struct net_device *);
static int ec_eoedev_set_mac(struct net_device *netdev, void *p);


/*****************************************************************************/

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 29)
/**
 * @brief EoE接口的设备操作。
 * @details 该结构定义了EoE接口的设备操作函数。
 */
static const struct net_device_ops ec_eoedev_ops = {
    .ndo_open = ec_eoedev_open,                /**< 打开设备的操作函数。 */
    .ndo_stop = ec_eoedev_stop,                /**< 停止设备的操作函数。 */
    .ndo_start_xmit = ec_eoedev_tx,            /**< 发送数据的操作函数。 */
    .ndo_get_stats = ec_eoedev_stats,          /**< 获取统计信息的操作函数。 */
    .ndo_set_mac_address = ec_eoedev_set_mac,  /**< 设置MAC地址的操作函数。 */
};
#endif

/*****************************************************************************/

/**
 * @brief 更改网络接口设备的以太网地址。
 * @param netdev 网络接口设备结构。
 * @param p 指向地址结构的指针。
 * @return 成功返回0，失败返回负值。
 * @details 该函数用于更改网络接口设备的以太网地址。它会将传入地址结构中的地址复制到设备的地址中。
 */
static int ec_eoedev_set_mac(struct net_device *netdev, void *p)
{
    struct sockaddr *addr = p;

    if (!is_valid_ether_addr(addr->sa_data))
    {
        return -EADDRNOTAVAIL;
    }

    memcpy(netdev->dev_addr, addr->sa_data, netdev->addr_len);

    return 0;
}

/*****************************************************************************/

/**
 * @brief 从字符串中解析EOE接口。
 * @param eoe EOE接口字符串。
 * @param master_idx 主站索引。
 * @param alias 别名。
 * @param posn 位置。
 * @return 成功返回0，否则返回小于0的值。
 * @details 该函数从字符串中解析EOE接口。EOE接口必须与正则表达式"eoe([0-9]*)([as])([0-9]*)"匹配。
 */
int ec_eoe_parse(const char *eoe, int *master_idx,
                 uint16_t *alias, uint16_t *posn)
{
    unsigned int value;
    const char *orig = eoe;
    char *rem;

    if (!strlen(eoe))
    {
        EC_ERR("EOE接口不能为空。\n");
        return -EINVAL;
    }

    // 必须以"eoe"开头
    if (strncmp(eoe, "eoe", 3) != 0)
    {
        EC_ERR("无效的EOE接口\"%s\"。\n", orig);
        return -EINVAL;
    }
    eoe += 3;

    // 获取主站索引，此处仅检查主站索引不为负数
    value = simple_strtoul(eoe, &rem, 10);
    if (value < 0)
    {
        EC_ERR("无效的EOE接口\"%s\"，主站索引：%d\n", orig, value);
        return -EINVAL;
    }
    *master_idx = value;
    eoe = rem;

    // 获取别名或位置指示器
    if (eoe[0] == 'a')
    {
        eoe++;
        value = simple_strtoul(eoe, &rem, 10);
        if ((value <= 0) || (value >= 0xFFFF))
        {
            EC_ERR("无效的EOE接口\"%s\"，无效的别名：%d\n",
                   orig, value);
            return -EINVAL;
        }
        *alias = value;
        *posn = 0;
    }
    else if (eoe[0] == 's')
    {
        eoe++;
        value = simple_strtoul(eoe, &rem, 10);
        if ((value < 0) || (value >= 0xFFFF))
        {
            EC_ERR("无效的EOE接口\"%s\"，无效的环位置：%d\n",
                   orig, value);
            return -EINVAL;
        }
        *alias = 0;
        *posn = value;
    }
    else
    {
        EC_ERR("无效的EOE接口\"%s\"，无效的别名/位置指示器：%c\n",
               orig, eoe[0]);
        return -EINVAL;
    }

    // 检查无剩余字符
    if (rem[0] != '\0')
    {
        EC_ERR("无效的EOE接口\"%s\"，意外的结尾字符：%s\n",
               orig, rem);
        return -EINVAL;
    }

    return 0;
}


/*****************************************************************************/

/**
 * @brief EoE显式初始化构造函数。
 *
 * 在配置从站之前初始化EoE处理程序，创建net_device并注册它。
 *
 * @param master EtherCAT主站
 * @param eoe EoE处理程序
 * @param alias EtherCAT从站别名
 * @param ring_position EtherCAT从站环位置
 * @return 成功返回零，否则返回负错误代码。
 * @details 该函数用于在配置从站之前初始化EoE处理程序，包括初始化各种参数和创建net_device，并将其注册。同时会设置设备的MAC地址，最后将设备注册到内核中。
 */
int ec_eoe_init(
    ec_master_t *master,   /**< EtherCAT主站 */
    ec_eoe_t *eoe,         /**< EoE处理程序 */
    uint16_t alias,        /**< EtherCAT从站别名 */
    uint16_t ring_position /**< EtherCAT从站环位置 */
)
{
    ec_eoe_t **priv;
    int ret = 0;
    char name[EC_DATAGRAM_NAME_SIZE];

    struct net_device *dev;
    unsigned char lo_mac[ETH_ALEN] = {0};
    unsigned int use_master_mac = 0;

    eoe->master = master;
    eoe->slave = NULL;
    eoe->have_mbox_lock = 0;
    eoe->auto_created = 0;

    ec_datagram_init(&eoe->datagram);
    eoe->queue_datagram = 0;
    eoe->state = ec_eoe_state_rx_start;
    eoe->opened = 0;
    eoe->rx_skb = NULL;
    eoe->rx_expected_fragment = 0;

    eoe->tx_ring_count = EC_EOE_TX_RING_SIZE;
    eoe->tx_ring_size = sizeof(struct sk_buff *) * eoe->tx_ring_count;
    eoe->tx_ring = kmalloc(eoe->tx_ring_size, GFP_KERNEL);
    memset(eoe->tx_ring, 0, eoe->tx_ring_size);
    eoe->tx_next_to_use = 0;
    eoe->tx_next_to_clean = 0;
    eoe->tx_skb = NULL;
    eoe->tx_queue_active = 0;

    eoe->tx_frame_number = 0xFF;
    memset(&eoe->stats, 0, sizeof(struct net_device_stats));

    eoe->rx_counter = 0;
    eoe->tx_counter = 0;
    eoe->rx_rate = 0;
    eoe->tx_rate = 0;
    eoe->rate_jiffies = 0;
    eoe->rx_idle = 1;
    eoe->tx_idle = 1;

    /* 设备名称为eoe<MASTER>[as]<SLAVE>，因为网络脚本不喜欢在接口名中使用连字符等特殊字符。 */
    if (alias)
    {
        snprintf(name, EC_DATAGRAM_NAME_SIZE, "eoe%ua%u", master->index, alias);
    }
    else
    {
        snprintf(name, EC_DATAGRAM_NAME_SIZE, "eoe%us%u", master->index, ring_position);
    }

    snprintf(eoe->datagram.name, EC_DATAGRAM_NAME_SIZE, name);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 17, 0)
    eoe->dev = alloc_netdev(sizeof(ec_eoe_t *), name, NET_NAME_UNKNOWN,
                            ether_setup);
#else
    eoe->dev = alloc_netdev(sizeof(ec_eoe_t *), name, ether_setup);
#endif
    if (!eoe->dev)
    {
        EC_MASTER_ERR(master, "无法为EoE处理程序分配net_device %s！\n",
                      name);
        ret = -ENODEV;
        goto out_return;
    }

    // 初始化net_device
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 29)
    eoe->dev->netdev_ops = &ec_eoedev_ops;
#else
    eoe->dev->open = ec_eoedev_open;
    eoe->dev->stop = ec_eoedev_stop;
    eoe->dev->hard_start_xmit = ec_eoedev_tx;
    eoe->dev->get_stats = ec_eoedev_stats;
#endif

    // 首先检查分配给主站的MAC地址是否是全球唯一的
    if ((master->devices[EC_DEVICE_MAIN].dev->dev_addr[0] & 0x02) !=
        0x02)
    {
        // 主站MAC地址是唯一的，可以使用NIC部分作为EoE接口的MAC地址
        use_master_mac = 1;
    }
    else
    {
        // 主站MAC地址不是唯一的，因此我们检查其他接口中是否有唯一的MAC地址
        dev = first_net_device(&init_net);
        while (dev)
        {
            // 检查是否是全球唯一的MAC地址
            if (dev->addr_len == ETH_ALEN)
            {
                if (memcmp(dev->dev_addr, lo_mac, ETH_ALEN) != 0)
                {
                    if ((dev->dev_addr[0] & 0x02) != 0x02)
                    {
                        // 找到了第一个全球唯一的MAC地址
                        break;
                    }
                }
            }
            dev = next_net_device(dev);
        }
        if (eoe->dev->addr_len == ETH_ALEN)
        {
            if (dev)
            {
                // 在其他网络接口中找到了一个唯一的MAC地址，可以使用NIC部分作为EoE接口的MAC地址。
                EC_MASTER_INFO(master, "%s MAC地址派生自%s MAC地址的NIC部分\n",
                               eoe->dev->name, dev->name);
                eoe->dev->dev_addr[1] = dev->dev_addr[3];
                eoe->dev->dev_addr[2] = dev->dev_addr[4];
                eoe->dev->dev_addr[3] = dev->dev_addr[5];
            }
            else
            {
                use_master_mac = 1;
            }
        }
    }
    if (eoe->dev->addr_len == ETH_ALEN)
    {
        if (use_master_mac)
        {
            EC_MASTER_INFO(master, "%s MAC地址派生自%s MAC地址的NIC部分\n",
                           eoe->dev->name,
                           master->devices[EC_DEVICE_MAIN].dev->name);
            eoe->dev->dev_addr[1] =
                master->devices[EC_DEVICE_MAIN].dev->dev_addr[3];
            eoe->dev->dev_addr[2] =
                master->devices[EC_DEVICE_MAIN].dev->dev_addr[4];
            eoe->dev->dev_addr[3] =
                master->devices[EC_DEVICE_MAIN].dev->dev_addr[5];
        }
        eoe->dev->dev_addr[0] = 0x02;
        if (alias)
        {
            eoe->dev->dev_addr[4] = (uint8_t)(alias >> 8);
            eoe->dev->dev_addr[5] = (uint8_t)(alias);
        }
        else
        {
            eoe->dev->dev_addr[4] = (uint8_t)(ring_position >> 8);
            eoe->dev->dev_addr[5] = (uint8_t)(ring_position);
        }
    }

    // 初始化私有数据
    priv = netdev_priv(eoe->dev);
    *priv = eoe;

    // 将net_device连接到内核
    ret = register_netdev(eoe->dev);
    if (ret)
    {
        EC_MASTER_ERR(master, "无法注册net_device %s：错误 %i\n",
                      eoe->dev->name, ret);
        goto out_free;
    }

    // 在打开之前设置carrier off状态
    EC_MASTER_DBG(eoe->master, 1, "%s：carrier off。\n", eoe->dev->name);
    netif_carrier_off(eoe->dev);

    return 0;

out_free:
    free_netdev(eoe->dev);
    eoe->dev = NULL;
out_return:
    return ret;
}

/*****************************************************************************/

/** EoE从站自动初始化构造函数。
 *
 * 初始化EoE处理程序，创建net_device并注册它。
 *
 * \return 成功返回零，否则返回负错误代码。
 * @param eoe EoE处理程序
 * @param slave EtherCAT从站
 * @details 该函数用于自动初始化EoE处理程序，调用ec_eoe_init函数初始化EoE处理程序并设置自动创建标志，然后连接EoE处理程序和从站。
 */
int ec_eoe_auto_init(
    ec_eoe_t *eoe,    /**< EoE处理程序 */
    ec_slave_t *slave /**< EtherCAT从站 */
)
{
    int ret = 0;

    if ((ret = ec_eoe_init(slave->master, eoe, slave->effective_alias,
                           slave->ring_position)) != 0)
    {
        return ret;
    }

    // 设置自动创建标志
    eoe->auto_created = 1;

    ec_eoe_link_slave(eoe, slave);

    return ret;
}


/*****************************************************************************/

/**
 * @brief EoE链接从站。
 *
 * 在从站连接或重新配置后，将从站链接到处理程序。
 *
 * @param eoe EoE处理程序
 * @param slave EtherCAT从站
 * @return 无
 * @details 该函数用于将从站链接到EoE处理程序，设置EoE处理程序的从站指针，并根据情况设置MTU和carrier状态。
 */
void ec_eoe_link_slave(
    ec_eoe_t *eoe,    /**< EoE处理程序 */
    ec_slave_t *slave /**< EtherCAT从站 */
)
{
    eoe->slave = slave;

    if (eoe->slave)
    {
        EC_SLAVE_INFO(slave, "链接到EoE处理程序 %s\n",
                      eoe->dev->name);

        // 通常，适当设置MTU会使上层进行帧分段。在某些情况下，这种方法不起作用，因此MTU保持在以太网标准值上，并进行“手动”分段。
#if 0
        eoe->dev->mtu = slave->configured_rx_mailbox_size - ETH_HLEN - 10;
#endif

        EC_MASTER_DBG(eoe->master, 1, "%s：carrier on。\n", eoe->dev->name);
        netif_carrier_on(eoe->dev);
    }
    else
    {
        EC_MASTER_ERR(eoe->master, "%s：未提供从站给ec_eoe_link_slave()。\n",
                      eoe->dev->name);
        EC_MASTER_DBG(eoe->master, 1, "%s：carrier off。\n", eoe->dev->name);
        netif_carrier_off(eoe->dev);
    }
}

/*****************************************************************************/

/** EoE清除从站。
 *
 * 将从站与处理程序解除链接，以便在从站断开连接时保留EoE接口。
 *
 * @param eoe EoE处理程序
 * @return 无
 * @details 该函数用于清除EoE处理程序中的从站。关闭carrier状态，并清空发送队列。
 * 如果存在待发送的skb（套接字缓冲区），则释放该skb并增加tx_errors统计值。如果存在待接收的skb，同样释放该skb并增加rx_errors统计值。
 * 将EoE处理程序的state状态设置为ec_eoe_state_rx_start，表示接收状态。
 * 最后，将EoE处理程序的slave成员设置为NULL，并根据需要打印调试信息表示从站链接已清除。
 */
void ec_eoe_clear_slave(ec_eoe_t *eoe /**< EoE处理程序 */)
{
#if EOE_DEBUG_LEVEL >= 1
    ec_slave_t *slave = eoe->slave;
#endif

    EC_MASTER_DBG(eoe->master, 1, "%s：carrier off。\n", eoe->dev->name);
    netif_carrier_off(eoe->dev);

    // 清空发送队列
    ec_eoe_flush(eoe);

    if (eoe->tx_skb)
    {
        dev_kfree_skb(eoe->tx_skb);
        eoe->tx_skb = NULL;
        eoe->stats.tx_errors++;
    }

    if (eoe->rx_skb)
    {
        dev_kfree_skb(eoe->rx_skb);
        eoe->rx_skb = NULL;
        eoe->stats.rx_errors++;
    }

    eoe->state = ec_eoe_state_rx_start;

    eoe->slave = NULL;

#if EOE_DEBUG_LEVEL >= 1
    if (slave)
    {
        EC_MASTER_DBG(eoe->master, 0, "%s从站链接已清除。\n", eoe->dev->name);
    }
#endif
}

/*****************************************************************************/

/** EoE析构函数。
 *
 * 注销net_device并释放分配的内存。
 *
 * @param eoe EoE处理程序
 * @return 无
 * @details 该函数用于释放EoE处理程序的资源。
 * 注销net_device，并可能调用关闭回调函数。
 * 然后，清空发送队列。
 * 如果存在待发送的skb（套接字缓冲区），则释放该skb。如果存在待接收的skb，同样释放该skb。
 * 释放EoE处理程序的tx_ring内存，并释放net_device。最后，清除EoE处理程序的datagram数据结构。
 */
void ec_eoe_clear(ec_eoe_t *eoe /**< EoE处理程序 */)
{
    unregister_netdev(eoe->dev); // 可能会调用关闭回调函数

    // 清空发送队列
    ec_eoe_flush(eoe);

    if (eoe->tx_skb)
        dev_kfree_skb(eoe->tx_skb);

    if (eoe->rx_skb)
        dev_kfree_skb(eoe->rx_skb);

    kfree(eoe->tx_ring);

    free_netdev(eoe->dev);

    ec_datagram_clear(&eoe->datagram);
}

/*****************************************************************************/

/** 清空发送队列。
 *
 * @param eoe EoE处理程序
 * @return 无
 * @details 该函数用于清空EoE处理程序的发送队列。
 * 首先检查是否存在mbox锁，如果存在则清除该锁。
 * 然后，循环遍历发送队列，释放每个skb（套接字缓冲区），并将对应的tx_ring元素设置为NULL。
 * 同时，增加tx_dropped统计值。
 * 如果tx_next_to_clean加1后的值等于tx_ring_count，则将tx_next_to_clean设置为0，以实现循环队列的效果。
 * 最后，将tx_next_to_use和tx_next_to_clean都设置为0，表示发送队列已清空。
 */
void ec_eoe_flush(ec_eoe_t *eoe /**< EoE处理程序 */)
{
    struct sk_buff *skb;

    if (eoe->have_mbox_lock)
    {
        eoe->have_mbox_lock = 0;
        ec_read_mbox_lock_clear(eoe->slave);
    }

    while (eoe->tx_next_to_clean != eoe->tx_next_to_use)
    {
        skb = eoe->tx_ring[eoe->tx_next_to_clean];
        dev_kfree_skb(skb);
        eoe->tx_ring[eoe->tx_next_to_clean] = NULL;

        eoe->stats.tx_dropped++;

        if (unlikely(++eoe->tx_next_to_clean == eoe->tx_ring_count))
        {
            eoe->tx_next_to_clean = 0;
        }
    }

    eoe->tx_next_to_use = 0;
    eoe->tx_next_to_clean = 0;
}

/*****************************************************************************/

/**
 * @brief 获取EoE处理程序中待发送的帧数。
 * 
 * @param eoe EoE处理程序
 * @return 待发送的帧数
 * @details 该函数用于获取EoE处理程序中待发送的帧数。
 * 通过比较tx_next_to_use和tx_next_to_clean的值来计算待发送的帧数。
 * 如果tx_next_to_use大于或等于tx_next_to_clean，则返回它们之间的差值。
 * 否则，返回tx_ring_count加上tx_next_to_use和tx_next_to_clean之间的差值。
 */
unsigned int ec_eoe_tx_queued_frames(const ec_eoe_t *eoe /**< EoE处理程序 */)
{
    unsigned int next_to_use = eoe->tx_next_to_use;
    unsigned int next_to_clean = eoe->tx_next_to_clean;

    if (next_to_use >= next_to_clean)
    {
        return next_to_use - next_to_clean;
    }
    else
    {
        return next_to_use + eoe->tx_ring_count - next_to_clean;
    }
}

/*****************************************************************************/

/**
 * @brief 获取EoE处理程序中未使用的帧数。
 * 
 * @param eoe EoE处理程序
 * @return 未使用的帧数
 * @details 该函数用于获取EoE处理程序中未使用的帧数。
 * 通过比较tx_next_to_clean和tx_next_to_use的值来计算未使用的帧数。
 * 如果tx_next_to_clean大于tx_next_to_use，则返回它们之间的差值减1。
 * 否则，返回tx_ring_count加上tx_next_to_clean和tx_next_to_use之间的差值减1。
 * 注意，减1是为了避免尾部接触头部的情况。
 */
static unsigned int eoe_tx_unused_frames(ec_eoe_t *eoe /**< EoE处理程序 */)
{
    unsigned int next_to_use = eoe->tx_next_to_use;
    unsigned int next_to_clean = eoe->tx_next_to_clean;

    // 注意：-1是为了避免尾部接触头部
    if (next_to_clean > next_to_use)
    {
        return next_to_clean - next_to_use - 1;
    }
    else
    {
        return next_to_clean + eoe->tx_ring_count - next_to_use - 1;
    }
}

/*****************************************************************************/

/**
 * @brief 发送帧或下一个片段。
 * 
 * @param eoe EoE处理器
 * @return 成功时返回零，否则返回负错误代码。
 * @details 该函数根据当前的传输进度发送帧或下一个片段。它首先计算剩余数据的大小和当前片段的大小，然后确定是否为最后一个片段。接下来，根据片段的序号和偏移量构造消息头，并将数据拷贝到消息体中。最后，更新传输进度并返回结果。
 */
int ec_eoe_send(ec_eoe_t *eoe /**< EoE处理器 */)
{
    size_t remaining_size, current_size, complete_offset;
    unsigned int last_fragment;
    uint8_t *data;
#if EOE_DEBUG_LEVEL >= 3
    unsigned int i;
#endif

    if (!eoe->slave)
    {
        return -ECHILD;
    }

    remaining_size = eoe->tx_skb->len - eoe->tx_offset;

    if (remaining_size <= eoe->slave->configured_tx_mailbox_size - 10)
    {
        current_size = remaining_size;
        last_fragment = 1;
    }
    else
    {
        current_size =
            ((eoe->slave->configured_tx_mailbox_size - 10) / 32) * 32;
        last_fragment = 0;
    }

    if (eoe->tx_fragment_number)
    {
        complete_offset = eoe->tx_offset / 32;
    }
    else
    {
        // complete size in 32 bit blocks, rounded up.
        complete_offset = remaining_size / 32 + 1;
    }

#if EOE_DEBUG_LEVEL >= 2
    EC_SLAVE_DBG(eoe->slave, 0, "EoE %s TX sending fragment %u%s"
                                " with %zu octets (%zu). %u frames queued.\n",
                 eoe->dev->name, eoe->tx_fragment_number,
                 last_fragment ? "" : "+", current_size, complete_offset,
                 ec_eoe_tx_queued_frames(eoe));
#endif

#if EOE_DEBUG_LEVEL >= 3
    EC_SLAVE_DBG(eoe->slave, 0, "");
    for (i = 0; i < current_size; i++)
    {
        printk(KERN_CONT "%02X ", eoe->tx_skb->data[eoe->tx_offset + i]);
        if ((i + 1) % 16 == 0)
        {
            printk(KERN_CONT "\n");
            EC_SLAVE_DBG(eoe->slave, 0, "");
        }
    }
    printk(KERN_CONT "\n");
#endif

    data = ec_slave_mbox_prepare_send(eoe->slave, &eoe->datagram,
                                      EC_MBOX_TYPE_EOE, current_size + 4);
    if (IS_ERR(data))
    {
        return PTR_ERR(data);
    }

    EC_WRITE_U8(data, EC_EOE_TYPE_FRAME_FRAG); // Initiate EoE Tx Request
    EC_WRITE_U8(data + 1, last_fragment);
    EC_WRITE_U16(data + 2, ((eoe->tx_fragment_number & 0x3F) |
                            (complete_offset & 0x3F) << 6 |
                            (eoe->tx_frame_number & 0x0F) << 12));

    memcpy(data + 4, eoe->tx_skb->data + eoe->tx_offset, current_size);
    eoe->queue_datagram = 1;

    eoe->tx_offset += current_size;
    eoe->tx_fragment_number++;
    return 0;
}

/*****************************************************************************/

/**
 * @brief 运行EoE状态机。
 * 
 * @param eoe EoE处理器
 * @details 该函数运行EoE状态机。首先检查EoE处理器的状态，如果处理器未打开、未关联从站或网络接口不可用，则直接返回。然后检查是否需要发送数据报文或已收到数据报文，如果是则跳过当前循环。接下来调用状态函数执行状态转换逻辑。最后更新统计信息并输出数据报文的统计信息。
 */
void ec_eoe_run(ec_eoe_t *eoe /**< EoE处理器 */)
{
    if (!eoe->opened || !eoe->slave || !netif_carrier_ok(eoe->dev))
    {
        return;
    }

    // if the datagram was not sent, or is not yet received, skip this cycle
    if (eoe->queue_datagram || eoe->datagram.state == EC_DATAGRAM_SENT)
    {
        return;
    }

    // call state function
    eoe->state(eoe);

    // update statistics
    if (jiffies - eoe->rate_jiffies > HZ)
    {
        eoe->rx_rate = eoe->rx_counter;
        eoe->tx_rate = eoe->tx_counter;
        eoe->rx_counter = 0;
        eoe->tx_counter = 0;
        eoe->rate_jiffies = jiffies;
    }

    ec_datagram_output_stats(&eoe->datagram);
}

/*****************************************************************************/

/**
 * @brief 如果需要，将数据报文加入队列。
 * 
 * @param eoe EoE处理器
 * @details 该函数将数据报文加入队列，如果需要且从站已关联。它通过调用ec_master_queue_datagram_ext函数将数据报文加入主站队列，并将eoe->queue_datagram标志位清零。
 */
void ec_eoe_queue(ec_eoe_t *eoe /**< EoE处理器 */)
{
    if (eoe->queue_datagram && eoe->slave)
    {
        ec_master_queue_datagram_ext(eoe->slave->master, &eoe->datagram);
        eoe->queue_datagram = 0;
    }
}

/*****************************************************************************/

/**
 * @brief 返回设备的状态。
 * 
 * @param eoe EoE处理器
 * @return 如果设备处于"up"状态则返回1，如果设备处于"down"状态则返回0。
 */
int ec_eoe_is_open(const ec_eoe_t *eoe /**< EoE处理器 */)
{
    return eoe->opened;
}

/*****************************************************************************/

/**
 * @brief 返回设备的空闲状态。
 * 
 * @param eoe EoE处理器
 * @retval 1 设备处于空闲状态。
 * @retval 0 设备正在忙碌。
 */
int ec_eoe_is_idle(const ec_eoe_t *eoe /**< EoE处理器 */)
{
    return eoe->rx_idle && eoe->tx_idle;
}

/*****************************************************************************/

/**
 * @brief 返回EoE设备的名称。
 * 
 * @param eoe EoE处理器
 * @return 设备名称。
 */
char *ec_eoe_name(const ec_eoe_t *eoe /**< EoE处理器 */)
{
    return eoe->dev->name;
}


/******************************************************************************
 *  STATE PROCESSING FUNCTIONS
 *****************************************************************************/

/**
 * @brief 状态处理函数：RX_START。
 *
 * @param eoe EoE处理器
 * @details 该函数在接收状态机的"RX_START"状态下执行。
 * 它通过将一个用于检查从站邮箱中是否有新的EoE数据报文的数据报文加入队列来启动新的接收序列。
 * 如果从站未关联或存在错误标志，或者主站设备的链路状态未连接，则将接收和发送空闲标志设置为1并返回。
 * 如果已经存在进行中的邮箱读取请求，则跳过邮箱读取检查。
 * 否则，准备一个用于检查邮箱的数据报文，将其加入队列，并将状态设置为"RX_CHECK"。
 */
void ec_eoe_state_rx_start(ec_eoe_t *eoe /**< EoE处理器 */)
{
    if (!eoe->slave || eoe->slave->error_flag ||
        !eoe->slave->master->devices[EC_DEVICE_MAIN].link_state)
    {
        eoe->rx_idle = 1;
        eoe->tx_idle = 1;
        return;
    }

    // 如果已经存在进行中的邮箱读取请求，则跳过邮箱读取检查
    if (ec_read_mbox_locked(eoe->slave))
    {
        eoe->state = ec_eoe_state_rx_fetch_data;
    }
    else
    {
        eoe->have_mbox_lock = 1;
        ec_slave_mbox_prepare_check(eoe->slave, &eoe->datagram);
        eoe->queue_datagram = 1;
        eoe->state = ec_eoe_state_rx_check;
    }
}

/*****************************************************************************/

/**
 * @brief 状态处理函数：RX_CHECK。
 *
 * @param eoe EoE处理器
 * @details 该函数在接收状态机的"RX_CHECK"状态下执行。它处理在"RX_START"状态中发送的检查数据报文，并在有新数据可用时发起接收数据报文。
 */
void ec_eoe_state_rx_check(ec_eoe_t *eoe /**< EoE处理器 */)
{
    if (eoe->datagram.state != EC_DATAGRAM_RECEIVED)
    {
#if EOE_DEBUG_LEVEL >= 1
        EC_SLAVE_WARN(eoe->slave, "无法接收来自%s的邮箱检查数据报文。\n",
                      eoe->dev->name);
        eoe->stats.rx_errors++;
#endif
        eoe->state = ec_eoe_state_tx_start;
        eoe->have_mbox_lock = 0;
        ec_read_mbox_lock_clear(eoe->slave);
        return;
    }

    if (!ec_slave_mbox_check(&eoe->datagram))
    {
        eoe->rx_idle = 1;
        eoe->have_mbox_lock = 0;
        ec_read_mbox_lock_clear(eoe->slave);
        // 检查数据是否已经被其他读取请求接收
        if (eoe->slave->mbox_eoe_frag_data.payload_size > 0)
        {
            eoe->state = ec_eoe_state_rx_fetch_data;
            eoe->state(eoe);
        }
        else
        {
            eoe->state = ec_eoe_state_tx_start;
        }
        return;
    }

    eoe->rx_idle = 0;
    ec_slave_mbox_prepare_fetch(eoe->slave, &eoe->datagram);
    eoe->queue_datagram = 1;
    eoe->state = ec_eoe_state_rx_fetch;
}

/*****************************************************************************/

/**
 * @brief 状态处理函数：RX_FETCH。
 *
 * @param eoe EoE处理器
 * @details 该函数在接收状态机的"RX_FETCH"状态下执行。它检查是否接收到了RX_CHECK请求的数据，并处理EoE数据报文。
 */
void ec_eoe_state_rx_fetch(ec_eoe_t *eoe /**< EoE处理器 */)
{
    if (eoe->datagram.state != EC_DATAGRAM_RECEIVED)
    {
        eoe->stats.rx_errors++;
#if EOE_DEBUG_LEVEL >= 1
        EC_SLAVE_WARN(eoe->slave, "无法接收来自%s的邮箱获取数据报文。\n",
                      eoe->dev->name);
#endif
        eoe->state = ec_eoe_state_tx_start;
        eoe->have_mbox_lock = 0;
        ec_read_mbox_lock_clear(eoe->slave);
        return;
    }
    eoe->have_mbox_lock = 0;
    ec_read_mbox_lock_clear(eoe->slave);
    eoe->state = ec_eoe_state_rx_fetch_data;
    eoe->state(eoe);
}

/*****************************************************************************/

/**
 * @brief 状态处理函数：RX_FETCH DATA。
 *
 * @param eoe EoE处理器
 * @details 该函数在接收状态机的"RX_FETCH DATA"状态下执行。它处理EoE数据。
 */
void ec_eoe_state_rx_fetch_data(ec_eoe_t *eoe /**< EoE处理器 */)
{
    size_t rec_size, data_size;
    uint8_t *data, eoe_type, last_fragment, time_appended, mbox_prot;
    uint8_t fragment_offset, fragment_number;
#if EOE_DEBUG_LEVEL >= 2
    uint8_t frame_number;
#endif
    off_t offset;
#if EOE_DEBUG_LEVEL >= 3
    unsigned int i;
#endif

    if (eoe->slave->mbox_eoe_frag_data.payload_size > 0)
    {
        eoe->slave->mbox_eoe_frag_data.payload_size = 0;
    }
    else
    {
        // 如果所需数据不可用，则发起新的邮箱读取检查
        if (!ec_read_mbox_locked(eoe->slave))
        {
            eoe->have_mbox_lock = 1;
            ec_slave_mbox_prepare_check(eoe->slave, &eoe->datagram);
            eoe->queue_datagram = 1;
            eoe->state = ec_eoe_state_rx_check;
        }
        return;
    }

    data = ec_slave_mbox_fetch(eoe->slave, &eoe->slave->mbox_eoe_frag_data,
                               &mbox_prot, &rec_size);
    if (IS_ERR(data))
    {
        eoe->stats.rx_errors++;
#if EOE_DEBUG_LEVEL >= 1
        EC_SLAVE_WARN(eoe->slave, "无效的邮箱响应数据报文：%s。\n",
                      eoe->dev->name);
#endif
        eoe->state = ec_eoe_state_tx_start;
        return;
    }

    if (mbox_prot != EC_MBOX_TYPE_EOE)
    { // FIXME 需要邮箱处理器
        eoe->stats.rx_errors++;
#if EOE_DEBUG_LEVEL >= 1
        EC_SLAVE_WARN(eoe->slave, "其他邮箱协议响应数据报文：%s。\n",
                      eoe->dev->name);
#endif
        eoe->state = ec_eoe_state_tx_start;
        return;
    }

    eoe_type = EC_READ_U8(data) & 0x0F;

    if (eoe_type != EC_EOE_TYPE_FRAME_FRAG)
    {
        EC_SLAVE_ERR(eoe->slave, "%s：EoE接口处理器接收到其他的EoE类型响应数据报文（类型：%x）。丢弃。\n",
                     eoe->dev->name, eoe_type);
        eoe->stats.rx_dropped++;
        eoe->state = ec_eoe_state_tx_start;
        return;
    }

    // 接收到EoE Fragment请求

    last_fragment = (EC_READ_U16(data) >> 8) & 0x0001;
    time_appended = (EC_READ_U16(data) >> 9) & 0x0001;
    fragment_number = EC_READ_U16(data + 2) & 0x003F;
    fragment_offset = (EC_READ_U16(data + 2) >> 6) & 0x003F;
#if EOE_DEBUG_LEVEL >= 2
    frame_number = (EC_READ_U16(data + 2) >> 12) & 0x000F;
#endif

#if EOE_DEBUG_LEVEL >= 2
    EC_SLAVE_DBG(eoe->slave, 0, "EoE %s 接收到分片 %u%s，偏移 %u，帧 %u%s，%zu字节\n",
                 eoe->dev->name, fragment_number,
                 last_fragment ? "" : "+", fragment_offset, frame_number,
                 time_appended ? "，带时间戳" : "",
                 time_appended ? rec_size - 8 : rec_size - 4);
#endif

#if EOE_DEBUG_LEVEL >= 3
    EC_SLAVE_DBG(eoe->slave, 0, "");
    for (i = 0; i < rec_size - 4; i++)
    {
        printk(KERN_CONT "%02X ", data[i + 4]);
        if ((i + 1) % 16 == 0)
        {
            printk(KERN_CONT "\n");
            EC_SLAVE_DBG(eoe->slave, 0, "");
        }
    }
    printk(KERN_CONT "\n");
#endif

    data_size = time_appended ? rec_size - 8 : rec_size - 4;

    if (!fragment_number)
    {
        if (eoe->rx_skb)
        {
            EC_SLAVE_WARN(eoe->slave, "EoE RX 释放旧的套接字缓冲区。\n");
            dev_kfree_skb(eoe->rx_skb);
        }

        // 新的套接字缓冲区
        if (!(eoe->rx_skb = dev_alloc_skb(fragment_offset * 32)))
        {
            if (printk_ratelimit())
                EC_SLAVE_WARN(eoe->slave, "EoE RX 内存不足，丢弃帧。\n");
            eoe->stats.rx_dropped++;
            eoe->state = ec_eoe_state_tx_start;
            return;
        }

        eoe->rx_skb_offset = 0;
        eoe->rx_skb_size = fragment_offset * 32;
        eoe->rx_expected_fragment = 0;
    }
    else
    {
        if (!eoe->rx_skb)
        {
            eoe->stats.rx_dropped++;
            eoe->state = ec_eoe_state_tx_start;
            return;
        }

        offset = fragment_offset * 32;
        if (offset != eoe->rx_skb_offset ||
            offset + data_size > eoe->rx_skb_size ||
            fragment_number != eoe->rx_expected_fragment)
        {
            dev_kfree_skb(eoe->rx_skb);
            eoe->rx_skb = NULL;
            eoe->stats.rx_errors++;
#if EOE_DEBUG_LEVEL >= 1
            EC_SLAVE_WARN(eoe->slave, "在%s处发生分片错误。\n",
                          eoe->dev->name);
#endif
            eoe->state = ec_eoe_state_tx_start;
            return;
        }
    }

    // 将分片复制到套接字缓冲区
    memcpy(skb_put(eoe->rx_skb, data_size), data + 4, data_size);
    eoe->rx_skb_offset += data_size;

    if (last_fragment)
    {
        // 更新统计信息
        eoe->stats.rx_packets++;
        eoe->stats.rx_bytes += eoe->rx_skb->len;
        eoe->rx_counter += eoe->rx_skb->len;

#if EOE_DEBUG_LEVEL >= 2
        EC_SLAVE_DBG(eoe->slave, 0, "EoE %s 接收到完整帧，长度为 %u 字节。\n",
                     eoe->dev->name, eoe->rx_skb->len);
#endif

        // 将套接字缓冲区传递给网络堆栈
        eoe->rx_skb->dev = eoe->dev;
        eoe->rx_skb->protocol = eth_type_trans(eoe->rx_skb, eoe->dev);
        eoe->rx_skb->ip_summed = CHECKSUM_UNNECESSARY;
        if (netif_rx_ni(eoe->rx_skb))
        {
            EC_SLAVE_WARN(eoe->slave, "EoE RX netif_rx 失败。\n");
        }
        eoe->rx_skb = NULL;

        eoe->state = ec_eoe_state_tx_start;
    }
    else
    {
        eoe->rx_expected_fragment++;
#if EOE_DEBUG_LEVEL >= 2
        EC_SLAVE_DBG(eoe->slave, 0, "EoE %s 接收到分片 %u\n",
                     eoe->dev->name, eoe->rx_expected_fragment);
#endif
        eoe->state = ec_eoe_state_rx_start;
    }
}

/*****************************************************************************/

/**
 * @brief 启动新的传输序列。如果没有可用的数据，则启动新的接收序列。
 * 
 * @param eoe EoE处理程序
 * @return 无
 * @details 此函数用于启动新的传输序列。如果没有可用的数据，则会启动新的接收序列。
 * 如果从设备未连接、从设备存在错误标志或主设备的链路状态未连接，将设置接收和传输空闲标志，并直接返回。
 * 如果传输队列为空，将检查是否需要重新启动队列，并设置传输空闲标志。如果有可用的数据帧，则从环形缓冲区中获取帧并将其移出。
 * 如果需要重新启动队列，则唤醒网络接口队列。设置传输空闲标志为非空闲状态。更新帧号、片段号和偏移量。调用ec_eoe_send函数发送数据帧。
 * 如果发送出错，释放数据帧并更新统计信息，并设置状态为启动新的接收序列。
 * 如果需要唤醒队列，则输出调试信息。设置尝试次数和状态为已发送状态。
 */
void ec_eoe_state_tx_start(ec_eoe_t *eoe /**< EoE处理程序 */)
{
#if EOE_DEBUG_LEVEL >= 2
    unsigned int wakeup = 0;
#endif

    if (!eoe->slave || eoe->slave->error_flag ||
        !eoe->slave->master->devices[EC_DEVICE_MAIN].link_state)
    {
        eoe->rx_idle = 1;
        eoe->tx_idle = 1;
        return;
    }

    if (eoe->tx_next_to_use == eoe->tx_next_to_clean)
    {
        // check if the queue needs to be restarted
        if (!eoe->tx_queue_active)
        {
            eoe->tx_queue_active = 1;
            netif_wake_queue(eoe->dev);
        }

        eoe->tx_idle = 1;
        // no data available.
        // start a new receive immediately.
        ec_eoe_state_rx_start(eoe);
        return;
    }

    // get the frame and take it out of the ring
    eoe->tx_skb = eoe->tx_ring[eoe->tx_next_to_clean];
    eoe->tx_ring[eoe->tx_next_to_clean] = NULL;
    if (unlikely(++eoe->tx_next_to_clean == eoe->tx_ring_count))
    {
        eoe->tx_next_to_clean = 0;
    }

    // restart queue?
    if (!eoe->tx_queue_active &&
        (ec_eoe_tx_queued_frames(eoe) <= eoe->tx_ring_count / 2))
    {
        eoe->tx_queue_active = 1;
        netif_wake_queue(eoe->dev);
#if EOE_DEBUG_LEVEL >= 2
        wakeup = 1;
#endif
    }

    eoe->tx_idle = 0;

    eoe->tx_frame_number++;
    eoe->tx_frame_number %= 16;
    eoe->tx_fragment_number = 0;
    eoe->tx_offset = 0;

    if (ec_eoe_send(eoe))
    {
        dev_kfree_skb(eoe->tx_skb);
        eoe->tx_skb = NULL;
        eoe->stats.tx_errors++;
        eoe->state = ec_eoe_state_rx_start;
#if EOE_DEBUG_LEVEL >= 1
        EC_SLAVE_WARN(eoe->slave, "发送错误：%s。\n", eoe->dev->name);
#endif
        return;
    }

#if EOE_DEBUG_LEVEL >= 2
    if (wakeup)
    {
        EC_SLAVE_DBG(eoe->slave, 0, "EoE %s 唤醒传输队列...\n",
                     eoe->dev->name);
    }
#endif

    eoe->tries = EC_EOE_TRIES;
    eoe->state = ec_eoe_state_tx_sent;
}

/*****************************************************************************/

/** 状态: TX SENT.
 *
 * 检查前一个传输数据报是否成功，并在必要时发送下一个分片。
 *
 * @param eoe EoE处理程序
 *
 * @return 无
 *
 * @details 此函数检查前一个传输的数据报的状态。如果数据报的状态不是EC_DATAGRAM_RECEIVED，
 * 则检查尝试次数。如果尝试次数不为零，则减少尝试次数并将queue_datagram标志设置为1。
 * 如果尝试次数为零，则增加tx_errors统计计数并将状态设置为ec_eoe_state_rx_start。
 * 
 * 如果数据报的working_counter不等于1，则检查尝试次数。如果尝试次数不为零，则减少尝试次数并将queue_datagram标志设置为1。
 * 如果尝试次数为零，则增加tx_errors统计计数，并根据EOE_DEBUG_LEVEL的设置，记录日志信息，并将状态设置为ec_eoe_state_rx_start。
 * 
 * 如果已完全发送帧，则增加tx_packets统计计数、tx_bytes统计计数和tx_counter统计计数，并释放tx_skb缓冲区，将其设置为NULL，并将状态设置为ec_eoe_state_rx_start。
 * 
 * 否则，发送下一个分片。如果发送失败，则释放tx_skb缓冲区，将其设置为NULL，并根据EOE_DEBUG_LEVEL的设置，记录日志信息，并将状态设置为ec_eoe_state_rx_start。
 */
void ec_eoe_state_tx_sent(ec_eoe_t *eoe /**< EoE处理程序 */)
{
    if (eoe->datagram.state != EC_DATAGRAM_RECEIVED)
    {
        if (eoe->tries)
        {
            eoe->tries--; // 再次尝试
            eoe->queue_datagram = 1;
        }
        else
        {
#if EOE_DEBUG_LEVEL >= 1
            /* 仅记录每1000次 */
            if (eoe->stats.tx_errors++ % 1000 == 0)
            {
                EC_SLAVE_WARN(eoe->slave, "失败：在 %s 的 %u 次尝试后未能接收发送的数据报。\n",
                              eoe->dev->name, EC_EOE_TRIES);
            }
#else
            eoe->stats.tx_errors++;
#endif
            eoe->state = ec_eoe_state_rx_start;
        }
        return;
    }

    if (eoe->datagram.working_counter != 1)
    {
        if (eoe->tries)
        {
            eoe->tries--; // 再次尝试
            eoe->queue_datagram = 1;
        }
        else
        {
            eoe->stats.tx_errors++;
#if EOE_DEBUG_LEVEL >= 1
            EC_SLAVE_WARN(eoe->slave, "未发送响应：在 %s 的 %u 次尝试后。\n",
                          eoe->dev->name, EC_EOE_TRIES);
#endif
            eoe->state = ec_eoe_state_rx_start;
        }
        return;
    }

    // 帧完全发送
    if (eoe->tx_offset >= eoe->tx_skb->len)
    {
        eoe->stats.tx_packets++;
        eoe->stats.tx_bytes += eoe->tx_skb->len;
        eoe->tx_counter += eoe->tx_skb->len;
        dev_kfree_skb(eoe->tx_skb);
        eoe->tx_skb = NULL;
        eoe->state = ec_eoe_state_rx_start;
    }
    else
    { // 发送下一个分片
        if (ec_eoe_send(eoe))
        {
            dev_kfree_skb(eoe->tx_skb);
            eoe->tx_skb = NULL;
            eoe->stats.tx_errors++;
#if EOE_DEBUG_LEVEL >= 1
            EC_SLAVE_WARN(eoe->slave, "在 %s 发送错误。\n", eoe->dev->name);
#endif
            eoe->state = ec_eoe_state_rx_start;
        }
    }
}

/******************************************************************************
 *  NET_DEVICE函数
 *****************************************************************************/

/** 打开虚拟网络设备。
 *
 * @param dev EoE net_device
 *
 * @return 始终返回零（成功）。
 *
 * @details 此函数打开虚拟网络设备。它将carrier设置为关闭状态，直到我们知道链路状态。
 * 然后，刷新EoE处理程序，并设置opened、rx_idle、tx_idle和tx_queue_active标志。
 * 启动网络设备队列，并根据EOE_DEBUG_LEVEL的设置，记录日志信息。
 * 
 * 如果EoE处理程序关联了从站，则更新carrier链路状态，并根据EOE_DEBUG_LEVEL的设置，记录日志信息。
 */
int ec_eoedev_open(struct net_device *dev /**< EoE net_device */)
{
    ec_eoe_t *eoe = *((ec_eoe_t **)netdev_priv(dev));

    // 设置carrier为关闭状态，直到我们知道链路状态
    EC_MASTER_DBG(eoe->master, 1, "%s: carrier off.\n", dev->name);
    netif_carrier_off(dev);

    ec_eoe_flush(eoe);
    eoe->opened = 1;
    eoe->rx_idle = 0;
    eoe->tx_idle = 0;
    eoe->tx_queue_active = 1;
    netif_start_queue(dev);
#if EOE_DEBUG_LEVEL >= 2
    EC_MASTER_DBG(eoe->master, 0, "%s 已打开。\n", dev->name);
#endif

    // 更新carrier链路状态
    if (eoe->slave)
    {
        EC_MASTER_DBG(eoe->master, 1, "%s: carrier on.\n", dev->name);
        netif_carrier_on(dev);
    }

    return 0;
}

/*****************************************************************************/

/** 停止虚拟网络设备。
 *
 * @param dev EoE net_device
 *
 * @return 始终返回零（成功）。
 *
 * @details 此函数停止虚拟网络设备。它将carrier设置为关闭状态，停止网络设备队列，并根据EOE_DEBUG_LEVEL的设置，记录日志信息。
 * 同时，将tx_queue_active、rx_idle和tx_idle标志设置为相应的值，并刷新EoE处理程序。
 */
int ec_eoedev_stop(struct net_device *dev /**< EoE net_device */)
{
    ec_eoe_t *eoe = *((ec_eoe_t **)netdev_priv(dev));

    EC_MASTER_DBG(eoe->master, 1, "%s: carrier off.\n", dev->name);
    netif_carrier_off(dev);
    netif_stop_queue(dev);
    eoe->tx_queue_active = 0;
    eoe->rx_idle = 1;
    eoe->tx_idle = 1;
    eoe->opened = 0;
    ec_eoe_flush(eoe);
#if EOE_DEBUG_LEVEL >= 2
    EC_MASTER_DBG(eoe->master, 0, "%s 已停止。\n", dev->name);
#endif
    return 0;
}

/*****************************************************************************/

/** 通过虚拟网络设备传输数据。
 *
 * @param skb 要传输的套接字缓冲区
 * @param dev EoE net_device
 *
 * @return 成功返回零，失败返回非零值。
 *
 * @details 此函数通过虚拟网络设备传输数据。如果EoE处理程序未关联从站，则释放skb并增加tx_dropped统计计数，
 * 然后返回NETDEV_TX_OK。
 * 
 * 如果skb的长度超过从站配置的tx_mailbox_size减去10，则释放skb并增加tx_dropped统计计数，然后返回0。
 * 
 * 将skb设置在环形缓冲区中，并递增tx_next_to_use索引。如果tx_next_to_use等于tx_ring_count，则将其设置为0。
 * 如果未使用的帧数为0，则停止队列，并将tx_queue_active标志设置为0。
 * 
 * 根据EOE_DEBUG_LEVEL的设置，记录日志信息。
 */
int ec_eoedev_tx(struct sk_buff *skb,   /**< transmit socket buffer */
                 struct net_device *dev /**< EoE net_device */
)
{
    ec_eoe_t *eoe = *((ec_eoe_t **)netdev_priv(dev));

    if (!eoe->slave)
    {
        if (skb)
        {
            dev_kfree_skb(skb);
            eoe->stats.tx_dropped++;
        }

        return NETDEV_TX_OK;
    }

#if 0
    if (skb->len > eoe->slave->configured_tx_mailbox_size - 10) {
        EC_SLAVE_WARN(eoe->slave, "EoE TX frame (%u octets)"
                " exceeds MTU. dropping.\n", skb->len);
        dev_kfree_skb(skb);
        eoe->stats.tx_dropped++;
        return 0;
    }
#endif

    // 将skb设置在环形缓冲区中
    eoe->tx_ring[eoe->tx_next_to_use] = skb;

    // 递增索引
    if (unlikely(++eoe->tx_next_to_use == eoe->tx_ring_count))
    {
        eoe->tx_next_to_use = 0;
    }

    // 停止队列？
    if (eoe_tx_unused_frames(eoe) == 0)
    {
        netif_stop_queue(dev);
        eoe->tx_queue_active = 0;
    }

#if EOE_DEBUG_LEVEL >= 2
    EC_SLAVE_DBG(eoe->slave, 0, "EoE %s TX queued frame"
                                " with %u octets (%u frames queued).\n",
                 eoe->dev->name, skb->len, ec_eoe_tx_queued_frames(eoe));
    if (!eoe->tx_queue_active)
    {
        EC_SLAVE_WARN(eoe->slave, "EoE TX queue is now full.\n");
    }
#endif

    return 0;
}

/*****************************************************************************/

/** 获取虚拟网络设备的统计信息。
 *
 * @param dev EoE net_device
 *
 * @return 统计信息。
 *
 * @details 此函数返回虚拟网络设备的统计信息。
 */
struct net_device_stats *ec_eoedev_stats(
    struct net_device *dev /**< EoE net_device */
)
{
    ec_eoe_t *eoe = *((ec_eoe_t **)netdev_priv(dev));
    return &eoe->stats;
}

/*****************************************************************************/

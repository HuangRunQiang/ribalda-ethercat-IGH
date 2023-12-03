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
   EtherCAT device methods.
*/

/*****************************************************************************/

#include <linux/module.h>
#include <linux/skbuff.h>
#include <linux/if_ether.h>
#include <linux/netdevice.h>
#include <linux/jiffies.h>

#include "device.h"
#include "master.h"

#ifdef EC_DEBUG_RING
#define timersub(a, b, result)                           \
    do                                                   \
    {                                                    \
        (result)->tv_sec = (a)->tv_sec - (b)->tv_sec;    \
        (result)->tv_usec = (a)->tv_usec - (b)->tv_usec; \
        if ((result)->tv_usec < 0)                       \
        {                                                \
            --(result)->tv_sec;                          \
            (result)->tv_usec += 1000000;                \
        }                                                \
    } while (0)
#endif

/*****************************************************************************/

/**
 * @brief 初始化EtherCAT设备。
 *
 * @param device EtherCAT设备
 * @param master 拥有该设备的主站
 * 
 * @return 如果成功则为0，否则为 \a -ENODEV。
 * 
 * @details 初始化EtherCAT设备结构，并分配和设置相关资源。
 */
int ec_device_init(
    ec_device_t *device, /**< EtherCAT设备 */
    ec_master_t *master  /**< 拥有该设备的主站 */
)
{
    int ret;
    unsigned int i;
    struct ethhdr *eth;
#ifdef EC_DEBUG_IF
    char ifname[10];
    char mb = 'x';
#endif

    device->master = master;
    device->dev = NULL;
    device->poll = NULL;
    device->module = NULL;
    device->open = 0;
    device->link_state = 0;
    for (i = 0; i < EC_TX_RING_SIZE; i++)
    {
        device->tx_skb[i] = NULL;
    }
    device->tx_ring_index = 0;
#ifdef EC_HAVE_CYCLES
    device->cycles_poll = 0;
#endif
#ifdef EC_DEBUG_RING
    device->timeval_poll.tv_sec = 0;
    device->timeval_poll.tv_usec = 0;
#endif
    device->jiffies_poll = 0;

    ec_device_clear_stats(device);

#ifdef EC_DEBUG_RING
    for (i = 0; i < EC_DEBUG_RING_SIZE; i++)
    {
        ec_debug_frame_t *df = &device->debug_frames[i];
        df->dir = TX;
        df->t.tv_sec = 0;
        df->t.tv_usec = 0;
        memset(df->data, 0, EC_MAX_DATA_SIZE);
        df->data_size = 0;
    }
#endif
#ifdef EC_DEBUG_RING
    device->debug_frame_index = 0;
    device->debug_frame_count = 0;
#endif

#ifdef EC_DEBUG_IF
    if (device == &master->devices[EC_DEVICE_MAIN])
    {
        mb = 'm';
    }
    else
    {
        mb = 'b';
    }

    sprintf(ifname, "ecdbg%c%u", mb, master->index);

    ret = ec_debug_init(&device->dbg, device, ifname);
    if (ret < 0)
    {
        EC_MASTER_ERR(master, "初始化调试设备失败！\n");
        goto out_return;
    }
#endif

    for (i = 0; i < EC_TX_RING_SIZE; i++)
    {
        if (!(device->tx_skb[i] = dev_alloc_skb(ETH_FRAME_LEN)))
        {
            EC_MASTER_ERR(master, "分配设备套接字缓冲区失败！\n");
            ret = -ENOMEM;
            goto out_tx_ring;
        }

        // 添加以太网II头部
        skb_reserve(device->tx_skb[i], ETH_HLEN);
        eth = (struct ethhdr *)skb_push(device->tx_skb[i], ETH_HLEN);
        eth->h_proto = htons(0x88A4);
        memset(eth->h_dest, 0xFF, ETH_ALEN);
    }

    return 0;

out_tx_ring:
    for (i = 0; i < EC_TX_RING_SIZE; i++)
    {
        if (device->tx_skb[i])
        {
            dev_kfree_skb(device->tx_skb[i]);
        }
    }
#ifdef EC_DEBUG_IF
    ec_debug_clear(&device->dbg);
out_return:
#endif
    return ret;
}

/*****************************************************************************/

/**
 * @brief 析构函数。
 *
 * @param device EtherCAT设备
 * 
 * @details 清理EtherCAT设备结构，并释放相关资源。
 */
void ec_device_clear(
    ec_device_t *device /**< EtherCAT设备 */
)
{
    unsigned int i;

    if (device->open)
    {
        ec_device_close(device);
    }
    for (i = 0; i < EC_TX_RING_SIZE; i++)
        dev_kfree_skb(device->tx_skb[i]);
#ifdef EC_DEBUG_IF
    ec_debug_clear(&device->dbg);
#endif
}

/*****************************************************************************/

/**
 * @brief 关联net_device。
 *
 * @param device EtherCAT设备
 * @param net_dev net_device结构
 * @param poll 设备的轮询函数指针
 * @param module 设备的模块
 * 
 * @details 将EtherCAT设备与net_device关联，并设置相关参数。
 */
void ec_device_attach(
    ec_device_t *device,        /**< EtherCAT设备 */
    struct net_device *net_dev, /**< net_device结构 */
    ec_pollfunc_t poll,         /**< 设备的轮询函数指针 */
    struct module *module       /**< 设备的模块 */
)
{
    unsigned int i;
    struct ethhdr *eth;

    ec_device_detach(device); // 重置字段

    device->dev = net_dev;
    device->poll = poll;
    device->module = module;

    for (i = 0; i < EC_TX_RING_SIZE; i++)
    {
        device->tx_skb[i]->dev = net_dev;
        eth = (struct ethhdr *)(device->tx_skb[i]->data);
        memcpy(eth->h_source, net_dev->dev_addr, ETH_ALEN);
    }

#ifdef EC_DEBUG_IF
    ec_debug_register(&device->dbg, net_dev);
#endif
}

/*****************************************************************************/

/**
 * @brief 与net_device断开连接。
 *
 * @param device EtherCAT设备
 * 
 * @details 将EtherCAT设备与net_device断开连接，并重置相关参数。
 */
void ec_device_detach(
    ec_device_t *device /**< EtherCAT设备 */
)
{
    unsigned int i;

#ifdef EC_DEBUG_IF
    ec_debug_unregister(&device->dbg);
#endif

    device->dev = NULL;
    device->poll = NULL;
    device->module = NULL;
    device->open = 0;
    device->link_state = 0; // 下线

    ec_device_clear_stats(device);

    for (i = 0; i < EC_TX_RING_SIZE; i++)
    {
        device->tx_skb[i]->dev = NULL;
    }
}

/*****************************************************************************/

/**
 * @brief 打开EtherCAT设备。
 *
 * @param device EtherCAT设备
 * 
 * @return 如果成功则为0，否则为 \a -ENODEV。
 * 
 * @details 打开EtherCAT设备，启动设备的操作。
 */
int ec_device_open(
    ec_device_t *device /**< EtherCAT设备 */
)
{
    int ret;

    if (!device->dev)
    {
        EC_MASTER_ERR(device->master, "没有要打开的net_device！\n");
        return -ENODEV;
    }

    if (device->open)
    {
        EC_MASTER_WARN(device->master, "设备已经打开！\n");
        return 0;
    }

    device->link_state = 0;

    ec_device_clear_stats(device);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 29)
    ret = device->dev->netdev_ops->ndo_open(device->dev);
#else
    ret = device->dev->open(device->dev);
#endif
    if (!ret)
        device->open = 1;

    return ret;
}

/*****************************************************************************/

/**
 * @brief 关闭EtherCAT设备。
 *
 * @param device EtherCAT设备
 * 
 * @return 如果成功则为0，否则为 \a -ENODEV。
 * 
 * @details 关闭EtherCAT设备，停止设备的操作。
 */
int ec_device_close(
    ec_device_t *device /**< EtherCAT设备 */
)
{
    int ret;

    if (!device->dev)
    {
        EC_MASTER_ERR(device->master, "没有要关闭的设备！\n");
        return -ENODEV;
    }

    if (!device->open)
    {
        EC_MASTER_WARN(device->master, "设备已经关闭！\n");
        return 0;
    }

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 29)
    ret = device->dev->netdev_ops->ndo_stop(device->dev);
#else
    ret = device->dev->stop(device->dev);
#endif
    if (!ret)
        device->open = 0;

    return ret;
}

/*****************************************************************************/

/**
 * @brief 记录一个数据包到主站的pcap缓冲区（如果有空间）。
 *
 * @param device EtherCAT设备
 * @param data 数据包的数据
 * @param size 数据包的大小
 * 
 * @details 检查是否有足够的空间将数据帧复制到pcap内存中，并记录到pcap缓冲区中。
 */
static void pcap_record(
    ec_device_t *device, /**< EtherCAT设备 */
    const void *data,    /**< 数据包数据 */
    size_t size          /**< 数据包大小 */
)
{
    // 检查是否有足够的空间将帧复制到pcap内存中
    if (unlikely(device->master->pcap_data))
    {
        // 获取当前数据指针
        void *curr_data = device->master->pcap_curr_data;
        long available = pcap_size - (curr_data - device->master->pcap_data);
        long reqd = size + sizeof(pcaprec_hdr_t);
        if (unlikely(reqd <= available))
        {
            pcaprec_hdr_t *pcaphdr;
            struct timeval t;

            // 更新当前数据指针
            device->master->pcap_curr_data = curr_data + reqd;

            // 填写pcap帧头信息
            pcaphdr = curr_data;
#ifdef EC_RTDM
            jiffies_to_timeval(device->jiffies_poll, &t);
#else
            t = device->timeval_poll;
#endif
            pcaphdr->ts_sec = t.tv_sec;
            pcaphdr->ts_usec = t.tv_usec;
            pcaphdr->incl_len = size;
            pcaphdr->orig_len = size;
            curr_data += sizeof(pcaprec_hdr_t);

            // 复制数据帧
            memcpy(curr_data, data, size);
        }
    }
}

/*****************************************************************************/

/**
 * @brief 返回指向设备传输内存的指针。
 *
 * @param device EtherCAT设备。
 * @return 指向TX套接字缓冲区的指针。
 *
 * @details 此函数返回指向设备的传输内存的指针。
 * 它通过循环遍历套接字缓冲区，以防止多个帧在发送过程中发生竞争条件，
 * 如果DMA在它们之间没有被调度。
 */
uint8_t *ec_device_tx_data(
    ec_device_t *device /**< EtherCAT设备 */
)
{
    /* 循环遍历套接字缓冲区，以防止多个帧在发送过程中发生竞争条件 */
    device->tx_ring_index++;
    device->tx_ring_index %= EC_TX_RING_SIZE;
    return device->tx_skb[device->tx_ring_index]->data + ETH_HLEN;
}

/*****************************************************************************/

/**
 * @brief 发送传输套接字缓冲区的内容。
 *
 * 将套接字缓冲区的内容截断为（现在已知的）大小，并调用分配的net_device的start_xmit()函数。
 *
 * @param device EtherCAT设备。
 * @param size 要发送的字节数。
 *
 * @details 此函数发送传输套接字缓冲区的内容。
 * 它将套接字缓冲区的长度设置为ETH_HLEN + size。
 * 如果设备的debug_level大于1，则打印发送的帧内容。
 * 然后开始发送。
 *
 * @note 如果成功发送帧，则更新设备的统计信息。
 */
void ec_device_send(
    ec_device_t *device, /**< EtherCAT设备 */
    size_t size          /**< 要发送的字节数 */
)
{
    struct sk_buff *skb = device->tx_skb[device->tx_ring_index];

    // 设置数据的正确长度
    skb->len = ETH_HLEN + size;

    if (unlikely(device->master->debug_level > 1))
    {
        EC_MASTER_DBG(device->master, 2, "发送帧:\n");
        ec_print_data(skb->data, ETH_HLEN + size);
    }

    // 开始发送
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 29)
    if (device->dev->netdev_ops->ndo_start_xmit(skb, device->dev) ==
        NETDEV_TX_OK)
#else
    if (device->dev->hard_start_xmit(skb, device->dev) == NETDEV_TX_OK)
#endif
    {
        device->tx_count++;
        device->master->device_stats.tx_count++;
        device->tx_bytes += ETH_HLEN + size;
        device->master->device_stats.tx_bytes += ETH_HLEN + size;
        pcap_record(device, skb->data, ETH_HLEN + size);
#ifdef EC_DEBUG_IF
        ec_debug_send(&device->dbg, skb->data, ETH_HLEN + size);
#endif
#ifdef EC_DEBUG_RING
        ec_device_debug_ring_append(
            device, TX, skb->data + ETH_HLEN, size);
#endif
    }
    else
    {
        device->tx_errors++;
    }
}

/*****************************************************************************/

/**
 * @brief 清除帧统计信息。
 *
 * @param device EtherCAT设备。
 *
 * @details 此函数将帧统计信息归零。
 */
void ec_device_clear_stats(
    ec_device_t *device /**< EtherCAT设备 */
)
{
    unsigned int i;

    // 归零帧统计信息
    device->tx_count = 0;
    device->last_tx_count = 0;
    device->rx_count = 0;
    device->last_rx_count = 0;
    device->tx_bytes = 0;
    device->last_tx_bytes = 0;
    device->rx_bytes = 0;
    device->last_rx_bytes = 0;
    device->tx_errors = 0;

    for (i = 0; i < EC_RATE_COUNT; i++)
    {
        device->tx_frame_rates[i] = 0;
        device->rx_frame_rates[i] = 0;
        device->tx_byte_rates[i] = 0;
        device->rx_byte_rates[i] = 0;
    }
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 0, 0)

/**
 * @brief 获取当前时间。
 *
 * @param tv 指向时间结构的指针。
 *
 * @details 此函数获取当前的时间。
 */
static void do_gettimeofday(struct timeval *tv)
{
    struct timespec64 ts;

    ktime_get_ts64(&ts);
    tv->tv_sec = ts.tv_sec;
    tv->tv_usec = ts.tv_nsec / NSEC_PER_USEC;
}

#endif

/*****************************************************************************/

#ifdef EC_DEBUG_RING
/**
 * @brief 将帧数据追加到调试环形缓冲区。
 *
 * @param device EtherCAT 设备
 * @param dir 方向
 * @param data 帧数据
 * @param size 数据大小
 *
 * @details 此函数将帧数据追加到 EtherCAT 设备的调试环形缓冲区中。
 *          它会将数据复制到缓冲区中，并更新相关的索引和计数。
 *
 * @note 调试环形缓冲区的大小由 EC_DEBUG_RING_SIZE 定义。
 */
void ec_device_debug_ring_append(
    ec_device_t *device,      /**< EtherCAT 设备 */
    ec_debug_frame_dir_t dir, /**< 方向 */
    const void *data,         /**< 帧数据 */
    size_t size               /**< 数据大小 */
)
{
    ec_debug_frame_t *df = &device->debug_frames[device->debug_frame_index];

    df->dir = dir;
    if (dir == TX)
    {
        do_gettimeofday(&df->t);
    }
    else
    {
        df->t = device->timeval_poll;
    }
    memcpy(df->data, data, size);
    df->data_size = size;

    device->debug_frame_index++;
    device->debug_frame_index %= EC_DEBUG_RING_SIZE;
    if (unlikely(device->debug_frame_count < EC_DEBUG_RING_SIZE))
        device->debug_frame_count++;
}


/*****************************************************************************/

/** 
 * @brief 输出调试环形缓冲区。
 *
 * @param device EtherCAT 设备
 *
 * @details 此函数用于输出 EtherCAT 设备的调试环形缓冲区。
 *          它会按照时间顺序打印调试帧的信息和数据。
 */
void ec_device_debug_ring_print(
    const ec_device_t *device /**< EtherCAT 设备 */
)
{
    int i;
    unsigned int ring_index;
    const ec_debug_frame_t *df;
    struct timeval t0, diff;

    // 计算环形缓冲区中最新帧的索引以获取其时间
    ring_index = (device->debug_frame_index + EC_DEBUG_RING_SIZE - 1) % EC_DEBUG_RING_SIZE;
    t0 = device->debug_frames[ring_index].t;

    EC_MASTER_DBG(device->master, 1, "调试环形缓冲区 %u:\n", ring_index);

    // 计算环形缓冲区中最旧帧的索引
    ring_index = (device->debug_frame_index + EC_DEBUG_RING_SIZE - device->debug_frame_count) % EC_DEBUG_RING_SIZE;

    for (i = 0; i < device->debug_frame_count; i++)
    {
        df = &device->debug_frames[ring_index];
        timersub(&t0, &df->t, &diff);

        EC_MASTER_DBG(device->master, 1, "帧 %u, dt=%u.%06u 秒, %s:\n",
                      i + 1 - device->debug_frame_count,
                      (unsigned int)diff.tv_sec,
                      (unsigned int)diff.tv_usec,
                      (df->dir == TX) ? "发送" : "接收");
        ec_print_data(df->data, df->data_size);

        ring_index++;
        ring_index %= EC_DEBUG_RING_SIZE;
    }
}

#endif

/*****************************************************************************/

/**
 * @brief 调用分配的 net_device 的 poll 函数。
 *
 * 主站本身不使用中断。因此，必须由主站手动调用 ISR 来处理接收到的数据和网络设备状态的变化。
 *
 * @param device EtherCAT 设备
 *
 * @details
 * 该函数调用分配的 net_device 的 poll 函数。如果启用 EC_HAVE_CYCLES 宏，将记录循环的时间。
 * 然后，记录 jiffies 和时间戳，用于调试目的。最后，调用设备的 poll 函数，处理接收到的数据和状态变化。
 */
void ec_device_poll(
    ec_device_t *device /**< EtherCAT 设备 */
)
{
#ifdef EC_HAVE_CYCLES
    device->cycles_poll = get_cycles();
#endif
    device->jiffies_poll = jiffies;
#ifdef EC_DEBUG_RING
    do_gettimeofday(&device->timeval_poll);
#elif !defined(EC_RTDM)
    if (pcap_size)
        do_gettimeofday(&device->timeval_poll);
#endif
    device->poll(device->dev);
}


/*****************************************************************************/

/**
 * @brief 更新设备统计信息。
 * 
 * 更新设备的传输统计信息，包括帧率和字节率，使用低通滤波器对统计信息进行平滑处理。
 * 
 * @param device EtherCAT设备指针。
 * 
 * @details
 * 该函数用于更新EtherCAT设备的统计信息，包括传输帧率和传输字节率。通过计算帧率和字节率的差值，
 * 然后应用低通滤波器，以平滑处理这些统计信息，提供更稳定的结果。更新后的统计信息存储在设备结构体中，
 * 以备将来使用。
 * 
 * 低通滤波器的公式如下：
 * Y_n = y_(n - 1) + T / tau * (x - y_(n - 1))
 * 其中，T为采样周期，此处取值为1。对于每种统计信息（帧率和字节率），分别应用该滤波器计算平滑值。
 * 
 * @return 无返回值。
 */
void ec_device_update_stats(
    ec_device_t *device /**< EtherCAT设备指针 */
)
{
    unsigned int i;

    s32 tx_frame_rate = (device->tx_count - device->last_tx_count) * 1000;
    s32 rx_frame_rate = (device->rx_count - device->last_rx_count) * 1000;
    s32 tx_byte_rate = (device->tx_bytes - device->last_tx_bytes);
    s32 rx_byte_rate = (device->rx_bytes - device->last_rx_bytes);

    /* 低通滤波器：
     *      Y_n = y_(n - 1) + T / tau * (x - y_(n - 1))   | T = 1
     *   -> Y_n += (x - y_(n - 1)) / tau
     */
    for (i = 0; i < EC_RATE_COUNT; i++)
    {
        s32 n = rate_intervals[i];
        device->tx_frame_rates[i] +=
            (tx_frame_rate - device->tx_frame_rates[i]) / n;
        device->rx_frame_rates[i] +=
            (rx_frame_rate - device->rx_frame_rates[i]) / n;
        device->tx_byte_rates[i] +=
            (tx_byte_rate - device->tx_byte_rates[i]) / n;
        device->rx_byte_rates[i] +=
            (rx_byte_rate - device->rx_byte_rates[i]) / n;
    }

    device->last_tx_count = device->tx_count;
    device->last_rx_count = device->rx_count;
    device->last_tx_bytes = device->tx_bytes;
    device->last_rx_bytes = device->rx_bytes;
}


/******************************************************************************
 *  设备接口
 *****************************************************************************/

/** 从主站中撤销一个EtherCAT设备。
 *
 * 设备从主站断开连接，并释放所有设备资源。
 *
 * \attention 在调用此函数之前，必须调用ecdev_stop()函数，以确保主站不再使用该设备。
 * \ingroup DeviceInterface
 *
 * @param device EtherCAT设备
 * @return 无
 * @details
 *     如果设备是主站的主设备（EC_DEVICE_MAIN），将dev_str设置为"主站"；
 *     如果设备是主站的备份设备（EC_DEVICE_BACKUP），将dev_str设置为"备份"；
 *     否则，记录警告信息并将dev_str设置为"未知设备"。
 *     输出释放设备的信息，包括设备类型和MAC地址。
 */
void ecdev_withdraw(ec_device_t *device /**< EtherCAT设备 */)
{
    ec_master_t *master = device->master;
    char dev_str[20], mac_str[20];

    ec_mac_print(device->dev->dev_addr, mac_str);

    if (device == &master->devices[EC_DEVICE_MAIN])
    {
        sprintf(dev_str, "主站");
    }
    else if (device == &master->devices[EC_DEVICE_BACKUP])
    {
        sprintf(dev_str, "备份");
    }
    else
    {
        EC_MASTER_WARN(master, "%s() 被调用，使用了未知设备 %s!\n",
                       __func__, mac_str);
        sprintf(dev_str, "未知设备");
    }

    EC_MASTER_INFO(master, "释放 %s 设备 %s。\n", dev_str, mac_str);

    ec_lock_down(&master->device_sem);
    ec_device_detach(device);
    ec_lock_up(&master->device_sem);
}


/*****************************************************************************/

/** 打开网络设备并使主站进入IDLE阶段。
 *
 * \return 成功返回0，否则返回 < 0
 * \ingroup DeviceInterface
 *
 * @param device EtherCAT设备
 * @details
 *     打开指定的EtherCAT设备，并检查所有设备是否已打开。如果所有设备都已打开，
 *     则使主站进入IDLE阶段。如果设备打开失败，记录错误信息并返回相应的错误代码。
 *     如果进入IDLE阶段失败，记录主站错误信息并返回相应的错误代码。
 */
int ecdev_open(ec_device_t *device /**< EtherCAT设备 */)
{
    int ret;
    ec_master_t *master = device->master;
    unsigned int all_open = 1, dev_idx;

    ret = ec_device_open(device);
    if (ret)
    {
        EC_MASTER_ERR(master, "打开设备失败！\n");
        return ret;
    }

    for (dev_idx = EC_DEVICE_MAIN;
         dev_idx < ec_master_num_devices(device->master); dev_idx++)
    {
        if (!master->devices[dev_idx].open)
        {
            all_open = 0;
            break;
        }
    }

    if (all_open)
    {
        ret = ec_master_enter_idle_phase(device->master);
        if (ret)
        {
            EC_MASTER_ERR(device->master, "进入IDLE阶段失败！\n");
            return ret;
        }
    }

    return 0;
}


/*****************************************************************************/

/** 使主站离开IDLE阶段并关闭网络设备。
 *
 * \return 成功返回0，否则返回 < 0
 * \ingroup DeviceInterface
 *
 * @param device EtherCAT设备
 * @details
 *     如果主站处于IDLE阶段，使主站离开IDLE阶段。
 *     然后关闭指定的EtherCAT设备。如果关闭设备失败，记录警告信息。
 */
void ecdev_close(ec_device_t *device /**< EtherCAT设备 */)
{
    ec_master_t *master = device->master;

    if (master->phase == EC_IDLE)
    {
        ec_master_leave_idle_phase(master);
    }

    if (ec_device_close(device))
    {
        EC_MASTER_WARN(master, "关闭设备失败！\n");
    }
}

/*****************************************************************************/

/** 接受一个接收到的帧。
 *
 * 将接收到的数据转发给主站。主站将分析帧并将接收到的命令分派到发送实例。
 *
 * 数据必须以以太网头（目标MAC地址）开头。
 *
 * \ingroup DeviceInterface
 *
 * @param device EtherCAT设备
 * @param data 指向接收到的数据的指针
 * @param size 接收到的字节数
 * @details
 *     如果数据为NULL，记录警告信息并返回。
 *     更新设备的接收统计信息，并将接收到的数据写入调试记录。
 *     如果主站的调试级别大于1，记录接收到的帧的信息。
 *     将数据传递给pcap记录和EtherCAT调试发送功能。
 *     将数据传递给主站的接收数据报处理函数。
 */
void ecdev_receive(
    ec_device_t *device, /**< EtherCAT设备 */
    const void *data,    /**< 指向接收到的数据的指针 */
    size_t size          /**< 接收到的字节数 */
)
{
    const void *ec_data = data + ETH_HLEN;
    size_t ec_size = size - ETH_HLEN;

    if (unlikely(!data))
    {
        EC_MASTER_WARN(device->master, "%s() 被调用，使用了NULL数据。\n",
                       __func__);
        return;
    }

    device->rx_count++;
    device->master->device_stats.rx_count++;
    device->rx_bytes += size;
    device->master->device_stats.rx_bytes += size;

    if (unlikely(device->master->debug_level > 1))
    {
        EC_MASTER_DBG(device->master, 2, "接收到的帧信息:\n");
        ec_print_data(data, size);
    }

    pcap_record(device, data, size);
#ifdef EC_DEBUG_IF
    ec_debug_send(&device->dbg, data, size);
#endif
#ifdef EC_DEBUG_RING
    ec_device_debug_ring_append(device, RX, ec_data, ec_size);
#endif

    ec_master_receive_datagrams(device->master, device, ec_data, ec_size);
}

/*****************************************************************************/

/** 设置新的链路状态。
 *
 * 如果设备通知主站链路已断开，主站将不会尝试使用此设备发送帧。
 *
 * \ingroup DeviceInterface
 *
 * @param device EtherCAT设备
 * @param state 新的链路状态
 * @details
 *     如果设备不为NULL，设置设备的链路状态为新的状态。
 *     记录链路状态的改变信息。
 */
void ecdev_set_link(
    ec_device_t *device, /**< EtherCAT设备 */
    uint8_t state        /**< 新的链路状态 */
)
{
    if (unlikely(!device))
    {
        EC_WARN("ecdev_set_link() 被调用，使用了null设备！\n");
        return;
    }

    if (likely(state != device->link_state))
    {
        device->link_state = state;
        EC_MASTER_INFO(device->master,
                       "%s 的链路状态已更改为 %s。\n",
                       device->dev->name, (state ? "UP" : "DOWN"));
    }
}

/*****************************************************************************/

/** 读取链路状态。
 *
 * \ingroup DeviceInterface
 *
 * @param device EtherCAT设备
 * @return 链路状态
 * @details
 *     如果设备不为NULL，返回设备的链路状态。
 *     否则，记录警告信息并返回0。
 */
uint8_t ecdev_get_link(
    const ec_device_t *device /**< EtherCAT设备 */
)
{
    if (unlikely(!device))
    {
        EC_WARN("ecdev_get_link() 被调用，使用了null设备！\n");
        return 0;
    }

    return device->link_state;
}


/*****************************************************************************/

/** \cond */

EXPORT_SYMBOL(ecdev_withdraw);
EXPORT_SYMBOL(ecdev_open);
EXPORT_SYMBOL(ecdev_close);
EXPORT_SYMBOL(ecdev_receive);
EXPORT_SYMBOL(ecdev_get_link);
EXPORT_SYMBOL(ecdev_set_link);

/** \endcond */

/*****************************************************************************/

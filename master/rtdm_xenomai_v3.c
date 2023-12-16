/*****************************************************************************
 *
 *  $Id$
 *
 *  Copyright (C) 2009-2010  Moehwald GmbH B. Benner
 *                     2011  IgH Andreas Stewering-Bone
 *                     2012  Florian Pose <fp@igh-essen.com>
 *
 *  This file is part of the IgH EtherCAT master.
 *
 *  The IgH EtherCAT master is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License as published
 *  by the Free Software Foundation; version 2 of the License.
 *
 *  The IgH EtherCAT master is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 *  Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with the IgH EtherCAT master. If not, see <http://www.gnu.org/licenses/>.
 *
 *  The license mentioned above concerns the source code only. Using the
 *  EtherCAT technology and brand is only permitted in compliance with the
 *  industrial property and similar rights of Beckhoff Automation GmbH.
 *
 ****************************************************************************/

/** \file
 * RTDM interface.
 */

#include <linux/module.h>
#include <linux/vmalloc.h>
#include <rtdm/driver.h>

#include "master.h"
#include "ioctl.h"
#include "rtdm.h"

/**
 * @brief 打开 RTDM 设备。
 * @param fd RTDM 文件描述符。
 * @param oflags 打开标志。
 * @return 成功返回零，否则返回错误代码。
 * @details 初始化 RTDM 上下文结构，并设置相应的文件描述符和上下文参数。
 */
static int ec_rtdm_open(struct rtdm_fd *fd, int oflags)
{
	struct ec_rtdm_context *ctx = rtdm_fd_to_private(fd);
#if DEBUG_RTDM
	struct rtdm_device *dev = rtdm_fd_device(fd);
	ec_rtdm_dev_t *rtdm_dev = dev->device_data;
#endif

	ctx->fd = fd;

	ctx->ioctl_ctx.writable = oflags & O_WRONLY || oflags & O_RDWR;
	ctx->ioctl_ctx.requested = 0;
	ctx->ioctl_ctx.process_data = NULL;
	ctx->ioctl_ctx.process_data_size = 0;

#if DEBUG_RTDM
	EC_MASTER_INFO(rtdm_dev->master, "已打开 RTDM 设备 %s。\n",
				   dev->name);
#endif

	return 0;
}

/**
 * @brief 关闭 RTDM 设备。
 * @param fd RTDM 文件描述符。
 * @return 无返回值。
 * @details 清理 RTDM 上下文结构，并释放相关资源。
 */
static void ec_rtdm_close(struct rtdm_fd *fd)
{
	struct ec_rtdm_context *ctx = rtdm_fd_to_private(fd);
	struct rtdm_device *dev = rtdm_fd_device(fd);
	ec_rtdm_dev_t *rtdm_dev = dev->device_data;

	if (ctx->ioctl_ctx.requested)
		ecrt_release_master(rtdm_dev->master);

	if (ctx->ioctl_ctx.process_data)
		vfree(ctx->ioctl_ctx.process_data);

#if DEBUG_RTDM
	EC_MASTER_INFO(rtdm_dev->master, "已关闭 RTDM 设备 %s。\n",
				   dev->name);
#endif
}

#if DEBUG_RTDM
/**
 * @brief EC IOCTL 描述结构体。
 */
struct ec_ioctl_desc
{
	unsigned int cmd;     /**< IOCTL 命令。 */
	const char *name;     /**< IOCTL 名称。 */
};

/**
 * @brief 定义 EC IOCTL 宏。
 * @param ioctl IOCTL 命令。
 * @details 定义一个 EC IOCTL 宏，用于设置 IOCTL 命令和名称的映射关系。
 */
#define EC_IOCTL_DEF(ioctl) \
	[_IOC_NR(ioctl)] = {    \
		.cmd = ioctl,       \
		.name = #ioctl}     \
		/**< EC IOCTL 宏定义。 */

static const struct ec_ioctl_desc ec_ioctls[] = {
	EC_IOCTL_DEF(EC_IOCTL_MODULE),                       /**< 模块 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_MASTER),                       /**< 主站 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_SLAVE),                        /**< 从站 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_SLAVE_SYNC),                   /**< 从站同步 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_SLAVE_SYNC_PDO),               /**< 从站同步 PDO IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_SLAVE_SYNC_PDO_ENTRY),         /**< 从站同步 PDO 条目 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_DOMAIN),                       /**< 域 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_DOMAIN_FMMU),                  /**< 域 FMMU IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_DOMAIN_DATA),                  /**< 域数据 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_MASTER_DEBUG),                 /**< 主站调试 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_MASTER_RESCAN),                /**< 主站重新扫描 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_SLAVE_STATE),                  /**< 从站状态 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_SLAVE_SDO),                    /**< 从站 SDO IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_SLAVE_SDO_ENTRY),              /**< 从站 SDO 条目 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_SLAVE_SDO_UPLOAD),             /**< 从站 SDO 上传 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_SLAVE_SDO_DOWNLOAD),           /**< 从站 SDO 下载 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_SLAVE_SII_READ),               /**< 从站 SII 读取 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_SLAVE_SII_WRITE),              /**< 从站 SII 写入 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_SLAVE_REG_READ),               /**< 从站寄存器读取 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_SLAVE_REG_WRITE),              /**< 从站寄存器写入 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_SLAVE_FOE_READ),               /**< 从站 FOE 读取 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_SLAVE_FOE_WRITE),              /**< 从站 FOE 写入 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_SLAVE_SOE_READ),               /**< 从站 SOE 读取 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_SLAVE_SOE_WRITE),              /**< 从站 SOE 写入 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_SLAVE_EOE_IP_PARAM),           /**< 从站 EOE IP 参数 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_CONFIG),                       /**< 配置 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_CONFIG_PDO),                   /**< 配置 PDO IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_CONFIG_PDO_ENTRY),             /**< 配置 PDO 条目 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_CONFIG_SDO),                   /**< 配置 SDO IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_CONFIG_IDN),                   /**< 配置 IDN IOCTL。 */
#ifdef EC_EOE
	EC_IOCTL_DEF(EC_IOCTL_EOE_HANDLER),                  /**< EOE 处理程序 IOCTL。 */
#endif
	EC_IOCTL_DEF(EC_IOCTL_SLAVE_DICT_UPLOAD),            /**< 从站字典上传 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_REQUEST),                      /**< 请求 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_CREATE_DOMAIN),                /**< 创建域 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_CREATE_SLAVE_CONFIG),          /**< 创建从站配置 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_SELECT_REF_CLOCK),             /**< 选择参考时钟 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_ACTIVATE),                     /**< 激活 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_DEACTIVATE),                   /**< 停用 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_SEND),                         /**< 发送数据 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_RECEIVE),                      /**< 接收数据 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_MASTER_STATE),                 /**< 主站状态 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_MASTER_LINK_STATE),            /**< 主站链路状态 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_APP_TIME),                     /**< 应用时间 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_SYNC_REF),                     /**< 同步参考时钟 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_SYNC_SLAVES),                  /**< 同步从站 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_REF_CLOCK_TIME),               /**< 参考时钟时间 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_SYNC_MON_QUEUE),               /**< 同步监视队列 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_SYNC_MON_PROCESS),             /**< 同步监视进程 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_RESET),                        /**< 复位 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_SC_SYNC),                      /**< SC 同步 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_SC_WATCHDOG),                  /**< SC 看门狗 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_SC_ADD_PDO),                   /**< SC 添加 PDO IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_SC_CLEAR_PDOS),                /**< SC 清除 PDOs IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_SC_ADD_ENTRY),                 /**< SC 添加条目 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_SC_CLEAR_ENTRIES),             /**< SC 清除条目 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_SC_REG_PDO_ENTRY),             /**< SC 注册 PDO 条目 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_SC_REG_PDO_POS),               /**< SC 注册 PDO 位置 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_SC_DC),                        /**< SC DC IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_SC_SDO),                       /**< SC SDO IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_SC_EMERG_SIZE),                /**< SC 紧急数据大小 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_SC_EMERG_POP),                 /**< SC 弹出紧急数据 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_SC_EMERG_CLEAR),               /**< SC 清除紧急数据 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_SC_EMERG_OVERRUNS),            /**< SC 紧急数据溢出数 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_SC_SDO_REQUEST),               /**< SC SDO 请求 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_SC_REG_REQUEST),               /**< SC 寄存器请求 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_SC_VOE),                       /**< SC VOE IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_SC_STATE),                     /**< SC 状态 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_SC_IDN),                       /**< SC IDN IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_DOMAIN_SIZE),                  /**< 域大小 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_DOMAIN_OFFSET),                /**< 域偏移量 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_DOMAIN_PROCESS),               /**< 域处理 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_DOMAIN_QUEUE),                 /**< 域队列 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_DOMAIN_STATE),                 /**< 域状态 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_SDO_REQUEST_INDEX),            /**< SDO 请求索引 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_SDO_REQUEST_TIMEOUT),          /**< SDO 请求超时 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_SDO_REQUEST_STATE),            /**< SDO 请求状态 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_SDO_REQUEST_READ),             /**< SDO 请求读取 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_SDO_REQUEST_WRITE),            /**< SDO 请求写入 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_SDO_REQUEST_DATA),             /**< SDO 请求数据 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_REG_REQUEST_DATA),             /**< 寄存器请求数据 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_REG_REQUEST_STATE),            /**< 寄存器请求状态 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_REG_REQUEST_WRITE),            /**< 寄存器请求写入 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_REG_REQUEST_READ),             /**< 寄存器请求读取 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_VOE_SEND_HEADER),              /**< VOE 发送头部 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_VOE_REC_HEADER),               /**< VOE 接收头部 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_VOE_READ),                     /**< VOE 读取 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_VOE_READ_NOSYNC),              /**< VOE 无同步读取 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_VOE_WRITE),                    /**< VOE 写入 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_VOE_EXEC),                     /**< VOE 执行 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_VOE_DATA),                     /**< VOE 数据 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_SET_SEND_INTERVAL),            /**< 设置发送间隔 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_SC_OVERLAPPING_IO),            /**< SC 重叠 IO IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_SLAVE_REBOOT),                 /**< 从站重启 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_SLAVE_REG_READWRITE),          /**< 从站寄存器读写 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_REG_REQUEST_READWRITE),        /**< 寄存器请求读写 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_SETUP_DOMAIN_MEMORY),          /**< 设置域内存 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_DEACTIVATE_SLAVES),            /**< 停用从站 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_64_REF_CLK_TIME_QUEUE),        /**< 64 位参考时钟时间队列 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_64_REF_CLK_TIME),              /**< 64 位参考时钟时间 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_SC_FOE_REQUEST),               /**< SC FOE 请求 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_FOE_REQUEST_FILE),             /**< FOE 请求文件 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_FOE_REQUEST_TIMEOUT),          /**< FOE 请求超时 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_FOE_REQUEST_STATE),            /**< FOE 请求状态 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_FOE_REQUEST_READ),             /**< FOE 请求读取 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_FOE_REQUEST_WRITE),            /**< FOE 请求写入 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_FOE_REQUEST_DATA),             /**< FOE 请求数据 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_RT_SLAVE_REQUESTS),            /**< 实时从站请求 IOCTL。 */
	EC_IOCTL_DEF(EC_IOCTL_EXEC_SLAVE_REQUESTS),          /**< 执行从站请求 IOCTL。 */
};
#endif

/**
 * @brief 在实时环境下处理 RTDM 设备的 IOCTL 请求。
 * @param fd RTDM 文件描述符。
 * @param request IOCTL 请求代码。
 * @param arg 用户空间指针，指向 IOCTL 参数。
 * @return 返回操作结果的整数值。
 * @details
 * - 获取与文件描述符关联的 EC RTDM 上下文。
 * - 获取与文件描述符关联的 RTDM 设备。
 * - 获取 RTDM 设备的 EC RTDM 数据结构。
 * - 如果启用了调试模式，输出 IOCTL 请求的信息。
 * - 对于某些特定的 IOCTL 请求，允许在非实时上下文中执行。
 * - 调用 ec_ioctl_rtdm 函数处理 IOCTL 请求，并返回结果。
 */
static int ec_rtdm_ioctl_rt(struct rtdm_fd *fd, unsigned int request,
							void __user *arg)
{
	struct ec_rtdm_context *ctx = rtdm_fd_to_private(fd);
	struct rtdm_device *dev = rtdm_fd_device(fd);
	ec_rtdm_dev_t *rtdm_dev = dev->device_data;

#if DEBUG_RTDM
	unsigned int nr = _IOC_NR(request);
	const struct ec_ioctl_desc *ioctl = &ec_ioctls[nr];

	EC_MASTER_INFO(rtdm_dev->master, "在 RTDM 设备 %s 上执行 ioctl_rt(request = %u, ctl = %02x %s)。\n",
				   dev->name, request, _IOC_NR(request), ioctl->name);
#endif

	/*
	 * FIXME: 除了以下几个 IOCTL 请求外，在非实时上下文中执行 ioctls，以避免任何未知的系统挂起。
	 */
	switch (request)
	{
	case EC_IOCTL_SEND:
	case EC_IOCTL_RECEIVE:
	case EC_IOCTL_MASTER_STATE:
	case EC_IOCTL_APP_TIME:
	case EC_IOCTL_SYNC_REF:
	case EC_IOCTL_SYNC_SLAVES:
	case EC_IOCTL_REF_CLOCK_TIME:
	case EC_IOCTL_SC_STATE:
	case EC_IOCTL_DOMAIN_PROCESS:
	case EC_IOCTL_DOMAIN_QUEUE:
	case EC_IOCTL_DOMAIN_STATE:
		break;
	default:
		return -ENOSYS;
	}

	return ec_ioctl_rtdm(rtdm_dev->master, &ctx->ioctl_ctx, request, arg);
}

/**
 * @brief 处理 RTDM 设备的 IOCTL 请求。
 * @param fd RTDM 文件描述符。
 * @param request IOCTL 请求代码。
 * @param arg 用户空间指针，指向 IOCTL 参数。
 * @return 返回操作结果的整数值。
 * @details
 * - 获取与文件描述符关联的 EC RTDM 上下文。
 * - 获取与文件描述符关联的 RTDM 设备。
 * - 获取 RTDM 设备的 EC RTDM 数据结构。
 * - 如果启用了调试模式，输出 IOCTL 请求的信息。
 * - 调用 ec_ioctl_rtdm 函数处理 IOCTL 请求，并返回结果。
 */
static int ec_rtdm_ioctl(struct rtdm_fd *fd, unsigned int request,
						 void __user *arg)
{
	struct ec_rtdm_context *ctx = rtdm_fd_to_private(fd);
	struct rtdm_device *dev = rtdm_fd_device(fd);
	ec_rtdm_dev_t *rtdm_dev = dev->device_data;

#if DEBUG_RTDM
	unsigned int nr = _IOC_NR(request);
	const struct ec_ioctl_desc *ioctl = &ec_ioctls[nr];

	EC_MASTER_INFO(rtdm_dev->master, "在 RTDM 设备 %s 上执行 ioctl(request = %u, ctl = %02x %s)。\n",
				   dev->name, request, _IOC_NR(request), ioctl->name);
#endif

	return ec_ioctl_rtdm(rtdm_dev->master, &ctx->ioctl_ctx, request, arg);
}

/**
 * @brief EC RTDM 驱动程序的结构体。
 * @details
 * - profile_info: 驱动程序的配置信息，包括驱动程序名称、RTDM 类别、主设备号和次设备号。
 * - device_flags: RTDM 设备的标志，表示命名设备。
 * - device_count: RTDM 设备的数量，此处为 1。
 * - context_size: RTDM 设备上下文的大小，用于存储与设备相关的数据。
 * - ops: RTDM 设备操作的函数指针集合。
 *   - open: 打开设备的回调函数。
 *   - close: 关闭设备的回调函数。
 *   - ioctl_rt: 在实时上下文中处理 IOCTL 请求的回调函数。
 *   - ioctl_nrt: 在非实时上下文中处理 IOCTL 请求的回调函数。
 */
static struct rtdm_driver ec_rtdm_driver = {
	.profile_info = RTDM_PROFILE_INFO(ec_rtdm,
									  RTDM_CLASS_EXPERIMENTAL,
									  222,
									  0),
	.device_flags = RTDM_NAMED_DEVICE,
	.device_count = 1,
	.context_size = sizeof(struct ec_rtdm_context),
	.ops = {
		.open = ec_rtdm_open,
		.close = ec_rtdm_close,
		.ioctl_rt = ec_rtdm_ioctl_rt,
		.ioctl_nrt = ec_rtdm_ioctl,
	},
};

/**
 * @brief 初始化 EC RTDM 设备。
 * @param rtdm_dev EC RTDM 设备的指针。
 * @param master EC 主站的指针。
 * @return 返回操作结果的整数值。
 * @details
 * - 将 EC 主站指针赋值给 EC RTDM 设备。
 * - 分配内存以存储 RTDM 设备的结构体。
 * - 如果内存分配失败，输出错误信息并返回 -ENOMEM。
 * - 设置 RTDM 设备的相关属性：驱动程序、设备数据、设备标签。
 * - 注册 RTDM 设备。
 * - 如果注册失败，输出错误信息，释放内存并返回错误码。
 * - 输出注册成功的信息。
 * - 返回 0 表示成功。
 */
int ec_rtdm_dev_init(ec_rtdm_dev_t *rtdm_dev, ec_master_t *master)
{
	struct rtdm_device *dev;
	int ret;

	rtdm_dev->master = master;

	rtdm_dev->dev = kzalloc(sizeof(struct rtdm_device), GFP_KERNEL);
	if (!rtdm_dev->dev)
	{
		EC_MASTER_ERR(master, "无法为 RTDM 设备保留内存。\n");
		return -ENOMEM;
	}

	dev = rtdm_dev->dev;

	dev->driver = &ec_rtdm_driver;
	dev->device_data = rtdm_dev;
	dev->label = "EtherCAT%u";

	ret = rtdm_dev_register(dev);
	if (ret)
	{
		EC_MASTER_ERR(master, "RTDM 接口初始化失败 (返回值 %i)。\n", ret);
		kfree(dev);
		return ret;
	}

	EC_MASTER_INFO(master, "已注册 RTDM 设备 %s。\n", dev->name);

	return 0;
}

/**
 * @brief 清除 EC RTDM 设备。
 * @param rtdm_dev EC RTDM 设备的指针。
 * @details
 * - 取消注册 RTDM 设备。
 * - 输出取消注册成功的信息。
 * - 释放 RTDM 设备的内存。
 */
void ec_rtdm_dev_clear(ec_rtdm_dev_t *rtdm_dev)
{
	rtdm_dev_unregister(rtdm_dev->dev);

	EC_MASTER_INFO(rtdm_dev->master, "已取消注册 RTDM 设备 %s。\n",
				   rtdm_dev->dev->name);

	kfree(rtdm_dev->dev);
}

/**
 * @brief 将 IOCTL 上下文的数据映射到用户空间。
 * @param ioctl_ctx IOCTL 上下文的指针。
 * @param user_address 用户空间的指针。
 * @return 返回操作结果的整数值。
 * @details
 * - 根据 IOCTL 上下文获取 EC RTDM 上下文。
 * - 将进程数据映射到用户空间，设置读写权限。
 * - 如果映射失败，返回错误码。
 * - 返回 0 表示成功。
 */
int ec_rtdm_mmap(ec_ioctl_context_t *ioctl_ctx, void **user_address)
{
	struct ec_rtdm_context *ctx =
		container_of(ioctl_ctx, struct ec_rtdm_context, ioctl_ctx);
	int ret;

	ret = rtdm_mmap_to_user(ctx->fd,
							ioctl_ctx->process_data, ioctl_ctx->process_data_size,
							PROT_READ | PROT_WRITE,
							user_address,
							NULL, NULL);
	if (ret < 0)
		return ret;

	return 0;
}

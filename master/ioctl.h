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
   EtherCAT master character device IOCTL commands.
*/

/*****************************************************************************/

#ifndef __EC_IOCTL_H__
#define __EC_IOCTL_H__

#include <linux/ioctl.h>

#include "globals.h"

/*****************************************************************************/

/** \cond */

#define EC_IOCTL_TYPE 0xa4  // 自定义IOCTL类型

#define EC_IO(nr) _IO(EC_IOCTL_TYPE, nr)  // 定义无参数的IOCTL
#define EC_IOR(nr, type) _IOR(EC_IOCTL_TYPE, nr, type)  // 定义读取参数的IOCTL
#define EC_IOW(nr, type) _IOW(EC_IOCTL_TYPE, nr, type)  // 定义写入参数的IOCTL
#define EC_IOWR(nr, type) _IOWR(EC_IOCTL_TYPE, nr, type)  // 定义读写参数的IOCTL

/** EtherCAT主站ioctl()版本魔数。
 *
 * 在更改ioctl接口时递增该值！
 */
#define EC_IOCTL_VERSION_MAGIC 36

// 命令行工具
#define EC_IOCTL_MODULE EC_IOR(0x00, ec_ioctl_module_t)  // 模块IOCTL
#define EC_IOCTL_MASTER EC_IOR(0x01, ec_ioctl_master_t)  // 主站IOCTL
#define EC_IOCTL_SLAVE EC_IOWR(0x02, ec_ioctl_slave_t)  // 从站IOCTL
#define EC_IOCTL_SLAVE_SYNC EC_IOWR(0x03, ec_ioctl_slave_sync_t)  // 从站同步IOCTL
#define EC_IOCTL_SLAVE_SYNC_PDO EC_IOWR(0x04, ec_ioctl_slave_sync_pdo_t)  // 从站同步PDO IOCT
#define EC_IOCTL_SLAVE_SYNC_PDO_ENTRY EC_IOWR(0x05, ec_ioctl_slave_sync_pdo_entry_t)  // 从站同步PDO条目IOCTL
#define EC_IOCTL_DOMAIN EC_IOWR(0x06, ec_ioctl_domain_t)  // 域IOCTL
#define EC_IOCTL_DOMAIN_FMMU EC_IOWR(0x07, ec_ioctl_domain_fmmu_t)  // 域FMMU IOCT
#define EC_IOCTL_DOMAIN_DATA EC_IOWR(0x08, ec_ioctl_domain_data_t)  // 域数据IOCTL
#define EC_IOCTL_MASTER_DEBUG EC_IO(0x09)  // 主站调试IOCTL
#define EC_IOCTL_MASTER_RESCAN EC_IO(0x0a)  // 主站重新扫描IOCTL
#define EC_IOCTL_SLAVE_STATE EC_IOW(0x0b, ec_ioctl_slave_state_t)  // 从站状态IOCTL
#define EC_IOCTL_SLAVE_SDO EC_IOWR(0x0c, ec_ioctl_slave_sdo_t)  // 从站SDO IOCT
#define EC_IOCTL_SLAVE_SDO_ENTRY EC_IOWR(0x0d, ec_ioctl_slave_sdo_entry_t)  // 从站SDO条目IOCTL
#define EC_IOCTL_SLAVE_SDO_UPLOAD EC_IOWR(0x0e, ec_ioctl_slave_sdo_upload_t)  // 从站SDO上传IOCTL
#define EC_IOCTL_SLAVE_SDO_DOWNLOAD EC_IOWR(0x0f, ec_ioctl_slave_sdo_download_t)  // 从站SDO下载IOCTL
#define EC_IOCTL_SLAVE_SII_READ EC_IOWR(0x10, ec_ioctl_slave_sii_t)  // 从站SII读取IOCTL
#define EC_IOCTL_SLAVE_SII_WRITE EC_IOW(0x11, ec_ioctl_slave_sii_t)  // 从站SII写入IOCTL
#define EC_IOCTL_SLAVE_REG_READ EC_IOWR(0x12, ec_ioctl_slave_reg_t)  // 从站寄存器读取IOCTL
#define EC_IOCTL_SLAVE_REG_WRITE EC_IOW(0x13, ec_ioctl_slave_reg_t)  // 从站寄存器写入IOCTL
#define EC_IOCTL_SLAVE_FOE_READ EC_IOWR(0x14, ec_ioctl_slave_foe_t)  // 从站FOE读取IOCTL
#define EC_IOCTL_SLAVE_FOE_WRITE EC_IOW(0x15, ec_ioctl_slave_foe_t)  // 从站FOE写入IOCTL
#define EC_IOCTL_SLAVE_SOE_READ EC_IOWR(0x16, ec_ioctl_slave_soe_read_t)  // 从站SOE读取IOCTL
#define EC_IOCTL_SLAVE_SOE_WRITE EC_IOWR(0x17, ec_ioctl_slave_soe_write_t)  // 从站SOE写入IOCTL
#ifdef EC_EOE
#define EC_IOCTL_SLAVE_EOE_IP_PARAM EC_IOW(0x18, ec_ioctl_slave_eoe_ip_t)  // 从站EOE IP参数IOCTL
#endif
#define EC_IOCTL_CONFIG EC_IOWR(0x19, ec_ioctl_config_t)  // 配置IOCTL
#define EC_IOCTL_CONFIG_PDO EC_IOWR(0x1a, ec_ioctl_config_pdo_t)  // 配置PDO IOCT
#define EC_IOCTL_CONFIG_PDO_ENTRY EC_IOWR(0x1b, ec_ioctl_config_pdo_entry_t)  // 配置PDO条目IOCTL
#define EC_IOCTL_CONFIG_SDO EC_IOWR(0x1c, ec_ioctl_config_sdo_t)  // 配置SDO IOCT
#define EC_IOCTL_CONFIG_IDN EC_IOWR(0x1d, ec_ioctl_config_idn_t)  // 配置IDN IOCT
#ifdef EC_EOE
#define EC_IOCTL_EOE_HANDLER EC_IOWR(0x1e, ec_ioctl_eoe_handler_t)  // EOE处理器IOCTL
#endif
#define EC_IOCTL_SLAVE_DICT_UPLOAD EC_IOW(0x7f, ec_ioctl_slave_dict_upload_t)  // 从站字典上传IOCTL

// 应用程序接口
#define EC_IOCTL_REQUEST EC_IO(0x1f)  // 请求IOCTL
#define EC_IOCTL_CREATE_DOMAIN EC_IO(0x20)  // 创建域
#define EC_IOCTL_CREATE_SLAVE_CONFIG EC_IOWR(0x21, ec_ioctl_config_t)  // 创建从站配置
#define EC_IOCTL_SELECT_REF_CLOCK EC_IOW(0x22, uint32_t)  // 选择参考时钟
#define EC_IOCTL_ACTIVATE EC_IOR(0x23, ec_ioctl_master_activate_t)  // 激活
#define EC_IOCTL_DEACTIVATE EC_IO(0x24)  // 停用
#define EC_IOCTL_SEND EC_IO(0x25)  // 发送
#define EC_IOCTL_RECEIVE EC_IO(0x26)  // 接收
#define EC_IOCTL_MASTER_STATE EC_IOR(0x27, ec_master_state_t)  // 主站状态
#define EC_IOCTL_MASTER_LINK_STATE EC_IOWR(0x28, ec_ioctl_link_state_t)  // 主站链路状态
#define EC_IOCTL_APP_TIME EC_IOW(0x29, uint64_t)  // 应用程序时间
#define EC_IOCTL_SYNC_REF EC_IO(0x2a)  // 同步参考
#define EC_IOCTL_SYNC_REF_TO EC_IOW(0x2b, uint64_t)  // 同步参考到
#define EC_IOCTL_SYNC_SLAVES EC_IO(0x2c)  // 同步从站
#define EC_IOCTL_REF_CLOCK_TIME EC_IOR(0x2d, uint32_t)  // 参考时钟时间
#define EC_IOCTL_SYNC_MON_QUEUE EC_IO(0x2e)  // 同步监控队列
#define EC_IOCTL_SYNC_MON_PROCESS EC_IOR(0x2f, uint32_t)  // 同步监控处理
#define EC_IOCTL_RESET EC_IO(0x30)  // 复位
#define EC_IOCTL_SC_SYNC EC_IOW(0x31, ec_ioctl_config_t)  // SC同步
#define EC_IOCTL_SC_WATCHDOG EC_IOW(0x32, ec_ioctl_config_t)  // SC看门狗
#define EC_IOCTL_SC_ADD_PDO EC_IOW(0x33, ec_ioctl_config_pdo_t)  // SC添加PDO
#define EC_IOCTL_SC_CLEAR_PDOS EC_IOW(0x34, ec_ioctl_config_pdo_t)  // SC清除PDOS
#define EC_IOCTL_SC_ADD_ENTRY EC_IOW(0x35, ec_ioctl_add_pdo_entry_t)  // SC添加条目
#define EC_IOCTL_SC_CLEAR_ENTRIES EC_IOW(0x36, ec_ioctl_config_pdo_t)  // SC清除条目
#define EC_IOCTL_SC_REG_PDO_ENTRY EC_IOWR(0x37, ec_ioctl_reg_pdo_entry_t)  // SC注册PDO条目
#define EC_IOCTL_SC_REG_PDO_POS EC_IOWR(0x38, ec_ioctl_reg_pdo_pos_t)  // SC注册PDO位置
#define EC_IOCTL_SC_DC EC_IOW(0x39, ec_ioctl_config_t)  // SC DC
#define EC_IOCTL_SC_SDO EC_IOW(0x3a, ec_ioctl_sc_sdo_t)  // SC SDO
#define EC_IOCTL_SC_EMERG_SIZE EC_IOW(0x3b, ec_ioctl_sc_emerg_t)  // SC紧急大小
#define EC_IOCTL_SC_EMERG_POP EC_IOWR(0x3c, ec_ioctl_sc_emerg_t)  // SC紧急弹出
#define EC_IOCTL_SC_EMERG_CLEAR EC_IOW(0x3d, ec_ioctl_sc_emerg_t)  // SC紧急清除
#define EC_IOCTL_SC_EMERG_OVERRUNS EC_IOWR(0x3e, ec_ioctl_sc_emerg_t)  // SC紧急溢出
#define EC_IOCTL_SC_SDO_REQUEST EC_IOWR(0x3f, ec_ioctl_sdo_request_t)  // SC SDO请求
#define EC_IOCTL_SC_REG_REQUEST EC_IOWR(0x40, ec_ioctl_reg_request_t)  // SC注册请求
#define EC_IOCTL_SC_VOE EC_IOWR(0x41, ec_ioctl_voe_t)  // SC VOE
#define EC_IOCTL_SC_STATE EC_IOWR(0x42, ec_ioctl_sc_state_t)  // SC状态
#define EC_IOCTL_SC_IDN EC_IOW(0x43, ec_ioctl_sc_idn_t)  // SC IDN
#define EC_IOCTL_DOMAIN_SIZE EC_IO(0x44)  // 域大小
#define EC_IOCTL_DOMAIN_OFFSET EC_IO(0x45)  // 域偏移量
#define EC_IOCTL_DOMAIN_PROCESS EC_IO(0x46)  // 域处理
#define EC_IOCTL_DOMAIN_QUEUE EC_IO(0x47)  // 域队列
#define EC_IOCTL_DOMAIN_STATE EC_IOWR(0x48, ec_ioctl_domain_state_t)  // 域状态
#define EC_IOCTL_SDO_REQUEST_INDEX EC_IOWR(0x49, ec_ioctl_sdo_request_t)  // SDO请求索引
#define EC_IOCTL_SDO_REQUEST_TIMEOUT EC_IOWR(0x4a, ec_ioctl_sdo_request_t)  // SDO请求超时
#define EC_IOCTL_SDO_REQUEST_STATE EC_IOWR(0x4b, ec_ioctl_sdo_request_t)  // SDO请求状态
#define EC_IOCTL_SDO_REQUEST_READ EC_IOWR(0x4c, ec_ioctl_sdo_request_t)  // SDO请求读取
#define EC_IOCTL_SDO_REQUEST_WRITE EC_IOWR(0x4d, ec_ioctl_sdo_request_t)  // SDO请求写入
#define EC_IOCTL_SDO_REQUEST_DATA EC_IOWR(0x4e, ec_ioctl_sdo_request_t)  // SDO请求数据
#define EC_IOCTL_REG_REQUEST_DATA EC_IOWR(0x4f, ec_ioctl_reg_request_t)  // 注册请求数据
#define EC_IOCTL_REG_REQUEST_STATE EC_IOWR(0x50, ec_ioctl_reg_request_t)  // 注册请求状态
#define EC_IOCTL_REG_REQUEST_WRITE EC_IOWR(0x51, ec_ioctl_reg_request_t)  // 注册请求写入
#define EC_IOCTL_REG_REQUEST_READ EC_IOWR(0x52, ec_ioctl_reg_request_t)  // 注册请求读取
#define EC_IOCTL_VOE_SEND_HEADER EC_IOW(0x53, ec_ioctl_voe_t)  // VOE发送头部
#define EC_IOCTL_VOE_REC_HEADER EC_IOWR(0x54, ec_ioctl_voe_t)  // VOE接收头部
#define EC_IOCTL_VOE_READ EC_IOW(0x55, ec_ioctl_voe_t)  // VOE读取
#define EC_IOCTL_VOE_READ_NOSYNC EC_IOW(0x56, ec_ioctl_voe_t)  // VOE无同步读取
#define EC_IOCTL_VOE_WRITE EC_IOWR(0x57, ec_ioctl_voe_t)  // VOE写入
#define EC_IOCTL_VOE_EXEC EC_IOWR(0x58, ec_ioctl_voe_t)  // VOE执行
#define EC_IOCTL_VOE_DATA EC_IOWR(0x59, ec_ioctl_voe_t)  // VOE数据
#define EC_IOCTL_SET_SEND_INTERVAL EC_IOW(0x5a, size_t)  // 设置发送间隔
#define EC_IOCTL_SC_OVERLAPPING_IO EC_IOW(0x5b, ec_ioctl_config_t)  // SC重叠IO
#define EC_IOCTL_SLAVE_REBOOT EC_IOW(0x5c, ec_ioctl_slave_reboot_t)  // 从站重启
#define EC_IOCTL_SLAVE_REG_READWRITE EC_IOWR(0x5d, ec_ioctl_slave_reg_t)  // 从站注册读写
#define EC_IOCTL_REG_REQUEST_READWRITE EC_IOWR(0x5e, ec_ioctl_reg_request_t)  // 注册请求读写
#define EC_IOCTL_SETUP_DOMAIN_MEMORY EC_IOR(0x60, ec_ioctl_master_activate_t)  // 设置域内存
#define EC_IOCTL_DEACTIVATE_SLAVES EC_IO(0x61)  // 停用从站
#define EC_IOCTL_64_REF_CLK_TIME_QUEUE EC_IO(0x62)  // 64位参考时钟时间队列
#define EC_IOCTL_64_REF_CLK_TIME EC_IOR(0x63, uint64_t)  // 64位参考时钟时间
#define EC_IOCTL_SC_FOE_REQUEST EC_IOWR(0x64, ec_ioctl_foe_request_t)  // SC FOE请求
#define EC_IOCTL_FOE_REQUEST_FILE EC_IOWR(0x65, ec_ioctl_foe_request_t)  // FOE请求文件
#define EC_IOCTL_FOE_REQUEST_TIMEOUT EC_IOWR(0x66, ec_ioctl_foe_request_t)  // FOE请求超时
#define EC_IOCTL_FOE_REQUEST_STATE EC_IOWR(0x67, ec_ioctl_foe_request_t)  // FOE请求状态
#define EC_IOCTL_FOE_REQUEST_READ EC_IOWR(0x68, ec_ioctl_foe_request_t)  // FOE请求读取
#define EC_IOCTL_FOE_REQUEST_WRITE EC_IOWR(0x69, ec_ioctl_foe_request_t)  // FOE请求写入
#define EC_IOCTL_FOE_REQUEST_DATA EC_IOWR(0x6a, ec_ioctl_foe_request_t)  // FOE请求数据
#define EC_IOCTL_RT_SLAVE_REQUESTS EC_IOW(0x6b, uint32_t)  // RT从站请求
#define EC_IOCTL_EXEC_SLAVE_REQUESTS EC_IO(0x6c)  // 执行从站请求

#if defined(EC_RTDM) && (EC_EOE)
#define EC_IOCTL_EOE_IS_OPEN EC_IO(0x6d)  // EOE是否打开
#define EC_IOCTL_EOE_PROCESS EC_IO(0x6e)  // EOE处理
#define EC_IOCTL_SEND_EXT EC_IO(0x6f)  // 发送扩展
#endif

#ifdef EC_EOE
#define EC_IOCTL_EOE_ADDIF EC_IOWR(0x70, ec_ioctl_eoe_if_t)  // EOE添加接口
#define EC_IOCTL_EOE_DELIF EC_IOWR(0x71, ec_ioctl_eoe_if_t)  // EOE删除接口
#endif

#define EC_IOCTL_PCAP_DATA EC_IOWR(0x72, ec_ioctl_pcap_data_t)  // PCAP数据

// 邮箱网关
#define EC_IOCTL_MBOX_GATEWAY EC_IOWR(0x73, ec_ioctl_mbox_gateway_t)  // 邮箱网关

/*****************************************************************************/

#define EC_IOCTL_STRING_SIZE 64  // 字符串大小

/*****************************************************************************/

typedef struct
{
    uint32_t ioctl_version_magic;  // ioctl版本魔数
    uint32_t master_count;  // 主站数量
} ec_ioctl_module_t;

/*****************************************************************************/

typedef struct
{
    uint32_t slave_count;  // 从站数量
    uint32_t config_count;  // 配置数量
    uint32_t domain_count;  // 域数量
#ifdef EC_EOE
    uint32_t eoe_handler_count;  // EOE处理器数量
#endif
    uint8_t phase;  // 相位
    uint8_t active;  // 活动状态
    uint8_t scan_busy;  // 扫描忙状态
    struct ec_ioctl_device
    {
        uint8_t address[6];  // 地址
        uint8_t attached;  // 是否已连接
        uint8_t link_state;  // 连接状态
        uint64_t tx_count;  // 发送计数
        uint64_t rx_count;  // 接收计数
        uint64_t tx_bytes;  // 发送字节数
        uint64_t rx_bytes;  // 接收字节数
        uint64_t tx_errors;  // 发送错误计数
        int32_t tx_frame_rates[EC_RATE_COUNT];  // 发送帧速率
        int32_t rx_frame_rates[EC_RATE_COUNT];  // 接收帧速率
        int32_t tx_byte_rates[EC_RATE_COUNT];  // 发送字节速率
        int32_t rx_byte_rates[EC_RATE_COUNT];  // 接收字节速率
    } devices[EC_MAX_NUM_DEVICES];  // 设备列表
    uint32_t num_devices;  // 设备数量
    uint64_t tx_count;  // 总发送计数
    uint64_t rx_count;  // 总接收计数
    uint64_t tx_bytes;  // 总发送字节数
    uint64_t rx_bytes;  // 总接收字节数
    int32_t tx_frame_rates[EC_RATE_COUNT];  // 总发送帧速率
    int32_t rx_frame_rates[EC_RATE_COUNT];  // 总接收帧速率
    int32_t tx_byte_rates[EC_RATE_COUNT];  // 总发送字节速率
    int32_t rx_byte_rates[EC_RATE_COUNT];  // 总接收字节速率
    int32_t loss_rates[EC_RATE_COUNT];  // 丢包率
    uint64_t app_time;  // 应用程序时间
    uint64_t dc_ref_time;  // DC参考时间
    uint16_t ref_clock;  // 参考时钟
    uint32_t pcap_size;  // PCAP文件大小
} ec_ioctl_master_t;

/*****************************************************************************/

typedef struct
{
    // 输入
    uint16_t position;  // 位置

    // 输出
    unsigned int device_index;  // 设备索引
    uint32_t vendor_id;  // 厂商ID
    uint32_t product_code;  // 产品代码
    uint32_t revision_number;  // 修订号
    uint32_t serial_number;  // 序列号
    uint16_t alias;  // 别名
    uint16_t boot_rx_mailbox_offset;  // 启动接收邮箱偏移量
    uint16_t boot_rx_mailbox_size;  // 启动接收邮箱大小
    uint16_t boot_tx_mailbox_offset;  // 启动发送邮箱偏移量
    uint16_t boot_tx_mailbox_size;  // 启动发送邮箱大小
    uint16_t std_rx_mailbox_offset;  // 标准接收邮箱偏移量
    uint16_t std_rx_mailbox_size;  // 标准接收邮箱大小
    uint16_t std_tx_mailbox_offset;  // 标准发送邮箱偏移量
    uint16_t std_tx_mailbox_size;  // 标准发送邮箱大小
    uint16_t mailbox_protocols;  // 邮箱协议
    uint8_t has_general_category;  // 是否具有一般类别
    ec_sii_coe_details_t coe_details;  // COE详细信息
    ec_sii_general_flags_t general_flags;  // 一般标志
    int16_t current_on_ebus;  // 总线上的电流
    struct
    {
        ec_slave_port_desc_t desc;  // 端口描述
        ec_slave_port_link_t link;  // 端口连接
        uint32_t receive_time;  // 接收时间
        uint16_t next_slave;  // 下一个从站
        uint32_t delay_to_next_dc;  // 到下一个DC的延迟
    } ports[EC_MAX_PORTS];  // 端口列表
    uint8_t upstream_port;  // 上游端口
    uint8_t fmmu_bit;  // FMMU位
    uint8_t dc_supported;  // 是否支持DC
    ec_slave_dc_range_t dc_range;  // DC范围
    uint8_t has_dc_system_time;  // 是否具有DC系统时间
    uint32_t transmission_delay;  // 传输延迟
    uint8_t al_state;  // AL状态
    uint8_t error_flag;  // 错误标志
    uint8_t scan_required;  // 是否需要扫描
    uint8_t ready;  // 准备状态
    uint8_t sync_count;  // 同步计数
    uint16_t sdo_count;  // SDO计数
    uint32_t sii_nwords;  // SII字数
    char group[EC_IOCTL_STRING_SIZE];  // 组名
    char image[EC_IOCTL_STRING_SIZE];  // 镜像
    char order[EC_IOCTL_STRING_SIZE];  // 顺序
    char name[EC_IOCTL_STRING_SIZE];  // 名称
} ec_ioctl_slave_t;

/*****************************************************************************/

typedef struct
{
    // 输入
    uint16_t slave_position;  // 从站位置
    uint32_t sync_index;  // 同步索引

    // 输出
    uint16_t physical_start_address;  // 物理起始地址
    uint16_t default_size;  // 默认大小
    uint8_t control_register;  // 控制寄存器
    uint8_t enable;  // 使能标志
    uint8_t pdo_count;  // PDO计数
} ec_ioctl_slave_sync_t;

/*****************************************************************************/

typedef struct
{
    // 输入
    uint16_t slave_position;  // 从站位置
    uint32_t sync_index;  // 同步索引
    uint32_t pdo_pos;  // PDO位置

    // 输出
    uint16_t index;  // 索引
    uint8_t entry_count;  // 条目计数
    int8_t name[EC_IOCTL_STRING_SIZE];  // 名称
} ec_ioctl_slave_sync_pdo_t;

/*****************************************************************************/

typedef struct
{
    // 输入
    uint16_t slave_position;  // 从站位置
    uint32_t sync_index;  // 同步索引
    uint32_t pdo_pos;  // PDO位置
    uint32_t entry_pos;  // 条目位置

    // 输出
    uint16_t index;  // 索引
    uint8_t subindex;  // 子索引
    uint8_t bit_length;  // 位长度
    int8_t name[EC_IOCTL_STRING_SIZE];  // 名称
} ec_ioctl_slave_sync_pdo_entry_t;

/*****************************************************************************/

typedef struct
{
    // 输入
    uint32_t index;  // 域索引

    // 输出
    uint32_t data_size;  // 数据大小
    uint32_t logical_base_address;  // 逻辑基地址
    uint16_t working_counter[EC_MAX_NUM_DEVICES];  // 工作计数器
    uint16_t expected_working_counter;  // 期望工作计数器
    uint32_t fmmu_count;  // FMMU计数
} ec_ioctl_domain_t;

/*****************************************************************************/

typedef struct
{
    // 输入
    uint32_t domain_index;  // 域索引
    uint32_t fmmu_index;  // FMMU索引

    // 输出
    uint16_t slave_config_alias;  // 从站配置别名
    uint16_t slave_config_position;  // 从站配置位置
    uint8_t sync_index;  // 同步索引
    ec_direction_t dir;  // 传输方向
    uint32_t logical_address;  // 逻辑地址
    uint32_t data_size;  // 数据大小
} ec_ioctl_domain_fmmu_t;

/*****************************************************************************/

typedef struct
{
    // 输入
    uint32_t domain_index;  // 域索引
    uint32_t data_size;  // 数据大小
    uint8_t *target;  // 目标数据缓冲区
} ec_ioctl_domain_data_t;

/*****************************************************************************/

typedef struct
{
    // 输入
    uint32_t data_size;  // 数据大小
    uint8_t reset_data;  // 复位数据标志
    uint8_t *target;  // 目标数据缓冲区
} ec_ioctl_pcap_data_t;

/*****************************************************************************/

typedef struct
{
    // 输入
    uint16_t slave_position;  // 从站位置
    uint8_t al_state;  // AL状态
} ec_ioctl_slave_state_t;

/*****************************************************************************/

typedef struct
{
    // 输入
    uint16_t slave_position;  // 从站位置
    uint16_t sdo_position;  // SDO位置

    // 输出
    uint16_t sdo_index;  // SDO索引
    uint8_t max_subindex;  // 最大子索引
    int8_t name[EC_IOCTL_STRING_SIZE];  // 名称
} ec_ioctl_slave_sdo_t;

/*****************************************************************************/

typedef struct
{
    // 输入
    uint16_t slave_position;  // 从站位置
    int sdo_spec;             // 正数：索引，负数：列表位置
    uint8_t sdo_entry_subindex;  // SDO子索引

    // 输出
    uint16_t data_type;  // 数据类型
    uint16_t bit_length;  // 位长度
    uint8_t read_access[EC_SDO_ENTRY_ACCESS_COUNT];  // 读访问权限
    uint8_t write_access[EC_SDO_ENTRY_ACCESS_COUNT];  // 写访问权限
    int8_t description[EC_IOCTL_STRING_SIZE];  // 描述
} ec_ioctl_slave_sdo_entry_t;

/*****************************************************************************/

typedef struct
{
    // 输入
    uint16_t slave_position;  // 从站位置
    uint16_t sdo_index;  // SDO索引
    uint8_t sdo_entry_subindex;  // SDO子索引
    uint8_t complete_access;  // 完全访问标志
    size_t target_size;  // 目标数据大小
    uint8_t *target;  // 目标数据缓冲区

    // 输出
    size_t data_size;  // 数据大小
    uint32_t abort_code;  // 中止代码
} ec_ioctl_slave_sdo_upload_t;

/*****************************************************************************/

typedef struct
{
    // 输入
    uint16_t slave_position;  // 从站位置
    uint16_t sdo_index;  // SDO索引
    uint8_t sdo_entry_subindex;  // SDO条目子索引
    uint8_t complete_access;  // 完全访问标志
    size_t data_size;  // 数据大小
    const uint8_t *data;  // 数据指针

    // 输出
    uint32_t abort_code;  // 中止代码
} ec_ioctl_slave_sdo_download_t;

/*****************************************************************************/

typedef struct
{
    // 输入
    uint16_t slave_position;  // 从站位置
    uint16_t offset;  // 偏移量
    uint32_t nwords;  // 字数
    uint16_t *words;  // 字指针
} ec_ioctl_slave_sii_t;

/*****************************************************************************/

typedef struct
{
    // 输入
    uint16_t slave_position;  // 从站位置
    uint8_t emergency;  // 紧急标志
    uint16_t address;  // 地址
    size_t size;  // 大小
    uint8_t *data;  // 数据指针
} ec_ioctl_slave_reg_t;

/*****************************************************************************/

typedef struct
{
    // 输入
    uint16_t slave_position;  // 从站位置
    uint8_t broadcast;  // 广播标志
} ec_ioctl_slave_reboot_t;

/*****************************************************************************/

typedef struct
{
    // 输入
    uint32_t password;  // 密码
    uint16_t slave_position;  // 从站位置
    uint16_t offset;  // 偏移量
    size_t buffer_size;  // 缓冲区大小
    uint8_t *buffer;  // 缓冲区指针

    // 输出
    size_t data_size;  // 数据大小
    uint32_t result;  // 结果
    uint32_t error_code;  // 错误代码

    char file_name[255];  // 文件名
} ec_ioctl_slave_foe_t;

/*****************************************************************************/

typedef struct
{
    // 输入
    uint16_t slave_position;  // 从站位置
    uint8_t drive_no;  // 驱动器号
    uint16_t idn;  // IDN
    size_t mem_size;  // 内存大小
    uint8_t *data;  // 数据指针

    // 输出
    size_t data_size;  // 数据大小
    uint16_t error_code;  // 错误代码
} ec_ioctl_slave_soe_read_t;

/*****************************************************************************/

typedef struct
{
    // 输入
    uint16_t slave_position;  // 从站位置
    uint8_t drive_no;  // 驱动器号
    uint16_t idn;  // IDN
    size_t data_size;  // 数据大小
    uint8_t *data;  // 数据指针

    // 输出
    uint16_t error_code;  // 错误代码
} ec_ioctl_slave_soe_write_t;

/*****************************************************************************/

typedef struct
{
    // 输入
    uint32_t config_index;  // 配置索引

    // 输出
    uint16_t alias;  // 别名
    uint16_t position;  // 位置
    uint32_t vendor_id;  // 厂商ID
    uint32_t product_code;  // 产品代码
    struct
    {
        ec_direction_t dir;  // 方向
        ec_watchdog_mode_t watchdog_mode;  // 看门狗模式
        uint32_t pdo_count;  // PDO计数
        uint8_t config_this;  // 配置标志
    } syncs[EC_MAX_SYNC_MANAGERS];  // 同步管理器数组
    uint16_t watchdog_divider;  // 看门狗分频器
    uint16_t watchdog_intervals;  // 看门狗间隔
    uint32_t sdo_count;  // SDO计数
    uint32_t idn_count;  // IDN计数
    int32_t slave_position;  // 从站位置
    uint16_t dc_assign_activate;  // DC分配激活标志
    ec_sync_signal_t dc_sync[EC_SYNC_SIGNAL_COUNT];  // DC同步信号数组
    uint8_t allow_overlapping_pdos;  // 允许重叠PDO标志
} ec_ioctl_config_t;

/*****************************************************************************/

typedef struct
{
    // 输入
    uint32_t config_index;  // 配置索引
    uint8_t sync_index;  // 同步索引
    uint16_t pdo_pos;  // PDO位置

    // 输出
    uint16_t index;  // 索引
    uint8_t entry_count;  // 条目计数
    int8_t name[EC_IOCTL_STRING_SIZE];  // 名称
} ec_ioctl_config_pdo_t;

/*****************************************************************************/

typedef struct
{
    // 输入
    uint32_t config_index;  // 配置索引
    uint8_t sync_index;  // 同步索引
    uint16_t pdo_pos;  // PDO位置
    uint8_t entry_pos;  // 条目位置

    // 输出
    uint16_t index;  // 索引
    uint8_t subindex;  // 子索引
    uint8_t bit_length;  // 位长度
    int8_t name[EC_IOCTL_STRING_SIZE];  // 名称
} ec_ioctl_config_pdo_entry_t;

/*****************************************************************************/

/** 显示的SDO数据的最大大小。
 * \todo 使其动态。
 */
#define EC_MAX_SDO_DATA_SIZE 1024

typedef struct
{
    // 输入
    uint32_t config_index;  // 配置索引
    uint32_t sdo_pos;  // SDO位置

    // 输出
    uint16_t index;  // 索引
    uint8_t subindex;  // 子索引
    size_t size;  // 大小
    uint8_t data[EC_MAX_SDO_DATA_SIZE];  // 数据数组
    uint8_t complete_access;  // 完全访问标志
} ec_ioctl_config_sdo_t;

/*****************************************************************************/

/** 显示的IDN数据的最大大小。
 * \todo 使其动态。
 */
#define EC_MAX_IDN_DATA_SIZE 1024

typedef struct
{
    // 输入
    uint32_t config_index;  // 配置索引
    uint32_t idn_pos;  // IDN位置

    // 输出
    uint8_t drive_no;  // 驱动器号
    uint16_t idn;  // IDN
    ec_al_state_t state;  // 状态
    size_t size;  // 大小
    uint8_t data[EC_MAX_IDN_DATA_SIZE];  // 数据数组
} ec_ioctl_config_idn_t;

/*****************************************************************************/

#ifdef EC_EOE

typedef struct
{
    // 输入
    uint16_t eoe_index;  // EOE索引

    // 输出
    char name[EC_DATAGRAM_NAME_SIZE];  // 名称
    uint16_t slave_position;  // 从站位置
    uint8_t open;  // 打开标志
    uint32_t rx_bytes;  // 接收字节数
    uint32_t rx_rate;  // 接收速率
    uint32_t tx_bytes;  // 发送字节数
    uint32_t tx_rate;  // 发送速率
    uint32_t tx_queued_frames;  // 发送队列中的帧数
    uint32_t tx_queue_size;  // 发送队列大小
} ec_ioctl_eoe_handler_t;

/*****************************************************************************/

typedef struct
{
    // 输入
    uint16_t alias;  // 别名
    uint16_t position;  // 位置
} ec_ioctl_eoe_if_t;

#endif

/*****************************************************************************/

#define EC_ETH_ALEN 6
#ifdef ETH_ALEN
#if ETH_ALEN != EC_ETH_ALEN
#error Ethernet address length mismatch
#endif
#endif

#ifdef EC_EOE
typedef struct
{
    // 输入
    uint16_t slave_position;  // 从站位置

    uint8_t mac_address_included;  // MAC地址是否包含
    uint8_t ip_address_included;  // IP地址是否包含
    uint8_t subnet_mask_included;  // 子网掩码是否包含
    uint8_t gateway_included;  // 网关是否包含
    uint8_t dns_included;  // DNS是否包含
    uint8_t name_included;  // 名称是否包含

    unsigned char mac_address[EC_ETH_ALEN];  // MAC地址
    uint32_t ip_address;  // IP地址
    uint32_t subnet_mask;  // 子网掩码
    uint32_t gateway;  // 网关
    uint32_t dns;  // DNS
    char name[EC_MAX_HOSTNAME_SIZE];  // 名称

    // 输出
    uint16_t result;  // 结果
} ec_ioctl_slave_eoe_ip_t;

#endif
/*****************************************************************************/

typedef struct
{
    // 输出
    void *process_data;  // 进程数据指针
    size_t process_data_size;  // 进程数据大小
} ec_ioctl_master_activate_t;

/*****************************************************************************/

typedef struct
{
    // 输入
    uint32_t config_index;  // 配置索引
    uint16_t pdo_index;  // PDO索引
    uint16_t entry_index;  // 条目索引
    uint8_t entry_subindex;  // 条目子索引
    uint8_t entry_bit_length;  // 条目位长度
} ec_ioctl_add_pdo_entry_t;

/*****************************************************************************/

typedef struct
{
    // 输入
    uint32_t config_index;  // 配置索引
    uint16_t entry_index;  // 条目索引
    uint8_t entry_subindex;  // 条目子索引
    uint32_t domain_index;  // 域索引

    // 输出
    unsigned int bit_position;  // 位位置
} ec_ioctl_reg_pdo_entry_t;

/*****************************************************************************/

typedef struct
{
    // 输入
    uint32_t config_index;  // 配置索引
    uint32_t sync_index;  // 同步索引
    uint32_t pdo_pos;  // PDO位置
    uint32_t entry_pos;  // 条目位置
    uint32_t domain_index;  // 域索引

    // 输出
    unsigned int bit_position;  // 位位置
} ec_ioctl_reg_pdo_pos_t;

/*****************************************************************************/

typedef struct
{
    // 输入
    uint32_t config_index;  // 配置索引
    uint16_t index;  // 索引
    uint8_t subindex;  // 子索引
    const uint8_t *data;  // 数据指针
    size_t size;  // 大小
    uint8_t complete_access;  // 完全访问标志
} ec_ioctl_sc_sdo_t;

/*****************************************************************************/

typedef struct
{
    // 输入
    uint32_t config_index;  // 配置索引
    size_t size;  // 大小
    uint8_t *target;  // 目标指针

    // 输出
    int32_t overruns;  // 超限次数
} ec_ioctl_sc_emerg_t;

/*****************************************************************************/

typedef struct
{
    // 输入
    uint32_t config_index;  // 配置索引

    // 输出
    ec_slave_config_state_t *state;  // 从站配置状态指针
} ec_ioctl_sc_state_t;

/*****************************************************************************/

typedef struct
{
    // 输入
    uint32_t config_index;  // 配置索引
    uint8_t drive_no;  // 驱动器号
    uint16_t idn;  // IDN
    ec_al_state_t al_state;  // AL状态
    const uint8_t *data;  // 数据指针
    size_t size;  // 大小
} ec_ioctl_sc_idn_t;

/*****************************************************************************/

typedef struct
{
    // 输入
    uint32_t domain_index;  // 域索引

    // 输出
    ec_domain_state_t *state;  // 域状态指针
} ec_ioctl_domain_state_t;

/*****************************************************************************/

typedef struct
{
    // 输入
    uint32_t config_index;  // 配置索引

    // 输入/输出
    uint32_t request_index;  // 请求索引
    uint16_t sdo_index;  // SDO索引
    uint8_t sdo_subindex;  // SDO子索引
    uint8_t complete_access;  // 完全访问标志
    size_t size;  // 大小
    uint8_t *data;  // 数据指针
    uint32_t timeout;  // 超时时间
    ec_request_state_t state;  // 请求状态
} ec_ioctl_sdo_request_t;

/*****************************************************************************/

typedef struct
{
    // 输入
    uint32_t config_index;  // 配置索引

    // 输入/输出
    uint32_t request_index;  // 请求索引
    uint32_t password;  // 密码
    size_t size;  // 大小
    size_t progress;  // 进度
    uint8_t *data;  // 数据指针
    uint32_t timeout;  // 超时时间
    ec_request_state_t state;  // 请求状态
    ec_foe_error_t result;  // FOE结果
    uint32_t error_code;  // 错误代码

    char file_name[255];  // 文件名
} ec_ioctl_foe_request_t;

/*****************************************************************************/

typedef struct
{
    // 输入
    uint32_t config_index;  // 配置索引
    size_t mem_size;  // 内存大小

    // 输入/输出
    uint32_t request_index;  // 请求索引
    uint8_t *data;  // 数据指针
    ec_request_state_t state;  // 请求状态
    uint8_t new_data;  // 新数据标志
    uint16_t address;  // 地址
    size_t transfer_size;  // 传输大小
} ec_ioctl_reg_request_t;

/*****************************************************************************/

typedef struct
{
    // 输入
    uint32_t config_index;  // 配置索引

    // 输入/输出
    uint32_t voe_index;  // VOE索引
    uint32_t *vendor_id;  // 厂商ID指针
    uint16_t *vendor_type;  // 厂商类型指针
    size_t size;  // 大小
    uint8_t *data;  // 数据指针
    ec_request_state_t state;  // 请求状态
} ec_ioctl_voe_t;

/*****************************************************************************/

typedef struct
{
    // 输入
    uint32_t dev_idx;  // 设备索引

    // 输出
    ec_master_link_state_t *state;  // 主站链路状态指针
} ec_ioctl_link_state_t;

/*****************************************************************************/

typedef struct
{
    // 输入
    uint16_t slave_position;  // 从站位置
} ec_ioctl_slave_dict_upload_t;

/*****************************************************************************/

typedef struct
{
    // 输入/输出
    size_t data_size;  // 数据大小
    size_t buff_size;  // 缓冲区大小
    uint8_t *data;  // 数据指针
} ec_ioctl_mbox_gateway_t;

/*****************************************************************************/

#ifdef __KERNEL__

/** 文件句柄的上下文数据结构。
 */
typedef struct
{
    unsigned int writable;    /**< 设备以写权限打开。 */
    unsigned int requested;   /**< 主站通过此文件句柄请求。 */
    uint8_t *process_data;    /**< 进程数据区域。 */
    size_t process_data_size; /**< \a process_data 的大小。 */
} ec_ioctl_context_t;

long ec_ioctl(ec_master_t *, ec_ioctl_context_t *, unsigned int,
              void __user *);

#ifdef EC_RTDM

long ec_ioctl_rtdm(ec_master_t *, ec_ioctl_context_t *, unsigned int,
                   void __user *);
int ec_rtdm_mmap(ec_ioctl_context_t *, void **);

#endif

#endif

/*****************************************************************************/

/** \endcond */

#endif

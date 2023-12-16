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
   EtherCAT从站状态机。
*/

/*****************************************************************************/

#include "globals.h"
#include "master.h"
#include "mailbox.h"
#include "sii_firmware.h"
#include "slave_config.h"

#include "fsm_slave_scan.h"

/** 从站扫描重试前等待的时间 [毫秒]。
 *
 * 用于基于节拍计数器计算时间。
 *
 * \attention 必须大于10，以避免在以100 Hz运行的内核上出现问题。
 */
#define SCAN_RETRY_TIME 100

/*****************************************************************************/

void ec_fsm_slave_scan_state_start(ec_fsm_slave_scan_t *, ec_datagram_t *);
void ec_fsm_slave_scan_state_address(ec_fsm_slave_scan_t *, ec_datagram_t *);
void ec_fsm_slave_scan_state_state(ec_fsm_slave_scan_t *, ec_datagram_t *);
void ec_fsm_slave_scan_state_base(ec_fsm_slave_scan_t *, ec_datagram_t *);
void ec_fsm_slave_scan_state_dc_cap(ec_fsm_slave_scan_t *, ec_datagram_t *);
void ec_fsm_slave_scan_state_dc_times(ec_fsm_slave_scan_t *, ec_datagram_t *);
void ec_fsm_slave_scan_state_datalink(ec_fsm_slave_scan_t *, ec_datagram_t *);
#ifdef EC_SII_ASSIGN
void ec_fsm_slave_scan_state_assign_sii(ec_fsm_slave_scan_t *, ec_datagram_t *);
#endif
#ifdef EC_SII_CACHE
void ec_fsm_slave_scan_state_sii_identity(ec_fsm_slave_scan_t *, ec_datagram_t *);
#endif
#ifdef EC_SII_OVERRIDE
void ec_fsm_slave_scan_state_sii_device(ec_fsm_slave_scan_t *, ec_datagram_t *);
void ec_fsm_slave_scan_state_sii_request(ec_fsm_slave_scan_t *, ec_datagram_t *);
#endif
void ec_fsm_slave_scan_state_sii_size(ec_fsm_slave_scan_t *, ec_datagram_t *);
void ec_fsm_slave_scan_state_sii_data(ec_fsm_slave_scan_t *, ec_datagram_t *);
void ec_fsm_slave_scan_state_sii_parse(ec_fsm_slave_scan_t *, ec_datagram_t *);
void ec_fsm_slave_scan_state_mailbox_cleared(ec_fsm_slave_scan_t *, ec_datagram_t *);
#ifdef EC_REGALIAS
void ec_fsm_slave_scan_state_regalias(ec_fsm_slave_scan_t *, ec_datagram_t *);
#endif
void ec_fsm_slave_scan_state_preop(ec_fsm_slave_scan_t *, ec_datagram_t *);
void ec_fsm_slave_scan_state_sync(ec_fsm_slave_scan_t *, ec_datagram_t *);
void ec_fsm_slave_scan_state_pdos(ec_fsm_slave_scan_t *, ec_datagram_t *);

void ec_fsm_slave_scan_state_end(ec_fsm_slave_scan_t *, ec_datagram_t *);
void ec_fsm_slave_scan_state_error(ec_fsm_slave_scan_t *, ec_datagram_t *);
void ec_fsm_slave_scan_state_retry(ec_fsm_slave_scan_t *, ec_datagram_t *);
void ec_fsm_slave_scan_state_retry_wait(ec_fsm_slave_scan_t *, ec_datagram_t *);

void ec_fsm_slave_scan_enter_datalink(ec_fsm_slave_scan_t *, ec_datagram_t *);
#ifdef EC_REGALIAS
void ec_fsm_slave_scan_enter_regalias(ec_fsm_slave_scan_t *, ec_datagram_t *);
#endif
#ifdef EC_SII_CACHE
void ec_fsm_slave_scan_enter_sii_identity(ec_fsm_slave_scan_t *, ec_datagram_t *);
#endif
#ifdef EC_SII_OVERRIDE
void ec_fsm_slave_scan_enter_sii_request(ec_fsm_slave_scan_t *, ec_datagram_t *);
#endif
void ec_fsm_slave_scan_enter_attach_sii(ec_fsm_slave_scan_t *, ec_datagram_t *);
void ec_fsm_slave_scan_enter_sii_size(ec_fsm_slave_scan_t *, ec_datagram_t *);
void ec_fsm_slave_scan_enter_preop(ec_fsm_slave_scan_t *, ec_datagram_t *);
void ec_fsm_slave_scan_enter_clear_mailbox(ec_fsm_slave_scan_t *, ec_datagram_t *);
void ec_fsm_slave_scan_enter_pdos(ec_fsm_slave_scan_t *, ec_datagram_t *);

/*****************************************************************************/

/**
@brief 构造函数。初始化Slave扫描状态机。
@param fsm Slave扫描状态机。
@param slave 要配置的从站。
@param fsm_slave_config 用于使用的从站配置状态机。
@param fsm_pdo 用于使用的PDO配置机。
@return 无
@details
该函数用于初始化Slave扫描状态机。它接受以下参数：
- `fsm`：Slave扫描状态机。
- `slave`：要配置的从站。
- `fsm_slave_config`：要使用的从站配置状态机。
- `fsm_pdo`：要使用的PDO配置机。

在函数内部，它执行以下操作：
- 将`slave`赋值给`fsm->slave`。
- 将`NULL`赋值给`fsm->datagram`。
- 将`fsm_slave_config`赋值给`fsm->fsm_slave_config`。
- 将`fsm_pdo`赋值给`fsm->fsm_pdo`。
- 初始化子状态机`ec_fsm_sii`。

注意：该函数是构造函数，用于初始化Slave扫描状态机的实例。
*/

void ec_fsm_slave_scan_init(
    ec_fsm_slave_scan_t *fsm,                /**< Slave扫描状态机。 */
    ec_slave_t *slave,                       /**< 要配置的从站 */
    ec_fsm_slave_config_t *fsm_slave_config, /**< 要使用的从站配置状态机。 */
    ec_fsm_pdo_t *fsm_pdo                    /**< 要使用的PDO配置机。 */
)
{
    fsm->slave = slave;
    fsm->datagram = NULL;
    fsm->fsm_slave_config = fsm_slave_config;
    fsm->fsm_pdo = fsm_pdo;

    // 初始化子状态机
    ec_fsm_sii_init(&fsm->fsm_sii);
}

/*****************************************************************************/

/**
@brief 析构函数。清除Slave扫描状态机。
@param fsm Slave状态机。
@return 无
@details
该函数用于清除Slave扫描状态机。它接受以下参数：
- `fsm`：Slave状态机。

在函数内部，它执行以下操作：
- 清除子状态机`ec_fsm_sii`。

注意：该函数是析构函数，用于清除Slave扫描状态机的实例。
*/

void ec_fsm_slave_scan_clear(ec_fsm_slave_scan_t *fsm /**< Slave状态机 */)
{
    // 清除子状态机
    ec_fsm_sii_clear(&fsm->fsm_sii);
}

/*****************************************************************************/

/**
@brief 启动Slave扫描状态机。
@param fsm Slave状态机。
@return 无
@details
该函数用于启动Slave扫描状态机。它接受以下参数：
- `fsm`：Slave状态机。

在函数内部，它执行以下操作：
- 将`EC_FSM_RETRIES`赋值给`fsm->scan_retries`。
- 将`ec_fsm_slave_scan_state_start`赋值给`fsm->state`。

注意：该函数用于启动Slave扫描状态机的执行。
*/

void ec_fsm_slave_scan_start(
    ec_fsm_slave_scan_t *fsm /**< Slave状态机 */
)
{
    fsm->scan_retries = EC_FSM_RETRIES;
    fsm->state = ec_fsm_slave_scan_state_start;
}

/*****************************************************************************/

/**
@brief 检查Slave扫描状态机是否正在运行。
@param fsm Slave状态机。
@return 返回值为0表示状态机已终止，否则返回非零值。
@details
该函数用于检查Slave扫描状态机是否正在运行。它接受以下参数：
- `fsm`：Slave状态机。

函数内部执行以下操作：
- 检查`fsm->state`是否等于`ec_fsm_slave_scan_state_end`或`ec_fsm_slave_scan_state_error`。
- 如果是，则返回0表示状态机已终止。
- 如果不是，则返回非零值表示状态机正在运行。

注意：该函数用于检查Slave扫描状态机是否仍在执行。
*/

int ec_fsm_slave_scan_running(const ec_fsm_slave_scan_t *fsm /**< Slave状态机 */)
{
    return fsm->state != ec_fsm_slave_scan_state_end && fsm->state != ec_fsm_slave_scan_state_error;
}

/*****************************************************************************/

/**
@brief 执行Slave扫描状态机的当前状态。
@param fsm Slave状态机。
@param datagram 要使用的数据报文。
@return 返回值为0表示状态机已终止，否则返回非零值。
@details
该函数用于执行Slave扫描状态机的当前状态。它接受以下参数：
- `fsm`：Slave状态机。
- `datagram`：要使用的数据报文。

函数内部执行以下操作：
- 检查状态机是否已终止，如果是，则直接返回0。
- 调用`fsm->state`函数，传递`fsm`和`datagram`作为参数，执行当前状态的逻辑。
- 再次检查状态机是否已终止，如果是，则将`fsm->datagram`设置为`NULL`并返回0。
- 否则，将`fsm->datagram`设置为`datagram`并返回非零值。

注意：该函数用于执行Slave扫描状态机的当前状态，并在适当时延迟执行。
*/

int ec_fsm_slave_scan_exec(
    ec_fsm_slave_scan_t *fsm, /**< Slave状态机 */
    ec_datagram_t *datagram   /**< 要使用的数据报文。 */
)
{
    if (!ec_fsm_slave_scan_running(fsm))
        return 0;

    fsm->state(fsm, datagram);

    if (!ec_fsm_slave_scan_running(fsm))
    {
        fsm->datagram = NULL;
        return 0;
    }

    fsm->datagram = datagram;
    return 1;
}

/*****************************************************************************/

/**
@brief 检查Slave扫描状态机是否正常终止。
@param fsm Slave状态机。
@return 返回值为非零值表示状态机正常终止，否则返回0。
@details
该函数用于检查Slave扫描状态机是否正常终止。它接受以下参数：
- `fsm`：Slave状态机。

函数内部执行以下操作：
- 检查`fsm->state`是否等于`ec_fsm_slave_scan_state_end`。
- 如果是，则返回非零值表示状态机正常终止。
- 如果不是，则返回0表示状态机未正常终止。

注意：该函数用于检查Slave扫描状态机是否正常终止。
*/

int ec_fsm_slave_scan_success(const ec_fsm_slave_scan_t *fsm /**< Slave状态机 */)
{
    return fsm->state == ec_fsm_slave_scan_state_end;
}

/******************************************************************************
 *  Slave扫描状态机
 *****************************************************************************/

/**
@brief Slave扫描状态：START。
@param fsm Slave状态机。
@param datagram 要使用的数据报文。
@return 无
@details
该函数是Slave扫描状态机的第一个状态。它用于将站地址写入从站，根据从站的环位置。

函数内部执行以下操作：
- 使用`ec_datagram_apwr`函数，将从站的环位置作为参数，写入站地址的位置（0x0010）。
- 使用`EC_WRITE_U16`宏，将从站的站地址写入`datagram->data`。
- 将`EC_FSM_RETRIES`赋值给`fsm->retries`。
- 将`ec_fsm_slave_scan_state_address`赋值给`fsm->state`。

注意：该函数是Slave扫描状态机的第一个状态，用于配置从站的站地址。
*/

void ec_fsm_slave_scan_state_start(
    ec_fsm_slave_scan_t *fsm, /**< Slave状态机 */
    ec_datagram_t *datagram   /**< 要使用的数据报文。 */
)
{
    // 写入站地址
    ec_datagram_apwr(datagram, fsm->slave->ring_position, 0x0010, 2);
    EC_WRITE_U16(datagram->data, fsm->slave->station_address);
    fsm->retries = EC_FSM_RETRIES;
    fsm->state = ec_fsm_slave_scan_state_address;
}

/*****************************************************************************/

/**
@brief Slave扫描状态：ADDRESS。
@param fsm Slave状态机。
@param datagram 要使用的数据报文。
@return 无
@details
该函数是Slave扫描状态机的状态之一。

函数内部执行以下操作：
- 检查`fsm->datagram->state`是否等于`EC_DATAGRAM_TIMED_OUT`并且`fsm->retries--`。
  - 如果是，则使用`ec_datagram_repeat`函数，将`datagram`复制到`fsm->datagram`。
  - 然后返回。
- 检查`fsm->datagram->state`是否不等于`EC_DATAGRAM_RECEIVED`。
  - 如果是，则将`fsm->state`设置为`ec_fsm_slave_scan_state_error`，并在错误日志中打印无法接收到站地址数据报文的信息。
  - 然后返回。
- 检查`fsm->datagram->working_counter`是否不等于1。
  - 如果是，则将`fsm->slave->error_flag`设置为1，将`fsm->state`设置为`ec_fsm_slave_scan_state_error`，并在错误日志中打印无法写入站地址的信息。
  - 然后返回。

- 使用`ec_datagram_fprd`函数，读取AL状态。
- 使用`ec_datagram_zero`函数，将`datagram`清零。
- 将`EC_FSM_RETRIES`赋值给`fsm->retries`。
- 将`ec_fsm_slave_scan_state_state`赋值给`fsm->state`。

注意：该函数是Slave扫描状态机的状态之一，用于处理从站地址的接收和写入。
*/

void ec_fsm_slave_scan_state_address(
    ec_fsm_slave_scan_t *fsm, /**< Slave状态机 */
    ec_datagram_t *datagram   /**< 要使用的数据报文。 */
)
{
    if (fsm->datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--)
    {
        ec_datagram_repeat(datagram, fsm->datagram);
        return;
    }

    if (fsm->datagram->state != EC_DATAGRAM_RECEIVED)
    {
        fsm->state = ec_fsm_slave_scan_state_error;
        EC_SLAVE_ERR(fsm->slave,
                     "无法接收到站地址数据报文：");
        ec_datagram_print_state(fsm->datagram);
        return;
    }

    if (fsm->datagram->working_counter != 1)
    {
        fsm->slave->error_flag = 1;
        fsm->state = ec_fsm_slave_scan_state_error;
        EC_SLAVE_ERR(fsm->slave, "无法写入站地址：");
        ec_datagram_print_wc_error(fsm->datagram);
        return;
    }

    // 读取AL状态
    ec_datagram_fprd(datagram, fsm->slave->station_address, 0x0130, 2);
    ec_datagram_zero(datagram);
    fsm->retries = EC_FSM_RETRIES;
    fsm->state = ec_fsm_slave_scan_state_state;
}

/*****************************************************************************/

/**
@brief Slave扫描状态：STATE。
@param fsm Slave状态机。
@param datagram 要使用的数据报文。
@return 无
@details
该函数是Slave扫描状态机的状态之一。

函数内部执行以下操作：
- 检查`fsm->datagram->state`是否等于`EC_DATAGRAM_TIMED_OUT`并且`fsm->retries--`。
  - 如果是，则使用`ec_datagram_repeat`函数，将`datagram`复制到`fsm->datagram`。
  - 然后返回。
- 检查`fsm->datagram->state`是否不等于`EC_DATAGRAM_RECEIVED`。
  - 如果是，则将`fsm->state`设置为`ec_fsm_slave_scan_state_error`，并在错误日志中打印无法接收到AL状态数据报文的信息。
  - 然后返回。
- 检查`fsm->datagram->working_counter`是否不等于1。
  - 如果是，则将`fsm->slave->error_flag`设置为1，将`fsm->state`设置为`ec_fsm_slave_scan_state_error`，并在错误日志中打印无法读取AL状态的信息。
  - 然后返回。

- 使用`EC_READ_U8`宏，从`fsm->datagram->data`读取当前状态并将其赋值给`slave->current_state`。
- 如果`slave->current_state`的`EC_SLAVE_STATE_ACK_ERR`位被设置，
  - 则使用`ec_state_string`函数，将`slave->current_state`转换为字符串形式，并将其打印为警告日志。
- 使用`ec_datagram_fprd`函数，读取基本数据。
- 使用`ec_datagram_zero`函数，将`datagram`清零。
- 将`EC_FSM_RETRIES`赋值给`fsm->retries`。
- 将`ec_fsm_slave_scan_state_base`赋值给`fsm->state`。

注意：该函数是Slave扫描状态机的状态之一，用于处理AL状态的接收和读取基本数据。
*/

void ec_fsm_slave_scan_state_state(
    ec_fsm_slave_scan_t *fsm, /**< Slave状态机 */
    ec_datagram_t *datagram   /**< 要使用的数据报文。 */
)
{
    ec_slave_t *slave = fsm->slave;

    if (fsm->datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--)
    {
        ec_datagram_repeat(datagram, fsm->datagram);
        return;
    }

    if (fsm->datagram->state != EC_DATAGRAM_RECEIVED)
    {
        fsm->state = ec_fsm_slave_scan_state_error;
        EC_SLAVE_ERR(slave, "无法接收到AL状态数据报文：");
        ec_datagram_print_state(fsm->datagram);
        return;
    }

    if (fsm->datagram->working_counter != 1)
    {
        fsm->slave->error_flag = 1;
        fsm->state = ec_fsm_slave_scan_state_error;
        EC_SLAVE_ERR(slave, "无法读取AL状态：");
        ec_datagram_print_wc_error(fsm->datagram);
        return;
    }

    slave->current_state = EC_READ_U8(fsm->datagram->data);
    if (slave->current_state & EC_SLAVE_STATE_ACK_ERR)
    {
        char state_str[EC_STATE_STRING_SIZE];
        ec_state_string(slave->current_state, state_str, 0);
        EC_SLAVE_WARN(slave, "从站设置了状态错误位 (%s)！\n",
                      state_str);
    }

    // 读取基本数据
    ec_datagram_fprd(datagram, fsm->slave->station_address, 0x0000, 12);
    ec_datagram_zero(datagram);
    fsm->retries = EC_FSM_RETRIES;
    fsm->state = ec_fsm_slave_scan_state_base;
}

/*****************************************************************************/

/**
@brief Slave扫描状态：BASE。
@param fsm Slave状态机。
@param datagram 要使用的数据报文。
@return 无
@details
该函数是Slave扫描状态机的状态之一。

函数内部执行以下操作：
- 检查`fsm->datagram->state`是否等于`EC_DATAGRAM_TIMED_OUT`并且`fsm->retries--`。
  - 如果是，则使用`ec_datagram_repeat`函数，将`datagram`复制到`fsm->datagram`。
  - 然后返回。
- 检查`fsm->datagram->state`是否不等于`EC_DATAGRAM_RECEIVED`。
  - 如果是，则将`fsm->state`设置为`ec_fsm_slave_scan_state_error`，并在错误日志中打印无法接收到基本数据数据报文的信息。
  - 然后返回。
- 检查`fsm->datagram->working_counter`是否不等于1。
  - 如果是，则将`fsm->slave->error_flag`设置为1，将`fsm->state`设置为`ec_fsm_slave_scan_state_error`，并在错误日志中打印无法读取基本数据的信息。
  - 然后返回。

- 使用`EC_READ_U8`宏，从`fsm->datagram->data`读取基本类型并将其赋值给`slave->base_type`。
- 使用`EC_READ_U8`宏，从`fsm->datagram->data + 1`读取基本修订版本并将其赋值给`slave->base_revision`。
- 使用`EC_READ_U16`宏，从`fsm->datagram->data + 2`读取基本构建版本并将其赋值给`slave->base_build`。

- 使用`EC_READ_U8`宏，从`fsm->datagram->data + 4`读取基本FMMU计数并将其赋值给`slave->base_fmmu_count`。
  - 如果`slave->base_fmmu_count`大于`EC_MAX_FMMUS`，
    - 则将`EC_MAX_FMMUS`赋值给`slave->base_fmmu_count`，并在警告日志中打印从站的FMMU计数超过主站能处理的信息。

- 使用`EC_READ_U8`宏，从`fsm->datagram->data + 5`读取基本同步管理器计数并将其赋值给`slave->base_sync_count`。
  - 如果`slave->base_sync_count`大于`EC_MAX_SYNC_MANAGERS`，
    - 则将`EC_MAX_SYNC_MANAGERS`赋值给`slave->base_sync_count`，并在警告日志中打印从站提供的同步管理器数量超过主站能处理的信息。

- 使用`EC_READ_U8`宏，从`fsm->datagram->data + 7`读取一个字节的数据，并将其赋值给`octet`。
  - 对于每个端口$i$，
    - 将`(octet >> (2 * i)) & 0x03`赋值给`slave->ports[i].desc`。

- 使用`EC_READ_U8`宏，从`fsm->datagram->data + 8`读取一个字节的数据，并将其赋值给`octet`。
  - 将`octet & 0x01`赋值给`slave->base_fmmu_bit_operation`。
  - 将`(octet >> 2) & 0x01`赋值给`slave->base_dc_supported`。
  - 如果`octet >> 3`的值为1，
    - 则将`EC_DC_64`赋值给`slave->base_dc_range`。
  - 否则，
    - 将`EC_DC_32`赋值给`slave->base_dc_range`。

- 如果`slave->base_dc_supported`为真，
  - 则使用`ec_datagram_fprd`函数，读取DC能力。
  - 使用`ec_datagram_zero`函数，将`datagram`清零。
  - 将`EC_FSM_RETRIES`赋值给`fsm->retries`。
  - 将`ec_fsm_slave_scan_state_dc_cap`赋值给`fsm->state`。
- 否则，
  - 调用`ec_fsm_slave_scan_enter_datalink`函数，传递`fsm`和`datagram`作为参数。

注意：该函数是Slave扫描状态机的状态之一，用于处理基本数据的接收和读取。
*/

void ec_fsm_slave_scan_state_base(
    ec_fsm_slave_scan_t *fsm, /**< Slave状态机 */
    ec_datagram_t *datagram   /**< 要使用的数据报文。 */
)
{
    ec_slave_t *slave = fsm->slave;
    u8 octet;
    int i;

    if (fsm->datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--)
    {
        ec_datagram_repeat(datagram, fsm->datagram);
        return;
    }

    if (fsm->datagram->state != EC_DATAGRAM_RECEIVED)
    {
        fsm->state = ec_fsm_slave_scan_state_error;
        EC_SLAVE_ERR(slave, "无法接收到基本数据数据报文：");
        ec_datagram_print_state(fsm->datagram);
        return;
    }

    if (fsm->datagram->working_counter != 1)
    {
        fsm->slave->error_flag = 1;
        fsm->state = ec_fsm_slave_scan_state_error;
        EC_SLAVE_ERR(slave, "无法读取基本数据：");
        ec_datagram_print_wc_error(fsm->datagram);
        return;
    }

    slave->base_type = EC_READ_U8(fsm->datagram->data);
    slave->base_revision = EC_READ_U8(fsm->datagram->data + 1);
    slave->base_build = EC_READ_U16(fsm->datagram->data + 2);

    slave->base_fmmu_count = EC_READ_U8(fsm->datagram->data + 4);
    if (slave->base_fmmu_count > EC_MAX_FMMUS)
    {
        EC_SLAVE_WARN(slave, "从站的FMMU计数（%u）超过主站能处理的数量（%u）。\n",
                      slave->base_fmmu_count, EC_MAX_FMMUS);
        slave->base_fmmu_count = EC_MAX_FMMUS;
    }

    slave->base_sync_count = EC_READ_U8(fsm->datagram->data + 5);
    if (slave->base_sync_count > EC_MAX_SYNC_MANAGERS)
    {
        EC_SLAVE_WARN(slave, "从站提供的同步管理器数量（%u）超过主站能处理的数量（%u）。\n",
                      slave->base_sync_count, EC_MAX_SYNC_MANAGERS);
        slave->base_sync_count = EC_MAX_SYNC_MANAGERS;
    }

    octet = EC_READ_U8(fsm->datagram->data + 7);
    for (i = 0; i < EC_MAX_PORTS; i++)
    {
        slave->ports[i].desc = (octet >> (2 * i)) & 0x03;
    }

    octet = EC_READ_U8(fsm->datagram->data + 8);
    slave->base_fmmu_bit_operation = octet & 0x01;
    slave->base_dc_supported = (octet >> 2) & 0x01;
    slave->base_dc_range = ((octet >> 3) & 0x01) ? EC_DC_64 : EC_DC_32;

    if (slave->base_dc_supported)
    {
        // 读取DC能力
        ec_datagram_fprd(datagram, slave->station_address, 0x0910,
                         slave->base_dc_range == EC_DC_64 ? 8 : 4);
        ec_datagram_zero(datagram);
        fsm->retries = EC_FSM_RETRIES;
        fsm->state = ec_fsm_slave_scan_state_dc_cap;
    }
    else
    {
        ec_fsm_slave_scan_enter_datalink(fsm, datagram);
    }
}

/*****************************************************************************/

/**
@brief Slave扫描状态：DC CAPABILITIES。
@param fsm Slave状态机。
@param datagram 要使用的数据报文。
@return 无
@details
该函数是Slave扫描状态机的状态之一。

函数内部执行以下操作：
- 检查`fsm->datagram->state`是否等于`EC_DATAGRAM_TIMED_OUT`并且`fsm->retries--`。
  - 如果是，则使用`ec_datagram_repeat`函数，将`datagram`复制到`fsm->datagram`。
  - 然后返回。
- 检查`fsm->datagram->state`是否不等于`EC_DATAGRAM_RECEIVED`。
  - 如果是，则将`fsm->state`设置为`ec_fsm_slave_scan_state_error`，并在错误日志中打印无法接收到系统时间数据报文的信息。
  - 然后返回。

- 检查`fsm->datagram->working_counter`的值。
  - 如果为1，
    - 则将`slave->has_dc_system_time`设置为1，并在调试日志中打印从站具有系统时间寄存器的信息。
  - 否则，如果为0，
    - 则在调试日志中打印从站没有系统时间寄存器的信息。
  - 否则，
    - 将`fsm->slave->error_flag`设置为1，将`fsm->state`设置为`ec_fsm_slave_scan_state_error`，并在错误日志中打印无法确定系统时间寄存器是否支持的信息。

- 使用`ec_datagram_fprd`函数，读取DC端口的接收时间。
- 使用`ec_datagram_zero`函数，将`datagram`清零。
- 将`EC_FSM_RETRIES`赋值给`fsm->retries`。
- 将`ec_fsm_slave_scan_state_dc_times`赋值给`fsm->state`。

注意：该函数是Slave扫描状态机的状态之一，用于处理DC能力的接收和读取DC端口的接收时间。
*/

void ec_fsm_slave_scan_state_dc_cap(
    ec_fsm_slave_scan_t *fsm, /**< Slave状态机 */
    ec_datagram_t *datagram   /**< 要使用的数据报文。 */
)
{
    ec_slave_t *slave = fsm->slave;

    if (fsm->datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--)
    {
        ec_datagram_repeat(datagram, fsm->datagram);
        return;
    }

    if (fsm->datagram->state != EC_DATAGRAM_RECEIVED)
    {
        fsm->state = ec_fsm_slave_scan_state_error;
        EC_SLAVE_ERR(slave, "无法接收到系统时间数据报文：");
        ec_datagram_print_state(fsm->datagram);
        return;
    }

    if (fsm->datagram->working_counter == 1)
    {
        slave->has_dc_system_time = 1;
        EC_SLAVE_DBG(slave, 1, "从站具有系统时间寄存器。\n");
    }
    else if (fsm->datagram->working_counter == 0)
    {
        EC_SLAVE_DBG(slave, 1, "从站没有系统时间寄存器；仅进行延迟测量。\n");
    }
    else
    {
        fsm->slave->error_flag = 1;
        fsm->state = ec_fsm_slave_scan_state_error;
        EC_SLAVE_ERR(slave, "无法确定系统时间寄存器是否支持：");
        ec_datagram_print_wc_error(fsm->datagram);
        return;
    }

    // 读取DC端口的接收时间
    ec_datagram_fprd(datagram, slave->station_address, 0x0900, 16);
    ec_datagram_zero(datagram);
    fsm->retries = EC_FSM_RETRIES;
    fsm->state = ec_fsm_slave_scan_state_dc_times;
}

/*****************************************************************************/

/**
@brief Slave扫描状态：DC TIMES。
@param fsm Slave状态机。
@param datagram 要使用的数据报文。
@return 无
@details
该函数是Slave扫描状态机的状态之一。

函数内部执行以下操作：
- 检查`fsm->datagram->state`是否等于`EC_DATAGRAM_TIMED_OUT`并且`fsm->retries--`。
  - 如果是，则使用`ec_datagram_repeat`函数，将`datagram`复制到`fsm->datagram`。
  - 然后返回。
- 检查`fsm->datagram->state`是否不等于`EC_DATAGRAM_RECEIVED`。
  - 如果是，则将`fsm->state`设置为`ec_fsm_slave_scan_state_error`，并在错误日志中打印无法接收到系统时间数据报文的信息。
  - 然后返回。
- 检查`fsm->datagram->working_counter`是否不等于1。
  - 如果是，则将`fsm->slave->error_flag`设置为1，将`fsm->state`设置为`ec_fsm_slave_scan_state_error`，并在错误日志中打印无法获取DC接收时间的信息。
  - 然后返回。

- 对于每个端口$i$，
  - 使用`EC_READ_U32`宏，从`fsm->datagram->data + 4 * i`读取一个32位的数据，并将其赋值给`new_time`。
  - 如果`new_time`等于`slave->ports[i].receive_time`，
    - 则表示自初始扫描以来时间没有改变；该端口尚未处理广播定时数据报文。
    - 这可能发生在某些冗余方案中，也可能发生在端口关闭的情况下，所以在这个阶段我们无法确定是否是一个问题。
    - 将`slave->ports[i].link.bypassed`设置为1。
  - 将`new_time`赋值给`slave->ports[i].receive_time`。

- 调用`ec_fsm_slave_scan_enter_datalink`函数，传递`fsm`和`datagram`作为参数。

注意：该函数是Slave扫描状态机的状态之一，用于处理DC端口的接收时间。
*/

void ec_fsm_slave_scan_state_dc_times(
    ec_fsm_slave_scan_t *fsm, /**< Slave状态机 */
    ec_datagram_t *datagram   /**< 要使用的数据报文。 */
)
{
    ec_slave_t *slave = fsm->slave;
    int i;

    if (fsm->datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--)
    {
        ec_datagram_repeat(datagram, fsm->datagram);
        return;
    }

    if (fsm->datagram->state != EC_DATAGRAM_RECEIVED)
    {
        fsm->state = ec_fsm_slave_scan_state_error;
        EC_SLAVE_ERR(slave, "无法接收到系统时间数据报文：");
        ec_datagram_print_state(fsm->datagram);
        return;
    }

    if (fsm->datagram->working_counter != 1)
    {
        fsm->slave->error_flag = 1;
        fsm->state = ec_fsm_slave_scan_state_error;
        EC_SLAVE_ERR(slave, "无法获取DC接收时间：");
        ec_datagram_print_wc_error(fsm->datagram);
        return;
    }

    for (i = 0; i < EC_MAX_PORTS; i++)
    {
        u32 new_time = EC_READ_U32(fsm->datagram->data + 4 * i);
        if (new_time == slave->ports[i].receive_time)
        {
            // 时间自初始扫描以来没有改变；该端口尚未处理广播定时数据报文。
            // 这可能发生在某些冗余方案中，也可能发生在端口关闭的情况下，所以在这个阶段我们无法确定是否是一个问题。
            slave->ports[i].link.bypassed = 1;
        }
        slave->ports[i].receive_time = new_time;
    }

    ec_fsm_slave_scan_enter_datalink(fsm, datagram);
}

/*****************************************************************************/

/**
 * @brief 进入从站扫描状态：数据链路。
 * @param fsm 从站状态机。
 * @param datagram 用于使用的数据报。
 * @return 无返回值。
 * @details
 * - 读取数据链路状态。
 * - 清零数据报。
 * - 设置重试次数为默认值。
 * - 将状态设置为数据链路状态。
 */

void ec_fsm_slave_scan_enter_datalink(
    ec_fsm_slave_scan_t *fsm, /**< 从站状态机 */
    ec_datagram_t *datagram   /**< 用于使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;

    // 读取数据链路状态
    ec_datagram_fprd(datagram, slave->station_address, 0x0110, 2);
    ec_datagram_zero(datagram);
    fsm->retries = EC_FSM_RETRIES;
    fsm->state = ec_fsm_slave_scan_state_datalink;
}

#ifdef EC_SII_CACHE
/*****************************************************************************/

/** 进入从站扫描状态：SII_IDENTITY。
 */
void ec_fsm_slave_scan_enter_sii_identity(
    ec_fsm_slave_scan_t *fsm, /**< 从站状态机 */
    ec_datagram_t *datagram   /**< 用于使用的数据报 */
)
{
    // 开始获取SII序列号
    fsm->sii_offset = EC_ALIAS_SII_OFFSET;
    ec_fsm_sii_read(&fsm->fsm_sii, fsm->slave, fsm->sii_offset,
                    EC_FSM_SII_USE_CONFIGURED_ADDRESS);
    fsm->state = ec_fsm_slave_scan_state_sii_identity;
    fsm->state(fsm, datagram); // 立即执行状态
}
#endif


/*****************************************************************************/

/**
@brief 进入从站扫描状态 ATTACH_SII。
@param fsm 从站状态机。
@param datagram 使用的数据报。
@return 无。
@details
- 检查从站是否可以重用存储的 SII 图像数据。
- 如果可以重用，则更新从站的相关信息，并进入 PREOP 状态。
- 如果不能重用，则创建从站的 SII 图像数据，并进入 SII_SIZE 状态。
*/

void ec_fsm_slave_scan_enter_attach_sii(
    ec_fsm_slave_scan_t *fsm, /**< 从站状态机 */
    ec_datagram_t *datagram   /**< 使用的数据报 */
)
{
    ec_sii_image_t *sii_image;
    ec_slave_t *slave = fsm->slave;

#ifdef EC_SII_CACHE
    unsigned int i = 0;
    unsigned int found = 0;

    if ((slave->effective_alias != 0) || (slave->effective_serial_number != 0))
    {
        list_for_each_entry(sii_image, &slave->master->sii_images, list)
        {
            // 检查从站是否与存储的具有别名、序列号、厂商ID和产品代码的 SII 图像匹配。
            if ((slave->effective_alias != 0) &&
                (slave->effective_alias == sii_image->sii.alias) &&
                (slave->effective_revision_number == sii_image->sii.revision_number))
            {
                EC_SLAVE_DBG(slave, 1, "从站可以重用已存储的 SII 图像数据。"
                                       " 通过别名 %u 进行识别。\n",
                             (uint32_t)slave->effective_alias);
                found = 1;
                break;
            }
            else if ((slave->effective_vendor_id == sii_image->sii.vendor_id) &&
                     (slave->effective_product_code == sii_image->sii.product_code) &&
                     (slave->effective_revision_number == sii_image->sii.revision_number) &&
                     (slave->effective_serial_number == sii_image->sii.serial_number))
            {
                EC_SLAVE_DBG(slave, 1, "从站可以重用已存储的 SII 图像数据。"
                                       " 通过厂商ID 0x%08x、"
                                       " 产品代码 0x%08x、修订号 0x%08x 和序列号 0x%08x 进行识别。\n",
                             slave->effective_vendor_id,
                             slave->effective_product_code,
                             slave->effective_revision_number,
                             slave->effective_serial_number);
                found = 1;
                break;
            }
        }
    }
    else
    {
        EC_SLAVE_DBG(slave, 1, "无法唯一识别从站。"
                               "无法重用 SII 图像数据！\n");
    }

    if (found)
    {
        // 在从站初始化期间丢失的从站引用进行更新
        slave->effective_vendor_id = sii_image->sii.vendor_id;
        slave->effective_product_code = sii_image->sii.product_code;
        slave->effective_revision_number = sii_image->sii.revision_number;
        slave->effective_serial_number = sii_image->sii.serial_number;
        slave->sii_image = sii_image;
        for (i = 0; i < slave->sii_image->sii.sync_count; i++)
        {
            slave->sii_image->sii.syncs[i].slave = slave;
        }
        // SII 图像数据已经可用，可以进入 PREOP 状态
#ifdef EC_REGALIAS
        ec_fsm_slave_scan_enter_regalias(fsm, datagram);
#else
        if (slave->sii_image->sii.mailbox_protocols & EC_MBOX_COE)
        {
            ec_fsm_slave_scan_enter_preop(fsm, datagram);
        }
        else
        {
            fsm->state = ec_fsm_slave_scan_state_end;
        }
#endif
    }
    else
#endif
    {
        EC_MASTER_DBG(slave->master, 1, "为从站 %u 创建 SII 图像\n",
                      fsm->slave->ring_position);

        if (!(sii_image = (ec_sii_image_t *)kmalloc(sizeof(ec_sii_image_t),
                                                    GFP_KERNEL)))
        {
            fsm->state = ec_fsm_slave_scan_state_error;
            EC_MASTER_ERR(fsm->slave->master, "无法为从站 SII 图像分配内存。\n");
            return;
        }
        // 初始化 SII 图像数据
        ec_slave_sii_image_init(sii_image);
        // 将 SII 图像附加到从站
        slave->sii_image = sii_image;
        // 将 SII 图像存储以供以后重用
        list_add_tail(&sii_image->list, &slave->master->sii_images);

        ec_fsm_slave_scan_enter_sii_size(fsm, datagram);
    }
}

/*****************************************************************************/

/**
@brief 进入从站扫描状态 SII_SIZE。
@param fsm 从站状态机。
@param datagram 使用的数据报。
@return 无。
@details
- 开始获取 SII 大小。
*/

void ec_fsm_slave_scan_enter_sii_size(
    ec_fsm_slave_scan_t *fsm, /**< 从站状态机 */
    ec_datagram_t *datagram   /**< 使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;

#ifdef EC_SII_OVERRIDE
    if (!slave->vendor_words)
    {
        if (!(slave->vendor_words =
                  (uint16_t *)kmalloc(32, GFP_KERNEL)))
        {
            EC_SLAVE_ERR(slave, "无法分配 16 个字的 SII 数据的内存。\n");
            slave->error_flag = 1;
            fsm->state = ec_fsm_slave_scan_state_error;
            return;
        }
    }

    // 开始获取设备标识
    fsm->sii_offset = 0;
    fsm->state = ec_fsm_slave_scan_state_sii_device;
#else
    // 开始获取 SII 大小
    fsm->sii_offset = EC_FIRST_SII_CATEGORY_OFFSET; // 第一个类别头
    fsm->state = ec_fsm_slave_scan_state_sii_size;
#endif

    ec_fsm_sii_read(&fsm->fsm_sii, slave, fsm->sii_offset,
                    EC_FSM_SII_USE_CONFIGURED_ADDRESS);

    fsm->state(fsm, datagram); // 立即执行状态
}

/*****************************************************************************/

#ifdef EC_SII_ASSIGN

/**
@brief 进入从站扫描状态 ASSIGN_SII。
@param fsm 从站状态机。
@param datagram 使用的数据报。
@return 无。
@details
- 将 SII 分配给 EtherCAT。
*/

void ec_fsm_slave_scan_enter_assign_sii(
    ec_fsm_slave_scan_t *fsm, /**< 从站状态机 */
    ec_datagram_t *datagram   /**< 使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;

    EC_SLAVE_DBG(slave, 1, "将 SII 访问权限分配给 EtherCAT。\n");

    // 将 SII 分配给 ECAT
    ec_datagram_fpwr(datagram, slave->station_address, 0x0500, 1);
    EC_WRITE_U8(datagram->data, 0x00); // EtherCAT
    fsm->retries = EC_FSM_RETRIES;
    fsm->state = ec_fsm_slave_scan_state_assign_sii;
}

#endif

/*****************************************************************************/

/**
@brief ：从数据链路层接收从站扫描状态。
@param fsm ：从站状态机
@param datagram ：要使用的数据报
@return ：无
@details ：该函数用于从数据链路层接收从站的扫描状态，并根据接收到的状态进行相应的处理。
           函数逻辑如下：
           1. 如果数据报的状态为超时且重试次数未达到上限，则重复发送数据报并返回。
           2. 如果数据报的状态不是接收到，则将状态机的状态设置为错误，并打印错误信息和数据报的状态，并返回。
           3. 如果数据报的工作计数器不等于1，则将从站的错误标志设置为1，并将状态机的状态设置为错误，并打印错误信息和数据报的工作计数器错误，并返回。
           4. 设置从站的数据链路状态为数据报中的数据。
           5. 根据编译选项选择不同的处理方式：
              - 如果定义了EC_SII_ASSIGN，则调用ec_fsm_slave_scan_enter_assign_sii函数进入分配SII状态。
              - 如果定义了EC_SII_CACHE，则调用ec_fsm_slave_scan_enter_sii_identity函数进入SII标识状态。
              - 否则，调用ec_fsm_slave_scan_enter_attach_sii函数进入附加SII状态。
*/

void ec_fsm_slave_scan_state_datalink(
    ec_fsm_slave_scan_t *fsm, /**< 从站状态机 */
    ec_datagram_t *datagram   /**< 要使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;

    if (fsm->datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--)
    {
        ec_datagram_repeat(datagram, fsm->datagram);
        return;
    }

    if (fsm->datagram->state != EC_DATAGRAM_RECEIVED)
    {
        fsm->state = ec_fsm_slave_scan_state_error;
        EC_SLAVE_ERR(slave, "无法接收到数据链路层状态数据报：");
        ec_datagram_print_state(fsm->datagram);
        return;
    }

    if (fsm->datagram->working_counter != 1)
    {
        fsm->slave->error_flag = 1;
        fsm->state = ec_fsm_slave_scan_state_error;
        EC_SLAVE_ERR(slave, "无法读取数据链路层状态：");
        ec_datagram_print_wc_error(fsm->datagram);
        return;
    }

    ec_slave_set_dl_status(slave, EC_READ_U16(fsm->datagram->data));

#ifdef EC_SII_ASSIGN
    ec_fsm_slave_scan_enter_assign_sii(fsm, datagram);
#elif defined(EC_SII_CACHE)
    ec_fsm_slave_scan_enter_sii_identity(fsm, datagram);
#else
    ec_fsm_slave_scan_enter_attach_sii(fsm, datagram);
#endif
}

/*****************************************************************************/

#ifdef EC_SII_ASSIGN

/**
@brief ：从站扫描状态：分配SII。
@param fsm ：从站状态机
@param datagram ：要使用的数据报
@return ：无
@details ：该函数用于在分配SII状态下执行从站扫描。根据接收到的数据报状态和工作计数器进行相应的处理。
           函数逻辑如下：
           1. 如果数据报的状态为超时且重试次数未达到上限，则重复发送数据报并返回。
           2. 如果数据报的状态不是接收到，则打印警告信息和数据报的状态，并尝试继续执行，可能分配是正确的，然后跳转到继续执行SII大小的处理。
           3. 如果数据报的工作计数器不等于1，则打印警告信息和数据报的工作计数器错误，并尝试继续执行，可能分配是正确的。
           4. 继续执行SII大小的处理：
              - 如果定义了EC_SII_CACHE，则调用ec_fsm_slave_scan_enter_sii_identity函数进入SII标识状态。
              - 否则，调用ec_fsm_slave_scan_enter_attach_sii函数进入附加SII状态。
*/

void ec_fsm_slave_scan_state_assign_sii(
    ec_fsm_slave_scan_t *fsm, /**< 从站状态机 */
    ec_datagram_t *datagram   /**< 要使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;

    if (fsm->datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--)
    {
        ec_datagram_repeat(datagram, fsm->datagram);
        return;
    }

    if (fsm->datagram->state != EC_DATAGRAM_RECEIVED)
    {
        EC_SLAVE_WARN(slave, "无法接收到SII分配数据报：");
        ec_datagram_print_state(fsm->datagram);
        // 尝试继续执行，可能分配是正确的
        goto continue_with_sii_size;
    }

    if (fsm->datagram->working_counter != 1)
    {
        EC_SLAVE_WARN(slave, "无法将SII分配给EtherCAT：");
        ec_datagram_print_wc_error(fsm->datagram);
        // 尝试继续执行，可能分配是正确的
    }

continue_with_sii_size:
#ifdef EC_SII_CACHE
    ec_fsm_slave_scan_enter_sii_identity(fsm, datagram);
#else
    ec_fsm_slave_scan_enter_attach_sii(fsm, datagram);
#endif
}

#endif

/**
@brief 从SII（Slave Information Interface）中获取从站的标识信息。
@param fsm 从站扫描状态机
@param datagram 用于通信的数据报
@return 无
@details
- 通过执行ec_fsm_sii_exec函数从SII中读取数据。
- 如果执行成功，函数返回。
- 如果无法确定SII标识信息，则输出错误信息并根据扫描重试次数决定下一步操作。
- 根据SII偏移量进行不同的处理：
    - 如果偏移量为EC_ALIAS_SII_OFFSET，则将从站的有效别名设置为读取到的值，并根据别名是否为零决定下一步偏移量。
    - 如果偏移量为EC_SERIAL_SII_OFFSET，则将从站的有效序列号设置为读取到的值，并判断序列号是否为零来决定是否进入附加SII的状态。
    - 如果偏移量为EC_VENDOR_SII_OFFSET，则将从站的有效供应商ID设置为读取到的值。
    - 如果偏移量为EC_PRODUCT_SII_OFFSET，则将从站的有效产品代码设置为读取到的值，并将偏移量设置为EC_REVISION_SII_OFFSET。
    - 如果偏移量为EC_REVISION_SII_OFFSET，则将从站的有效修订号设置为读取到的值，并进入附加SII的状态。
    - 如果偏移量为其他值，则输出错误信息并设置从站的错误标志。
- 使用ec_fsm_sii_read函数从SII中读取数据。
*/

#ifdef EC_SII_CACHE
/*****************************************************************************/

void ec_fsm_slave_scan_state_sii_identity(
    ec_fsm_slave_scan_t *fsm, /**< 从站状态机 */
    ec_datagram_t *datagram   /**< 用于通信的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;

    while (1)
    {
        if (ec_fsm_sii_exec(&fsm->fsm_sii, datagram))
            return;

        if (!ec_fsm_sii_success(&fsm->fsm_sii))
        {
            EC_SLAVE_ERR(slave, "无法确定SII标识信息\n");
            if (fsm->scan_retries--)
            {
                fsm->state = ec_fsm_slave_scan_state_retry;
            }
            else
            {
                fsm->slave->error_flag = 1;
                fsm->state = ec_fsm_slave_scan_state_error;
            }
            return;
        }

        switch (fsm->sii_offset)
        {
        case EC_ALIAS_SII_OFFSET:
            slave->effective_alias = EC_READ_U16(fsm->fsm_sii.value);
            EC_SLAVE_DBG(slave, 1, "别名: %u\n",
                         (uint32_t)slave->effective_alias);
            if (slave->effective_alias)
            {
                fsm->sii_offset = EC_REVISION_SII_OFFSET;
            }
            else
            {
                fsm->sii_offset = EC_SERIAL_SII_OFFSET;
            }
            break;
        case EC_SERIAL_SII_OFFSET:
            slave->effective_serial_number = EC_READ_U32(fsm->fsm_sii.value);
            EC_SLAVE_DBG(slave, 1, "序列号: 0x%08x\n",
                         slave->effective_serial_number);
            if (!slave->effective_serial_number)
            {
                ec_fsm_slave_scan_enter_attach_sii(fsm, datagram);
                return;
            }
            fsm->sii_offset = EC_VENDOR_SII_OFFSET;
            break;
        case EC_VENDOR_SII_OFFSET:
            slave->effective_vendor_id = EC_READ_U32(fsm->fsm_sii.value);
            EC_SLAVE_DBG(slave, 1, "供应商ID: 0x%08x\n",
                         slave->effective_vendor_id);
            fsm->sii_offset = EC_PRODUCT_SII_OFFSET;
            break;
        case EC_PRODUCT_SII_OFFSET:
            slave->effective_product_code = EC_READ_U32(fsm->fsm_sii.value);
            EC_SLAVE_DBG(slave, 1, "产品代码: 0x%08x\n",
                         slave->effective_product_code);
            fsm->sii_offset = EC_REVISION_SII_OFFSET;
            break;
        case EC_REVISION_SII_OFFSET:
            slave->effective_revision_number = EC_READ_U32(fsm->fsm_sii.value);
            EC_SLAVE_DBG(slave, 1, "修订号: 0x%08x\n",
                         slave->effective_revision_number);
            ec_fsm_slave_scan_enter_attach_sii(fsm, datagram);
            return;
        default:
            fsm->slave->error_flag = 1;
            fsm->state = ec_fsm_slave_scan_state_error;
            EC_SLAVE_ERR(slave, "在标识信息扫描中遇到意外的偏移量 %u。\n",
                         fsm->sii_offset);
            return;
        }

        ec_fsm_sii_read(&fsm->fsm_sii, fsm->slave, fsm->sii_offset,
                        EC_FSM_SII_USE_CONFIGURED_ADDRESS);
    }
}
#endif


#ifdef EC_SII_OVERRIDE
/*****************************************************************************/

/**
@brief 执行从站扫描状态：SII设备。
@param fsm 从站状态机
@param datagram 使用的数据报
@return 无
@details 
- 如果执行ec_fsm_sii_exec(&fsm->fsm_sii, datagram)成功，则直接返回。
- 如果ec_fsm_sii_success(&fsm->fsm_sii)返回false：
    - 输出错误日志，指示无法确定产品和供应商ID，读取字偏移0x%04x失败。
    - 如果还有扫描重试次数：
        - 设置状态为ec_fsm_slave_scan_state_retry。
    - 否则：
        - 设置从站的错误标志为1。
        - 设置状态为ec_fsm_slave_scan_state_error。
    - 直接返回。
- 复制fsm->fsm_sii.value的4个字节到slave->vendor_words + fsm->sii_offset。
- 如果fsm->sii_offset + 2 < 16：
    - 增加fsm->sii_offset的值2。
    - 调用ec_fsm_sii_read(&fsm->fsm_sii, slave, fsm->sii_offset, EC_FSM_SII_USE_CONFIGURED_ADDRESS)读取接下来的2个字。
    - 立即执行状态机。
    - 直接返回。
- 评估SII内容：
    - 设置slave->sii_image->sii.alias为EC_READ_U16(slave->vendor_words + EC_ALIAS_SII_OFFSET)。
    - 设置slave->sii_image->sii.vendor_id为EC_READ_U32(slave->vendor_words + EC_VENDOR_SII_OFFSET)。
    - 设置slave->sii_image->sii.product_code为EC_READ_U32(slave->vendor_words + EC_PRODUCT_SII_OFFSET)。
    - 设置slave->sii_image->sii.revision_number为EC_READ_U32(slave->vendor_words + EC_REVISION_SII_OFFSET)。
    - 设置slave->sii_image->sii.serial_number为EC_READ_U32(slave->vendor_words + EC_SERIAL_SII_OFFSET)。
- 检查无效的供应商ID和产品代码：
    - 如果slave->sii_image->sii.vendor_id为0或slave->sii_image->sii.product_code为0：
        - 输出错误日志，指示无法确定产品和供应商ID，SII返回了零值。
        - 如果还有扫描重试次数：
            - 设置状态为ec_fsm_slave_scan_state_retry。
        - 否则：
            - 设置从站的错误标志为1。
            - 设置状态为ec_fsm_slave_scan_state_error。
        - 直接返回。
- 设置slave的effective_alias为slave->sii_image->sii.alias。
#ifdef EC_SII_CACHE
- 设置slave的effective_vendor_id为slave->sii_image->sii.vendor_id。
- 设置slave的effective_product_code为slave->sii_image->sii.product_code。
- 设置slave的effective_revision_number为slave->sii_image->sii.revision_number。
- 设置slave的effective_serial_number为slave->sii_image->sii.serial_number。
#endif
- 进入从站扫描状态：SII请求。
*/

void ec_fsm_slave_scan_state_sii_device(
    ec_fsm_slave_scan_t *fsm, /**< 从站状态机 */
    ec_datagram_t *datagram   /**< 使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;

    if (ec_fsm_sii_exec(&fsm->fsm_sii, datagram))
        return;

    if (!ec_fsm_sii_success(&fsm->fsm_sii))
    {
        EC_SLAVE_ERR(slave, "无法确定产品和供应商ID。读取字偏移0x%04x失败。\n",
                     fsm->sii_offset);
        if (fsm->scan_retries--)
        {
            fsm->state = ec_fsm_slave_scan_state_retry;
        }
        else
        {
            fsm->slave->error_flag = 1;
            fsm->state = ec_fsm_slave_scan_state_error;
        }
        return;
    }

    memcpy(slave->vendor_words + fsm->sii_offset, fsm->fsm_sii.value, 4);

    if (fsm->sii_offset + 2 < 16)
    {
        // 获取接下来的2个字
        fsm->sii_offset += 2;
        ec_fsm_sii_read(&fsm->fsm_sii, slave, fsm->sii_offset,
                        EC_FSM_SII_USE_CONFIGURED_ADDRESS);
        ec_fsm_sii_exec(&fsm->fsm_sii, datagram); // 立即执行状态机
        return;
    }

    // 评估SII内容
    slave->sii_image->sii.alias = EC_READ_U16(slave->vendor_words + EC_ALIAS_SII_OFFSET);
    slave->sii_image->sii.vendor_id = EC_READ_U32(slave->vendor_words + EC_VENDOR_SII_OFFSET);
    slave->sii_image->sii.product_code = EC_READ_U32(slave->vendor_words + EC_PRODUCT_SII_OFFSET);
    slave->sii_image->sii.revision_number = EC_READ_U32(slave->vendor_words + EC_REVISION_SII_OFFSET);
    slave->sii_image->sii.serial_number = EC_READ_U32(slave->vendor_words + EC_SERIAL_SII_OFFSET);

    // 检查无效的供应商ID和产品代码
    if ((slave->sii_image->sii.vendor_id == 0) ||
        (slave->sii_image->sii.product_code == 0))
    {
        EC_SLAVE_ERR(slave, "无法确定产品和供应商ID。SII返回了零值。\n");
        if (fsm->scan_retries--)
        {
            fsm->state = ec_fsm_slave_scan_state_retry;
        }
        else
        {
            fsm->slave->error_flag = 1;
            fsm->state = ec_fsm_slave_scan_state_error;
        }
        return;
    }

    slave->effective_alias = slave->sii_image->sii.alias;
#ifdef EC_SII_CACHE
    slave->effective_vendor_id = slave->sii_image->sii.vendor_id;
    slave->effective_product_code = slave->sii_image->sii.product_code;
    slave->effective_revision_number = slave->sii_image->sii.revision_number;
    slave->effective_serial_number = slave->sii_image->sii.serial_number;
#endif

    ec_fsm_slave_scan_enter_sii_request(fsm, datagram);
}

/*****************************************************************************/

/**
@brief 结构体，用于固件请求上下文。
*/
struct firmware_request_context
{
    struct task_struct *fsm_task; /**< 任务结构体指针 */
    ec_fsm_slave_scan_t *fsm;     /**< 从站状态机指针 */
    ec_slave_t *slave;            /**< 从站指针 */
};

/**
@brief 无SII固件。
*/
static const struct firmware no_sii_firmware;

/**
@brief 固件请求完成的回调函数。
@param firmware 固件指针
@param context 上下文指针
@return 无
*/
static void firmware_request_complete(
    const struct firmware *firmware,
    void *context)
{
    struct firmware_request_context *ctx = context;
    ec_fsm_slave_scan_t *fsm = ctx->fsm;

    if (fsm->slave != ctx->slave)
    {
        printk(KERN_ERR "中止固件请求；FSM从站意外更改。\n");
        ec_release_sii_firmware(firmware);
    }
    else if (fsm->state != ec_fsm_slave_scan_state_sii_request)
    {
        EC_SLAVE_WARN(fsm->slave, "中止固件请求；FSM状态意外更改。\n");
        ec_release_sii_firmware(firmware);
    }
    else
    {
        fsm->sii_firmware = firmware ? firmware : &no_sii_firmware;
    }

    kfree(ctx);
}

/*****************************************************************************/

/**
@brief 进入从站扫描状态：SII请求。
@param fsm 从站状态机
@param datagram 使用的数据报
@return 无
@details
- 分配固件请求上下文。
- 如果分配失败：
    - 输出错误日志，指示无法分配固件请求上下文。
    - 设置状态为ec_fsm_slave_scan_state_error。
    - 直接返回。
- 设置fsm的sii_firmware为NULL。
- 设置状态为ec_fsm_slave_scan_state_sii_request。
- 发起SII固件请求。
- 立即执行状态机。
*/

void ec_fsm_slave_scan_enter_sii_request(
    ec_fsm_slave_scan_t *fsm, /**< 从站状态机 */
    ec_datagram_t *datagram   /**< 使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;
    struct firmware_request_context *ctx;

    if (!(ctx = kmalloc(sizeof(*ctx), GFP_KERNEL)))
    {
        EC_SLAVE_ERR(slave, "无法分配固件请求上下文。\n");
        fsm->state = ec_fsm_slave_scan_state_error;
        return;
    }

    ctx->fsm_task = current;
    ctx->fsm = fsm;
    ctx->slave = slave;

    fsm->sii_firmware = NULL;
    fsm->state = ec_fsm_slave_scan_state_sii_request;
    ec_request_sii_firmware(slave, ctx, firmware_request_complete);
    fsm->state(fsm, datagram); // 立即执行状态机
}

/*****************************************************************************/

/**
@brief 从站扫描状态：SII请求。
@param fsm 从站状态机
@param datagram 使用的数据报
@return 无
@details
- 获取固件指针。
- 如果固件为no_sii_firmware：
    - 输出调试日志，指示未找到SII固件文件，将从从站读取SII数据。
    - 将fsm的sii_firmware设置为NULL。
    - 设置sii_offset为EC_FIRST_SII_CATEGORY_OFFSET（第一个类别头）。
    - 调用ec_fsm_sii_read(&fsm->fsm_sii, slave, fsm->sii_offset, EC_FSM_SII_USE_CONFIGURED_ADDRESS)读取接下来的2个字。
    - 设置状态为ec_fsm_slave_scan_state_sii_size。
    - 立即执行状态机。
- 如果固件存在：
    - 输出调试日志，指示找到固件文件，读取固件大小。
    - 设置从站的sii_image->nwords为固件大小除以2。
    - 如果从站的sii_image->words不为NULL：
        - 输出警告日志，指示释放旧的SII数据。
        - 释放从站的sii_image->words内存。
    - 分配从站的sii_image->words内存，大小为sii_image->nwords乘以2。
    - 如果分配失败：
        - 输出错误日志，指示无法分配SII数据内存。
        - 设置从站的sii_image->nwords为0。
        - 设置从站的错误标志为1。
        - 释放固件内存。
        - 将fsm的sii_firmware设置为NULL。
        - 设置状态为ec_fsm_slave_scan_state_error。
        - 直接返回。
    - 复制固件数据到从站的sii_image->words。
    - 释放固件内存。
    - 将fsm的sii_firmware设置为NULL。
    - 设置状态为ec_fsm_slave_scan_state_sii_parse。
    - 立即执行状态机。
- 否则：
    - 数据报的状态设置为EC_DATAGRAM_INVALID，等待异步请求完成时不执行任何操作。
*/

void ec_fsm_slave_scan_state_sii_request(
    ec_fsm_slave_scan_t *fsm, /**< 从站状态机 */
    ec_datagram_t *datagram   /**< 使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;
    const struct firmware *firmware = fsm->sii_firmware;

    if (firmware == &no_sii_firmware)
    {
        EC_SLAVE_DBG(slave, 1, "未找到SII固件文件；从从站读取SII数据。\n");
        fsm->sii_firmware = NULL;

        fsm->sii_offset = EC_FIRST_SII_CATEGORY_OFFSET; // 第一个类别头
        ec_fsm_sii_read(&fsm->fsm_sii, slave, fsm->sii_offset,
                        EC_FSM_SII_USE_CONFIGURED_ADDRESS);
        fsm->state = ec_fsm_slave_scan_state_sii_size;
        fsm->state(fsm, datagram); // 立即执行状态机
    }
    else if (firmware)
    {
        EC_SLAVE_DBG(slave, 1, "找到固件文件，读取%zu字节。\n", firmware->size);

        slave->sii_image->nwords = firmware->size / 2;

        if (slave->sii_image->words)
        {
            EC_SLAVE_WARN(slave, "释放旧的SII数据...\n");
            kfree(slave->sii_image->words);
        }
        if (!(slave->sii_image->words =
                  (uint16_t *)kmalloc(slave->sii_image->nwords * 2, GFP_KERNEL)))
        {
            EC_SLAVE_ERR(slave, "无法分配%zu字的SII数据。\n",
                         slave->sii_image->nwords);
            slave->sii_image->nwords = 0;
            slave->error_flag = 1;
            ec_release_sii_firmware(firmware);
            fsm->sii_firmware = NULL;

            fsm->state = ec_fsm_slave_scan_state_error;
            return;
        }

        memcpy(slave->sii_image->words, firmware->data, slave->sii_image->nwords * 2);
        ec_release_sii_firmware(firmware);
        fsm->sii_firmware = NULL;

        fsm->state = ec_fsm_slave_scan_state_sii_parse;
        fsm->state(fsm, datagram); // 立即执行状态机
    }
    else
    {
        // 等待异步请求完成时不执行任何操作
        datagram->state = EC_DATAGRAM_INVALID;
    }
}
#endif

/*****************************************************************************/

/**
@brief 从站扫描状态：SII SIZE。
@param fsm 从站状态机
@param datagram 使用的数据报
@return 无
@details
- 如果执行ec_fsm_sii_exec(&fsm->fsm_sii, datagram)成功，则直接返回。
- 如果从站的sii_image为NULL：
    - 输出错误日志，指示从站没有附加SII图像。
    - 设置从站的错误标志为1。
    - 设置状态为ec_fsm_slave_scan_state_error。
    - 直接返回。
- 如果执行ec_fsm_sii_success(&fsm->fsm_sii)返回false：
    - 如果还有扫描重试次数：
        - 输出错误日志，指示无法确定SII内容大小，重试中。
        - 设置状态为ec_fsm_slave_scan_state_retry。
    - 否则：
        - 设置从站的错误标志为1。
        - 设置状态为ec_fsm_slave_scan_state_error。
        - 输出错误日志，指示无法确定SII内容大小：读取字偏移0x%04x失败。假设%u个字。
        - 设置从站的sii_image->nwords为EC_FIRST_SII_CATEGORY_OFFSET。
        - 跳转到alloc_sii。
    - 直接返回。
- 读取cat_type和cat_size。
- 如果cat_type不等于0xFFFF：
    - 如果下一个偏移超过EC_MAX_SII_SIZE：
        - 输出警告日志，指示SII大小超过%u个字（缺失0xffff限制器？）。
        - 设置从站的sii_image->nwords为EC_FIRST_SII_CATEGORY_OFFSET。
        - 跳转到alloc_sii。
    - 设置fsm的sii_offset为下一个偏移。
    - 调用ec_fsm_sii_read(&fsm->fsm_sii, slave, fsm->sii_offset, EC_FSM_SII_USE_CONFIGURED_ADDRESS)读取接下来的2个字。
    - 立即执行状态机。
    - 直接返回。
- 设置从站的sii_image->nwords为fsm->sii_offset + 1。
- 跳转到alloc_sii。

alloc_sii:
- 如果从站的sii_image->words不为NULL：
    - 输出警告日志，指示释放旧的SII数据。
    - 释放从站的sii_image->words内存。
- 分配从站的sii_image->words内存，大小为sii_image->nwords乘以2。
- 如果分配失败：
    - 输出错误日志，指示无法分配%zu个字的SII数据。
    - 设置从站的sii_image->nwords为0。
    - 设置从站的错误标志为1。
    - 设置状态为ec_fsm_slave_scan_state_error。
    - 直接返回。
- 如果定义了EC_SII_OVERRIDE：
    - 将从站的sii_image->words复制到从站的sii_image->words。
    - 释放从站的vendor_words内存。
    - 设置fsm的sii_offset为0x0010。
- 否则：
    - 设置fsm的sii_offset为0x0000。
- 设置状态为ec_fsm_slave_scan_state_sii_data。
- 调用ec_fsm_sii_read(&fsm->fsm_sii, slave, fsm->sii_offset, EC_FSM_SII_USE_CONFIGURED_ADDRESS)读取接下来的2个字。
- 立即执行状态机。
*/

void ec_fsm_slave_scan_state_sii_size(
    ec_fsm_slave_scan_t *fsm, /**< 从站状态机 */
    ec_datagram_t *datagram   /**< 使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;
    uint16_t cat_type, cat_size;

    if (ec_fsm_sii_exec(&fsm->fsm_sii, datagram))
        return;

    if (!slave->sii_image)
    {
        EC_SLAVE_ERR(slave, "从站没有附加SII图像！\n");
        slave->error_flag = 1;
        fsm->state = ec_fsm_slave_scan_state_error;
        return;
    }

    if (!ec_fsm_sii_success(&fsm->fsm_sii))
    {
        if (fsm->scan_retries--)
        {
            EC_SLAVE_ERR(slave, "无法确定SII内容大小，重试中。\n");
            fsm->state = ec_fsm_slave_scan_state_retry;
            return;
        }
        else
        {
            fsm->slave->error_flag = 1;
            fsm->state = ec_fsm_slave_scan_state_error;
            EC_SLAVE_ERR(slave, "无法确定SII内容大小：读取字偏移0x%04x失败。假设%u个字。\n",
                         fsm->sii_offset, EC_FIRST_SII_CATEGORY_OFFSET);
            slave->sii_image->nwords = EC_FIRST_SII_CATEGORY_OFFSET;
            goto alloc_sii;
        }
    }

    cat_type = EC_READ_U16(fsm->fsm_sii.value);
    cat_size = EC_READ_U16(fsm->fsm_sii.value + 2);

    if (cat_type != 0xFFFF)
    {
        off_t next_offset = 2UL + fsm->sii_offset + cat_size;
        if (next_offset >= EC_MAX_SII_SIZE)
        {
            EC_SLAVE_WARN(slave, "SII大小超过%u个字（缺失0xffff限制器？）。\n",
                          EC_MAX_SII_SIZE);
            // 切断类别数据...
            slave->sii_image->nwords = EC_FIRST_SII_CATEGORY_OFFSET;
            goto alloc_sii;
        }
        fsm->sii_offset = next_offset;
        ec_fsm_sii_read(&fsm->fsm_sii, slave, fsm->sii_offset,
                        EC_FSM_SII_USE_CONFIGURED_ADDRESS);
        ec_fsm_sii_exec(&fsm->fsm_sii, datagram); // 立即执行状态机
        return;
    }

    slave->sii_image->nwords = fsm->sii_offset + 1;

alloc_sii:
    if (slave->sii_image->words)
    {
        EC_SLAVE_WARN(slave, "释放旧的SII数据...\n");
        kfree(slave->sii_image->words);
    }

    if (!(slave->sii_image->words =
              (uint16_t *)kmalloc(slave->sii_image->nwords * 2, GFP_KERNEL)))
    {
        EC_SLAVE_ERR(slave, "无法分配%zu个字的SII数据。\n",
                     slave->sii_image->nwords);
        slave->sii_image->nwords = 0;
        slave->error_flag = 1;
        fsm->state = ec_fsm_slave_scan_state_error;
        return;
    }

#ifdef EC_SII_OVERRIDE
    // 将vendor数据复制到sii words
    memcpy(slave->sii_image->words, slave->vendor_words, 32);
    kfree(slave->vendor_words);
    slave->vendor_words = NULL;

    // 开始获取SII内容的其余部分
    fsm->sii_offset = 0x0010;
#else
    // 开始获取SII内容
    fsm->sii_offset = 0x0000;
#endif
    fsm->state = ec_fsm_slave_scan_state_sii_data;
    ec_fsm_sii_read(&fsm->fsm_sii, slave, fsm->sii_offset,
                    EC_FSM_SII_USE_CONFIGURED_ADDRESS);
    ec_fsm_sii_exec(&fsm->fsm_sii, datagram); // 立即执行状态机
}

/*****************************************************************************/

/**
@brief 从站扫描状态：SII DATA。
@param fsm 从站状态机
@param datagram 使用的数据报
@return 无
@details
- 如果执行ec_fsm_sii_exec(&fsm->fsm_sii, datagram)成功，则直接返回。
- 如果执行ec_fsm_sii_success(&fsm->fsm_sii)返回false：
    - 输出错误日志，指示无法获取SII内容。
    - 如果还有扫描重试次数：
        - 设置状态为ec_fsm_slave_scan_state_retry。
    - 否则：
        - 设置从站的错误标志为1。
        - 设置状态为ec_fsm_slave_scan_state_error。
    - 直接返回。
- 如果从站的sii_image为NULL：
    - 输出错误日志，指示从站没有附加SII图像。
    - 设置从站的错误标志为1。
    - 设置状态为ec_fsm_slave_scan_state_error。
    - 直接返回。
- 2个字已获取。
- 如果fsm->sii_offset + 2小于等于从站的sii_image->nwords：
    - 将fsm->fsm_sii.value的4个字节复制到从站的sii_image->words + fsm->sii_offset。
- 否则：
    - 将fsm->fsm_sii.value的2个字节复制到从站的sii_image->words + fsm->sii_offset。
- 如果fsm->sii_offset + 2 < 从站的sii_image->nwords：
    - 获取接下来的2个字
    - 增加fsm->sii_offset的值2。
    - 调用ec_fsm_sii_read(&fsm->fsm_sii, slave, fsm->sii_offset, EC_FSM_SII_USE_CONFIGURED_ADDRESS)读取接下来的2个字。
    - 立即执行状态机。
    - 直接返回。
- 设置状态为ec_fsm_slave_scan_state_sii_parse。
- 立即执行状态机。
*/

void ec_fsm_slave_scan_state_sii_data(
    ec_fsm_slave_scan_t *fsm, /**< 从站状态机 */
    ec_datagram_t *datagram   /**< 使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;

    if (ec_fsm_sii_exec(&fsm->fsm_sii, datagram))
        return;

    if (!ec_fsm_sii_success(&fsm->fsm_sii))
    {
        EC_SLAVE_ERR(slave, "无法获取SII内容。\n");
        if (fsm->scan_retries--)
        {
            fsm->state = ec_fsm_slave_scan_state_retry;
        }
        else
        {
            fsm->slave->error_flag = 1;
            fsm->state = ec_fsm_slave_scan_state_error;
        }
        return;
    }

    if (!slave->sii_image)
    {
        EC_SLAVE_ERR(slave, "从站没有附加SII图像！\n");
        slave->error_flag = 1;
        fsm->state = ec_fsm_slave_scan_state_error;
        return;
    }

    // 2个字已获取

    if (fsm->sii_offset + 2 <= slave->sii_image->nwords)
    {
        memcpy(slave->sii_image->words + fsm->sii_offset, fsm->fsm_sii.value, 4);
    }
    else
    {
        memcpy(slave->sii_image->words + fsm->sii_offset, fsm->fsm_sii.value, 2);
    }

    if (fsm->sii_offset + 2 < slave->sii_image->nwords)
    {
        // 获取接下来的2个字
        fsm->sii_offset += 2;
        ec_fsm_sii_read(&fsm->fsm_sii, slave, fsm->sii_offset,
                        EC_FSM_SII_USE_CONFIGURED_ADDRESS);
        ec_fsm_sii_exec(&fsm->fsm_sii, datagram); // 立即执行状态机
        return;
    }

    fsm->state = ec_fsm_slave_scan_state_sii_parse;
    fsm->state(fsm, datagram); // 立即执行状态机
}

/*****************************************************************************/

/**
@brief 从站扫描状态：SII PARSE。
@param fsm 从站状态机
@param datagram 使用的数据报
@return 无
@details
- 评估SII内容。
- 清除从站的同步管理器。
- 如果未定义EC_SII_OVERRIDE：
    - 设置从站的sii_image->sii.alias为EC_READ_U16(slave->sii_image->words + 0x0004)。
    - 设置从站的effective_alias为从站的sii_image->sii.alias。
    - 设置从站的sii_image->sii.vendor_id为EC_READ_U32(slave->sii_image->words + 0x0008)。
    - 设置从站的sii_image->sii.product_code为EC_READ_U32(slave->sii_image->words + 0x000A)。
    - 设置从站的sii_image->sii.revision_number为EC_READ_U32(slave->sii_image->words + 0x000C)。
    - 设置从站的sii_image->sii.serial_number为EC_READ_U32(slave->sii_image->words + 0x000E)。
- 设置从站的sii_image->sii.boot_rx_mailbox_offset为EC_READ_U16(slave->sii_image->words + 0x0014)。
- 设置从站的sii_image->sii.boot_rx_mailbox_size为EC_READ_U16(slave->sii_image->words + 0x0015)。
- 设置从站的sii_image->sii.boot_tx_mailbox_offset为EC_READ_U16(slave->sii_image->words + 0x0016)。
- 设置从站的sii_image->sii.boot_tx_mailbox_size为EC_READ_U16(slave->sii_image->words + 0x0017)。
- 设置从站的sii_image->sii.std_rx_mailbox_offset为EC_READ_U16(slave->sii_image->words + 0x0018)。
- 设置从站的sii_image->sii.std_rx_mailbox_size为EC_READ_U16(slave->sii_image->words + 0x0019)。
- 设置从站的sii_image->sii.std_tx_mailbox_offset为EC_READ_U16(slave->sii_image->words + 0x001A)。
- 设置从站的sii_image->sii.std_tx_mailbox_size为EC_READ_U16(slave->sii_image->words + 0x001B)。
- 设置从站的sii_image->sii.mailbox_protocols为EC_READ_U16(slave->sii_image->words + 0x001C)。
- 如果（从站的sii_image->sii.boot_rx_mailbox_offset等于0xFFFF）或（从站的sii_image->sii.boot_tx_mailbox_offset等于0xFFFF）或（从站的sii_image->sii.std_rx_mailbox_offset等于0xFFFF）或（从站的sii_image->sii.std_tx_mailbox_offset等于0xFFFF）：
    - 设置从站的sii_image->sii.boot_rx_mailbox_offset为0。
    - 设置从站的sii_image->sii.boot_tx_mailbox_offset为0。
    - 设置从站的sii_image->sii.boot_rx_mailbox_size为0。
    - 设置从站的sii_image->sii.boot_tx_mailbox_size为0。
    - 设置从站的sii_image->sii.std_rx_mailbox_offset为0。
    - 设置从站的sii_image->sii.std_tx_mailbox_offset为0。
    - 设置从站的sii_image->sii.std_rx_mailbox_size为0。
    - 设置从站的sii_image->sii.std_tx_mailbox_size为0。
    - 设置从站的sii_image->sii.mailbox_protocols为0。
    - 输出错误日志，指示SII数据中的偏移量不正确。
- 如果从站的sii_image->nwords等于EC_FIRST_SII_CATEGORY_OFFSET：
    - 设置状态为ec_fsm_slave_scan_state_end。
    - 直接返回。
- 如果从站的sii_image->nwords小于EC_FIRST_SII_CATEGORY_OFFSET + 1：
    - 输出错误日志，指示SII数据意外结束：缺少第一个类别头。
    - 跳转到end。
- 设置cat_word为从站的sii_image->words + EC_FIRST_SII_CATEGORY_OFFSET。
- 当EC_READ_U16(cat_word)不等于0xFFFF时：
    - 如果cat_word + 2 - 从站的sii_image->words大于从站的sii_image->nwords：
        - 输出错误日志，指示SII数据意外结束：类别头不完整。
        - 跳转到end。
    - 设置cat_type为EC_READ_U16(cat_word) & 0x7FFF。
    - 设置cat_size为EC_READ_U16(cat_word + 1)。
    - 将cat_word增加2。
    - 如果cat_word + cat_size - 从站的sii_image->words大于从站的sii_image->nwords：
        - 输出警告日志，指示SII数据意外结束：类别数据不完整。
        - 跳转到end。
    - 根据cat_type执行相应的操作：
        - 0x000A：调用ec_slave_fetch_sii_strings(slave, (uint8_t *)cat_word, cat_size * 2)。
        - 0x001E：调用ec_slave_fetch_sii_general(slave, (uint8_t *)cat_word, cat_size * 2)。
        - 0x0028：无操作。
        - 0x0029：调用ec_slave_fetch_sii_syncs(slave, (uint8_t *)cat_word, cat_size * 2)。
        - 0x0032：调用ec_slave_fetch_sii_pdos(slave, (uint8_t *)cat_word, cat_size * 2, EC_DIR_INPUT)。
        - 0x0033：调用ec_slave_fetch_sii_pdos(slave, (uint8_t *)cat_word, cat_size * 2, EC_DIR_OUTPUT)。
        - 其他：输出调试日志，指示未知的类别类型0x%04X。
    - 将cat_word增加cat_size。
    - 如果cat_word - 从站的sii_image->words大于等于从站的sii_image->nwords：
        - 输出警告日志，指示SII数据意外结束：缺少下一个类别头。
        - 跳转到end。
- 如果定义了EC_REGALIAS：
    - 调用ec_fsm_slave_scan_enter_regalias(fsm, datagram)。
- 否则：
    - 如果从站的sii_image->sii.mailbox_protocols & EC_MBOX_COE为真：
        - 调用ec_fsm_slave_scan_enter_preop(fsm, datagram)。
    - 否则：
        - 设置状态为ec_fsm_slave_scan_state_end。
    - 直接返回。

end:
- 输出错误日志，指示无法分析类别数据。
- 设置从站的错误标志为1。
- 设置状态为ec_fsm_slave_scan_state_error。
*/

void ec_fsm_slave_scan_state_sii_parse(
    ec_fsm_slave_scan_t *fsm, /**< 从站状态机 */
    ec_datagram_t *datagram   /**< 使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;
    uint16_t *cat_word, cat_type, cat_size;

    // 评估SII内容
    ec_slave_clear_sync_managers(slave);

#ifndef EC_SII_OVERRIDE
    slave->sii_image->sii.alias =
        EC_READ_U16(slave->sii_image->words + 0x0004);
    slave->effective_alias = slave->sii_image->sii.alias;
    slave->sii_image->sii.vendor_id =
        EC_READ_U32(slave->sii_image->words + 0x0008);
    slave->sii_image->sii.product_code =
        EC_READ_U32(slave->sii_image->words + 0x000A);
    slave->sii_image->sii.revision_number =
        EC_READ_U32(slave->sii_image->words + 0x000C);
    slave->sii_image->sii.serial_number =
        EC_READ_U32(slave->sii_image->words + 0x000E);
#endif
    slave->sii_image->sii.boot_rx_mailbox_offset =
        EC_READ_U16(slave->sii_image->words + 0x0014);
    slave->sii_image->sii.boot_rx_mailbox_size =
        EC_READ_U16(slave->sii_image->words + 0x0015);
    slave->sii_image->sii.boot_tx_mailbox_offset =
        EC_READ_U16(slave->sii_image->words + 0x0016);
    slave->sii_image->sii.boot_tx_mailbox_size =
        EC_READ_U16(slave->sii_image->words + 0x0017);
    slave->sii_image->sii.std_rx_mailbox_offset =
        EC_READ_U16(slave->sii_image->words + 0x0018);
    slave->sii_image->sii.std_rx_mailbox_size =
        EC_READ_U16(slave->sii_image->words + 0x0019);
    slave->sii_image->sii.std_tx_mailbox_offset =
        EC_READ_U16(slave->sii_image->words + 0x001A);
    slave->sii_image->sii.std_tx_mailbox_size =
        EC_READ_U16(slave->sii_image->words + 0x001B);
    slave->sii_image->sii.mailbox_protocols =
        EC_READ_U16(slave->sii_image->words + 0x001C);

    if ((slave->sii_image->sii.boot_rx_mailbox_offset == 0xFFFF) ||
        (slave->sii_image->sii.boot_tx_mailbox_offset == 0xFFFF) ||
        (slave->sii_image->sii.std_rx_mailbox_offset == 0xFFFF) ||
        (slave->sii_image->sii.std_tx_mailbox_offset == 0xFFFF))
    {
        slave->sii_image->sii.boot_rx_mailbox_offset = 0;
        slave->sii_image->sii.boot_tx_mailbox_offset = 0;
        slave->sii_image->sii.boot_rx_mailbox_size = 0;
        slave->sii_image->sii.boot_tx_mailbox_size = 0;
        slave->sii_image->sii.std_rx_mailbox_offset = 0;
        slave->sii_image->sii.std_tx_mailbox_offset = 0;
        slave->sii_image->sii.std_rx_mailbox_size = 0;
        slave->sii_image->sii.std_tx_mailbox_size = 0;
        slave->sii_image->sii.mailbox_protocols = 0;
        EC_SLAVE_ERR(slave, "SII数据中的偏移量不正确。\n");
    }

    if (slave->sii_image->nwords == EC_FIRST_SII_CATEGORY_OFFSET)
    {
        // sii不包含类别数据
        fsm->state = ec_fsm_slave_scan_state_end;
        return;
    }

    if (slave->sii_image->nwords < EC_FIRST_SII_CATEGORY_OFFSET + 1)
    {
        EC_SLAVE_ERR(slave, "SII数据意外结束：缺少第一个类别头。\n");
        goto end;
    }

    // 评估类别数据
    cat_word = slave->sii_image->words + EC_FIRST_SII_CATEGORY_OFFSET;
    while (EC_READ_U16(cat_word) != 0xFFFF)
    {

        // 类型和大小的字必须匹配
        if (cat_word + 2 - slave->sii_image->words > slave->sii_image->nwords)
        {
            EC_SLAVE_ERR(slave, "SII数据意外结束：类别头不完整。\n");
            goto end;
        }

        cat_type = EC_READ_U16(cat_word) & 0x7FFF;
        cat_size = EC_READ_U16(cat_word + 1);
        cat_word += 2;

        if (cat_word + cat_size - slave->sii_image->words > slave->sii_image->nwords)
        {
            EC_SLAVE_WARN(slave, "SII数据意外结束：类别数据不完整。\n");
            goto end;
        }

        switch (cat_type)
        {
        case 0x000A:
            if (ec_slave_fetch_sii_strings(slave, (uint8_t *)cat_word,
                                           cat_size * 2))
                goto end;
            break;
        case 0x001E:
            if (ec_slave_fetch_sii_general(slave, (uint8_t *)cat_word,
                                           cat_size * 2))
                goto end;
            break;
        case 0x0028:
            break;
        case 0x0029:
            if (ec_slave_fetch_sii_syncs(slave, (uint8_t *)cat_word,
                                         cat_size * 2))
                goto end;
            break;
        case 0x0032:
            if (ec_slave_fetch_sii_pdos(slave, (uint8_t *)cat_word,
                                        cat_size * 2, EC_DIR_INPUT))
                goto end;
            break;
        case 0x0033:
            if (ec_slave_fetch_sii_pdos(slave, (uint8_t *)cat_word,
                                        cat_size * 2, EC_DIR_OUTPUT))
                goto end;
            break;
        default:
            EC_SLAVE_DBG(slave, 1, "未知的类别类型0x%04X。\n",
                         cat_type);
        }

        cat_word += cat_size;
        if (cat_word - slave->sii_image->words >= slave->sii_image->nwords)
        {
            EC_SLAVE_WARN(slave, "SII数据意外结束：缺少下一个类别头。\n");
            goto end;
        }
    }

#ifdef EC_REGALIAS
    ec_fsm_slave_scan_enter_regalias(fsm, datagram);
#else
    if (slave->sii_image->sii.mailbox_protocols & EC_MBOX_COE)
    {
        ec_fsm_slave_scan_enter_preop(fsm, datagram);
    }
    else
    {
        fsm->state = ec_fsm_slave_scan_state_end;
    }
#endif
    return;

end:
    EC_SLAVE_ERR(slave, "无法分析类别数据。\n");
    fsm->slave->error_flag = 1;
    fsm->state = ec_fsm_slave_scan_state_error;
}

/*****************************************************************************/

#ifdef EC_REGALIAS

/**
@brief 从站扫描入口函数：REGALIAS。
@param fsm 从站状态机
@param datagram 使用的数据报
@return 无
@details
- 从寄存器中读取别名。
- 设置状态为ec_fsm_slave_scan_state_regalias。
*/

void ec_fsm_slave_scan_enter_regalias(
    ec_fsm_slave_scan_t *fsm, /**< 从站状态机 */
    ec_datagram_t *datagram   /**< 使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;

    // 从寄存器中读取别名
    EC_SLAVE_DBG(slave, 1, "从寄存器中读取别名。\n");
    ec_datagram_fprd(datagram, slave->station_address, 0x0012, 2);
    ec_datagram_zero(datagram);
    fsm->retries = EC_FSM_RETRIES;
    fsm->state = ec_fsm_slave_scan_state_regalias;
}

/*****************************************************************************/

/**
@brief 从站扫描状态：REGALIAS。
@param fsm 从站状态机
@param datagram 使用的数据报
@return 无
@details
- 如果数据报的状态为EC_DATAGRAM_TIMED_OUT并且还有重试次数：
    - 重复发送数据报。
    - 直接返回。
- 如果数据报的状态不等于EC_DATAGRAM_RECEIVED：
    - 设置状态为ec_fsm_slave_scan_state_error。
    - 输出错误日志，指示无法接收到寄存器别名的数据报。
    - 输出数据报的状态。
    - 直接返回。
- 如果数据报的工作计数器不等于1：
    - 输出调试日志，指示无法读取寄存器别名。
- 否则：
    - 设置从站的effective_alias为EC_READ_U16(fsm->datagram->data)。
    - 输出调试日志，指示从寄存器中读取到别名%u。
- 如果从站的sii_image为NULL：
    - 输出错误日志，指示从站没有附加SII图像。
    - 设置从站的错误标志为1。
    - 设置状态为ec_fsm_slave_scan_state_error。
    - 直接返回。
- 如果从站的sii_image->sii.mailbox_protocols & EC_MBOX_COE为真：
    - 调用ec_fsm_slave_scan_enter_preop(fsm, datagram)。
- 否则：
    - 设置状态为ec_fsm_slave_scan_state_end。
*/

void ec_fsm_slave_scan_state_regalias(
    ec_fsm_slave_scan_t *fsm, /**< 从站状态机 */
    ec_datagram_t *datagram   /**< 使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;

    if (fsm->datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--)
    {
        ec_datagram_repeat(datagram, fsm->datagram);
        return;
    }

    if (fsm->datagram->state != EC_DATAGRAM_RECEIVED)
    {
        fsm->state = ec_fsm_slave_scan_state_error;
        EC_SLAVE_ERR(slave, "无法接收到寄存器别名的数据报：");
        ec_datagram_print_state(fsm->datagram);
        return;
    }

    if (fsm->datagram->working_counter != 1)
    {
        EC_SLAVE_DBG(slave, 1, "无法读取寄存器别名。\n");
    }
    else
    {
        slave->effective_alias = EC_READ_U16(fsm->datagram->data);
        EC_SLAVE_DBG(slave, 1, "从寄存器中读取到别名%u。\n",
                     slave->effective_alias);
    }

    if (!slave->sii_image)
    {
        EC_SLAVE_ERR(slave, "从站没有附加SII图像！\n");
        slave->error_flag = 1;
        fsm->state = ec_fsm_slave_scan_state_error;
        return;
    }

    if (slave->sii_image->sii.mailbox_protocols & EC_MBOX_COE)
    {
        ec_fsm_slave_scan_enter_preop(fsm, datagram);
    }
    else
    {
        fsm->state = ec_fsm_slave_scan_state_end;
    }
}

#endif // defined EC_REGALIAS

/*****************************************************************************/

/**
@brief 从站扫描入口函数：PREOP。
@param fsm 从站状态机
@param datagram 使用的数据报
@return 无
@details
- 如果从站的当前状态不是PREOP、SAFEOP或OP：
    - 如果主站的调试级别不为0：
        - 输出调试日志，指示从站不处于进行邮箱通信的状态，将其设置为PREOP。
    - 设置状态为ec_fsm_slave_scan_state_preop。
    - 调用ec_slave_request_state(slave, EC_SLAVE_STATE_PREOP)将从站状态设置为PREOP。
    - 调用ec_fsm_slave_config_start(fsm->fsm_slave_config)开始从站配置状态机。
    - 调用ec_fsm_slave_config_exec(fsm->fsm_slave_config, datagram)立即执行状态机。
- 否则：
    - 输出调试日志，指示正在读取邮箱同步管理器配置。
    - 从从站的station_address地址处读取0x0800到EC_SYNC_PAGE_SIZE * 2个字节的数据报。
    - 清空数据报。
    - 设置重试次数为EC_FSM_RETRIES。
    - 设置状态为ec_fsm_slave_scan_state_sync。
*/

void ec_fsm_slave_scan_enter_preop(
    ec_fsm_slave_scan_t *fsm, /**< 从站状态机 */
    ec_datagram_t *datagram   /**< 使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;
    uint8_t current_state = slave->current_state & EC_SLAVE_STATE_MASK;

    if (current_state != EC_SLAVE_STATE_PREOP && current_state != EC_SLAVE_STATE_SAFEOP && current_state != EC_SLAVE_STATE_OP)
    {
        if (slave->master->debug_level)
        {
            char str[EC_STATE_STRING_SIZE];
            ec_state_string(current_state, str, 0);
            EC_SLAVE_DBG(slave, 0, "从站不处于进行邮箱通信的状态（%s），将其设置为PREOP。\n",
                         str);
        }

        fsm->state = ec_fsm_slave_scan_state_preop;
        ec_slave_request_state(slave, EC_SLAVE_STATE_PREOP);
        ec_fsm_slave_config_start(fsm->fsm_slave_config);
        ec_fsm_slave_config_exec(fsm->fsm_slave_config, datagram);
    }
    else
    {
        EC_SLAVE_DBG(slave, 1, "正在读取邮箱同步管理器配置。\n");

        /* 扫描当前同步管理器配置以获取配置的邮箱大小。 */
        ec_datagram_fprd(datagram, slave->station_address, 0x0800,
                         EC_SYNC_PAGE_SIZE * 2);
        ec_datagram_zero(datagram);
        fsm->retries = EC_FSM_RETRIES;
        fsm->state = ec_fsm_slave_scan_state_sync;
    }
}

/*****************************************************************************/

/**
@brief 从站扫描状态：PREOP。
@param fsm 从站状态机
@param datagram 使用的数据报
@return 无
@details
- 如果执行ec_fsm_slave_config_exec(fsm->fsm_slave_config, datagram)成功，则直接返回。
- 如果执行ec_fsm_slave_config_success(fsm->fsm_slave_config)返回false：
    - 设置状态为ec_fsm_slave_scan_state_error。
    - 直接返回。
- 如果从站的sii_image为NULL：
    - 输出错误日志，指示从站没有附加SII图像。
    - 设置从站的错误标志为1。
    - 设置状态为ec_fsm_slave_scan_state_error。
    - 直接返回。
- 分配支持的邮箱协议的邮箱响应数据的内存。
- 调用ec_fsm_slave_scan_enter_clear_mailbox(fsm, datagram)。
*/

void ec_fsm_slave_scan_state_preop(
    ec_fsm_slave_scan_t *fsm, /**< 从站状态机 */
    ec_datagram_t *datagram   /**< 使用的数据报 */
)
{
    if (ec_fsm_slave_config_exec(fsm->fsm_slave_config, datagram))
        return;

    if (!ec_fsm_slave_config_success(fsm->fsm_slave_config))
    {
        fsm->state = ec_fsm_slave_scan_state_error;
        return;
    }

    ec_fsm_slave_scan_enter_clear_mailbox(fsm, datagram);
}

/*****************************************************************************/

/**
@brief 从站扫描状态：SYNC。
@param fsm 从站状态机
@param datagram 使用的数据报
@return 无
@details
- 如果数据报的状态为EC_DATAGRAM_TIMED_OUT并且还有重试次数：
    - 重复发送数据报。
    - 直接返回。
- 如果数据报的状态不等于EC_DATAGRAM_RECEIVED：
    - 设置状态为ec_fsm_slave_scan_state_error。
    - 输出错误日志，指示无法接收到同步管理器配置数据报。
    - 输出数据报的状态。
    - 直接返回。
- 如果数据报的工作计数器不等于1：
    - 设置从站的错误标志为1。
    - 设置状态为ec_fsm_slave_scan_state_error。
    - 输出错误日志，指示无法读取DL状态。
    - 输出数据报的工作计数器错误。
    - 直接返回。
- 从数据报的数据中读取配置的邮箱偏移和大小。
- 输出调试日志，指示邮箱配置。
- 如果从站的sii_image为NULL：
    - 输出错误日志，指示从站没有附加SII图像。
    - 设置从站的错误标志为1。
    - 设置状态为ec_fsm_slave_scan_state_error。
    - 直接返回。
- 调用ec_fsm_slave_scan_enter_clear_mailbox(fsm, datagram)。
*/

void ec_fsm_slave_scan_state_sync(
    ec_fsm_slave_scan_t *fsm, /**< 从站状态机 */
    ec_datagram_t *datagram   /**< 使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;

    if (fsm->datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--)
    {
        ec_datagram_repeat(datagram, fsm->datagram);
        return;
    }

    if (fsm->datagram->state != EC_DATAGRAM_RECEIVED)
    {
        fsm->state = ec_fsm_slave_scan_state_error;
        EC_SLAVE_ERR(slave, "无法接收到同步管理器配置数据报：");
        ec_datagram_print_state(fsm->datagram);
        return;
    }

    if (fsm->datagram->working_counter != 1)
    {
        fsm->slave->error_flag = 1;
        fsm->state = ec_fsm_slave_scan_state_error;
        EC_SLAVE_ERR(slave, "无法读取DL状态：");
        ec_datagram_print_wc_error(fsm->datagram);
        return;
    }

    slave->configured_rx_mailbox_offset = EC_READ_U16(fsm->datagram->data);
    slave->configured_rx_mailbox_size = EC_READ_U16(fsm->datagram->data + 2);
    slave->configured_tx_mailbox_offset = EC_READ_U16(fsm->datagram->data + 8);
    slave->configured_tx_mailbox_size = EC_READ_U16(fsm->datagram->data + 10);

    EC_SLAVE_DBG(slave, 1, "邮箱配置：\n");
    EC_SLAVE_DBG(slave, 1, " RX 偏移=0x%04x 大小=%u\n",
                 slave->configured_rx_mailbox_offset,
                 slave->configured_rx_mailbox_size);
    EC_SLAVE_DBG(slave, 1, " TX 偏移=0x%04x 大小=%u\n",
                 slave->configured_tx_mailbox_offset,
                 slave->configured_tx_mailbox_size);

    if (!slave->sii_image)
    {
        EC_SLAVE_ERR(slave, "从站没有附加SII图像！\n");
        slave->error_flag = 1;
        fsm->state = ec_fsm_slave_scan_state_error;
        return;
    }

    // 为支持的邮箱协议预分配邮箱响应数据的内存
    ec_mbox_prot_data_prealloc(slave, slave->sii_image->sii.mailbox_protocols, slave->configured_tx_mailbox_size);

    ec_fsm_slave_scan_enter_clear_mailbox(fsm, datagram);
}

/*****************************************************************************/

/**
@brief 从站扫描状态：清空邮箱。
@param fsm 从站状态机
@param datagram 使用的数据报
@return 无
@details
- 如果从站的邮箱中有旧数据，读取并丢弃它。我们不需要先检查邮箱，只需忽略错误或空邮箱响应。
- 准备从从站中获取邮箱数据。
- 设置重试次数为EC_FSM_RETRIES。
- 设置状态为ec_fsm_slave_scan_state_mailbox_cleared。
- 将从站的valid_mbox_data标志设置为0。
*/

void ec_fsm_slave_scan_enter_clear_mailbox(
    ec_fsm_slave_scan_t *fsm, /**< 从站状态机 */
    ec_datagram_t *datagram   /**< 使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;

    // 如果从站的邮箱中有旧数据，读取并丢弃它。我们不需要先检查邮箱，只需忽略错误或空邮箱响应。
    ec_slave_mbox_prepare_fetch(fsm->slave, datagram);
    fsm->retries = EC_FSM_RETRIES;
    fsm->state = ec_fsm_slave_scan_state_mailbox_cleared;

    slave->valid_mbox_data = 0;
}

/*****************************************************************************/

/**
@brief 从站扫描状态：邮箱已清空。
@param fsm 从站状态机
@param datagram 使用的数据报
@return 无
@details
- 如果数据报的状态为EC_DATAGRAM_TIMED_OUT并且还有重试次数：
    - 准备从从站中获取邮箱数据。
    - 直接返回。
- 如果主站的调试级别大于0并且数据报的状态为EC_DATAGRAM_RECEIVED并且工作计数器为1：
    - 输出信息日志，指示已清空邮箱中的旧数据。
- 将从站的valid_mbox_data标志设置为1。
- 如果从站的sii_image为NULL：
    - 输出错误日志，指示从站没有附加SII图像。
    - 设置从站的错误标志为1。
    - 设置状态为ec_fsm_slave_scan_state_error。
    - 直接返回。
- 进入PDOS状态。
*/

void ec_fsm_slave_scan_state_mailbox_cleared(
    ec_fsm_slave_scan_t *fsm, /**< 从站状态机 */
    ec_datagram_t *datagram   /**< 使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;

#ifdef EC_SII_CACHE
    unsigned int i = 0;
    unsigned int fetch_pdos = 1;
#endif

    if (fsm->datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--)
    {
        ec_slave_mbox_prepare_fetch(fsm->slave, datagram);
        return;
    }

    if (unlikely(slave->master->debug_level > 0) && fsm->datagram->state == EC_DATAGRAM_RECEIVED && fsm->datagram->working_counter == 1)
        EC_SLAVE_INFO(slave, "已清空邮箱中的旧数据\n");

    slave->valid_mbox_data = 1;

    if (!slave->sii_image)
    {
        EC_SLAVE_ERR(slave, "从站没有附加SII图像！\n");
        slave->error_flag = 1;
        fsm->state = ec_fsm_slave_scan_state_error;
        return;
    }

#ifdef EC_SII_CACHE
    if ((slave->effective_alias != 0) || (slave->effective_serial_number != 0))
    {
        // SII数据已存储
        for (i = 0; i < slave->sii_image->sii.sync_count; i++)
        {
            if (!list_empty(&slave->sii_image->sii.syncs[i].pdos.list))
            {
                fetch_pdos = 0; // PDOs已经获取
                break;
            }
        }
    }
    if (!fetch_pdos)
    {
        fsm->state = ec_fsm_slave_scan_state_end;
    }
    else
#endif
    {
        ec_fsm_slave_scan_enter_pdos(fsm, datagram);
    }
}

/*****************************************************************************/

/**
@brief 进入从站扫描状态的PDOS。
@param fsm 从站状态机
@param datagram 要使用的数据报。
@return 无返回值
@details 
- 扫描PDO分配和映射。
- 将状态设置为ec_fsm_slave_scan_state_pdos。
- 启动PDO读取。
- 执行PDO。
*/
void ec_fsm_slave_scan_enter_pdos(
    ec_fsm_slave_scan_t *fsm, /**< 从站状态机 */
    ec_datagram_t *datagram   /**< 要使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;

    EC_SLAVE_DBG(slave, 1, "正在扫描PDO分配和映射。\n");
    fsm->state = ec_fsm_slave_scan_state_pdos;
    ec_fsm_pdo_start_reading(fsm->fsm_pdo, slave);
    ec_fsm_pdo_exec(fsm->fsm_pdo, datagram); // 立即执行
}

/*****************************************************************************/

/**
@brief 从站扫描状态：PDOS。
@param fsm 从站状态机
@param datagram 要使用的数据报。
@return 无返回值
@details 
- 如果执行PDO成功，则返回。
- 如果执行PDO失败，则将状态设置为ec_fsm_slave_scan_state_error。
- 如果PDO配置读取完成，则将状态设置为ec_fsm_slave_scan_state_end。
*/
void ec_fsm_slave_scan_state_pdos(
    ec_fsm_slave_scan_t *fsm, /**< 从站状态机 */
    ec_datagram_t *datagram   /**< 要使用的数据报 */
)
{
    if (ec_fsm_pdo_exec(fsm->fsm_pdo, datagram))
    {
        return;
    }

    if (!ec_fsm_pdo_success(fsm->fsm_pdo))
    {
        fsm->state = ec_fsm_slave_scan_state_error;
        return;
    }

    // PDO配置读取完成
    fsm->state = ec_fsm_slave_scan_state_end;
}

/*****************************************************************************/

/**
@brief 从站扫描状态：扫描重试。
@param fsm 从站状态机
@param datagram 要使用的数据报。
@return 无返回值
@details 
- 设置扫描开始的jiffies。
- 将状态设置为ec_fsm_slave_scan_state_retry_wait。
- 输出警告信息，表示正在重试从站扫描。
*/
void ec_fsm_slave_scan_state_retry(
    ec_fsm_slave_scan_t *fsm, /**< 从站状态机 */
    ec_datagram_t *datagram   /**< 要使用的数据报 */
)
{
    ec_slave_t *slave = fsm->slave;

    fsm->scan_jiffies_start = jiffies;
    fsm->state = ec_fsm_slave_scan_state_retry_wait;
    EC_SLAVE_WARN(slave, "正在重试从站扫描。\n");
    return;
}

/*****************************************************************************/

/**
@brief 从站扫描状态：扫描重试等待。
@param fsm 从站状态机
@param datagram 要使用的数据报。
@return 无返回值
@details 
- 等待超时。
- 如果超时时间达到SCAN_RETRY_TIME，则将状态设置为ec_fsm_slave_scan_state_start。
*/
void ec_fsm_slave_scan_state_retry_wait(
    ec_fsm_slave_scan_t *fsm, /**< 从站状态机 */
    ec_datagram_t *datagram   /**< 要使用的数据报 */
)
{
    // 等待超时
    unsigned long diff_ms =
        (jiffies - fsm->scan_jiffies_start) * 1000 / HZ;

    if (diff_ms >= SCAN_RETRY_TIME)
    {
        fsm->state = ec_fsm_slave_scan_state_start;
    }
}

/******************************************************************************
 * 通用状态函数
 *****************************************************************************/

/**
@brief 状态：错误。
@param fsm 从站状态机
@param datagram 要使用的数据报。
@return 无返回值
@details 无操作。
*/
void ec_fsm_slave_scan_state_error(
    ec_fsm_slave_scan_t *fsm, /**< 从站状态机 */
    ec_datagram_t *datagram   /**< 要使用的数据报 */
)
{
}

/*****************************************************************************/

/**
@brief 状态：结束。
@param fsm 从站状态机
@param datagram 要使用的数据报。
@return 无返回值
@details 无操作。
*/
void ec_fsm_slave_scan_state_end(
    ec_fsm_slave_scan_t *fsm, /**< 从站状态机 */
    ec_datagram_t *datagram   /**< 要使用的数据报 */
)
{
}

/*****************************************************************************/

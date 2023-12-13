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
   EtherCAT从站信息接口有限状态机。
*/

/*****************************************************************************/

#include "globals.h"
#include "mailbox.h"
#include "master.h"
#include "fsm_sii.h"

/** EEPROM加载超时时间[毫秒]。
 *
 * 根据jiffies计数器计算超时时间。
 *
 * \attention 必须大于10，以避免在定时器中断频率为100 Hz的内核上出现问题。
 */
#define SII_LOAD_TIMEOUT 500

/** 读/写超时时间[毫秒]。
 *
 * 根据jiffies计数器计算超时时间。
 *
 * \attention 必须大于10，以避免在定时器中断频率为100 Hz的内核上出现问题。
 */
#define SII_TIMEOUT 20

/** 写入操作前的等待时间[毫秒]。
 */
#define SII_INHIBIT 5

// #define SII_DEBUG

/*****************************************************************************/

void ec_fsm_sii_state_start_reading(ec_fsm_sii_t *, ec_datagram_t *);
void ec_fsm_sii_state_read_check(ec_fsm_sii_t *, ec_datagram_t *);
void ec_fsm_sii_state_read_fetch(ec_fsm_sii_t *, ec_datagram_t *);
void ec_fsm_sii_state_start_writing(ec_fsm_sii_t *, ec_datagram_t *);
void ec_fsm_sii_state_write_check(ec_fsm_sii_t *, ec_datagram_t *);
void ec_fsm_sii_state_write_check2(ec_fsm_sii_t *, ec_datagram_t *);
void ec_fsm_sii_state_end(ec_fsm_sii_t *, ec_datagram_t *);
void ec_fsm_sii_state_error(ec_fsm_sii_t *, ec_datagram_t *);

/*****************************************************************************/

/**
   \brief 构造函数。
   \details 初始化有限状态机。
   \param fsm 有限状态机。
   \return 无。
*/
void ec_fsm_sii_init(ec_fsm_sii_t *fsm /**< 有限状态机 */)
{
    fsm->state = NULL;
    fsm->datagram = NULL;
}

/*****************************************************************************/

/**
   \brief 析构函数。
   \details 清除有限状态机。
   \param fsm 有限状态机。
   \return 无。
*/
void ec_fsm_sii_clear(ec_fsm_sii_t *fsm /**< 有限状态机 */)
{
}

/*****************************************************************************/

/**
   \brief 初始化SII读取状态机。
   \details 初始化SII读取状态机，设置从站、偏移量和寻址方案。
   \param fsm 有限状态机。
   \param slave 要读取的从站。
   \param word_offset 要读取的偏移量。
   \param mode 寻址方案。
   \return 无。
*/
void ec_fsm_sii_read(ec_fsm_sii_t *fsm,           /**< 有限状态机 */
                     ec_slave_t *slave,           /**< 要读取的从站 */
                     uint16_t word_offset,        /**< 要读取的偏移量 */
                     ec_fsm_sii_addressing_t mode /**< 寻址方案 */
)
{
    fsm->state = ec_fsm_sii_state_start_reading;
    fsm->slave = slave;
    fsm->word_offset = word_offset;
    fsm->mode = mode;
}

/*****************************************************************************/

/**
   \brief 初始化SII写入状态机。
   \details 初始化SII写入状态机，设置从站、偏移量、数据和寻址方案。
   \param fsm 有限状态机。
   \param slave 要写入的从站。
   \param word_offset 要写入的偏移量。
   \param value 指向2个字节数据的指针。
   \param mode 寻址方案。
   \return 无。
*/
void ec_fsm_sii_write(ec_fsm_sii_t *fsm,           /**< 有限状态机 */
                      ec_slave_t *slave,           /**< 要写入的从站 */
                      uint16_t word_offset,        /**< 要写入的偏移量 */
                      const uint16_t *value,       /**< 指向2个字节数据的指针 */
                      ec_fsm_sii_addressing_t mode /**< 寻址方案 */
)
{
    fsm->state = ec_fsm_sii_state_start_writing;
    fsm->slave = slave;
    fsm->word_offset = word_offset;
    fsm->mode = mode;
    memcpy(fsm->value, value, 2);
}

/*****************************************************************************/

/**
   \brief 执行SII状态机。
   \details 执行SII状态机，并返回状态机是否已终止。
   \return 如果状态机已终止，则返回false。
*/
int ec_fsm_sii_exec(ec_fsm_sii_t *fsm,      /**< 有限状态机 */
                    ec_datagram_t *datagram /**< 要使用的数据报结构 */
)
{
    if (fsm->state == ec_fsm_sii_state_end || fsm->state == ec_fsm_sii_state_error)
        return 0;
    if (fsm->datagram &&
        (fsm->datagram->state == EC_DATAGRAM_INIT ||
         fsm->datagram->state == EC_DATAGRAM_QUEUED ||
         fsm->datagram->state == EC_DATAGRAM_SENT))
    {
        // 数据报尚未接收
        if (datagram != fsm->datagram)
            datagram->state = EC_DATAGRAM_INVALID;
        return 1;
    }

    fsm->state(fsm, datagram);

    if (fsm->state == ec_fsm_sii_state_end || fsm->state == ec_fsm_sii_state_error)
    {
        fsm->datagram = NULL;
        return 0;
    }

    fsm->datagram = datagram;
    return 1;
}

/*****************************************************************************/

/**
   \brief 返回主站启动状态机是否成功终止。
   \return 如果成功终止，则返回非零值。
*/
int ec_fsm_sii_success(ec_fsm_sii_t *fsm /**< 有限状态机 */)
{
    return fsm->state == ec_fsm_sii_state_end;
}

/******************************************************************************
 * datagram functions
 *****************************************************************************/

/**
   \brief 准备读取操作。
   \details 初始化读取操作，根据寻址方案选择不同的读取方式。
   \param fsm 有限状态机。
   \param datagram 使用的数据报结构。
   \return 无。
*/
static void ec_fsm_sii_prepare_read(
    ec_fsm_sii_t *fsm,      /**< 有限状态机 */
    ec_datagram_t *datagram /**< 要使用的数据报结构 */
)
{
    // 启动读取操作
    switch (fsm->mode)
    {
    case EC_FSM_SII_USE_INCREMENT_ADDRESS:
        ec_datagram_apwr(datagram, fsm->slave->ring_position, 0x502, 4);
        break;
    case EC_FSM_SII_USE_CONFIGURED_ADDRESS:
        ec_datagram_fpwr(datagram, fsm->slave->station_address, 0x502, 4);
        break;
    }

    EC_WRITE_U8(datagram->data, 0x80);     // 两个地址八位字节
    EC_WRITE_U8(datagram->data + 1, 0x01); // 请求读取操作
    EC_WRITE_U16(datagram->data + 2, fsm->word_offset);
}

/*****************************************************************************/

/**
   \brief 准备读取检查操作。
   \details 发送检查/获取数据报。
   \param fsm 有限状态机。
   \param datagram 使用的数据报结构。
   \return 无。
*/
static void ec_fsm_sii_prepare_read_check(
    ec_fsm_sii_t *fsm,      /**< 有限状态机 */
    ec_datagram_t *datagram /**< 要使用的数据报结构 */
)
{
    // 发送检查/获取数据报
    switch (fsm->mode)
    {
    case EC_FSM_SII_USE_INCREMENT_ADDRESS:
        ec_datagram_aprd(datagram, fsm->slave->ring_position, 0x502, 10);
        break;
    case EC_FSM_SII_USE_CONFIGURED_ADDRESS:
        ec_datagram_fprd(datagram, fsm->slave->station_address, 0x502, 10);
        break;
    }

    ec_datagram_zero(datagram);
}

/*****************************************************************************/

/**
   \brief 准备写入操作。
   \details 初始化写入操作。
   \param fsm 有限状态机。
   \param datagram 使用的数据报结构。
   \return 无。
*/
static void ec_fsm_sii_prepare_write(
    ec_fsm_sii_t *fsm,      /**< 有限状态机 */
    ec_datagram_t *datagram /**< 要使用的数据报结构 */
)
{
    // 启动写入操作
    ec_datagram_fpwr(datagram, fsm->slave->station_address, 0x502, 8);
    EC_WRITE_U8(datagram->data, 0x81);     /* 两个地址八位字节
                                              + 启用写入访问 */
    EC_WRITE_U8(datagram->data + 1, 0x02); // 请求写入操作
    EC_WRITE_U16(datagram->data + 2, fsm->word_offset);
    memset(datagram->data + 4, 0x00, 2);
    memcpy(datagram->data + 6, fsm->value, 2);
}

/*****************************************************************************/

/**
 * @brief 准备写入检查
 * @param fsm 有限状态机
 * @param datagram 使用的数据报
 * @return 无
 * @details 发送检查数据报，将数据报清零
 */
static void ec_fsm_sii_prepare_write_check(
    ec_fsm_sii_t *fsm,      /**< 有限状态机 */
    ec_datagram_t *datagram /**< 使用的数据报 */
)
{
    // 发送检查数据报
    ec_datagram_fprd(datagram, fsm->slave->station_address, 0x502, 2);
    // 数据报清零
    ec_datagram_zero(datagram);
}

/******************************************************************************
 * 状态函数
 *****************************************************************************/

/**
 * SII状态：开始读取
 * 开始读取从站信息接口
 */
void ec_fsm_sii_state_start_reading(
    ec_fsm_sii_t *fsm,      /**< 有限状态机 */
    ec_datagram_t *datagram /**< 使用的数据报 */
)
{
    ec_fsm_sii_prepare_read(fsm, datagram);

#ifdef SII_DEBUG
    EC_SLAVE_DBG(fsm->slave, 0, "正在读取SII数据，字 %u:\n",
                 fsm->word_offset);
    ec_print_data(datagram->data, 4);
#endif

    fsm->retries = EC_FSM_RETRIES;
    fsm->state = ec_fsm_sii_state_read_check;
}

/*****************************************************************************/

/**
 * SII状态：读取检查
 * 检查SII读取数据报是否已发送并发出获取数据报
 */
void ec_fsm_sii_state_read_check(
    ec_fsm_sii_t *fsm,      /**< 有限状态机 */
    ec_datagram_t *datagram /**< 使用的数据报 */
)
{
    if (fsm->datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--)
    {
        ec_fsm_sii_prepare_read(fsm, datagram);
        return;
    }

    if (fsm->datagram->state != EC_DATAGRAM_RECEIVED)
    {
        fsm->state = ec_fsm_sii_state_error;
        EC_SLAVE_ERR(fsm->slave, "无法接收SII读取数据报: ");
        ec_datagram_print_state(fsm->datagram);
        return;
    }

    if (fsm->datagram->working_counter != 1)
    {
        fsm->state = ec_fsm_sii_state_error;
        EC_SLAVE_ERR(fsm->slave, "接收SII读取数据报失败: ");
        ec_datagram_print_wc_error(fsm->datagram);
        return;
    }

    fsm->jiffies_start = fsm->datagram->jiffies_sent;
    fsm->check_once_more = 1;
    fsm->eeprom_load_retry = 0;

    ec_fsm_sii_prepare_read_check(fsm, datagram);
    fsm->retries = EC_FSM_RETRIES;
    fsm->state = ec_fsm_sii_state_read_fetch;
}

/*****************************************************************************/

/**
 * @brief SII状态：读取获取
 * @param fsm 有限状态机
 * @param datagram 使用的数据报
 * @return 无
 * @details 获取SII读取数据报的结果
 */
void ec_fsm_sii_state_read_fetch(
    ec_fsm_sii_t *fsm,      /**< 有限状态机 */
    ec_datagram_t *datagram /**< 使用的数据报 */
)
{
    if (fsm->datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--)
    {
        ec_fsm_sii_prepare_read_check(fsm, datagram);
        return;
    }

    if (fsm->datagram->state != EC_DATAGRAM_RECEIVED)
    {
        fsm->state = ec_fsm_sii_state_error;
        EC_SLAVE_ERR(fsm->slave,
                     "无法接收SII检查/获取数据报: ");
        ec_datagram_print_state(fsm->datagram);
        return;
    }

    if (fsm->datagram->working_counter != 1)
    {
        fsm->state = ec_fsm_sii_state_error;
        EC_SLAVE_ERR(fsm->slave,
                     "接收SII检查/获取数据报失败: ");
        ec_datagram_print_wc_error(fsm->datagram);
        return;
    }

#ifdef SII_DEBUG
    EC_SLAVE_DBG(fsm->slave, 0, "检查SII读取状态:\n");
    ec_print_data(fsm->datagram->data, 10);
#endif

    if (EC_READ_U8(fsm->datagram->data + 1) & 0x20)
    {
        EC_SLAVE_ERR(fsm->slave, "在读取SII字0x%04x时出错。\n",
                     fsm->word_offset);
        fsm->state = ec_fsm_sii_state_error;
        return;
    }

    // 检查"EEPROM Loading bit"
    if (EC_READ_U8(fsm->datagram->data + 1) & 0x10)
    { /* EEPROM未加载 */
        unsigned long diff_ms;

        if (fsm->eeprom_load_retry == 0)
        {
            fsm->eeprom_load_retry = 1;
            EC_SLAVE_WARN(fsm->slave,
                          "SII读取错误，EEPROM未加载。正在重试...\n");
        }

        // EEPROM仍未加载... 超时？
        // 可能是由于EEPROM加载错误
        diff_ms =
            (fsm->datagram->jiffies_received - fsm->jiffies_start) * 1000 / HZ;
        if (diff_ms >= SII_LOAD_TIMEOUT)
        {
            if (fsm->check_once_more)
            {
                fsm->check_once_more = 0;
            }
            else
            {
                EC_SLAVE_ERR(fsm->slave,
                             "SII错误：等待EEPROM加载超时。\n");
                fsm->state = ec_fsm_sii_state_error;
                return;
            }
        }

        // 再次发出检查/获取数据报
        ec_fsm_sii_prepare_read_check(fsm, datagram);
        fsm->retries = EC_FSM_RETRIES;
        return;
    }
    else if (fsm->eeprom_load_retry)
    {
        fsm->eeprom_load_retry = 0;
        EC_SLAVE_INFO(fsm->slave, "SII EEPROM已加载。继续。\n");

        // 重新开始读取SII值
        fsm->state = ec_fsm_sii_state_start_reading;
        return;
    }

    // 检查"busy bit"
    if (EC_READ_U8(fsm->datagram->data + 1) & 0x81)
    { /* busy bit or
    read operation busy */
        // 仍在忙... 超时？
        unsigned long diff_ms =
            (fsm->datagram->jiffies_received - fsm->jiffies_start) * 1000 / HZ;
        if (diff_ms >= SII_TIMEOUT)
        {
            if (fsm->check_once_more)
            {
                fsm->check_once_more = 0;
            }
            else
            {
                EC_SLAVE_ERR(fsm->slave, "SII：读取超时。\n");
                fsm->state = ec_fsm_sii_state_error;
                return;
            }
        }

        // 再次发出检查/获取数据报
        ec_fsm_sii_prepare_read_check(fsm, datagram);
        fsm->retries = EC_FSM_RETRIES;
        return;
    }

    // 收到SII值
    memcpy(fsm->value, fsm->datagram->data + 6, 4);
    fsm->state = ec_fsm_sii_state_end;
}

/*****************************************************************************/

/**
 * @brief SII状态：开始写入
 * @param fsm 有限状态机
 * @param datagram 使用的数据报
 * @return 无
 * @details 开始通过从站信息接口写入一个字
 */
void ec_fsm_sii_state_start_writing(
    ec_fsm_sii_t *fsm,      /**< 有限状态机 */
    ec_datagram_t *datagram /**< 使用的数据报 */
)
{
    ec_fsm_sii_prepare_write(fsm, datagram);

#ifdef SII_DEBUG
    EC_SLAVE_DBG(fsm->slave, 0, "写入SII数据:\n");
    ec_print_data(datagram->data, 8);
#endif

    fsm->retries = EC_FSM_RETRIES;
    fsm->state = ec_fsm_sii_state_write_check;
}

/*****************************************************************************/

/**
 * @brief SII状态：写入检查
 * @param fsm 有限状态机
 * @param datagram 使用的数据报
 * @return 无
 * @details 无
 */
void ec_fsm_sii_state_write_check(
    ec_fsm_sii_t *fsm,      /**< 有限状态机 */
    ec_datagram_t *datagram /**< 使用的数据报 */
)
{
    if (fsm->datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--)
    {
        ec_fsm_sii_prepare_write(fsm, datagram);
        return;
    }

    if (fsm->datagram->state != EC_DATAGRAM_RECEIVED)
    {
        fsm->state = ec_fsm_sii_state_error;
        EC_SLAVE_ERR(fsm->slave, "无法接收SII写入数据报: ");
        ec_datagram_print_state(fsm->datagram);
        return;
    }

    if (fsm->datagram->working_counter != 1)
    {
        fsm->state = ec_fsm_sii_state_error;
        EC_SLAVE_ERR(fsm->slave, "接收SII写入数据报失败: ");
        ec_datagram_print_wc_error(fsm->datagram);
        return;
    }

    fsm->jiffies_start = fsm->datagram->jiffies_sent;
    fsm->check_once_more = 1;

    ec_fsm_sii_prepare_write_check(fsm, datagram);
    fsm->retries = EC_FSM_RETRIES;
    fsm->state = ec_fsm_sii_state_write_check2;
}

/*****************************************************************************/

/**
 * @brief SII状态：写入检查2
 * @param fsm 有限状态机
 * @param datagram 使用的数据报
 * @return 无
 * @details 无
 */
void ec_fsm_sii_state_write_check2(
    ec_fsm_sii_t *fsm,      /**< 有限状态机 */
    ec_datagram_t *datagram /**< 使用的数据报 */
)
{
    unsigned long diff_ms;

    if (fsm->datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--)
    {
        ec_fsm_sii_prepare_write_check(fsm, datagram);
        return;
    }

    if (fsm->datagram->state != EC_DATAGRAM_RECEIVED)
    {
        fsm->state = ec_fsm_sii_state_error;
        EC_SLAVE_ERR(fsm->slave,
                     "无法接收SII写入检查数据报: ");
        ec_datagram_print_state(fsm->datagram);
        return;
    }

    if (fsm->datagram->working_counter != 1)
    {
        fsm->state = ec_fsm_sii_state_error;
        EC_SLAVE_ERR(fsm->slave,
                     "接收SII写入检查数据报失败: ");
        ec_datagram_print_wc_error(fsm->datagram);
        return;
    }

#ifdef SII_DEBUG
    EC_SLAVE_DBG(fsm->slave, 0, "检查SII写入状态:\n");
    ec_print_data(fsm->datagram->data, 2);
#endif

    if (EC_READ_U8(fsm->datagram->data + 1) & 0x20)
    {
        EC_SLAVE_ERR(fsm->slave, "SII：上一个SII命令出错！\n");
        fsm->state = ec_fsm_sii_state_error;
        return;
    }

    /* FIXME: 一些从站永远不会以设置忙标志的方式回答...
     * 等待几毫秒以完成写操作。 */
    diff_ms = (fsm->datagram->jiffies_received - fsm->jiffies_start) * 1000 / HZ;
    if (diff_ms < SII_INHIBIT)
    {
#ifdef SII_DEBUG
        EC_SLAVE_DBG(fsm->slave, 0, "太早了。\n");
#endif
        // 再次发出检查数据报
        fsm->retries = EC_FSM_RETRIES;
        return;
    }

    if (EC_READ_U8(fsm->datagram->data + 1) & 0x82)
    { /* busy bit or
    write operation busy bit */
        // 仍在忙... 超时？
        if (diff_ms >= SII_TIMEOUT)
        {
            if (fsm->check_once_more)
            {
                fsm->check_once_more = 0;
            }
            else
            {
                EC_SLAVE_ERR(fsm->slave, "SII：写入超时。\n");
                fsm->state = ec_fsm_sii_state_error;
                return;
            }
        }

        // 再次发出检查数据报
        ec_fsm_sii_prepare_write_check(fsm, datagram);
        fsm->retries = EC_FSM_RETRIES;
        return;
    }

    if (EC_READ_U8(fsm->datagram->data + 1) & 0x40)
    {
        EC_SLAVE_ERR(fsm->slave, "SII：写入操作失败！\n");
        fsm->state = ec_fsm_sii_state_error;
        return;
    }

    // 成功
    fsm->state = ec_fsm_sii_state_end;
}

/*****************************************************************************/

/**
 * @brief 状态：错误
 * @param fsm 有限状态机
 * @param datagram 使用的数据报
 * @return 无
 * @details 无
 */
void ec_fsm_sii_state_error(
    ec_fsm_sii_t *fsm,      /**< 有限状态机 */
    ec_datagram_t *datagram /**< 使用的数据报 */
)
{
}

/*****************************************************************************/

/**
 * @brief 状态：结束
 * @param fsm 有限状态机
 * @param datagram 使用的数据报
 * @return 无
 * @details 无
 */
void ec_fsm_sii_state_end(
    ec_fsm_sii_t *fsm,      /**< 有限状态机 */
    ec_datagram_t *datagram /**< 使用的数据报 */
)
{
}

/*****************************************************************************/

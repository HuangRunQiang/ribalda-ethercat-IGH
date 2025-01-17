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
   Global definitions and macros.
*/

/*****************************************************************************/

#ifndef __EC_GLOBALS_H__
#define __EC_GLOBALS_H__

#include "config.h"

/******************************************************************************
 *  全局宏定义
 *****************************************************************************/

/** EC_STR()的辅助宏，将宏参数转换为字符串。
 *
 * \param X 要转换为字符串的参数。
 */
#define EC_LIT(X) #X

/** 将宏参数转换为字符串。
 *
 * \param X 要转换为字符串的参数。
 */
#define EC_STR(X) EC_LIT(X)

/** 主站版本字符串
 */
#define EC_MASTER_VERSION VERSION " " EC_STR(REV)

/*****************************************************************************/

#endif

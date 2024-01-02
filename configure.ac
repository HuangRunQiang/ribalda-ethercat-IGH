#------------------------------------------------------------------------------
#
#  $Id$
#
#  版权所有 (C) 2006-2012 Florian Pose, Ingenieurgemeinschaft IgH
#
#  此文件是 IgH EtherCAT Master 的一部分。
#
#  IgH EtherCAT Master 是自由软件；您可以根据 GNU General Public License 第2版的条款重新分发和修改它，该许可证由自由软件基金会发布。
#
#  IgH EtherCAT Master 希望它是有用的，但没有任何担保；甚至没有暗示的担保，适销性或适用于特定目的的担保。有关更多详细信息，请参阅 GNU General Public License。
#
#  您应该收到了 GNU General Public License 的副本；如果没有，请写信给 Free Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA。
#
#  ---
#
#  上述许可证仅涉及源代码。使用 EtherCAT 技术和品牌仅在遵守 Beckhoff Automation GmbH 的工业产权和类似权利的情况下允许。
#
#------------------------------------------------------------------------------

#
# 发布流程
# - 更新以发布内核版本的以太网驱动程序
# - 编写自上次发布以来的变更的 NEWS 条目
# - 检查是否有完整的 Doxygen 注释
# - 更新文档中的版本号
# - 更新 ChangeLog -> hg log -bstable-1.5 --style=changelog > ChangeLog
# - 在下方更新版本号
# - 运行 make dist-bzip2

AC_PREREQ(2.59)   # 检查Autoconf版本是否满足要求
AC_INIT([ethercat],[1.5.2],[fp@igh-essen.com])   # 设置软件包的名称、版本和联系人信息
AC_CONFIG_AUX_DIR([autoconf])   # 指定辅助文件的目录
AM_INIT_AUTOMAKE([-Wall -Werror dist-bzip2 subdir-objects])   # 初始化Automake，并设置编译选项
AC_CONFIG_HEADERS([config.h])   # 生成config.h文件
AC_CONFIG_SRCDIR([config.h.in])   # 指定源代码目录中的配置文件
AC_CONFIG_MACRO_DIR([m4])   # 指定宏文件的目录


#------------------------------------------------------------------------------
# Global
#------------------------------------------------------------------------------

AM_PROG_CC_C_O   # 检查并设置C编译器
m4_ifdef([AM_PROG_AR], [AM_PROG_AR])   # 检查并设置静态库工具（可选）
AC_PROG_CXX   # 检查并设置C++编译器
AC_PROG_LIBTOOL   # 检查并设置libtool工具


#------------------------------------------------------------------------------
# Kernel modules
#------------------------------------------------------------------------------

AC_MSG_CHECKING([whether to build kernel modules])

AC_ARG_ENABLE([kernel],
    AS_HELP_STRING([--enable-kernel],
                   [Enable building kernel modules]),
    [
        case "${enableval}" in
            yes) enablekernel=1
                ;;
            no) enablekernel=0
                ;;
            *) AC_MSG_ERROR([Invalid value for --enable-generic])
                ;;
        esac
    ],
    [enablekernel=1]
)

if test "x$enablekernel" = "x1"; then
    AC_MSG_RESULT([yes])
else
    AC_MSG_RESULT([no])
fi

AM_CONDITIONAL(ENABLE_KERNEL, test "x$enablekernel" = "x1")
AC_SUBST(ENABLE_KERNEL,[$enablekernel])

#------------------------------------------------------------------------------
# Linux sources
#------------------------------------------------------------------------------

if test "x$enablekernel" = "x1"; then
    # 检查是否启用内核配置

    AC_ARG_WITH([linux-dir],
        AC_HELP_STRING(
            [--with-linux-dir=<DIR>],
            [Linux kernel sources @<:@running kernel@:>@]
        ),
        [
            sourcedir=[$withval]
        ],
        [
            version=[`uname -r`]
            modulesdir=/lib/modules/${version}
            if test \! -d ${modulesdir} || test \! -d ${modulesdir}/build; then
                echo
                AC_MSG_ERROR([Failed to find Linux sources. Use --with-linux-dir!])
            fi
            sourcedir=`cd ${modulesdir}/build && pwd -P`
        ]
    )

    AC_MSG_CHECKING([for Linux kernel sources])
    # 检查Linux内核源代码路径

    if test \! -r ${sourcedir}/.config; then
        echo
        AC_MSG_ERROR([No configured Linux kernel sources in $sourcedir])
    fi
    # 检查是否存在配置的Linux内核源代码

    # 尝试获取内核版本字符串
    if test -r ${sourcedir}/include/config/kernel.release; then
        kernelrelease=`cat $sourcedir/include/config/kernel.release`
    elif test -r ${sourcedir}/.kernelrelease; then
        kernelrelease=`cat $sourcedir/.kernelrelease`
    elif test -r ${sourcedir}/include/linux/utsrelease.h; then
        hdr=${sourcedir}/include/linux/utsrelease.h
        kernelrelease=`grep UTS_RELEASE $hdr | cut -d " " -f 3- | tr -d \"`
    elif test -r ${sourcedir}/include/linux/version.h; then
        hdr=${sourcedir}/include/linux/version.h
        kernelrelease=`grep UTS_RELEASE $hdr | cut -d " " -f 3- | tr -d \"`
    fi

    if test -z "$kernelrelease"; then
        echo
        AC_MSG_ERROR([Failed to extract Linux kernel version!])
    fi
    # 检查是否成功提取了Linux内核版本号

    if test ${kernelrelease%%.*} -gt 2; then
        regex="^[[0-9]]+\.[[0-9]]+"
    else
        regex="^[[0-9]]+\.[[0-9]]+\.[[0-9]]+"
    fi
    # 根据内核版本号的格式设置正则表达式

    # 从内核版本中提取数字
    linuxversion=`echo $kernelrelease | grep -oE "$regex"`

    AC_SUBST(LINUX_SOURCE_DIR,[$sourcedir])
    AC_MSG_RESULT([$LINUX_SOURCE_DIR (Kernel $linuxversion)])
    # 保存内核源代码路径和版本信息

fi



#------------------------------------------------------------------------------
# Linux module installation subdirectory
#------------------------------------------------------------------------------

AC_ARG_WITH([module-dir],
    AC_HELP_STRING(
        [--with-module-dir=<DIR>],
        [Linux module installation dir. Default: ethercat]
    ),
    [moddir=[$withval]],
    [moddir="ethercat"]
)
AC_SUBST(INSTALL_MOD_DIR,[$moddir])

AC_MSG_CHECKING([for Linux modules installation directory])
AC_MSG_RESULT([$INSTALL_MOD_DIR])

#------------------------------------------------------------------------------
# Generic Ethernet driver
#------------------------------------------------------------------------------

AC_ARG_ENABLE([generic],
    AS_HELP_STRING([--enable-generic],
                   [Enable generic Ethernet driver]),
    [
        case "${enableval}" in
            yes) enablegeneric=1
                ;;
            no) enablegeneric=0
                ;;
            *) AC_MSG_ERROR([Invalid value for --enable-generic])
                ;;
        esac
    ],
    [enablegeneric=$enablekernel]
)

AM_CONDITIONAL(ENABLE_GENERIC, test "x$enablegeneric" = "x1")
AC_SUBST(ENABLE_GENERIC,[$enablegeneric])


#------------------------------------------------------------------------------
# CCAT driver
#------------------------------------------------------------------------------

AC_MSG_CHECKING([whether to build the CCAT driver])

AC_ARG_ENABLE([ccat],
    AS_HELP_STRING([--enable-ccat],
                   [Enable CCAT driver]),
    [
        case "${enableval}" in
            yes) enableccat=1
                ;;
            no) enableccat=0
                ;;
            *) AC_MSG_ERROR([Invalid value for --enable-ccat])
                ;;
        esac
    ],
    [enableccat=0] # disabled by default
)

if test "x${enableccat}" = "x1"; then
    AC_MSG_RESULT([yes])
else
    AC_MSG_RESULT([no])
fi

AM_CONDITIONAL(ENABLE_CCAT, test "x$enableccat" = "x1")
AC_SUBST(ENABLE_CCAT,[$enableccat])

#------------------------------------------------------------------------------
# RTAI path (optional)
#------------------------------------------------------------------------------

AC_ARG_WITH([rtai-dir],
    AC_HELP_STRING(
        [--with-rtai-dir=<DIR>],
        [RTAI path, for RTDM interface and RTAI examples]
    ),
    [
        rtaidir=[$withval]
        rtai=1
    ],
    [
        rtaidir=""
        rtai=0
    ]
)

AC_MSG_CHECKING([for RTAI path])

if test -z "${rtaidir}"; then
    AC_MSG_RESULT([not specified.])
else
    if test \! -r ${rtaidir}/include/rtai.h; then
        AC_MSG_ERROR([no RTAI installation found in ${rtaidir}!])
    fi
    AC_MSG_RESULT([$rtaidir])

    rtai_lxrt_cflags=`$rtaidir/bin/rtai-config --lxrt-cflags`
    rtai_lxrt_ldflags=`$rtaidir/bin/rtai-config --lxrt-ldflags`
fi

AC_SUBST(RTAI_DIR,[$rtaidir])
AM_CONDITIONAL(ENABLE_RTAI, test "x$rtai" = "x1")
AC_SUBST(ENABLE_RTAI,[$rtai])

AC_SUBST(RTAI_LXRT_CFLAGS,[$rtai_lxrt_cflags])
AC_SUBST(RTAI_LXRT_LDFLAGS,[$rtai_lxrt_ldflags])

#------------------------------------------------------------------------------
# Xenomai path (optional)
#------------------------------------------------------------------------------

AC_ARG_WITH([xenomai-dir],
    AC_HELP_STRING(
        [--with-xenomai-dir=<DIR>],
        [Xenomai path, for RTDM interface and Xenomai examples]
    ),
    [
        xenomaidir=[$withval]
        xeno=1
    ],
    [
        xenomaidir=""
        xeno=0
    ]
)

AC_MSG_CHECKING([for Xenomai path])

if test -z "${xenomaidir}"; then
    AC_MSG_RESULT([not specified.])
else
    if test \! -r ${xenomaidir}/include/xeno_config.h; then
        AC_MSG_ERROR([no Xenomai installation found in ${xenomaidir}!])
    fi
    AC_MSG_RESULT([$xenomaidir])

    xenomai_ver=`grep -R "VERSION_STRING" ${xenomaidir}/include/xeno_config.h | awk '{print $3}'`
    xenomai_ver=${xenomai_ver##\"}
    xenomai_ver=${xenomai_ver%%\"}
    echo "xenomai version: " $xenomai_ver

    if test ${xenomai_ver%%.*} -gt 2; then
        xeno_alchemy_cflags=`$xenomaidir/bin/xeno-config --skin alchemy --cflags`
        xeno_alchemy_ldflags=`$xenomaidir/bin/xeno-config --skin alchemy --auto-init-solib --ldflags`
        xeno_posix_ldflags=`$xenomaidir/bin/xeno-config --skin posix --auto-init-solib --ldflags`
        xeno_rtdm_ldflags=`$xenomaidir/bin/xeno-config --skin rtdm --auto-init-solib --ldflags`
        xeno_v3=1
        xeno=0
    else
        xeno_native_cflags=`$xenomaidir/bin/xeno-config --skin native --cflags`
        xeno_native_ldflags=`$xenomaidir/bin/xeno-config --skin native --ldflags`
        xeno_posix_ldflags=`$xenomaidir/bin/xeno-config --skin posix --ldflags`
        xeno_rtdm_ldflags=`$xenomaidir/bin/xeno-config --skin rtdm --ldflags`
    fi

    xeno_posix_cflags=`$xenomaidir/bin/xeno-config --skin posix --cflags`
    xeno_rtdm_cflags=`$xenomaidir/bin/xeno-config --skin rtdm --cflags`
fi

AC_SUBST(XENOMAI_DIR,[$xenomaidir])
AM_CONDITIONAL(ENABLE_XENOMAI, test "x$xeno" = "x1")
AC_SUBST(ENABLE_XENOMAI,[$xeno])
AM_CONDITIONAL(ENABLE_XENOMAI_V3, test "x$xeno_v3" = "x1")
AC_SUBST(ENABLE_XENOMAI_V3,[$xeno_v3])

AC_SUBST(XENOMAI_ALCHEMY_CFLAGS,[$xeno_alchemy_cflags])
AC_SUBST(XENOMAI_ALCHEMY_LDFLAGS,[$xeno_alchemy_ldflags])
AC_SUBST(XENOMAI_NATIVE_CFLAGS,[$xeno_native_cflags])
AC_SUBST(XENOMAI_NATIVE_LDFLAGS,[$xeno_native_ldflags])
AC_SUBST(XENOMAI_POSIX_CFLAGS,[$xeno_posix_cflags])
AC_SUBST(XENOMAI_POSIX_LDFLAGS,[$xeno_posix_ldflags])
AC_SUBST(XENOMAI_RTDM_CFLAGS,[$xeno_rtdm_cflags])
AC_SUBST(XENOMAI_RTDM_LDFLAGS,[$xeno_rtdm_ldflags])

#------------------------------------------------------------------------------
# RTDM interface (optional)
#------------------------------------------------------------------------------

AC_ARG_ENABLE([rtdm],
    AC_HELP_STRING(
        [--enable-rtdm],
        [Enable RTDM interface, depends on RTAI or Xenomai]
    ),
    [
        case "${enableval}" in
            yes) rtdm=1
                ;;
            no) rtdm=0
                ;;
            *) AC_MSG_ERROR([Invalid value for --enable-rtdm])
                ;;
        esac
    ],
    [rtdm=0]
)

AC_MSG_CHECKING([whether to build RTDM interface])

if test "x${rtdm}" = "x1"; then
    AC_DEFINE([EC_RTDM], [1], [RTDM interface enabled])
    AC_MSG_RESULT([yes])
else
    AC_MSG_RESULT([no])
fi

AM_CONDITIONAL(ENABLE_RTDM, test "x$rtdm" = "x1")
AC_SUBST(ENABLE_RTDM,[$rtdm])

#------------------------------------------------------------------------------
# Debug interface
#------------------------------------------------------------------------------

AC_MSG_CHECKING([whether to build the debug interface])

AC_ARG_ENABLE([debug-if],
    AS_HELP_STRING([--enable-debug-if],
                   [Create a debug interface for each master @<:@NO@:>@]),
    [
        case "${enableval}" in
            yes) dbg=1
                ;;
            no) dbg=0
                ;;
            *) AC_MSG_ERROR([Invalid value for --enable-debug-if])
                ;;
        esac
    ],
    [dbg=0]
)

if test "x${dbg}" = "x1"; then
    AC_DEFINE([EC_DEBUG_IF], [1], [Debug interfaces enabled])
    AC_MSG_RESULT([yes])
else
    AC_MSG_RESULT([no])
fi

AM_CONDITIONAL(ENABLE_DEBUG_IF, test "x$dbg" = "x1")
AC_SUBST(ENABLE_DEBUG_IF,[$dbg])

#------------------------------------------------------------------------------
# Debug ring
#------------------------------------------------------------------------------

AC_MSG_CHECKING([whether to build the debug ring])

AC_ARG_ENABLE([debug-ring],
    AS_HELP_STRING([--enable-debug-ring],
                   [Create a debug ring to record frames @<:@NO@:>@]),
    [
        case "${enableval}" in
            yes) debugring=1
                ;;
            no) debugring=0
                ;;
            *) AC_MSG_ERROR([Invalid value for --enable-debug-ring])
                ;;
        esac
    ],
    [debugring=0]
)

if test "x${debugring}" = "x1"; then
    AC_DEFINE([EC_DEBUG_RING], [1], [Debug ring enabled])
    AC_MSG_RESULT([yes])
else
    AC_MSG_RESULT([no])
fi

#------------------------------------------------------------------------------
# Ethernet over EtherCAT support
#------------------------------------------------------------------------------

AC_MSG_CHECKING([whether to build with EoE support])

AC_ARG_ENABLE([eoe],
    AS_HELP_STRING([--enable-eoe],
                   [Enable EoE support (default: yes)]),
    [
        case "${enableval}" in
            yes) eoe=1
                ;;
            no) eoe=0
                ;;
            *) AC_MSG_ERROR([Invalid value for --enable-eoe])
                ;;
        esac
    ],
    [eoe=1]
)

if test "x${eoe}" = "x1"; then
    AC_DEFINE([EC_EOE], [1], [EoE support enabled])
    AC_MSG_RESULT([yes])
else
    AC_MSG_RESULT([no])
fi

AM_CONDITIONAL(ENABLE_EOE, test "x$eoe" = "x1")
AC_SUBST(ENABLE_EOE,[$eoe])

#------------------------------------------------------------------------------
# CPU timestamp counter support
#------------------------------------------------------------------------------

AC_MSG_CHECKING([whether to use the CPU timestamp counter])

AC_ARG_ENABLE([cycles],
    AS_HELP_STRING([--enable-cycles],
                   [Use CPU timestamp counter (default: no)]),
    [
        case "${enableval}" in
            yes) cycles=1
                ;;
            no) cycles=0
                ;;
            *) AC_MSG_ERROR([Invalid value for --enable-cycles])
                ;;
        esac
    ],
    [cycles=0]
)

if test "x${cycles}" = "x1"; then
    AC_DEFINE([EC_HAVE_CYCLES], [1], [Use CPU timestamp counter])
    AC_MSG_RESULT([yes])
else
    AC_MSG_RESULT([no])
fi

#------------------------------------------------------------------------------
# Realtime Mutex support
#------------------------------------------------------------------------------

AC_MSG_CHECKING([whether to use rtmutexes])

AC_ARG_ENABLE([rtmutex],
    AS_HELP_STRING([--enable-rtmutex],
                   [Use rtmutexes for synchronization (default: no)]),
    [
        case "${enableval}" in
            yes) rtmutex=1
                ;;
            no) rtmutex=0
                ;;
            *) AC_MSG_ERROR([Invalid value for --enable-rtmutex])
                ;;
        esac
    ],
    [rtmutex=0]
)

if test "x${rtmutex}" = "x1"; then
    AC_DEFINE([EC_USE_RTMUTEX], [1], [Use rtmutex for synchronization])
    AC_MSG_RESULT([yes])
else
    AC_MSG_RESULT([no])
fi

#------------------------------------------------------------------------------
# High-resolution timer support
#------------------------------------------------------------------------------

AC_MSG_CHECKING([whether to use high-resolution timers for scheduling])

AC_ARG_ENABLE([hrtimer],
    AS_HELP_STRING([--enable-hrtimer],
                   [Use high-resolution timer for scheduling (default: no)]),
    [
        case "${enableval}" in
            yes) hrtimer=1
                ;;
            no) hrtimer=0
                ;;
            *) AC_MSG_ERROR([Invalid value for --enable-hrtimer])
                ;;
        esac
    ],
    [hrtimer=0]
)

if test "x${hrtimer}" = "x1"; then
    AC_DEFINE([EC_USE_HRTIMER], [1], [Use hrtimer for scheduling])
    AC_MSG_RESULT([yes])
else
    AC_MSG_RESULT([no])
fi

#------------------------------------------------------------------------------
# Read alias address from register
#------------------------------------------------------------------------------

AC_MSG_CHECKING([whether to read alias addresses from registers])

AC_ARG_ENABLE([regalias],
    AS_HELP_STRING([--enable-regalias],
                   [Read alias adresses from register (default: no)]),
    [
        case "${enableval}" in
            yes) regalias=1
                ;;
            no) regalias=0
                ;;
            *) AC_MSG_ERROR([Invalid value for --enable-regalias])
                ;;
        esac
    ],
    [regalias=0]
)

if test "x${regalias}" = "x1"; then
    AC_DEFINE([EC_REGALIAS], [1], [Read alias adresses from register])
    AC_MSG_RESULT([yes])
else
    AC_MSG_RESULT([no])
fi

#------------------------------------------------------------------------------
# Force the reference clock to OP even if unconfigured
#------------------------------------------------------------------------------

AC_MSG_CHECKING([whether to force the reference clock to OP])

AC_ARG_ENABLE([refclkop],
    AS_HELP_STRING([--enable-refclkop],
                   [Force reference clock to OP (default: no)]),
    [
        case "${enableval}" in
            yes) refclkop=1
                ;;
            no) refclkop=0
                ;;
            *) AC_MSG_ERROR([Invalid value for --enable-refclkop])
                ;;
        esac
    ],
    [refclkop=0]
)

if test "x${refclkop}" = "x1"; then
    AC_DEFINE([EC_REFCLKOP], [1], [Force refclk to OP])
    AC_MSG_RESULT([yes])
else
    AC_MSG_RESULT([no])
fi

#------------------------------------------------------------------------------
# Command-line tool
#-----------------------------------------------------------------------------

AC_MSG_CHECKING([whether to build the command-line tool])

AC_ARG_ENABLE([tool],
    AS_HELP_STRING([--enable-tool],
                   [Build command-line tool (default: yes)]),
    [
        case "${enableval}" in
            yes) tool=1
                ;;
            no) tool=0
                ;;
            *) AC_MSG_ERROR([Invalid value for --enable-tool])
                ;;
        esac
    ],
    [tool=1]
)

if test "x${tool}" = "x1"; then
    AC_MSG_RESULT([yes])
else
    AC_MSG_RESULT([no])
fi

AM_CONDITIONAL(BUILD_TOOL, test "x$tool" = "x1")

#------------------------------------------------------------------------------
# mailbox gateway server
#-----------------------------------------------------------------------------

AC_MSG_CHECKING([whether to build the mailbox gateway server])

AC_ARG_ENABLE([mbg],
    AS_HELP_STRING([--enable-mbg],
                   [Build mailbox gateway server (default: yes)]),
    [
        case "${enableval}" in
            yes) mbg=1
                ;;
            no) mbg=0
                ;;
            *) AC_MSG_ERROR([Invalid value for --enable-mbg])
                ;;
        esac
    ],
    [mbg=1]
)

if test "x${mbg}" = "x1"; then
    AC_MSG_RESULT([yes])
else
    AC_MSG_RESULT([no])
fi

AM_CONDITIONAL(BUILD_MBG, test "x$mbg" = "x1")

#------------------------------------------------------------------------------
# Userspace library generation
#------------------------------------------------------------------------------

AC_MSG_CHECKING([whether to build the userspace library])

AC_ARG_ENABLE([userlib],
    AS_HELP_STRING([--enable-userlib],
                   [Generation of the userspace library (default: yes)]),
    [
        case "${enableval}" in
            yes) userlib=1
                ;;
            no) userlib=0
                ;;
            *) AC_MSG_ERROR([Invalid value for --enable-userlib])
                ;;
        esac
    ],
    [userlib=1]
)

if test "x${userlib}" = "x1"; then
    AC_MSG_RESULT([yes])
else
    AC_MSG_RESULT([no])
fi

AM_CONDITIONAL(ENABLE_USERLIB, test "x$userlib" = "x1")

#------------------------------------------------------------------------------
# TTY driver
#------------------------------------------------------------------------------

AC_MSG_CHECKING([whether to build the tty driver])

AC_ARG_ENABLE([tty],
    AS_HELP_STRING([--enable-tty],
                   [Generation of the ec_tty module (default: no)]),
    [
        case "${enableval}" in
            yes) tty=1
                ;;
            no) tty=0
                ;;
            *) AC_MSG_ERROR([Invalid value for --enable-tty])
                ;;
        esac
    ],
    [tty=0]
)

if test "x${tty}" = "x1"; then
    AC_MSG_RESULT([yes])
else
    AC_MSG_RESULT([no])
fi

AM_CONDITIONAL(ENABLE_TTY, test "x$tty" = "x1")
AC_SUBST(ENABLE_TTY,[$tty])

#------------------------------------------------------------------------------
# Slave identification wildcards
#------------------------------------------------------------------------------

AC_MSG_CHECKING([whether to allow identification wildcards])

AC_ARG_ENABLE([wildcards],
    AS_HELP_STRING([--enable-wildcards],
                   [Enable vendor ID / product code wildcards (default: no)]),
    [
        case "${enableval}" in
            yes) wildcards=1
                ;;
            no) wildcards=0
                ;;
            *) AC_MSG_ERROR([Invalid value for --enable-wildcards])
                ;;
        esac
    ],
    [wildcards=0]
)

if test "x${wildcards}" = "x1"; then
    AC_DEFINE([EC_IDENT_WILDCARDS], [1],
        [Use vendor id / product code wildcards])
    AC_MSG_RESULT([yes])
else
    AC_MSG_RESULT([no])
fi

#------------------------------------------------------------------------------
# Redundancy (number of devices)
#------------------------------------------------------------------------------

AC_ARG_WITH([devices],
    AC_HELP_STRING(
        [--with-devices=<NUMBER>],
        [Number of Ethernet devices per master. Default: 1]
    ),
    [
        devices=[$withval]
    ],
    [
        devices=1
    ]
)

AC_MSG_CHECKING([for number of Ethernet devices])

if test "${devices}" -lt 1; then
    AC_MSG_ERROR([Number must be greater zero!])
else
    if test "${devices}" -gt 1; then
        AC_MSG_RESULT([$devices (Redundancy enabled)])
    else
        AC_MSG_RESULT([$devices (Redundancy disabled)])
    fi
fi

AC_DEFINE_UNQUOTED([EC_MAX_NUM_DEVICES], $devices,
    [Max. number of Ethernet devices per master])

#------------------------------------------------------------------------------
# SII assignment
#------------------------------------------------------------------------------

AC_MSG_CHECKING([whether to assign the SII to PDI])

AC_ARG_ENABLE([sii-assign],
    AS_HELP_STRING([--enable-sii-assign],
                   [Enable SII assignment to PDI (default: yes)]),
    [
        case "${enableval}" in
            yes) siiassign=1
                ;;
            no) siiassign=0
                ;;
            *) AC_MSG_ERROR([Invalid value for --enable-sii-assign])
                ;;
        esac
    ],
    [siiassign=1]
)

if test "x${siiassign}" = "x1"; then
    AC_DEFINE([EC_SII_ASSIGN], [1], [Assign SII to PDI])
    AC_MSG_RESULT([yes])
else
    AC_MSG_RESULT([no])
fi

#------------------------------------------------------------------------------
# SII cache
#------------------------------------------------------------------------------

AC_MSG_CHECKING([whether to cache the SII for re-use])

AC_ARG_ENABLE([sii-cache],
    AS_HELP_STRING([--disable-sii-cache],
                   [Disable SII cache for devices with aliases or serials]),
    [
        case "${enableval}" in
            yes) siicache=1
                ;;
            no) siicache=0
                ;;
            *) AC_MSG_ERROR([Invalid value for --enable-sii-cache])
                ;;
        esac
    ],
    [siicache=1]
)

if test "x${siicache}" = "x1"; then
    AC_DEFINE([EC_SII_CACHE], [1], [Cache the SII images to improve ]
        [slave re-scan performance in operational state.])
    AC_MSG_RESULT([yes])
else
    AC_MSG_RESULT([no])
fi

#------------------------------------------------------------------------------
# Quick OP
#------------------------------------------------------------------------------

AC_MSG_CHECKING([whether to allow quick OP])

AC_ARG_ENABLE([quick-op],
    AS_HELP_STRING([--disable-quick-op],
                   [Disable quick return to OP]),
    [
        case "${enableval}" in
            yes) quickop=1
                ;;
            no) quickop=0
                ;;
            *) AC_MSG_ERROR([Invalid value for --enable-quick-op])
                ;;
        esac
    ],
    [quickop=1]
)

if test "x${quickop}" = "x1"; then
    AC_DEFINE([EC_QUICK_OP], [1], [Allow quick return to OP ]
        [from SAFEOP+PD_watchdog.])
    AC_MSG_RESULT([yes])
else
    AC_MSG_RESULT([no])
fi

#------------------------------------------------------------------------------
# Alternate SII firmware loading
#------------------------------------------------------------------------------

AC_MSG_CHECKING([whether to load SII locally if available])

AC_ARG_ENABLE([sii-override],
    AS_HELP_STRING([--enable-sii-override],
                   [Enable SII override (default: no)
--enable-sii-override=/lib/firmware to load directly]),
    [
        case "${enableval}" in
            yes) siioverride=1
                siidir=""
                ;;
            no) siioverride=0
                siidir=""
                ;;
            */) AC_MSG_ERROR([Must omit trailing slash in --enable-sii-override.])
                ;;
            *) siioverride=1
               siidir="${enableval}"
                ;;
        esac
    ],
    [siioverride=0]
    [siidir=""]
)

if test "x${siioverride}" = "x1"; then
    AC_DEFINE([EC_SII_OVERRIDE], [1], [Allow locally loadable SII])
    if test "x${siidir}" = "x"; then
        AC_MSG_RESULT([yes, from hotplug])
    else
        AC_MSG_RESULT([yes, from $siidir])
        AC_DEFINE_UNQUOTED([EC_SII_DIR], ["$siidir"], [Direct SII firmware base path])
    fi
else
    AC_MSG_RESULT([no])
fi

#------------------------------------------------------------------------------
# syslog output in realtime context
#------------------------------------------------------------------------------

AC_MSG_CHECKING([whether to syslog in realtime context])

AC_ARG_ENABLE([rt-syslog],
    AS_HELP_STRING([--enable-rt-syslog],
                   [Enable RT syslog (default: yes)]),
    [
        case "${enableval}" in
            yes) rtsyslog=1
                ;;
            no) rtsyslog=0
                ;;
            *) AC_MSG_ERROR([Invalid value for --enable-rt-syslog])
                ;;
        esac
    ],
    [rtsyslog=1]
)

if test "x${rtsyslog}" = "x1"; then
    AC_DEFINE([EC_RT_SYSLOG], [1], [Output to syslog in RT context])
    AC_MSG_RESULT([yes])
else
    AC_MSG_RESULT([no])
fi

#------------------------------------------------------------------------------
# use loop control registers to open slave ports
#------------------------------------------------------------------------------

AC_MSG_CHECKING([whether to use loop control registers])

AC_ARG_ENABLE([loop-control],
    AS_HELP_STRING([--enable-loop-control],
                   [Use loop control registers (default: no)]),
    [
        case "${enableval}" in
            yes) loopctl=1
                ;;
            no) loopctl=0
                ;;
            *) AC_MSG_ERROR([Invalid value for --enable-loop-control])
                ;;
        esac
    ],
    [loopctl=0]
)

if test "x${loopctl}" = "x1"; then
    AC_DEFINE([EC_LOOP_CONTROL], [1], [Use loop control registers])
    AC_MSG_RESULT([yes])
else
    AC_MSG_RESULT([no])
fi

#------------------------------------------------------------------------------

AC_CONFIG_FILES([
        Doxyfile
        Kbuild
        Makefile
        devices/Kbuild
        devices/Makefile
        devices/ccat/Kbuild
        devices/ccat/Makefile
        devices/cx2100/Kbuild
        devices/cx2100/Makefile
        devices/e1000/Kbuild
        devices/e1000/Makefile
        devices/e1000e/Kbuild
        devices/e1000e/Makefile
        devices/igb/Kbuild
        devices/igb/Makefile
        ethercat.spec
        examples/Kbuild
        examples/Makefile
        examples/dc_rtai/Kbuild
        examples/dc_rtai/Makefile
        examples/dc_user/Makefile
        examples/mini/Kbuild
        examples/mini/Makefile
        examples/rtai/Kbuild
        examples/rtai/Makefile
        examples/rtai_rtdm/Makefile
        examples/rtai_rtdm_dc/Makefile
        examples/tty/Kbuild
        examples/tty/Makefile
        examples/user/Makefile
        examples/xenomai/Makefile
        examples/xenomai_posix/Makefile
        include/Makefile
        lib/Makefile
        m4/Makefile
        master/Kbuild
        master/Makefile
        script/Makefile
        script/ethercat.service
        script/ethercatctl
        script/init.d/Makefile
        script/init.d/ethercat
        script/sysconfig/Makefile
        tool/Makefile
        mailbox_gateway/Makefile
        tty/Kbuild
        tty/Makefile
])
AC_OUTPUT

#------------------------------------------------------------------------------

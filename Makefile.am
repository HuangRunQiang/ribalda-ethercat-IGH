#------------------------------------------------------------------------------
#
#  $Id$
#
#  Copyright (C) 2006-2008  Florian Pose, Ingenieurgemeinschaft IgH
#
#  This file is part of the IgH EtherCAT Master.
#
#  The IgH EtherCAT Master is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License version 2, as
#  published by the Free Software Foundation.
#
#  The IgH EtherCAT Master is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
#  Public License for more details.
#
#  You should have received a copy of the GNU General Public License along
#  with the IgH EtherCAT Master; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
#
#  ---
#
#  The license mentioned above concerns the source code only. Using the
#  EtherCAT technology and brand is only permitted in compliance with the
#  industrial property and similar rights of Beckhoff Automation GmbH.
#
#------------------------------------------------------------------------------

# 设置 aclocal 的标志，指定 m4 目录
ACLOCAL_AMFLAGS = -I m4

# 指定需要进行构建的子目录
SUBDIRS = \
    include \
    script

# 如果启用了内核
if ENABLE_KERNEL
    # 添加 devices 子目录到构建目录列表
    SUBDIRS += devices
    # 添加 master 子目录到构建目录列表
    SUBDIRS += master
endif


# 如果构建工具被启用
if BUILD_TOOL
    # 添加 tool 子目录到构建目录列表
    SUBDIRS += tool
endif

# 如果构建 MBG 被启用
if BUILD_MBG
    # 添加 mailbox_gateway 子目录到构建目录列表
    SUBDIRS += mailbox_gateway
endif

# 如果启用用户库
if ENABLE_USERLIB
    # 添加 lib 子目录到构建目录列表
    SUBDIRS += lib
endif

# 如果启用 TTY
if ENABLE_TTY
    # 添加 tty 子目录到构建目录列表
    SUBDIRS += tty
endif


# 将 examples 子目录添加到构建目录列表中，表示用户空间示例依赖于 lib/
SUBDIRS += examples

# 指定需要打包分发的子目录
DIST_SUBDIRS = \
    devices \
    examples \
    include \
    lib \
    m4 \
    mailbox_gateway \
    master \
    script \
    tool \
    tty


# 指定不需要安装的头文件列表
noinst_HEADERS = \
    globals.h

# 指定额外需要分发但不需要安装的文件列表
EXTRA_DIST = \
    Doxyfile.in \
    FEATURES \
    Kbuild.in \
    README.EoE \
    ethercat.spec \
    ethercat.spec.in


BUILT_SOURCES = \
	Doxyfile \  # BUILT_SOURCES包含需要在构建过程中生成的文件列表，这些文件可能是通过其他命令或脚本生成的
	Kbuild \    # 例如，Doxyfile可能是用于Doxygen文档生成的配置文件，Kbuild可能是用于内核模块构建的配置文件
	ethercat.spec

modules:
	$(MAKE) -C "$(LINUX_SOURCE_DIR)" M="@abs_srcdir@" modules  # 通过执行命令$(MAKE)来编译内核模块

modules_install:
	$(MAKE) -C "$(LINUX_SOURCE_DIR)" M="@abs_srcdir@" \
		INSTALL_MOD_DIR="$(INSTALL_MOD_DIR)" modules_install  # 通过执行命令$(MAKE)来安装内核模块


clean-local:
	$(MAKE) -C "$(LINUX_SOURCE_DIR)" M="@abs_srcdir@" clean

mydist:
	hg log --style=changelog $(srcdir) > ChangeLog
	@REV=`hg id -i $(srcdir)` && \
		$(MAKE) dist-bzip2 distdir=$(PACKAGE)-$(VERSION)-$${REV}

dist-hook:
	if which hg >/dev/null 2>&1; then \
		hg id -i $(srcdir) 2>/dev/null >$(distdir)/revision; \
	fi

mrproper: clean cleandoc
	rm -rf \  # "mrproper"目标用于清理项目中的临时文件和生成的文件，以便进行干净的重新构建
		*~ \  # 删除临时文件
		ChangeLog \  # 删除ChangeLog文件
		Doxyfile \  # 删除Doxyfile文件
		Kbuild \  # 删除Kbuild文件
		Makefile \  # 删除Makefile文件
		Makefile.in \  # 删除Makefile.in文件
		TAGS \  # 删除TAGS文件
		aclocal.m4 \  # 删除aclocal.m4文件
		autoconf \  # 删除autoconf目录
		autom4te.cache \  # 删除autom4te.cache目录
		config.h \  # 删除config.h文件
		config.h.in \  # 删除config.h.in文件
		config.log \  # 删除config.log文件
		config.status \  # 删除config.status文件
		configure \  # 删除configure文件
		configure.in \  # 删除configure.in文件
		libtool \  # 删除libtool文件
		stamp-h1 \  # 删除stamp-h1文件
		ethercat.spec \  # 删除ethercat.spec文件
		script/init.d/ethercat \  # 删除script/init.d/ethercat文件
		m4/*.m4 \  # 删除m4目录下的所有.m4文件
		`find -path ./documentation -prune -o "(" -name Makefile -o -name Makefile.in -o -name Kbuild -o -name .deps -o -name Module.symvers -o -name .tmp_versions ")" -print`  # 通过find命令删除特定的文件和目录

doc:
	doxygen Doxyfile

cleandoc:
	@rm -rf doxygen-output

#------------------------------------------------------------------------------

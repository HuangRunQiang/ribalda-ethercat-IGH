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
   EtherCAT master character device.
*/

/*****************************************************************************/

#include <linux/module.h>
#include <linux/vmalloc.h>
#include <linux/mm.h>

#include "cdev.h"
#include "master.h"
#include "slave_config.h"
#include "voe_handler.h"
#include "ethernet.h"
#include "ioctl.h"

/** 设置为1以启用设备操作调试。
 */
#define DEBUG 0

/*****************************************************************************/

static int eccdev_open(struct inode *, struct file *);
static int eccdev_release(struct inode *, struct file *);
static long eccdev_ioctl(struct file *, unsigned int, unsigned long);
static int eccdev_mmap(struct file *, struct vm_area_struct *);

/** 这是可用于vm_operations_struct的.fault成员的内核版本。
 */
#define PAGE_FAULT_VERSION KERNEL_VERSION(2, 6, 23)

#if LINUX_VERSION_CODE >= PAGE_FAULT_VERSION
static
#if LINUX_VERSION_CODE > KERNEL_VERSION(5, 0, 0)
vm_fault_t
#else
int
#endif
eccdev_vma_fault(
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 11, 0)
        struct vm_area_struct *,
#endif
        struct vm_fault *);
#else
static struct page *eccdev_vma_nopage(
        struct vm_area_struct *, unsigned long, int *);
#endif

/*****************************************************************************/

/** EtherCAT字符设备的文件操作回调函数。
 */
static struct file_operations eccdev_fops = {
    .owner          = THIS_MODULE,
    .open           = eccdev_open,
    .release        = eccdev_release,
    .unlocked_ioctl = eccdev_ioctl,
    .mmap           = eccdev_mmap
};

/** 用于使用ecdevc_mmap()检索的虚拟内存区域的回调函数。
 */
struct vm_operations_struct eccdev_vm_ops = {
#if LINUX_VERSION_CODE >= PAGE_FAULT_VERSION
    .fault = eccdev_vma_fault
#else
    .nopage = eccdev_vma_nopage
#endif
};

/*****************************************************************************/

```cpp
/**
 * @brief 文件句柄的私有数据结构
 */
typedef struct {
    ec_cdev_t *cdev; /**< 字符设备 */
    ec_ioctl_context_t ctx; /**< 上下文 */
} ec_cdev_priv_t;

/*****************************************************************************/

/**
 * @brief 初始化字符设备
 *
 * @param cdev EtherCAT主设备字符设备
 * @param master 父主设备
 * @param dev_num 设备号
 * @return 成功返回0，否则返回负值
 *
 * @details 此函数用于初始化字符设备。它将主设备指针和文件操作结构初始化，并将文件操作结构的所有者设置为当前模块。
 * 然后，它将字符设备添加到字符设备表中。如果添加失败，则打印错误信息。
 */
int ec_cdev_init(
        ec_cdev_t *cdev, /**< EtherCAT主设备字符设备 */
        ec_master_t *master, /**< 父主设备 */
        dev_t dev_num /**< 设备号 */
        )
{
    int ret;

    cdev->master = master;

    cdev_init(&cdev->cdev, &eccdev_fops);
    cdev->cdev.owner = THIS_MODULE;

    ret = cdev_add(&cdev->cdev,
            MKDEV(MAJOR(dev_num), master->index), 1);
    if (ret) {
        EC_MASTER_ERR(master, "添加字符设备失败！\n");
    }

    return ret;
}

/*****************************************************************************/

/**
 * @brief 清除字符设备
 *
 * @param cdev EtherCAT XML设备
 *
 * @details 此函数用于清除字符设备。它从字符设备表中删除字符设备。
 */
void ec_cdev_clear(ec_cdev_t *cdev /**< EtherCAT XML设备 */)
{
    cdev_del(&cdev->cdev);
}
```

/******************************************************************************
 * 文件操作
 *****************************************************************************/

/**
 * @brief 打开字符设备时调用
 *
 * @param inode inode结构指针
 * @param filp 文件结构指针
 * @return 成功返回0，否则返回负值
 *
 * @details 此函数在打开字符设备时调用。它分配并初始化私有数据结构，并将文件的私有数据指针指向该结构。
 */
int eccdev_open(struct inode *inode, struct file *filp)
{
    ec_cdev_t *cdev = container_of(inode->i_cdev, ec_cdev_t, cdev);
    ec_cdev_priv_t *priv;

    priv = kmalloc(sizeof(ec_cdev_priv_t), GFP_KERNEL);
    if (!priv) {
        EC_MASTER_ERR(cdev->master,
                "分配私有数据结构的内存失败。\n");
        return -ENOMEM;
    }

    priv->cdev = cdev;
    priv->ctx.writable = (filp->f_mode & FMODE_WRITE) != 0;
    priv->ctx.requested = 0;
    priv->ctx.process_data = NULL;
    priv->ctx.process_data_size = 0;

    filp->private_data = priv;

#if DEBUG
    EC_MASTER_DBG(cdev->master, 0, "文件已打开。\n");
#endif
    return 0;
}

/*****************************************************************************/

/**
 * @brief 关闭字符设备时调用
 *
 * @param inode inode结构指针
 * @param filp 文件结构指针
 * @return 成功返回0，否则返回负值
 *
 * @details 此函数在关闭字符设备时调用。它释放私有数据结构的内存，并根据需要释放相关资源。
 */
int eccdev_release(struct inode *inode, struct file *filp)
{
    ec_cdev_priv_t *priv = (ec_cdev_priv_t *) filp->private_data;
    ec_master_t *master = priv->cdev->master;

    if (priv->ctx.requested) {
        ecrt_release_master(master);
    }

    if (priv->ctx.process_data) {
        vfree(priv->ctx.process_data);
    }

#if DEBUG
    EC_MASTER_DBG(master, 0, "文件已关闭。\n");
#endif

    kfree(priv);
    return 0;
}

/*****************************************************************************/

/**
 * @brief 发出ioctl()命令时调用
 *
 * @param filp 文件结构指针
 * @param cmd ioctl命令
 * @param arg 参数
 * @return 返回值
 *
 * @details 此函数在发出ioctl()命令时调用。它将调用ec_ioctl()函数来处理命令。
 */
long eccdev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    ec_cdev_priv_t *priv = (ec_cdev_priv_t *) filp->private_data;

#if DEBUG
    EC_MASTER_DBG(priv->cdev->master, 0,
            "ioctl(filp = 0x%p, cmd = 0x%08x (0x%02x), arg = 0x%lx)\n",
            filp, cmd, _IOC_NR(cmd), arg);
#endif

    return ec_ioctl(priv->cdev->master, &priv->ctx, cmd, (void __user *) arg);
}


/*****************************************************************************/

#ifndef VM_DONTDUMP
/** 
 * \def VM_DONTDUMP
 * \brief VM_RESERVED在3.7版本中已经被删除。
 */
#define VM_DONTDUMP VM_RESERVED
#endif

/** 
 * \fn int eccdev_mmap(struct file *filp, struct vm_area_struct *vma)
 * \brief EtherCAT字符设备的内存映射回调函数。
 *
 * 实际的映射将在虚拟内存区域的eccdev_vma_nopage()回调函数中进行。
 *
 * \param filp 文件指针。
 * \param vma 虚拟内存区域结构体指针。
 * \return 总是返回0（成功）。
 *
 * \details 此函数用于将EtherCAT字符设备映射到用户空间的虚拟内存区域。
 * 它将虚拟内存区域的vm_ops成员设置为eccdev_vm_ops，以便在访问虚拟内存区域时调用相应的操作。
 * 此外，它将虚拟内存区域的vm_flags成员设置为VM_DONTDUMP，以防止页面被交换出去。
 * 最后，它将虚拟内存区域的vm_private_data成员设置为priv，以便在操作中使用私有数据。
 */
int eccdev_mmap(struct file *filp, struct vm_area_struct *vma)
{
    ec_cdev_priv_t *priv = (ec_cdev_priv_t *) filp->private_data;

    EC_MASTER_DBG(priv->cdev->master, 1, "mmap()\n");

    vma->vm_ops = &eccdev_vm_ops;
    vma->vm_flags |= VM_DONTDUMP; /* 页面将不会被交换出去 */
    vma->vm_private_data = priv;

    return 0;
}

/*****************************************************************************/

#if LINUX_VERSION_CODE >= PAGE_FAULT_VERSION

/** 
 * \fn static vm_fault_t eccdev_vma_fault(struct vm_area_struct *vma, struct vm_fault *vmf)
 * \brief 虚拟内存区域的页面错误回调函数。
 *
 * 在使用ecdev_mmap()检索到的虚拟内存区域上的第一次访问时调用。
 *
 * \param vma 虚拟内存区域结构体指针。
 * \param vmf 错误数据结构体指针。
 * \return 成功返回零，否则返回负的错误代码。
 *
 * \details 此函数用于处理虚拟内存区域的页面错误。它首先检查偏移量是否超出了私有数据的大小，
 * 如果超出了范围，则返回VM_FAULT_SIGBUS表示总线错误。然后，它将偏移量转换为页面，并获取页面的引用。
 * 最后，它将页面指针赋值给vmf->page，以便内核可以将该页面映射到用户空间。
 */
static
#if LINUX_VERSION_CODE > KERNEL_VERSION(5, 0, 0)
vm_fault_t
#else
int
#endif
/**
 * @brief 执行虚拟内存区域的错误处理函数。
 *
 * @param vma 虚拟内存区域结构体指针。
 * @param vmf 虚拟内存错误结构体指针。
 * @return 如果虚拟内存区域的错误处理仍在进行中，则返回1；否则返回0。
 *
 * @details 此函数用于执行虚拟内存区域的错误处理。它根据虚拟内存错误结构体中的偏移量计算页面的地址。
 * 如果偏移量超出了私有数据的大小，则返回VM_FAULT_SIGBUS表示总线错误。
 * 否则，它将页面指针赋值给虚拟内存错误结构体中的page成员，并获取页面的引用。
 * 最后，它打印调试信息，包括虚拟地址、偏移量和页面指针。
 */
int eccdev_vma_fault(struct vm_area_struct *vma, struct vm_fault *vmf)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 11, 0)
    struct vm_area_struct *vma = vmf->vma;
#endif
    unsigned long offset = vmf->pgoff << PAGE_SHIFT;
    ec_cdev_priv_t *priv = (ec_cdev_priv_t *) vma->vm_private_data;
    struct page *page;

    if (offset >= priv->ctx.process_data_size) {
        return VM_FAULT_SIGBUS;
    }

    page = vmalloc_to_page(priv->ctx.process_data + offset);
    if (!page) {
        return VM_FAULT_SIGBUS;
    }

    get_page(page);
    vmf->page = page;

    EC_MASTER_DBG(priv->cdev->master, 1, "虚拟内存错误处理, 虚拟地址 = %p,"
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4,10,0))
            " 偏移量 = %lu, 页面 = %p\n", (void*)vmf->address, offset, page);
#else
            " 偏移量 = %lu, 页面 = %p\n", vmf->virtual_address, offset, page);
#endif

    return 0;
}

#else

/** 
 * \fn struct page *eccdev_vma_nopage(struct vm_area_struct *vma, unsigned long address, int *type)
 * \brief 虚拟内存区域的无页回调函数。
 *
 * 在使用ecdev_mmap()检索到的虚拟内存区域上的第一次访问时调用。
 *
 * \param vma 虚拟内存区域结构体指针，由内核初始化。
 * \param address 请求的虚拟地址。
 * \param type 类型输出参数。
 * \return struct page* 指向页面的指针。
 *
 * \details 此函数用于处理虚拟内存区域的无页错误。它首先计算偏移量，然后检查偏移量是否超出了私有数据的大小。
 * 如果超出了范围，则返回NOPAGE_SIGBUS表示总线错误。然后，它将偏移量转换为页面，并获取页面的引用。
 * 最后，它将页面指针赋值给type参数，以便在内核中进行进一步处理。
 */ 
struct page *eccdev_vma_nopage(
        struct vm_area_struct *vma, /**< 虚拟内存区域结构体指针，由内核初始化。 */
        unsigned long address, /**< 请求的虚拟地址。 */
        int *type /**< 类型输出参数。 */
        )
{
    unsigned long offset;
    struct page *page = NOPAGE_SIGBUS;
    ec_cdev_priv_t *priv = (ec_cdev_priv_t *) vma->vm_private_data;
    ec_master_t *master = priv->cdev->master;

    offset = (address - vma->vm_start) + (vma->vm_pgoff << PAGE_SHIFT);

    if (offset >= priv->ctx.process_data_size)
        return NOPAGE_SIGBUS;

    page = vmalloc_to_page(priv->ctx.process_data + offset);

    EC_MASTER_DBG(master, 1, "无页错误回调函数 vma, address = %#lx,"
            " offset = %#lx, page = %p\n", address, offset, page);

    get_page(page);
    if (type)
        *type = VM_FAULT_MINOR;

    return page;
}

#endif

/*****************************************************************************/

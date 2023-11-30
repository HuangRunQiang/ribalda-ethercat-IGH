Etherlab主补丁集20190904

这是由Gavin Lambert gavin.lambert@tomra.com维护的非官方补丁集。

该补丁集中的补丁由多个作者贡献；有关更多信息，请查看补丁本身。

该补丁集当前基于IgH EtherCAT Master库的默认分支的修订版本33b922，还包括来自stable-1.5分支的附加提交，最高修订版本为0c011d。

该补丁集经过测试，应该可以安全使用。然而，补丁和补丁集都不是稳定的，这意味着将来的补丁集版本可能会添加、删除、重新排序或修改现有的补丁；尽管除了解决错误之外，通常只会在重新基于上游更改时发生这种情况。

在某些情况下，您可能只想使用补丁的子集。如果您在任何给定的补丁之前停止（在应用所有先前的补丁之前并且在之后不应用任何补丁），代码应该可以编译和运行。根据所选的各个补丁之间的依赖关系和重叠，其他补丁组合可能会产生模糊或无法应用或无法编译或工作的情况。补丁已经组织成目录，既可以减少混乱，又可以提供有关哪些补丁彼此最相关的附加上下文信息（在“features”目录下的补丁比其他补丁更可选，但如果省略或按照“series”文件中指定的顺序应用，则可能仍会导致模糊或应用失败）。

对补丁集的反馈应该发布到etherlab-dev邮件列表。请记住提及您使用的补丁集的版本。
实时API兼容性

一些补丁会改变内核和用户应用程序之间的ABI。在使用ecrt.h API时，重要的是首先验证接口版本，例如：

if (ecrt_version_magic() != ECRT_VERSION_MAGIC) {
    fprintf(stderr, "Expecting EtherCAT API version %x but found %x.\n",
            ECRT_VERSION_MAGIC, ecrt_version_magic());
    exit(1);
}

如果在应用程序中出现此错误，意味着您需要重新编译Etherlab模块或您的应用程序，或者两者都需要重新编译。

用户空间库已经执行此操作，以验证内核和用户空间库本身之间的兼容性；但是，在您的用户空间应用程序中仍然应该这样做，以验证用户空间库与您的应用程序之间的兼容性，或者在内核空间应用程序中验证EtherLab模块与您自己的模块之间的兼容性。

这依赖于更新ECRT_VERSION_MAGIC值，每当在ecrt.h中的结构中进行更改时。由于这个补丁集中的几个补丁进行了此类修改，但您可能不会一次应用所有补丁，因此这主要取决于您自己的判断，而不包含在各个补丁中（如果按照原始顺序应用可能会导致冲突）；但是，补丁集中确实包含了一个单独的补丁，将该值更新为与上游源不同的值，以防您忘记。
Ioctl兼容性

类似地，当使用ioctl接口时，您还应验证此接口的接口版本，使用以下代码：

ec_ioctl_module_t mod;
if (ioctl(fd, EC_IOCTL_MODULE, &mod) < 0) {
    perror("EC_IOCTL_MODULE");
    exit(1);
}
if (mod.ioctl_version_magic != EC_IOCTL_VERSION_MAGIC) {
    fprintf(stderr, "Expecting EtherCAT ioctl version %u but found %u.\n",
            EC_IOCTL_VERSION_MAGIC, mod.ioctl_version_magic);
    exit(1);
}

这里，fd是已打开的任何EtherCAT主设备的描述符。同样，如果出现此错误，意味着您需要重新编译相关组件。

这依赖于EC_IOCTL_VERSION_MAGIC值的更新，每当对IOCTL或相关结构进行更改时。虽然一些补丁进行了此类更改，但它们没有在自己的补丁中修改版本；而是有一个单独的补丁在更新它。
使用补丁

    如果您尚未安装Mercurial，请安装它，例如：

     sudo apt install mercurial

    创建或编辑您的~/.hgrc文件，并确保其至少具有以下内容：

     [extensions]
     mq =

    克隆主要的上游存储库：

     hg clone -u 33b922ec1871 http://hg.code.sf.net/p/etherlabmaster/code etherlab

    克隆补丁存储库：

     hg clone http://hg.code.sf.net/u/uecasm/etherlab-patches etherlab/.hg/patches

    应用补丁：

     cd etherlab
     hg qpush -a

    现在您已经准备好像往常一样进行构建：

     ./bootstrap
     ./configure --help

    然后选择您希望的选项，并按照INSTALL文件中的其余说明进行操作。

从旧版本的补丁更新

推荐的方法是删除旧的克隆并创建新的克隆，按照新的补丁集的说明进行操作。这是获得与预期配置完全相同的最简单方法。

或者（为了节省一些网络流量），您可以尝试按如下方式进行就地更新：

cd etherlab
hg qpop -a
hg pull -u --mq

此时，您将拥有新的补丁；阅读新的.hg/patches/README.md以查看其是否基于相同的上游。如果需要，执行以下操作更新上游：

hg pull
hg update 33b922ec1871

（请记住使用新README中的哈希值。）最后，使用以下命令应用新的补丁：

hg qpush -a
autoreconf

省略某些补丁

在发出hg qpush -a命令之前，您可以编辑series文件，并注释掉（使用前导#）或删除某些行以排除这些补丁。您还可以选择使用特殊的“保护”来仅有条件地应用某些补丁，尽管这可能与其他补丁管理系统不兼容。

请注意，补丁集的结构假定所有补丁都按照series文件中指定的顺序应用；如果跳过某些补丁或按照不正确的顺序应用补丁，则可能会出现模糊或失败的块，并且某些代码可能无法编译。
使用其他版本控制系统

在按照上述方法更新HG工作副本后，您可能希望将结果导出到另一个版本控制系统以用于主要应用程序。有许多方法可以实现这一点，但最简单的方法就是将结果源代码（不包括.hg目录）复制到其他源代码树中，并将其作为与先前版本的单一增量提交进行提交，然后重新应用您在新源代码之上的任何其他自定义补丁。某些版本控制系统可能有其他推荐做法，例如供应商分支或将自定义补丁作为新源代码上的单独提交导入。

虽然我们建议按照上述Mercurial方式应用补丁，但也可以通过其他方法应用它们，例如使用quilt在源代码tarball上应用补丁，或将相同tarball导入Git，然后将每个补丁作为单独的git提交导入。无论哪种情况，为了获得最佳结果，您应该遵循series文件中指定的应用顺序。

例如，假设您有一个已应用了所有所需补丁的HG工作副本，您可以通过发出命令hg export --git -r NNNN: > all.patch将其导出为与Git兼容的补丁文件（可以使用git am作为一系列提交导入），其中NNNN是第一个补丁的ID（可以从hg log中获取；无法提供示例，因为它会随着主存储库上的提交添加而改变，即使是在不同的分支上）。

如果由于某种原因您真的不想首先在Mercurial工作副本中应用补丁，可以运行附带的mkmbox.sh脚本生成一个大致等效的mbox文件，然后使用git am --whitespace=nowarn -m all.mbx导入它（-m是可选的，但是在提交消息中包含原始补丁名称是为了方便起见）。您必须先导入并提交了HG更改集的原始上游源。请注意，该脚本与保护不兼容，尽管它可以处理已删除或已注释的行。
E1000E警告

如果您在Linux 4.4或更高版本上使用基于主板的e1000e适配器和ec_e1000e驱动程序，则已知mei_me硬件驱动程序的最近更改可能会导致某些硬件和内核配置出现问题。

插入式e1000e卡似乎不受影响，ec_generic驱动程序也不受影响（即使在使用受影响的适配器时）。

观察到的行为是，当在初始引导时一切正常通信时，如果您断开然后重新连接主站的EtherCAT网络电缆（或重新启动第一个从站），所有EtherCAT数据报都将超时；即使在建立连接后。重新插入电缆没有任何区别，但是重新启动etherlab服务将在下次链接断开之前暂时恢复。

为了解决受影响系统的问题，请创建/etc/udev/rules.d/20-mei.rules文件，其中包含以下内容：

ACTION=="add",KERNEL=="mei0",ATTR{../../power/control}="on"

在不包含Intel MEI硬件的系统上包含此内容是完全无害的。在包含Intel MEI硬件的系统上，这将禁用此设备的电源挂起，这不太可能对运行EtherCAT的系统造成任何问题。
补丁目录

补丁应按照以下顺序从目录中应用：

    stable
    此目录包含在stable-1.5分支上进行的提交，尚未合并到主存储库的default分支中。（因此，这些提交比其他补丁略微“官方”。）

    linux
    此目录包含修复在较新版本的Linux内核下的编译错误的补丁。但不包括新的设备驱动程序。

    devices
    此目录包含以太网设备驱动程序的补丁或对其他内核版本的支持。请注意，大多数情况下，新的驱动程序版本仅保证编译（并且原始源与相应内核中的源代码匹配）；尽管我们已经尽力将相关的EtherCAT修改移植到驱动程序中，但它们尚未经过测试，可能存在错误或遗漏，并且驱动程序可能无法正常工作或根本无法工作。

    base
    此目录包含所有修复错误的补丁和其他重要内容。

    features
    此目录包含新功能或类似改进，通常是可选的。

补丁列表
stable/*

这些补丁没有单独记录，但如上所述，由尚未合并到默认分支的主存储库中的stable-1.5分支上进行的多个提交组成。

当前，此处包含的一些重要更改是更新ccat驱动程序和添加igb驱动程序。
linux/0001-debugif-3.17.patch

修复了针对Linux 3.17及更高版本中的debug-if中的alloc_netdev调用。
linux/0002-kern_cont.patch

将printk调用中的KERN_CONT标记添加到以前行的printk调用中。

虽然这已经是建议的做法了，但大约在内核4.9左右，如果您不这样做，它开始打印错误的换行符。这尤其会导致ethercat debug 1的输出变得混乱。
linux/0003-vm_fault-4.10.patch

修复了从内核4.10开始的编译错误。

注意：上游（在补丁stable/0007中）从此代码中删除了VM地址，这也解决了相关的错误。现在，此补丁将其恢复；不知道它是否被删除，因为它是多余的或不重要的，或者只是因为这比修复它更容易。
linux/0004-signal-4.11.patch

修复了在内核4.11上的构建问题。
linux/0005-tty-4.15.patch

针对内核4.15及更高版本的TTY模块进行了定时器API更改和其他修复。
devices/0001-update.patch

修复了上游设备更新脚本中的一些不一致（在稳定补丁之后）。
devices/0002-update.patch

对update.sh设备辅助脚本进行了微小调整，以使拒绝更易于理解。
devices/0003-e1000e-link-detection.patch

修复了3.10-3.16版本的e1000e驱动程序中的链接检测问题。
devices/0004-linux-3.18.patch

将8139too、r8169、e100、e1000和e1000e驱动程序更新到内核3.18。（igb驱动程序最初是为此内核提供的。）
devices/0005-linux-4.1.patch

将8139too、r8169、e100、e1000、e1000e和igb驱动程序更新到内核4.1。
devices/0006-linux-4.4.patch

修复了内核4.4版本中稳定版本的igb驱动程序中的一些错误。
devices/0007-linux-4.9.patch

将8139too、r8169、e100、e1000、e1000e和igb驱动程序更新到内核4.9。
devices/0008-linux-4.14.patch

将8139too、r8169、e100、e1000、e1000e和igb驱动程序更新到内核4.14。
devices/0009-linux-4.19.patch

将8139too、e100、e1000、e1000e和igb驱动程序更新到内核4.19。不包括r8169。
devices/0010-cx2100-2.6.patch

为Linux 2.6.32（仅限）添加了CX2100设备驱动程序。
devices/0011-cx2100-4.9.patch

为Linux 4.9（仅限）添加了（未经测试的）CX2100设备驱动程序。
devices/0012-e1000-unused-variable.patch

修复了e1000驱动程序中的未初始化变量警告。
devices/0013-r8152-4.9.patch

支持Realtek RTL8152/RTL8153的Linux 4.9。
devices/0014-r8152-3.18.patch

支持Realtek RTL8152/RTL8153的Linux 3.18。
devices/0015-r8152-4.4.patch

支持Realtek RTL8152/RTL8153的Linux 4.4。
base/0000-version-magic.patch

增加接口版本。

这只是一个占位符，假设您将应用更改ecrt和ioctl接口的ABI的补丁。

它不应直接应用到上游（应使用不同的数字），如果您一次只应用补丁的子集（并且希望知道您是否有不匹配的库和应用程序代码，并且需要重新编译），则可以在应用不同的补丁集时增加版本，以便在应用某个集合时，如果忘记重新编译某些内容，应用程序将给出错误。
base/0001-Distributed-Clock-fixes-and-helpers.patch

添加了ecrt_master_setup_domain_memory和ecrt_master_deactivate_slaves；修复了应用程序选择的参考时钟。
base/0002-Distributed-Clock-fixes-from-Jun-Yuan.patch

解决了与从站DC同步相关的一些时间问题。
base/0003-print-sync-signed.patch

更正了DC同步日志消息，使用带符号格式，因为它可能具有负值。
base/0004-fix-eoe-clear.patch

修复了从站处置时EoE请求取消的拼写错误。
base/0005-disable-eoe.patch

修复了使用--disable-eoe时的编译问题。
base/0006-avoid-ssize_t.patch

修复了一个不需要使用ssize_t的错误，该错误在某些环境中导致编译错误。
base/0007-replace-fprintf-calls-with-EC_PRINT_ERR.patch

使用用户空间RTAI通过RTDM时，避免使用fprintf(stderr)。
base/0008-read-reference-slave-clock-64bit-time.patch

添加了用于排队和读取参考从站时钟的64位时间的功能。主要用于诊断。
base/0009-Avoid-changing-running-slaves-DC-offset.patch

如果网络在主应用程序运行时重新扫描（例如由于响应从站的数量发生变化），则没有任何补丁的情况下，重新配置过程可能会更新从站的System Time Offset寄存器，这会导致从站的DC时钟发生立即的步进变化，有时可能导致它错过脉冲并可能完全停止。

    如果给定从站的System Time Offset和Delay寄存器已经正确，它将根本不会对它们进行写入。
    如果它想要更改System Time Offset寄存器，但从站已经处于SAFEOP或OP状态，则它不会更改它（但仍将使用传输延迟更新System Time Delay）。
        修改System Time Offset寄存器（0x0920）会导致从站的System Time发生突变，这可能导致它错过下一个同步时间（对于32位从站，可能需要4秒才能恢复（可能不同步），对于64位从站，可能永远不会再次同步）。
        修改System Time Delay寄存器（0x0928）只是更改它在正常时间同步数据报循环中使用的值（据我所知）；这是漂移补偿的，因此它将逐渐漂移到正确的时间，而不是立即跳转到正确的时间，因此不应引起上述问题。
        补丁base/0001和base/0002使主站更有可能要更新System Time Offset（尽管它们确实改进了其他方面，对于初始启动情况非常好-只是对于OP期间的重新配置情况来说不太好）。
    如果它更新了偏移量（仅当从站不处于SAFEOP或OP状态时才会发生），它还将写入寄存器0x0930以重置漂移滤波器（这在数据手册中推荐）。

现在，这比以前的补丁版本更干净和安全（以前的版本会禁用和重新启用同步输出，只有在从站支持并启用AssignActivate 0x2000时才有效，并且可能会错过一些脉冲），并且对于一般用途来说更好，因为这允许正在运行的从站始终使用漂移补偿来逐渐调整其时钟，而不是立即跳变（虽然从PREOP进行配置的从站仍然可以立即跳变）。

然而请记住，我使用DC的方式不会注意到小的时间错误。因此，我希望那些更广泛使用DC的人（尤其是使用要挑剔的电机从站）可以验证它。
base/0010-Make-busy-logging-a-little-less-irritating.patch

使[a2701a]“内部SDO请求现在与外部请求同步。”少一些噪音。
[a2701a]:https://sourceforge.net/p/etherlabmaster/code/ci/a2701a/tree/
base/0011-Reduced-printing-to-avoid-syslog-spam.patch

删除了lib/master.c中的不必要的printf和ioctl.c中的printk，以避免syslog垃圾邮件。如果在系统软件中的ECAT_GetSlaveInfo中调用了不存在的从站，则会在stderr中获得printf和syslog中的printk。但是，在某些应用程序中，每个IO卡上每10ms都会执行此操作。已返回的错误代码足以进行正确的错误处理。
base/0012-Added-newline-to-syslog-message-MAC-address-derived.patch

为syslog消息“MAC address derived ...”添加了换行符。
base/0013-Do-not-reuse-the-index-of-a-pending-datagram.patch

不要重用挂起数据报的索引，以防止损坏。
base/0014-Fix-NOHZ-local_softirq_pending-08-warning.patch

修复了调试接口中的NOHZ内核警告。
base/0015-Clear-configuration-on-deactivate-even-if-not-activa.patch

即使在调用ecrt_master_activate之前没有调用ecrt_master_deactivate，也在调用ecrt_master_deactivate时清除主站配置（尽管仍会记录警告）。特别是，这允许在开始实时循环之前创建从站配置和请求对象，然后将它们丢弃，例如在实时循环开始之前使用异步API。

请注意，这不适用于ecrt_master_deactivate_slaves（由补丁base/0001添加），尽管它们在其他方面有很多共同之处；这没有技术原因，只是不是预期的使用模式。
base/0016-If-enable-rtmutex-use-rtmutexes-instead-of-semaphore.patch

配置选项--enable-rtmutex使主站在使用常规信号量而不是rtmutex时使用rtmutex。这提供了优先级继承以减少锁争用时的延迟。

以前的补丁已合并到此补丁中，然后进行了修改以解决死锁问题，尤其是在使用EoE和/或RTAI时。
base/0017-Master-locks-to-avoid-corrupted-datagram-queue.patch

添加了锁来保护可能从多个并发应用程序任务中调用的函数。在某些情况下，应用程序可能仍然需要外部锁。

当编译为RTDM时，会跳过Linux锁，以避免在从RT任务调用时出现二次上下文和死锁。如果您有多个RT任务，则您有责任使用适当的RTAI/Xenomai锁。

##### base/0018-Use-call-back-functions.patch

确保在使用IOCTL接口时使用带IO锁支持的回调函数。

我不确定此补丁是否对RTAI安全，欢迎反馈。

##### base/0019-Support-for-multiple-mailbox-protocols.patch

在读取从站邮箱时，可能会以任何顺序返回任何先前请求的响应，或者返回未经请求的响应（例如，EoE，CoE紧急情况）。此补丁根据协议类型将数据路由到FSM，以避免FSM混淆，只要同一时间对于给定从站的任何给定协议只有一个请求在进行中。

##### base/0020-eoe-ip.patch

修复了IP状态机（在稳定补丁中添加）和数据包状态机之间的EoE邮箱冲突。

还更正了IP地址包的格式。

##### base/0021-Await-SDO-dictionary-to-be-fetched.patch

在SDO字典完成获取之前不处理请求，以避免超时。

##### base/0022-Clear-slave-mailboxes-after-a-re-scan.patch

在重新扫描从站（从而丢弃任何先前挂起的FSM）时，明确清除从站邮箱，以避免被陈旧的响应混淆。

##### base/0023-Skip-output-statistics-during-re-scan.patch

在重新扫描期间，预计会出现UNMATCHED数据报；不要报告它们。

##### base/0024-Sdo-directory-now-only-fetched-on-request.patch

仅在工具明确请求时才获取SDO字典。这样可以提高扫描性能，因为获取字典可能非常耗时，并且在初始配置之后通常不需要。

##### base/0025-Ignore-mailbox-settings-if-corrupted-sii-file.patch

如果邮箱设置似乎来自空白EEPROM，则忽略SII指定的邮箱设置。

##### base/0026-EoE-processing-is-now-only-allowed-in-state-PREOP.patch

禁止在INIT、BOOT和无效状态下使用EoE，以防止错误。

##### base/0027-Prevent-abandoning-the-mailbox-state-machines-early-.patch

修复了邮箱FSM的退出条件-以前它们返回它们是否正在发送数据报，而父级则假定如果它们不想发送数据报，则它们已经完成。由于之前的补丁现在可能导致空闲周期，因此这可能会导致意外的早期退出，因此它们现在明确返回它们是否已完成。

##### base/0028-ec_master_exec_slave_fsms-external-datagram-fix.patch

在不实际使用从站环的情况下不消耗从站环数据报。

##### base/0029-Tool-Withdraw-EEPROM-control-for-SII-read-write.patch

除非PDI锁定了EEPROM，否则在所有正常的SII读写操作中撤销PDI EEPROM控制。但是，由PDI锁定的EEPROM仍然可以使用SII读写操作的强制选项进行撤销。

##### base/0030-Print-redundancy-device-name-with-ring-positions-as-.patch

更改了记录从站环位置的多个位置，以显示链接名称（“main”或“backup”）。这是因为当主端口冗余处于活动状态时，每个链接上响应的第一个从站以前都是“0-0”；现在一个是“0-main-0”，另一个是“0-backup-0”。即使未启用冗余，这仍会影响日志输出。

##### base/0031-ext-timeout.patch

当ext_ring（主/从FSM，而不是应用程序域）数据报不适合当前周期时，根据填充数据报的时间检查超时-但是实际上从未设置此原始时间，导致检查自上次使用该数据报“槽”发送其他数据报以来的时间。这导致所有此类数据报都超时，而不是按预期推迟到下一个周期。

##### base/0032-dc-sync1-offset.patch

在计算SYNC1寄存器值时，使用`ecrt_slave_config_dc`的`sync1_cycle`和`sync1_shift`参数的组合（为了向后兼容，您可以继续在`sync1_cycle`中指定两个偏移量的组合，并将`sync1_shift`保持为0）。

还解决了在b101637f503c中引入的对于只想移位SYNC1而不使用从属周期的从站，或者想将从属周期与移位组合的从站的DC启动时间的错误计算。

##### features/xenomai3/0001-Support-Xenomai-version-3.patch

支持Xenomai v3中的Alchemy RTDM。

##### features/net-up/0001-Add-support-for-bringup-up-network-interface-when-st.patch

在启动EtherCAT时添加了支持网络接口启动（例如，当使用`generic`驱动程序时）。

##### features/sii-cache/0001-Improved-EtherCAT-rescan-performance.patch

如果从站具有别名或序列号（因此即使其位置发生更改，仍可以在网络上唯一标识它），则可以将其SII缓存以避免重新读取它。这提高了扫描性能，特别是在较大的网络上。

##### features/sii-cache/0002-Redundancy-name.patch

由于先前的补丁引入了另一个打印从站地址的地方，因此将其扩展到base/0028。

##### features/sii-cache/0003-rescan-check-revision.patch

1. 可以通过在配置时使用`--disable-sii-cache`来禁用SII缓存和重用行为，而不需要修改头文件。
2. 在使用缓存版本之前，还验证修订号（这解决了设备固件升级时的一些问题）。
3. 如果别名和序列号都读取为0，则不再读取供应商/产品/修订号，因为现在已知SII不在缓存中。
4. 将几个相似的状态合并为一个。

##### features/reboot/0001-Add-command-to-request-hardware-reboot-for-slaves-th.patch

为工具添加了“reboot”命令，以执行支持此功能的从站的软件重启（通过寄存器0x0040）。

##### features/quick-op/0001-After-a-comms-interruption-allow-SAFEOP-OP-directly-.patch

允许SAFEOP+ERR状态下具有同步看门狗超时状态（0x001B）的从站直接转换回OP状态（不经过PREOP，不重新配置）。这使得设备可以更快地从不会导致网络结构更改的通信中断中恢复。

作为副作用，它还缓存了从站的最后一个AL状态代码，这可能可以提供给应用程序使用（尽管此补丁不会这样做）。

默认情况下启用此功能，但是如果它与某些DC从站发生问题，可以在配置时使用`--disable-quick-op`来禁用它。

##### features/status/0001-Adding-some-more-state-to-avoid-calling-the-more-exp.patch

向ecrt.h接口添加了一些额外字段，以便应用程序可以更多地了解正在进行的情况以及请求是否可能需要一段时间才能开始。

##### features/status/0002-Detect-bypassed-ports-timestamp-not-updated.patch

在进行延迟测量时，检测完全绕过的端口（例如，由从站为基础的冗余环拓扑），允许忽略它们而不是使用无效的时间戳。

##### features/status/0003-Calculate-most-likely-upstream-port-for-each-slave.patch

指示每个设备的最有可能的上游端口。通常情况下，这应该始终是0，但是如果设备错误地连接到了2-3个端口（例如，意外连接或主动冗余的结果），则可能是1-3。用于诊断。

##### features/status/0004-slave-config-position.patch

为`ecrt_slave_config_state`返回的结构添加了`position`字段。这允许您从相对别名：偏移地址快速获取从站的环位置，从而允许您调用其他需要此功能的API（例如`ecrt_master_get_slave`）。

请注意，只有在`online`为true时，`position`才有效，并且该值可能过时（即，从站在此期间可能已经移动到其他位置）。

##### features/rdwr/0001-Add-register-read-write-support.patch

* 添加了`ecrt_reg_request_readwrite` API，以执行寄存器读取+写入请求（将寄存器数据写入并读取先前的值，使用单个数据报）。

* 在工具中添加了`reg_rdwr`命令，它从命令行执行相同的功能。

这对于确保数据一致性并避免丢失更新非常有用，特别适用于错误计数的读取+清除。

##### features/rdwr/0002-Display-more-info-about-the-register-requests-in-pro.patch

为寄存器请求添加了一些额外的1级日志记录。

##### features/complete/0001-Support-SDO-upload-via-complete-access.patch

* 添加了`ecrt_master_sdo_upload_complete`同步API，通过SDO完全访问从从站读取整个对象（如果从站支持，尽管大多数都支持）。

* 在工具中的`upload`命令中添加了完全访问功能。它默认为`octet_string`数据类型，以便更容易将其写入文件或进行其他处理（例如，`ethercat upload -p0 INDEX | hd`将以良好的格式显示数据）。

与现有的下载功能一样，仅支持来自子索引0的完全访问。不要忘记子索引0是8位值，后面跟着8位填充；不要将其视为16位值。

##### features/complete/0002-add-sdo-write-with-size.patch

添加了`ecrt_sdo_request_write_with_size` API，允许SDO请求写入与立即前一个读取不同大小的数据。请求的写入大小始终必须小于或等于创建请求时指定的大小；如果不是这样，API将给出错误，这意味着您已经提交了缓冲区溢出。

结合现有的`ecrt_sdo_request_index` API，这允许您为相同从站上的任何SDO重用一个请求对象，前提是初始大小足够大。

以前，您必须使用至少一个对象来处理不同的对象大小，因为除了在创建时设置写入大小之外，没有其他方法，并且在进行读取时还会更改写入大小。

##### features/complete/0003-sdo-requests-complete.patch

在SDO请求中支持SDO完全访问。

添加了`ecrt_slave_config_create_sdo_request_complete`。
添加了`ecrt_sdo_request_index_complete`。

在以前的补丁集中，这些作为附加参数实现，而不是单独的方法；虽然我仍然认为这是更清晰的API（也许最好与上游集成），但此版本与现有代码更兼容。

##### features/master-redundancy/0001-e1000e-fix-watchdog-redundancy.patch

更改e1000e驱动程序的行为：

* 看门狗任务在Linux后台线程上执行，而不是在RT线程上执行。（该任务可能较慢。）
* 无论是否接收到数据包，都会执行看门狗任务。

后者对于正确支持主端口冗余（`--with-devices=2`）非常重要，因为在这种情况下，一个端口仅发送数据包，另一个端口仅接收数据包。如果没有进行此更改，链接检测在这种情况下无法正常工作。

此补丁可能不适用于RTAI。

##### features/sii-file/0001-load-sii-from-file.patch

添加了使用文件覆盖从站的SII数据的功能（如果从站的SII EEPROM太小而无法容纳正确的数据）。

* 默认情况下禁用此功能。
* 在配置时，可以使用`--enable-sii-override`来激活它，使用标准的udev/hotplug查找过程。
* 在配置时，可以使用`--enable-sii-override=/lib/firmware`（或其他路径）来使用直接加载方法来激活它。
* 它将按预期与features/sii-cache合作，尽管请注意，它不像它本可以那样高效（它将重新加载features/sii-cache已读取的一些值时检查SII缓存；但是，尝试改进此将使代码变得非常笨拙）。

##### features/diag/0001-ethercat-diag.patch

为工具添加了“diag”命令，查询从站的错误计数寄存器，以帮助定位丢失的链接和其他网络故障。

**注意**：上游还添加了类似的“crc”命令；它们并不完全相同。也许在某个时候应该将其与“crc”合并，然后“diag”命令将消失。

##### features/diag/0002-diag-readwrite.patch

修改了先前的补丁，使用feature `rdwr`的`reg_rdwr`功能来改进读取+重置请求的原子性。

##### features/foe/0001-fsm_foe-simplify.patch

* 从FSM_foe中删除了一些冗余字段；其中一些是未使用的复制/粘贴保留物，而另一些是在读操作和写操作之间不必要地重复，而这些操作无法同时进行。

* 修复了传入数据超过提供的缓冲区时的情况，以正确终止状态机，而不是留下悬空状态。尽管请注意，这仍然会使FoE对话本身悬空，因此如果发生这种情况，则在下一个请求上可能会收到错误。

##### features/foe/0002-foe-password.patch

* 添加了支持在读取或写入请求中发送FoE密码的功能。

* 为`foe_read`命令实现了`-o`选项（已记录但未实现）。

* 使`foe_read`后面的ioctl实际使用调用方请求的缓冲区大小（而不是硬编码的值）；尽管请注意，`foe_read`本身仍然使用自己的硬编码值（但是它更大，因此现在应该可以读取更大的文件）。有可能在内存有限的嵌入式系统上的用户需要减小此值，但是就RAM大小而言，它仍然相当保守。

##### features/foe/0003-foe-requests.patch

将FoE传输请求转换为公共的`ecrt_`* API，类似于SDO请求。

主要是为了使FoE传输成为非阻塞的，以便可以从同一请求线程并发地从多个从站进行传输（以前只能使用单独的线程，因为唯一的API是阻塞的）。请注意，由于补丁base/0015，您可以调用`ecrt_master_deactivate()`在完成时删除这些请求，即使尚未调用`ecrt_master_activate()`。

这可能还有一个副作用，即现在可以从实时上下文启动和监视FoE传输，尽管由于FoE主要用于固件更新，这在实践中可能不太有用。

##### features/foe/0004-foe-request-progress.patch

添加了一种获取“当前进度”值（实际上是字节偏移量）的方法，用于异步FoE传输。

##### features/parallel-slave/0001-fsm_sii_external-datagram.patch

为以后的补丁做准备：使`fsm_sii`使用外部数据报。

##### features/parallel-slave/0002-fsm_change-external-datagram.patch

为以后的补丁做准备：使`fsm_change`使用外部数据报。

##### features/parallel-slave/0003-fsm_slave_config-external-datagram.patch

为以后的补丁做准备：使`fsm_slave_config`使用外部数据报。

##### features/parallel-slave/0004-fsm_slave_scan-external-datagram.patch

为以后的补丁做准备：使`fsm_slave_scan`使用外部数据报。

##### features/parallel-slave/0005-fsm_slave-handles-all-sdos.patch

将内部SDO请求和SDO字典请求（如果禁用了补丁base/0023中的`EC_SKIP_SDO_DICT`）从`fsm_master`移动到`fsm_slave`。

这做了两件重要的事情：首先，消除了内部和外部SDO请求之间在CoE邮箱上的争夺（使双方的繁忙检查变得不必要）。其次，它允许这两者在后台并行进行，并且可以在多个从站之间同时进行，而不需要使用单独的线程。
##### features/parallel-slave/0006-fsm_slave_config_scan-to-fsm_slave.patch

与先前的补丁类似，这将 `fsm_slave_scan` 和 `fsm_slave_config` 从 `fsm_master` 移动到了 `fsm_slave`。这允许多个从站的扫描和配置并行进行。（请注意，在开始配置任何从站之前，必须完成扫描所有从站。）

这还为 `ec_slave_info_t` 添加了 `scan_required` 字段；当为 true 时，其他字段是不可靠的（应该被忽略），因为扫描尚未开始或仍在进行中。

动机是一个约有100个从站设备的网络；尽管扫描很快（在具有所有设备上的别名/序列号的“热”网络上不到一秒钟，之前的SII补丁后），将从站从PREOP状态配置为OP状态大约需要80秒钟（您可以看到每个从站的灯逐个亮起）。使用此补丁后，大约需要20秒钟。

我最初只打算移动 `fsm_slave_config`，但代码的结构要求也移动 `fsm_slave_scan`。从逻辑上讲，它们确实都属于从站状态机。

请注意，在这种情况下，“并行”并不意味着单独的线程 - 所有FSM（主站和所有从站）仍在单个线程上执行。但是，现在可以在同一帧中为多个从站包括数据报。使用的是现有的 `fsm_slave` 的限流机制，因此它将以块的方式配置从站，而不是一次性全部配置（因此，如果有大量从站，网络不会过载，尽管网络使用率将比以前更高）。默认设置下，它每次处理 **16** 个；这由 `EC_EXT_RING_SIZE/2` 控制。

##### features/parallel-slave/0007-fsm-exec-simplify.patch

由于大多数FSM现在从 `fsm_slave` 执行，它们不再需要检查数据报状态，因为 `ec_master_exec_slave_fsms` 在 master.c 中提前执行此操作。这简化了FSM执行函数。

##### features/sii-wait-load/0001-slave-scan-retry.patch

如果发生错误，重试读取SII。

##### features/sii-wait-load/0002-fsm_sii-loading-check.patch

如果“loaded”位未设置，重试读取SII。

##### features/rt-slave/0001-allow-app-to-process-slave-requests-from-realtime.patch

添加了 `ecrt_master_rt_slave_requests` 和 `ecrt_master_exec_slave_requests`，允许应用程序在主应用程序周期任务（或其他RT任务）中排队和处理后台从站请求。当使用RTAI应用程序和慢速Linux内核HZ时，这可能有助于加快处理从站请求的速度。

**注意**：如果使用不慎，此方法的风险更高，可能会导致竞争条件。即使不使用此功能，应用补丁也是安全的。

##### features/eoe-rtdm/0001-eoe-addif-delif-tools.patch

允许预先显式配置EoE接口，即使其对应的从站不存在或尚未配置（这反映在接口的载体状态中）。

##### features/eoe-rtdm/0002-eoe-via-rtdm.patch

允许应用程序线程管理EoE接口，而不是主线程；这允许用户空间RTDM应用程序的任务之间更好的同步。

##### features/pcap/0001-pcap-logging.patch

添加了`pcap`命令，用于创建包含固定数量EtherCAT数据包的Wireshark格式文件，用于诊断目的。

    ethercat pcap >log.pcap
    ethercat pcap -r >log.pcap

当内部缓冲区已满时，停止捕获。重置缓冲区将重新开始捕获，但在此期间可能会丢失一些数据包。

捕获日志大小是硬编码的。

##### features/pcap/0002-runtime-size.patch

修改上述内容，允许在ethercat配置文件中指定pcap捕获缓冲区大小，而不是在编译时。默认情况下禁用捕获（以节省内存）。

这使您可以在不重新安装主站的情况下从生产系统获取捕获结果 - 只需重新启动主站即可。

##### features/pcap/0003-report-size.patch

修复了在捕获缓冲区已满之前执行 `ethercat pcap` 可能导致错误的问题。

##### features/pcap/0004-high-precision.patch

在pcap文件中使用微秒精度的时间戳，除非使用RTAI/Xenomai。

##### features/mbg/0001-mailbox-gateway.patch

添加了EtherCAT邮箱网关服务器。[详细信息](http://lists.etherlab.org/pipermail/etherlab-dev/2019/000702.html)

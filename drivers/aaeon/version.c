#include <linux/dmi.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/utsname.h>
#include <generated/utsrelease.h>
#include <aaeon/aaeon_api.h>
#include "version.h"

const char aa_version_banner[] =
		"AAEON System Information:\n"
		"  BUILD VERSION: %s\n"
		"  OS VERSION: %s\n"
		"  OS TYPE: %s\n"
		"  OS Compatibility: %s\n"
		"  KERNEL VERSION: %s\n"
		"  BOOTLOADER INFO:\n"
		"  - BOOTLOADER VENDOR: %s\n"
		"  - BOOTLOADER VERSION: %s\n"
		"  - BOOTLOADER RELEASE DATE: %s\n"
		"  PRODUCT INFO:\n"
		"  - PRODUCT NAME: %s\n"
		"  - PRODUCT VERSION: %s\n"
		"  BOARD INFO:\n"
		"  - BOARD VENDOR: %s\n"
		"  - BOARD NAME: %s\n"
		"  - BOARD VERSION: %s\n";

static int aa_version_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, aa_version_banner,
		build_version, // BUILD VERSION
		OS_VERSION, // OS VERSION
		OS_TYPE, // OS TYPE
		board_name, // OS Compatibility
		UTS_RELEASE, // KERNEL VERSION
		bootloader_vendor,
		bootloader_version,
		bootloader_date,
		product_name,
		product_version,
		board_vendor,
		board_name,
		board_version
	);

	return 0;
}

static int aa_version_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, aa_version_proc_show, NULL);
}

static const struct file_operations aa_version_proc_fops = {
	.open		= aa_version_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

// call in core.c
int __init aa_proc_version_init(void)
{
	proc_create("version", 0, aaeon_proc_root, &aa_version_proc_fops);
	return 0;
}


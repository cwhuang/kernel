#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/dmi.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/proc_fs.h>

#include <aaeon/aaeon_api.h>
#include "version.h"

#ifndef DEBUG
#define DEBUG 1
#endif
#if DEBUG
#define DBG(format, args...) printk("%s:%d: "format, __func__, __LINE__, ##args)
#else
#define DBG(...)
#endif

struct proc_dir_entry *aaeon_proc_root = NULL;
EXPORT_SYMBOL(aaeon_proc_root);

struct class *aaeon_class = NULL;
EXPORT_SYMBOL(aaeon_class);

#define AAEON_CORE_DEV	"core"
#define CORE_DEV_NUM	1

char bootloader_vendor[MAX_STRING_LEN] = { 0 };
char bootloader_version[MAX_STRING_LEN] = { 0 };
char bootloader_date[MAX_STRING_LEN] = { 0 };
char product_name[MAX_STRING_LEN] = { 0 };
char product_version[MAX_STRING_LEN] = { 0 };
char board_vendor[MAX_STRING_LEN] = { 0 };
char board_name[MAX_STRING_LEN] = { 0 };
char board_version[MAX_STRING_LEN] = { 0 };
char build_version[MAX_STRING_LEN] = { 0 };

static unsigned int core_major = 0;
static unsigned int core_minor = 0;
static struct cdev core_cdev;
static dev_t core_dev;

ssize_t core_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	return 0;
}

struct file_operations core_fops = {
	.read = core_read,
};

static int __init aaeon_core_init(void)
{
	int err = -1;
	int ret = 0;
	int cdev_err = 0;
	int major;

	struct device *class_dev = NULL;

	dev_t dev = MKDEV(core_major, core_minor);

	ret = alloc_chrdev_region(&dev, 0, CORE_DEV_NUM, AAEON_CORE_DEV);
	if (ret)
		goto alloc_chrdev_fail;

	core_major = major = MAJOR(dev);

	cdev_init(&core_cdev, &core_fops);
	core_cdev.owner = THIS_MODULE;
	core_cdev.ops = &core_fops;
	cdev_err = cdev_add(&core_cdev, MKDEV(core_major, core_minor), 1);
	if (cdev_err)
		goto chardev_add_fail;

	// create: /sys/class/aaeon/ for aaeon modules
	DBG("create: /sys/class/aaeon/\n");
	aaeon_class = class_create(THIS_MODULE, AAEON_CORE_ROOT);
	if (IS_ERR(aaeon_class)) {
		err = PTR_ERR(aaeon_class);
		printk(KERN_ERR "failed to create aaeon class.\n");
		goto class_create_fail;
	}

	core_dev = MKDEV(core_major, core_minor);

	class_dev = device_create(aaeon_class, NULL, core_dev, NULL, AAEON_CORE_DEV);
	if (IS_ERR(class_dev)) {
		err = PTR_ERR(class_dev);
		goto class_device_create_fail;
	}

	DBG("create: /proc/aaeon/\n");
	// create: /proc/aaeon/ for aaeon proc fs
	aaeon_proc_root = proc_mkdir(AAEON_CORE_ROOT, NULL);

	return 0;

class_device_create_fail:
	class_destroy(aaeon_class);
class_create_fail:
	cdev_del(&core_cdev);
chardev_add_fail:
	unregister_chrdev_region(core_dev, CORE_DEV_NUM);
alloc_chrdev_fail:
	return err;
}


#ifdef AAEON_BOARD_VENDOR
#undef AAEON_BOARD_VENDOR
#endif // AAEON_BOARD_VENDOR
#define AAEON_BOARD_VENDOR	"AAEON"

#ifdef AAEON_BOARD_GENERIC
#undef AAEON_BOARD_GENERIC
#endif // AAEON_BOARD_GENERIC
#define AAEON_BOARD_GENERIC	"TX2_GENERIC"

// The AAEON_BOARD_NAME should be defined in device.mk and exported by KCFLAGS.
// The value of AAEON_BOARD_NAME should be same with the value of PRODUCT_MODEL.
#ifndef AAEON_BOARD_NAME
#pragma message ("Warrning: AAEON_BOARD_NAME should be definded.")
#endif // AAEON_BOARD_NAME

static inline void board_check(void)
{
	int result;
	const char *board_vendor_str = board_vendor;
	const char *board_name_str = board_name;

	result = memcmp(AAEON_BOARD_VENDOR, board_vendor_str, sizeof(AAEON_BOARD_VENDOR));
	BUG_ON(result);

#ifdef AAEON_BOARD_NAME
	result = memcmp(AAEON_BOARD_NAME, board_name_str, sizeof(AAEON_BOARD_NAME));
	if (result && memcmp(AAEON_BOARD_NAME, AAEON_BOARD_GENERIC, sizeof(AAEON_BOARD_GENERIC))) {
		// not the target board && not the generic image, bug on
		BUG_ON(result);
	} else {
		pr_info("%s: Manufacturer=%s, Board=%s\n", AAEON_BOARD_NAME, board_vendor_str, board_name_str);
	}
#else
	pr_info("AAEON_BOARD_NAME is not defined\n");
	BUG();
#endif

	return;
}

static int __init bootloader_vendor_setup(char *s)
{
	int len = (int)strlen(s) + 1;
	if (len <= MAX_STRING_LEN) {
		memcpy(bootloader_vendor, s, len);
	}
	return 1;
}
__setup("bl_vendor=", bootloader_vendor_setup);

static int __init bootloader_version_setup(char *s)
{
	int len = (int)strlen(s) + 1;
	if (len <= MAX_STRING_LEN) {
		memcpy(bootloader_version, s, len);
	}
	return 1;
}
__setup("bl_version=", bootloader_version_setup);

static int __init bootloader_date_setup(char *s)
{
	int len = (int)strlen(s) + 1;
	if (len <= MAX_STRING_LEN) {
		memcpy(bootloader_date, s, len);
	}
	return 1;
}
__setup("bl_date=", bootloader_date_setup);

static int __init product_name_setup(char *s)
{
	int len = (int)strlen(s) + 1;
	if (len <= MAX_STRING_LEN) {
		memcpy(product_name, s, len);
	}
	return 1;
}
__setup("product_name=", product_name_setup);

static int __init product_version_setup(char *s)
{
	int len = (int)strlen(s) + 1;
	if (len <= MAX_STRING_LEN) {
		memcpy(product_version, s, len);
	}
	return 1;
}
__setup("product_version=", product_version_setup);

static int __init board_vendor_setup(char *s)
{
	int len = (int)strlen(s) + 1;
	if (len <= MAX_STRING_LEN) {
		memcpy(board_vendor, s, len);
	}
	return 1;
}
__setup("board_vendor=", board_vendor_setup);

static int __init board_name_setup(char *s)
{
	int len = (int)strlen(s) + 1;
	if (len <= MAX_STRING_LEN) {
		memcpy(board_name, s, len);
	}
	return 1;
}
__setup("board_name=", board_name_setup);

static int __init board_version_setup(char *s)
{
	int len = (int)strlen(s) + 1;
	if (len <= MAX_STRING_LEN) {
		memcpy(board_version, s, len);
	}
	return 1;
}
__setup("board_version=", board_version_setup);

#ifdef BUILD_VERSION
static void load_build_version(void)
{
	int len = (int)strlen(BUILD_VERSION) + 1;
	if (len <= MAX_STRING_LEN) {
		memcpy(build_version, BUILD_VERSION, len);
	}
}
#endif

static int __init aaeon_init(void)
{
	int result;

#ifdef BUILD_VERSION
	load_build_version();
#endif
	result = aaeon_core_init();
	aa_proc_version_init();

	//board_check();

	return result;
}

static void __exit aaeon_exit(void)
{
	device_destroy(aaeon_class, core_dev);
	class_destroy(aaeon_class);
	cdev_del(&core_cdev);
	unregister_chrdev_region(core_dev, CORE_DEV_NUM);
}


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Menghui Wu <Menghui_Wu@asus.com>");
MODULE_DESCRIPTION("AAEON Core Driver");

module_init(aaeon_init);
module_exit(aaeon_exit);

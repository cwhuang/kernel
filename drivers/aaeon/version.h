#ifndef __AAEON_VERSION_H__
#define __AAEON_VERSION_H__

#define MAX_STRING_LEN 50
#ifdef PLATFORM_VERSION
#define OS_VERSION PLATFORM_VERSION
#else
#define OS_VERSION "Debian 10.8"
#endif
#ifdef CONFIG_ANDROID_VERSION
#define OS_TYPE "Android"
#else
#define OS_TYPE "Linux"
#endif

extern char bootloader_vendor[MAX_STRING_LEN];
extern char bootloader_version[MAX_STRING_LEN];
extern char bootloader_date[MAX_STRING_LEN];
extern char product_name[MAX_STRING_LEN];
extern char product_version[MAX_STRING_LEN];
extern char board_vendor[MAX_STRING_LEN];
extern char board_name[MAX_STRING_LEN];
extern char board_version[MAX_STRING_LEN];
extern char build_version[MAX_STRING_LEN];

int aa_proc_version_init(void);

#endif

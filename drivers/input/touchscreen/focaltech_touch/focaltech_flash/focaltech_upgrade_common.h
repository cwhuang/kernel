/************************************************************************
* Copyright (C) 2012-2016, Focaltech Systems (R)，All Rights Reserved.
*
* File Name: focaltech_upgrade_common.h
*
*    Author: fupeipei
*
*   Created: 2016-08-16
*
*  Abstract:
*
************************************************************************/
#ifndef __LINUX_FOCALTECH_UPGRADE_COMMON_H__
#define __LINUX_FOCALTECH_UPGRADE_COMMON_H__

/*****************************************************************************
* 1.Included header files
*****************************************************************************/
#include "../focaltech_flash.h"

/*****************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/

/*****************************************************************************
* Private enumerations, structures and unions using typedef
*****************************************************************************/

/*****************************************************************************
* Static variables
*****************************************************************************/

/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/

/*****************************************************************************
* Static function prototypes
*****************************************************************************/
bool fts_check_app_bin(u8 *pbt_buf, u32 dw_lenth);

int fts_ctpm_erase_flash(struct i2c_client *client);
int fts_ctpm_pramboot_ecc(struct i2c_client *client);
bool fts_ctpm_check_in_rom(struct i2c_client *client);
bool fts_ctpm_start_pramboot(struct i2c_client *client);
bool fts_ctpm_start_fw_upgrade(struct i2c_client *client);
bool fts_ctpm_check_in_pramboot(struct i2c_client *client);
int fts_ctpm_upgrade_idc_init(struct i2c_client *client, u32 dw_lenth);
int fts_ctpm_write_app_for_idc(struct i2c_client *client, u32 length,
			       u8 *readbuf);
int fts_ctpm_upgrade_ecc(struct i2c_client *client, u32 startaddr, u32 length);
int fts_ctpm_write_pramboot_for_idc(struct i2c_client *client, u32 length,
				    u8 *readbuf);
int fts_writeflash(struct i2c_client *client, u32 writeaddr, u32 length,
		   u8 *readbuf, u32 cnt);

int fts_ctpm_get_app_ver(void);
int fts_ctpm_fw_upgrade(struct i2c_client *client);
int fts_ctpm_lcd_cfg_upgrade(struct i2c_client *client);

#endif

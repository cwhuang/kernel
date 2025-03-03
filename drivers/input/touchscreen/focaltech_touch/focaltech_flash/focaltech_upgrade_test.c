/*
 *
 * FocalTech fts TouchScreen driver.
 *
 * Copyright (c) 2010-2016, Focaltech Ltd. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/*****************************************************************************
*
* File Name: focaltech_upgrade_test.c
*
* Author:    fupeipei
*
* Created:    2016-08-22
*
* Abstract:
*
* Reference:
*
*****************************************************************************/

/*****************************************************************************
* 1.Included header files
*****************************************************************************/
#include "../focaltech_core.h"
#include <linux/wakelock.h>
#include <linux/timer.h>

/*****************************************************************************
* Static variables
*****************************************************************************/

/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/
struct wake_lock ps_lock;
extern u8 upgrade_ecc;

#define FTS_DEBUG_UPGRADE(fmt, args...)                                                                                    \
	do {                                                                                                               \
		printk(KERN_ERR                                                                                            \
		       "[FTS][UPGRADE]:##############################################################################\n"); \
		printk(KERN_ERR "[FTS][UPGRADE]: " fmt "\n", ##args);                                                      \
		printk(KERN_ERR                                                                                            \
		       "[FTS][UPGRADE]:##############################################################################\n"); \
	} while (0)

/*****************************************************************************
* Static function prototypes
*****************************************************************************/
#if (FTS_UPGRADE_PINGPONG_TEST)
/************************************************************************
* Name: fts_ctpm_auto_upgrade_pingpong
* Brief:  0
* Input:  0
* Output: 0
* Return: 0
***********************************************************************/
static int fts_ctpm_auto_upgrade_pingpong(struct i2c_client *client)
{
	u8 uc_tp_fm_ver;
	int i_ret = 0;
	bool bUpgradeFlag = false;
	u8 uc_upgrade_times = 0;

	FTS_FUNC_ENTER();

	/* check need upgrade or not */
	bUpgradeFlag = fts_ctpm_check_need_upgrade(client);
	FTS_DEBUG("[UPGRADE]: bUpgradeFlag = 0x%x", bUpgradeFlag);

	/* pingpong test mode, need upgrade */
	FTS_INFO("[UPGRADE]: pingpong test mode, need upgrade!!");
	do {
		fts_updatefun_curr.need_upgrade = true;
		uc_upgrade_times++;

		/* esd check */
#if FTS_ESDCHECK_EN
		fts_esdcheck_switch(DISABLE);
#endif

		/* fw upgrade */
		i_ret = fts_ctpm_fw_upgrade(client);

#if FTS_ESDCHECK_EN
		fts_esdcheck_switch(ENABLE);
#endif
		if (i_ret == 0) /* upgrade success */
		{
			fts_updatefun_curr.app_valid = true;
			fts_updatefun_curr.need_upgrade = false;
			fts_updatefun_curr.get_projectid =
				FTS_GET_PROJECT_ID_FROM_FLASH;

			fts_i2c_read_reg(client, FTS_REG_FW_VER, &uc_tp_fm_ver);
			FTS_DEBUG("[UPGRADE]: upgrade to new version 0x%x",
				  uc_tp_fm_ver);
		} else /* upgrade fail */
		{
			fts_updatefun_curr.app_valid = false;
			fts_updatefun_curr.need_upgrade = true;
			fts_updatefun_curr.get_projectid =
				FTS_GET_PROJECT_ID_FROM_FLASH;

			/* if upgrade fail, reset to run ROM. if app in flash is ok. TP will work success */
			FTS_INFO("[UPGRADE]: upgrade fail, reset now!!");
			fts_ctpm_rom_or_pram_reset(client,
						   (uc_upgrade_times - 1));
		}
	} while ((i_ret != 0) &&
		 (uc_upgrade_times <
		  2)); /* if upgrade fail, upgrade again. then return */

	FTS_FUNC_EXIT();
	return i_ret;
}

/************************************************************************
* Name: fts_ctpm_auto_upgrade
* Brief:  0
* Input:  0
* Output: 0
* Return: 0
***********************************************************************/
void fts_ctpm_display_upgrade_time(bool start_time)
{
#if FTS_GET_UPGRADE_TIME
	static struct timeval tpend;
	static struct timeval tpstart;
	static int timeuse;

	if (start_time) {
		do_gettimeofday(&tpstart);
	} else {
		do_gettimeofday(&tpend);
		timeuse = 1000000 * (tpend.tv_sec - tpstart.tv_sec) +
			  tpend.tv_usec - tpstart.tv_usec;
		timeuse /= 1000000;
		FTS_DEBUG("[UPGRADE]: upgrade success : Use time: %d Seconds!!",
			  timeuse);
	}
#endif
}

/************************************************************************
* Name: fts_ctpm_auto_upgrade
* Brief:  0
* Input:  0
* Output: 0
* Return: 0
***********************************************************************/
int fts_ctpm_auto_upgrade(struct i2c_client *client)
{
	int i_ret = 0;
	static int uc_ErrorTimes = 0;
	static int uc_UpgradeTimes = 0;

	wake_lock_init(&ps_lock, WAKE_LOCK_SUSPEND, "tp_wakelock");

	wake_lock(&ps_lock);

	do {
		uc_UpgradeTimes++;

		FTS_DEBUG_UPGRADE("start to upgrade %d times !!",
				  uc_UpgradeTimes);

		fts_ctpm_display_upgrade_time(true);

		i_ret = fts_ctpm_auto_upgrade_pingpong(client);
		if (i_ret == 0) {
			fts_ctpm_display_upgrade_time(false);
		} else {
			uc_ErrorTimes++;
		}

		FTS_DEBUG_UPGRADE("upgrade %d times, error %d times!!",
				  uc_UpgradeTimes, uc_ErrorTimes);
	} while (uc_UpgradeTimes < (FTS_UPGRADE_TEST_NUMBER));

	fts_updatefun_curr.need_upgrade = true;

	wake_unlock(&ps_lock);

	return 0;
}
#endif

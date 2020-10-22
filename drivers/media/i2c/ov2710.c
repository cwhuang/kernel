// SPDX-License-Identifier: GPL-2.0
/*
 * ov2710 driver
 *
 * Copyright (C) 2019 Fuzhou Rockchip Electronics Co., Ltd.
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/media.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/rk-camera-module.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>

#define DRIVER_VERSION			KERNEL_VERSION(0, 0x01, 0x1)

#ifndef V4L2_CID_DIGITAL_GAIN
#define V4L2_CID_DIGITAL_GAIN		V4L2_CID_GAIN
#endif

/* 45Mhz * 4 Binning */

#define OV2710_XVCLK_FREQ		24000000
#define REG_NULL			0xFFFF
#define PAGE_SELECT_REG		0xfd
#define PAGE_ZERO			0x00
#define PAGE_ONE			0x01
#define PAGE_TWO			0x02
#define PAGE_OTP			0x04

//PAGE0
#define OV2710_PIDH_ADDR	0x300A
#define OV2710_PIDL_ADDR	0x300B
#define OV2710_PIDH_MAGIC	0x27
#define OV2710_PIDL_MAGIC	0x10

//PAGE1
#define STREAM_CTRL_REG		0xa0
#define STREAM_ON			0x01
#define STREAM_OFF			0x00

#define UPDOWN_MIRROR_REG	0x3818
#define H_V_NORMAL			0x80
#define H_MIRROR			0x40
#define V_FLIP				0x20
#define MIRROR_AND_FLIP		0x03

#define OV2710_VTS_HIGH_REG		0x380e
#define OV2710_VTS_LOW_REG		0x380f
#define OV2710_COARSE_INTG_TIME_MIN		1
#define OV2710_COARSE_INTG_TIME_MAX		4
#define OV2710_VTS_ENABLE_REG	0x3503
#define OV2710_VTS_ENABLE_VALUE	0x07
#define OV2710_FRAME_SYNC_REG	0x4810
#define OV2710_FRAME_SYNC_VALUE	0xff
#define OV2710_REG_TEST_PATTERN 0xb2
#define OV2710_HTS_HIGH_REG		0x380c
#define OV2710_HTS_LOW_REG		0x380d
#define OV2710_TEST_PATTERN_ENABLE		0xfe
#define OV2710_TEST_PATTERN_DISABLE		BIT(0)
#define OV2710_FINE_INTG_TIME_MIN		0
#define OV2710_FINE_INTG_TIME_MAX_MARGIN 0
#define OV2710_COARSE_INTG_TIME_MIN		1
#define OV2710_COARSE_INTG_TIME_MAX_MARGIN 4

#define OV2710_AEC_PK_LONG_EXPO_3RD_REG 0x3500
#define OV2710_AEC_PK_LONG_EXPO_2ND_REG	0x3501	/* Exposure Bits 8-15 */
#define OV2710_AEC_PK_LONG_EXPO_1ST_REG	0x3502	/* Exposure Bits  0-7 */
#define OV2710_FETCH_3RD_BYTE_EXP(VAL)  (((VAL) >> 12) & 0xF)	/* 4 Bits */
#define OV2710_FETCH_2ND_BYTE_EXP(VAL)	(((VAL) >> 4) & 0xFF)
#define OV2710_FETCH_1ST_BYTE_EXP(VAL)	(((VAL) & 0x0F) << 4)

#define OV2710_AEC_PK_GAIN_REG	0x350b	/* GAIN Bits 0 -7 */
#define OV2710_FETCH_LSB_GAIN(VAL)		(VAL & 0x00FF)
#define OV2710_FETCH_MSB_GAIN(VAL)		((VAL >> 8) & 0x01)

#define	OV2710_EXPOSURE_MIN		4
#define	OV2710_EXPOSURE_STEP		1
#define OV2710_VTS_MAX			0xfff
#define	ANALOG_GAIN_MIN			0x00
#define	ANALOG_GAIN_MAX			0x7f
#define	ANALOG_GAIN_STEP		1
#define	ANALOG_GAIN_DEFAULT		0x00

#define OV2710_REG_TYPE_TIMEOUT		0x00

#define OV2710_NAME			"ov2710"

static const char * const ov2710_supply_names[] = {
	"dovdd",	/* Digital I/O power */
};

#define OV2710_NUM_SUPPLIES ARRAY_SIZE(ov2710_supply_names)

struct regval {
	u16 addr;
	u8 val;
};

struct ov2710_mode {
	u32 width;
	u32 height;
	struct v4l2_fract max_fps;
	u32 hts_def;
	u32 vts_def;
	u32 exp_def;
	const struct regval *reg_list;
};

struct ov2710 {
	struct i2c_client	*client;
	struct clk		*xvclk;
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*pwdn_gpio;
	struct regulator_bulk_data supplies[OV2710_NUM_SUPPLIES];

	struct v4l2_subdev	subdev;
	struct media_pad	pad;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl	*exposure;
	struct v4l2_ctrl	*anal_gain;
	struct v4l2_ctrl	*digi_gain;
	struct v4l2_ctrl	*hblank;
	struct v4l2_ctrl	*vblank;
	struct v4l2_ctrl	*test_pattern;
	struct mutex		mutex;
	bool			streaming;
	const struct ov2710_mode *cur_mode;
	u32			module_index;
	const char		*module_facing;
	const char		*module_name;
	const char		*len_name;
};

#define to_ov2710(sd) container_of(sd, struct ov2710, subdev)

/*
 * Base sensor configs
 * ov2710_init_tab_1920_1080_30fps
 * MCLK:24MHz  1920x1080  30fps   mipi 2lane   420Mbps/lane
 */
static struct regval ov2710_1920_1080_30fps[] = {
	{0x3008, 0x82},
	{0x3008, 0x42},
	{0x4201, 0x00},
	{0x4202, 0x0f},
	{0x3103, 0x93},
	{0x3017, 0x7f},
	{0x3018, 0xfc},
	{0x3706, 0x61},
	{0x3712, 0x0c},
	{0x3630, 0x6d},
	{0x3800, 0x01},
	{0x3801, 0xb4},
	{0x3802, 0x00},
	{0x3803, 0x0a},
	{0x3818, 0x80},
	{0x3804, 0x07},
	{0x3805, 0x80},
	{0x3806, 0x04},
	{0x3807, 0x38},
	{0x3808, 0x07},
	{0x3809, 0x80},
	{0x380a, 0x04},
	{0x380b, 0x38},
	{0x3810, 0x10},
	{0x3811, 0x06},
	{0x3812, 0x00},
	{0x3813, 0x00},
	{0x3621, 0x04},
	{0x3604, 0x60},
	{0x3603, 0xa7},
	{0x3631, 0x26},
	{0x3600, 0x04},
	{0x3620, 0x37},
	{0x3623, 0x00},
	{0x3702, 0x9e},
	{0x3703, 0x5c},
	{0x3704, 0x40},
	{0x370d, 0x0f},
	{0x3713, 0x9f},
	{0x3714, 0x4c},
	{0x3710, 0x9e},
	{0x3801, 0xc4},
	{0x3605, 0x05},
	{0x3606, 0x3f},
	{0x302d, 0x90},
	{0x370b, 0x40},
	{0x3716, 0x31},
	{0x3707, 0x52},
	{0x380d, 0x74},
	{0x5181, 0x20},
	{0x518f, 0x00},
	{0x4301, 0xff},
	{0x4303, 0x00},
	{0x3a00, 0x78},
	{0x300f, 0x88},
	{0x3011, 0x28},
	{0x3a1a, 0x06},
	{0x3a18, 0x00},
	{0x3a19, 0x7a},
	{0x3a13, 0x54},
	{0x382e, 0x0f},
	{0x381a, 0x1a},
	{0x401d, 0x02},
	{0x5688, 0x03},
	{0x5684, 0x07},
	{0x5685, 0xa0},
	{0x5686, 0x04},
	{0x5687, 0x43},
	{0x3011, 0x0a},
	{0x300f, 0x8a},
	{0x3017, 0x00},
	{0x3018, 0x00},
	{0x300e, 0x04},
	{0x4801, 0x0f},
	{0x300f, 0xc3},
	{0x3a0f, 0x40},
	{0x3a10, 0x38},
	{0x3a1b, 0x48},
	{0x3a1e, 0x30},
	{0x3a11, 0x90},
	{0x3a1f, 0x10},
	{0x380c, 0x09},/* HTS H */
	{0x380d, 0x74},/* HTS L */
	{0x380e, 0x04},/* VTS H */
	{0x380f, 0x50},/* VTS L */
	{0x3500, 0x00},
	{0x3501, 0x28},
	{0x3502, 0x90},
	{0x3503, 0x07},
	{0x350a, 0x00},
	{0x350b, 0x1f},
	{0x5000, 0x5f},
	{0x5001, 0x4e},
	{0x3406, 0x01},
	{0x3400, 0x04},
	{0x3401, 0x00},
	{0x3402, 0x04},
	{0x3403, 0x00},
	{0x3404, 0x04},
	{0x3405, 0x00},
	{0x4800, 0x24},
	{0x4201, 0x00},
	{0x4202, 0x00},
	{0x3008, 0x02},
	{OV2710_REG_TYPE_TIMEOUT, 40},
	{0x3008, 0x42},
	{0x4201, 0x00},
	{0x4202, 0x0f}
};

#define HTS_DEF 0x0974
#define VTS_DEF 0x0450
#define EXP_DEF 0x2890
#define MAX_FPS 30

static const struct ov2710_mode supported_modes[] = {
	{
		.width = 1920,
		.height = 1080,
		.max_fps = {
			.numerator = 10000,
			.denominator = MAX_FPS * 10000,
		},
		.exp_def = 0x18f,
		.hts_def = HTS_DEF,
		.vts_def = VTS_DEF,
		.reg_list = ov2710_1920_1080_30fps,
	},
};

#define OV2710_LINK_FREQ_420MHZ		400000000
#define OV2710_PIXEL_RATE		(MAX_FPS * HTS_DEF * VTS_DEF)
static const s64 link_freq_menu_items[] = {
	OV2710_LINK_FREQ_420MHZ
};

static const char * const ov2710_test_pattern_menu[] = {
	"Disabled",
	"Vertical Color",
};

/* Write registers up to 4 at a time */
static int ov2710_write_reg(struct i2c_client *client, u16 reg,
			      u32 val)
{
	u32 buf_i, val_i;
	u8 buf[6];
	u8 *val_p;
	__be32 val_be;
	u32 len = 1;
	if (len > 4)
		return -EINVAL;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;

	val_be = cpu_to_be32(val);
	val_p = (u8 *)&val_be;
	buf_i = 2;
	val_i = 4 - len;

	while (val_i < 4)
		buf[buf_i++] = val_p[val_i++];

	if (i2c_master_send(client, buf, len + 2) != len + 2)
		return -EIO;

	return 0;
}

static int ov2710_write_array(struct i2c_client *client,
			      const struct regval *regs)
{
	int i, ret = 0;

	i = 0;
	while (regs[i].addr != REG_NULL) {
		if (regs[i].addr == OV2710_REG_TYPE_TIMEOUT) {
			usleep_range(regs[i].val * 1000, regs[i].val * 2000);
			i++;
			continue;
		}
		ret = ov2710_write_reg(client, regs[i].addr, regs[i].val);
		if (ret) {
			dev_err(&client->dev, "%s failed !\n", __func__);
			break;
		}

		i++;
	}

	return ret;
}

/* Read registers up to 4 at a time */
/* sensor register read */
static int ov2710_read_reg(struct i2c_client *client, u16 reg,
			     u32 *val)
{
	struct i2c_msg msgs[2];
	u8 *data_be_p;
	u32 len = 1;
	__be32 data_be = 0;
	__be16 reg_addr_be = cpu_to_be16(reg);
	int ret;

	if (len > 4 || !len)
		return -EINVAL;

	data_be_p = (u8 *)&data_be;
	/* Write register address */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 2;
	msgs[0].buf = (u8 *)&reg_addr_be;

	/* Read data from register */
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = &data_be_p[4 - len];

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	*val = be32_to_cpu(data_be);

	return 0;
}

static int ov2710_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct ov2710 *ov2710 = to_ov2710(sd);
	const struct ov2710_mode *mode;
	s64 h_blank, vblank_def;

	mutex_lock(&ov2710->mutex);
	mode = &supported_modes[0];
	fmt->format.code = MEDIA_BUS_FMT_SBGGR10_1X10;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&ov2710->mutex);
		return -ENOTTY;
#endif
	} else {
		ov2710->cur_mode = mode;
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(ov2710->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(ov2710->vblank, vblank_def,
					 OV2710_VTS_MAX - mode->height,
					 1, vblank_def);
	}

	mutex_unlock(&ov2710->mutex);

	return 0;
}

static int ov2710_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct ov2710 *ov2710 = to_ov2710(sd);
	const struct ov2710_mode *mode = ov2710->cur_mode;

	mutex_lock(&ov2710->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&ov2710->mutex);
		return -ENOTTY;
#endif
	} else {
		fmt->format.width = mode->width;
		fmt->format.height = mode->height;
		fmt->format.code = MEDIA_BUS_FMT_SBGGR10_1X10;
		fmt->format.field = V4L2_FIELD_NONE;
	}
	mutex_unlock(&ov2710->mutex);

	return 0;
}

static int ov2710_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index != 0)
		return -EINVAL;
	code->code = MEDIA_BUS_FMT_SBGGR10_1X10;

	return 0;
}

static int ov2710_enum_frame_sizes(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	if (fse->code != MEDIA_BUS_FMT_SBGGR10_1X10)
		return -EINVAL;

	fse->min_width  = supported_modes[fse->index].width;
	fse->max_width  = supported_modes[fse->index].width;
	fse->max_height = supported_modes[fse->index].height;
	fse->min_height = supported_modes[fse->index].height;

	return 0;
}

static int ov2710_enum_frame_interval(struct v4l2_subdev *sd,
				      struct v4l2_subdev_pad_config *cfg,
				      struct v4l2_subdev_frame_interval_enum *fie)
{
	u32 index = fie->index;

	if (index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	fie->width = supported_modes[index].width;
	fie->height = supported_modes[index].height;
	fie->interval = supported_modes[index].max_fps;

	return 0;
}

static int ov2710_enable_test_pattern(struct ov2710 *ov2710, u32 pattern)
{
	int ret;
	u32 val;

	ret = ov2710_read_reg(ov2710->client, OV2710_REG_TEST_PATTERN, &val);
	if (ret < 0)
		return ret;

	switch (pattern) {
	case 0:
		val &= ~OV2710_TEST_PATTERN_ENABLE;
		break;
	case 1:
		val |= OV2710_TEST_PATTERN_ENABLE;
		break;
	}

	return ov2710_write_reg(ov2710->client,
				 OV2710_REG_TEST_PATTERN,
				 val);
}

static void ov2710_get_module_inf(struct ov2710 *ov2710,
				  struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strlcpy(inf->base.sensor, OV2710_NAME, sizeof(inf->base.sensor));
	strlcpy(inf->base.module, ov2710->module_name,
		sizeof(inf->base.module));
	strlcpy(inf->base.lens, ov2710->len_name, sizeof(inf->base.lens));
}

static long ov2710_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct ov2710 *ov2710 = to_ov2710(sd);
	long ret = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		ov2710_get_module_inf(ov2710, (struct rkmodule_inf *)arg);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long ov2710_compat_ioctl32(struct v4l2_subdev *sd,
				  unsigned int cmd, unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct rkmodule_inf *inf;
	//struct rkmodule_awb_cfg *cfg;
	long ret;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		inf = kzalloc(sizeof(*inf), GFP_KERNEL);
		if (!inf) {
			ret = -ENOMEM;
			return ret;
		}

		ret = ov2710_ioctl(sd, cmd, inf);
		if (!ret)
			ret = copy_to_user(up, inf, sizeof(*inf));
		kfree(inf);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}
#endif

static int __ov2710_start_stream(struct ov2710 *ov2710)
{
	int ret;
	ret = ov2710_write_array(ov2710->client, ov2710->cur_mode->reg_list);
	if (ret)
		return ret;
	//ret = ov2710_write_reg(ov2710->client, PAGE_SELECT_REG, PAGE_ONE);
	//if (ret)
	//	return ret;
	/* In case these controls are set before streaming */
	mutex_unlock(&ov2710->mutex);
	ret = v4l2_ctrl_handler_setup(&ov2710->ctrl_handler);
	mutex_lock(&ov2710->mutex);
	if (ret)
		return ret;
	ret = ov2710_write_reg(ov2710->client, 0x3008, 0x02);
	ret |= ov2710_write_reg(ov2710->client, 0x4201, 0x00);
	ret |= ov2710_write_reg(ov2710->client, 0x4202, 0x00);
	return ret;
}

static int __ov2710_stop_stream(struct ov2710 *ov2710)
{
	int ret;

	ret = ov2710_write_reg(ov2710->client, 0x3008, 0x42);
	ret |= ov2710_write_reg(ov2710->client, 0x4201, 0x00);
	ret |= ov2710_write_reg(ov2710->client, 0x4202, 0x0f);

	return ret;
}

static int ov2710_s_stream(struct v4l2_subdev *sd, int on)
{
	struct ov2710 *ov2710 = to_ov2710(sd);
	struct i2c_client *client = ov2710->client;
	int ret = 0;

	dev_info(sd->dev, "%s %s", __func__, on ? "on" : "off");
	mutex_lock(&ov2710->mutex);
	on = !!on;
	if (on == ov2710->streaming)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = __ov2710_start_stream(ov2710);
		if (ret) {
			dev_err(sd->dev, "start stream failed while write regs: %d\n", ret);
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
	} else {
		__ov2710_stop_stream(ov2710);
		pm_runtime_put(&client->dev);
	}

	ov2710->streaming = on;

unlock_and_return:
	mutex_unlock(&ov2710->mutex);

	return ret;
}

static int ov2710_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct ov2710 *ov2710 = to_ov2710(sd);

	mutex_lock(&ov2710->mutex);
	fi->interval = ov2710->cur_mode->max_fps;
	mutex_unlock(&ov2710->mutex);

	return 0;
}

static int __ov2710_power_on(struct ov2710 *ov2710)
{
	int ret;
	struct device *dev = &ov2710->client->dev;

	if (!IS_ERR(ov2710->pwdn_gpio)) {
		gpiod_set_value_cansleep(ov2710->pwdn_gpio, 1);
		usleep_range(2000, 5000);
	}

	ret = regulator_bulk_enable(OV2710_NUM_SUPPLIES, ov2710->supplies);
	usleep_range(20000, 50000);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto disable_clk;
	}

	if (!IS_ERR(ov2710->pwdn_gpio)) {
		gpiod_set_value_cansleep(ov2710->pwdn_gpio, 0);
		usleep_range(2000, 5000);
	}

	if (!IS_ERR(ov2710->reset_gpio)) {
		gpiod_set_value_cansleep(ov2710->reset_gpio, 1);
		usleep_range(2000, 5000);
	}
	if (!IS_ERR(ov2710->xvclk)) {
		ret = clk_prepare_enable(ov2710->xvclk);
		if (ret < 0)
			dev_info(dev, "Failed to enable xvclk\n");
	}

	usleep_range(20000, 30000);
	dev_info(&ov2710->client->dev, "%s OK\n", __func__);

	return 0;

disable_clk:
	clk_disable_unprepare(ov2710->xvclk);

	return ret;
}

static void __ov2710_power_off(struct ov2710 *ov2710)
{
	if (!IS_ERR(ov2710->pwdn_gpio))
		gpiod_set_value_cansleep(ov2710->pwdn_gpio, 1);
	clk_disable_unprepare(ov2710->xvclk);
	if (!IS_ERR(ov2710->reset_gpio))
		gpiod_set_value_cansleep(ov2710->reset_gpio, 1);
	regulator_bulk_disable(OV2710_NUM_SUPPLIES, ov2710->supplies);
	dev_info(&ov2710->client->dev, "%s OK\n", __func__);
}

static int ov2710_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov2710 *ov2710 = to_ov2710(sd);

	return __ov2710_power_on(ov2710);
}

static int ov2710_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov2710 *ov2710 = to_ov2710(sd);

	__ov2710_power_off(ov2710);

	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int ov2710_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct ov2710 *ov2710 = to_ov2710(sd);
	struct v4l2_mbus_framefmt *try_fmt =
				v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct ov2710_mode *def_mode = &supported_modes[0];
	mutex_lock(&ov2710->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = MEDIA_BUS_FMT_SBGGR10_1X10;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&ov2710->mutex);
	/* No crop or compose */
	return 0;
}
#endif

static const struct dev_pm_ops ov2710_pm_ops = {
	SET_RUNTIME_PM_OPS(ov2710_runtime_suspend,
			   ov2710_runtime_resume, NULL)
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops ov2710_internal_ops = {
	.open = ov2710_open,
};
#endif

static const struct v4l2_subdev_core_ops ov2710_core_ops = {
	.ioctl = ov2710_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = ov2710_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops ov2710_video_ops = {
	.s_stream = ov2710_s_stream,
	.g_frame_interval = ov2710_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops ov2710_pad_ops = {
	.enum_mbus_code = ov2710_enum_mbus_code,
	.enum_frame_size = ov2710_enum_frame_sizes,
	.enum_frame_interval = ov2710_enum_frame_interval,
	.get_fmt = ov2710_get_fmt,
	.set_fmt = ov2710_set_fmt,
};

static const struct v4l2_subdev_ops ov2710_subdev_ops = {
	.core	= &ov2710_core_ops,
	.video	= &ov2710_video_ops,
	.pad	= &ov2710_pad_ops,
};

static int ov2710_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ov2710 *ov2710 = container_of(ctrl->handler,
					     struct ov2710, ctrl_handler);
	struct i2c_client *client = ov2710->client;
	s64 max;
	int ret = 0;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = ov2710->cur_mode->height + ctrl->val - 4;
		__v4l2_ctrl_modify_range(ov2710->exposure,
					 ov2710->exposure->minimum, max,
					 ov2710->exposure->step,
					 ov2710->exposure->default_value);
		break;
	}

	/*
	 * Applying V4L2 control value only happens
	 * when power is up for streaming
	 */
	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	//ret = ov2710_write_reg(client, PAGE_SELECT_REG, PAGE_ONE);
	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		ret |= ov2710_write_reg(client,
			 OV2710_AEC_PK_LONG_EXPO_3RD_REG,
			 OV2710_FETCH_3RD_BYTE_EXP(ctrl->val));
		ret |= ov2710_write_reg(client,
			 OV2710_AEC_PK_LONG_EXPO_2ND_REG,
			 OV2710_FETCH_2ND_BYTE_EXP(ctrl->val));
		ret |= ov2710_write_reg(client,
			 OV2710_AEC_PK_LONG_EXPO_1ST_REG,
			 OV2710_FETCH_1ST_BYTE_EXP(ctrl->val));
		break;
	case V4L2_CID_ANALOGUE_GAIN:
	//	ret |= ov2710_write_reg(client, OV2710_AEC_PK_GAIN_REG,
	//		ctrl->val);
		ret |= ov2710_write_reg(client,
			 0x350a,
			 OV2710_FETCH_MSB_GAIN(ctrl->val));
		ret |= ov2710_write_reg(client,
			 0x350b,
			 OV2710_FETCH_LSB_GAIN(ctrl->val));
		break;
	case V4L2_CID_VBLANK:
		ret |= ov2710_write_reg(client, OV2710_VTS_ENABLE_REG,
			 OV2710_VTS_ENABLE_VALUE);
		ret |= ov2710_write_reg(client, OV2710_VTS_LOW_REG,
			 (ctrl->val + ov2710->cur_mode->height) & 0xFF);
		ret |= ov2710_write_reg(client, OV2710_VTS_HIGH_REG,
			 ((ctrl->val + ov2710->cur_mode->height) >> 8) & 0x0F);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = ov2710_enable_test_pattern(ov2710, ctrl->val);

		break;
	default:
		dev_warn(&client->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			 __func__, ctrl->id, ctrl->val);
		break;
	}
	//ret |= ov2710_write_reg(client, OV2710_FRAME_SYNC_REG,
		//	 OV2710_FRAME_SYNC_VALUE);

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops ov2710_ctrl_ops = {
	.s_ctrl = ov2710_set_ctrl,
};

static int ov2710_initialize_controls(struct ov2710 *ov2710)
{
	const struct ov2710_mode *mode;
	struct v4l2_ctrl_handler *handler;
	struct v4l2_ctrl *ctrl;
	s64 exposure_max, vblank_def;
	u32 h_blank;
	int ret;

	handler = &ov2710->ctrl_handler;
	mode = ov2710->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 7);
	if (ret)
		return ret;
	handler->lock = &ov2710->mutex;

	ctrl = v4l2_ctrl_new_int_menu(handler, NULL, V4L2_CID_LINK_FREQ,
				      0, 0, link_freq_menu_items);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE,
			  0, OV2710_PIXEL_RATE, 1, OV2710_PIXEL_RATE);

	h_blank = mode->hts_def - mode->width;
	ov2710->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
				h_blank, h_blank, 1, h_blank);
	if (ov2710->hblank)
		ov2710->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank_def = mode->vts_def - mode->height;
	ov2710->vblank = v4l2_ctrl_new_std(handler, &ov2710_ctrl_ops,
				V4L2_CID_VBLANK, vblank_def,
				OV2710_VTS_MAX - mode->height,
				1, vblank_def);

	exposure_max = mode->vts_def - 4;
	ov2710->exposure = v4l2_ctrl_new_std(handler, &ov2710_ctrl_ops,
				V4L2_CID_EXPOSURE, OV2710_EXPOSURE_MIN,
				exposure_max, OV2710_EXPOSURE_STEP,
				mode->exp_def);

	ov2710->anal_gain = v4l2_ctrl_new_std(handler, &ov2710_ctrl_ops,
				V4L2_CID_ANALOGUE_GAIN, ANALOG_GAIN_MIN,
				ANALOG_GAIN_MAX, ANALOG_GAIN_STEP,
				ANALOG_GAIN_DEFAULT);

	ov2710->test_pattern = v4l2_ctrl_new_std_menu_items(handler,
				&ov2710_ctrl_ops, V4L2_CID_TEST_PATTERN,
				ARRAY_SIZE(ov2710_test_pattern_menu) - 1,
				0, 0, ov2710_test_pattern_menu);

	if (handler->error) {
		ret = handler->error;
		dev_err(&ov2710->client->dev,
			"Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	ov2710->subdev.ctrl_handler = handler;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static int ov2710_check_sensor_id(struct ov2710 *ov2710,
				  struct i2c_client *client)
{
	struct device *dev = &ov2710->client->dev;
	int ret;

	u32 pidh = 0x55, pidl = 0xaa;

	//ret = ov2710_write_reg(ov2710->client, PAGE_SELECT_REG, PAGE_ZERO);
	ret = ov2710_read_reg(ov2710->client, OV2710_PIDH_ADDR, &pidh);
	ret |= ov2710_read_reg(ov2710->client, OV2710_PIDL_ADDR, &pidl);
	if (IS_ERR_VALUE(ret)) {
		dev_err(dev,
			"register read failed, camera module powered off?\n");
		goto err;
	}

	if ((pidh == OV2710_PIDH_MAGIC) && (pidl == OV2710_PIDL_MAGIC)) {
		dev_info(dev,
			"Found cameraID 0x%02x%02x\n", pidh, pidl);
	} else {
		dev_err(dev,
			"wrong camera ID, expected 0x%02x%02x, detected 0x%02x%02x\n",
			OV2710_PIDH_MAGIC, OV2710_PIDL_MAGIC, pidh, pidl);
		ret = -EINVAL;
		goto err;
	}

	return 0;
err:
	dev_err(dev, "failed with error (%d)\n", ret);
	return ret;
}

static int ov2710_configure_regulators(struct ov2710 *ov2710)
{
	size_t i;

	for (i = 0; i < OV2710_NUM_SUPPLIES; i++)
		ov2710->supplies[i].supply = ov2710_supply_names[i];

	return devm_regulator_bulk_get(&ov2710->client->dev,
				       OV2710_NUM_SUPPLIES,
				       ov2710->supplies);
}

static int ov2710_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct ov2710 *ov2710;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		DRIVER_VERSION >> 16,
		(DRIVER_VERSION & 0xff00) >> 8,
		DRIVER_VERSION & 0x00ff);

	ov2710 = devm_kzalloc(dev, sizeof(*ov2710), GFP_KERNEL);
	if (!ov2710)
		return -ENOMEM;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &ov2710->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &ov2710->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &ov2710->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &ov2710->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}

	ov2710->client = client;
	ov2710->cur_mode = &supported_modes[0];

	ov2710->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(ov2710->xvclk)) {
		dev_err(dev, "Failed to get xvclk\n");
		return -EINVAL;
	}
	ret = clk_set_rate(ov2710->xvclk, OV2710_XVCLK_FREQ);
	if (ret < 0) {
		dev_err(dev, "Failed to set xvclk rate (24MHz)\n");
		return ret;
	}
	if (clk_get_rate(ov2710->xvclk) != OV2710_XVCLK_FREQ)
		dev_warn(dev, "xvclk mismatched, modes are based on 24MHz\n");

	ov2710->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ov2710->reset_gpio))
		dev_warn(dev, "Failed to get reset-gpios\n");

	ov2710->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_OUT_LOW);
	if (IS_ERR(ov2710->pwdn_gpio))
		dev_warn(dev, "Failed to get pwdn-gpios\n");

	ret = ov2710_configure_regulators(ov2710);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}

	mutex_init(&ov2710->mutex);

	sd = &ov2710->subdev;
	v4l2_i2c_subdev_init(sd, client, &ov2710_subdev_ops);
	ret = ov2710_initialize_controls(ov2710);
	if (ret)
		goto err_destroy_mutex;

	ret = __ov2710_power_on(ov2710);
	if (ret)
		goto err_free_handler;

	ret = ov2710_check_sensor_id(ov2710, client);
	if (ret)
		goto err_power_off;

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &ov2710_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
	ov2710->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &ov2710->pad);
	if (ret < 0)
		goto err_power_off;
#endif

	memset(facing, 0, sizeof(facing));
	if (strcmp(ov2710->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 ov2710->module_index, facing,
		 OV2710_NAME, dev_name(sd->dev));
	ret = v4l2_async_register_subdev_sensor_common(sd);
	if (ret) {
		dev_err(dev, "v4l2 async register subdev failed\n");
		goto err_clean_entity;
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	return 0;

err_clean_entity:
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
err_power_off:
	__ov2710_power_off(ov2710);
err_free_handler:
	v4l2_ctrl_handler_free(&ov2710->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&ov2710->mutex);

	return (ret == -EIO) ? -EPROBE_DEFER : ret;
}

static int ov2710_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov2710 *ov2710 = to_ov2710(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&ov2710->ctrl_handler);
	mutex_destroy(&ov2710->mutex);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__ov2710_power_off(ov2710);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id ov2710_of_match[] = {
	{ .compatible = "ovti,ov2710" },
	{},
};
MODULE_DEVICE_TABLE(of, ov2710_of_match);
#endif

static const struct i2c_device_id ov2710_match_id[] = {
	{ "ovti,ov2710", 0 },
	{ },
};

static struct i2c_driver ov2710_i2c_driver = {
	.driver = {
		.name = OV2710_NAME,
		.pm = &ov2710_pm_ops,
		.of_match_table = of_match_ptr(ov2710_of_match),
	},
	.probe		= &ov2710_probe,
	.remove		= &ov2710_remove,
	.id_table	= ov2710_match_id,
};

static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&ov2710_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&ov2710_i2c_driver);
}

device_initcall_sync(sensor_mod_init);
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION("OmniVision ov2710 sensor driver");
MODULE_LICENSE("GPL v2");

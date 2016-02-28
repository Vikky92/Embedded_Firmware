/*
 * Aptina AP0100 sensor driver
 *
 * Copyright (C) 2012 Aptina Imaging
 *
 * Leverage mt9p031.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/videodev2.h>

#include <media/ap0100.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#define AP0100_COMMAND_REGISTER 	0x0040
#define AP0100_CMD_PARAM_0		0xFC00
#define AP0100_CMD_PARAM_1		0xFC02
#define AP0100_CMD_PARAM_2		0xFC04
#define AP0100_CMD_PARAM_3		0xFC06
#define AP0100_CMD_PARAM_4		0xFC08
#define AP0100_CMD_PARAM_5		0xFC0A
#define AP0100_CMD_PARAM_6		0xFC0C
#define AP0100_CMD_PARAM_7		0xFC0E
#define AP0100_CHANGE_CONFIG		0x2800
#define AP0100_SUSPEND			0x4000
#define AP0100_SOFT_STANDBY		0x5000
#define AP0100_SET_STATE 		0x8100
#define AP0100_GET_STATE		0x8101
#define AP0100_PIXEL_ARRAY_WIDTH	1280
#define AP0100_PIXEL_ARRAY_HEIGHT	720

#define	AP0100_ROW_START_MIN		0
#define	AP0100_ROW_START_MAX		720
#define	AP0100_ROW_START_DEF		0
#define	AP0100_COLUMN_START_MIN		0
#define	AP0100_COLUMN_START_MAX		1280
#define	AP0100_COLUMN_START_DEF		0
#define	AP0100_WINDOW_HEIGHT_MIN	2
#define	AP0100_WINDOW_HEIGHT_MAX	720
#define	AP0100_WINDOW_HEIGHT_DEF	720
#define	AP0100_WINDOW_WIDTH_MIN		2
#define	AP0100_WINDOW_WIDTH_MAX		1280
#define	AP0100_WINDOW_WIDTH_DEF		1280
#define AP0100_ENABLE			1
#define AP0100_DISABLE			0

#define AP0100_CHIP_VERSION_REG		0x0000
#define AP0100_CHIP_ID			0x0062
#define AP0100_RESET_REG		0x001A
#define AP0100_RESET			0x0E05
#define AP0100_NORMAL			0x0E04
#define AP0100_SEQ_PORT			0x3086
#define AP0100_SEQ_PORT_CTRL		0x3088
#define AP0100_TEST_REG			0x3070
#define AP0100_READ_MODE		0xC846
#define AP0100_DEBUG			1
#define AP0100_MODE_SELECT		0xC88C
#define AP0100_PATTERN_SELECT		0xC88F
#define AP0100_SENSOR			0x0
#define AP0100_TEST_GENERATOR		0x2
#define AP0100_SOLID_COLOR		0x1
#define AP0100_COLOR_BAR		0x4

struct ap0100_frame_size {
	u16 width;
	u16 height;
};

struct ap0200_priv {
	struct v4l2_subdev subdev;
	struct media_pad pad;
	struct v4l2_rect crop;  /* Sensor window */
	struct v4l2_mbus_framefmt format;
	struct v4l2_ctrl_handler ctrls;
	struct ap0200_platform_data *pdata;
};

/************************************************************************
			Helper Functions
************************************************************************/
/**
 * to_ap0100 - A helper function which returns pointer to the
 * private data structure
 * @client: pointer to i2c client
 *
 */
static struct ap0100_priv *to_ap0100(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client),
			struct ap0100_priv, subdev);
}

/**
 * ap0100_read - reads the data from the given register
 * @client: pointer to i2c client
 * @addr: address of the register which is to be read
 *
 */
static int ap0200_read(struct i2c_client *client, u16 addr)
{
	struct i2c_msg msg[2];
	u8 buf[2];
	u16 __addr;
	u16 ret;

	/* 16 bit addressable register */
	__addr = cpu_to_be16(addr);

	msg[0].addr  = client->addr;
	msg[0].flags = 0;
	msg[0].len   = 2;
	msg[0].buf   = (u8 *)&__addr;

	msg[1].addr  = client->addr;
	msg[1].flags = I2C_M_RD; /* 1 */
	msg[1].len   = 2;
	msg[1].buf   = buf;

	/*
	* if return value of this function is < 0,
	* it means error.
	* else, under 16bit is valid data.
	*/
	ret = i2c_transfer(client->adapter, msg, 2);

	if (ret < 0) {
		v4l_err(client, "Read from offset 0x%x error %d", addr, ret);
		return ret;
	}

	return (buf[0] << 8) | buf[1];
}

/**
 * ap0100_read_8 - reads the data from the given register
 * @client: pointer to i2c client
 * @addr: address of the register which is to be read
 *
 */
static int ap0100_read_8(struct i2c_client *client, u16 addr)
{
	struct i2c_msg msg[2];
	u8 buf;
	u16 __addr;
	u16 ret;

	/* 16 bit addressable register */
	__addr = cpu_to_be16(addr);

	msg[0].addr  = client->addr;
	msg[0].flags = 0;
	msg[0].len   = 2;
	msg[0].buf   = (u8 *)&__addr;

	msg[1].addr  = client->addr;
	msg[1].flags = I2C_M_RD; /* 1 */
	msg[1].len   = 1;
	msg[1].buf   = (u8 *)&buf;

	/*
	* if return value of this function is < 0,
	* it means error.
	* else, under 16bit is valid data.
	*/
	ret = i2c_transfer(client->adapter, msg, 2);

	if (ret < 0) {
		v4l_err(client, "Read from offset 0x%x error %d", addr, ret);
		return ret;
	}

	return buf;
}
/**
 * ap0100_write - writes the data into the given register
 * @client: pointer to i2c client
 * @addr: address of the register in which to write
 * @data: data to be written into the register
 *
 */
static int ap0100_write(struct i2c_client *client, u16 addr,
				u16 data)
{
	struct i2c_msg msg;
	u8 buf[4];
	u16 __addr, __data;
	int ret;

	/* 16-bit addressable register */

	__addr = cpu_to_be16(addr);
	__data = cpu_to_be16(data);

	buf[0] = __addr & 0xff;
	buf[1] = __addr >> 8;
	buf[2] = __data & 0xff;
	buf[3] = __data >> 8;
	msg.addr  = client->addr;
	msg.flags = 0;
	msg.len   = 4;
	msg.buf   = buf;

	/* i2c_transfer returns message length, but function should return 0 */
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret >= 0)
		return 0;

	v4l_err(client, "Write failed at 0x%x error %d\n", addr, ret);
	return ret;
}

/**
 * ap0100_write_8 - writes the data into the given register
 * @client: pointer to i2c client
 * @addr: address of the register in which to write
 * @data: data to be written into the register
 *
 */
static int ap0100_write_8(struct i2c_client *client, u16 addr,
				u8 data)
{
	struct i2c_msg msg;
	u8 buf[3];
	u16 __addr;
	int ret;

	/* 16-bit addressable register */

	__addr = cpu_to_be16(addr);

	buf[0] = __addr & 0xff;
	buf[1] = __addr >> 8;
	buf[2] = data;
	msg.addr  = client->addr;
	msg.flags = 0;
	msg.len   = 3;
	msg.buf   = buf;

	/* i2c_transfer returns message length, but function should return 0 */
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret >= 0)
		return 0;

	v4l_err(client, "Write failed at 0x%x error %d\n", addr, ret);
	return ret;
}

/**
 * ap0100_get_state - get the current state
 * @ap0100: pointer to private data structure
 */
void ap0100_get_state(struct i2c_client *client)
{
	u16 data;
	int count;

	ap0100_write(client, AP0100_COMMAND_REGISTER, AP0100_GET_STATE);
	data = ap0100_read(client, AP0100_COMMAND_REGISTER);
	count = 0;
	while(data){
		data = ap0100_read(client, AP0100_COMMAND_REGISTER);
		if(count++ > 5){
			printk(KERN_INFO "Failed to GET STATE: ERROR = 0x%x\n",data);
			break;
		}
		msleep(10);
	}
	data = ap0100_read(client, AP0100_CMD_PARAM_0) >> 8;
	switch(data){
	case 0x20:
		printk(KERN_INFO"Current state of AP0100 = idle\n");
		break;
	case 0x31:
		printk(KERN_INFO"Current state of AP0100 = streaming\n");
		break;
	case 0x41:
		printk(KERN_INFO"Current state of AP0100 = suspended\n");
		break;
	case 0x53:
		printk(KERN_INFO"Current state of AP0100 = soft standby\n");
		break;
	case 0x5b:
		printk(KERN_INFO"Current state of AP0100 = hard standby\n");
		break;
	default:
		printk(KERN_INFO"Current state of AP0100 = unknown 0x%x\n", data);
		break;
	}
}

/**
 * ap0100_reset - Soft resets the sensor
 * @client: pointer to the i2c client
 *
 */
static int ap0100_reset(struct i2c_client *client)
{
	int ret;

	ret = ap0100_write(client, AP0100_RESET_REG, AP0100_RESET);
	if (ret < 0)
		return ret;

	msleep(20);

	return ap0100_write(client, AP0100_RESET_REG, AP0100_NORMAL);
}

/**
 * ap0100_power_on - power on the sensor
 * @ap0100: pointer to private data structure
 *
 */
void ap0100_power_on(struct ap0100_priv *ap0100)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ap0100->subdev);
	u16 data;
	int count;

	/* Enable clock */
	if (ap0100->pdata->set_xclk) {
		ap0100->pdata->set_xclk(&ap0100->subdev,
		ap0100->pdata->ext_freq);
		usleep_range(100, 200);
	}

	/* Ensure RESET_BAR is low for 50 clk cycles*/
	if (ap0100->pdata->reset) {
		ap0100->pdata->reset(&ap0100->subdev, 1);
		usleep_range(100, 200);
	}

	/* Maintain RESET_BAR high for 100 clk cycles before first i2c communication*/
	if (ap0100->pdata->reset) {
		ap0100->pdata->reset(&ap0100->subdev, 0);
		usleep_range(300, 400);
	}

	ap0100_write(client, AP0100_CMD_PARAM_0, AP0100_CHANGE_CONFIG);
	ap0100_write(client, AP0100_COMMAND_REGISTER, AP0100_SET_STATE);
	data = ap0100_read(client, AP0100_COMMAND_REGISTER);
	count = 0;
	while(data){
		data = ap0100_read(client, AP0100_COMMAND_REGISTER);
		if(count++ > 5){
			ap0100_write(client, AP0100_CMD_PARAM_0, AP0100_CHANGE_CONFIG);
			ap0100_write(client, AP0100_COMMAND_REGISTER, AP0100_SET_STATE);
			data = ap0100_read(client, AP0100_COMMAND_REGISTER);
			usleep_range(100, 200);
			if(!data)
				break;
			else
				count = 0;
		}
		usleep_range(100, 200);
	}
}

/**
 * ap0100_power_off - power off the sensor
 * @ap0100: pointer to private data structure
 *
 */
void ap0100_power_off(struct ap0100_priv *ap0100)
{
	if (ap0100->pdata->set_xclk)
		ap0100->pdata->set_xclk(&ap0100->subdev, 0);
}


/**
 * ap0100_change_config - issue change config command
 * @ap0100: pointer to private data structure
 * Issue a change config command and return the current state
 */
int ap0100_change_config(struct i2c_client *client)
{
	int count, ret;
	unsigned int data;

	ret = ap0100_write(client, AP0100_CMD_PARAM_0, AP0100_CHANGE_CONFIG);
	if (ret < 0)
		return ret;
	ret = ap0100_write(client, AP0100_COMMAND_REGISTER, AP0100_SET_STATE);
	if (ret < 0)
		return ret;
	data = ap0100_read(client, AP0100_COMMAND_REGISTER);
	count = 0;
	while(data){
		data = ap0100_read(client, AP0100_COMMAND_REGISTER);
		if(count++ > 5){
			printk(KERN_INFO "Failed to set CHANGE CONFIG state: ERROR = 0x%x\n",data);
			break;
		}
		msleep(10);
	}
	return 0;
}


/************************************************************************
			v4l2_subdev_core_ops
************************************************************************/
#define V4L2_CID_TEST_PATTERN           (V4L2_CID_USER_BASE | 0x1001)

static int ap0100_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ap0100_priv *ap0100 = container_of(ctrl->handler,
					struct ap0100_priv, ctrls);
	struct i2c_client *client = v4l2_get_subdevdata(&ap0100->subdev);
	int ret = 0;
	int data = 0;
	int state;

	switch (ctrl->id) {
	case V4L2_CID_HFLIP:
		data = ap0100_read(client, AP0100_READ_MODE);
		if (ctrl->val){
			data |= 0x0001;
			ret = ap0100_write(client, AP0100_READ_MODE, data);
			break;
		}
		data &= 0xfffe;
		ret = ap0100_write(client, AP0100_READ_MODE, data);
		break;
	
	case V4L2_CID_VFLIP:
		data = ap0100_read(client, AP0100_READ_MODE);
		if (ctrl->val) {
			data |= 0x0002;
			ret = ap0100_write(client, AP0100_READ_MODE, data);
			break;
		}
		data &= 0xfffc;
		ret = ap0100_write(client, AP0100_READ_MODE, data);
		break;

	case V4L2_CID_TEST_PATTERN:
		if (!ctrl->val){
			ret = ap0100_write(client, AP0100_MODE_SELECT, AP0100_SENSOR);
			break;
		}
		ret = ap0100_write(client, AP0100_MODE_SELECT, AP0100_TEST_GENERATOR);
		if (ret < 0)
			goto out;
		switch(ctrl->val){
		case 1:
			ret = ap0100_write(client, AP0100_PATTERN_SELECT, AP0100_SOLID_COLOR);
			break;
		default:
			ret = ap0100_write(client, AP0100_PATTERN_SELECT, AP0100_COLOR_BAR);
			break;
		}
	default:
		goto out;
	
	}
	state = ap0100_change_config(client);

out:	
	return ret;
}

static struct v4l2_ctrl_ops ap0100_ctrl_ops = {
	.s_ctrl = ap0100_s_ctrl,
};

static const char * const ap0100_test_pattern_menu[] = {
        "Disabled",
	"Solid color",
	"100% color bars",
};

static const struct v4l2_ctrl_config ap0100_ctrls[] = {
        {
                .ops            = &ap0100_ctrl_ops,
                .id             = V4L2_CID_TEST_PATTERN,
                .type           = V4L2_CTRL_TYPE_MENU,
                .name           = "Test Pattern",
                .min            = 0,
                .max            = ARRAY_SIZE(ap0100_test_pattern_menu) - 1,
                .step           = 0,
                .def            = 0,
                .flags          = 0,
                .menu_skip_mask = 0,
                .qmenu          = ap0100_test_pattern_menu,
        }
};

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int ap0100_g_reg(struct v4l2_subdev *sd,
				struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int data;

	reg->size = 2;
	data = ap0100_read(client, reg->reg);
	if (data < 0)
		return data;

	reg->val = data;
	return 0;
}

static int ap0100_s_reg(struct v4l2_subdev *sd,
				struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	return ap0100_write(client, reg->reg, reg->val);
}
#endif

static int ap0100_s_power(struct v4l2_subdev *sd, int on)
{
	struct ap0100_priv *ap0100 = container_of(sd,
				struct ap0100_priv, subdev);
	int ret = 0;

	mutex_lock(&ap0100->power_lock);

	/*
	* If the power count is modified from 0 to != 0 or from != 0 to 0,
	* update the power state.
	*/
	if (ap0100->power_count == !on) {
		if (on) {
				ap0100_power_on(ap0100);
				ret = v4l2_ctrl_handler_setup(&ap0100->ctrls);
                                if (ret < 0)
                                        goto out;
		} else
			ap0100_power_off(ap0100);
	}
	/* Update the power count. */
	ap0100->power_count += on ? 1 : -1;
	WARN_ON(ap0100->power_count < 0);
out:
	mutex_unlock(&ap0100->power_lock);
	return ret;
}

/***************************************************
		v4l2_subdev_video_ops
****************************************************/

static int ap0100_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret, count;
	u16 data;
	u16 state;

	if (!enable) {
		ret = ap0100_write(client, AP0100_CMD_PARAM_0, AP0100_SUSPEND);
	        if (ret < 0)
        	        return ret;
	        ret = ap0100_write(client, AP0100_COMMAND_REGISTER, AP0100_SET_STATE);
	        if (ret < 0)
        	        return ret;
	        data = ap0100_read(client, AP0100_COMMAND_REGISTER);
        	count = 0;
	        while(data){
        	        data = ap0100_read(client, AP0100_COMMAND_REGISTER);
                	if(count++ > 5){
                        	printk(KERN_INFO "Failed to set SUSPEND state: ERROR = 0x%x\n",data);
                        break;
                	}
                	msleep(10);
        	}
#ifdef AP0100_DEBUG
		ap0100_get_state(client);
#endif	
		return 0;
	}

	state = ap0100_change_config(client);
#ifdef AP0100_DEBUG
	ap0100_get_state(client);
#endif	
	return 0;
}

/***************************************************
		v4l2_subdev_pad_ops
****************************************************/
static int ap0100_enum_mbus_code(struct v4l2_subdev *sd,
				struct v4l2_subdev_fh *fh,
				struct v4l2_subdev_mbus_code_enum *code)
{
	struct ap0100_priv *ap0100 = container_of(sd,
					struct ap0100_priv, subdev);

	if (code->pad || code->index)
		return -EINVAL;

	code->code = ap0100->format.code;
	return 0;
}

static int ap0100_enum_frame_size(struct v4l2_subdev *sd,
				struct v4l2_subdev_fh *fh,
				struct v4l2_subdev_frame_size_enum *fse)
{
	struct ap0100_priv *ap0100 = container_of(sd,
					struct ap0100_priv, subdev);

	if (fse->index != 0 || fse->code != ap0100->format.code)
		return -EINVAL;

	fse->min_width = AP0100_WINDOW_WIDTH_MIN;
	fse->max_width = AP0100_WINDOW_WIDTH_MAX;
	fse->min_height = AP0100_WINDOW_HEIGHT_MIN;
	fse->max_height = AP0100_WINDOW_HEIGHT_MAX;

	return 0;
}

static struct v4l2_mbus_framefmt *
__ap0100_get_pad_format(struct ap0100_priv *ap0100, struct v4l2_subdev_fh *fh,
			unsigned int pad, u32 which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_format(fh, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &ap0100->format;
	default:
		return NULL;
	}
}

static struct v4l2_rect *
__ap0100_get_pad_crop(struct ap0100_priv *ap0100, struct v4l2_subdev_fh *fh,
	unsigned int pad, u32 which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_crop(fh, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &ap0100->crop;
	default:
		return NULL;
	}
}

static int ap0100_get_format(struct v4l2_subdev *sd,
				struct v4l2_subdev_fh *fh,
				struct v4l2_subdev_format *fmt)
{
	struct ap0100_priv *ap0100 = container_of(sd,
					struct ap0100_priv, subdev);

	fmt->format = *__ap0100_get_pad_format(ap0100, fh, fmt->pad,
						fmt->which);

	return 0;
}

static int ap0100_set_format(struct v4l2_subdev *sd,
				struct v4l2_subdev_fh *fh,
				struct v4l2_subdev_format *format)
{
	struct ap0100_priv *ap0100 = container_of(sd,
					struct ap0100_priv, subdev);

	ap0100->format.width	= AP0100_WINDOW_WIDTH_DEF;
	ap0100->format.height	= AP0100_WINDOW_HEIGHT_DEF;
	ap0100->format.code 	= V4L2_MBUS_FMT_YUYV8_1X16;

	format->format = ap0100->format;

	return 0;
}

static int ap0100_get_crop(struct v4l2_subdev *sd,
			struct v4l2_subdev_fh *fh,
			struct v4l2_subdev_crop *crop)
{
	struct ap0100_priv *ap0100 = container_of(sd,
					struct ap0100_priv, subdev);

	crop->rect = *__ap0100_get_pad_crop(ap0100, fh, crop->pad, crop->which);

	return 0;
}

static int ap0100_set_crop(struct v4l2_subdev *sd,
			struct v4l2_subdev_fh *fh,
			struct v4l2_subdev_crop *crop)
{
	struct ap0100_priv *ap0100 = container_of(sd,
					struct ap0100_priv, subdev);

	ap0100->crop.left	= crop->rect.left;
	ap0100->crop.top	= crop->rect.top;
	ap0100->crop.width	= crop->rect.width;
	ap0100->crop.height	= crop->rect.height;

	return 0;
}

/***********************************************************
	V4L2 subdev internal operations
************************************************************/
static int ap0100_registered(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ap0100_priv *ap0100 = to_ap0100(client);
	s32 data;

	ap0100_power_on(ap0100);

	/* Read out the chip version register */
	data = ap0100_read(client, AP0100_CHIP_VERSION_REG);
	if (data != AP0100_CHIP_ID) {
		dev_err(&client->dev, "AP0100 not detected, chip ID read:0x%4.4x\n",
				data);
		return -ENODEV;
	}
	dev_info(&client->dev, "AP0100 detected at address 0x%02x:chip ID = 0x%4.4x\n",
			client->addr, AP0100_CHIP_ID);


	ap0100_power_off(ap0100);

	return 0;
}

static int ap0100_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	return ap0100_s_power(sd, 1);
}

static int ap0100_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	return ap0100_s_power(sd, 0);
}

/***************************************************
		v4l2_subdev_ops
****************************************************/
static struct v4l2_subdev_core_ops ap0100_subdev_core_ops = {
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register	= ap0100_g_reg,
	.s_register	= ap0100_s_reg,
#endif
	.s_power	= ap0100_s_power,
};

static struct v4l2_subdev_video_ops ap0100_subdev_video_ops = {
	.s_stream	= ap0100_s_stream,
};

static struct v4l2_subdev_pad_ops ap0100_subdev_pad_ops = {
	.enum_mbus_code	 = ap0100_enum_mbus_code,
	.enum_frame_size = ap0100_enum_frame_size,
	.get_fmt	 = ap0100_get_format,
	.set_fmt	 = ap0100_set_format,
	.get_crop	 = ap0100_get_crop,
	.set_crop	 = ap0100_set_crop,
};

static struct v4l2_subdev_ops ap0100_subdev_ops = {
	.core	= &ap0100_subdev_core_ops,
	.video	= &ap0100_subdev_video_ops,
	.pad	= &ap0100_subdev_pad_ops,
};

/*
 * Internal ops. Never call this from drivers, only the v4l2 framework can call
 * these ops.
 */
static const struct v4l2_subdev_internal_ops ap0100_subdev_internal_ops = {
	.registered	= ap0100_registered,
	.open		= ap0100_open,
	.close		= ap0100_close,
};

/***************************************************
		I2C driver
****************************************************/
static int ap0100_probe(struct i2c_client *client,
			const struct i2c_device_id *did)
{
	struct ap0100_platform_data *pdata = client->dev.platform_data;
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct ap0100_priv *ap0100;
	int ret;
	int i;

	if (pdata == NULL) {
		dev_err(&client->dev, "No platform data\n");
		return -EINVAL;
	}

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WORD_DATA)) {
		dev_warn(&client->dev, "I2C-Adapter doesn't support I2C_FUNC_SMBUS_WORD\n");
		return -EIO;
	}

	ap0100 = devm_kzalloc(&client->dev, sizeof(struct ap0100_priv),
				GFP_KERNEL);
	if (ap0100 == NULL)
		return -ENOMEM;

	ap0100->pdata = pdata;

	v4l2_ctrl_handler_init(&ap0100->ctrls,ARRAY_SIZE(ap0100_ctrls) + 2);

	v4l2_ctrl_new_std(&ap0100->ctrls, &ap0100_ctrl_ops,
				V4L2_CID_HFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(&ap0100->ctrls, &ap0100_ctrl_ops,
				V4L2_CID_VFLIP, 0, 1, 1, 0);
	for (i = 0; i < ARRAY_SIZE(ap0100_ctrls); ++i)
		v4l2_ctrl_new_custom(&ap0100->ctrls, &ap0100_ctrls[i], NULL);
	
	ap0100->subdev.ctrl_handler = &ap0100->ctrls;

	if (ap0100->ctrls.error) {
		ret = ap0100->ctrls.error;
		dev_err(&client->dev, "Control initialization error: %d\n",
			ret);
		goto done;
	}

	mutex_init(&ap0100->power_lock);
	v4l2_i2c_subdev_init(&ap0100->subdev, client, &ap0100_subdev_ops);
	ap0100->subdev.internal_ops = &ap0100_subdev_internal_ops;

	ap0100->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_init(&ap0100->subdev.entity, 1, &ap0100->pad, 0);
	if (ret < 0)
		goto done;

	ap0100->subdev.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	ap0100->crop.width	= AP0100_WINDOW_WIDTH_DEF;
	ap0100->crop.height	= AP0100_WINDOW_HEIGHT_DEF;
	ap0100->crop.left	= AP0100_COLUMN_START_DEF;
	ap0100->crop.top	= AP0100_ROW_START_DEF;

	ap0100->format.code 		= V4L2_MBUS_FMT_YUYV8_1X16;
	ap0100->format.width		= AP0100_WINDOW_WIDTH_DEF;
	ap0200->format.height		= AP0100_WINDOW_HEIGHT_DEF;
	ap0200->format.field		= V4L2_FIELD_NONE;
	ap0200->format.colorspace	= V4L2_COLORSPACE_SRGB;

done:
	if (ret < 0) {
		v4l2_ctrl_handler_free(&ap0100->ctrls);
		media_entity_cleanup(&ap0100->subdev.entity);
		dev_err(&client->dev, "Probe failed\n");
	}

	return ret;
}

static int ap0100_remove(struct i2c_client *client)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct ap0100_priv *ap0100 = to_ap0100(client);

	v4l2_ctrl_handler_free(&ap0100->ctrls);
	v4l2_device_unregister_subdev(subdev);
	media_entity_cleanup(&subdev->entity);

	return 0;
}

static const struct i2c_device_id ap0200_id[] = {
	{ "ap0200", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ap0200_id);

static struct i2c_driver ap0200_i2c_driver = {
	.driver = {
		 .name = "ap0200",
	},
	.probe    = ap0200_probe,
	.remove   = ap0200_remove,
	.id_table = ap0200_id,
};

static int __init ap0200_module_init(void)
{
	return i2c_add_driver(&ap0200_i2c_driver);
}

static void __exit ap0200_module_exit(void)
{
	i2c_del_driver(&ap0200_i2c_driver);
}
module_init(ap0200_module_init);
module_exit(ap0200_module_exit);

MODULE_DESCRIPTION("Aptina AP0100 Camera driver");
MODULE_AUTHOR("Aptina Imaging <drivers@aptina.com>");
MODULE_LICENSE("GPL v2");
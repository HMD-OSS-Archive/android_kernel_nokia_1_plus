/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": %s: " fmt, __func__

#include <linux/types.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/slab.h>

#include "flashlight-core.h"
#include "flashlight-dt.h"

/* define device tree */
#ifndef LM36010_DTNAME_I2C
#define LM36010_DTNAME_I2C "mediatek,flashlights_lm36010_i2c"
#endif

#define LM36010_NAME "flashlights-lm36010"

/* define registers */
#define LM36010_REG_ENABLE   (0x01)
#define LM36010_ENABLE_TORCH (0x22)
#define LM36010_ENABLE_FLASH (0x23)
#define LM36010_DISABLE      (0x20)

#define LM36010_REG_FLASH_LEVEL (0x03)
#define LM36010_REG_TORCH_LEVEL (0x04)

//#define LM36010_REG_LEVEL (0x09)

//#define LM36010_REG_RESET (0x00)
//#define LM36010_FLASH_RESET (0x80)

#define LM36010_REG_FLASH_FEATURE (0x02)
#define LM36010_FLASH_TIMEOUT (0x15)

/* define level */
#define LM36010_LEVEL_NUM 18
#define LM36010_LEVEL_TORCH 4
#define LM36010_HW_TIMEOUT 800 /* ms */

/* define mutex and work queue */
static DEFINE_MUTEX(lm36010_mutex);
static struct work_struct lm36010_work;

/* define usage count */
static int use_count;

/* define i2c */
static struct i2c_client *lm36010_i2c_client;

/* platform data */
struct lm36010_platform_data {
	int channel_num;
	struct flashlight_device_id *dev_id;
};

/* lm36010 chip data */
struct lm36010_chip_data {
	struct i2c_client *client;
	struct lm36010_platform_data *pdata;
	struct mutex lock;
};


/******************************************************************************
 * lm36010 operations
 *****************************************************************************/
static const int lm36010_current[LM36010_LEVEL_NUM] = {
	 49,  93,  140,  187,  281,  375,  468,  562, 656, 750,
	843, 937, 1031, 1125, 1218, 1312, 1406, 1500
};

static const unsigned char lm36010_flash_level[LM36010_LEVEL_NUM] = {
	0x2E, 0x2E, 0x20, 0x2f, 0x00, 0x09, 0x15, 0x1b, 0x2f, 0x3f,
	0x47, 0x5f, 0x60, 0x64, 0x68, 0x6c, 0x70, 0x7F
};

static int lm36010_level = -1;

static int lm36010_is_torch(int level)
{
	if (level >= LM36010_LEVEL_TORCH)
		return -1;

	return 0;
}

static int lm36010_verify_level(int level)
{
	if (level < 0)
		level = 0;
	else if (level >= LM36010_LEVEL_NUM)
		level = LM36010_LEVEL_NUM - 1;

	return level;
}

/* i2c wrapper function */
static int lm36010_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	int ret;
	struct lm36010_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	ret = i2c_smbus_write_byte_data(client, reg, val);
	mutex_unlock(&chip->lock);

	if (ret < 0)
		pr_err("failed writing at 0x%02x\n", reg);

	return ret;
}

/* flashlight enable function */
static int lm36010_enable(void)
{
	unsigned char reg, val;

	reg = LM36010_REG_ENABLE;
	if (!lm36010_is_torch(lm36010_level)) {
		/* torch mode */
		val = LM36010_ENABLE_TORCH;
	} else {
		/* flash mode */
		val = LM36010_ENABLE_FLASH;
	}

	return lm36010_write_reg(lm36010_i2c_client, reg, val);
}

/* flashlight disable function */
static int lm36010_disable(void)
{
	unsigned char reg, val;

	reg = LM36010_REG_ENABLE;
	val = LM36010_DISABLE;

	return lm36010_write_reg(lm36010_i2c_client, reg, val);
}

/* set flashlight level */
static int lm36010_set_level(int level)
{
	unsigned char reg, val;

	level = lm36010_verify_level(level);
	lm36010_level = level;

	if (level <= 4)
		reg = LM36010_REG_TORCH_LEVEL;
	else
		reg = LM36010_REG_FLASH_LEVEL;
	val = lm36010_flash_level[level];

	return lm36010_write_reg(lm36010_i2c_client, reg, val);
}

/* flashlight init */
int lm36010_init(void)
{
	int ret;
	unsigned char reg, val;

	/* reset chip */
	//reg = LM36010_REG_RESET;
	//val = LM36010_FLASH_RESET;
	//ret = lm36010_write_reg(lm36010_i2c_client, reg, val);

	/* set flash timeout */
	reg = LM36010_REG_FLASH_FEATURE;
	val = LM36010_FLASH_TIMEOUT;
	ret = lm36010_write_reg(lm36010_i2c_client, reg, val);

	return ret;
}

/* flashlight uninit */
int lm36010_uninit(void)
{
	lm36010_disable();

	return 0;
}

/******************************************************************************
 * Timer and work queue
 *****************************************************************************/
static struct hrtimer lm36010_timer;
static unsigned int lm36010_timeout_ms;

static void lm36010_work_disable(struct work_struct *data)
{
	pr_debug("work queue callback\n");
	lm36010_disable();
}

static enum hrtimer_restart lm36010_timer_func(struct hrtimer *timer)
{
	schedule_work(&lm36010_work);
	return HRTIMER_NORESTART;
}


/******************************************************************************
 * Flashlight operations
 *****************************************************************************/
static int lm36010_ioctl(unsigned int cmd, unsigned long arg)
{
	struct flashlight_dev_arg *fl_arg;
	int channel;
	ktime_t ktime;

	fl_arg = (struct flashlight_dev_arg *)arg;
	channel = fl_arg->channel;

	switch (cmd) {
	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		pr_debug("FLASH_IOC_SET_TIME_OUT_TIME_MS(%d): %d\n",
				channel, (int)fl_arg->arg);
		lm36010_timeout_ms = fl_arg->arg;
		break;

	case FLASH_IOC_SET_DUTY:
		pr_debug("FLASH_IOC_SET_DUTY(%d): %d\n",
				channel, (int)fl_arg->arg);
		lm36010_set_level(fl_arg->arg);
		break;

	case FLASH_IOC_SET_ONOFF:
		pr_debug("FLASH_IOC_SET_ONOFF(%d): %d\n",
				channel, (int)fl_arg->arg);
		if (fl_arg->arg == 1) {
			if (lm36010_timeout_ms) {
				ktime = ktime_set(lm36010_timeout_ms / 1000,
						(lm36010_timeout_ms % 1000) * 1000000);
				hrtimer_start(&lm36010_timer, ktime, HRTIMER_MODE_REL);
			}
			lm36010_enable();
		} else {
			lm36010_disable();
			hrtimer_cancel(&lm36010_timer);
		}
		break;

	case FLASH_IOC_GET_DUTY_NUMBER:
		pr_debug("FLASH_IOC_GET_DUTY_NUMBER(%d)\n", channel);
		fl_arg->arg = LM36010_LEVEL_NUM;
		break;

	case FLASH_IOC_GET_MAX_TORCH_DUTY:
		pr_debug("FLASH_IOC_GET_MAX_TORCH_DUTY(%d)\n", channel);
		fl_arg->arg = LM36010_LEVEL_TORCH - 1;
		break;

	case FLASH_IOC_GET_DUTY_CURRENT:
		fl_arg->arg = lm36010_verify_level(fl_arg->arg);
		pr_debug("FLASH_IOC_GET_DUTY_CURRENT(%d): %d\n",
				channel, (int)fl_arg->arg);
		fl_arg->arg = lm36010_current[fl_arg->arg];
		break;

	case FLASH_IOC_GET_HW_TIMEOUT:
		pr_debug("FLASH_IOC_GET_HW_TIMEOUT(%d)\n", channel);
		fl_arg->arg = LM36010_HW_TIMEOUT;
		break;

	default:
		pr_info("No such command and arg(%d): (%d, %d)\n",
				channel, _IOC_NR(cmd), (int)fl_arg->arg);
		return -ENOTTY;
	}

	return 0;
}

static int lm36010_open(void)
{
	/* Move to set driver for saving power */
	return 0;
}

static int lm36010_release(void)
{
	/* Move to set driver for saving power */
	return 0;
}

static int lm36010_set_driver(int set)
{
	int ret = 0;

	/* set chip and usage count */
	mutex_lock(&lm36010_mutex);
	if (set) {
		if (!use_count)
			ret = lm36010_init();
		use_count++;
		pr_debug("Set driver: %d\n", use_count);
	} else {
		use_count--;
		if (!use_count)
			ret = lm36010_uninit();
		if (use_count < 0)
			use_count = 0;
		pr_debug("Unset driver: %d\n", use_count);
	}
	mutex_unlock(&lm36010_mutex);

	return ret;
}

static ssize_t lm36010_strobe_store(struct flashlight_arg arg)
{
	lm36010_set_driver(1);
	lm36010_set_level(arg.level);
	lm36010_timeout_ms = 0;
	lm36010_enable();
	if (arg.dur == 0) // to flash on without timeout
	    return 0;
	msleep(arg.dur);
	lm36010_disable();
	lm36010_set_driver(0);

	return 0;
}

static struct flashlight_operations lm36010_ops = {
	lm36010_open,
	lm36010_release,
	lm36010_ioctl,
	lm36010_strobe_store,
	lm36010_set_driver
};


/******************************************************************************
 * I2C device and driver
 *****************************************************************************/
static int lm36010_chip_init(struct lm36010_chip_data *chip)
{
	/* NOTE: Chip initialication move to "set driver" operation for power saving issue.
	 * lm36010_init();
	 */

	return 0;
}

static int lm36010_parse_dt(struct device *dev,
		struct lm36010_platform_data *pdata)
{
	struct device_node *np, *cnp;
	u32 decouple = 0;
	int i = 0;

	if (!dev || !dev->of_node || !pdata)
		return -ENODEV;

	np = dev->of_node;

	pdata->channel_num = of_get_child_count(np);
	if (!pdata->channel_num) {
		pr_info("Parse no dt, node.\n");
		return 0;
	}
	pr_info("Channel number(%d).\n", pdata->channel_num);

	if (of_property_read_u32(np, "decouple", &decouple))
		pr_info("Parse no dt, decouple.\n");

	pdata->dev_id = devm_kzalloc(dev,
			pdata->channel_num * sizeof(struct flashlight_device_id),
			GFP_KERNEL);
	if (!pdata->dev_id)
		return -ENOMEM;

	for_each_child_of_node(np, cnp) {
		if (of_property_read_u32(cnp, "type", &pdata->dev_id[i].type))
			goto err_node_put;
		if (of_property_read_u32(cnp, "ct", &pdata->dev_id[i].ct))
			goto err_node_put;
		if (of_property_read_u32(cnp, "part", &pdata->dev_id[i].part))
			goto err_node_put;
		snprintf(pdata->dev_id[i].name, FLASHLIGHT_NAME_SIZE, LM36010_NAME);
		pdata->dev_id[i].channel = i;
		pdata->dev_id[i].decouple = decouple;

		pr_info("Parse dt (type,ct,part,name,channel,decouple)=(%d,%d,%d,%s,%d,%d).\n",
				pdata->dev_id[i].type, pdata->dev_id[i].ct,
				pdata->dev_id[i].part, pdata->dev_id[i].name,
				pdata->dev_id[i].channel, pdata->dev_id[i].decouple);
		i++;
	}

	return 0;

err_node_put:
	of_node_put(cnp);
	return -EINVAL;
}

static int lm36010_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct lm36010_platform_data *pdata = dev_get_platdata(&client->dev);
	struct lm36010_chip_data *chip;
	int err;
	int i;

	pr_debug("Probe start.\n");

	/* check i2c */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("Failed to check i2c functionality.\n");
		err = -ENODEV;
		goto err_out;
	}

	/* init chip private data */
	chip = kzalloc(sizeof(struct lm36010_chip_data), GFP_KERNEL);
	if (!chip) {
		err = -ENOMEM;
		goto err_out;
	}
	chip->client = client;

	/* init platform data */
	if (!pdata) {
		pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
		if (!pdata) {
			err = -ENOMEM;
			goto err_free;
		}
		client->dev.platform_data = pdata;
		err = lm36010_parse_dt(&client->dev, pdata);
		if (err)
			goto err_free;
	}
	chip->pdata = pdata;
	i2c_set_clientdata(client, chip);
	lm36010_i2c_client = client;

	/* init mutex and spinlock */
	mutex_init(&chip->lock);

	/* init work queue */
	INIT_WORK(&lm36010_work, lm36010_work_disable);

	/* init timer */
	hrtimer_init(&lm36010_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	lm36010_timer.function = lm36010_timer_func;
	lm36010_timeout_ms = 400;

	/* init chip hw */
	lm36010_chip_init(chip);

	/* clear usage count */
	use_count = 0;

	/* register flashlight device */
	if (pdata->channel_num) {
		for (i = 0; i < pdata->channel_num; i++)
			if (flashlight_dev_register_by_device_id(&pdata->dev_id[i], &lm36010_ops)) {
				err = -EFAULT;
				goto err_free;
			}
	} else {
		if (flashlight_dev_register(LM36010_NAME, &lm36010_ops)) {
			err = -EFAULT;
			goto err_free;
		}
	}

	pr_debug("Probe done.\n");

	return 0;

err_free:
	i2c_set_clientdata(client, NULL);
	kfree(chip);
err_out:
	return err;
}

static int lm36010_i2c_remove(struct i2c_client *client)
{
	struct lm36010_platform_data *pdata = dev_get_platdata(&client->dev);
	struct lm36010_chip_data *chip = i2c_get_clientdata(client);
	int i;

	pr_debug("Remove start.\n");

	client->dev.platform_data = NULL;

	/* unregister flashlight device */
	if (pdata && pdata->channel_num)
		for (i = 0; i < pdata->channel_num; i++)
			flashlight_dev_unregister_by_device_id(&pdata->dev_id[i]);
	else
		flashlight_dev_unregister(LM36010_NAME);

	/* flush work queue */
	flush_work(&lm36010_work);

	/* free resource */
	kfree(chip);

	pr_debug("Remove done.\n");

	return 0;
}

static const struct i2c_device_id lm36010_i2c_id[] = {
	{LM36010_NAME, 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, lm36010_i2c_id);

#ifdef CONFIG_OF
static const struct of_device_id lm36010_i2c_of_match[] = {
	{.compatible = LM36010_DTNAME_I2C},
	{},
};
MODULE_DEVICE_TABLE(of, lm36010_i2c_of_match);
#endif

static struct i2c_driver lm36010_i2c_driver = {
	.driver = {
		.name = LM36010_NAME,
#ifdef CONFIG_OF
		.of_match_table = lm36010_i2c_of_match,
#endif
	},
	.probe = lm36010_i2c_probe,
	.remove = lm36010_i2c_remove,
	.id_table = lm36010_i2c_id,
};

module_i2c_driver(lm36010_i2c_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Simon Wang <Simon-TCH.Wang@mediatek.com>");
MODULE_DESCRIPTION("MTK Flashlight LM36010 Driver");

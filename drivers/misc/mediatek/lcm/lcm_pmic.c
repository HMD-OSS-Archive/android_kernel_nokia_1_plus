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

#include <linux/regulator/consumer.h>
#include <linux/string.h>
#include <linux/kernel.h>

#if defined(CONFIG_RT5081_PMU_DSV) || defined(CONFIG_MT6370_PMU_DSV)
static struct regulator *disp_bias_pos;
static struct regulator *disp_bias_neg;
static int regulator_inited;

int display_bias_regulator_init(void)
{
	int ret = 0;

	if (regulator_inited)
		return ret;

	/* please only get regulator once in a driver */
	disp_bias_pos = regulator_get(NULL, "dsv_pos");
	if (IS_ERR(disp_bias_pos)) { /* handle return value */
		ret = PTR_ERR(disp_bias_pos);
		pr_err("get dsv_pos fail, error: %d\n", ret);
		return ret;
	}

	disp_bias_neg = regulator_get(NULL, "dsv_neg");
	if (IS_ERR(disp_bias_neg)) { /* handle return value */
		ret = PTR_ERR(disp_bias_neg);
		pr_err("get dsv_neg fail, error: %d\n", ret);
		return ret;
	}

	regulator_inited = 1;
	return ret; /* must be 0 */

}
EXPORT_SYMBOL(display_bias_regulator_init);

int display_bias_enable(void)
{
	int ret = 0;
	int retval = 0;

	display_bias_regulator_init();

	/* set voltage with min & max*/
	ret = regulator_set_voltage(disp_bias_pos, 5400000, 5400000);
	if (ret < 0)
		pr_err("set voltage disp_bias_pos fail, ret = %d\n", ret);
	retval |= ret;

	ret = regulator_set_voltage(disp_bias_neg, 5400000, 5400000);
	if (ret < 0)
		pr_err("set voltage disp_bias_neg fail, ret = %d\n", ret);
	retval |= ret;

#if 0
	/* get voltage */
	ret = mtk_regulator_get_voltage(&disp_bias_pos);
	if (ret < 0)
		pr_err("get voltage disp_bias_pos fail\n");
	pr_debug("pos voltage = %d\n", ret);

	ret = mtk_regulator_get_voltage(&disp_bias_neg);
	if (ret < 0)
		pr_err("get voltage disp_bias_neg fail\n");
	pr_debug("neg voltage = %d\n", ret);
#endif
	/* enable regulator */
	ret = regulator_enable(disp_bias_pos);
	if (ret < 0)
		pr_err("enable regulator disp_bias_pos fail, ret = %d\n", ret);
	retval |= ret;

	ret = regulator_enable(disp_bias_neg);
	if (ret < 0)
		pr_err("enable regulator disp_bias_neg fail, ret = %d\n", ret);
	retval |= ret;

	return retval;
}
EXPORT_SYMBOL(display_bias_enable);

int display_bias_disable(void)
{
	int ret = 0;
	int retval = 0;

	display_bias_regulator_init();

	ret = regulator_disable(disp_bias_neg);
	if (ret < 0)
		pr_err("disable regulator disp_bias_neg fail, ret = %d\n", ret);
	retval |= ret;

	ret = regulator_disable(disp_bias_pos);
	if (ret < 0)
		pr_err("disable regulator disp_bias_pos fail, ret = %d\n", ret);
	retval |= ret;

	return retval;
}
EXPORT_SYMBOL(display_bias_disable);

#else
int display_bias_regulator_init(void)
{
	return 0;
}
EXPORT_SYMBOL(display_bias_regulator_init);

int display_bias_enable(void)
{
	return 0;
}
EXPORT_SYMBOL(display_bias_enable);

int display_bias_disable(void)
{
	return 0;
}
EXPORT_SYMBOL(display_bias_disable);
#endif

#ifdef CONFIG_FIH_LCM_VGP_SUPPLY
struct regulator *fih_lcm_vgp;
bool vgp_is_set_in_lk = false;
extern unsigned int pmic_config_interface(unsigned int RegNum, unsigned int val, unsigned int MASK, unsigned int SHIFT);
/* get LDO supply */
int fih_lcm_get_vgp_supply(void)
{
	int ret;
	struct regulator *lcm_vgp_ldo;

	printk(KERN_INFO "LCM: lcm_get_vgp_supply is going\n");

	lcm_vgp_ldo = regulator_get(NULL, "vldo28");
	if (IS_ERR(lcm_vgp_ldo)) {
		ret = PTR_ERR(lcm_vgp_ldo);
		printk(KERN_ERR "failed to get lcm vldo28, %d\n", ret);
		return ret;
	}

	printk(KERN_INFO "LCM: lcm get supply ok.\n");

	/* get current voltage settings */
	ret = regulator_get_voltage(lcm_vgp_ldo);
	if(ret == 2800000){
		vgp_is_set_in_lk = true;
	}
	printk(KERN_INFO "lcm LDO voltage = %d in LK stage\n", ret);

	fih_lcm_vgp = lcm_vgp_ldo;

	return ret;
}
EXPORT_SYMBOL(fih_lcm_get_vgp_supply);

int fih_lcm_vgp_supply_enable(void)
{
	int ret;
	unsigned int volt;

	printk(KERN_INFO "LCM: lcm_vgp_supply_enable\n");

	if (fih_lcm_vgp == NULL){
		printk(KERN_ERR "LCM: fih_lcm_vgp is null\n");
		return 0;
	}

	printk(KERN_INFO "LCM: set regulator voltage lcm_vgp voltage to 2.8V\n");
	/* set voltage to 2.8V */
	ret = regulator_set_voltage(fih_lcm_vgp, 2800000, 2800000);
	if (ret != 0) {
		printk(KERN_ERR "LCM: lcm failed to set lcm_vgp voltage: %d\n", ret);
		return ret;
	}

	/* get voltage settings again */
	volt = regulator_get_voltage(fih_lcm_vgp);
	if (volt == 2800000)
		printk(KERN_ERR "LCM: check regulator voltage=2800000 pass!\n");
	else
		printk(KERN_ERR "LCM: check regulator voltage=2800000 fail! (voltage: %d)\n", volt);

	ret = regulator_enable(fih_lcm_vgp);
	if (ret != 0) {
		printk(KERN_ERR "LCM: Failed to enable lcm_vgp: %d\n", ret);
		return ret;
	}

	return ret;
}
EXPORT_SYMBOL(fih_lcm_vgp_supply_enable);

int fih_lcm_vgp_supply_disable(void)
{
	int ret = 0;
	unsigned int isenable;

	if (fih_lcm_vgp == NULL){
		printk(KERN_ERR "LCM: fih_lcm_vgp is null\n");
		return 0;
	}

	if(vgp_is_set_in_lk){
		pmic_config_interface(0x1C24, 0x0, 0x03, 8);
		pmic_config_interface(0x1B40, 0x0, 0x01, 0);
		vgp_is_set_in_lk = false;
		printk(KERN_INFO "LCM: force disable regulator, because on in lk\n");
		return 0;
	}

	/* disable regulator */
	isenable = regulator_is_enabled(fih_lcm_vgp);

	printk(KERN_INFO "LCM: lcm query regulator enable status[%d]\n", isenable);

	if (isenable) {
		ret = regulator_disable(fih_lcm_vgp);
		if (ret != 0) {
			printk(KERN_ERR "LCM: lcm failed to disable lcm_vgp: %d\n", ret);
			return ret;
		}
		/* verify */
		isenable = regulator_is_enabled(fih_lcm_vgp);
		if (!isenable)
			printk(KERN_ERR "LCM: lcm regulator disable pass\n");
	}

	return ret;
}
EXPORT_SYMBOL(fih_lcm_vgp_supply_disable);

#endif

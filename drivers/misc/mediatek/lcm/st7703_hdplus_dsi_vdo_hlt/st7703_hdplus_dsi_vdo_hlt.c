/* BEGIN PN: , Added by h84013687, 2013.08.13*/
#ifndef BUILD_LK
#include <linux/string.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
#include <platform/disp_drv_platform.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
//#include <mach/gpio_const.h>
#include <mt-plat/mtk_gpio.h>
#endif

#ifdef BUILD_LK
#define LCD_DEBUG(fmt, args...)  dprintf(CRITICAL, fmt)
#else
#define LCD_DEBUG(fmt, args...)  pr_debug("[KERNEL/LCM]"fmt, ##args)
#endif

#define I2C_I2C_LCD_BIAS_CHANNEL 	1
#define GPIO_LCD_BIAS_ENP_PIN		(GPIO102 | 0x80000000)
#define GPIO_LCD_BIAS_ENN_PIN		(GPIO101 | 0x80000000)
//#define GPIO_DISP_LRSTB_PIN			(GPIO146 | 0x80000000)
//#define GPIO_LCDBL_EN_PIN			(GPIO82  | 0x80000000)

//extern u16 fih_hwid;

/*********************************************************
 * Gate Driver
 *********************************************************/

/*****************************************************************************
 * Define
 *****************************************************************************/
/*for I2C channel 1*/
#define TPS_I2C_BUSNUM	I2C_I2C_LCD_BIAS_CHANNEL
#define TPS_ADDR		0x3E
#define I2C_ID_NAME		"tps65132"

#ifndef BUILD_LK
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
/* #include <linux/jiffies.h> */
#include <linux/uaccess.h>
/* #include <linux/delay.h> */
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>

/*****************************************************************************
 * GLobal Variable
 *****************************************************************************/
static const struct of_device_id lcm_of_match[] = {
		{.compatible = "mediatek,I2C_LCD_BIAS"},
		{},
};

static struct i2c_client *tps65132_i2c_client;

/*****************************************************************************
 * Function Prototype
 *****************************************************************************/
static int tps65132_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tps65132_remove(struct i2c_client *client);


/*****************************************************************************
 * Data Structure
 *****************************************************************************/
struct tps65132_dev {
	struct i2c_client *client;
};

static const struct i2c_device_id tps65132_id[] = {
	{I2C_ID_NAME, 0},
	{}
};

/* #if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36)) */
/* static struct i2c_client_address_data addr_data = { .forces = forces,}; */
/* #endif */
static struct i2c_driver tps65132_iic_driver = {
	.id_table = tps65132_id,
	.probe = tps65132_probe,
	.remove = tps65132_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "tps65132",
		.of_match_table = lcm_of_match,
	},
};

/*****************************************************************************
 * Extern Area
 *****************************************************************************/



/*****************************************************************************
 * Function
 *****************************************************************************/
static int tps65132_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	printk("tps65132_iic_probe\n");
	pr_info("TPS: info==>name=%s addr=0x%x\n", client->name, client->addr);
	tps65132_i2c_client = client;
	return 0;
}

static int tps65132_remove(struct i2c_client *client)
{
	printk("tps65132_remove\n");
	tps65132_i2c_client = NULL;
	i2c_unregister_device(client);
	return 0;
}

static int tps65132_write_bytes(unsigned char addr, unsigned char value)
{
	int ret = 0;
	struct i2c_client *client = tps65132_i2c_client;
	char write_data[2] = { 0 };
	write_data[0] = addr;
	write_data[1] = value;
	ret = i2c_master_send(client, write_data, 2);
	if (ret < 0)
		printk("tps65132 write data fail !!\n");
	return ret;
}


/*
 * module load/unload record keeping
 */

static int __init tps65132_iic_init(void)
{
	printk("tps65132_iic_init\n");
	i2c_add_driver(&tps65132_iic_driver);
	printk("tps65132_iic_init success\n");
	return 0;
}

static void __exit tps65132_iic_exit(void)
{
	printk("tps65132_iic_exit\n");
	i2c_del_driver(&tps65132_iic_driver);
}


module_init(tps65132_iic_init);
module_exit(tps65132_iic_exit);

MODULE_AUTHOR("Xiaokuan Shi");
MODULE_DESCRIPTION("MTK TPS65132 I2C Driver");

#else
#include <platform/mt_i2c.h>
static int tps65132_write_bytes(unsigned char addr, unsigned char value)
{
	int ret = 0;
	struct mt_i2c_t tps65132_i2c;
	char write_data[2] = { 0 };

	write_data[0] = addr;
	write_data[1] = value;

	tps65132_i2c.id    = TPS_I2C_BUSNUM;
	tps65132_i2c.addr  = TPS_ADDR;
	tps65132_i2c.mode  = ST_MODE;
	tps65132_i2c.speed = 100;

	ret = i2c_write(&tps65132_i2c, write_data, 2);
	if (ret < 0)
		printf("tps65132 write data fail !!\n");
	return ret;
}
#endif


//const static unsigned char LCD_MODULE_ID = 0x09;	//ID0->1;ID1->X
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
#define LCM_DSI_CMD_MODE		0
#define FRAME_WIDTH  			(720)
#define FRAME_HEIGHT 			(1440)
#define LCM_DENSITY			(240)

#define LCM_PHYSICAL_WIDTH		(61877)
#define LCM_PHYSICAL_HEIGHT		(126978)
#define REGFLAG_DELAY           	0xFC
#define REGFLAG_END_OF_TABLE    	0xFD   // END OF REGISTERS MARKER

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

const static unsigned int BL_MIN_LEVEL =20;
static LCM_UTIL_FUNCS lcm_util = {0};

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
#define SET_RESET_PIN(v)  					(lcm_util.set_reset_pin((v)))

#define MDELAY(n) 						(lcm_util.mdelay(n))
#define UDELAY(n)						(lcm_util.udelay(n))

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)						lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)			lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)						lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   		lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)
#define set_gpio_lcd_enp(cmd) 					lcm_util.set_gpio_lcd_enp_bias(cmd)
struct LCM_setting_table {
	unsigned char cmd;
	unsigned char count;
	unsigned char para_list[128];
};

//update initial param for IC st7703 0.01
static struct LCM_setting_table lcm_initialization_setting_3lane[] = {
	{0x11,0,{}},
	{REGFLAG_DELAY, 120, {}},
	{0x36,1,{0xc0}},
	{0x29,0,{}},
	{REGFLAG_DELAY, 20, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}

};

static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
	//	Display off sequence
	//	{0x28, 1, {0x00}},
	{0x28, 0, {}},
	{REGFLAG_DELAY, 20, {}},

	//	Sleep Mode On
	//	{0x10, 1, {0x00}},
	{0x10, 0, {}},
#if 1
	{REGFLAG_DELAY, 120, {}},
#endif
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

/*
   static struct LCM_setting_table lcm_deep_sleep_out_setting[] = {
//	Display off sequence
{0x11, 1, {0x00}},
{REGFLAG_DELAY, 120, {}},

//	Sleep Mode On
{0x29, 1, {0x00}},
{REGFLAG_DELAY, 20, {}},
{REGFLAG_END_OF_TABLE, 0x00, {}}
};
 */

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

	for(i = 0; i < count; i++)
	{
		unsigned cmd;
		cmd = table[i].cmd;

		switch (cmd) {
			case REGFLAG_DELAY :
				if(table[i].count <= 10)
					MDELAY(table[i].count);
				else
					MDELAY(table[i].count);
				break;

			case REGFLAG_END_OF_TABLE :
				break;

			default:
				dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
		}
	}
}

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));
	params->type   = LCM_TYPE_DSI;

	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;
	params->physical_width = LCM_PHYSICAL_WIDTH/1000;
	params->physical_height = LCM_PHYSICAL_HEIGHT/1000;
	params->physical_width_um = LCM_PHYSICAL_WIDTH;
	params->physical_height_um = LCM_PHYSICAL_HEIGHT;
	params->density = LCM_DENSITY;

#if (LCM_DSI_CMD_MODE)
	params->dsi.mode   = CMD_MODE;
#else
	params->dsi.mode   = SYNC_PULSE_VDO_MODE;  //SYNC_EVENT_VDO_MODE  BURST_VDO_MODE    SYNC_PULSE_VDO_MODE
#endif

	// DSI
	/* Command mode setting */
	params->dsi.LANE_NUM			= 3;	// 3
	params->dsi.PLL_CLOCK			= 280;  //3lane

#if 1
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;
#else
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_LSB_FIRST;
	params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_MSB;
	params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;
#endif

	// Highly depends on LCD driver capability.
	// Not support in MT6573
	params->dsi.packet_size=256;
	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active		= 3;
	params->dsi.vertical_backporch			= 10;
	params->dsi.vertical_frontporch			= 18;

	params->dsi.vertical_active_line	= FRAME_HEIGHT;

	params->dsi.horizontal_sync_active		= 10;
	params->dsi.horizontal_backporch		= 30;
	params->dsi.horizontal_frontporch		= 30;
	params->dsi.horizontal_active_pixel	= FRAME_WIDTH;

	//improve clk quality
	//params->dsi.PLL_CLOCK 		= 390; 	  //3lane

	//params->dsi.PLL_CLOCK			= 257;    //257  4lane
	params->dsi.compatibility_for_nvk 	= 1;
	params->dsi.ssc_disable 		= 1;
	params->dsi.ssc_range	=4;

	params->dsi.esd_check_enable = 1;
	params->dsi.customization_esd_check_enable = 1;
	params->dsi.lcm_esd_check_table[0].cmd = 0x0A;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;
}

static void lcm_init(void)
{
	unsigned char cmd = 0x0;
	unsigned char data = 0xFF;
	int ret = -1;	
	cmd=0x00;
	data=0x0f;

	SET_RESET_PIN(0);
	MDELAY(5);

	set_gpio_lcd_enp(1);

	ret = tps65132_write_bytes(cmd, data);
	if(ret < 0){
		LCD_DEBUG("ST7703 write error, set enp failed!");
	}else{
		LCD_DEBUG("ST7703 write ok, set enp sucess!");
	}

	cmd = 0x01;
	data= 0x0f;
	
	ret = tps65132_write_bytes(cmd, data);
	if(ret < 0){
		LCD_DEBUG("ST7703 write error, set enn failed!");
	}else{
		LCD_DEBUG("ST7703 write ok, set enn sucess!");
	}

	//reset high to low to high
	SET_RESET_PIN(1);
	MDELAY(1);
	SET_RESET_PIN(0);
	MDELAY(1);
	SET_RESET_PIN(1);
	MDELAY(1);

	push_table(lcm_initialization_setting_3lane, sizeof(lcm_initialization_setting_3lane) / sizeof(struct LCM_setting_table), 1);

	LCD_DEBUG("st7703_lcm_init\n");
}

//extern int lcm_vgp_supply_enable(void);
//extern int lcm_vgp_supply_disable(void);
static void lcm_init_power(void)
{
}

static void lcm_suspend_power(void)
{
	//lcm_vgp_supply_disable();
}

static void lcm_resume_power(void)
{
	//lcm_vgp_supply_enable();
}

static void lcm_suspend(void)
{
	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1
);
	set_gpio_lcd_enp(0);
#if 1
	//reset low

	SET_RESET_PIN(0);
	MDELAY(1);
#endif

	LCD_DEBUG("kernel:st7703_lcm_suspend\n");
}

static void lcm_resume(void)
{
	lcm_init();
	LCD_DEBUG("kernel:st7703_lcm_resume\n");
}

LCM_DRIVER st7703_hdplus_dsi_vdo_hlt_lcm_drv =
{
	.name           	= "st7703_hdplus_dsi_vdo_hlt",
	.set_util_funcs 	= lcm_set_util_funcs,
	.get_params     	= lcm_get_params,
	.init           	= lcm_init,
	.suspend        	= lcm_suspend,
	.resume         	= lcm_resume,
	.init_power 	= lcm_init_power,
	.suspend_power 	= lcm_suspend_power,
	.resume_power 	= lcm_resume_power,
	//.compare_id     	= lcm_compare_id,
};

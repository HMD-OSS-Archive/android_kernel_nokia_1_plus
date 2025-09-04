/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 *
 * MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

//#include <cust_eint.h>

#include "st16xx_custom.h"
#ifndef TPD_NO_GPIO
#endif
#include <linux/wakelock.h>

#include <mach/wd_api.h>

#include "mtk_boot_common.h"
#include <linux/regulator/consumer.h>
#include <linux/of_irq.h>
#include <linux/namei.h>
#include <linux/device.h>

static DEFINE_MUTEX(i2c_rw_access);

//#define I2C_SUPPORT_RS_DMA

static unsigned int touch_irq = 0;
#define I2C_RETRY_NUMBER		3
#define ABS(x)				((x<0)?-x:x)
#define TPD_OK				0
#define MAX_BUFFER_SIZE			144
#define CTP_NAME			"ST16XX"
#define IOC_MAGIC			'd'
#define IOCTL_SET 			_IOW(IOC_MAGIC, 1, struct ioctl_cmd168)
#define IOCTL_GET 			_IOR(IOC_MAGIC, 2, struct ioctl_cmd168)
#define HAS_8_BYTES_LIMIT

//#define MAX_BUFFER_SIZE        1028//144
#define MAX_CMD_BUFFER_SIZE        	144
#define MAX_FINGER_NUMBER    		10
#define MAX_PRESSURE        		15
#define DEVICE_NAME         		"ST16XX"
#define DEVICE_VENDOR        		0
#define DEVICE_PRODUCT        		0
#define DEVICE_VERSION        		0
#define ST16XX_X_RESOLUTION    		672
#define ST16XX_Y_RESOLUTION    		672
#define SCREEN_X_RESOLUTION    		1440
#define SCREEN_Y_RESOLUTION    		768
#define VERSION_ABOVE_ANDROID_20
#define MAX_COUNT            		3000
#define COMMAND_SUCCESS        		0x0000
#define COMMAND_ERROR         		0x0200
#define ERROR_QUERY_TIME_OUT    	0x0800

#define QUERY_SUCCESS        		0x00
#define QUERY_BUSY        		0x01
#define QUERY_ERROR        		0x02

#define MAX_FILE_SIZE        		0x10000
#define IOC_MAGIC        		'd'
#define IOCTL_SET         		_IOW(IOC_MAGIC, 1, struct ioctl_cmd168)
#define IOCTL_GET         		_IOR(IOC_MAGIC, 2, struct ioctl_cmd168)
#define IOCTL_AP_GET         		0xF0
#define IOCTL_AP_SET         		0xF1
#define IOCTL_AP_CMD         		0xF2
#define IOCTL_AP_WRITE_FW      		0xF3
#define IOCTL_AP_COMPARE_FLASH 		0xF4
#define IOCTL_AP_SEND_FILE      	0xF5

/*
//add by FHH for I2C DMA mode start
#define MAX_I2C_NDMA_LEN 		8
#define MAX_I2C_DMA_LEN 		240  //should less than 256
static DEFINE_MUTEX(st16xx_i2c_mutex);
static int tpd_i2c_dma_write(struct i2c_client *client, u32 len, u8 const *data);
static int tpd_i2c_dma_read(struct i2c_client *client, u32 len, u8 const *data);
//add by FHH for I2C DMA mode end
*/
extern struct tpd_device *tpd;

static int tpd_flag = 0;
static int tpd_halt=0;
#ifdef I2C_SUPPORT_RS_DMA
static u8 *I2CDMABuf_va = NULL;
static u32 I2CDMABuf_pa = NULL;
#endif
static struct task_struct *thread = NULL;
static DECLARE_WAIT_QUEUE_HEAD(waiter);

bool is_loading_firmware_done = false;

extern void fih_info_set_touch(char *info);

#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
static int tpd_wb_start_local[TPD_WARP_CNT] = TPD_WARP_START;
static int tpd_wb_end_local[TPD_WARP_CNT]   = TPD_WARP_END;
#endif
#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
static int tpd_calmat_local[8]     = TPD_CALIBRATION_MATRIX;
static int tpd_def_calmat_local[8] = TPD_CALIBRATION_MATRIX;
#endif


static int st16xx_get_status(void);
static void st16xx_print_version(void);
static void tpd_eint_interrupt_handler(void);
static int touch_event_handler(void *unused);
static int tpd_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tpd_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
static int tpd_i2c_remove(struct i2c_client *client);
static int tpd_i2c_write(struct i2c_client *client, uint8_t *buf, int len);
static int tpd_i2c_read(struct i2c_client *client, uint8_t *buf, int len , uint8_t addr);
static int fih_tp_create_sysfs(void);


#ifdef ST_UPGRADE_FIRMWARE
int st_upgrade_fw(void *data);
#endif //ST_UPGRADE_FIRMWARE

static struct i2c_client *i2c_client = NULL;

static const struct i2c_device_id tpd_i2c_id[] ={{CTP_NAME,0},{}}; // {{"mtk-tpd",0},{}};

static const struct of_device_id tpd_of_match[] = {
	{.compatible = "mediatek,stironix_touch"},
	{},
};
static struct i2c_driver tpd_i2c_driver = {
	.driver = {
		.name = CTP_NAME,
#ifdef CONFIG_OF
		.of_match_table = tpd_of_match,
#endif
		.owner = THIS_MODULE,
	},
	.probe = tpd_i2c_probe,
	.remove = tpd_i2c_remove,
	.detect = tpd_i2c_detect,
	.driver.name 	= CTP_NAME, //"mtk-tpd",
	.id_table = tpd_i2c_id,
	//   .address_list 	= forces,
};
struct st16xx_data {
	rwlock_t lock;
	unsigned short bufferIndex;
	unsigned short length;
	unsigned short buffer[MAX_BUFFER_SIZE];
};

struct ioctl_cmd168 {
	unsigned short bufferIndex;
	unsigned short length;
	unsigned short buffer[MAX_BUFFER_SIZE];
};

/*
   static int tpd_i2c_dma_read(struct i2c_client *client, u32 len, u8 const *data)
   {
   u8* dmaVa = NULL;
   dma_addr_t dmaPa = 0;
   u32 read = 0 ;
   u32 dmaLen = MAX_I2C_DMA_LEN;
   u32 ext_flag;
   int ret;

   if ((!data) || (len == 0)) {
   pr_err("%s data is null\n", __FUNCTION__);
   return -EINVAL;
   }

   mutex_lock(&st16xx_i2c_mutex);

   if (len <= MAX_I2C_NDMA_LEN) {
   client->ext_flag &= (~I2C_DMA_FLAG);
   ret = i2c_master_recv(client, (char*)data, len);
   if (len != ret) {
   mutex_unlock(&st16xx_i2c_mutex);
   pr_err("%s NDMA failed: ret=%d, length=%d\n", __FUNCTION__, ret, len);
   return -EIO;
   }
   } else {
   client->dev.coherent_dma_mask = DMA_BIT_MASK(32);
   dmaVa = dma_alloc_coherent(&(client->dev), MAX_I2C_DMA_LEN, &dmaPa, GFP_KERNEL);
   ext_flag = client->ext_flag;
   client->ext_flag |= (I2C_ENEXT_FLAG | I2C_DMA_FLAG);

   while (read < len) {
   if (read + MAX_I2C_DMA_LEN > len) {
   dmaLen = len - read;
   }
   ret = i2c_master_recv(client, (u8*)dmaPa, dmaLen);
   if (dmaLen != ret) {
   pr_err("%s DMA failed: ret=%d\n", __FUNCTION__, ret);
   }
   memcpy((void*)&data[read], dmaVa, dmaLen);
   read += dmaLen;
   }

   dma_free_coherent(&(client->dev), MAX_I2C_DMA_LEN, dmaVa, dmaPa);
   client->ext_flag = ext_flag;
   }

   mutex_unlock(&st16xx_i2c_mutex);

   return 0;
   }*/
//add by FHH for I2C DMA mode end

/*
   int st16xx_open(struct inode *inode, struct file *filp) {
   int i;
   struct st16xx_data *dev;

   pr_info("=st16xx_open=\n");
   dev = kmalloc(sizeof(struct st16xx_data), GFP_KERNEL);
   if (dev == NULL) {
   return -ENOMEM;
   }

   rwlock_init(&dev->lock);
   for (i = 0; i < MAX_BUFFER_SIZE; i++) {
   dev->buffer[i] = 0xFF;
   }

   filp->private_data = dev;

   return 0;
   }

   int st16xx_close(struct inode *inode, struct file *filp)
   {
   struct st16xx_data *dev = filp->private_data;

   pr_info("=st16xx_close=\n");
   if (dev) {
   kfree(dev);
   }

   return 0;
   }

   struct file_operations st16xx_fops = {
   .owner		= THIS_MODULE,
   .open		= st16xx_open,
   .release 	= st16xx_close,
//.ioctl		= st16xx_ioctl,
//.unlocked_ioctl = st16xx_ioctl,
};

static struct miscdevice ctp_dev = {
.minor	= MISC_DYNAMIC_MINOR,
.name	= CTP_NAME,
//.fops	= &st16xx_fops,
};
 */

static int st_irq_off(void)
{
	disable_irq_nosync(touch_irq);
	tpd_halt = 1;
	return 0;
}

static int st_irq_on(void)
{
	enable_irq(touch_irq);
	tpd_halt = 0;
	return 0;
}


static int tpd_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)\
{
	//  strcpy(info->type, "mtk-tpd");
	strcpy(info->type, CTP_NAME);
	return 0;
}

static int tpd_i2c_write(struct i2c_client *client, uint8_t *writebuf, int writelen)
{
	int ret = 0;
	int i = 0;

	if (client == NULL) {
		ST_INFO("[IIC][%s]i2c_client==NULL!", __func__);
		return -EINVAL;
	}

	mutex_lock(&i2c_rw_access);
	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
				.addr = client->addr,
				.flags = 0,
				.len = writelen,
				.buf = writebuf,
			},
		};
		for (i = 0; i < I2C_RETRY_NUMBER; i++) {
			ret = i2c_transfer(client->adapter, msgs, 1);
			if (ret < 0) {
				ST_ERR("[IIC]: i2c_transfer(write) error, ret=%d!!", ret);
			} else
				break;
		}
	}
	mutex_unlock(&i2c_rw_access);

	return ret;
}

static int tpd_i2c_read(struct i2c_client *client, uint8_t *buf, int len , uint8_t addr)
{
	int ret = 0;
	int i = 0;

	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &addr,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = buf,
		},
	};

	mutex_lock(&i2c_rw_access);

	for (i = 0; i < I2C_RETRY_NUMBER; i++) {
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret < 0) {
			ST_ERR("[IIC]: i2c_transfer(write) error, ret=%d!!", ret);
		} else
			break;
	}

	mutex_unlock(&i2c_rw_access);
	return ret;
}

static int tpd_irq_registration(void)
{
	struct device_node *node = NULL;
	int ret = 0;
	u32 ints[2] = { 0, 0 };

	ST_INFO("Device Tree Tpd_irq_registration!\n");

	node = of_find_matching_node(node, touch_of_match);
	if (node) {
		if(of_property_read_u32_array(node, "debounce", ints, ARRAY_SIZE(ints)) == 0){
			gpio_set_debounce(ints[0], ints[1]);
		}else{
			ST_ERR("debounce time not found");
		}

		touch_irq = irq_of_parse_and_map(node, 0);

		ret = request_irq(touch_irq, (irq_handler_t) tpd_eint_interrupt_handler, IRQF_TRIGGER_FALLING,
				"TOUCH_PANEL-eint", NULL); //IRQF_TRIGGER_FALLING  IRQF_TRIGGER_RISING
		if (ret > 0) {
			ret = -1;
			ST_ERR("tpd request_irq IRQ LINE NOT AVAILABLE!.\n");
		}

	} else {
		ST_ERR("tpd request_irq can not find touch eint device node!.\n");
		ret = -1;
	}

	ST_INFO("irq:%d\n", touch_irq);
	return ret;
}

static int st16xx_get_status(void)
{
	char buffer[8];
	int ret = -1;
	ret = tpd_i2c_read(i2c_client, buffer, 8, 0x1);
	if(ret < 0)
	{
		ST_INFO("i2c communcate error in getting status : 0x%x\n", ret);
		return -1;
	}
	ST_INFO("ST16XX Touch Panel status %x \n", buffer[0]);
	return buffer[0];
}

static void st16xx_print_version(void)
{
	char buffer;
	int ic_type = 0;
	char buf[32]={0};
	int vendor_id = 0;
	int ret = -1;

	ret = tpd_i2c_read(i2c_client, &buffer, 0x1, 0x0);
	if(ret < 0)
	{
		ST_ERR("i2c communcate error in getting FW version : 0x%x\n", ret);
		return ;
	}
	ret = snprintf(buf, 31, "04_%02x_%02x_%02x\n", ic_type, buffer, vendor_id);

	fih_info_set_touch(buf);

	ST_INFO("ST16XX Touch Panel Firmware version %x\n", buffer);
}

static int tpd_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;//, ret = -1
	int status = 0;
	// unsigned char Wrbuf[2] = { 0x20, 0x07 };
	// unsigned char Rdbuffer[10];
	//	char buffer1[14];

	i2c_client = client;

	ST_INFO("Sitronix st16xx touch panel i2c probe:%s\n", id->name);
	ST_INFO("Sitronix st16xx touch panel i2c addr:0x%x\n", client->addr);

	tpd_gpio_output(GTP_RST_PORT, 1); //ugrec_tky gpio_direction_output(tpd_rst_gpio_number, 0);
	msleep(10);
	tpd_gpio_output(GTP_RST_PORT, 0); //ugrec_tky gpio_direction_output(tpd_rst_gpio_number, 1);
	msleep(10);
	tpd_gpio_output(GTP_RST_PORT, 1); //ugrec_tky gpio_direction_output(tpd_rst_gpio_number, 0);
	msleep(300);

#ifdef I2C_SUPPORT_RS_DMA
	I2CDMABuf_va = (u8 *)dma_alloc_coherent(NULL, 4096, &I2CDMABuf_pa, GFP_KERNEL);
	if(!I2CDMABuf_va)
	{
		ST_ERR("st16xx Allocate Touch DMA I2C Buffer failed!\n");
		return -1;
	}
#endif
	//ret = sysfs_create_group(&(client->dev.kobj), &st16xx_attr_group);
	//err = sysfs_create_group(&client->dev.kobj, &ite_attr_group);

	tpd_irq_registration();
	msleep(100);

	status =  st16xx_get_status();
	if(status != 6)
	{
		tpd_halt = 1;
		// upgrade panel here
		tpd_halt = 0;
	}

#ifdef ST_UPGRADE_FIRMWARE
	thread = kthread_run(st_upgrade_fw, (void *)NULL, "sitronix_update");
	if (IS_ERR(thread)) {
		err = PTR_ERR(thread);
		ST_ERR("st16xx failed to create firmware update thread: %d\n", err);
	}
#endif //ST_UPGRADE_FIRMWARE

	thread = kthread_run(touch_event_handler, (void *)NULL, CTP_NAME);
	if (IS_ERR(thread)) {
		err = PTR_ERR(thread);
		ST_ERR("st16xx failed to create touch event handle thread: %d\n", err);
	}

	fih_tp_create_sysfs();

	st16xx_print_version(); //read firmware version

	tpd_load_status = 1;
	return 0;
}

void tpd_eint_interrupt_handler(void)
{
	tpd_flag = 1;
	//ST_INFO("tp_int\n");
	wake_up_interruptible(&waiter);
}

static int tpd_i2c_remove(struct i2c_client *client)
{
#ifdef I2C_SUPPORT_RS_DMA
	if( I2CDMABuf_va ){
		dma_free_coherent(NULL, 4096, I2CDMABuf_va, I2CDMABuf_pa);
		I2CDMABuf_va = NULL;
		I2CDMABuf_pa = 0;
	}
#endif
	return 0;
}

void tpd_down(int raw_x, int raw_y, int x, int y, int p)
{
	input_report_abs(tpd->dev,  ABS_MT_TRACKING_ID, p+1);
	input_report_abs(tpd->dev, ABS_MT_PRESSURE, 128);
	input_report_key(tpd->dev, BTN_TOUCH, 1);
	input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 128);
	input_report_abs(tpd->dev, ABS_MT_WIDTH_MAJOR, 128);
	input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
	input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);

	input_mt_sync(tpd->dev);
	//ST_INFO("D[%4d %4d %4d]\n", x, y, p); //by FHH
	//TPD_EM_PRINT(raw_x, raw_y, x, y, p, 1);
}

void tpd_up(int raw_x, int raw_y, int x, int y, int p)
{
	input_report_abs(tpd->dev, ABS_MT_PRESSURE, 0);
	input_report_key(tpd->dev, BTN_TOUCH, 0);
	input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 0);
	input_report_abs(tpd->dev, ABS_MT_WIDTH_MAJOR, 0);
	input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
	input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
	input_mt_sync(tpd->dev);
	//ST_INFO("U[%4d %4d %4d]\n", x, y, 0); //by FHH
	//TPD_EM_PRINT(raw_x, raw_y, x, y, p, 0);
}

static int touch_event_handler( void *unused )
{
	struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };
	unsigned char buf[ST16XX_MAX_TOUCHES*4];
	int ret = 0,i=0;
	int x,y;
	u8 touchCount=0;

	sched_setscheduler(current, SCHED_RR, &param);
	do
	{
		set_current_state(TASK_INTERRUPTIBLE);
		
		while(tpd_halt){
			tpd_flag = 0; 
			msleep(20);
		}
		
		wait_event_interruptible(waiter, tpd_flag != 0);
		
		tpd_flag = 0;
		TPD_DEBUG_SET_TIME;
		set_current_state(TASK_RUNNING);
		touchCount=0;

		ret = tpd_i2c_read(i2c_client, buf, 8, 0x12);
		if(ST16XX_MAX_TOUCHES>2)
			ret = tpd_i2c_read(i2c_client, buf+8, 8, 0x1A);
		if(ST16XX_MAX_TOUCHES>4)
			ret = tpd_i2c_read(i2c_client, buf+16, 4, 0x22);


		for(i=0;i<ST16XX_MAX_TOUCHES;i++)
		{
			if(buf[4*i] & 0x80)
			{
				x = (int)(buf[i*4] & 0x70) << 4 | buf[i * 4 + 1];
#ifdef TPD_SWAP_Y
				y = (TPD_RES_Y) - ((int)(buf[i*4] & 0x07) << 8 | buf[i * 4 + 2]);
#else
				y = (int)(buf[i*4] & 0x07) << 8 | buf[i * 4 + 2];
#endif
				touchCount++;
				ST_DEBUG("ST16XX touch point: %d (%d,%d)\n",i,x,y);
				tpd_down(0,0, x, y, i);
			}
		}

		if(touchCount == 0)
		{
			tpd_up(0,0, 0,0,0);
		}

		input_sync(tpd->dev);

	} while (!kthread_should_stop());

	return 0;
}

int tpd_local_init(void)
{

#ifdef STTP_USE_POWER //def CONFIG_ARCH_MT6580
	int ret;
	tpd->reg=regulator_get(tpd->tpd_dev, "vtouch"); // get pointer to regulator structure
	if (IS_ERR(tpd->reg)) {
		ST_ERR("regulator_get() failed!\n");
		return -1;
	}

	ret=regulator_set_voltage(tpd->reg, 2800000, 2800000);	// set 2.8v
	if (ret){
		ST_ERR("regulator_set_voltage() failed!\n");
		return -1;
	}
	ret=regulator_enable(tpd->reg);  //enable regulator
	if (ret){
		ST_ERR("regulator_enable() failed!\n");
		return -1;
	}
#endif
	if(i2c_add_driver(&tpd_i2c_driver)!=0) {
		ST_INFO("unable to add i2c driver.\n");
		return -1;
	}

	if (tpd_load_status == 0) {
		/* if(tpd_load_status == 0) // disable auto load touch driver for linux3.0 porting */
		ST_ERR("add error touch panel driver.");
		i2c_del_driver(&tpd_i2c_driver);
		return -1;
	}

#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
	TPD_DO_WARP = 1;
	memcpy(tpd_wb_start, tpd_wb_start_local, TPD_WARP_CNT*4);
	memcpy(tpd_wb_end, tpd_wb_start_local, TPD_WARP_CNT*4);
#endif

#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
	memcpy(tpd_calmat, tpd_def_calmat_local, 8*4);
	memcpy(tpd_def_calmat, tpd_def_calmat_local, 8*4);
#endif
	ST_INFO("Init end\n");

	tpd_type_cap = 1;

	return 0;
}

/* Function to manage low power suspend */
static void tpd_suspend(struct device *h)
{

	int ret;
	if(is_loading_firmware_done != true){
		ST_ERR("firmware is updateing, not suspend!\n");
		return;
	}
#if 0
	unsigned char buf[2];
	int status;
	buf[0] = 0x02;
	buf[1] = 0x02;
	ret = tpd_i2c_write(i2c_client, buf, 2);
	msleep(100);
	status  = st16xx_get_status();
	if(status == 5)
		ST_INFO("ST16xx go power down mode success\n");
	else
		ST_INFO("ST16xx go power down mode fail\n");
#endif
	st_irq_off();

	tpd_gpio_output(GTP_RST_PORT, 0);

#ifdef STTP_USE_POWER
	ret = regulator_disable(tpd->reg);  //enable regulator
	if( ret )
		ST_ERR("regulator_disable() failed!\n");
#endif
	ST_INFO("ST16xx suspend done\n");
}

/* Function to manage power-on resume */
static void tpd_resume(struct device *h)
{
	int ret;

	if(is_loading_firmware_done != true){
		ST_ERR("firmware is updateing, not suspend!\n");
		return;
	}

#ifdef STTP_USE_POWER
	ret = regulator_enable(tpd->reg);  //enable regulator
	if( ret ){
		ST_ERR("regulator_enable() failed!\n");

	}
#endif
	tpd_gpio_output(GTP_RST_PORT, 1); //ugrec_tky gpio_direction_output(tpd_rst_gpio_number, 0);
	msleep(10);
	tpd_gpio_output(GTP_RST_PORT, 0); //ugrec_tky gpio_direction_output(tpd_rst_gpio_number, 1);
	msleep(10);
	tpd_gpio_output(GTP_RST_PORT, 1); //ugrec_tky gpio_direction_output(tpd_rst_gpio_number, 1);
	msleep(300);

	ret = st_irq_on();
	ST_INFO("ST16XX resume done\n");
}

static struct tpd_driver_t tpd_device_driver = {
	.tpd_device_name =CTP_NAME,  // TPD_DEVICE,
	.tpd_local_init = tpd_local_init,
	.suspend = tpd_suspend,
	.resume = tpd_resume,
#ifdef TPD_HAVE_BUTTON
	.tpd_have_button = 1,
#else
	.tpd_have_button = 0,
#endif
};


/* called when loaded into kernel */
static int __init tpd_driver_init(void)
{
	tpd_get_dts_info();
	ST_INFO("Sitronix st16xx touch panel driver init\n");
	
	if(tpd_driver_add(&tpd_device_driver) < 0)
		ST_ERR("add  generic driver failed\n");

	return 0;
}

/* should never be called */
static void __exit tpd_driver_exit(void)
{
	ST_INFO("MediaTek ST16XX touch panel driver exit\n");
	//input_unregister_device(tpd->dev);
	tpd_driver_remove(&tpd_device_driver);
}


module_init(tpd_driver_init);
module_exit(tpd_driver_exit);



#if defined(ST_TEST_RAW) || defined(ST_UPGRADE_FIRMWARE)

static int st_i2c_read_direct(st_u8 *rxbuf, int len)
{

	int ret = 0;

	ret = i2c_master_recv(i2c_client, rxbuf, len);

	if (ret < 0){
		ST_ERR("read direct error (%d)\n", ret);
		return ret;
	}
	return len;
}

static int st_i2c_read_bytes(st_u8 addr, st_u8 *rxbuf, int len)
{
	int ret = 0;

	ret = tpd_i2c_read(i2c_client, rxbuf, len, addr);
	if (ret < 0){
		ST_ERR("write 0x%x error (%d)\n", addr, ret);
		return ret;
	}
	return len;
}

static int st_i2c_write_bytes(st_u8 *txbuf, int len)
{

	int ret = 0;
	if(txbuf == NULL)
		return -1;
	ret = tpd_i2c_write(i2c_client, txbuf, len);

	if (ret < 0){
		ST_ERR("write 0x%x error (%d)\n", *txbuf, ret);
		return ret;
	}
	return len;
}
#endif

#ifdef ST_UPGRADE_FIRMWARE

static int st_check_chipid(void)
{
	int ret = 0;
	unsigned char buffer;

	ret = st_i2c_read_bytes(0xF4, &buffer, 1);
	if (ret < 0){
		ST_ERR("read status reg error (%d)\n", ret);
		return ret;
	}else{
		ST_INFO("ChipID = 0x%x\n", buffer);
	}

#ifdef ST_IC_A1802
	if(buffer != 0xC)
	{
		ST_ERR("This IC is not A1802 , cancel upgrade\n");
		return -1;
	}
#endif
	return 0;
}

static int st_get_device_status(void)
{
	int ret = 0;
	st_u8 buffer[8];

	ret = st_i2c_read_bytes(1, buffer, 8);
	if (ret < 0){
		ST_ERR("read status reg error (%d)\n", ret);
		return ret;
	}else{
		ST_INFO("status reg = %d\n", buffer[0]);
	}


	return buffer[0]&0xF;
}

static int st_check_device_status(int ck1,int ck2,int delay)
{
	int maxTimes = 3;
	int isInStauts = 0;
	int status = -1;
	while(maxTimes-->0 && isInStauts==0)
	{
		status = st_get_device_status();
		ST_INFO("status : %d\n",status);
		if(status == ck1 || status == ck2)
			isInStauts=1;
		msleep(delay);
	}
	if(isInStauts==0)
		return -1;
	else
		return 0;
}

static int st_power_up(void)
{
	unsigned char reset[2];
	reset[0] = 2;
	reset[1] = 0;
	return st_i2c_write_bytes(reset,2);
}
static int st_isp_on(void)
{
	unsigned char IspKey[] = {0,'S',0,'T',0,'X',0,'_',0,'F',0,'W',0,'U',0,'P'};
	unsigned char i;
	int icStatus = st_get_device_status();

	ST_INFO("ISP on\n");

	if(icStatus <0)
		return -1;
	if(icStatus == 0x6)
		return 0;
	else if(icStatus == 0x5)
		st_power_up();

	for(i=0;i<sizeof(IspKey); i+=2)
	{
		if(st_i2c_write_bytes(&IspKey[i],2) < 0)
		{
			ST_ERR("Entering ISP fail.\n");
			return -1;
		}
	}
	msleep(150);	//This delay is very important for ISP mode changing.
	//Do not remove this delay arbitrarily.
	//return st_check_device_status(6,99,10);
	return st_check_device_status(6,99,1);
}

static int st_compare_array(unsigned char *b1,unsigned char *b2,int len)
{
	int i=0;
	for(i=0;i<len;i++)
	{
		if(b1[i] != b2[i])
			return -1;
	}
	return 0;
}

#ifdef ST_FIREWARE_FILE
static int st_load_cfg_from_file(unsigned st_char *buf)
{
	int j;
	struct file  *cfg_fp;
	mm_segment_t fs;
	int fileSize = 0;
	loff_t pos = 0;

	cfg_fp = filp_open(ST_CFG_PATH, O_RDONLY, 0);
	if(IS_ERR(cfg_fp)){
		ST_ERR("Test: filp_open error!!. %d\n",j);
	}else
	{
		fs = get_fs();
		set_fs(get_ds());

		fileSize = vfs_read(cfg_fp,buf, ST_CFG_LEN, &pos);
		ST_INFO("Read cfg file size=0x%x\n", fileSize);
		set_fs(fs);
		filp_close(cfg_fp, NULL);
	}
	return fileSize;
}

static int st_check_fs_mounted(st_char *path_name)
{
	struct path root_path;
	struct path path;
	s32 err;

	err = kern_path("/", LOOKUP_FOLLOW, &root_path);
	if (err)
		return ERROR_PATH;

	err = kern_path(path_name, LOOKUP_FOLLOW, &path);
	if (err) {
		err = ERROR_PATH;
		goto check_fs_fail;
	}

	if (path.mnt->mnt_sb == root_path.mnt->mnt_sb) {
		// not mounted
		err = ERROR_PATH;
	} else {
		err = 0;
	}

	path_put(&path);
check_fs_fail:
	path_put(&root_path);
	return err;

}

static int st_load_fw_from_file(unsigned st_char *buf)
{
	int j;
	struct st_file  *fw_fp;
	mm_segment_t fs;
	loff_t pos = 0;
	int fileSize = 0;


	for(j=0; j<30; j++)
	{
		st_msleep(1000);
		ST_INFO("Wati for %s FS to be mounted-cycle-%d\n", ST_FW_DIR, j);

		if (st_check_fs_mounted(ST_FW_DIR) == 0)
		{
			ST_ERR("%s mounted ~!!!!\n", ST_FW_DIR);
			fileSize = 1;
			break;
		}
	}

	if(fileSize ==0)
	{
		ST_ERR("%s don't mounted ~!!!!\n", ST_FW_DIR);
	   	return -1;
	}

	for(j=0; j<10; j++)
	{
		st_msleep(1000);
		fileSize = 0;
		fw_fp = filp_open(ST_FW_PATH, O_RDONLY, 0);
		if(IS_ERR(fw_fp)){
			ST_ERR("Test: filp_open error!!. %d\n",j);
			fileSize = 0;
		}else
		{
			fileSize = 0;
			fs = get_fs();
			set_fs(get_ds());
			fileSize = vfs_read(fw_fp, buf, ST_FW_LEN, &pos);
			set_fs(fs);
			ST_ERR("Read fw file size:0x%X\n", fileSize);
			filp_close(fw_fp, NULL);
			break;
		}
	}
	return fileSize;
}

unsigned char fw_buf[ST_FW_LEN];
unsigned char cfg_buf[ST_CFG_LEN];
#else
unsigned char fw_buf[] = SITRONIX_FW;
unsigned char cfg_buf[] = SITRONIX_CFG;
#endif //ST_FIREWARE_FILE


#ifdef ST_IC_A1802

static int st_get_fw_info_offset(int fwSize,unsigned char *data)
{
	int cksOffset;
	int i=0;
	cksOffset = data[0x84] * 0x100 + data[0x85];


	for(i=cksOffset-4;i>=4;i--)
	{
		if(	data[i]   == 0x54 &&
				data[i+1] == 0x46 &&
				data[i+2] == 0x49 &&
				data[i+3] == 0x33 )
		{
			ST_INFO("TOUCH_FW_INFO offset = 0x%X\n",i+4);
			return i+4;
		}
	}

	ST_INFO("can't find TOUCH_FW_INFO offset\n");
	return -1;
}

void ChecksumCalculation(unsigned short *pChecksum,unsigned char *pInData,unsigned long Len)
{
	unsigned long i;
	unsigned char LowByteChecksum;
	for(i = 0; i < Len; i++)
	{
		*pChecksum += (unsigned short)pInData[i];
		LowByteChecksum = (unsigned char)(*pChecksum & 0xFF);
		LowByteChecksum = (LowByteChecksum) >> 7 | (LowByteChecksum) << 1;
		*pChecksum = (*pChecksum & 0xFF00) | LowByteChecksum;
	}
}


static int st_v2_format(st_u16 addr,st_u8 isRead,unsigned char *txbuf,unsigned char *rxbuf, int len)
{
	int result;

	unsigned char isp_tbuf[ST_ISP_MAX_WRITE_LEN+2] = {0};
	//unsigned st_u8 isp_rbuf[ST_ISP_MAX_TRANS_LEN+2] = {0};

	isp_tbuf[0] = (addr>>8);
	isp_tbuf[1] = (addr&0xFF);
	if(isRead)
	{

		result = st_i2c_write_bytes(isp_tbuf, 2);

		//ST_INFO("Read off = %x %x ,len = %d, result = %d \n",isp_tbuf[0],isp_tbuf[1] ,len, result);
		msleep(1);
		result = st_i2c_read_direct(rxbuf,len);
		//ST_INFO("Read data = %x %x %x %x, result = %d \n",rxbuf[0],rxbuf[1] , rxbuf[2] , rxbuf[3] , result);

		if( result < 0)
			return result;
		else
			return len;
	}
	else
	{
		memcpy(isp_tbuf+2,txbuf,len);
		result = st_i2c_write_bytes(isp_tbuf,len+2);
		//ST_INFO("Write off = %x %x ,len = %d, data = %x %x %x %x %x %x %x %x, result = %d \n",isp_tbuf[0],isp_tbuf[1] ,len, isp_tbuf[2],isp_tbuf[3],isp_tbuf[4],isp_tbuf[5],isp_tbuf[6],isp_tbuf[7],isp_tbuf[8],isp_tbuf[9],result);
		if( result < 0)
			return result;
		else
			return len;
	}
	return 0;
}




static int st_v2_len_check(st_u16 addr,st_u8 isRead,unsigned char *txbuf,unsigned char *rxbuf, int len)
{
	int nowlen = len;
	int nowoff = 0;
	int ret = 0;


	if(isRead)
	{
		while(nowlen > 0 )
		{
			if(nowlen > ST_ISP_MAX_TRANS_LEN)
			{

				ret += st_v2_format(addr+nowoff,isRead,txbuf+nowoff, rxbuf+nowoff, ST_ISP_MAX_TRANS_LEN);

				nowoff += ST_ISP_MAX_TRANS_LEN;
				nowlen -= ST_ISP_MAX_TRANS_LEN;
			}
			else
			{
				ret += st_v2_format(addr+nowoff,isRead,txbuf+nowoff, rxbuf+nowoff, nowlen);
				nowlen = 0;
			}
		}
	}
	else
	{
		while(nowlen > 0 )
		{
			if(nowlen > ST_ISP_MAX_WRITE_LEN)
			{

				ret += st_v2_format(addr+nowoff,isRead,txbuf+nowoff, rxbuf+nowoff, ST_ISP_MAX_WRITE_LEN);

				nowoff += ST_ISP_MAX_WRITE_LEN;
				nowlen -= ST_ISP_MAX_WRITE_LEN;
			}
			else
			{
				ret += st_v2_format(addr+nowoff,isRead,txbuf+nowoff, rxbuf+nowoff, nowlen);
				nowlen = 0;
			}
		}
	}

	return ret;
}


static int st_v2_read_bytes(st_u16 addr,unsigned char *rxbuf, int len)
{
	unsigned char tmp_buf[len];
	return st_v2_len_check(addr,1,tmp_buf, rxbuf, len);
}

static int st_v2_write_bytes(st_u16 addr,unsigned char *txbuf, int len)
{
	unsigned char tmp_buf[len];
	return st_v2_len_check(addr,0,txbuf, tmp_buf, len);
}

int st_v2_set_2B_mode(void)
{
	unsigned char data[2];
	int rt = 0;
	data[0] = 0xF1;
	data[1] = 0x20;
	rt += st_i2c_write_bytes(data,2);
	if(rt < 0)
	{
		ST_ERR("Set 2 byte mode error\n");
		return -1;
	}
	st_msleep(1);

	return 0;
}

int st_v2_isp_off(void)
{
	unsigned char data[1];
	int rt = 0;
	data[0] = 1;
	rt = st_v2_write_bytes(0x2,data,1);
	//rt += st_i2c_write_bytes(data,2);
	if(rt < 0)
	{
		ST_ERR("ISP off error\n");
		return -1;
	}
	//st_msleep(300);
	st_msleep(100);


	return st_check_device_status(0,4,10);
}


static int st_v2_cmd_write(unsigned char *buf,int len)
{

	unsigned short pCheckSum=0;
	unsigned char pStatus[8]={0};
	int retryCount=0;
	int isSuccess=0;


	ChecksumCalculation(&pCheckSum,buf,len);
	buf[len] = pCheckSum&0xFF;	//CheckSum
	//buf[len+1] = 0;

	if(st_v2_write_bytes(0xD0,buf,len+1) != len+1)
	{
		ST_ERR("ST_SPIW_1 Send 0xD0 fail.\n");
		return -1;
	}

	//st_msleep(10);
	pStatus[0] = 1;
	if(st_v2_write_bytes(0xF8,pStatus,1) != 1)
	{
		ST_ERR("ST_SPIW_2 Send 0xF8 fail.\n");
		return -1;
	}


	//st_msleep(100);
	st_msleep(5);

	retryCount=0;
	isSuccess=0;

	while(isSuccess==0 && retryCount++ < ST_ISP_RETRY_MAX)
	{
		if(st_v2_read_bytes(0xF8,pStatus,1) != 1)
		{
			isSuccess=0;
		}
		else
			isSuccess = 1;


		if(isSuccess == 1)
		{
			if(pStatus[0] != 0)
			{
				ST_ERR("ST_SPIW_4 Read 0xF8  fail. retry %d\n",retryCount);
				ST_ERR("ST_SPIW_4 status of 0xF8 error ,error:%x.\n",pStatus[0]);
				isSuccess = 0;
			}
			else
				isSuccess = 1;
		}
	//	st_msleep(10);
	}

	return isSuccess-1;

	return 0;
}



static int st_v2_flash_read_page(unsigned char *Buf,unsigned short PageNumber)
{

	unsigned char PacketData[8];
	int i;
	st_u32 offset;

	for(i=0;i<ST_FLASH_PAGE_SIZE/ST_ISP_BLOCK_SIZE;i++)
	{
		memset(PacketData,0,8);
		PacketData[0] = 0x15;
		PacketData[1] = 6;

		offset = PageNumber*ST_FLASH_PAGE_SIZE+i*ST_ISP_BLOCK_SIZE;
		PacketData[2] = (offset>>16)&0xFF;
		PacketData[3] = (offset>>8)&0xFF;
		PacketData[4] = (offset)&0xFF;
		PacketData[5] = 0x1;
		PacketData[6] = 0x0;

		if(st_v2_cmd_write(PacketData,7) <0)
			return -1;

		if(st_v2_read_bytes(0x200,Buf+(i*ST_ISP_BLOCK_SIZE),ST_ISP_BLOCK_SIZE) != ST_ISP_BLOCK_SIZE)
		{
			ST_ERR("ST_FR_2 Read Flash Data fail.\n");
			return -1;
		}
	}

	//ST_INFO("flash %x %x %x %x\n",Buf[0x100],Buf[0x101],Buf[0x102],Buf[0x103]);



	return 0;
}

static int st_v2_flash_unlock(void)
{
	unsigned char PacketData[12];

	int retryCount=0;
	int isSuccess=0;

	PacketData[0] = 0x10;
	PacketData[1] = 9;
	PacketData[2] = 0x55;
	PacketData[3] = 0xAA;
	PacketData[4] = 0x55;
	PacketData[5] = 0x6E;
	PacketData[6] = 0x4C;
	PacketData[7] = 0x7F;
	PacketData[8] = 0x83;
	PacketData[9] = 0x9B;



	while(isSuccess==0 && retryCount++ < ST_ISP_RETRY_MAX)
	{
		if(st_v2_cmd_write(PacketData,10) ==0)
			isSuccess = 1;

		if(isSuccess ==0)
		{
			ST_INFO("Read ISP_Unlock_Ready packet fail retry : %d\n",retryCount);
			//MSLEEP(30);
		}
	}


	if(isSuccess == 0)
	{
		ST_INFO("st_flash_unlock fail.\n");
		return -1;
	}

	return 0;
}

int st_v2_flash_erase_page(unsigned short PageNumber)
{
	unsigned char PacketData[8];
	st_u32 offset;
	int retryCount=0;
	int isSuccess=0;

	PacketData[0] = 0x13;
	PacketData[1] = 4;
	offset = PageNumber*ST_FLASH_PAGE_SIZE;
	PacketData[2] = (offset>>16)&0xFF;
	PacketData[3] = (offset>>8)&0xFF;
	PacketData[4] = (offset)&0xFF;

	while(isSuccess==0 && retryCount++ < ST_ISP_RETRY_MAX)
	{
		if(st_v2_cmd_write(PacketData,5) ==0)
			isSuccess = 1;

		if(isSuccess ==0)
		{
			ST_INFO("Read ISP_Erase_Ready packet fail with page %d retry : %d\n",PageNumber,retryCount);
			//MSLEEP(30);
		}
	}


	if(isSuccess == 0)
	{
		ST_ERR("st_flash_erase_page fail.\n");
		return -1;
	}

	return 0;
}


static int st_v2_flash_write_page(unsigned char *Buf,unsigned short PageNumber)
{
	unsigned char PacketData[10];
	unsigned char RetryCount;
	unsigned short DataCheckSum=0;
	int i;
	st_u32 offset;

	//ST_INFO("Write page start\n");

	for(i=0;i<ST_FLASH_PAGE_SIZE/ST_ISP_BLOCK_SIZE;i++)
	{
	//	ST_INFO("Write page start flash unlock-cycle=%d\n", i);
		if(st_v2_flash_unlock()<0)
			return -1;

		RetryCount = 0;
		PacketData[0] = 0x14;
		PacketData[1] = 7;

		offset = PageNumber*ST_FLASH_PAGE_SIZE+i*ST_ISP_BLOCK_SIZE;
		PacketData[2] = (offset>>16)&0xFF;
		PacketData[3] = (offset>>8)&0xFF;
		PacketData[4] = (offset)&0xFF;
		PacketData[5] = 0x1;
		PacketData[6] = 0x0;

	//	ST_INFO("Write page start check sum-cycle=%d\n", i);

		DataCheckSum=0;
		ChecksumCalculation(&DataCheckSum,Buf+(i*ST_ISP_BLOCK_SIZE),ST_ISP_BLOCK_SIZE);

		PacketData[7] = DataCheckSum;

	//	ST_INFO("Write page start write bytes start-cycle=%d\n", i);

		if(st_v2_write_bytes(0x200,Buf+(i*ST_ISP_BLOCK_SIZE),ST_ISP_BLOCK_SIZE) != ST_ISP_BLOCK_SIZE)
		{
			ST_ERR("ST_FW_1 Write Flash Data fail.\n");
			return -1;
		}
	//	ST_INFO("Write page start write bytes done-cycle=%d\n", i);

		//st_msleep(10);

		if(st_v2_cmd_write(PacketData,8) <0)
			return -1;
	}

	//ST_INFO("Write page end\n");

	return 0;
}


int st_v2_flash_write(unsigned char *Buf, int Offset, int NumByte)
{
	unsigned short StartPage;
	unsigned short PageOffset;
	int WriteNumByte;
	short WriteLength;
	//unsigned char TempBuf[ST_FLASH_PAGE_SIZE];
	unsigned char *TempBuf;
	int retry = 0;
	int isSuccess = 0;

	TempBuf = kzalloc(ST_FLASH_PAGE_SIZE, GFP_KERNEL);

	ST_INFO("Write flash offset:0x%X , length:0x%X\n",Offset,NumByte);

	WriteNumByte = 0;
	if(NumByte == 0)
		return WriteNumByte;

	if((Offset + NumByte) > ST_FLASH_SIZE)
		NumByte = ST_FLASH_SIZE - Offset;

	StartPage = Offset / ST_FLASH_PAGE_SIZE;
	PageOffset = Offset % ST_FLASH_PAGE_SIZE;
	while(NumByte > 0)
	{
		if((PageOffset != 0) || (NumByte < ST_FLASH_PAGE_SIZE))
		{
			if(st_v2_flash_read_page(TempBuf,StartPage) < 0)
				return -1;
		}

		WriteLength = ST_FLASH_PAGE_SIZE - PageOffset;
		if(NumByte < WriteLength)
			WriteLength = NumByte;
		memcpy(&TempBuf[PageOffset],Buf,WriteLength);

		retry = 0;
		isSuccess = 0;
		while(retry++ <2 && isSuccess ==0)
		{
			if(st_v2_flash_unlock() >= 0 &&  st_v2_flash_erase_page(StartPage) >= 0)
			{
				ST_INFO("write page:%d start\n",StartPage);
				if(st_v2_flash_write_page(TempBuf,StartPage) >= 0)
					isSuccess =1;
				ST_INFO("write page:%d done\n",StartPage);
			}
			isSuccess =1;

			if(isSuccess==0)
				ST_INFO("FIOCTL_IspPageWrite write page %d retry: %d\n",StartPage,retry);
		}
		if(isSuccess==0)
		{
			ST_ERR("FIOCTL_IspPageWrite write page %d error\n",StartPage);
			return -1;
		}
		else
			StartPage++;

		NumByte -= WriteLength;
		Buf += WriteLength;
		WriteNumByte += WriteLength;
		PageOffset = 0;
	}
	kfree(TempBuf);
	return WriteNumByte;
}

#endif

unsigned char fw_check[ST_FLASH_PAGE_SIZE];
#ifdef ST_UPGRADE_BY_ID
unsigned char id_buf[] = SITRONIX_IDS;
st_u16 id_off[] = SITRONIX_ID_OFF;

unsigned char fw_buf0[] = SITRONIX_FW;
unsigned char cfg_buf0[] = SITRONIX_CFG;
unsigned char fw_buf1[] = SITRONIX_FW1;
unsigned char cfg_buf1[] = SITRONIX_CFG1;
unsigned char fw_buf2[] = SITRONIX_FW2;
unsigned char cfg_buf2[] = SITRONIX_CFG2;

static void st_replace_fw_by_id(int id)
{
	if(id==0)
	{
		ST_INFO("Found id by SITRONIX_FW and SITRONIX_CFG\n");
		memcpy(fw_buf,fw_buf0,sizeof(fw_buf0));
		memcpy(cfg_buf,cfg_buf0,sizeof(cfg_buf0));
	}
	else if(id==1)
	{
		ST_INFO("Found id by SITRONIX_FW1 and SITRONIX_CFG1\n");
		memcpy(fw_buf,fw_buf1,sizeof(fw_buf1));
		memcpy(cfg_buf,cfg_buf1,sizeof(cfg_buf1));
	}
	else if(id==2)
	{
		ST_INFO("Found id by SITRONIX_FW2 and SITRONIX_CFG2\n");
		memcpy(fw_buf,fw_buf2,sizeof(fw_buf2));
		memcpy(cfg_buf,cfg_buf2,sizeof(cfg_buf2));
	}
}

static int st_select_fw_by_id(void)
{
	int ret=0;
	unsigned char buf[8];
	unsigned char id[4];
	int i=0;
	int idlen = sizeof(id_buf) / 4;
	int isFindID = 0;
	int status = st_get_device_status();
	if(status < 0)
	{
		return -1;
	}
	else if(status != 0x6)
	{
		st_i2c_read_bytes(0xC, buf, 4);
		st_i2c_read_bytes(0xF1, buf+4, 1);
		buf[6] = buf[4];
		buf[5] =  (buf[4] &0xFC) | 1;
		buf[4] = 0xF1;
		st_i2c_write_bytes(buf+4, 2);
		st_msleep(1);
		//
		st_i2c_read_bytes(0xF1, id, 1);
		ST_INFO("customer bank: %x \n",id[0]);
		//

		st_i2c_read_bytes(0xC, id, 4);
		buf[5] = buf[6];
		st_i2c_write_bytes(buf+4, 2);
		st_msleep(1);
		ST_INFO("customer ids: %x %x %x %x ,buf %x %x %x %x \n",id[0],id[1],id[2],id[3],buf[0],buf[1],buf[2],buf[3]);
		if(	id[0] == buf[0]
				&&	id[1] == buf[1]
				&&	id[2] == buf[2]
				&&	id[3] == buf[3])
		{
			ST_ERR("read customer id fail \n");
			return -1;
		}
		else
			ST_INFO("customer ids: %x %x %x %x \n",id[0],id[1],id[2],id[3]);


		for(i=0;i<idlen;i++)
		{
			if(	id[0] == id_buf[i*4]
					&&	id[1] == id_buf[i*4+1]
					&&	id[2] == id_buf[i*4+2]
					&&	id[3] == id_buf[i*4+3])
			{
				isFindID =1;
				st_replace_fw_by_id(i);
			}
		}
		if(0== isFindID)
			return -1;
	}
	else
	{
		ST_INFO("IC's status : boot code \n");
		// if bootcode
		if(0==st_v2_isp_off())
		{
			//could ispoff
			ST_ERR("IC's could go normat status \n");
			return st_select_fw_by_id();
		}
		else
		{
			ST_INFO("IC really bootcode mode \n");
			ret = st_v2_flash_read_page(fw_check,ST_CHECK_FW_OFFSET / ST_FLASH_PAGE_SIZE);
			if(ret < 0 )
			{
				ST_ERR("read flase fail! (%x)\n",ret);
			}

			for(i=0;i<idlen;i++)
			{
				if(	fw_check[0+id_off[i]] == id_buf[i*4]
						&&	fw_check[1+id_off[i]] == id_buf[i*4+1]
						&&	fw_check[2+id_off[i]] == id_buf[i*4+2]
						&&	fw_check[3+id_off[i]] == id_buf[i*4+3])
				{
					isFindID =1;
					st_replace_fw_by_id(i);
				}
			}

			if(0== isFindID)
			{
				//read file
				memset(fw_check,0xFF,ST_CFG_LEN);
				st_load_cfg_from_file(fw_check);

				for(i=0;i<idlen;i++)
				{
					if(	fw_check[0x0+id_off[i]] == id_buf[i*4]
							&&	fw_check[0x1+id_off[i]] == id_buf[i*4+1]
							&&	fw_check[0x2+id_off[i]] == id_buf[i*4+2]
							&&	fw_check[0x3+id_off[i]] == id_buf[i*4+3])
					{
						isFindID =1;
						st_replace_fw_by_id(i);
					}
				}

				if(0== isFindID)
					return -1;
			}
		}


	}
	return 0;
}
#endif //ST_UPGRADE_BY_ID

int st_upgrade_fw(void *data)
{

	int rt=0;
	int fwSize =0;
	int cfgSize =0;
	int fwInfoOff = 0;
	int fwInfoLen = ST_FW_INFO_LEN;
	int powerfulWrite = 0;

	is_loading_firmware_done = false;
	ST_INFO("firmware update start\n");

#ifdef ST_FIREWARE_FILE
	fwSize = st_load_fw_from_file(fw_buf);

	cfgSize = st_load_cfg_from_file(cfg_buf);

#else

#ifdef ST_UPGRADE_BY_ID

	if(0 != st_select_fw_by_id())
	{
		ST_ERR("find id fail , cancel upgrade\n");
		return -1;
	}
#endif

	fwSize = sizeof(fw_buf);
	cfgSize = sizeof(cfg_buf);
	ST_INFO("fwSize 0x%X,cfgsize 0x%X\n",fwSize,cfgSize);
#endif //ST_FIREWARE_FILE

	if(fwSize != 0)
	{
		fwInfoOff = st_get_fw_info_offset(fwSize,fw_buf);
		if(fwInfoOff <0){
			fwSize = 0;
			ST_ERR("check fwInfoOff Len error (%x), cancel upgrade\n",fwInfoOff);
			return -1;
		}
#ifdef ST_IC_A8008
		fwInfoLen = fw_buf[fwInfoOff]+1+4;
		if(fwInfoLen != ST_CFG_OFFSET - fwInfoOff)
		{
			ST_ERR("check FwInfo Len error (%x), cancel upgrade\n",fwInfoLen);
			return -1;
		}
#endif
#ifdef ST_IC_A8010
		fwInfoLen = fw_buf[fwInfoOff]+1+4;
#endif
#ifdef ST_IC_A1802
		fwInfoLen = fw_buf[fwInfoOff]*0x100 + fw_buf[fwInfoOff+1] +2+3;
#endif
		ST_INFO("fwInfoOff 0x%X , fwInfoLen 0x%x\n",fwInfoOff,fwInfoLen);
	}

	cfgSize = min(cfgSize,ST_CFG_LEN);
	if(cfgSize != 0)
	{
#if defined(ST_IC_A8008) || defined(ST_IC_A8010)
		if(cfg_buf[0] != 0x53 || cfg_buf[1] != 0x54 || cfg_buf[2] != 0x46 || cfg_buf[3] != 0x57 )
#endif
#if defined(ST_IC_A1802)
			if(cfg_buf[0] != 0x43 || cfg_buf[1] != 0x46 || cfg_buf[2] != 0x54 || cfg_buf[3] != 0x31 )
#endif
			{
				ST_ERR("cfg_buf error (invalid CFG)\n");
				return -1;
			}
	}

	if(fwSize ==0 && cfgSize ==0)
	{
		ST_ERR("can't find FW or CFG , cancel upgrade\n");
		return -1;
	}

	if(st_get_device_status() == 0x6)
		powerfulWrite = 1;

	st_irq_off();

	rt = st_isp_on();
	if(rt !=0)
	{
		ST_INFO("ISP on fail\n");
		goto ST_IRQ_ON;
	}

	if(st_check_chipid() < 0)
	{
		ST_INFO("Check ChipId fail\n");
		rt = -1;
		goto ST_ISP_OFF;
	}
#if defined(ST_IC_A1802)
	st_v2_set_2B_mode();
#endif

	if(powerfulWrite ==0 &&(fwSize !=0 || cfgSize!=0))
	{
		//check fw and cfg
		int checkOff = (fwInfoOff / ST_FLASH_PAGE_SIZE) *ST_FLASH_PAGE_SIZE;
#if defined(ST_IC_A8008) || defined(ST_IC_A8010)
		if(st_flash_read_page(fw_check,fwInfoOff / ST_FLASH_PAGE_SIZE)< 0 )
#endif
#if defined(ST_IC_A1802)
			if(st_v2_flash_read_page(fw_check,fwInfoOff / ST_FLASH_PAGE_SIZE)< 0 )
#endif
			{
				ST_INFO("read flash fail , cancel upgrade\n");
				rt = -1;
				goto ST_ISP_OFF;
			}

		if(fwSize !=0)
		{
			if(0 == st_compare_array(fw_check+(fwInfoOff-checkOff),fw_buf+fwInfoOff,fwInfoLen) )
			{
				ST_INFO("fw compare :same\n");
				fwSize = 0;
			}
			else
			{
				ST_INFO("fw compare :different\n");
			}

		}

		if(cfgSize !=0)
		{

#ifdef ST_IC_A8008
			if(0 == st_compare_array(fw_check+(ST_CFG_OFFSET-checkOff),cfg_buf,cfgSize))
#endif
#ifdef ST_IC_A8010
				st_flash_read_page(fw_check,ST_CFG_OFFSET / ST_FLASH_PAGE_SIZE);
			if(0 == st_compare_array(fw_check,cfg_buf,cfgSize))
#endif
#ifdef ST_IC_A1802
				st_v2_flash_read_page(fw_check,ST_CFG_OFFSET / ST_FLASH_PAGE_SIZE);
			if(0 == st_compare_array(fw_check,cfg_buf,cfgSize))
#endif
			{
				ST_INFO("cfg compare :same\n");
				cfgSize = 0;
			}
			else
			{
				ST_INFO("cfg compare : different\n");
			}

		}

	}


#if defined(ST_IC_A8008) || defined(ST_IC_A8010)
	if(cfgSize !=0)
		st_flash_write(cfg_buf,ST_CFG_OFFSET,cfgSize);

	if(fwSize !=0)
		st_flash_write(fw_buf,0,fwSize);
#endif

#if defined(ST_IC_A1802)
	if(cfgSize !=0)
		st_v2_flash_write(cfg_buf,ST_CFG_OFFSET,cfgSize);
	if(fwSize !=0)
		st_v2_flash_write(fw_buf,0,fwSize);
#endif



ST_ISP_OFF:
#if defined(ST_IC_A8008) || defined(ST_IC_A8010)
	rt = st_isp_off();
#endif
#if defined(ST_IC_A1802)
	rt = st_v2_isp_off();
#endif
ST_IRQ_ON:
	st_irq_on();
	is_loading_firmware_done = true;
	if(cfgSize != 0 || fwSize != 0){
		st16xx_print_version(); //update firmware version
		ST_INFO("update firmware sucess!\n");
		return 1;
	}
	else{
		ST_ERR("update firmware failed!\n");
		return 0;
	}

}

#endif

static ssize_t fih_tpfwver_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t num_read_chars = 0;

	char buffer;
	int ret = -1;
	int vendor_id = 0;
	int ic_type = 0;

	ret = tpd_i2c_read(i2c_client, &buffer, 1, 0x0);

	if( ret < 0 ){
		ST_ERR("[mtk-tpd] i2c communcate error in getting FW version : 0x%x\n", ret);
		return -1;
	}
//04 for stironix
	num_read_chars = snprintf(buf, PAGE_SIZE, "04_%02x_%02x_%02x\n", ic_type, buffer, vendor_id);

	return num_read_chars;
}

static ssize_t fih_tpfwver_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    return -EPERM;
}

static DEVICE_ATTR(tp_fw_version, S_IRUGO | S_IWUSR, fih_tpfwver_show, fih_tpfwver_store);

static struct attribute *fih_attributes[] = {
	&dev_attr_tp_fw_version.attr,
	NULL
};

static struct attribute_group fih_attribute_group = {
	.attrs = fih_attributes
};

static int fih_tp_create_sysfs(void)
{
	int ret = 0;
	struct kobject *fih_tp_kobj;

	fih_tp_kobj = kobject_create_and_add("android_tp", NULL);
	if(fih_tp_kobj == NULL) {
		ST_ERR("fih create kobject is failed\n");
		return -1;
	}

	ret = sysfs_create_group(fih_tp_kobj, &fih_attribute_group);
	if (ret) {
	    ST_ERR("sysfs_create_group() failed!!\n");
	    sysfs_remove_group(fih_tp_kobj, &fih_attribute_group);
	    return -ENOMEM;
	} else {
	    ST_INFO("sysfs_create_group() succeeded!!\n");
	}

	return ret;
}

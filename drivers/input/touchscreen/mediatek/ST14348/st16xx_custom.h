/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
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
#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/bitops.h>
#include <linux/kernel.h>
#include <linux/byteorder/generic.h>
#include <linux/kthread.h>
#include <linux/mount.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include <linux/interrupt.h>
#include <linux/time.h>
//#include <linux/rtpm_prio.h>
#include <linux/dma-mapping.h>
#include <asm/uaccess.h>
#include <linux/miscdevice.h>

#include <linux/delay.h>

#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/jiffies.h>
#include "tpd.h"


#ifndef TOUCHPANEL_H__
#define TOUCHPANEL_H__

#ifndef RTPM_PRIO_TPD
#define RTPM_PRIO_TPD		0x04
#endif

#define STTP_USE_POWER 		1

/* Pre-defined definition */
#define TPD_TYPE_CAPACITIVE
//#define TPD_TYPE_RESISTIVE
//#define TPD_POWER_SOURCE

#define TPD_I2C_NUMBER           1
#define TPD_WAKEUP_TRIAL         60
#define TPD_WAKEUP_DELAY         100
#define TPD_DELAY                (2*HZ/100)

#define TPD_SWAP_Y		 1
#define ERROR_PATH     		 -13  //Mount path error

#define TPD_RES_X                720//320
#define TPD_RES_Y                1440//320


//#define TPD_HAVE_BUTTON
//#define TPD_HAVE_PHYSICAL_BUTTON

#define CHIP_WORK_MODE_ACTIVE       0x0
#define CHIP_WORK_MODE_MONITOR      0x1
#define CHIP_WORK_MODE_HIBERNATE    0x3

#define INDEX_CMD_WRITE             0x20
#define INDEX_QUERY_READ            0x80
#define INDEX_CMD_READ              0xA0
#define INDEX_POINT_READ            0xE0

#define QUERY_COMMAND_READY         0x00
#define QUERY_COMMAND_BUSY          0x01
#define QUERY_COMMAND_ERROR         0x02
#define COMMAND_RESPONSE_SUCCESS    0x0000
#define SIGNATURE_LENGTH            16


//Configure register
#define REG_VALID_DETECT            0x80
#define REG_ZOOM_DISTANCE           0x97
#define REG_PERIODACTIVE            0x88
#define SCI_ASSERT_TP
#define REG_GESTURE_ID              0x01
#define REG_NUM_FINGER              0x02
#define REG_1ST_FINGER              0x03
#define REG_2ED_FINGER              0x09
#define REG_3RD_FINGER              0x0F
#define REG_4TH_FINGER              0x15
#define REG_5TH_FINGER              0x1B
#define REG_GT_ZOOM_OUT             0x49
#define REG_GT_ZOOM_IN              0x48
#define REG_FIRMWARE_ID		    0xa6
#define REG_CALI_TPC		    0xfc

#define TOUCH_VALID_THRESHOLD       0x16
#define TOUCH_VAL_ZOOM_THRESHOLD    0x30
#define TOUCH_PERIODACTIVE          0x6

//touchpanel firmware update
#define TP_ERR_READID   	    0x02
#define TP_ERR_ECC    		    0x03
#define TP_OK			    0x04
/// #define MAX_BUFFER_SIZE             130

#define DEBUG_ST 0

#define ST_INFO(fmt, arg...)	\
	do{ \
		pr_info("STTP-DBG[%s:%d]"fmt"\n", __func__, __LINE__, ##arg);\
	}while(0)

#define ST_ERR(fmt, arg...)	\
	do{ \
		pr_err("STTP-DBG[%s:%d]"fmt"\n", __func__, __LINE__, ##arg);\
	}while(0)

#if DEBUG_ST
#define ST_DEBUG(fmt, arg...)	\
	do{ \
		pr_debug("STTP-DBG[%s:%d]"fmt"\n", __func__, __LINE__, ##arg);\
	}while(0)

#else
#define ST_DEBUG(fmt, arg...)	\
	do{ \
	}while(0)
#endif//end fo DEBUG_ST

#define ST16XX_MAX_TOUCHES	5

typedef enum{
	FIRMWARE_VERSION,
	STATUS_REG,
	DEVICE_CONTROL_REG,
	TIMEOUT_TO_IDLE_REG,
	XY_RESOLUTION_HIGH,
	X_RESOLUTION_LOW,
	Y_RESOLUTION_LOW,
	DEVICE_CONTROL_REG2 = 0x09,
	FIRMWARE_REVISION_3 = 0x0C,
	FIRMWARE_REVISION_2,
	FIRMWARE_REVISION_1,
	FIRMWARE_REVISION_0,
	FINGERS,
	KEYS_REG,
	XY0_COORD_H,
	X0_COORD_L,
	Y0_COORD_L,
	I2C_PROTOCOL = 0x3E,
	MAX_NUM_TOUCHES,
	DATA_0_HIGH,
	DATA_0_LOW,
	MISC_CONTROL = 0xF1,
	SMART_WAKE_UP_REG = 0xF2,
	CHIP_ID = 0xF4,
	PAGE_REG = 0xff,
}RegisterOffset;

//Upgrade

#define ST_UPGRADE_FIRMWARE

#ifdef ST_UPGRADE_FIRMWARE
//#define ST_UPGRADE_BY_ID
//#define ST_REPLACE_CFG
#define ST_FIREWARE_FILE
//#define ST_IC_A8008
//#define ST_IC_A8010
#define ST_IC_A1802


#ifdef ST_IC_A8008
#define ST_FW_LEN	0x4000
#define ST_CFG_LEN	0xFE
#define ST_CFG_OFFSET	0x3F00
#define ST_FLASH_PAGE_SIZE 1024
#define ST_FLASH_SIZE 0x3FFE
#define ST_CHECK_FW_OFFSET 	0x3F00
#endif

#ifdef ST_IC_A8010
#define ST_FW_LEN	0x6900
#define ST_CFG_LEN	0x2C0
#define ST_CFG_OFFSET	0xBC00
#define ST_FLASH_PAGE_SIZE 1024
#define ST_FLASH_SIZE 0xBEC0
#define ST_CHECK_FW_OFFSET 	0x6900
#endif

#ifdef ST_IC_A1802
#define ST_FW_LEN	0xF000
#define ST_CFG_LEN	0x800
#define ST_CFG_OFFSET	0xF000
#define ST_FLASH_PAGE_SIZE 4096
#define ST_FLASH_SIZE 0x10000
#define ST_FLASH_READ_SIZE 0xF800
#define ST_CHECK_FW_OFFSET 0xF000

#define ST_ISP_RETRY_MAX	1
//#define ST_ISP_MAX_TRANS_LEN	8
#define ST_ISP_MAX_TRANS_LEN	258
#define ST_ISP_MAX_WRITE_LEN	ST_ISP_MAX_TRANS_LEN-2
#define ST_ISP_BLOCK_SIZE	256
#define st_u16 u16
#define st_u32 u32
#endif

#define ISP_PACKET_SIZE 8
#define ST_FW_INFO_LEN	0x10

#define ISP_CMD_ERASE						0x80
#define ISP_CMD_SEND_DATA					0x81
#define	ISP_CMD_WRITE_FLASH					0x82
#define	ISP_CMD_READ_FLASH					0x83
#define	ISP_CMD_RESET						0x84
#define	ISP_CMD_UNLOCK						0x87
#define	ISP_CMD_READY						0x8F

#ifdef ST_FIREWARE_FILE
// from file
#define st_file file
#define st_filp_open filp_open
#define st_filp_close filp_close
#define ST_FW_DIR "/vendor"
#define ST_FW_PATH "/vendor/etc/Sitronix_FW_FIH.bin"
#define ST_CFG_PATH "/vendor/etc/Sitronix_CFG_FIH.bin"

#else

#define SITRONIX_FW {\
}

#define SITRONIX_CFG {\
}

#ifdef ST_UPGRADE_BY_ID
#define SITRONIX_IDS {\
	0x0,0x0,0x0,0x0,\
	0x1,0x2,0x3,0x4\
}
#define SITRONIX_ID_OFF {\
	0xD6,\
	0xD6\
}

#define SITRONIX_FW1 {\
}

#define SITRONIX_CFG1 {\
}

#define SITRONIX_FW2 {\
}

#define SITRONIX_CFG2 {\
}

#endif	//ST_UPGRADE_BY_ID

#ifdef ST_REPLACE_CFG
#define STIRONIX_CFG_CHECKSUM_OFFSET	0xFC
#define STIRONIX_FW_SET_COUNT	2
#define SITRONIX_CFG_VERSION {\
	0x47,\
	0x48\
}
#define SITRONIX_F_OFFSET {\
	0x9E,0x9F,0xA0,\
	0x9E,0x9F,0xA0\
}
#define SITRONIX_T_OFFSET {\
	0x9E,0x9F,0xA0\
}
#endif	//ST_REPLACE_CFG

#endif //ST_FIREWARE_FILE

#endif //ST_UPGRADE_FIRMWARE
//

#if defined(ST_TEST_RAW) || defined(ST_UPGRADE_FIRMWARE)
#define st_u8 u8
#define st_char char
#define st_msleep msleep
#define st_int int
#endif

#endif /* TOUCHPANEL_H__ */


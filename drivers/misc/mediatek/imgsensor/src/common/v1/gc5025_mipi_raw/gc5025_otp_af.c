/*
 * Driver for CAM_CAL
 *
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include "kd_camera_typedef.h"
#include "cam_cal.h"
#include "cam_cal_define.h"
//#include "gc5025a_otp.h"
//#include <asm/system.h>  // for SMP
#include <linux/dma-mapping.h>

#define PFX "GC5025A_OTP_FMT"


#define CAM_CALGETDLT_DEBUG
#define CAM_CAL_DEBUG
#ifdef CAM_CAL_DEBUG
#define CAM_CALINF(format, args...)    printk(PFX "[%s] " format, __func__, ##args)
#define CAM_CALDB(format, args...)     printk(PFX "[%s] " format, __func__, ##args)
#define CAM_CALERR(format, args...)    printk(KERN_ERR format, ##args)
#else
#define CAM_CALINF(x,...)
#define CAM_CALDB(x,...)
#define CAM_CALERR(x, ...)
#endif

static DEFINE_SPINLOCK(g_CAM_CALLock); // for SMP


#define USHORT             unsigned short
#define BYTE               unsigned char
#define Sleep(ms) mdelay(ms)

#define CAM_CAL_DEV_MAJOR_NUMBER 226
/*******************************************************************************
*
********************************************************************************/
#define CAM_CAL_DRVNAME "CAM_CAL_DRV"

static dev_t g_CAM_CALdevno = MKDEV(CAM_CAL_DEV_MAJOR_NUMBER,0);
static struct cdev * g_pCAM_CAL_CharDrv = NULL;
static struct class *CAM_CAL_class = NULL;

extern int GetAfCode_GC5025A(u16 *Inf,u16 *Macro);


typedef struct {
    u16    AfInf;
    u16    AfMacro;
}AFOTP;

typedef union {
        u16   Data[2];
        AFOTP AfOtpData;
} AFOTP_DATA;

AFOTP_DATA GC5025AOTP = {{0}};
static int selective_read_region(u32 sensorID,u32 offset, BYTE* data, u32 size)
{

    CAM_CALDB("[gc5025a_otp]selective_read_region SensorID=0x%x offset =%x size %d \n",sensorID,offset,size);
    if (offset == 0xef && size == 1) { //layout check
         *data = 0xfe;
    }
	else if(sensorID == 0x5025) {
    	if(GetAfCode_GC5025A(&GC5025AOTP.Data[0],&GC5025AOTP.Data[1])) {
            CAM_CALDB("[gc5025a_otp]selective_read region AFInf= %d, AFMacro=%d \n",
                    GC5025AOTP.AfOtpData.AfInf,GC5025AOTP.AfOtpData.AfMacro);
            memcpy(data, &GC5025AOTP.Data[offset], size);
	    }
        else {
            CAM_CALDB("use predefined data  AFInf= %d, AFMacro=%d \n",
                    GC5025AOTP.AfOtpData.AfInf,GC5025AOTP.AfOtpData.AfMacro);
            memcpy(data, &GC5025AOTP.Data[offset], size);
        }
    }
    return 1;
}

/*******************************************************************************
*
********************************************************************************/
static long CAM_CAL_Ioctl(struct file *file, unsigned int a_u4Command,
    unsigned long a_u4Param)
{
    int i4RetValue = 0;
    u8 * pBuff = NULL;
    u8 * pu1Params = NULL;
    struct stCAM_CAL_INFO_STRUCT *ptempbuf;
    CAM_CALDB("[gc5025a_otp]CAM_CAL_Ioctl Enter \n");
    if(_IOC_NONE == _IOC_DIR(a_u4Command))
    {
         CAM_CALERR("[gc5025a_otp]CAM_CAL_Ioctl _IOC_NONE\n");
    }
    else
    {
        pBuff = (u8 *)kmalloc(sizeof(struct stCAM_CAL_INFO_STRUCT),GFP_KERNEL);

        if(NULL == pBuff)
        {
            CAM_CALERR("[gc5025a_otp]ioctl allocate mem failed\n");
            return -ENOMEM;
        }

        if(_IOC_WRITE & _IOC_DIR(a_u4Command))
        {
            if(copy_from_user((u8 *) pBuff , (u8 *) a_u4Param, sizeof( struct stCAM_CAL_INFO_STRUCT)))
            {    //get input structure address
                kfree(pBuff);
                CAM_CALERR("[gc5025a_otp]ioctl copy from user failed\n");
                return -EFAULT;
            }
        }
    }

    ptempbuf = (struct stCAM_CAL_INFO_STRUCT *)pBuff;
    pu1Params = (u8*)kmalloc(ptempbuf->u4Length,GFP_KERNEL);
    if(NULL == pu1Params)
    {
        kfree(pBuff);
        CAM_CALERR("[gc5025a_otp]ioctl allocate mem failed\n");
        return -ENOMEM;
    }

    if(copy_from_user((u8*)pu1Params, (u8*)ptempbuf->pu1Params, ptempbuf->u4Length))
    {
        kfree(pBuff);
        kfree(pu1Params);
        CAM_CALERR("[gc5025a_otp]ioctl copy from user failed\n");
        return -EFAULT;
    }
    CAM_CALDB("[gc5025a_otp] SensorID= 0x%x \n",ptempbuf->sensorID);
    switch(a_u4Command)
    {
        case CAM_CALIOC_S_WRITE:
            CAM_CALDB("[gc5025a_otp]Write CMD \n");
            i4RetValue = 0;//iWriteData((u16)ptempbuf->u4Offset, ptempbuf->u4Length, pu1Params);
            break;
        case CAM_CALIOC_G_READ:
            CAM_CALDB("[gc5025a_otp]Read CMD \n");
            i4RetValue = selective_read_region(ptempbuf->sensorID,ptempbuf->u4Offset, pu1Params, ptempbuf->u4Length);
            CAM_CALDB("[gc5025a_otp]Read CMD pu1Params = %x , %x\n",(int)(*pu1Params),(int)(*(pu1Params+1)));
            CAM_CALDB("[gc5025a_otp]Read CMD i4RetValue=0x%x \n",i4RetValue);

            break;
        default :
      	     CAM_CALINF("[CAM_CAL] No CMD \n");
            i4RetValue = -EPERM;
        break;
    }

    if(_IOC_READ & _IOC_DIR(a_u4Command))
    {
        //copy data to user space buffer, keep other input paremeter unchange.
        if(copy_to_user((u8 __user *) ptempbuf->pu1Params , (u8 *)pu1Params , ptempbuf->u4Length))
        {
            kfree(pBuff);
            kfree(pu1Params);
            CAM_CALERR("[gc5025a_otp]ioctl copy to user failed\n");
            return -EFAULT;
        }
    }

    kfree(pBuff);
    kfree(pu1Params);
    return i4RetValue;
}


static u32 g_u4Opened = 0;
static int CAM_CAL_Open(struct inode * a_pstInode, struct file * a_pstFile)
{
    CAM_CALDB("[gc5025a_otp]CAM_CAL_Open\n");
    spin_lock(&g_CAM_CALLock);
    if(g_u4Opened)
    {
        spin_unlock(&g_CAM_CALLock);
		CAM_CALERR("[gc5025a_otp]Opened, return -EBUSY\n");
        return -EBUSY;
    }
    else
    {
        g_u4Opened = 1;
    }
    spin_unlock(&g_CAM_CALLock);
    return 0;
}

static int CAM_CAL_Release(struct inode * a_pstInode, struct file * a_pstFile)
{
    CAM_CALDB("[gc5025a_otp]CAM_CAL_Release\n");
    spin_lock(&g_CAM_CALLock);
    g_u4Opened = 0;
    spin_unlock(&g_CAM_CALLock);

    return 0;
}

static const struct file_operations g_stCAM_CAL_fops =
{
    .owner = THIS_MODULE,
    .open = CAM_CAL_Open,
    .release = CAM_CAL_Release,
    .unlocked_ioctl = CAM_CAL_Ioctl
};


inline static int RegisterCAM_CALCharDrv(void)
{
    struct device* CAM_CAL_device = NULL;
    CAM_CALDB("[gc5025a_otp]RegisterCAM_CALCharDrv\n");

    if(alloc_chrdev_region(&g_CAM_CALdevno, 0, 1, CAM_CAL_DRVNAME))
    {
        CAM_CALERR("[gc5025a_otp]Allocate device no failed\n");

        return -EAGAIN;
    }

    //Allocate driver
    g_pCAM_CAL_CharDrv = cdev_alloc();

    if(NULL == g_pCAM_CAL_CharDrv)
    {
        unregister_chrdev_region(g_CAM_CALdevno, 1);

        CAM_CALERR("[gc5025a_otp]Allocate mem for kobject failed\n");

        return -ENOMEM;
    }

    //Attatch file operation.
    cdev_init(g_pCAM_CAL_CharDrv, &g_stCAM_CAL_fops);

    g_pCAM_CAL_CharDrv->owner = THIS_MODULE;

    //Add to system
    if(cdev_add(g_pCAM_CAL_CharDrv, g_CAM_CALdevno, 1))
    {
        CAM_CALERR("[gc5025a_otp]Attatch file operation failed\n");

        unregister_chrdev_region(g_CAM_CALdevno, 1);

        return -EAGAIN;
    }

    CAM_CAL_class = class_create(THIS_MODULE, "CAM_CALdrv_GC5025A");
    if (IS_ERR(CAM_CAL_class)) {
        int ret = PTR_ERR(CAM_CAL_class);
        CAM_CALERR("[gc5025a_otp]Unable to create class, err = %d\n", ret);
        return ret;
    }
    CAM_CAL_device = device_create(CAM_CAL_class, NULL, g_CAM_CALdevno, NULL, CAM_CAL_DRVNAME);

    return 0;
}

inline static void UnregisterCAM_CALCharDrv(void)
{
    //Release char driver
    cdev_del(g_pCAM_CAL_CharDrv);

    unregister_chrdev_region(g_CAM_CALdevno, 1);

    device_destroy(CAM_CAL_class, g_CAM_CALdevno);
    class_destroy(CAM_CAL_class);
}

static int CAM_CAL_probe(struct platform_device *pdev)
{
    return 0;
}

static int CAM_CAL_remove(struct platform_device *pdev)
{
    return 0;
}

// platform structure
static struct platform_driver g_stCAM_CAL_Driver = {
    .probe		= CAM_CAL_probe,
    .remove	    = CAM_CAL_remove,
    .driver		= {
        .name	= CAM_CAL_DRVNAME,
        .owner	= THIS_MODULE,
    }
};


static struct platform_device g_stCAM_CAL_Device = {
    .name = CAM_CAL_DRVNAME,
    .id = 0,
    .dev = {
    }
};

static int __init CAM_CAL_init(void)
{
    int i4RetValue = 0;
    CAM_CALDB("[gc5025a_otp]CAM_CAL_i2C_init\n");
   //Register char driver
	i4RetValue = RegisterCAM_CALCharDrv();
    if(i4RetValue){
 	   CAM_CALDB("[gc5025a_otp]register char device failed!\n");
	   return i4RetValue;
	}
	CAM_CALDB("[gc5025a_otp]Attached!! \n");

    if(platform_driver_register(&g_stCAM_CAL_Driver)){
        CAM_CALERR("[gc5025a_otp]failed to register gc5025a_otp driver\n");
        return -ENODEV;
    }

    if (platform_device_register(&g_stCAM_CAL_Device))
    {
        CAM_CALERR("[gc5025a_otp]failed to register gc5025a_otp driver, 2nd time\n");
        return -ENODEV;
    }

    return 0;
}

static void __exit CAM_CAL_exit(void)
{
	platform_driver_unregister(&g_stCAM_CAL_Driver);
}

module_init(CAM_CAL_init);
module_exit(CAM_CAL_exit);

MODULE_DESCRIPTION("CAM_CAL otp driver");
MODULE_AUTHOR("Sean Lin <Sean.Lin@Mediatek.com>");
MODULE_LICENSE("GPL");



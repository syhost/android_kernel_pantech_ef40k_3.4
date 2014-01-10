/* Source for:
 * Cypress TrueTouch(TM) Standard Product I2C touchscreen driver.
 * drivers/input/touchscreen/cyttsp-i2c.c
 *
 * Copyright (C) 2009, 2010 Cypress Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * Cypress reserves the right to make changes without further notice
 * to the materials described herein. Cypress does not assume any
 * liability arising out of the application described herein.
 *
 * Contact Cypress Semiconductor at www.cypress.com
 *     
 */   

#include <linux/delay.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <mach/gpio.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/byteorder/generic.h>
#include <linux/bitops.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif /* CONFIG_HAS_EARLYSUSPEND */
#include <linux/regulator/consumer.h>
#include <mach/vreg.h>
#include <linux/wakelock.h>
#include <linux/input/mt.h>

#define CYTTSP_DECLARE_GLOBALS
#include <linux/miscdevice.h>
#include <linux/cyttsp.h>
#ifdef FEATURE_CYTTSP_FIRMWAREUPGRADE
#include <linux/issp_defs.h>
#include <linux/issp_extern.h>
#endif

#include <linux/uaccess.h>
#include "touch_ioctl.h"
uint32_t cyttsp_tsdebug1;

module_param_named(tsdebug1, cyttsp_tsdebug1, uint, 0664);

#define FEATURE_TOUCH_KEY

/* -------------------------------------------------------------------- */
/* EF33S gpio & resolution & key area*/
/* -------------------------------------------------------------------- */
#define GPIO_TOUCH_RST			95
#define GPIO_TOUCH_CHG			61
#define GPIO_TOUCH_SDA			64
#define GPIO_TOUCH_SCL			65
#define GPIO_TOUCH_ID			93
#define IRQ_TOUCH_INT			gpio_to_irq(GPIO_TOUCH_CHG)


/* -------------------------------------------------------------------- */
/* debug option */
/* -------------------------------------------------------------------- */
//#define TOUCH_DBG_ENABLE
#ifdef TOUCH_DBG_ENABLE
#define dbg(fmt, args...)   printk("[TOUCH]" fmt, ##args)
#else
#define dbg(fmt, args...)
#endif
#define dbg_func_in()       dbg("[FUNC_IN] %s\n", __func__)
#define dbg_func_out()      dbg("[FUNC_OUT] %s\n", __func__)
#define dbg_line()          dbg("[LINE] %d(%s)\n", __LINE__, __func__)
/* -------------------------------------------------------------------- */

#define FEATURE_SKY_PROCESS_CMD_KEY

#ifdef FEATURE_TOUCH_KEY
#define X_MAX	480
#define Y_MAX	800

#define NULL_KEY_AREA	840

#define MENU_KEY_MIN	40
#define MENU_KEY_MAX	140

#define HOME_KEY_MIN	210
#define HOME_KEY_MAX	280

#define BACK_KEY_MIN	360
#define BACK_KEY_MAX	460
#endif

#ifdef FEATURE_CYTTSP_HEARTBEAT
#define CYTTSP_HEARTBEAT_TIME	3
#endif

#define TOUCH_MAX_NUM   4 //	 2

#define SENSOR_X	12
#define SENSOR_Y	20
#define MAX_NODE 	SENSOR_X*SENSOR_Y

#define CYTTSP_BASE_MIN	65
#define CYTTSP_BASE_MAX	135

#define CYTTSP_MUTEX_LOCK //ST_LIM_1201XX I2C 통신일 경우 MUTEX LOCK UNLOCK 구현.
#define CYTTSP_TOUCH_DEBUG_ENABLE //ST_LIM_120517 Debug 메세지 추가.


//----------------- Added --------------//


/* abs settings */
/* abs value offsets */
#define CY_NUM_ABS_VAL              5 /* number of abs values per setting */
#define CY_SIGNAL_OST               0
#define CY_MIN_OST                  1
#define CY_MAX_OST                  2
#define CY_FUZZ_OST                 3
#define CY_FLAT_OST                 4
/* axis signal offsets */
#define CY_NUM_ABS_SET              5 /* number of abs signal sets */
#define CY_ABS_X_OST                0
#define CY_ABS_Y_OST                1
#define CY_ABS_P_OST                2
#define CY_ABS_W_OST                3
#define CY_ABS_ID_OST               4
#define CY_IGNORE_VALUE             0xFFFF /* mark unused signals as ignore */

#define HI_TRACKID(reg)        ((reg & 0xF0) >> 4)
#define LO_TRACKID(reg)        ((reg & 0x0F) >> 0)

#if 1 // Fastfinger tapping
#define ABS(a,b)    (a>b?a-b:b-a)
#define FINGER_SEP_SQRT_XY  (25600) // 160*160 pixel
#endif

int prev_touches=0;

//----------------- --------------------------//

/* ****************************************************************************
 * static value
 * ************************************************************************** */
static struct cyttsp_gen3_xydata_t g_xy_data;
//static struct cyttsp_bootloader_data_t g_bl_data;
//static struct cyttsp_sysinfo_data_t g_sysinfo_data;
//static struct cyttsp_gen3_xydata_t g_wake_data;
static const struct i2c_device_id cyttsp_id[] = {
	{ CYTTSP_I2C_NAME, 0 },  { }
};

/* Touch structure */
struct cyttsp_trk{
	bool tch;
	int abs[CY_NUM_ABS_SET];
};

/* CY TTSP I2C Driver private data */
struct cyttsp {
	struct i2c_client *client;
	struct input_dev *input;
	struct work_struct work;
#ifdef FEATURE_CYTTSP_HEARTBEAT
	struct work_struct work2;
#endif
#ifdef FEATURE_CYTTSP_FIRMWAREUPGRADE
	struct delayed_work work3;
#endif
	struct timer_list timer;
	struct mutex mutex;
#ifdef CYTTSP_MUTEX_LOCK
	struct mutex lock_mutex;
#endif
	char phys[32];
	struct cyttsp_platform_data *platform_data;
	u8 num_prev_touch;
	u16 active_track[CYTTSP_NUM_TRACK_ID];
	u16 prev_st_touch[CYTTSP_NUM_ST_TOUCH_ID];
	u16 prev_mt_touch[CYTTSP_NUM_MT_TOUCH_ID];
	u16 prev_mt_pos[CYTTSP_NUM_MT_TOUCH_ID][2];

    struct cyttsp_trk prv_trk[CYTTSP_NUM_TRACK_ID];


	atomic_t irq_enabled;
	struct early_suspend early_suspend;
};

#ifdef FEATURE_CYTTSP_HEARTBEAT
static int start_heartbeat_timer = false;
#endif

#ifdef FEATURE_SKY_PROCESS_CMD_KEY
struct cyttsp *cyttsp_data = NULL;
#endif

/* To check touch chip */
static int Touch_Dbg_Enable =0;
//static u16 prev_mt_pos[CYTTSP_NUM_TRACK_ID][2];
static struct wake_lock touch_wake_lock;

typedef enum touch_status {
        TOUCH_POWERON,
        TOUCH_POWEROFF,
        TOUCH_UPDATE
} touch_status;

typedef enum
{
        BATTERY_PLUGGED_NONE = 0,
        BATTERY_PLUGGED_AC = 1,
        BATTERY_PLUGGED_USB = 2,
        BATTERY_PLUGGED_SLEEP = 10
} CHARGER_MODE;

static int Touch_Status =TOUCH_POWERON;

static int Touch_ChagerMode = BATTERY_PLUGGED_NONE;

static unsigned char bBlack=false;

struct cyttsp *ts_temp;

MODULE_DEVICE_TABLE(i2c, cyttsp_id);

/* ****************************************************************************
 * Prototypes for static functions
 * ************************************************************************** */
static void cyttsp_xy_worker(struct work_struct *work);
#ifdef FEATURE_CYTTSP_HEARTBEAT
static void cyttsp_check_heartbeat(struct work_struct *work2);
#endif
#ifdef FEATURE_CYTTSP_FIRMWAREUPGRADE
void check_firmware_update(struct work_struct *work3);
#endif
static irqreturn_t cyttsp_irq(int irq, void *handle);
#if 0
static int cyttsp_inlist(u16 prev_track[], u8 curr_track_id, u8 *prev_loc, u8 num_touches);
static int cyttsp_next_avail_inlist(u16 curr_track[], u8 *new_loc, u8 num_touches);
#endif
#ifdef CYTTSP_INCLUDE_LOAD_FILE	//[BIH] ICS port...
static int cyttsp_putbl(struct cyttsp *ts, int show, int show_status, int show_version, int show_cid);
#endif// CYTTSP_INCLUDE_LOAD_FILE	//[BIH] ICS port...
static int __devinit cyttsp_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int __devexit cyttsp_remove(struct i2c_client *client);
static int cyttsp_resume(struct i2c_client *client);
static int cyttsp_suspend(struct i2c_client *client, pm_message_t message);

#ifdef FEATURE_SKY_PROCESS_CMD_KEY
static long ts_fops_ioctl(struct file *filp,unsigned int cmd, unsigned long arg);
static int ts_fops_open(struct inode *inode, struct file *filp);
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void cyttsp_early_suspend(struct early_suspend *handler);
static void cyttsp_late_resume(struct early_suspend *handler);
#endif /* CONFIG_HAS_EARLYSUSPEND */

static int pantech_auto_check(u8*);
#ifdef FEATURE_CYTTSP_FIRMWAREUPGRADE
static int firmware_update_by_user(void);
static int firmware_version_check(void);
#endif

static int pantech_selftest_check(void);

void Change_Active_Distance(u8 value); //test

/* ****************************************************************************
 *
 * ************************************************************************** */

static struct i2c_driver cyttsp_driver = {
	.driver = {
		.name = CYTTSP_I2C_NAME,
		.owner = THIS_MODULE,
	},
	.probe = cyttsp_probe,
	.remove = __devexit_p(cyttsp_remove),
//	.suspend = cyttsp_suspend,
//	.resume = cyttsp_resume,
	.id_table = cyttsp_id,
};

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Cypress TrueTouch(R) Standard touchscreen driver");
MODULE_AUTHOR("Cypress");

#ifdef FEATURE_SKY_PROCESS_CMD_KEY
struct cyttsp *sky_process_cmd_ts=NULL;

static struct file_operations ts_fops = {
	.owner = THIS_MODULE,
	.open = ts_fops_open,
//	.release = ts_fops_close,
	.unlocked_ioctl = ts_fops_ioctl,
};

static struct miscdevice touch_event = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "touch_fops",
	.fops = &ts_fops,
};

static int ts_fops_open(struct inode *inode, struct file *filp)
{
	//filp->private_data = cyttsp_data;
	return 0;
}

#if 1	//[BIH] ICS port...  ioctl changed to unlocked_ioctl or compact_ioctl...
//static DEFINE_MUTEX(cyttsp_mutex);
    
static long ts_fops_ioctl(struct file *filp,
	       unsigned int cmd, unsigned long arg)
#else
static int ts_fops_ioctl(struct inode *inode, struct file *filp,
	       unsigned int cmd, unsigned long arg)
#endif
{
	void __user *argp = (void __user *)arg;

	if(cyttsp_data ==NULL)
	{
	       printk("[TOUCH] Null Device\n");
		return 0;
	}

       dbg("[TOUCH] cmd = %d, argp = 0x%x\n", cmd, (unsigned int)argp);

//	mutex_lock(&cyttsp_mutex);

	switch (cmd)
	{
		case TOUCH_IOCTL_READ_LASTKEY:
			break;
		case TOUCH_IOCTL_DO_KEY:
			if ( (int)argp == 0x20a )
				input_report_key(cyttsp_data->input, 0xe3, 1);
			else if ( (int)argp == 0x20b )
				input_report_key(cyttsp_data->input, 0xe4, 1);
			else
				input_report_key(cyttsp_data->input, (int)argp, 1);

			input_sync(cyttsp_data->input);

			if((int)argp == KEY_9)
			{
				printk("Enable Touch Debug!!\n");
				Touch_Dbg_Enable = true;
			}
			else if((int)argp == KEY_8)
			{
				printk("Disable Touch Debug!!\n");
				Touch_Dbg_Enable = false;
			}
			/*
			else if((int)argp == KEY_F2)
			{
				int ret = 0;
				printk("Start Touch Firmware update!!\n");
				ret = firmware_update_by_user();
			}
			*/
			break;
		case TOUCH_IOCTL_RELEASE_KEY:
			if ( (int)argp == 0x20a )
				input_report_key(cyttsp_data->input, 0xe3, 0);
			else if ( (int)argp == 0x20b )
				input_report_key(cyttsp_data->input, 0xe4, 0);
			else
				input_report_key(cyttsp_data->input, (int)argp, 0);

			input_sync(cyttsp_data->input);
			
			break;
		// +++ FEATURE_P_VZW_PS_STABILITY_AT_CMD
		case TOUCH_IOCTL_PRESS_TOUCH:
			{
				int touchX=arg&0x0000FFFF;
				int touchY= (arg >> 16) & 0x0000FFFF;
				
				input_report_abs(cyttsp_data->input, ABS_MT_TOOL_TYPE , 1);
				input_report_abs(cyttsp_data->input, ABS_MT_TOUCH_MAJOR, CYTTSP_TOUCH);
	            input_report_abs(cyttsp_data->input, ABS_MT_WIDTH_MAJOR, CYTTSP_SMALL_TOOL_WIDTH);
				input_report_abs(cyttsp_data->input, ABS_MT_POSITION_X, touchX);
	            input_report_abs(cyttsp_data->input, ABS_MT_POSITION_Y, touchY);
				CYTTSP_MT_SYNC(cyttsp_data->input);
				input_sync(cyttsp_data->input);
     	   		}
			break;

		case TOUCH_IOCTL_RELEASE_TOUCH:
			{
				int touchX=arg&0x0000FFFF;
				int touchY= (arg >> 16) & 0x0000FFFF;
				
				input_report_abs(cyttsp_data->input, ABS_MT_TOOL_TYPE , 1);
				input_report_abs(cyttsp_data->input, ABS_MT_TOUCH_MAJOR, CYTTSP_NOTOUCH);
	            input_report_abs(cyttsp_data->input, ABS_MT_WIDTH_MAJOR, CYTTSP_SMALL_TOOL_WIDTH);
				input_report_abs(cyttsp_data->input, ABS_MT_POSITION_X, touchX);
	            input_report_abs(cyttsp_data->input, ABS_MT_POSITION_Y, touchY);

				CYTTSP_MT_SYNC(cyttsp_data->input);
				input_sync(cyttsp_data->input);
			}
	       	break;    
		// ---
		case TOUCH_IOCTL_SENSOR_X:
			{
				int send_data;

				send_data = SENSOR_X;

				if (copy_to_user(argp, &send_data, sizeof(send_data)))
					return false;
			}
			break;

		case TOUCH_IOCTL_SENSOR_Y:
			{
				int send_data;

				send_data = SENSOR_Y;

				if (copy_to_user(argp, &send_data, sizeof(send_data)))
					return false;
			}
			break;

		case TOUCH_IOCTL_CHECK_BASE:
			{
				u8 send_byte[MAX_NODE];
				//printk("TOUCH_IOCTL_CHECK_BASE!!\n");

				disable_irq_nosync(ts_temp->client->irq);
				pantech_auto_check(send_byte);
				enable_irq(ts_temp->client->irq);

				if (copy_to_user(argp, send_byte, MAX_NODE))
					return false;
			}
			break;

#ifdef FEATURE_CYTTSP_FIRMWAREUPGRADE

		case TOUCH_IOCTL_READ_IC_VERSION:
			{
				int ret = 0;
				ret = firmware_version_check();
				if (copy_to_user(argp, &ret, sizeof(ret)))
					return false;
			}
			break;

		case TOUCH_IOCTL_READ_FW_VERSION:
			{
				int ret =0;

				if(bBlack == false)	// White Model etc..
					ret = CYTTPS_NONBLACK_FIRMWARE_VER_ID;
				else					// Black Model
					ret = CYTTPS_FIRMWARE_VER_ID;

				if (copy_to_user(argp, &ret, sizeof(ret)))
					return false;
			}
			break;
		case TOUCH_IOCTL_START_UPDATE:
			{
				int ret = 0;
				ret = firmware_update_by_user(); // if ret == 0 success, or not fail
				printk("TOUCH_IOCTL_START_UPDATE ret  : %d\n", ret);
				if (copy_to_user(argp, &ret, sizeof(ret)))
					return false;
			}
			break;
#endif

		case TOUCH_IOCTL_CHARGER_MODE:
			printk("TOUCH_CHARGER_MODE Setting : %d\n", (int)arg);
			Touch_ChagerMode = arg;
			break;

		case TOUCH_IOCTL_SELF_TEST:
			{
				int ret = 0;
				ret = pantech_selftest_check();
				
				//printk("Test result: %d\n", ret);
				if(ret != 0)
					return 0;
				else
					return 1;
			}
			break;
		case TOUCH_IOCTL_SET_COLOR:
			bBlack = arg;
			break;

		default:
	     		break;
	}

//	mutex_unlock(&cyttsp_mutex);

	return true;
}
#endif

void Change_Active_Distance(u8 value)
{
	int rc = -1;
	u8 byte_data;
	struct cyttsp *ts = ts_temp;
#ifdef CYTTSP_MUTEX_LOCK
		mutex_lock(&ts->lock_mutex);
#endif
	rc = i2c_smbus_read_i2c_block_data(ts->client, CYTTSP_REG_GEST_SET,sizeof(byte_data), &byte_data);

	//printk("Chage_Active_Distance : %02x\n", byte_data);

	byte_data = value;
	rc = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_GEST_SET, sizeof(byte_data), &byte_data);
#ifdef CYTTSP_MUTEX_LOCK
	mutex_unlock(&ts->lock_mutex);
#endif
	return;
}

static ssize_t cyttsp_irq_status(struct device *dev,
                                 struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev,
	                                         struct i2c_client, dev);
	struct cyttsp *ts = i2c_get_clientdata(client);
	return sprintf(buf, "%u\n", atomic_read(&ts->irq_enabled));
}

static ssize_t cyttsp_irq_enable(struct device *dev,
                                 struct device_attribute *attr,
                                 const char *buf, size_t size)
{
	struct i2c_client *client = container_of(dev,
	                                         struct i2c_client, dev);
	struct cyttsp *ts = i2c_get_clientdata(client);
	int err = 0;
	unsigned long value;
/*
	struct qtm_obj_message *msg;
*/

	if (size > 2)
		return -EINVAL;

	err = strict_strtoul(buf, 10, &value);
	if (err != 0)
		return err;

	switch (value) {
	case 0:
		if (atomic_cmpxchg(&ts->irq_enabled, 1, 0)) {
			pr_info("touch irq disabled!\n");
			disable_irq_nosync(ts->client->irq);
		}
		err = size;
		break;
	case 1:
		if (!atomic_cmpxchg(&ts->irq_enabled, 0, 1)) {
			pr_info("touch irq enabled!\n");
/*
			msg = cyttsp_read_msg(ts);
			if (msg == NULL)
				pr_err("%s: Cannot read message\n", __func__);
*/
			enable_irq(ts->client->irq);
		}
		err = size;
		break;
	default:
		pr_info("cyttsp_irq_enable failed -> irq_enabled = %d\n",
		atomic_read(&ts->irq_enabled));
		err = -EINVAL;
		break;
	}

	return err;
}

static DEVICE_ATTR(irq_enable, 0664, cyttsp_irq_status, cyttsp_irq_enable);

#if 1 //ST_LIM_120522 User 모드에서 디버깅 사용 가능하게 변경.
static ssize_t cyttsp_debug_status(struct device *dev, struct device_attribute *attr, char *buf)
{
	cyttsp_tsdebug1 ++;

	if(cyttsp_tsdebug1 >=3)
		cyttsp_tsdebug1 = 0;
	
	return sprintf(buf, "cyttsp_tsdebug1 : %d\n", cyttsp_tsdebug1);
}

static ssize_t cyttsp_debug_enable(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long value;
	int err = 0;

	err = strict_strtoul(buf, 0, &value);

	if (err)
		return err;

	if((int)value>2)
		cyttsp_tsdebug1 = 2;
	else if((int)value<0)
		cyttsp_tsdebug1 = 0;		
	else
		cyttsp_tsdebug1 = (int)value;
	
	return count;
}

static DEVICE_ATTR(debug_enable, 0664, cyttsp_debug_status, cyttsp_debug_enable);

static struct attribute *touch_attributes[] = {
	&dev_attr_debug_enable.attr,
	NULL
};

static struct attribute_group touch_attribute_group = {
	.attrs = touch_attributes
};
#endif


int pantech_ctl_update(int cmd, int value)
{
	int rt = -1;
	struct regulator *vreg_touch, *vreg_touch_temp;

	switch(cmd)
	{
		case ISSP_IOCTL_SCLK_TO_GPIO:
			if(value){
				gpio_tlmm_config(GPIO_CFG(GPIO_TOUCH_SCL, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),GPIO_CFG_ENABLE);
			}
			else{
				gpio_tlmm_config(GPIO_CFG(GPIO_TOUCH_SCL, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),GPIO_CFG_DISABLE);
				gpio_tlmm_config(GPIO_CFG(GPIO_TOUCH_SCL, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),GPIO_CFG_ENABLE);
			}
			rt = 1;
			break;

		case ISSP_IOCTL_DATA_TO_GPIO:
			if(value){
				gpio_tlmm_config(GPIO_CFG(GPIO_TOUCH_SDA, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),GPIO_CFG_ENABLE);
			}
			else{
				gpio_tlmm_config(GPIO_CFG(GPIO_TOUCH_SDA, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),GPIO_CFG_DISABLE);
				gpio_tlmm_config(GPIO_CFG(GPIO_TOUCH_SDA, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),GPIO_CFG_ENABLE);
			}
			rt = 1;
			break;

		case ISSP_IOCTL_SCLK:
			gpio_set_value(GPIO_TOUCH_SCL, value);
			rt = 1;
			break;

		case ISSP_IOCTL_DATA:
			gpio_set_value(GPIO_TOUCH_SDA, value);
			rt = 1;
			break;

		case ISSP_IOCTL_RESET:
			break;

		case ISSP_IOCTL_POWER:

			//printk("Touch Power: %d, cmd: %d\n", value, cmd);

			vreg_touch = regulator_get(NULL, "8058_l11");
			regulator_set_voltage(vreg_touch, 1900000, 1900000);
		
			if(value)
				rt = regulator_enable(vreg_touch);
			else
				rt = regulator_disable(vreg_touch);

			regulator_put(vreg_touch);
			break;

		case ISSP_IOCTL_POWER_ALL:

			//printk("Touch Power All: %d, cmd: %d\n", value, cmd);

			vreg_touch_temp = regulator_get(NULL, "8058_l19");

			vreg_touch = regulator_get(NULL, "8058_l11");
			regulator_set_voltage(vreg_touch, 1900000, 1900000);

			regulator_set_voltage(vreg_touch_temp, 3000000, 3000000);

			if(value)
			{
				rt = regulator_enable(vreg_touch);
				rt = regulator_enable(vreg_touch_temp);
			}
			else
			{
				rt = regulator_disable(vreg_touch);
				rt = regulator_disable(vreg_touch_temp);
			}

			regulator_put(vreg_touch);
			regulator_put(vreg_touch_temp);
			break;


		case ISSP_IOCTL_READ_DATA_PIN:
			rt = gpio_get_value(GPIO_TOUCH_SDA);
			break;

		case ISSP_IOCTL_WAIT:
			udelay(value);
			break;

		case ISSP_IOCTL_INTR:
			gpio_tlmm_config(GPIO_CFG(GPIO_TOUCH_CHG, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),GPIO_CFG_ENABLE);
			gpio_set_value(GPIO_TOUCH_CHG, value);
			rt = 1;
			break;

		case ISSP_TEST_READ_SCL:
			rt = gpio_get_value(GPIO_TOUCH_SCL);
			break;

		case ISSP_TEST_READ_SDA:
			rt = gpio_get_value(GPIO_TOUCH_SDA);
			break;

		case ISSP_TEST_READ_RESET:
			rt = gpio_get_value(GPIO_TOUCH_RST);
			break;

		case ISSP_COMPLITED_UPDATA:
			gpio_tlmm_config(GPIO_CFG(GPIO_TOUCH_CHG, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),GPIO_CFG_ENABLE);
			gpio_tlmm_config(GPIO_CFG(GPIO_TOUCH_SCL, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),GPIO_CFG_DISABLE);
			gpio_tlmm_config(GPIO_CFG(GPIO_TOUCH_SDA, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),GPIO_CFG_DISABLE);
			gpio_tlmm_config(GPIO_CFG(GPIO_TOUCH_SDA, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),GPIO_CFG_ENABLE);
			gpio_tlmm_config(GPIO_CFG(GPIO_TOUCH_SCL, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),GPIO_CFG_ENABLE);
			rt = 1;
			break;

		default:
			dbg("UNKNOWN CMD\n");
			break;
	}
	return rt;
}

#ifdef FEATURE_CYTTSP_FIRMWAREUPGRADE
void check_firmware_update(struct work_struct *work3)
{
	int retry_cnt = 3;
	u8 byte_data[4];
	int rc = -1, check_update_pass = 0, curr_version =0;
	struct cyttsp *ts = ts_temp;

	// If phone enter a poweroff, Stop firmware update
	if(Touch_Status >= TOUCH_POWEROFF)
		return;

#ifdef FEATURE_CYTTSP_HEARTBEAT
	start_heartbeat_timer = false;
#endif

	wake_lock(&touch_wake_lock);

	disable_irq(ts->client->irq);

#ifdef CYTTSP_MUTEX_LOCK
	mutex_lock(&ts->lock_mutex);
#endif
	do {
		rc = i2c_smbus_read_i2c_block_data(ts->client, CYTTSP_REG_READ_VER_ID,sizeof(byte_data), (u8 *)&byte_data);
		udelay(2*1000);
	} while ((rc < CYTTSP_OPERATIONAL) && --retry_cnt);

	dbg("i2c communcation1 %s, byte_data = %d, %d, %d, %d\n", (rc < CYTTSP_OPERATIONAL) ? "FAIL" : "PASS",byte_data[0],byte_data[1],byte_data[2],byte_data[3]);

	if((int)byte_data[0] == 0 || rc < CYTTSP_OPERATIONAL)
	{
		dbg("Retry read firmware version!\n");

		msleep(200);
		pantech_ctl_update(ISSP_IOCTL_POWER, 0);
		msleep(100);
		pantech_ctl_update(ISSP_IOCTL_POWER, 1);
		msleep(200);

		retry_cnt = 3;
		do {
			rc = i2c_smbus_read_i2c_block_data(ts->client, CYTTSP_REG_READ_VER_ID,sizeof(byte_data), (u8 *)&byte_data);
			udelay(2*1000);
		} while ((rc < CYTTSP_OPERATIONAL) && --retry_cnt);
	}

	pantech_ctl_update(ISSP_IOCTL_INTR,0);

#ifdef FEATURE_SKY_NONBLACK_FIRMWARE
	if(rc >= CYTTSP_OPERATIONAL) // read success
		bBlack =  (int)byte_data[0] % 2;

	if(bBlack == false)	// White Model etc..
		curr_version = CYTTPS_NONBLACK_FIRMWARE_VER_ID;
	else					// Black Model
		curr_version = CYTTPS_FIRMWARE_VER_ID;
#else
	curr_version = CYTTPS_FIRMWARE_VER_ID;
#endif

	dbg("[Touch] Model Black Check: %d, Current Version: %d\n", bBlack, curr_version);

	if(((curr_version > byte_data[0])  && (byte_data[0] != 0)) || (rc < CYTTSP_OPERATIONAL))
	{
		retry_cnt = 5;

		dbg("Start Firmware Update chip id: %d\n", byte_data[0]);

		do{
			check_update_pass = touch_update_main(bBlack);
			udelay(2*1000);
		}while((check_update_pass != 0) && --retry_cnt);
	}

#ifdef CYTTSP_MUTEX_LOCK
	mutex_unlock(&ts->lock_mutex);
#endif

	pantech_ctl_update(ISSP_IOCTL_INTR,1);
	pantech_ctl_update(ISSP_COMPLITED_UPDATA,0);

	if(check_update_pass != 0)
	{
		msleep(200);
		pantech_ctl_update(ISSP_IOCTL_POWER, 0);
		msleep(100);
		pantech_ctl_update(ISSP_IOCTL_POWER, 1);
		msleep(100);
		cancel_work_sync(&ts->work);
	}

	dbg("check_firmware_update end!!(check_update_pass = %d)\n",check_update_pass);

	enable_irq(ts->client->irq);

#ifdef FEATURE_CYTTSP_HEARTBEAT
	start_heartbeat_timer = true;
#endif

	wake_unlock(&touch_wake_lock);

	return;
}

static int firmware_version_check(void)
{
	int rc = -1, retry_cnt = 3;
	u8 byte_data[4];
	struct cyttsp *ts = ts_temp;

	if(Touch_Status >= TOUCH_POWEROFF)
	{
		pantech_ctl_update(ISSP_IOCTL_POWER_ALL , 1);
		pantech_ctl_update(ISSP_IOCTL_INTR, 1);
		pantech_ctl_update(ISSP_COMPLITED_UPDATA, 0);
		msleep(300);
//		return false;
	}
#ifdef CYTTSP_MUTEX_LOCK
		mutex_lock(&ts->lock_mutex);
#endif
	do {
		rc = i2c_smbus_read_i2c_block_data(ts->client, CYTTSP_REG_READ_VER_ID,sizeof(byte_data), (u8 *)&byte_data);
		udelay(2*1000);
	}
	while ((rc < CYTTSP_OPERATIONAL) && --retry_cnt);
#ifdef CYTTSP_MUTEX_LOCK
		mutex_unlock(&ts->lock_mutex);
#endif

	if(rc < CYTTSP_OPERATIONAL)
	{
		printk("Can't read Touch Firmware Version\n");
		return 1;
	}

	printk("Touch Firmware Update Version : %d, Current Version: %d\n" ,
		CYTTPS_FIRMWARE_VER_ID,(int)byte_data[0]);

	if(Touch_Status >= TOUCH_POWEROFF)
	{
		pantech_ctl_update(ISSP_IOCTL_POWER_ALL , 0);
	}

	return (int)byte_data[0];	// Need not
}


static int firmware_set_charger_mode(int mode)
{
	int rc = -1, retry_cnt = 3;
	u8 byte_data[4], send_byte = 0x00;
	struct cyttsp *ts = ts_temp;

#ifdef CYTTSP_MUTEX_LOCK
		mutex_lock(&ts->lock_mutex);
#endif
	do {
		rc = i2c_smbus_read_i2c_block_data(ts->client, CYTTSP_REG_CHARGER_MODE,sizeof(byte_data), (u8 *)&byte_data);
		udelay(2*1000);
	}
	while ((rc < CYTTSP_OPERATIONAL) && --retry_cnt);
#ifdef CYTTSP_MUTEX_LOCK
		mutex_unlock(&ts->lock_mutex);
#endif

	if(rc < CYTTSP_OPERATIONAL)
	{
		printk("Can't read Touch Charger Mode\n");
		return 1;
	}

	if(cyttsp_tsdebug1>1)
		printk("Touch IC Charger Mode %02x\n" ,(int)byte_data[0]);

	if(mode > BATTERY_PLUGGED_NONE)	// charger mode on
	{
		if((int)byte_data[0] != CYTTPS_CHARGER_MODE)
		{
			send_byte = CYTTPS_CHARGER_MODE;
#ifdef CYTTSP_MUTEX_LOCK
			mutex_lock(&ts->lock_mutex);
#endif
			rc = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_CHARGER_MODE, sizeof(send_byte), &send_byte);
#ifdef CYTTSP_MUTEX_LOCK
			mutex_unlock(&ts->lock_mutex);
#endif

		}
	}
	else // charger mode off
	{
		if((int)byte_data[0] != 0x00)
		{
			send_byte = 0x00;
#ifdef CYTTSP_MUTEX_LOCK
			mutex_lock(&ts->lock_mutex);
#endif
			rc = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_CHARGER_MODE, sizeof(send_byte), &send_byte);
#ifdef CYTTSP_MUTEX_LOCK
			mutex_unlock(&ts->lock_mutex);
#endif
		}
	}

	return 0;
}


static int firmware_update_by_user(void)
{
	struct cyttsp *ts = ts_temp;

	int check_update_pass = -1;

	// If phone enter a poweroff, Stop firmware update
	if(Touch_Status >= TOUCH_POWEROFF)
	{
		pantech_ctl_update(ISSP_IOCTL_POWER_ALL , 1);
		msleep(300);
//		return false;
	}

#ifdef FEATURE_CYTTSP_HEARTBEAT
	start_heartbeat_timer = false;
#endif

	Touch_Status= TOUCH_UPDATE;

	wake_lock(&touch_wake_lock);

	/*인터럽터 PIN HIGH 상태를 변경하기 위해 IRQ를 해제함 */
	disable_irq(ts->client->irq);


	/* 인트럽터 PIN HIGH 상태로 인하여 2.6V_TOUCH 전원 OFF하여도 1.2V 전압 흐르는 현상으로 Value값을 0으로 설정 */
	pantech_ctl_update(ISSP_IOCTL_INTR,0);
#ifdef CYTTSP_MUTEX_LOCK
			mutex_lock(&ts->lock_mutex);
#endif

	check_update_pass = touch_update_main(bBlack);

#ifdef CYTTSP_MUTEX_LOCK
			mutex_unlock(&ts->lock_mutex);
#endif

	pantech_ctl_update(ISSP_IOCTL_INTR,1);
	pantech_ctl_update(ISSP_COMPLITED_UPDATA,0);

	msleep(100);
	pantech_ctl_update(ISSP_IOCTL_POWER, 0);
	msleep(100);
	pantech_ctl_update(ISSP_IOCTL_POWER, 1);
	msleep(100);

	cancel_work_sync(&ts->work);

	enable_irq(ts->client->irq);

#ifdef FEATURE_CYTTSP_HEARTBEAT
	start_heartbeat_timer = true;
#endif

	wake_unlock(&touch_wake_lock);

	Touch_Status= TOUCH_POWERON;

	return check_update_pass;
}

#endif

/* The cyttsp_xy_worker function reads the XY coordinates and sends them to
 * the input layer.  It is scheduled from the interrupt (or timer).
 */
#ifdef FEATURE_TOUCH_KEY
#define CYTTSP_MENU_KEY	0x01
#define CYTTSP_BACK_KEY	0x02
#define CYTTSP_HOME_KEY	0x04
#define CYTTSP_NULL_KEY	0x08

static int key_status = 0x00;
#endif

#if 0//def FEATURE_SKY_TOUCH_DELTA_DEBUG
static u16 pre_x_data;
static u16 pre_y_data;
static u16 delta_x;
static u16 delta_y;
#endif

#ifdef FEATURE_CYTTSP_HEARTBEAT
void cyttsp_check_heartbeat(struct work_struct *work2)
{
	struct cyttsp *ts = container_of(work2,struct cyttsp,work2);

	int retry_cnt = 3;
	u8 new_heartbeart_data[4];
	int rc = -1;
	static u8 old_heartbeat_data = 0xFF;

	memset((void*)new_heartbeart_data,0x00,sizeof(new_heartbeart_data));

	if(start_heartbeat_timer == false)
		return;

#ifdef CYTTSP_MUTEX_LOCK
		mutex_lock(&ts->lock_mutex);
#endif
	do {
		/* Read Heartbeat Count */
		rc = i2c_smbus_read_i2c_block_data(ts->client, CYTTSP_REG_READ_HEARTBEAT,sizeof(new_heartbeart_data), (u8 *)&new_heartbeart_data);
	}
	while ((rc < CYTTSP_OPERATIONAL) && --retry_cnt);
#ifdef CYTTSP_MUTEX_LOCK
		mutex_unlock(&ts->lock_mutex);
#endif

	if(cyttsp_tsdebug1>1)
		printk("##### Check Count = %s, byte_data = %d, %d, %d, %d\n", (rc < CYTTSP_OPERATIONAL) ? "FAIL" : "PASS",new_heartbeart_data[0],new_heartbeart_data[1],new_heartbeart_data[2],new_heartbeart_data[3]);

	if(start_heartbeat_timer == false)
		return;

	if(rc < CYTTSP_OPERATIONAL || old_heartbeat_data == new_heartbeart_data[0])
	{
		/* I2c error or Touch Chip's heartbeat value is not change */
		disable_irq(ts->client->irq);
		pantech_ctl_update(ISSP_IOCTL_INTR,0);
		pantech_ctl_update(ISSP_IOCTL_POWER,0);
		msleep(200);
		pantech_ctl_update(ISSP_IOCTL_INTR,1);
		gpio_tlmm_config(GPIO_CFG(GPIO_TOUCH_CHG, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),GPIO_CFG_ENABLE);
		enable_irq(ts->client->irq);
		pantech_ctl_update(ISSP_IOCTL_POWER,1);

		if(cyttsp_tsdebug1>1)
			printk("HeartBeat Fail old_data = %d, new_data = %d",old_heartbeat_data, new_heartbeart_data[0]);
	}

	if(!start_heartbeat_timer)
		old_heartbeat_data = 0xFF;

	// Set Charger Mode
	firmware_set_charger_mode(Touch_ChagerMode);

	return;
}

static bool cur_touchflag[TOUCH_MAX_NUM];
#endif
int touch_mask[TOUCH_MAX_NUM];
struct cyttsp_trk cur_trk[TOUCH_MAX_NUM];
struct cyttsp_trk prev_trk[TOUCH_MAX_NUM]; // Fastfinger tapping

void cyttsp_xy_worker(struct work_struct *work)
{
	struct cyttsp *ts = container_of(work,struct cyttsp,work);

    int i;
    int retval = 0;
    u8 curr_touches = 0;
    u8 id = 0;
//    int t = 0;
//    int num_sent = 0;
//    int signal = 0;
	int tch = 0;
#ifdef FEATURE_TOUCH_KEY
	int key_relese = true;
#endif

    #if 1 // Fastfinger tapping
    u16 distance_x = 0;
    u16 distance_y = 0;

	int drumming_flag = false;
	
    #endif

	for (i=0;i<TOUCH_MAX_NUM;i++)
	{
		touch_mask[i] = -1;
	}

#ifdef CYTTSP_TOUCH_DEBUG_ENABLE
		if(cyttsp_tsdebug1>1)
			printk("[TOUCH] Start cyttsp_xy_worker\n");
#endif

	if(Touch_Status >= TOUCH_POWEROFF)
	{
#ifdef CYTTSP_TOUCH_DEBUG_ENABLE
		if(cyttsp_tsdebug1>1)
			printk("[TOUCH] TOUCH_POWEROFF\n");
#endif			
		goto exit_xy_worker;
	}

	/* get event data from CYTTSP device */
	i = CYTTSP_NUM_RETRY;

#ifdef CYTTSP_MUTEX_LOCK
		mutex_lock(&ts->lock_mutex);
#endif
	do {
		retval = i2c_smbus_read_i2c_block_data(ts->client, CYTTSP_REG_BASE,
			sizeof(struct cyttsp_gen3_xydata_t), (u8 *)&g_xy_data);
	}
	while ((retval < CYTTSP_OPERATIONAL) && --i);
#ifdef CYTTSP_MUTEX_LOCK
		mutex_unlock(&ts->lock_mutex);
#endif

	/* return immediately on failure to read device on the i2c bus */
	if (retval < CYTTSP_OPERATIONAL) {
#ifdef CYTTSP_TOUCH_DEBUG_ENABLE
		if(cyttsp_tsdebug1>1)		
			printk("exit_xy_worker 1");
#endif
		goto exit_xy_worker;
	}

	cyttsp_xdebug("TTSP worker start 2:\n");

	if ((curr_touches = GET_NUM_TOUCHES(g_xy_data.tt_stat)) > CYTTSP_NUM_MT_TOUCH_ID) {
		/* if the number of fingers on the touch surface is more than the maximum
		 * then there will be no new track information even for the orginal
		 * touches. Therefore, ignore this touch event.
		 */
#ifdef CYTTSP_TOUCH_DEBUG_ENABLE
		if(cyttsp_tsdebug1>1)
			printk("exit_xy_worker 2");
#endif
		 goto exit_xy_worker;
	}
	else if (IS_LARGE_AREA(g_xy_data.tt_stat)==1) {
		/* terminate all active tracks */
		curr_touches = CYTTSP_NOTOUCH;
		cyttsp_debug("Large object detected. Terminating active tracks\n");
	}

#ifdef FEATURE_TOUCH_KEY

        for(i=0; i<curr_touches; i++)
        {
            int x =0, y=0;

            switch(i)
            {
                case 0:
                    x = be16_to_cpu(g_xy_data.x1);
                    y = be16_to_cpu(g_xy_data.y1);
                    break;
                case 1:
                    x = be16_to_cpu(g_xy_data.x2);
                    y = be16_to_cpu(g_xy_data.y2);
                    break;
                case 2:
                    x = be16_to_cpu(g_xy_data.x3);
                    y = be16_to_cpu(g_xy_data.y3);
                    break;
                case 3:
                    x = be16_to_cpu(g_xy_data.x4);
                    y = be16_to_cpu(g_xy_data.y4);
                    break;
                default:
                    break;
            }

            if(y > Y_MAX)
            {
                key_relese = false;

                if(y < NULL_KEY_AREA && (!key_status || key_status == CYTTSP_NULL_KEY))
                {
                    key_status = CYTTSP_NULL_KEY;
                    dbg("Down TOUCH NULL\n");
                }
                else if((MENU_KEY_MIN < x &&  x < MENU_KEY_MAX) && (!key_status || key_status == CYTTSP_MENU_KEY))
                {
                    key_status = CYTTSP_MENU_KEY;
                    input_report_key(ts->input, KEY_MENU, CYTTSP_TOUCH);
                    dbg("Down TOUCH MENU\n");
                    input_sync(ts->input);
                }
                else if((HOME_KEY_MIN < x &&  x < HOME_KEY_MAX) && (!key_status || key_status == CYTTSP_HOME_KEY))
                {
                    key_status = CYTTSP_HOME_KEY;
                    input_report_key(ts->input, KEY_HOME, CYTTSP_TOUCH);
                    dbg("Down TOUCH HOME\n");
                    input_sync(ts->input);
                }
                else if((BACK_KEY_MIN < x &&  x < BACK_KEY_MAX) && (!key_status || key_status == CYTTSP_BACK_KEY))
                {
                    key_status = CYTTSP_BACK_KEY;
                    input_report_key(ts->input, KEY_BACK, CYTTSP_TOUCH);
                    dbg("Down TOUCH BACK\n");
                    input_sync(ts->input);
                }
                else if(!key_status)
                    key_status = CYTTSP_NULL_KEY;
            }
        }

        if(key_relese && (curr_touches < prev_touches) && key_status)
        {
            if(key_status == CYTTSP_MENU_KEY)
                input_report_key(ts->input, KEY_MENU, CYTTSP_NOTOUCH);

            if(key_status == CYTTSP_HOME_KEY)
                input_report_key(ts->input, KEY_HOME, CYTTSP_NOTOUCH);

            if(key_status == CYTTSP_BACK_KEY)
                input_report_key(ts->input, KEY_BACK, CYTTSP_NOTOUCH);

            if(key_status != CYTTSP_NULL_KEY)
                input_sync(ts->input);

            dbg("Up Key: %02x\n", key_status);

            key_status = 0;
        }

#endif

	/* send no events if there were no previous touches and no new touches */
	if ((prev_touches == CYTTSP_NOTOUCH) &&
		((curr_touches == CYTTSP_NOTOUCH) || (curr_touches > CYTTSP_NUM_MT_TOUCH_ID))) {
#ifdef CYTTSP_TOUCH_DEBUG_ENABLE
		if(cyttsp_tsdebug1>1)
			printk("exit_xy_worker 3");
#endif		
		goto exit_xy_worker;
	}

	cyttsp_debug("prev=%d  curr=%d\n", prev_touches, curr_touches);

    dbg("cur#: %d, tch : 0x%x\n", curr_touches,tch);
	dbg("touch12_id : 0x%x, touch34_id : 0x%x\n", g_xy_data.touch12_id,g_xy_data.touch34_id);

    /* extract xy_data for all currently reported touches */
	for (tch = 0; tch < curr_touches; tch++)
    {
        if (tch < 2)
        {
            id = (tch & 0x01) ?
                            GET_TOUCH2_ID(g_xy_data.touch12_id) : //LO_TRACKID(*(ts->tch_map[tch].id)) :
                            GET_TOUCH1_ID(g_xy_data.touch12_id); //HI_TRACKID(*(ts->tch_map[tch].id));

//		touch_mask[id] = tch;

            if (tch == 0)
            {
                if(id < TOUCH_MAX_NUM){
                    cur_trk[id].tch = CYTTSP_TOUCH;
                    cur_trk[id].abs[CY_ABS_X_OST] = be16_to_cpu(g_xy_data.x1);
                    cur_trk[id].abs[CY_ABS_Y_OST] = be16_to_cpu(g_xy_data.y1);
                    cur_trk[id].abs[CY_ABS_P_OST] = g_xy_data.z1;
                    cur_trk[id].abs[CY_ABS_W_OST] = CYTTSP_SMALL_TOOL_WIDTH;
                    cur_trk[id].abs[CY_ABS_ID_OST] = id;
                }else{
                	if(Touch_Dbg_Enable)
	                printk("over Max touch ID id#: %d !!!\n", id);
                }

                #if 1 // Fastfinger tapping
                if((prev_touches == curr_touches)&&(prev_trk[id].abs[CY_ABS_ID_OST] == cur_trk[id].abs[CY_ABS_ID_OST]))
                {
                    distance_x = ABS(prev_trk[id].abs[CY_ABS_X_OST],cur_trk[id].abs[CY_ABS_X_OST]);
                    distance_y = ABS(prev_trk[id].abs[CY_ABS_Y_OST],cur_trk[id].abs[CY_ABS_Y_OST]);
                    if( ((distance_x * distance_x)+(distance_y * distance_y)) > FINGER_SEP_SQRT_XY )
                    {
	                    		if(Touch_Dbg_Enable)
					printk("drumming release id: %d[%d,%d] \n", id, cur_trk[id].abs[CY_ABS_X_OST], cur_trk[id].abs[CY_ABS_Y_OST]);
					//cur_trk[id].abs[CY_ABS_ID_OST] = -1;
					//cur_touchflag[id] = 0;	
                        		//input_mt_slot(ts->input, id);
                        		//input_report_abs(ts->input, ABS_MT_TRACKING_ID, -1);
                        		//input_sync(ts->input);
                        		drumming_flag = true;
                    }
                }
                #endif
            }
            if (tch == 1)
            {
                if(id < TOUCH_MAX_NUM){
                    cur_trk[id].tch = CYTTSP_TOUCH;
                    cur_trk[id].abs[CY_ABS_X_OST] = be16_to_cpu(g_xy_data.x2);
                    cur_trk[id].abs[CY_ABS_Y_OST] = be16_to_cpu(g_xy_data.y2);
                    cur_trk[id].abs[CY_ABS_P_OST] = g_xy_data.z2;
                    cur_trk[id].abs[CY_ABS_W_OST] = CYTTSP_SMALL_TOOL_WIDTH;
                    cur_trk[id].abs[CY_ABS_ID_OST] = id;
                }else{
                	if(Touch_Dbg_Enable)
	                printk("over Max touch ID id#: %d !!!\n", id);
                }
            }
        }
        else
        {
            id = (tch & 0x01) ?
                            GET_TOUCH4_ID(g_xy_data.touch34_id) : //LO_TRACKID(*(ts->tch_map[tch].id)) :
                            GET_TOUCH3_ID(g_xy_data.touch34_id); //HI_TRACKID(*(ts->tch_map[tch].id));

            if (tch == 2)
            {
                if(id < TOUCH_MAX_NUM){
                    cur_trk[id].tch = CYTTSP_TOUCH;
                    cur_trk[id].abs[CY_ABS_X_OST] = be16_to_cpu(g_xy_data.x3);
                    cur_trk[id].abs[CY_ABS_Y_OST] = be16_to_cpu(g_xy_data.y3);
                    cur_trk[id].abs[CY_ABS_P_OST] = g_xy_data.z3;
                    cur_trk[id].abs[CY_ABS_W_OST] = CYTTSP_SMALL_TOOL_WIDTH;
                    cur_trk[id].abs[CY_ABS_ID_OST] = id;
                }else{
                	if(Touch_Dbg_Enable)
	                printk("over Max touch ID id#: %d !!!\n", id);
                }
            }

            if (tch == 3)
            {
                if(id < TOUCH_MAX_NUM){
                    cur_trk[id].tch = CYTTSP_TOUCH;
                    cur_trk[id].abs[CY_ABS_X_OST] = be16_to_cpu(g_xy_data.x4);
                    cur_trk[id].abs[CY_ABS_Y_OST] = be16_to_cpu(g_xy_data.y4);
                    cur_trk[id].abs[CY_ABS_P_OST] = g_xy_data.z4;
                    cur_trk[id].abs[CY_ABS_W_OST] = CYTTSP_SMALL_TOOL_WIDTH;
                    cur_trk[id].abs[CY_ABS_ID_OST] = id;
                }else{
                	if(Touch_Dbg_Enable)
	                printk("over Max touch ID id#: %d !!!\n", id);
                }
            }
        }

        if(id < TOUCH_MAX_NUM){
            touch_mask[id] = tch;
        }else{
            if(Touch_Dbg_Enable)
                printk("over Max touch ID id#: %d !!!\n", id);
        }

        dbg("tch#: %d, ID: %d, Xpos: %d, Ypos: %d\n",
                      tch, id, cur_trk[id].abs[CY_ABS_X_OST], cur_trk[id].abs[CY_ABS_Y_OST]);
	}

	if(drumming_flag == true)
		curr_touches--;

    // Release Event
	if ( curr_touches == 0 )
    {
        dbg("Touch Released\n");

		for (i=0; i < TOUCH_MAX_NUM; i++)
		{
			if (cur_touchflag[i] /*cur_trk[i].abs[CY_ABS_ID_OST] >= 0*/)
            {
				cur_trk[i].abs[CY_ABS_ID_OST] = -1;
				cur_touchflag[i] = 0;
			    input_mt_slot(ts->input, i);
				input_report_abs(ts->input, ABS_MT_TRACKING_ID, -1);
				if(Touch_Dbg_Enable)
				printk("Touch Released 1, I : %d\n",i);
			}
		}
	}
    // Finger Touched
	else
    {
        dbg("Touch Pressed\n");

		for (i=0; i<TOUCH_MAX_NUM; i++)
        {
            if ( touch_mask[i] < 0 )    // 1 case - the 1st finger : touched / the 2nd finger : released
            {
#if 1
                if ( cur_trk[i].abs[CY_ABS_ID_OST] >= 0 )
                {
                    cur_touchflag[cur_trk[i].abs[CY_ABS_ID_OST]] = 0;
                    cur_trk[i].abs[CY_ABS_ID_OST] = -1;
                    input_mt_slot(ts->input, i);
                    input_report_abs(ts->input, ABS_MT_TRACKING_ID, cur_trk[i].abs[CY_ABS_ID_OST]);
					if(Touch_Dbg_Enable)		
					printk("[Touch UP] ID: %d\n", i);
                }
#endif
            }
            else
            {
				if(touch_mask[i] >= 0)  // if finger is touched
				{
	                input_mt_slot(ts->input, i);

	                if ( (cur_touchflag[cur_trk[i].abs[CY_ABS_ID_OST]] == 0) )
	                {
			                	if(Touch_Dbg_Enable)
			              	printk("Touch Pressed 2-1 I:%d, touchflag : %d\n",i,cur_touchflag[i]);
	                    cur_touchflag[cur_trk[i].abs[CY_ABS_ID_OST]] = 1;
	                    //cur_trk[i].abs[CY_ABS_ID_OST] = input_mt_new_trkid(ts->input);
		                    		//input_report_abs(ts->input, ABS_MT_TRACKING_ID, cur_trk[i].abs[CY_ABS_ID_OST]);
	                }

	                // Move Event
			  input_report_abs(ts->input, ABS_MT_TRACKING_ID, cur_trk[i].abs[CY_ABS_ID_OST]);
	                input_report_abs(ts->input, ABS_MT_POSITION_X, cur_trk[i].abs[CY_ABS_X_OST]);
	                input_report_abs(ts->input, ABS_MT_POSITION_Y, cur_trk[i].abs[CY_ABS_Y_OST]);
	                input_report_abs(ts->input, ABS_MT_PRESSURE, cur_trk[i].abs[CY_ABS_P_OST]);
	                input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, 1);
	                input_report_abs(ts->input, ABS_MT_WIDTH_MAJOR, cur_trk[i].abs[CY_ABS_P_OST]);

					if(Touch_Dbg_Enable)
					printk("[Touch Down] ID: %d [%d,%d]\n", i, cur_trk[i].abs[CY_ABS_X_OST], cur_trk[i].abs[CY_ABS_Y_OST]);
				}
            }
        }
	}
#ifdef CYTTSP_TOUCH_DEBUG_ENABLE
	if(cyttsp_tsdebug1)
	{
		printk("[T]c:%d,m[%d][%d][%d][%d],f[%d][%d][%d][%d],1[%d][%d][%d],2[%d][%d][%d],3[%d][%d][%d],4[%d][%d][%d]\n",
				curr_touches,
				touch_mask[0],touch_mask[1],touch_mask[2],touch_mask[3],
				cur_touchflag[0],cur_touchflag[1],cur_touchflag[2],cur_touchflag[3],		     
				cur_trk[0].abs[CY_ABS_ID_OST], cur_trk[0].abs[CY_ABS_X_OST],cur_trk[0].abs[CY_ABS_Y_OST],
				cur_trk[1].abs[CY_ABS_ID_OST], cur_trk[1].abs[CY_ABS_X_OST],cur_trk[1].abs[CY_ABS_Y_OST],
				cur_trk[2].abs[CY_ABS_ID_OST], cur_trk[2].abs[CY_ABS_X_OST],cur_trk[2].abs[CY_ABS_Y_OST],
				cur_trk[3].abs[CY_ABS_ID_OST], cur_trk[3].abs[CY_ABS_X_OST],cur_trk[3].abs[CY_ABS_Y_OST]);
	}
#endif
	input_report_key(ts->input, BTN_TOUCH, (curr_touches>0)? 1:0 );
	input_sync(ts->input);

	// update previous touch num
    prev_touches = curr_touches;

#if 1 // Fastfinger tapping
    for(i=0; i<TOUCH_MAX_NUM ; i++)
    {
        prev_trk[i] = cur_trk[i];
    }
#endif
    goto exit_xy_worker;

#if 0
    /* provide input event signaling for each active touch */
    for (id = 0, num_sent = 0; id < CYTTSP_NUM_TRACK_ID; id++)
    {
        if (cur_trk[id].tch)
        {
            t = cur_trk[id].abs[CY_ABS_ID_OST];

            /* send 0 based track id's */
            t -= 1;
            input_report_abs(ts->input, ABS_MT_TRACKING_ID, t);
            input_report_abs(ts->input, ABS_MT_POSITION_X, cur_trk[id].abs[CY_ABS_X_OST]);
            input_report_abs(ts->input, ABS_MT_POSITION_Y, cur_trk[id].abs[CY_ABS_Y_OST]);
            input_report_abs(ts->input, ABS_MT_PRESSURE, cur_trk[id].abs[CY_ABS_P_OST]);
            input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, cur_trk[id].abs[CY_ABS_W_OST]);

            num_sent++;
            input_mt_sync(ts->input);

            ts->prv_trk[id] = cur_trk[id];
            ts->prv_trk[id].abs[CY_ABS_ID_OST] = t;
#if 0
            cyttsp_dbg(ts, CY_DBG_LVL_1, "%s: ID:%3d  X:%3d  Y:%3d  " "Z:%3d  W=%3d  T=%3d\n", __func__, id,
            cur_trk[id].abs[CY_ABS_X_OST],
            cur_trk[id].abs[CY_ABS_Y_OST],
            cur_trk[id].abs[CY_ABS_P_OST],
            cur_trk[id].abs[CY_ABS_W_OST],
            t);
#endif
        }
        else if ((ABS_MT_PRESSURE == ABS_MT_TOUCH_MAJOR) && ts->prv_trk[id].tch)
        {
            /*
            * pre-Gingerbread:
            * need to report last position with
            * and report that position with
            * no touch if the touch lifts off
            */

            input_report_abs(ts->input, ABS_MT_TRACKING_ID, ts->prv_trk[id].abs[CY_ABS_ID_OST]);
            input_report_abs(ts->input, ABS_MT_POSITION_X, ts->prv_trk[id].abs[CY_ABS_X_OST]);
            input_report_abs(ts->input, ABS_MT_POSITION_Y, ts->prv_trk[id].abs[CY_ABS_Y_OST]);
            input_report_abs(ts->input, ABS_MT_PRESSURE, CYTTSP_NOTOUCH);
            input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, CYTTSP_NOTOUCH);

            num_sent++;
            input_mt_sync(ts->input);
            ts->prv_trk[id].tch = CYTTSP_NOTOUCH;

#if 0
            cyttsp_dbg(ts, CY_DBG_LVL_1,
            "%s: ID:%3d  X:%3d  Y:%3d  "
            "Z:%3d  W=%3d  T=%3d liftoff\n",
            __func__, ts->prv_trk[id].abs[CY_ABS_ID_OST],
            ts->prv_trk[id].abs[CY_ABS_X_OST],
            ts->prv_trk[id].abs[CY_ABS_Y_OST],
            CYTTSP_NOTOUCH,
            CYTTSP_NOTOUCH,
            ts->prv_trk[id].abs[CY_ABS_ID_OST]);
#endif
        }
    }


	if (num_sent == 0)
    {
        /* in case of 0-touch; all liftoff; Gingerbread+ */
        input_mt_sync(ts->input);
    }

	input_sync(ts->input);

    // update previous touch num
    prev_touches = curr_touches;

	goto exit_xy_worker;
#endif

exit_xy_worker:

#ifdef CYTTSP_TOUCH_DEBUG_ENABLE
	if(cyttsp_tsdebug1>1)
	{
		printk("[TOUCH] disable_touch = %d, irq = %d\n",cyttsp_disable_touch, ts->client->irq);
	}
#endif	

#if 0 //ST_LIM_120518 사용하는 곳은 없으나 간흘적으로 cyttsp_disable_touch가 되는 경우가 발생되어 인트럽터 동작되지 않음.
	if(cyttsp_disable_touch) {
		cyttsp_debug("Not enabling touch\n");
	}
	else 
#endif		
	{
		if(ts->client->irq == 0) {
			/* restart event timer */
			mod_timer(&ts->timer, jiffies + TOUCHSCREEN_TIMEOUT);
		}
		else {
			/* re-enable the interrupt after processing */
			enable_irq(ts->client->irq);
		}
	}
	return;
}
#if 0
static int cyttsp_inlist(u16 prev_track[], u8 curr_track_id, u8 *prev_loc, u8 num_touches)
{
	u8 id =0;

	*prev_loc = CYTTSP_IGNORE_TOUCH;

		cyttsp_xdebug("IN p[%d]=%d c=%d n=%d loc=%d\n", \
			id, prev_track[id], curr_track_id, num_touches, *prev_loc);
	for (id = 0, *prev_loc = CYTTSP_IGNORE_TOUCH;
		(id < num_touches); id++) {
		cyttsp_xdebug("p[%d]=%d c=%d n=%d loc=%d\n", \
			id, prev_track[id], curr_track_id, num_touches, *prev_loc);
		if (prev_track[id] == curr_track_id) {
			*prev_loc = id;
			break;
		}
	}
		cyttsp_xdebug("OUT p[%d]=%d c=%d n=%d loc=%d\n", \
			id, prev_track[id], curr_track_id, num_touches, *prev_loc);

	return ((*prev_loc < CYTTSP_NUM_TRACK_ID) ? true : false);
}

static int cyttsp_next_avail_inlist(u16 curr_track[], u8 *new_loc, u8 num_touches)
{
	u8 id;

	for (id = 0, *new_loc = CYTTSP_IGNORE_TOUCH;
		(id < num_touches); id++) {
		if (curr_track[id] > CYTTSP_NUM_TRACK_ID) {
			*new_loc = id;
			break;
		}
	}

	return ((*new_loc < CYTTSP_NUM_TRACK_ID) ? true : false);
}
#endif
/* Timer function used as dummy interrupt driver */
static void cyttsp_timer(unsigned long handle)
{
	struct cyttsp *ts = (struct cyttsp *) handle;

	cyttsp_xdebug("TTSP Device timer event\n");
 #ifdef FEATURE_CYTTSP_HEARTBEAT
	/* schedule motion signal handling */
	if(start_heartbeat_timer)
	{
		schedule_work(&ts->work2);
		mod_timer(&ts->timer, jiffies + CYTTSP_HEARTBEAT_TIME * HZ);
	}

#else
	/* schedule motion signal handling */
	schedule_work(&ts->work);
#endif
	return;
}

/* ************************************************************************
 * ISR function. This function is general, initialized in drivers init
 * function
 * ************************************************************************ */
static irqreturn_t cyttsp_irq(int irq, void *handle)
{
	struct cyttsp *ts = (struct cyttsp *) handle;

	cyttsp_xdebug("%s: Got IRQ\n", CYTTSP_I2C_NAME);

	if(Touch_Status >= TOUCH_POWEROFF)
		return IRQ_HANDLED;

	/* disable further interrupts until this interrupt is processed */
	disable_irq_nosync(ts->client->irq);

	/* schedule motion signal handling */
	schedule_work(&ts->work);
	return IRQ_HANDLED;
}

#ifdef CYTTSP_INCLUDE_LOAD_FILE	//[BIH] ICS port...
/* ************************************************************************
 * Probe initialization functions
 * ************************************************************************ */
static int cyttsp_putbl(struct cyttsp *ts, int show, int show_status, int show_version, int show_cid)
{
	int retval = CYTTSP_OPERATIONAL;

	int num_bytes = (show_status * 3) + (show_version * 6) + (show_cid * 3);

	if (show_cid) {
		num_bytes = sizeof(struct cyttsp_bootloader_data_t);
	}
	else if (show_version) {
		num_bytes = sizeof(struct cyttsp_bootloader_data_t) - 3;
	}
	else {
		num_bytes = sizeof(struct cyttsp_bootloader_data_t) - 9;
	}

	if (show) {
#ifdef CYTTSP_MUTEX_LOCK
		mutex_lock(&ts->lock_mutex);
#endif
		retval = i2c_smbus_read_i2c_block_data(ts->client, CYTTSP_REG_BASE,
			num_bytes, (u8 *)&g_bl_data);
#ifdef CYTTSP_MUTEX_LOCK
		mutex_unlock(&ts->lock_mutex);
#endif

		if (show_status) {
			cyttsp_debug("BL%d: f=%02X s=%02X err=%02X bl=%02X%02X bld=%02X%02X\n", \
				show, \
				g_bl_data.bl_file, g_bl_data.bl_status, g_bl_data.bl_error, \
				g_bl_data.blver_hi, g_bl_data.blver_lo, \
				g_bl_data.bld_blver_hi, g_bl_data.bld_blver_lo);
		}
		if (show_version) {
			cyttsp_debug("BL%d: ttspver=0x%02X%02X appid=0x%02X%02X appver=0x%02X%02X\n", \
				show, \
				g_bl_data.ttspver_hi, g_bl_data.ttspver_lo, \
				g_bl_data.appid_hi, g_bl_data.appid_lo, \
				g_bl_data.appver_hi, g_bl_data.appver_lo);
		}
		if (show_cid) {
			cyttsp_debug("BL%d: cid=0x%02X%02X%02X\n", \
				show, \
				g_bl_data.cid_0, g_bl_data.cid_1, g_bl_data.cid_2);
		}
		mdelay(CYTTSP_DELAY_DFLT);
	}

	return retval;
}
#endif //CYTTSP_INCLUDE_LOAD_FILE	//[BIH] ICS port...

#ifdef CYTTSP_INCLUDE_LOAD_FILE
#define CYTTSP_MAX_I2C_LEN	256
#define CYTTSP_MAX_TRY		10
#define CYTTSP_BL_PAGE_SIZE	16
#define CYTTSP_BL_NUM_PAGES	5
static int cyttsp_i2c_write_block_data(struct i2c_client *client, u8 command,
			       u8 length, const u8 *values)
{
	int retval = CYTTSP_OPERATIONAL;

	u8 dataray[CYTTSP_MAX_I2C_LEN];
	u8 try;
	dataray[0] = command;
	if (length) {
		memcpy(&dataray[1], values, length);
	}

	try = CYTTSP_MAX_TRY;
	do {
		retval = i2c_master_send(client, dataray, length+1);
		mdelay(CYTTSP_DELAY_DFLT*2);
	}
	while ((retval != length+1) && try--);

	return retval;
}

static int cyttsp_i2c_write_block_data_chunks(struct cyttsp *ts, u8 command,
			       u8 length, const u8 *values)
{
	int retval = CYTTSP_OPERATIONAL;
	int block = 1;

	u8 dataray[CYTTSP_MAX_I2C_LEN];

	/* first page already includes the bl page offset */
#ifdef CYTTSP_MUTEX_LOCK
	mutex_lock(&ts->lock_mutex);
#endif
	retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_BASE,
		CYTTSP_BL_PAGE_SIZE+1, values);

	mdelay(10);
	values += CYTTSP_BL_PAGE_SIZE+1;
	length -= CYTTSP_BL_PAGE_SIZE+1;

	/* rem blocks require bl page offset stuffing */
	while (length && (block < CYTTSP_BL_NUM_PAGES) && !(retval < CYTTSP_OPERATIONAL)) {
		dataray[0] = CYTTSP_BL_PAGE_SIZE*block;
		memcpy(&dataray[1], values,
			length >= CYTTSP_BL_PAGE_SIZE ? CYTTSP_BL_PAGE_SIZE : length);
		retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_BASE,
			length >= CYTTSP_BL_PAGE_SIZE ? CYTTSP_BL_PAGE_SIZE+1 : length+1, dataray);
		mdelay(10);
		values += CYTTSP_BL_PAGE_SIZE;
		length = length >= CYTTSP_BL_PAGE_SIZE ? length - CYTTSP_BL_PAGE_SIZE : 0;
		block++;
	}
#ifdef CYTTSP_MUTEX_LOCK
			mutex_unlock(&ts->lock_mutex);
#endif

	return retval;
}

static int cyttsp_bootload_app(struct cyttsp *ts)
{
	int retval = CYTTSP_OPERATIONAL;
	int i, tries;
	u8 host_reg;

	cyttsp_debug("load new firmware \n");

#ifdef CYTTSP_MUTEX_LOCK
		mutex_lock(&ts->lock_mutex);
#endif

	/* reset TTSP Device back to bootloader mode */
	host_reg = CYTTSP_SOFT_RESET_MODE;
	retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_BASE,
		sizeof(host_reg), &host_reg);

	/* wait for TTSP Device to complete reset back to bootloader */
//	mdelay(CYTTSP_DELAY_DFLT);
	mdelay(1000);
	cyttsp_putbl(ts,3, true, true, true);
	cyttsp_debug("load file -- tts_ver=0x%02X%02X  app_id=0x%02X%02X  app_ver=0x%02X%02X\n", \
		cyttsp_fw_tts_verh, cyttsp_fw_tts_verl, \
		cyttsp_fw_app_idh, cyttsp_fw_app_idl, \
		cyttsp_fw_app_verh, cyttsp_fw_app_verl);

	/* download new TTSP Application to the Bootloader
	 *
	 */
	if (!(retval < CYTTSP_OPERATIONAL)) {
		i = 0;
		/* send bootload initiation command */
		if (cyttsp_fw[i].Command == CYTTSP_BL_INIT_LOAD) {
			g_bl_data.bl_file = 0;
			g_bl_data.bl_status = 0;
			g_bl_data.bl_error = 0;

			retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_BASE,
				cyttsp_fw[i].Length, cyttsp_fw[i].Block);

			/* delay to allow bootloader to get ready for block writes */
			i++;
			tries = 0;
			cyttsp_debug("wait init f=%02X, s=%02X, e=%02X t=%d\n",g_bl_data.bl_file,
				g_bl_data.bl_status, g_bl_data.bl_error, tries);
			do {
				mdelay(1000);
				cyttsp_putbl(ts,4, true, false, false);
			}
			while (g_bl_data.bl_status != 0x10 &&
				g_bl_data.bl_status != 0x11 &&
				tries++ < 10);
			/* send bootload firmware load blocks -
			 * kernel limits transfers to I2C_SMBUS_BLOCK_MAX(32) bytes
			 */
			if (!(retval < CYTTSP_OPERATIONAL)) {
				while (cyttsp_fw[i].Command == CYTTSP_BL_WRITE_BLK) {
					retval = cyttsp_i2c_write_block_data_chunks(ts,
						CYTTSP_REG_BASE,
						cyttsp_fw[i].Length, cyttsp_fw[i].Block);
//					if (cyttsp_fw[i].Address & 0x01) {
//						mdelay(CYTTSP_DELAY_DNLOAD);
//					}
//					else {
//						mdelay(CYTTSP_DELAY_DNLOAD);
//					}
					/* bootloader requires delay after odd block addresses */
					mdelay(100);
					cyttsp_debug("BL DNLD Rec=% 3d Len=% 3d Addr=%04X\n",
						cyttsp_fw[i].Record, cyttsp_fw[i].Length,
						cyttsp_fw[i].Address);
					i++;
					if (retval < CYTTSP_OPERATIONAL) {
						cyttsp_debug("BL fail Rec=%3d retval=%d\n",cyttsp_fw[i-1].Record, retval);
						break;
					}
					else {
						/* reset TTSP I2C counter */
						retval = cyttsp_i2c_write_block_data(ts->client,
							CYTTSP_REG_BASE,
							0, NULL);
						mdelay(10);
						/* set arg2 to non-0 to activate */
						cyttsp_putbl(ts,5, true, false, false);
					}
				}
				if (!(retval < CYTTSP_OPERATIONAL)) {
					while (i < cyttsp_fw_records) {
						retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_BASE,
							cyttsp_fw[i].Length, cyttsp_fw[i].Block);
						i++;
						tries = 0;
						cyttsp_debug("wait init f=%02X, s=%02X, e=%02X t=%d\n",g_bl_data.bl_file,
							g_bl_data.bl_status, g_bl_data.bl_error, tries);
						do {
							mdelay(1000);
							cyttsp_putbl(ts,6, true, false, false);
						}
						while (g_bl_data.bl_status != 0x10 &&
							g_bl_data.bl_status != 0x11 &&
							tries++ < 10);
						cyttsp_putbl(ts,7, true, false, false);
						if (retval < CYTTSP_OPERATIONAL) {
							break;
						}
					}
				}
			}
		}
	}

	/* Do we need to reset TTSP Device back to bootloader mode?? */
	/*
	*/
	host_reg = CYTTSP_SOFT_RESET_MODE;
	retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_BASE,
		sizeof(host_reg), &host_reg);
	/* wait for TTSP Device to complete reset back to bootloader */
	/*
	*/
	mdelay(1000);

#ifdef CYTTSP_MUTEX_LOCK
		mutex_unlock(&ts->lock_mutex);
#endif

	/* set arg2 to non-0 to activate */
	retval = cyttsp_putbl(ts, 8, true, true, true);

	return retval;
}
#else
#if 0
static int cyttsp_bootload_app(struct cyttsp *ts)
{
	cyttsp_debug("no-load new firmware \n");
	return CYTTSP_OPERATIONAL;
}
#endif
#endif /* CYTTSP_INCLUDE_LOAD_FILE */

#if 0
static int cyttsp_power_on(struct cyttsp *ts)
{
	int retval = CYTTSP_OPERATIONAL;
	u8 host_reg;
	int tries;
	static u8 bl_cmd[] = {
		CYTTSP_BL_FILE0, CYTTSP_BL_CMD, CYTTSP_BL_EXIT,
		CYTTSP_BL_KEY0, CYTTSP_BL_KEY1, CYTTSP_BL_KEY2,
		CYTTSP_BL_KEY3, CYTTSP_BL_KEY4, CYTTSP_BL_KEY5,
		CYTTSP_BL_KEY6, CYTTSP_BL_KEY7};

	cyttsp_debug("Power up \n");

	/* check if the TTSP device has a bootloader installed */
	host_reg = CYTTSP_SOFT_RESET_MODE;
	retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_BASE,
		sizeof(host_reg), &host_reg);

	tries = 0;
	do {
		mdelay(1000);

		/* set arg2 to non-0 to activate */
		retval = cyttsp_putbl(ts, 1, true, true, true);

		cyttsp_info("BL%d: f=%02X s=%02X err=%02X bl=%02X%02X bld=%02X%02X R=%d\n", \
			101, \
			g_bl_data.bl_file, g_bl_data.bl_status, g_bl_data.bl_error, \
			g_bl_data.blver_hi, g_bl_data.blver_lo, \
			g_bl_data.bld_blver_hi, g_bl_data.bld_blver_lo,
			retval);
		cyttsp_info("BL%d: tver=%02X%02X a_id=%02X%02X aver=%02X%02X\n", \
			102, \
			g_bl_data.ttspver_hi, g_bl_data.ttspver_lo, \
			g_bl_data.appid_hi, g_bl_data.appid_lo, \
			g_bl_data.appver_hi, g_bl_data.appver_lo);
		cyttsp_info("BL%d: c_id=%02X%02X%02X\n", \
			103, \
			g_bl_data.cid_0, g_bl_data.cid_1, g_bl_data.cid_2);
	}
	while (!(retval < CYTTSP_OPERATIONAL) &&
		!GET_BOOTLOADERMODE(g_bl_data.bl_status) &&
		!(g_bl_data.bl_file == CYTTSP_OPERATE_MODE + CYTTSP_LOW_POWER_MODE) &&
		tries++ < 10);

	/* is bootloader missing? */
	if (!(retval < CYTTSP_OPERATIONAL)) {
		cyttsp_xdebug("Retval=%d  Check if bootloader is missing...\n", retval);
		if (!GET_BOOTLOADERMODE(g_bl_data.bl_status)) {
			/* skip all bootloader and sys info and go straight to operational mode */
			if (!(retval < CYTTSP_OPERATIONAL)) {
				cyttsp_xdebug("Bootloader is missing (retval = %d)\n", retval);
				host_reg = CYTTSP_OPERATE_MODE/* + CYTTSP_LOW_POWER_MODE*/;
				retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_BASE,
					sizeof(host_reg), &host_reg);
				/* wait for TTSP Device to complete switch to Operational mode */
				mdelay(1000);
				goto bypass;
			}
		}
	}


	/* take TTSP out of bootloader mode; go to TrueTouch operational mode */
	if (!(retval < CYTTSP_OPERATIONAL)) {
		cyttsp_xdebug1("exit bootloader; go operational\n");
		retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_BASE,
			sizeof(bl_cmd), bl_cmd);
		tries = 0;
		do {
			mdelay(1000);
			cyttsp_putbl(ts,4, true, false, false);
			cyttsp_info("BL%d: f=%02X s=%02X err=%02X bl=%02X%02X bld=%02X%02X\n", \
				104, \
				g_bl_data.bl_file, g_bl_data.bl_status, g_bl_data.bl_error, \
				g_bl_data.blver_hi, g_bl_data.blver_lo, \
				g_bl_data.bld_blver_hi, g_bl_data.bld_blver_lo);
		}
		while (GET_BOOTLOADERMODE(g_bl_data.bl_status) &&
			tries++ < 10);
	}



	if (!(retval < CYTTSP_OPERATIONAL) &&
		cyttsp_app_load()) {
		mdelay(1000);
		if (CYTTSP_DIFF(g_bl_data.ttspver_hi, cyttsp_tts_verh())  ||
			CYTTSP_DIFF(g_bl_data.ttspver_lo, cyttsp_tts_verl())  ||
			CYTTSP_DIFF(g_bl_data.appid_hi, cyttsp_app_idh())  ||
			CYTTSP_DIFF(g_bl_data.appid_lo, cyttsp_app_idl())  ||
			CYTTSP_DIFF(g_bl_data.appver_hi, cyttsp_app_verh())  ||
			CYTTSP_DIFF(g_bl_data.appver_lo, cyttsp_app_verl())  ||
			CYTTSP_DIFF(g_bl_data.cid_0, cyttsp_cid_0())  ||
			CYTTSP_DIFF(g_bl_data.cid_1, cyttsp_cid_1())  ||
			CYTTSP_DIFF(g_bl_data.cid_2, cyttsp_cid_2())  ||
			cyttsp_force_fw_load()) {
			cyttsp_debug("blttsp=0x%02X%02X flttsp=0x%02X%02X force=%d\n", \
				g_bl_data.ttspver_hi, g_bl_data.ttspver_lo, \
				cyttsp_tts_verh(), cyttsp_tts_verl(), cyttsp_force_fw_load());
			cyttsp_debug("blappid=0x%02X%02X flappid=0x%02X%02X\n", \
				g_bl_data.appid_hi, g_bl_data.appid_lo, \
				cyttsp_app_idh(), cyttsp_app_idl());
			cyttsp_debug("blappver=0x%02X%02X flappver=0x%02X%02X\n", \
				g_bl_data.appver_hi, g_bl_data.appver_lo, \
				cyttsp_app_verh(), cyttsp_app_verl());
			cyttsp_debug("blcid=0x%02X%02X%02X flcid=0x%02X%02X%02X\n", \
				g_bl_data.cid_0, g_bl_data.cid_1, g_bl_data.cid_2, \
				cyttsp_cid_0(), cyttsp_cid_1(), cyttsp_cid_2());
			/* enter bootloader to load new app into TTSP Device */
			retval = cyttsp_bootload_app(ts);
			/* take TTSP device out of bootloader mode; switch back to TrueTouch operational mode */
			if (!(retval < CYTTSP_OPERATIONAL)) {
				retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_BASE,
					sizeof(bl_cmd), bl_cmd);
				/* wait for TTSP Device to complete switch to Operational mode */
				mdelay(1000);
			}
		}
	}

bypass:
	/* switch to System Information mode to read versions and set interval registers */
	if (!(retval < CYTTSP_OPERATIONAL)) {
		cyttsp_debug("switch to sysinfo mode \n");
		host_reg = CYTTSP_SYSINFO_MODE;
		retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_BASE,
			sizeof(host_reg), &host_reg);
		/* wait for TTSP Device to complete switch to SysInfo mode */
		mdelay(1000);
		if (!(retval < CYTTSP_OPERATIONAL)) {
			retval = i2c_smbus_read_i2c_block_data(ts->client, CYTTSP_REG_BASE,
				sizeof(struct cyttsp_sysinfo_data_t), (u8 *)&g_sysinfo_data);
			cyttsp_debug("SI2: hst_mode=0x%02X mfg_cmd=0x%02X mfg_stat=0x%02X\n", \
				g_sysinfo_data.hst_mode, g_sysinfo_data.mfg_cmd, \
				g_sysinfo_data.mfg_stat);
			cyttsp_debug("SI2: bl_ver=0x%02X%02X\n", \
				g_sysinfo_data.bl_verh, g_sysinfo_data.bl_verl);
			cyttsp_debug("SI2: sysinfo act_int=0x%02X tch_tmout=0x%02X lp_int=0x%02X\n", \
				g_sysinfo_data.act_intrvl, g_sysinfo_data.tch_tmout, \
				g_sysinfo_data.lp_intrvl);
			cyttsp_info("SI%d: tver=%02X%02X a_id=%02X%02X aver=%02X%02X\n", \
				102, \
				g_sysinfo_data.tts_verh, g_sysinfo_data.tts_verl, \
				g_sysinfo_data.app_idh, g_sysinfo_data.app_idl, \
				g_sysinfo_data.app_verh, g_sysinfo_data.app_verl);
			cyttsp_info("SI%d: c_id=%02X%02X%02X\n", \
				103, \
				g_sysinfo_data.cid[0], g_sysinfo_data.cid[1], g_sysinfo_data.cid[2]);
			if (!(retval < CYTTSP_OPERATIONAL) &&
				(CYTTSP_DIFF(ts->platform_data->act_intrvl, CYTTSP_ACT_INTRVL_DFLT)  ||
				CYTTSP_DIFF(ts->platform_data->tch_tmout, CYTTSP_TCH_TMOUT_DFLT) ||
				CYTTSP_DIFF(ts->platform_data->lp_intrvl, CYTTSP_LP_INTRVL_DFLT))) {
				if (!(retval < CYTTSP_OPERATIONAL)) {
					u8 intrvl_ray[sizeof(ts->platform_data->act_intrvl) +
						sizeof(ts->platform_data->tch_tmout) +
						sizeof(ts->platform_data->lp_intrvl)];
					u8 i = 0;

					intrvl_ray[i++] = ts->platform_data->act_intrvl;
					intrvl_ray[i++] = ts->platform_data->tch_tmout;
					intrvl_ray[i++] = ts->platform_data->lp_intrvl;

					cyttsp_debug("SI2: platinfo act_intrvl=0x%02X tch_tmout=0x%02X lp_intrvl=0x%02X\n", \
						ts->platform_data->act_intrvl, ts->platform_data->tch_tmout, \
						ts->platform_data->lp_intrvl);
					// set intrvl registers
					retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_ACT_INTRVL,
						sizeof(intrvl_ray), intrvl_ray);
					mdelay(CYTTSP_DELAY_SYSINFO);
				}
			}
		}
		/* switch back to Operational mode */
		cyttsp_debug("switch back to operational mode \n");
		if (!(retval < CYTTSP_OPERATIONAL)) {
			host_reg = CYTTSP_OPERATE_MODE/* + CYTTSP_LOW_POWER_MODE*/;
			retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_BASE,
				sizeof(host_reg), &host_reg);
			/* wait for TTSP Device to complete switch to Operational mode */
			mdelay(1000);
		}
	}
	/* init gesture setup;
	 * this is required even if not using gestures
	 * in order to set the active distance */
	if (!(retval < CYTTSP_OPERATIONAL)) {
		u8 gesture_setup;
		cyttsp_debug("init gesture setup \n");
		gesture_setup = ts->platform_data->gest_set;
		retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_GEST_SET,
			sizeof(gesture_setup), &gesture_setup);
		mdelay(CYTTSP_DELAY_DFLT);
	}

	if (!(retval < CYTTSP_OPERATIONAL)) {
		ts->platform_data->power_state = CYTTSP_ACTIVE_STATE;
	}
	else {
		ts->platform_data->power_state = CYTTSP_IDLE_STATE;
	}
	cyttsp_debug("Retval=%d Power state is %s\n", retval, (ts->platform_data->power_state == CYTTSP_ACTIVE_STATE) ? "ACTIVE" : "IDLE");

	return retval;
}
#endif
/* cyttsp_initialize: Driver Initialization. This function takes
 * care of the following tasks:
 * 1. Create and register an input device with input layer
 * 2. Take CYTTSP device out of bootloader mode; go operational
 * 3. Start any timers/Work queues.  */
static int cyttsp_initialize(struct i2c_client *client, struct cyttsp *ts)
{
	struct input_dev *input_device;
	int error = 0;
	int retval = CYTTSP_OPERATIONAL;
	u8 id;

#ifdef FEATURE_SKY_PROCESS_CMD_KEY
	cyttsp_data = ts;
#endif

	/* Create the input device and register it. */
	input_device = input_allocate_device();
	if (!input_device) {
		error = -ENOMEM;
		cyttsp_xdebug1("err input allocate device\n");
		goto error_free_device;
	}

	if (!client) {
		error = ~ENODEV;
		cyttsp_xdebug1("err client is Null\n");
		goto error_free_device;
	}

	if (!ts) {
		error = ~ENODEV;
		cyttsp_xdebug1("err context is Null\n");
		goto error_free_device;
	}

	ts->input = input_device;
	input_device->name = CYTTSP_I2C_NAME;
	input_device->phys = ts->phys;
	input_device->dev.parent = &client->dev;


	set_bit(EV_SYN, input_device->evbit);
	set_bit(EV_KEY, input_device->evbit);
	set_bit(EV_ABS, input_device->evbit);
	set_bit(BTN_TOUCH, input_device->keybit);
//	set_bit(BTN_2, input_device->keybit);

#ifdef FEATURE_SKY_PROCESS_CMD_KEY
	// +++ FEATURE_P_VZW_PS_STABILITY_AT_CMD
	set_bit(KEY_MENU, input_device->keybit);
	set_bit(KEY_BACK, input_device->keybit);
	set_bit(KEY_POWER, input_device->keybit);
	set_bit(KEY_HOME, input_device->keybit);
	// ---
    set_bit(KEY_SEARCH, input_device->keybit);

    set_bit(KEY_0, input_device->keybit);
    set_bit(KEY_1, input_device->keybit);
    set_bit(KEY_2, input_device->keybit);
    set_bit(KEY_3, input_device->keybit);
    set_bit(KEY_4, input_device->keybit);
    set_bit(KEY_5, input_device->keybit);
    set_bit(KEY_6, input_device->keybit);
    set_bit(KEY_7, input_device->keybit);
    set_bit(KEY_8, input_device->keybit);
    set_bit(KEY_9, input_device->keybit);
    set_bit(0xe3, input_device->keybit); /* '*' */
    set_bit(0xe4, input_device->keybit); /* '#' */

    set_bit(KEY_LEFTSHIFT, input_device->keybit);
    set_bit(KEY_RIGHTSHIFT, input_device->keybit);


    set_bit(KEY_LEFT, input_device->keybit);
    set_bit(KEY_RIGHT, input_device->keybit);
    set_bit(KEY_UP, input_device->keybit);
    set_bit(KEY_DOWN, input_device->keybit);
    set_bit(KEY_ENTER, input_device->keybit);

    set_bit(KEY_SEND, input_device->keybit);
    set_bit(KEY_END, input_device->keybit);

    set_bit(KEY_VOLUMEUP, input_device->keybit);
    set_bit(KEY_VOLUMEDOWN, input_device->keybit);

    set_bit(KEY_CLEAR, input_device->keybit);

	set_bit(KEY_CAMERA, input_device->keybit);
	set_bit(KEY_DELETE, input_device->keybit);
	set_bit(KEY_WWW, input_device->keybit);

	set_bit(INPUT_PROP_DIRECT, input_device->propbit);

#endif // FEATURE_SKY_PROCESS_CMD_KEY



    // for ICS version - 0511 KJHW

    /* clear current touch tracking structures */
    memset(cur_trk, 0, sizeof(cur_trk));
    for (id = 0; id < TOUCH_MAX_NUM; id++)
    {
        cur_trk[id].abs[CY_ABS_ID_OST] = -1;
        cur_touchflag[id] = 0;
    }
    input_mt_init_slots(input_device, TOUCH_MAX_NUM);



	if (ts->platform_data->use_gestures) {
		set_bit(BTN_3, input_device->keybit);
	}

	input_set_abs_params(input_device, ABS_X, 0, ts->platform_data->maxx, 0, 0);
	input_set_abs_params(input_device, ABS_Y, 0, ts->platform_data->maxy, 0, 0);
	input_set_abs_params(input_device, ABS_TOOL_WIDTH, 0, CYTTSP_LARGE_TOOL_WIDTH, 0 ,0);
	input_set_abs_params(input_device, ABS_PRESSURE, 0, CYTTSP_MAXZ, 0, 0);
	input_set_abs_params(input_device, ABS_HAT0X, 0, ts->platform_data->maxx, 0, 0);
	input_set_abs_params(input_device, ABS_HAT0Y, 0, ts->platform_data->maxy, 0, 0);
	if (ts->platform_data->use_gestures) {
		input_set_abs_params(input_device, ABS_HAT1X, 0, CYTTSP_MAXZ, 0, 0);
		input_set_abs_params(input_device, ABS_HAT1Y, 0, CYTTSP_MAXZ, 0, 0);
	}

	if (ts->platform_data->use_mt) {
		input_set_abs_params(input_device, ABS_MT_POSITION_X, 0, ts->platform_data->maxx, 0, 0);
		input_set_abs_params(input_device, ABS_MT_POSITION_Y, 0, ts->platform_data->maxy, 0, 0);
		input_set_abs_params(input_device, ABS_MT_TOUCH_MAJOR, 0, CYTTSP_MAXZ, 0, 0);
		input_set_abs_params(input_device, ABS_MT_WIDTH_MAJOR, 0, CYTTSP_LARGE_TOOL_WIDTH, 0, 0);
		if (ts->platform_data->use_trk_id) {
			input_set_abs_params(input_device, ABS_MT_TRACKING_ID, 0, CYTTSP_NUM_TRACK_ID, 0, 0);
		}
	}
	// +++ FEATURE_P_VZW_PS_STABILITY_AT_CMD
//	input_set_abs_params(input_device, ABS_MT_TOOL_TYPE, 0, 1, 0, 0);
	// ---
	cyttsp_info("%s: Register input device\n", CYTTSP_I2C_NAME);

	error = input_register_device(input_device);
	if (error) {
		cyttsp_alert("%s: Failed to register input device\n", CYTTSP_I2C_NAME);
		retval = error;
		goto error_free_device;
	}
	else
		cyttsp_info("%s: Register input device success...\n", CYTTSP_I2C_NAME);


	/* Prepare our worker structure prior to setting up the timer/ISR */
	INIT_WORK(&ts->work,cyttsp_xy_worker);
#ifdef FEATURE_CYTTSP_HEARTBEAT
	INIT_WORK(&ts->work2,cyttsp_check_heartbeat);
#endif
#ifdef FEATURE_CYTTSP_FIRMWAREUPGRADE
	INIT_DELAYED_WORK(&ts->work3,check_firmware_update);
#endif
	/* Power on the chip and make sure that I/Os are set as specified
	 * in the platform
	 */
#if 0 //Cypress 업체 요청사항.
	retval = cyttsp_power_on(ts);
	if (retval < 0) {
		goto error_free_device;
	}
#endif
	/* Timer or Interrupt setup */
	if(ts->client->irq == 0) {
		cyttsp_info("Setting up timer\n");
		setup_timer(&ts->timer, cyttsp_timer, (unsigned long) ts);
		mod_timer(&ts->timer, jiffies + TOUCHSCREEN_TIMEOUT);
	}
	else {
#ifdef FEATURE_CYTTSP_HEARTBEAT
		start_heartbeat_timer = true;
		setup_timer(&ts->timer, cyttsp_timer, (unsigned long) ts);
		mod_timer(&ts->timer, jiffies + CYTTSP_HEARTBEAT_TIME * 20 * HZ); //처음 시작은 60초 뒤에 타이머 돌게 함 부팅시간에 영향을 주지 않기 위함.
#endif
		cyttsp_info("Setting up interrupt\n");
		/* request_irq() will also call enable_irq() */
		error = request_irq (client->irq,cyttsp_irq,IRQF_TRIGGER_FALLING,
			client->dev.driver->name,ts);
		if (error) {
			cyttsp_alert("error: could not request irq\n");
			retval = error;
			goto error_free_irq;
		}
	}

	atomic_set(&ts->irq_enabled, 1);
	retval = device_create_file(&ts->client->dev, &dev_attr_irq_enable);
	if (retval < CYTTSP_OPERATIONAL) {
		cyttsp_alert("File device creation failed: %d\n", retval);
		retval = -ENODEV;
		goto error_free_irq;
	}

	cyttsp_info("%s: Successful registration\n", CYTTSP_I2C_NAME);
	goto success;

error_free_irq:
	cyttsp_alert("Error: Failed to register IRQ handler\n");
	free_irq(client->irq,ts);

error_free_device:
	if (input_device) {
		input_free_device(input_device);
	}

success:
	return retval;
}

static int pantech_auto_check(u8* return_byte)
{
	u8 host_reg, byte_data[4], prev_data=0xff, byte_node1[MAX_NODE], byte_node2[MAX_NODE], send_byte[MAX_NODE];
	int retval = CYTTSP_OPERATIONAL, retry_cnt = 100, i;
	struct cyttsp *ts = ts_temp;

	dbg("pantech_auto_check!! start\n");

	// If phone enter a poweroff, Stop firmware update
	if(Touch_Status >= TOUCH_POWEROFF)
		return -1;


#ifdef CYTTSP_MUTEX_LOCK
	mutex_lock(&ts->lock_mutex);
#endif

	// Enter Test Mode
	host_reg = CYTTSP_TEST_MODE;
	retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_BASE, sizeof(host_reg), &host_reg);
	msleep(100);

	// Read Raw counts or baseline

	byte_data[0]  = 0x00;

	do {
		/* Read Count  */
		retval = i2c_smbus_read_i2c_block_data(ts->client, CYTTSP_REG_MODE , sizeof(byte_data), (u8 *)&byte_data);
		msleep(10);
	}
	while (byte_data[0] == prev_data && --retry_cnt);

	prev_data = byte_data[0];
	do {
		/* 	Read Count
			Must set a i2c.h	I2C_SMBUS_BLOCK_MAX	32 -> I2C_SMBUS_BLOCK_MAX	256
		*/

		retval = i2c_smbus_read_i2c_block_data(ts->client, CYTTSP_REG_SENSOR_BASE , sizeof(byte_node1), (u8 *)&byte_node1);
		msleep(10);
	}
	while (retval < CYTTSP_OPERATIONAL && --retry_cnt);
	//  Read Raw counts or baseline

	host_reg = CYTTSP_T_TEST_MODE;
	retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_BASE, sizeof(host_reg), &host_reg);
	msleep(100);

	byte_data[0]  = 0x00;
	retry_cnt = 100;

	do {
		/* Read Count  */
		retval = i2c_smbus_read_i2c_block_data(ts->client, CYTTSP_REG_MODE , sizeof(byte_data), (u8 *)&byte_data);
		msleep(10);
	}
	while (byte_data[0] == prev_data && --retry_cnt);

	prev_data = byte_data[0];

	do {
		/* Read Count  */
		retval = i2c_smbus_read_i2c_block_data(ts->client, CYTTSP_REG_SENSOR_BASE , sizeof(byte_node2), (u8 *)&byte_node2);
		msleep(10);
	}
	while (retval < CYTTSP_OPERATIONAL && --retry_cnt);

	for(i=0; i<MAX_NODE; i++)
	{
		if(byte_node1[i] >= CYTTSP_BASE_MIN && byte_node1[i] <= CYTTSP_BASE_MAX &&
			byte_node2[i] >= CYTTSP_BASE_MIN && byte_node2[i] <= CYTTSP_BASE_MAX)
			send_byte[i] = 0;
		else
			send_byte[i] = 1;

//		printk("Check Valid %d, byte_node1 %d, byte_node2 %d : %d\n", i , byte_node1[i], byte_node2[i], send_byte[i]);
	}

	// Retrun Operate Mode
	host_reg = CYTTSP_OPERATE_MODE;
	retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_BASE, sizeof(host_reg), &host_reg);

#ifdef CYTTSP_MUTEX_LOCK
		mutex_unlock(&ts->lock_mutex);
#endif

	msleep(100);

	dbg("pantech_auto_check!! end\n");

	for(i=0;i<MAX_NODE;i++)
		return_byte[i]=send_byte[i];

	return 0;

}

static int pantech_selftest_check(void)
{
	u8 host_reg, byte_data[2];
	int retval = CYTTSP_OPERATIONAL;
	struct cyttsp *ts = ts_temp;

	printk("pantech_selftest_check!! start\n");

	// If phone enter a poweroff, Stop firmware update
	if(Touch_Status >= TOUCH_POWEROFF)
		return -1;

#ifdef CYTTSP_MUTEX_LOCK
	mutex_lock(&ts->lock_mutex);
#endif

	// Enter system information Mode
	host_reg = CYTTSP_SYSINFO_MODE;
	retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_BASE, sizeof(host_reg), &host_reg);
	msleep(100);

	// Start self test
	host_reg = 0x01;
	retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_SELF_TEST, sizeof(host_reg), &host_reg);
	msleep(1500);

	// Read test result
	retval = i2c_smbus_read_i2c_block_data(ts->client, CYTTSP_REG_SELF_TEST , sizeof(byte_data), (u8 *)&byte_data);

	printk("0x18 test: %02x, 0x19 test: %02x\n", byte_data[0], byte_data[1]);

	// Retrun Operate Mode
	host_reg = CYTTSP_OPERATE_MODE;
	retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_BASE, sizeof(host_reg), &host_reg);
#ifdef CYTTSP_MUTEX_LOCK
	mutex_unlock(&ts->lock_mutex);
#endif
	msleep(100);

	printk("pantech_selftest_check!! end\n");

	if(byte_data[0] != 0)
		return CYTTSP_BIST_PROCESS;
	else if(byte_data[1] != 0xff)
	{
		if(!(byte_data[1] & 0x01))
			return CYTTSP_OPEN_TEST;
		else if(!(byte_data[1] & 0x02))
			return CYTTSP_SHORT_GND;
		else if(!(byte_data[1] & 0x04))
			return CYTTSP_SHORT_VDD;
		else if(!(byte_data[1] & 0x08))
			return CYTTSP_SHORT_PIN;
		else if(!(byte_data[1] & 0x10))
			return CYTTSP_LOCAL_IDAC;
		else if(!(byte_data[1] & 0x20))
			return CYTTSP_GLOBAL_IDAC;
		else if(!(byte_data[1] & 0x40))
			return CYTTSP_BASELINE_TEST;
		else if(!(byte_data[1] & 0x80))
			return CYTTSP_COMPLETE_BIT;
	}
	else
		return 0;

	return 0;
}


static void init_hw_setting(void)
{
	int rc;
	struct regulator *vreg_touch, *vreg_power_1_8;

	rc = gpio_request(GPIO_TOUCH_CHG, "touch_interrupt");

	if (rc) {
		printk(KERN_ERR "%s: Fail to set the TOUCH CHG (%d)\n",__func__, rc);
		return;
	}

	gpio_tlmm_config(GPIO_CFG(GPIO_TOUCH_CHG, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_set_value(GPIO_TOUCH_CHG, 0);

	// Power On, AVDD
	vreg_touch = regulator_get(NULL, "8058_l19");
	if (IS_ERR(vreg_touch))
	{
		rc = PTR_ERR(vreg_touch);
		printk(KERN_ERR "%s: regulator get of %s failed (%d)\n",
					__func__, (char *) vreg_touch, rc);
	}

	rc = regulator_set_voltage(vreg_touch, 3000000, 3000000);
       if (rc) {
		printk(KERN_ERR "%s: vreg set level failed (%d)\n", __func__, rc);
		return;
	}

	rc = regulator_enable(vreg_touch);

	if (rc) {
		printk(KERN_ERR "%s: vreg enable failed (%d)\n",__func__, rc);
		return;
	}

	vreg_power_1_8 = regulator_get(NULL, "8058_l11");
	if (IS_ERR(vreg_power_1_8))
	{
		rc = PTR_ERR(vreg_power_1_8);
		printk(KERN_ERR "%s: regulator get of %s failed (%d)\n",
					__func__, (char *) vreg_power_1_8, rc);
	}

	rc = regulator_set_voltage(vreg_power_1_8, 1900000, 1900000);
       if (rc) {
		printk(KERN_ERR "%s: vreg set level failed (%d)\n", __func__, rc);
		return;
	}

	rc = regulator_enable(vreg_power_1_8);

	if (rc) {
		printk(KERN_ERR "%s: vreg enable failed (%d)\n",__func__, rc);
		return;
	}

	gpio_set_value(GPIO_TOUCH_CHG, 1);
	gpio_tlmm_config(GPIO_CFG(GPIO_TOUCH_CHG, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);

	gpio_tlmm_config(GPIO_CFG(GPIO_TOUCH_ID, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),GPIO_CFG_ENABLE);
	gpio_set_value(GPIO_TOUCH_ID, 0);

	regulator_put(vreg_touch);
	regulator_put(vreg_power_1_8);

	msleep(100);

	Touch_Status = TOUCH_POWERON;
}


/* I2C driver probe function */
static int __devinit cyttsp_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct cyttsp *ts;
	int error;
	int retval = CYTTSP_OPERATIONAL;
#ifdef FEATURE_SKY_PROCESS_CMD_KEY
	int rc;
#endif

	cyttsp_info("Start Probe\n");

	/* allocate and clear memory */
	ts = kzalloc (sizeof(struct cyttsp),GFP_KERNEL);
	if (ts == NULL) {
		cyttsp_xdebug1("err kzalloc for cyttsp\n");
		retval = -ENOMEM;
	}

	init_hw_setting();
	if (!(retval < CYTTSP_OPERATIONAL)) {
		/* register driver_data */
		ts->client = client;
		ts->platform_data = client->dev.platform_data;
		i2c_set_clientdata(client,ts);
	       ts->client->irq = IRQ_TOUCH_INT;
		error = cyttsp_initialize(client, ts);
		if (error) {
			cyttsp_xdebug1("err cyttsp_initialize\n");
			if (ts != NULL) {
				/* deallocate memory */
				kfree(ts);
			}
/*
			i2c_del_driver(&cyttsp_driver);
*/
			retval = -ENODEV;
		}
		else {
			cyttsp_openlog();
		}
	}


#ifdef CONFIG_HAS_EARLYSUSPEND
	if (!(retval < CYTTSP_OPERATIONAL)) {
		ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
		ts->early_suspend.suspend = cyttsp_early_suspend;
		ts->early_suspend.resume = cyttsp_late_resume;
		register_early_suspend(&ts->early_suspend);
	}
#endif /* CONFIG_HAS_EARLYSUSPEND */

#ifdef FEATURE_SKY_PROCESS_CMD_KEY
	rc = misc_register(&touch_event);
	if (rc) {
		pr_err("::::::::: can''t register touch_fops\n");
	}
#endif
	cyttsp_info("Start Probe %s\n", (retval < CYTTSP_OPERATIONAL) ? "FAIL" : "PASS");

	ts_temp = ts;

#ifdef CYTTSP_MUTEX_LOCK
	mutex_init(&ts->lock_mutex);
#endif

#if 1//ST_LIM_120522 User 모드에서 디버깅 사용 가능하게 변경.
	error = sysfs_create_group(&ts->input->dev.kobj,&touch_attribute_group);
#endif

#ifdef FEATURE_CYTTSP_FIRMWAREUPGRADE
	schedule_delayed_work(&ts->work3, msecs_to_jiffies(200));
#endif

	wake_lock_init(&touch_wake_lock, WAKE_LOCK_SUSPEND, "touch");

	return retval;
}

/* Function to manage power-on resume */
static int cyttsp_resume(struct i2c_client *client)
{
	struct cyttsp *ts;
	int ret=0;
	int retval = CYTTSP_OPERATIONAL;

	dbg("Wake Up\n");

	ts = (struct cyttsp *) i2c_get_clientdata(client);

	if(ts == NULL)
		return retval;

	pantech_ctl_update(ISSP_IOCTL_POWER_ALL , 1);
	pantech_ctl_update(ISSP_IOCTL_INTR, 1);
	pantech_ctl_update(ISSP_COMPLITED_UPDATA, 0);

	Touch_Status = TOUCH_POWERON;

    // for ICS version - 0511 KJHW
    input_mt_init_slots(ts->input, TOUCH_MAX_NUM);

	msleep(100);

	ret = request_irq (client->irq,cyttsp_irq,IRQF_TRIGGER_FALLING, client->dev.driver->name,ts);

#ifdef FEATURE_CYTTSP_HEARTBEAT
	mod_timer(&ts->timer, jiffies + CYTTSP_HEARTBEAT_TIME * HZ);
	start_heartbeat_timer = true;
#endif

	/* re-enable the interrupt after resuming */
	//enable_irq(ts->client->irq);

	cyttsp_debug("Wake Up %s\n", (retval < CYTTSP_OPERATIONAL) ? "FAIL" : "PASS" );
	return retval;
}


/* Function to manage low power suspend */
static int cyttsp_suspend(struct i2c_client *client, pm_message_t message)
{
	struct cyttsp *ts;
	int retval = CYTTSP_OPERATIONAL, id =0;

	dbg("Enter Sleep\n");

	ts = (struct cyttsp *) i2c_get_clientdata(client);

	if(ts == NULL)
		return retval;

	/* disable worker */
	disable_irq_nosync(ts->client->irq);

#ifdef FEATURE_CYTTSP_HEARTBEAT
	start_heartbeat_timer = false;
	retval = cancel_work_sync(&ts->work2);
	del_timer(&ts->timer);
#endif
#ifdef FEATURE_CYTTSP_FIRMWAREUPGRADE
//	cancel_work_sync(&ts->work3);
#endif
	Touch_Status = TOUCH_POWEROFF;
	retval = cancel_work_sync(&ts->work);

	pantech_ctl_update(ISSP_IOCTL_POWER_ALL, 0);

	free_irq(client->irq,ts);

	pantech_ctl_update(ISSP_IOCTL_SCLK_TO_GPIO, 1);
	pantech_ctl_update(ISSP_IOCTL_DATA_TO_GPIO, 1);

	pantech_ctl_update(ISSP_IOCTL_INTR, 0);
	pantech_ctl_update(ISSP_IOCTL_SCLK, 0);
	pantech_ctl_update(ISSP_IOCTL_DATA, 0);


    // for ICS version - 0511 KJHW
	for (id = 0; id < TOUCH_MAX_NUM; id++)
	{
        cur_trk[id].abs[CY_ABS_ID_OST] = -1;
        cur_touchflag[id] = 0;
        input_mt_slot(ts->input, id);

        input_report_abs(ts->input, ABS_MT_TRACKING_ID, cur_trk[id].abs[CY_ABS_ID_OST]);
        input_report_key(ts->input, BTN_TOUCH, 0 );
        input_sync(ts->input);
	}

    // for ICS version - 0511 KJHW
    input_mt_destroy_slots(ts->input);


	return retval;
}

/* registered in driver struct */
static int __devexit cyttsp_remove(struct i2c_client *client)
{
	struct cyttsp *ts;
	int err;

	cyttsp_alert("Unregister\n");

	/* clientdata registered on probe */
	ts = i2c_get_clientdata(client);
	device_remove_file(&ts->client->dev, &dev_attr_irq_enable);

	/* Start cleaning up by removing any delayed work and the timer */
	if (cancel_delayed_work((struct delayed_work *)&ts->work)<0) {
		cyttsp_alert("error: could not remove work from workqueue\n");
	}

	/* free up timer or irq */
    if(ts->client->irq == 0) {
		err = del_timer(&ts->timer);
		if (err < 0) {
			cyttsp_alert("error: failed to delete timer\n");
		}
	}
	else {

#ifdef FEATURE_CYTTSP_HEARTBEAT
	start_heartbeat_timer = false;
	del_timer(&ts->timer);
#endif
		free_irq(client->irq,ts);
	}

#if 1 //ST_LIM_120522 User 모드에서 디버깅 사용 가능하게 변경.
	sysfs_remove_group(&ts->input->dev.kobj, &touch_attribute_group);
#endif
	/* housekeeping */
	if (ts != NULL) {
		kfree(ts);
	}

	/* clientdata registered on probe */
	cyttsp_alert("Leaving\n");

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void cyttsp_early_suspend(struct early_suspend *handler)
{
	struct cyttsp *ts;

	ts = container_of(handler, struct cyttsp, early_suspend);
	cyttsp_suspend(ts->client, PMSG_SUSPEND);
}

static void cyttsp_late_resume(struct early_suspend *handler)
{
	struct cyttsp *ts;

	ts = container_of(handler, struct cyttsp, early_suspend);
	cyttsp_resume(ts->client);
}
#endif  /* CONFIG_HAS_EARLYSUSPEND */

static int cyttsp_init(void)
{
	int ret;

	cyttsp_info("Cypress TrueTouch(R) Standard Product I2C Touchscreen Driver (Built %s @ %s)\n",__DATE__,__TIME__);
	ret = i2c_add_driver(&cyttsp_driver);

	return ret;
}

static void cyttsp_exit(void)
{
	return i2c_del_driver(&cyttsp_driver);
}

module_init(cyttsp_init);
module_exit(cyttsp_exit);


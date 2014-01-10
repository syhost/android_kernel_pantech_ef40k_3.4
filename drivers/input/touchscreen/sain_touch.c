/* 
 *
 * Sain touch driver
 *
 * Copyright (C) 2009 Sain, Inc.
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
// Version 2.0.0 : using reg data file (2010/11/05)
// Version 2.0.1 : syntxt bug fix (2010/11/09)
// Version 2.0.2 : Save status cmd delay bug (2010/11/10)
// Version 2.0.3 : modify delay 10ms -> 50ms for clear hw calibration bit
//		: modify SAIN_TOTAL_NUMBER_OF_Y register (read only -> read/write )
//		: modify SUPPORTED FINGER NUM register (read only -> read/write )
// Version 2.0.4 : [20101116]
//	Modify Firmware Upgrade routine.
// Version 2.0.5 : [20101118]
//	add esd timer function & some bug fix.
//	you can select request_threaded_irq or request_irq, setting USE_THREADED_IRQ.
// Version 2.0.6 : [20101123]
//	add ABS_MT_WIDTH_MAJOR Report
// Version 2.0.7 : [20101201]
//	Modify sain_early_suspend() / sain_late_resume() routine.
// Version 2.0.8 : [20101216]
//	add using spin_lock option
// Version 2.0.9 : [20101216]
//	Test Version
// Version 2.0.10 : [20101217]
//	add USE_THREAD_METHOD option. if  USE_THREAD_METHOD = 0, you use workqueue
// Version 2.0.11 : [20101229]
//	add USE_UPDATE_SYSFS option for update firmware. && TOUCH_MODE == 1 mode.
// Version 2.0.13 : [20110125]
//	modify esd timer routine
// Version 2.0.14 : [20110217]
//	esd timer bug fix. (kernel panic)
//	sysfs bug fix.
// Version 2.0.15 : [20110315]
//	add power off delay ,250ms
// Version 2.0.16 : [20110316]
//	add upgrade method using isp

#include <linux/slab.h>

#include <linux/module.h>
#include <linux/input.h>
#include <linux/i2c.h>		// I2C_M_NOSTART
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/hrtimer.h>
#include <linux/ioctl.h>
#include <linux/earlysuspend.h>
#include <linux/string.h>
#include <linux/semaphore.h>
#include <linux/kthread.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/firmware.h>
#include <linux/sched.h>
#include <linux/wakelock.h>

#include <asm/io.h>
#include <linux/delay.h>
#include <mach/vreg.h>          /* set a vreg */
#include <mach/gpio.h>
#include <linux/regulator/consumer.h>
//#include <asm/gpio.h>
#include <linux/uaccess.h>

#include "touch_ioctl.h"
#include "sain_touch.h"
#include "sain_touch_reg_data.h"
//#if TOUCH_ONESHOT_UPGRADE
#include "sain_touch_firmware_black.h"
#include "sain_touch_firmware_ws20.h"
#include "sain_touch_firmware_white.h"
const u8 *m_firmware_data;
//#endif


#define	SYSTEM_MAX_X_RESOLUTION	480
#define	SYSTEM_MAX_Y_RESOLUTION	800

#define	SAIN_DEBUG 0
static int TOUCH_DBG_ENABLE = 0;
static int INPUT_ENABLE = 1;

#define FILE_CONFIG
#define FEATURE_SKY_PROCESS_CMD_KEY

#define SAIN_DRIVER_NAME        "sain_touch"	//"sain_touch"

#define GPIO_TOUCH_IRQ  	gpio_to_irq(61)	// interrupt pin IRQ number
#define GPIO_TOUCH_PIN_NUMn  	61	// interrupt pin (active low) number
#define GPIO_TOUCH_RESETn 	93	// reset pin (active low) number
#define GPIO_TOUCH_ID 		95	// touch id pin number

#define GPIO_TOUCH_LDO_EN  	145 // WS20

#define POWER_OFF_SUSPEND

#define	SAIN_INIT_RETRY_CNT	10


#define	SAIN_ESD_TIMER_INTERVAL	2	//second : if 0, no use.
#define	SAIN_SCAN_RATE_HZ	60

#define	USE_THREAD_METHOD	0	// 1 = thread, 0 = workqueue

#define	USE_UPDATE_SYSFS	0

#define TOUCH_ICS_UPDATE //ST_LIM
#define TOUCH_USING_ISP_METHOD	0

#ifdef TOUCH_ICS_UPDATE
#include <linux/input/mt.h>
#define TOUCH_TRACKING_ID
#define SAIN_TOUCH 1
#define SAIN_NOTOUCH 0
#endif

#if	(!USE_THREAD_METHOD)
static struct workqueue_struct *sain_workqueue;
#endif

#if	SAIN_ESD_TIMER_INTERVAL
static struct workqueue_struct *sain_tmr_workqueue;
#endif

#if SAIN_DEBUG
#define	sain_debug(fmt, args...)	printk(KERN_INFO "[sain][%-18s:%5d]" fmt, __FUNCTION__, __LINE__, ## args)
#define	sain_debug_c(fmt, args...)	if(TOUCH_DBG_ENABLE)sain_debug(fmt, ## args)
#else
#define	sain_debug(fmt, args...)	do{}while(0)
#define	sain_debug_c(fmt, args...) 	do{}while(0)
#endif

#define swap_v(a, b, t)	((t) = (a), (a) = (b), (b) = (t))
#define swap_16(s) (((((s) & 0xff) << 8) | (((s) >> 8) & 0xff))) 

#define	SUPPORTED_FINGER_NUM			5
#define	SUPPORTED_BUTTON_NUM			3


#define	ICON_BUTTON_UNCHANGE			0
#define	ICON_BUTTON_DOWN			1
#define	ICON_BUTTON_UP				2


#define	I2C_SUCCESS				0

#define ts_write_cmd(client,reg) i2c_smbus_write_byte(client, reg)
#define ts_write_reg(client,reg,value) i2c_smbus_write_word_data(client, reg, value)
#define ts_select_reg(client,reg) i2c_smbus_write_byte(client, reg)

// include/linux/i2c.h	-> I2C_SMBUS_BLOCK_MAX °ª º¯°æ
inline s32 ts_read_data(struct i2c_client *client, u8 reg, u8 *values, u8 length)
{
	s32 ret;
	if((ret = i2c_master_send(client , &reg , 1)) < 0)	return ret;	// select register
	udelay(50);		// for setup tx transaction.
	if((ret = i2c_master_recv(client , values , length)) < 0)	return ret;
	return length;
}

#if	TOUCH_USING_ISP_METHOD
inline s32 ts_read_firmware_data(struct i2c_client *client, char *addr, u8 *values, u8 length)
{
	s32 ret;
	if((ret = i2c_master_send(client , addr , 2)) < 0)	return ret;	// select register
	mdelay(1);		// for setup tx transaction.
	if((ret = i2c_master_recv(client , values , length)) < 0)	return ret;
	return length;
}

inline s32 ts_write_firmware_data(struct i2c_client *client, u8 *values, u8 length)
{
	s32 ret;
	if((ret = i2c_master_send(client , values , length)) < 0)	return ret;	// select register
	return length;
}
#else
inline s32 ts_read_firmware_data(struct i2c_client *client, u8 reg, u8 *values, u8 length)
{
	s32 ret;
	if((ret = i2c_master_send(client , &reg , 1)) < 0)	return ret;	// select register
	mdelay(1);		// for setup tx transaction.
	if((ret = i2c_master_recv(client , values , length)) < 0)	return ret;
	return length;
}
#endif

#define ts_write_data(client,reg,values,length) i2c_smbus_write_i2c_block_data(client, reg, length, values)

// default values
//-------------------------------------------------------
#define	TOUCH_MODE				1

//-------------------------------------------------------

typedef	struct	
{
	u16	x;
	u16	y;
	//u16	width;	
	u8	width;	
	u8	sub_status;
}_ts_sain_coord;

typedef	struct	
{
	u16	status;
#if (TOUCH_MODE == 1)
	u16	event_flag;
#else
	u8	finger_cnt;
	u8	time_stamp;
#endif	
	_ts_sain_coord	coord[SUPPORTED_FINGER_NUM];

}_ts_sain_point_info;

typedef enum {
	black,
	white,
}tw_color;


#define	TOUCH_V_FLIP	0x01
#define	TOUCH_H_FLIP	0x02
#define	TOUCH_XY_SWAP	0x04

typedef	struct
{
	u16 chip_revision; 
	u16 chip_firmware_version;
	u16 chip_reg_data_version;		  
	u32 chip_fw_size;	
	u32 MaxX;
	u32 MaxY;
	u32 MinX;
	u32 MinY;
	u32 Orientation;
	u8 gesture_support;
	u8 multi_fingers;
}_ts_capa_info;

typedef struct
{
	struct input_dev *input_dev;
	struct task_struct *task;
	wait_queue_head_t	wait;
	struct work_struct  work;
	struct work_struct  tmr_work;
	struct i2c_client *client;
	struct semaphore update_lock;    
	u32 i2c_dev_addr;
	_ts_capa_info	cap_info;
	tw_color color;

	bool is_valid_event;
	_ts_sain_point_info touch_info;
	_ts_sain_point_info reported_touch_info;    	
	u16 icon_event_reg;
	u16 chip_int_mask;    
	u16 event_type;
	u32 int_gpio_num;
	u32 irq;
	u8 button[SUPPORTED_BUTTON_NUM];

#if	SAIN_ESD_TIMER_INTERVAL
	u8	use_esd_timer;
	struct semaphore esd_lock;    
	struct timer_list esd_timeout_tmr;		//for repeated card detecting work
	struct timer_list *p_esd_timeout_tmr;		//for repeated card detecting work
#endif


#ifdef CONFIG_HAS_EARLYSUSPEND	
	struct semaphore	suspend_lock;
	struct early_suspend early_suspend;
	struct wake_lock touch_wake_lock;
#endif
	bool is_calibrated;
} sain_touch_dev;

sain_touch_dev* touch_dev = NULL;


#define	TS_DRIVER_NAME	"sain_touch"	// for i2c
static struct i2c_device_id sain_idtable[] = {
	{TS_DRIVER_NAME, 0},  
	{ }   
};

//u32 BUTTON_MAPPING_KEY[SUPPORTED_BUTTON_NUM] = {KEY_SEARCH, KEY_BACK, KEY_HOME, KEY_MENU};
u32 BUTTON_MAPPING_KEY[SUPPORTED_BUTTON_NUM] = {KEY_BACK, KEY_HOME, KEY_MENU};

static int sain_touch_probe(struct i2c_client *client, const struct i2c_device_id *i2c_id);
static int sain_touch_remove(struct i2c_client *client);

#if (TOUCH_MODE == 1)
static void	sain_report_data(sain_touch_dev *touch_dev, int id);
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void sain_early_suspend(struct early_suspend *h);
static void sain_late_resume(struct early_suspend *h);
#endif

#ifdef CONFIG_PM
static int sain_suspend(struct device *dev);
static int sain_resume(struct device *dev);
#endif

#if	SAIN_ESD_TIMER_INTERVAL
static void ts_esd_timer_start(u16 sec, sain_touch_dev* touch_dev);
static void ts_esd_timer_stop(sain_touch_dev* touch_dev);
static void ts_esd_timer_init(sain_touch_dev* touch_dev);
static void ts_esd_timeout_handler(unsigned long data);
#endif


#ifdef FILE_CONFIG
static long ts_fops_ioctl(struct file *filp,unsigned int cmd, unsigned long arg);
static int ts_fops_open(struct inode *inode, struct file *filp);
static int ts_fops_close(struct inode *inode, struct file *filp);

static struct file_operations ts_fops = {
	.owner = THIS_MODULE,
	.open = ts_fops_open,
	.release = ts_fops_close,
	//.ioctl = ts_fops_ioctl,
	.unlocked_ioctl = ts_fops_ioctl,
};

static struct miscdevice touch_event = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "touch_fops",
	.fops = &ts_fops,
};

#endif //FILE_CONFIG

const static struct dev_pm_ops i2c_device_sain_pm_ops = {
#ifdef CONFIG_PM
	.suspend = sain_suspend,
	.resume = sain_resume,
#endif
};
// id -> include/linux/i2c-id.h 
static struct i2c_driver sain_touch_driver = {
	.probe     = sain_touch_probe,
	.remove    = sain_touch_remove,
	.id_table  = sain_idtable,
	.driver    = {
		.name  = SAIN_DRIVER_NAME,
		.pm = &i2c_device_sain_pm_ops,
	},
};


static bool ts_get_samples (sain_touch_dev* touch_dev)
{
	int i;

	sain_debug_c("ts_get_samples+\r\n");


	if (gpio_get_value(touch_dev->int_gpio_num))
	{
		//interrupt pin is high, not valid data.
		sain_debug_c(KERN_WARNING "woops... inturrpt pin is high\r\n");
		return false;
	}

#if (TOUCH_MODE == 1)

	memset(&touch_dev->touch_info, 0x0, sizeof(_ts_sain_point_info));	

	if (ts_read_data (touch_dev->client, SAIN_POINT_STATUS_REG, (u8*)(&touch_dev->touch_info), 4)< 0)
	{

		sain_debug("error read point info using i2c.-\r\n");
		return false;
	}
	sain_debug_c("status reg = 0x%x , event_flag = 0x%04x\r\n", touch_dev->touch_info.status, touch_dev->touch_info.event_flag);

	if(touch_dev->touch_info.status == 0x0)
	{
		sain_debug_c("periodical esd repeated int occured\r\n");
		return true;
	}

	if(sain_bit_test(touch_dev->touch_info.status, BIT_ICON_EVENT))
	{
		udelay(20);
		if (ts_read_data (touch_dev->client, SAIN_ICON_STATUS_REG, (u8*)(&touch_dev->icon_event_reg), 2) < 0)
		{
			sain_debug(KERN_INFO "error read icon info using i2c.\n");
			return false;
		}
		return true;
	}

	if(!sain_bit_test(touch_dev->touch_info.status, BIT_PT_EXIST))
	{
		//ts_write_cmd(touch_dev->client, SAIN_CLEAR_INT_STATUS_CMD);
		for(i=0; i < SUPPORTED_FINGER_NUM; i++)
		{		
			if(sain_bit_test(touch_dev->reported_touch_info.coord[i].sub_status, SUB_BIT_EXIST))
			{		
#ifdef TOUCH_ICS_UPDATE
				input_mt_slot(touch_dev->input_dev,i);			
#ifdef	TOUCH_TRACKING_ID
				input_report_abs(touch_dev->input_dev,ABS_MT_TRACKING_ID,i);
#endif
#else
				input_report_abs(touch_dev->input_dev, ABS_MT_TOUCH_MAJOR, 0);
				input_report_abs(touch_dev->input_dev, ABS_MT_WIDTH_MAJOR, 0);	
				input_report_abs(touch_dev->input_dev, ABS_MT_POSITION_X, touch_dev->reported_touch_info.coord[i].x);
				input_report_abs(touch_dev->input_dev, ABS_MT_POSITION_Y, touch_dev->reported_touch_info.coord[i].y);
				input_mt_sync(touch_dev->input_dev);	
#endif
				touch_dev->reported_touch_info.coord[i].sub_status = 0;

			}
		}	
#ifdef TOUCH_ICS_UPDATE
		input_report_key(touch_dev->input_dev, BTN_TOUCH, SAIN_NOTOUCH);	
#endif			
		input_sync(touch_dev->input_dev);
		return true;
	}	


	for(i=0; i < SUPPORTED_FINGER_NUM; i++)
	{
		if(sain_bit_test(touch_dev->touch_info.event_flag, i))
		{			
			udelay(20);
			if (ts_read_data (touch_dev->client, SAIN_POINT_STATUS_REG+2+i, (u8*)(&touch_dev->touch_info.coord[i]), sizeof(_ts_sain_coord))< 0)
			{
				sain_debug("error read point info using i2c.-\r\n");
				return false;
			}
			sain_bit_clr(touch_dev->touch_info.event_flag, i);
			if(touch_dev->touch_info.event_flag == 0)	
			{
				//ts_write_cmd(touch_dev->client, SAIN_CLEAR_INT_STATUS_CMD);
				sain_report_data(touch_dev, i);
				return true;
			}
			else
				sain_report_data(touch_dev, i);
		}		
	}	


#else
	if (ts_read_data (touch_dev->client, SAIN_POINT_STATUS_REG, (u8*)(&touch_dev->touch_info), sizeof(_ts_sain_point_info))< 0)
	{
		sain_debug("error read point info using i2c.-\r\n");
		return false;
	}

	sain_debug_c("status reg = 0x%x , point cnt = %d, time stamp = %d\r\n", touch_dev->touch_info.status, touch_dev->touch_info.finger_cnt, touch_dev->touch_info.time_stamp);

	if(touch_dev->touch_info.status == 0x0 && touch_dev->touch_info.finger_cnt == 100)
	{
		sain_debug_c("periodical esd repeated int occured\r\n");
		return true;
	}



	for(i=0; i< SUPPORTED_BUTTON_NUM; i++)	touch_dev->button[i] = ICON_BUTTON_UNCHANGE;

	if(sain_bit_test(touch_dev->touch_info.status, BIT_ICON_EVENT))
	{
		udelay(10);
		if (ts_read_data (touch_dev->client, SAIN_ICON_STATUS_REG, (u8*)(&touch_dev->icon_event_reg), 2) < 0)
		{
			sain_debug(KERN_INFO "error read icon info using i2c.\n");
			return false;
		}
	}
#endif		
	sain_debug_c("ts_get_samples-\r\n");


	return true;
}


static irqreturn_t ts_int_handler(int irq, void *dev)
{
	sain_touch_dev* touch_dev = (sain_touch_dev*)dev;
	sain_debug_c("interrupt occured+\r\n");

	disable_irq_nosync(irq);
#if USE_THREAD_METHOD	
	up(&touch_dev->update_lock); 	
#else
	queue_work(sain_workqueue, &touch_dev->work);
#endif	// USE_THREAD_METHOD
	return IRQ_HANDLED;
}

static bool ts_read_coord (sain_touch_dev * hDevice)
{
	sain_touch_dev* touch_dev = (sain_touch_dev*)hDevice;
	sain_debug_c("ts_read_coord+\r\n");

	if(ts_get_samples(touch_dev)==false)
	{
		return false;
	}
	ts_write_cmd(touch_dev->client, SAIN_CLEAR_INT_STATUS_CMD);
	udelay(20);
	return true;
}

//
static void ts_power_control(u8 ctl)
{
	int rc; 
	struct regulator *vreg_power_1_8;
	vreg_power_1_8 = regulator_get(NULL, "8058_lvs0");
	if (IS_ERR(vreg_power_1_8))
	{
		rc = PTR_ERR(vreg_power_1_8);
//		sain_debug(KERN_ERR "%s: regulator get of %s failed (%d)\n",
//				__func__, vreg_power_1_8, rc);
	}

	if(ctl == 0) //power off
	{
		rc = regulator_disable(vreg_power_1_8);
		gpio_set_value(GPIO_TOUCH_LDO_EN, 0);	
		gpio_set_value(GPIO_TOUCH_RESETn, 0);	
		gpio_set_value(GPIO_TOUCH_PIN_NUMn, 0);	
	}
	else if(ctl == 1)	//power on
	{
		gpio_set_value(GPIO_TOUCH_RESETn, 1);	
		rc = regulator_enable(vreg_power_1_8);
		gpio_set_value(GPIO_TOUCH_LDO_EN, 1);	
		gpio_set_value(GPIO_TOUCH_PIN_NUMn, 1);	
	}
	else if(ctl == 2)	//reset pin low
	{
	}
	else if(ctl == 3)	//reset pin high
	{
	}
}


static bool ts_mini_init_touch(sain_touch_dev * touch_dev)
{
	u16	reg_val;
	int i;	
	if(touch_dev == NULL)	
	{
		sain_debug(KERN_INFO "ts_mini_init_touch : error (touch_dev == NULL?)\r\n");
		return false;		
	}	
	if (ts_write_reg(touch_dev->client, SAIN_INT_ENABLE_FLAG, 0x0)!=I2C_SUCCESS)	goto fail_mini_init;	
	mdelay(10);
	if (ts_write_cmd(touch_dev->client, SAIN_CLEAR_INT_STATUS_CMD)!=I2C_SUCCESS)		goto fail_mini_init;
	if (ts_write_reg(touch_dev->client, SAIN_X_RESOLUTION, (u16)(touch_dev->cap_info.MaxX))!=I2C_SUCCESS) goto fail_mini_init;
	if (ts_write_reg(touch_dev->client, SAIN_Y_RESOLUTION, (u16)(touch_dev->cap_info.MaxY))!=I2C_SUCCESS) goto fail_mini_init;   
	if (ts_write_reg(touch_dev->client, SAIN_SUPPORTED_FINGER_NUM, (u16)touch_dev->cap_info.multi_fingers)!=I2C_SUCCESS) goto fail_mini_init;	
	reg_val = TOUCH_MODE;
	if (ts_write_reg(touch_dev->client, SAIN_TOUCH_MODE, reg_val)!=I2C_SUCCESS)	goto fail_mini_init;	// soft calibration
	if (ts_write_cmd(touch_dev->client, SAIN_CALIBRATE_CMD)!=I2C_SUCCESS)		goto fail_mini_init;
	if (ts_write_reg(touch_dev->client, SAIN_INT_ENABLE_FLAG, touch_dev->chip_int_mask)!=I2C_SUCCESS)	goto fail_mini_init;		
	//---------------------------------------------------------------------
	// read garbage data
	for(i=0; i<10; i++)
	{
		ts_write_cmd(touch_dev->client, SAIN_CLEAR_INT_STATUS_CMD);
	}

#if	SAIN_ESD_TIMER_INTERVAL	
	if(touch_dev->use_esd_timer)
	{
		if (ts_write_reg(touch_dev->client, SAIN_PERIODICAL_INTERRUPT_INTERVAL, SAIN_SCAN_RATE_HZ)!=I2C_SUCCESS)	
			goto fail_mini_init;
		ts_esd_timer_start(SAIN_ESD_TIMER_INTERVAL*1, touch_dev);
	}
#endif		

	return true;
fail_mini_init:
	sain_debug(KERN_ERR "error mini init\n");

	return false;
}


#if	SAIN_ESD_TIMER_INTERVAL

static void sain_touch_tmr_work(struct work_struct *work)
{
	sain_touch_dev *touch_dev = container_of(work, sain_touch_dev, tmr_work);	

	sain_debug(KERN_INFO "tmr queue work ++\r\n");
	if(touch_dev == NULL)
	{
		sain_debug(KERN_INFO "touch dev == NULL ?\r\n");
		goto fail_time_out_init;
	}

	disable_irq_nosync(touch_dev->client->irq);
#if (!USE_THREAD_METHOD)
	flush_workqueue(sain_workqueue);
#endif
	down(&touch_dev->esd_lock);
	printk(KERN_INFO "error. timeout occured. maybe ts device dead. so reset & reinit.\r\n");
	ts_power_control (0);	//power off
	mdelay(250);		//must
	ts_power_control (1);	//power on

	mdelay(500);

	if(ts_mini_init_touch(touch_dev) == false)
		goto fail_time_out_init;
	enable_irq(touch_dev->client->irq);	
	up(&touch_dev->esd_lock);
	sain_debug(KERN_INFO "tmr queue work ----\r\n");
	return;
fail_time_out_init:
	sain_debug(KERN_INFO "tmr work : restart error\r\n");	
	ts_esd_timer_start(SAIN_ESD_TIMER_INTERVAL*1, touch_dev);
	enable_irq(touch_dev->client->irq);
	up(&touch_dev->esd_lock);
}

static void ts_esd_timer_start(u16 sec, sain_touch_dev* touch_dev)
{	
	if(touch_dev->p_esd_timeout_tmr != NULL)	del_timer(touch_dev->p_esd_timeout_tmr);
	touch_dev->p_esd_timeout_tmr = NULL;

	init_timer(&(touch_dev->esd_timeout_tmr));
	touch_dev->esd_timeout_tmr.data = (unsigned long)(touch_dev);
	touch_dev->esd_timeout_tmr.function = ts_esd_timeout_handler;			
	touch_dev->esd_timeout_tmr.expires = jiffies + HZ*sec;	
	touch_dev->p_esd_timeout_tmr = &touch_dev->esd_timeout_tmr;
	add_timer(&touch_dev->esd_timeout_tmr);
}



static void ts_esd_timer_stop(sain_touch_dev* touch_dev)
{
	if(touch_dev->p_esd_timeout_tmr) del_timer(touch_dev->p_esd_timeout_tmr);
	touch_dev->p_esd_timeout_tmr = NULL;
}

//static void ts_esd_timer_modify(u16 sec, sain_touch_dev* touch_dev)
//{
//	mod_timer(&touch_dev->esd_timeout_tmr, jiffies + (HZ*sec));
//}

static void ts_esd_timer_init(sain_touch_dev* touch_dev)
{
	init_timer(&(touch_dev->esd_timeout_tmr));
	touch_dev->esd_timeout_tmr.data = (unsigned long)(touch_dev);
	touch_dev->esd_timeout_tmr.function = ts_esd_timeout_handler;		
	touch_dev->p_esd_timeout_tmr=NULL;
}

static void ts_esd_timeout_handler(unsigned long data)
{
	sain_touch_dev* touch_dev = (sain_touch_dev*)data;
	touch_dev->p_esd_timeout_tmr=NULL;	
	queue_work(sain_tmr_workqueue, &touch_dev->tmr_work);
}
#endif

#if	TOUCH_ONESHOT_UPGRADE
bool ts_check_need_upgrade(u16 curVersion, u16 curRegVersion)
{
	u16	newVersion;
	newVersion = (u16) (m_firmware_data[0] | (m_firmware_data[1]<<8));

	sain_debug(KERN_INFO "cur Version = 0x%x, new Version = 0x%x\n", curVersion, newVersion);

	if(curVersion < newVersion)		return true;
	else if(curVersion > newVersion)	return false;
	if(m_firmware_data[0x3FFE] == 0xff && m_firmware_data[0x3FFF] == 0xff)	return false;

	newVersion = (u16) (m_firmware_data[0x3FFE] | (m_firmware_data[0x3FFF]<<8));

	if(curRegVersion < newVersion)	return true;

	return false;
}


#define	TC_FIRMWARE_SIZE	(16*1024)
#define	TC_PAGE_SZ		64
#define	TC_SECTOR_SZ		8

u8 ts_upgrade_firmware(sain_touch_dev* touch_dev, const u8 *firmware_data, u32 size)
{
	u16 flash_addr;
	u32 i;
	u8  * verify_data;
	int	retry_cnt;
#if	TOUCH_USING_ISP_METHOD
	u8	i2c_buffer[TC_PAGE_SZ+2];
	unsigned short addr;
#endif	
	retry_cnt = 0;
	verify_data = (u8*)kzalloc(size, GFP_KERNEL);
	if(verify_data == NULL)
	{
		sain_debug(KERN_ERR "cannot alloc verify buffer\n");
		return false;
	}

#if	(TOUCH_USING_ISP_METHOD==0)

	do{
		sain_debug(KERN_INFO "reset command\n");
		if (ts_write_cmd(touch_dev->client, SAIN_REST_CMD)!=I2C_SUCCESS)
		{
			sain_debug(KERN_INFO "failed to reset\n");
			goto fail_upgrade;
		}

		sain_debug(KERN_INFO "Erase Flash\n");
		if (ts_write_reg(touch_dev->client, SAIN_ERASE_FLASH, 0xaaaa)!=I2C_SUCCESS)
		{
			sain_debug(KERN_INFO "failed to erase flash\n");
			goto fail_upgrade;
		}

		mdelay(500);

		sain_debug(KERN_INFO "writing firmware data\n");			

		for(flash_addr= 0; flash_addr< size; )
		{

			for(i=0; i< TC_PAGE_SZ/TC_SECTOR_SZ; i++)
			{
				//sain_debug(KERN_INFO "addr = %04x, len = %d\n", flash_addr, TC_SECTOR_SZ); // Error
				if(ts_write_data(touch_dev->client,SAIN_WRITE_FLASH,&firmware_data[flash_addr],TC_SECTOR_SZ)<0)
				{
					sain_debug(KERN_INFO"error : write sain tc firmare\n");
					goto fail_upgrade;
				}		
				flash_addr+= TC_SECTOR_SZ;	
				udelay(100);
			}
			mdelay(20);			
		}

		sain_debug(KERN_INFO "read firmware data\n");					
		for(flash_addr= 0; flash_addr< size; )
		{

			for(i=0; i< TC_PAGE_SZ/TC_SECTOR_SZ; i++)
			{
				//sain_debug(KERN_INFO "addr = %04x, len = %d\n", flash_addr, TC_SECTOR_SZ);
				if(ts_read_firmware_data(touch_dev->client,SAIN_READ_FLASH,&verify_data[flash_addr],TC_SECTOR_SZ)<0)
				{
					sain_debug(KERN_INFO "error : read sain tc firmare\n");
					goto fail_upgrade;
				}		
				flash_addr+= TC_SECTOR_SZ;			
			}
		}

		// verify
		sain_debug(KERN_INFO "verify firmware data\n");
		if(memcmp((u8*)&firmware_data[0], (u8*)&verify_data[0], size) == 0)
		{
			sain_debug(KERN_INFO "upgrade finished\n");
			kfree(verify_data);
			return true;
		}
		sain_debug(KERN_INFO "upgrade fail : so retry... (%d)\n", ++retry_cnt);

	}while(1);

fail_upgrade:
	kfree(verify_data);
	return false;

#else		//ISP


	addr = touch_dev->client->addr;
	// 7 bit eeprom addr
	touch_dev->client->addr = 0x50;

	//must be reset pin low
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++
	ts_power_control(2);	//reset pin low
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++

	for(flash_addr= 0; flash_addr< size; flash_addr+=TC_PAGE_SZ)
	{

		sain_debug(KERN_INFO "firmware write : addr = %04x, len = %d\n", flash_addr, TC_PAGE_SZ);
		i2c_buffer[0] = (flash_addr>>8)&0xff;	//addr_h
		i2c_buffer[1] = (flash_addr)&0xff;	//addr_l

		if(ts_write_firmware_data(touch_dev->client, i2c_buffer, TC_PAGE_SZ+2)<0)	
		{
			sain_debug(KERN_INFO"error : write sain tc firmare\n");
			goto fail_upgrade;
		}		
		mdelay(20);			
	}

	sain_debug(KERN_INFO "read firmware data\n");					
	for(flash_addr= 0; flash_addr< size; flash_addr+=TC_PAGE_SZ)
	{
		sain_debug(KERN_INFO "firmware read : addr = %04x, len = %d\n", flash_addr, TC_PAGE_SZ);
		i2c_buffer[0] = (flash_addr>>8)&0xff;	//addr_h
		i2c_buffer[1] = (flash_addr)&0xff;	//addr_l
		if(ts_read_firmware_data(touch_dev->client, i2c_buffer,&verify_data[flash_addr],TC_PAGE_SZ)<0)
		{
			sain_debug(KERN_INFO "error : read sain tc firmare\n");
			goto fail_upgrade;
		}				

	}
	// verify
	sain_debug(KERN_INFO "verify firmware data\n");
	if(memcmp((u8*)&firmware_data[0], (u8*)&verify_data[0], size) == 0)
	{
		sain_debug(KERN_INFO "upgrade finished\n");
		//must be reset pin high
		//+++++++++++++++++++++++++++++++++++++++++++++++++++++
		ts_power_control(3);	//reset pin high
		//+++++++++++++++++++++++++++++++++++++++++++++++++++++
		touch_dev->client->addr = addr;
		kfree(verify_data);
		return true;
	}
	sain_debug(KERN_INFO "upgrade fail : so retry... (%d)\n", ++retry_cnt);


fail_upgrade:
	//must be reset pin high
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++
	ts_power_control(3);	//reset pin high
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++
	touch_dev->client->addr = addr;
	kfree(verify_data);
	return false;


#endif

}

#endif

typedef enum
{
        BATTERY_PLUGGED_NONE = 0,
        BATTERY_PLUGGED_AC = 1,
        BATTERY_PLUGGED_USB = 2,
        BATTERY_PLUGGED_SLEEP = 10
} CHARGER_MODE;

//static int charger_mode = BATTERY_PLUGGED_NONE;
//static int current_charger_mode = BATTERY_PLUGGED_NONE;
//
//void switch_charger_mode(void) 
//{
//	if(current_charger_mode == charger_mode)
//		//return;
//	if(charger_mode == BATTERY_PLUGGED_NONE)
//	{
//		if (ts_write_reg(touch_dev->client, 0x16, 0)!=I2C_SUCCESS) 
//			return;
//	}
//	else if(charger_mode == BATTERY_PLUGGED_AC)
//	{
//		if (ts_write_reg(touch_dev->client, 0x16, 1)!=I2C_SUCCESS) 
//			return;
//	}
//	else if(charger_mode == BATTERY_PLUGGED_USB)
//	{
//		if (ts_write_reg(touch_dev->client, 0x16, 1)!=I2C_SUCCESS) 
//			return;
//	}
//	current_charger_mode = charger_mode;
//}


bool ts_init_touch(sain_touch_dev* touch_dev)
{
	u16	reg_val;
	int	i;
	u16 	SetMaxX = SYSTEM_MAX_X_RESOLUTION; //Max Position range from 0x0002 to 0x1fff
	u16 	SetMaxY = SYSTEM_MAX_Y_RESOLUTION; //Max Position range from 0x0002 to 0x1fff  
	u16	SupportedFingerNum = SUPPORTED_FINGER_NUM;

	u16 	CurMaxX = 1024;
	u16 	CurMaxY = 1920;      
	u16 	chip_revision;
	u16 	chip_firmware_version;
	u16	chip_reg_data_version;
	u16	chip_eeprom_info;
	s16	stmp;


	sain_debug("sain touch init +\r\n");

	if(touch_dev == NULL)
	{
		sain_debug(KERN_ERR "error touch_dev == null?\r\n");
		goto fail_init;
	}

	sain_debug_c("disable interrupt\r\n");
	for(i=0; i<SAIN_INIT_RETRY_CNT; i++)
	{
		if (ts_write_reg(touch_dev->client, SAIN_INT_ENABLE_FLAG, 0x0)==I2C_SUCCESS)	break;
		mdelay(10);
	}

	if(i==SAIN_INIT_RETRY_CNT)
	{
		sain_debug(KERN_INFO "fail to write interrupt register\r\n");
		goto fail_init;
	}
	sain_debug_c("successfully disable int. (retry cnt = %d)\r\n", i);

	sain_debug_c("send reset command\r\n");
	if (ts_write_cmd(touch_dev->client, SAIN_REST_CMD)!=I2C_SUCCESS)	goto fail_init;

	/* get chip revision id */
	if (ts_read_data(touch_dev->client, SAIN_CHIP_REVISION, (u8*)&chip_revision, 2)<0)
	{
		sain_debug(KERN_INFO "fail to read chip revision\r\n");
		goto fail_init;
	}	
	sain_debug(KERN_INFO "sain touch chip revision id = %x\r\n", chip_revision);

	touch_dev->cap_info.chip_fw_size = 16*1024;
	if(chip_firmware_version >=0x0a && chip_firmware_version <= 0x0b)
		touch_dev->cap_info.chip_fw_size = 16*1024;

	touch_dev->cap_info.multi_fingers = SUPPORTED_FINGER_NUM;	

	/* get chip firmware version */
	if (ts_read_data(touch_dev->client, SAIN_FIRMWARE_VERSION, (u8*)&chip_firmware_version, 2)<0) goto fail_init;
	sain_debug(KERN_INFO "sain touch chip firmware version = %x\r\n", chip_firmware_version);

#if	TOUCH_ONESHOT_UPGRADE
	chip_reg_data_version = 0xffff;
	if(chip_revision >= 0x0a && chip_firmware_version >= 0x68)
	{
		if (ts_read_data(touch_dev->client, SAIN_DATA_VERSION_REG, (u8*)&chip_reg_data_version, 2)<0) goto fail_init;
		sain_debug("touch reg data version = %d\r\n", chip_reg_data_version);
	}

	if(ts_check_need_upgrade(chip_firmware_version, chip_reg_data_version)==true)
	{
		sain_debug(KERN_INFO "start upgrade firmware\n");
		ts_upgrade_firmware(touch_dev, &m_firmware_data[2], touch_dev->cap_info.chip_fw_size);
		mdelay(10);  

		// must h/w reset (cold reset) and mdelay(500); 
		ts_power_control (0);	//power off
		mdelay(250);	
		ts_power_control (1);	//power on	
		mdelay(500);

		/* get chip revision id */
		if (ts_read_data(touch_dev->client, SAIN_CHIP_REVISION, (u8*)&chip_revision, 2)<0)
		{
			sain_debug(KERN_INFO "fail to read chip revision\r\n");
			goto fail_init;
		}		
		sain_debug(KERN_INFO "sain touch chip revision id = %x\r\n", chip_revision);

		/* get chip firmware version */
		if (ts_read_data(touch_dev->client, SAIN_FIRMWARE_VERSION, (u8*)&chip_firmware_version, 2)<0) goto fail_init;		
		sain_debug(KERN_INFO "sain touch chip renewed firmware version = %x\r\n", chip_firmware_version);


		if(chip_revision < 0x0a || chip_firmware_version < 0x68)
		{
			// h/w calibration
			if (ts_write_reg(touch_dev->client, SAIN_TOUCH_MODE, 0x07)!=I2C_SUCCESS) goto fail_init;
			if (ts_write_cmd(touch_dev->client, SAIN_CALIBRATE_CMD)!=I2C_SUCCESS)	goto fail_init;
			if (ts_write_cmd(touch_dev->client, SAIN_REST_CMD)!=I2C_SUCCESS)	goto fail_init;

			mdelay(3000); 
			//<----------------------------------------------------------

			if (ts_write_reg(touch_dev->client, SAIN_TOUCH_MODE, 0x00)!=I2C_SUCCESS) goto fail_init;
			if (ts_write_cmd(touch_dev->client, SAIN_REST_CMD)!=I2C_SUCCESS) goto fail_init;
		}

	}
#endif



	if(chip_revision >= 0x0a && chip_firmware_version >= 0x68)
	{
		if (ts_read_data(touch_dev->client, SAIN_DATA_VERSION_REG, (u8*)&chip_reg_data_version, 2)<0) goto fail_init;
		sain_debug("touch reg data version = %d\r\n", chip_reg_data_version);

		if(chip_reg_data_version < m_reg_data[SAIN_DATA_VERSION_REG].reg_val)
		{
			sain_debug("write new reg data( %d < %d)\r\n", chip_reg_data_version, m_reg_data[SAIN_DATA_VERSION_REG].reg_val);
			for(i=0; i < MAX_REG_COUNT; i++)
			{
				if(m_reg_data[i].valid == 1)
				{
					if(ts_write_reg(touch_dev->client, (u16)i, (u16)(m_reg_data[i].reg_val))!=I2C_SUCCESS) goto fail_init;
					if(i == SAIN_TOTAL_NUMBER_OF_X || i == SAIN_TOTAL_NUMBER_OF_Y ||i == SAIN_AFE_FREQUENCY)	mdelay(50);	//for clear hw calibration bit
					if(ts_read_data(touch_dev->client, (u16)i, (u8*)&stmp, 2)<0) goto fail_init;
					if(memcmp((char*)&m_reg_data[i].reg_val, (char*)&stmp, 2)!=0)		//if(m_reg_data[i].reg_val != stmp)
						sain_debug(KERN_WARNING "register data is different. (addr = 0x%02X , %d != %d)\r\n", i, m_reg_data[i].reg_val, stmp);					
				}
			}
			sain_debug("done new reg data( %d < %d)\r\n", chip_reg_data_version, m_reg_data[SAIN_DATA_VERSION_REG].reg_val);		
			if (ts_write_cmd(touch_dev->client, SAIN_SAVE_STATUS_CMD)!=I2C_SUCCESS) goto fail_init;			
			mdelay(1000);	// for fusing eeprom
		}
		if (ts_read_data(touch_dev->client, SAIN_EEPROM_INFO_REG, (u8*)&chip_eeprom_info, 2)<0) goto fail_init;

		sain_debug("touch eeprom info = 0x%04X\r\n", chip_eeprom_info);

		if(sain_bit_test(chip_eeprom_info, 0))		// hw calibration bit
		{
			// h/w calibration
			if (ts_write_reg(touch_dev->client, SAIN_TOUCH_MODE, 0x07)!=I2C_SUCCESS) goto fail_init;
			if (ts_write_cmd(touch_dev->client, SAIN_CALIBRATE_CMD)!=I2C_SUCCESS)	goto fail_init;
			if (ts_write_cmd(touch_dev->client, SAIN_REST_CMD)!=I2C_SUCCESS)	goto fail_init;
			mdelay(1); 		
			ts_write_cmd(touch_dev->client, SAIN_CLEAR_INT_STATUS_CMD);
			// wait for h/w calibration
			do{
				mdelay(1000); 
				if (ts_read_data(touch_dev->client, SAIN_EEPROM_INFO_REG, (u8*)&chip_eeprom_info, 2)<0) goto fail_init;
				sain_debug("touch eeprom info = 0x%04X\r\n", chip_eeprom_info);
				if(!sain_bit_test(chip_eeprom_info, 0))	break;
			}while(1);
			//<----------------------------------------------------------
			if (ts_write_reg(touch_dev->client, SAIN_TOUCH_MODE, 0x00)!=I2C_SUCCESS) goto fail_init;
			if (ts_write_cmd(touch_dev->client, SAIN_REST_CMD)!=I2C_SUCCESS) goto fail_init;
			if (ts_write_cmd(touch_dev->client, SAIN_SAVE_STATUS_CMD)!=I2C_SUCCESS) goto fail_init;
			mdelay(1000);	// for fusing eeprom
			if (ts_write_cmd(touch_dev->client, SAIN_REST_CMD)!=I2C_SUCCESS) goto fail_init;			
		}
	}

	touch_dev->cap_info.chip_revision = (u16)chip_revision;
	touch_dev->cap_info.chip_firmware_version = (u16)chip_firmware_version;
	touch_dev->cap_info.chip_reg_data_version = (u16)chip_reg_data_version;

	/* initialize */	
	if (ts_write_reg(touch_dev->client, SAIN_X_RESOLUTION, (u16)(SetMaxX))!=I2C_SUCCESS) goto fail_init;
	if (ts_write_reg(touch_dev->client, SAIN_Y_RESOLUTION, (u16)(SetMaxY))!=I2C_SUCCESS) goto fail_init;

	if (ts_read_data(touch_dev->client, SAIN_X_RESOLUTION, (u8*)&CurMaxX, 2)<0) goto fail_init;
	sain_debug_c("touch max x = %d\r\n", CurMaxX);
	if (ts_read_data(touch_dev->client, SAIN_Y_RESOLUTION, (u8*)&CurMaxY, 2)<0) goto fail_init;
	sain_debug_c("touch max y = %d\r\n", CurMaxY);    

	touch_dev->cap_info.MinX = (u32)0;
	touch_dev->cap_info.MinY = (u32)0;
	touch_dev->cap_info.MaxX = (u32)CurMaxX;
	touch_dev->cap_info.MaxY = (u32)CurMaxY;

	if(touch_dev->cap_info.chip_revision >= 0x0a && touch_dev->cap_info.chip_firmware_version >= 0x75)
	{
		if (ts_write_reg(touch_dev->client, SAIN_SUPPORTED_FINGER_NUM, (u16)SupportedFingerNum)!=I2C_SUCCESS) goto fail_init;
		if (ts_read_data(touch_dev->client, SAIN_SUPPORTED_FINGER_NUM, (u8*)&SupportedFingerNum, 2)<0) goto fail_init;
		sain_debug_c("supported finger num = %d\r\n", SupportedFingerNum); 
	}

	touch_dev->cap_info.Orientation = 0;
	//touch_dev->cap_info.Orientation = TOUCH_XY_SWAP + TOUCH_H_FLIP;	 	

	touch_dev->cap_info.gesture_support = 0;
	touch_dev->cap_info.multi_fingers = SupportedFingerNum;

	sain_debug_c("set other configuration\r\n");

	reg_val = TOUCH_MODE;
	if (ts_write_reg(touch_dev->client, SAIN_TOUCH_MODE, reg_val)!=I2C_SUCCESS)	goto fail_init;		

	// soft calibration
	if (ts_write_cmd(touch_dev->client, SAIN_CALIBRATE_CMD)!=I2C_SUCCESS)		goto fail_init;

	reg_val = 0;
	sain_bit_set(reg_val, BIT_PT_CNT_CHANGE);			
	sain_bit_set(reg_val, BIT_DOWN);
	sain_bit_set(reg_val, BIT_MOVE);
	sain_bit_set(reg_val, BIT_UP);

	if(SUPPORTED_BUTTON_NUM > 0)		sain_bit_set(reg_val, BIT_ICON_EVENT);

	touch_dev->chip_int_mask = reg_val;

	if (ts_write_reg(touch_dev->client, SAIN_INT_ENABLE_FLAG, touch_dev->chip_int_mask)!=I2C_SUCCESS)	goto fail_init;		
	//---------------------------------------------------------------------
	// read garbage data
	for(i=0; i<10; i++)
	{
		ts_write_cmd(touch_dev->client, SAIN_CLEAR_INT_STATUS_CMD);
	}

#if	SAIN_ESD_TIMER_INTERVAL	
	if(touch_dev->cap_info.chip_revision >= 0x0a && touch_dev->cap_info.chip_firmware_version >= 0x69)
	{
		if (ts_write_reg(touch_dev->client, SAIN_PERIODICAL_INTERRUPT_INTERVAL, SAIN_SCAN_RATE_HZ)!=I2C_SUCCESS)	goto fail_init;			
		sema_init(&touch_dev->esd_lock, 1);
		touch_dev->use_esd_timer = 1;
		ts_esd_timer_init(touch_dev);	
		ts_esd_timer_start(SAIN_ESD_TIMER_INTERVAL*1, touch_dev);	
		sain_debug_c(KERN_INFO " ts_esd_timer_start\n");	
	}
	else
		touch_dev->use_esd_timer = 0;
#endif


	sain_debug("successfully initialized\r\n");
	return true;

fail_init:
	sain_debug("failed initiallize\r\n");
	if (ts_write_cmd(touch_dev->client, SAIN_REST_CMD)!=I2C_SUCCESS);
	return false;

}


#if (TOUCH_MODE == 1)
static void	sain_report_data(sain_touch_dev *touch_dev, int id)
{
	int i;
//	u32 x, y, width;
	u32 x, y;
	u32 tmp;
#ifdef TOUCH_ICS_UPDATE	
	uint8_t reported = 0;
#endif
	
	if(id >= SUPPORTED_FINGER_NUM || id < 0)
	{
		return;
	}

	x = touch_dev->touch_info.coord[id].x;
	y = touch_dev->touch_info.coord[id].y;

	/* transformation from touch to screen orientation */
	if (touch_dev->cap_info.Orientation & TOUCH_V_FLIP)
	{
		y = touch_dev->cap_info.MaxY + touch_dev->cap_info.MinY - y;			               
	}
	if (touch_dev->cap_info.Orientation & TOUCH_H_FLIP)
	{
		x = touch_dev->cap_info.MaxX + touch_dev->cap_info.MinX - x;			               
	}
	if (touch_dev->cap_info.Orientation & TOUCH_XY_SWAP)
	{					
		swap_v(x, y, tmp);
	}
	touch_dev->reported_touch_info.coord[id].x = x;
	touch_dev->reported_touch_info.coord[id].y = y;				
	touch_dev->reported_touch_info.coord[id].width = touch_dev->touch_info.coord[id].width;
	touch_dev->reported_touch_info.coord[id].sub_status = touch_dev->touch_info.coord[id].sub_status;


	for(i=0; i< SUPPORTED_FINGER_NUM; i++)
	{
		if(sain_bit_test(touch_dev->reported_touch_info.coord[i].sub_status, SUB_BIT_EXIST)
				||sain_bit_test(touch_dev->reported_touch_info.coord[i].sub_status, SUB_BIT_DOWN)
				||sain_bit_test(touch_dev->reported_touch_info.coord[i].sub_status, SUB_BIT_MOVE))			
		{

			if(touch_dev->reported_touch_info.coord[i].width == 0)	touch_dev->reported_touch_info.coord[i].width = 5;
#ifdef TOUCH_ICS_UPDATE
			input_mt_slot(touch_dev->input_dev,i);			
#ifdef	TOUCH_TRACKING_ID
			if(sain_bit_test(touch_dev->reported_touch_info.coord[i].sub_status, SUB_BIT_DOWN))
			{
				input_report_abs(touch_dev->input_dev,ABS_MT_TRACKING_ID,i);
			}
#endif
#endif
			input_report_abs(touch_dev->input_dev, ABS_MT_TOUCH_MAJOR, (u32)touch_dev->reported_touch_info.coord[i].width);
			input_report_abs(touch_dev->input_dev, ABS_MT_WIDTH_MAJOR, (u32)touch_dev->reported_touch_info.coord[i].width);					
			input_report_abs(touch_dev->input_dev, ABS_MT_POSITION_X, touch_dev->reported_touch_info.coord[i].x);
			input_report_abs(touch_dev->input_dev, ABS_MT_POSITION_Y, touch_dev->reported_touch_info.coord[i].y);
			input_mt_sync(touch_dev->input_dev);	
#ifdef TOUCH_ICS_UPDATE
			reported++;	
#endif
		}
		else if(sain_bit_test(touch_dev->reported_touch_info.coord[i].sub_status, SUB_BIT_UP))			
		{
#ifdef TOUCH_ICS_UPDATE
			input_mt_slot(touch_dev->input_dev,i);			
#ifdef	TOUCH_TRACKING_ID
			input_report_abs(touch_dev->input_dev,ABS_MT_TRACKING_ID,-1);
#endif
#else			
			input_report_abs(touch_dev->input_dev, ABS_MT_TOUCH_MAJOR, 0);
			input_report_abs(touch_dev->input_dev, ABS_MT_WIDTH_MAJOR, 0);	
			input_report_abs(touch_dev->input_dev, ABS_MT_POSITION_X, touch_dev->reported_touch_info.coord[i].x);
			input_report_abs(touch_dev->input_dev, ABS_MT_POSITION_Y, touch_dev->reported_touch_info.coord[i].y);
#endif			
			input_mt_sync(touch_dev->input_dev);	
			touch_dev->reported_touch_info.coord[i].sub_status = 0;
		}
		else
			touch_dev->reported_touch_info.coord[i].sub_status = 0;
	}
#ifdef TOUCH_ICS_UPDATE
	input_report_key(touch_dev->input_dev, BTN_TOUCH, !!reported);
	if (reported){
		input_sync(touch_dev->input_dev);
		}
#else
	input_sync(touch_dev->input_dev);
#endif
}
#endif	// TOUCH_MODE == 1


#if USE_THREAD_METHOD	
static int sain_touch_thread(void *pdata)
#else
static void sain_touch_work(struct work_struct *work)
#endif	// USE_THREAD_METHOD
{
	bool read_coord_continued;
	int i;
	//u32 x, y, width;
	//u32 tmp;
	u8 reported = false;
#if USE_THREAD_METHOD
	sain_touch_dev *touch_dev = (sain_touch_dev*)pdata;   
#if 0
	struct task_struct *tsk = current;
	//struct sched_param param = { .sched_priority = MAX_RT_PRIO-1 };
	struct sched_param param = { .sched_priority = 1 };

	sched_setscheduler(tsk, SCHED_FIFO, &param);
#endif
	sain_debug(KERN_INFO "touch thread started.. \r\n");
#else
	sain_touch_dev *touch_dev = container_of(work, sain_touch_dev, work);
#endif


#if USE_THREAD_METHOD	
	for (;;)
	{
		down(&touch_dev->update_lock);
#endif	// USE_THREAD_METHOD

		sain_debug_c("sain_touch_thread : semaphore signalled\r\n");

#if	SAIN_ESD_TIMER_INTERVAL	
		if(touch_dev->use_esd_timer)
		{
			down(&touch_dev->esd_lock);
			ts_esd_timer_stop(touch_dev);	
		}
#endif				
		read_coord_continued = true;
		do
		{
			if (ts_read_coord(touch_dev)==false)
			{
				sain_debug(KERN_WARNING "couldn't read touch_dev sample\r\n");
				goto continue_read_samples;
			}

#if (TOUCH_MODE == 1)
			// invalid : maybe periodical repeated int.
			if(touch_dev->touch_info.status == 0x0)			
				goto continue_read_samples;
#else
			// invalid : maybe periodical repeated int.
			if(touch_dev->touch_info.status == 0x0 && touch_dev->touch_info.finger_cnt == 100)			
				goto continue_read_samples;
#endif	//TOUCH_MODE == 1
			reported = false;

			if(INPUT_ENABLE && sain_bit_test(touch_dev->touch_info.status, BIT_ICON_EVENT))
			{

				for(i=0; i<SUPPORTED_BUTTON_NUM; i++)
				{
					if(sain_bit_test(touch_dev->icon_event_reg, (BIT_O_ICON0_DOWN+i)))
					{
						touch_dev->button[i] = ICON_BUTTON_DOWN;
						input_report_key(touch_dev->input_dev, BUTTON_MAPPING_KEY[i], 1);					
						reported = true;						
						sain_debug_c("button down = %d \r\n", i);
					}
				}

				for(i=0; i<SUPPORTED_BUTTON_NUM; i++)
				{
					if(sain_bit_test(touch_dev->icon_event_reg, (BIT_O_ICON0_UP+i)))
					{
						touch_dev->button[i] = ICON_BUTTON_UP;	
						input_report_key(touch_dev->input_dev, BUTTON_MAPPING_KEY[i], 0);					
						reported = true;		
						sain_debug_c("button up = %d \r\n", i);
					}
				}
			}

			// if button press or up event occured...
			if(reported == true)
			{
#if (TOUCH_MODE == 1)
				//input_sync(touch_dev->input_dev);
				for(i=0; i< SUPPORTED_FINGER_NUM; i++)
				{

					if(sain_bit_test(touch_dev->reported_touch_info.coord[i].sub_status, SUB_BIT_EXIST))			
					{
#ifdef TOUCH_ICS_UPDATE
						input_mt_slot(touch_dev->input_dev,i);			
#else
						input_report_abs(touch_dev->input_dev, ABS_MT_TOUCH_MAJOR, 0);
						input_report_abs(touch_dev->input_dev, ABS_MT_WIDTH_MAJOR, 0);	
						input_report_abs(touch_dev->input_dev, ABS_MT_POSITION_X, touch_dev->reported_touch_info.coord[i].x);
						input_report_abs(touch_dev->input_dev, ABS_MT_POSITION_Y, touch_dev->reported_touch_info.coord[i].y);												
						input_mt_sync(touch_dev->input_dev);	
#endif
					}
					touch_dev->reported_touch_info.coord[i].sub_status = 0;
				}
#ifdef TOUCH_ICS_UPDATE
				input_report_key(touch_dev->input_dev, BTN_TOUCH, SAIN_NOTOUCH);	
#endif					
				input_sync(touch_dev->input_dev);
				//goto continue_read_samples;
			}
#else					
			for(i=0; i< SUPPORTED_FINGER_NUM; i++)
			{

				if(sain_bit_test(touch_dev->reported_touch_info.coord[i].sub_status, SUB_BIT_EXIST))			
				{
#ifdef TOUCH_ICS_UPDATE
					input_mt_slot(touch_dev->input_dev,i);			
#endif
					//input_report_abs(touch_dev->input_dev,ABS_MT_TRACKING_ID,i);
					//input_report_abs(touch_dev->input_dev, ABS_MT_TOUCH_MAJOR, 0);
					input_report_abs(touch_dev->input_dev, ABS_MT_WIDTH_MAJOR, 0);	
					input_report_abs(touch_dev->input_dev, ABS_MT_POSITION_X, touch_dev->reported_touch_info.coord[i].x);
					input_report_abs(touch_dev->input_dev, ABS_MT_POSITION_Y, touch_dev->reported_touch_info.coord[i].y);												
					input_mt_sync(touch_dev->input_dev);	
				}
			}
			memset(&touch_dev->reported_touch_info, 0x0, sizeof(_ts_sain_point_info));				
#ifdef TOUCH_ICS_UPDATE
			input_report_key(touch_dev->input_dev, BTN_TOUCH, SAIN_NOTOUCH);	
#endif			
			input_sync(touch_dev->input_dev);
			udelay(100);				
			goto continue_read_samples;				
		}


		if (touch_dev->touch_info.finger_cnt > SUPPORTED_FINGER_NUM)
			touch_dev->touch_info.finger_cnt = SUPPORTED_FINGER_NUM;

		if(!sain_bit_test(touch_dev->touch_info.status, BIT_PT_EXIST))
		{

			for(i=0; i< SUPPORTED_FINGER_NUM; i++)
			{
				if(sain_bit_test(touch_dev->reported_touch_info.coord[i].sub_status, SUB_BIT_EXIST))
				{
#ifdef TOUCH_ICS_UPDATE
					input_mt_slot(touch_dev->input_dev,i);			
#endif					
					//input_report_abs(touch_dev->input_dev,ABS_MT_TRACKING_ID,i);
					//input_report_abs(touch_dev->input_dev, ABS_MT_TOUCH_MAJOR, 0);
					input_report_abs(touch_dev->input_dev, ABS_MT_WIDTH_MAJOR, 0);	
					input_report_abs(touch_dev->input_dev, ABS_MT_POSITION_X, touch_dev->reported_touch_info.coord[i].x);
					input_report_abs(touch_dev->input_dev, ABS_MT_POSITION_Y, touch_dev->reported_touch_info.coord[i].y);						
					input_mt_sync(touch_dev->input_dev);	
				}
			}				
			memset(&touch_dev->reported_touch_info, 0x0, sizeof(_ts_sain_point_info));					
#ifdef TOUCH_ICS_UPDATE
			input_report_key(touch_dev->input_dev, BTN_TOUCH, SAIN_NOTOUCH);	
#endif			
			input_sync(touch_dev->input_dev);
			goto continue_read_samples;
		}


		for(i=0; i< SUPPORTED_FINGER_NUM; i++)
		{

			if(sain_bit_test(touch_dev->touch_info.coord[i].sub_status, SUB_BIT_DOWN)
					|| sain_bit_test(touch_dev->touch_info.coord[i].sub_status, SUB_BIT_MOVE)
					|| sain_bit_test(touch_dev->touch_info.coord[i].sub_status, SUB_BIT_EXIST))				
			{
				x = touch_dev->touch_info.coord[i].x;
				y = touch_dev->touch_info.coord[i].y;

				/* transformation from touch to screen orientation */
				if (touch_dev->cap_info.Orientation & TOUCH_V_FLIP)
				{
					y = touch_dev->cap_info.MaxY + touch_dev->cap_info.MinY - y;			               
				}
				if (touch_dev->cap_info.Orientation & TOUCH_H_FLIP)
				{
					x = touch_dev->cap_info.MaxX + touch_dev->cap_info.MinX - x;			               
				}
				if (touch_dev->cap_info.Orientation & TOUCH_XY_SWAP)
				{					
					swap_v(x, y, tmp);
				}
				touch_dev->touch_info.coord[i].x = x;
				touch_dev->touch_info.coord[i].y = y;

				sain_debug_c("finger [%02d] x = %d, y = %d \r\n", i, x, y);


				//input_report_abs(touch_dev->input_dev,ABS_MT_TRACKING_ID,i);
				if(touch_dev->touch_info.coord[i].width == 0)	touch_dev->touch_info.coord[i].width = 5;
#ifdef TOUCH_ICS_UPDATE
				input_mt_slot(touch_dev->input_dev,i);			
#endif				
				//input_report_abs(touch_dev->input_dev, ABS_MT_TOUCH_MAJOR, (u32)touch_dev->touch_info.coord[i].width);
				input_report_abs(touch_dev->input_dev, ABS_MT_WIDTH_MAJOR, (u32)touch_dev->touch_info.coord[i].width);					
				input_report_abs(touch_dev->input_dev, ABS_MT_POSITION_X, x);
				input_report_abs(touch_dev->input_dev, ABS_MT_POSITION_Y, y);
				input_mt_sync(touch_dev->input_dev);	


			}
			else if(sain_bit_test(touch_dev->touch_info.coord[i].sub_status, SUB_BIT_UP))			
			{
				sain_debug_c("finger [%02d] up \r\n", i);
				memset(&touch_dev->touch_info.coord[i], 0x0, sizeof(_ts_sain_coord));	
#ifdef TOUCH_ICS_UPDATE
				input_mt_slot(touch_dev->input_dev,i);			
#endif				
				//input_report_abs(touch_dev->input_dev, ABS_MT_TRACKING_ID,i);
				//input_report_abs(touch_dev->input_dev, ABS_MT_TOUCH_MAJOR, 0);
				input_report_abs(touch_dev->input_dev, ABS_MT_WIDTH_MAJOR, 0);	
				input_report_abs(touch_dev->input_dev, ABS_MT_POSITION_X, touch_dev->reported_touch_info.coord[i].x);
				input_report_abs(touch_dev->input_dev, ABS_MT_POSITION_Y, touch_dev->reported_touch_info.coord[i].y);
				input_mt_sync(touch_dev->input_dev);

			}		

			else
				memset(&touch_dev->touch_info.coord[i], 0x0, sizeof(_ts_sain_coord));	


		}		        
		memcpy((char*)&touch_dev->reported_touch_info, (char*)&touch_dev->touch_info, sizeof(_ts_sain_point_info));	
#ifdef TOUCH_ICS_UPDATE
		input_report_key(touch_dev->input_dev, BTN_TOUCH, SAIN_TOUCH);	
#endif		
		input_sync(touch_dev->input_dev);

#endif	// TOUCH_MODE == 1
continue_read_samples:

		//check_interrupt_pin, if high, enable int & wait signal
		if (gpio_get_value(touch_dev->int_gpio_num))
		{
			read_coord_continued = false;				
			enable_irq(touch_dev->client->irq);
#if	SAIN_ESD_TIMER_INTERVAL					
			if(touch_dev->use_esd_timer)
			{
				ts_esd_timer_start(SAIN_ESD_TIMER_INTERVAL*1, touch_dev);	
				up(&touch_dev->esd_lock);				    	
			}
#endif				
			//switch_charger_mode();
		} 
		else
		{
			sain_debug_c("interrupt pin is still low, so continue read \r\n");	
		}

	}while(read_coord_continued);
#if USE_THREAD_METHOD	
}
return 0;
#endif
}

typedef enum touch_status{
	TOUCH_POWEROFF,
	TOUCH_POWERON,
	TOUCH_BOOTDONE
}touch_status;

static int touch_power = TOUCH_BOOTDONE;

#ifdef CONFIG_HAS_EARLYSUSPEND

static void sain_early_suspend(struct early_suspend *h)
{
	sain_debug(KERN_INFO "early_suspend++\n");
	if(touch_dev == NULL)	return;

#if	SAIN_ESD_TIMER_INTERVAL	
	if(touch_dev->use_esd_timer)
	{				
		down(&touch_dev->esd_lock);	
		ts_write_reg(touch_dev->client, SAIN_PERIODICAL_INTERRUPT_INTERVAL, 0);
		ts_esd_timer_stop(touch_dev);
		up(&touch_dev->esd_lock);		
		sain_debug(KERN_INFO " ts_esd_timer_stop\n");			
	}
#endif
	//gpio_tlmm_config(GPIO_CFG(GPIO_TOUCH_PIN_NUMn, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	//gpio_set_value(GPIO_TOUCH_PIN_NUMn, 1);	

	disable_irq_nosync(touch_dev->client->irq);
#if (!USE_THREAD_METHOD)
	flush_workqueue(sain_workqueue);
#endif

	INPUT_ENABLE = 0;

	sain_debug(KERN_INFO "early_suspend--\n");
	return;
}


static int sain_suspend(struct device *dev) {

	sain_debug(KERN_INFO "suspend++\n");
	cancel_work_sync(&touch_dev->tmr_work);

	if(touch_dev->use_esd_timer)
	down(&touch_dev->esd_lock);
	ts_power_control (0);	//power off
	touch_power = TOUCH_POWEROFF;
	usleep(250*1000);
	sain_debug(KERN_INFO "suspend--\n");
	return 0;
}

static int sain_resume(struct device *dev) {

	sain_debug(KERN_INFO "resume++\n");
	touch_power = TOUCH_POWERON;
	ts_power_control(1);
	if(touch_dev->use_esd_timer)
	up(&touch_dev->esd_lock);
	sain_debug(KERN_INFO "resume--\n");
	return 0;
}


static void sain_late_resume(struct early_suspend *h)
{
	int ret;
	int count = 4, retry_cnt = 4;

	sain_debug(KERN_INFO "late_resume++\r\n");
	if(touch_dev == NULL)	return;
 
	if(touch_dev->use_esd_timer)
		down(&touch_dev->esd_lock);
	if(touch_power != TOUCH_BOOTDONE)
	{
		int version_check = false;
		u16 chip_firmware_version = 0x00;

		usleep(350*1000);		
		do{
			ret = ts_read_data(touch_dev->client, SAIN_FIRMWARE_VERSION, (u8*)&chip_firmware_version, 2);
			sain_debug("chip_firmware version: %02x\n",chip_firmware_version);
			if(chip_firmware_version > 0x01) {
				version_check = true;
				break;
			}
			usleep(100*1000);
		}while(--retry_cnt && (version_check == false));
		if (version_check == false)
			goto fail_resume;	

		sain_debug(KERN_INFO "calibrate touch.\n");
		while (count > 0){
			if(ts_mini_init_touch(touch_dev)) 
			{
				sain_debug(KERN_INFO "init_success\n");
				break;
			}
			count--;
			usleep(100*1000);
			if(count == 0)
				goto fail_resume;
		}
	}
	if(touch_dev->use_esd_timer)
		up(&touch_dev->esd_lock);


	if(touch_power == TOUCH_BOOTDONE)	
	{
#if	SAIN_ESD_TIMER_INTERVAL	
		if(touch_dev->use_esd_timer)
		{
			down(&touch_dev->esd_lock);
			if (ts_write_reg(touch_dev->client, SAIN_PERIODICAL_INTERRUPT_INTERVAL, SAIN_SCAN_RATE_HZ)!=I2C_SUCCESS)	
				goto fail_resume;
			ts_esd_timer_start(SAIN_ESD_TIMER_INTERVAL*1, touch_dev);
			up(&touch_dev->esd_lock);
		}
#endif		
	}
	//gpio_tlmm_config(GPIO_CFG(GPIO_TOUCH_PIN_NUMn, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);


	INPUT_ENABLE = 1;
	sain_debug(KERN_INFO "late_resume--\n");
	ts_write_cmd(touch_dev->client, SAIN_CLEAR_INT_STATUS_CMD);	
	touch_power = TOUCH_BOOTDONE;
	enable_irq(touch_dev->client->irq);
	return;	
fail_resume:
	sain_debug(KERN_ERR "failed to wakeup\n");
	ts_power_control (0);	//power off
	mdelay(250);		//must
	ts_power_control (1);	//power on	
	if(touch_dev->use_esd_timer)
		up(&touch_dev->esd_lock);
	enable_irq(touch_dev->client->irq);
	return;
}

#endif	// CONFIG_HAS_EARLYSUSPEND



#if	USE_UPDATE_SYSFS

static ssize_t sain_show_info(struct device *dev,
		struct device_attribute *attr,
		const char *buf)
{
	sain_touch_dev* touch_dev = dev_get_drvdata(dev);
	int count = 0;
	count += sprintf(buf, "sain ts chip info : revision = 0x%04x, fw version = 0x%04x, reg data version = 0x%04x\n",
			touch_dev->cap_info.chip_revision, touch_dev->cap_info.chip_firmware_version, touch_dev->cap_info.chip_reg_data_version);
	return count;
}

static ssize_t sain_update_fw(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	sain_touch_dev* touch_dev = dev_get_drvdata(dev);
	char *filename_ptr;
	int ret = 0;
	const struct firmware *fw = NULL;

	filename_ptr = kzalloc(count*2, GFP_KERNEL);

	if (!filename_ptr) {
		dev_err(dev, "cannot alloc memory\n");
		return -EINVAL;
	}

	ret = sscanf(buf, "%s", filename_ptr);
	if (ret != 1) {
		dev_err(dev, "invalid parameter\n");
		goto fail_fw;
	}

	ret = request_firmware(&fw, filename_ptr, dev);
	if (ret) {
		dev_err(dev, "cannot open firmware %s\n", filename_ptr);
		goto fail_fw;
	}

	if(fw->size != touch_dev->cap_info.chip_fw_size)
	{
		dev_err(dev, "invalid file size( %d != %d)\n", fw->size , touch_dev->cap_info.chip_fw_size);
		goto fail_upgrade_fw;
	}

	disable_irq(touch_dev->client->irq);
#if	SAIN_ESD_TIMER_INTERVAL	
	if(touch_dev->use_esd_timer)
	{
		ts_esd_timer_stop(touch_dev);
	}
#endif					
	dev_dbg(dev, "update firmware\n");
	ts_upgrade_firmware(touch_dev, fw, fw->size);

	dev_dbg(dev, "reset touch chip\n");	
	// must h/w reset (cold reset) and mdelay(500); 
	ts_power_control (0);	//power off
	mdelay(250);		//must
	ts_power_control (1);	//power on	
	mdelay(500);

	dev_dbg(dev, "initialize touch chip, do not touch.\n");	
	ts_init_touch(touch_dev);
	enable_irq(touch_dev->client->irq);
	release_firmware(fw);
	dev_dbg(dev, "finished update.\n");		
	return count;
fail_upgrade_fw:
	release_firmware(fw);	
fail_fw:
	kfree(filename_ptr);
	return -1;

}

static DEVICE_ATTR(sain_fw_manager, S_IRUGO|S_IWUGO, sain_show_info, sain_update_fw);

static struct attribute *sain_attrs[] = {
	&dev_sain_fw_manager.attr,
	NULL
};

static const struct attribute_group sain_attr_group = 
{
	.attrs = sain_attrs,
};

#endif	// USE_UPDATE_SYSFS

static void init_hw(void)
{
	int rc; 
//	unsigned gpioConfig;
//	unsigned gpioConfig_clk, gpioConfig_data;
	struct regulator *vreg_power_1_8;

	gpio_tlmm_config(GPIO_CFG(GPIO_TOUCH_PIN_NUMn, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_set_value(GPIO_TOUCH_PIN_NUMn, 1);	
	gpio_tlmm_config(GPIO_CFG(GPIO_TOUCH_RESETn, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_set_value(GPIO_TOUCH_RESETn, 1);	

	gpio_tlmm_config(GPIO_CFG(GPIO_TOUCH_ID, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);

	// Power On, AVDD
	vreg_power_1_8 = regulator_get(NULL, "8058_lvs0");
	if (IS_ERR(vreg_power_1_8))
	{
		rc = PTR_ERR(vreg_power_1_8);
//		sain_debug(KERN_ERR "%s: regulator get of %s failed (%d)\n",
//				__func__, vreg_power_1_8, rc);
	}

	rc = regulator_enable(vreg_power_1_8);
	//sain_debug("vreg_power_1_8 (8058_lvs0): regulator_enable return:  %d \n", rc);
	gpio_tlmm_config(GPIO_CFG(GPIO_TOUCH_LDO_EN, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_set_value(GPIO_TOUCH_LDO_EN, 1);	

	msleep(800);
}

static int sain_touch_probe(struct i2c_client *client, const struct i2c_device_id *i2c_id)
{
	int ret;
	//sain_touch_dev* touch_dev;
	int i;
	int rc;

	sain_debug("sain_touch_probe \n");
	sain_debug_c("sain_touch_probe+ \r\n");
	sain_debug_c("i2c check function \r\n");
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		sain_debug(KERN_ERR "error : not compatible i2c function \r\n");
		ret = -ENODEV;
		goto err_check_functionality;
	}

	sain_debug_c("touch data alloc \r\n");	
	touch_dev = kzalloc(sizeof(sain_touch_dev), GFP_KERNEL);
	if (!touch_dev) {
		sain_debug(KERN_ERR "unabled to allocate touch data \r\n");	
		ret = -ENOMEM;
		goto err_alloc_dev_data;
	}
	touch_dev->client = client;
	i2c_set_clientdata(client, touch_dev);

#if USE_THREAD_METHOD		
	sema_init(&touch_dev->update_lock, 0);
#else
	INIT_WORK(&touch_dev->work, sain_touch_work);
#endif	// USE_THREAD_METHOD

	if(I2C_SMBUS_BLOCK_MAX < sizeof(_ts_sain_point_info))
	{
		sain_debug(KERN_WARNING "max size error : i2c max size = %d, sain packet size = %d \r\n", I2C_SMBUS_BLOCK_MAX, sizeof(_ts_sain_point_info));	
		sain_debug(KERN_WARNING "must modify I2C_SMBUS_BLOCK_MAX field in include/linux/i2c.h\r\n");
	}

#if USE_THREAD_METHOD		

	if(TOUCH_DBG_ENABLE) sain_debug("touch thread create \r\n");	
	touch_dev->task = kthread_create(sain_touch_thread, touch_dev, "sain_touch_thread");
	if(touch_dev->task == NULL)
	{
		sain_debug(KERN_ERR "unabled to create touch thread \r\n");
		ret = -1;
		goto err_kthread_create_failed;
	}	
#else
	sain_workqueue = create_singlethread_workqueue("sain_workqueue");
	if (!sain_workqueue)
	{
		sain_debug(KERN_ERR "unabled to create touch thread \r\n");
		ret = -1;
		goto err_kthread_create_failed;
	}
#endif

#if	SAIN_ESD_TIMER_INTERVAL
	INIT_WORK(&touch_dev->tmr_work, sain_touch_tmr_work);

	sain_tmr_workqueue = create_singlethread_workqueue("sain_tmr_workqueue");
	if (!sain_tmr_workqueue)
	{
		sain_debug(KERN_ERR "unabled to create touch tmr work queue \r\n");
		ret = -1;
		goto err_kthread_create_failed;
	}
#endif

	//wake_up_process( touch_dev->task );
	sain_debug_c("allocate input device \r\n");
	touch_dev->input_dev = input_allocate_device();
	if (touch_dev->input_dev == 0) {
		sain_debug(KERN_ERR "unabled to allocate input device \r\n");
		ret = -ENOMEM;
		goto err_input_allocate_device;
	}

	// Fops Device
	rc = misc_register(&touch_event);
	if (rc) {
		sain_debug("::::::::: can''t register touch_fops\n");
	}

	//initialize sain touch ic
	touch_dev->int_gpio_num = GPIO_TOUCH_PIN_NUMn;	// for upgrade	
	touch_dev->color = gpio_get_value(GPIO_TOUCH_ID);
	sain_debug("color: %d (0:black, 1:white)",touch_dev->color);

	if (touch_dev->color == black) {
		m_firmware_data = m_firmware_data_black;
	}
	else {
		m_firmware_data = m_firmware_data_white;
	}

	memset(&touch_dev->reported_touch_info, 0x0, sizeof(_ts_sain_point_info));
	ts_init_touch(touch_dev);	

	touch_dev->input_dev->name = SAIN_DRIVER_NAME;
	//touch_dev->input_dev->phys = "sain_touch/input0";	// <- for compatability
	touch_dev->input_dev->id.bustype = BUS_I2C;
	touch_dev->input_dev->id.vendor = 0x0001;

	touch_dev->input_dev->id.product = 0x0002;
	touch_dev->input_dev->id.version = 0x0100;
	touch_dev->input_dev->dev.parent = &client->dev;

	set_bit(EV_SYN, touch_dev->input_dev->evbit);
	set_bit(EV_KEY, touch_dev->input_dev->evbit);
	set_bit(EV_ABS, touch_dev->input_dev->evbit);
	set_bit(BTN_TOUCH, touch_dev->input_dev->keybit);
	set_bit(BTN_2, touch_dev->input_dev->keybit);

	set_bit(KEY_MENU, touch_dev->input_dev->keybit);
	set_bit(KEY_BACK, touch_dev->input_dev->keybit);
	set_bit(KEY_POWER, touch_dev->input_dev->keybit);
	set_bit(KEY_HOME, touch_dev->input_dev->keybit);
	set_bit(KEY_SEARCH, touch_dev->input_dev->keybit);

	set_bit(KEY_0, touch_dev->input_dev->keybit);
	set_bit(KEY_1, touch_dev->input_dev->keybit);
	set_bit(KEY_2, touch_dev->input_dev->keybit);
	set_bit(KEY_3, touch_dev->input_dev->keybit);
	set_bit(KEY_4, touch_dev->input_dev->keybit);
	set_bit(KEY_5, touch_dev->input_dev->keybit);
	set_bit(KEY_6, touch_dev->input_dev->keybit);
	set_bit(KEY_7, touch_dev->input_dev->keybit);
	set_bit(KEY_8, touch_dev->input_dev->keybit);
	set_bit(KEY_9, touch_dev->input_dev->keybit);
	set_bit(0xe3, touch_dev->input_dev->keybit); /* '*' */
	set_bit(0xe4, touch_dev->input_dev->keybit); /* '#' */

	set_bit(KEY_LEFTSHIFT, touch_dev->input_dev->keybit);
	set_bit(KEY_RIGHTSHIFT, touch_dev->input_dev->keybit);


	set_bit(KEY_LEFT, touch_dev->input_dev->keybit);
	set_bit(KEY_RIGHT, touch_dev->input_dev->keybit);
	set_bit(KEY_UP, touch_dev->input_dev->keybit);
	set_bit(KEY_DOWN, touch_dev->input_dev->keybit);
	set_bit(KEY_ENTER, touch_dev->input_dev->keybit);

	set_bit(KEY_SEND, touch_dev->input_dev->keybit);
	set_bit(KEY_END, touch_dev->input_dev->keybit);

	set_bit(KEY_VOLUMEUP, touch_dev->input_dev->keybit);
	set_bit(KEY_VOLUMEDOWN, touch_dev->input_dev->keybit);

	set_bit(KEY_CLEAR, touch_dev->input_dev->keybit);

	set_bit(KEY_CAMERA, touch_dev->input_dev->keybit);
	set_bit(KEY_DELETE, touch_dev->input_dev->keybit);	
	set_bit(KEY_WWW, touch_dev->input_dev->keybit);

	set_bit(KEY_F1, touch_dev->input_dev->keybit);
	set_bit(KEY_F2, touch_dev->input_dev->keybit);
	set_bit(KEY_F3, touch_dev->input_dev->keybit);
	set_bit(KEY_F4, touch_dev->input_dev->keybit);
	set_bit(INPUT_PROP_DIRECT, touch_dev->input_dev->propbit);

	
	if(SUPPORTED_BUTTON_NUM > 0)
	{
		for(i=0; i< SUPPORTED_BUTTON_NUM; i++)	
			set_bit(BUTTON_MAPPING_KEY[i], touch_dev->input_dev->keybit);
	}	


	if (touch_dev->cap_info.Orientation & TOUCH_XY_SWAP)
	{
		input_set_abs_params(touch_dev->input_dev, ABS_MT_POSITION_Y, touch_dev->cap_info.MinX, touch_dev->cap_info.MaxX, 0, 0);		
		input_set_abs_params(touch_dev->input_dev, ABS_MT_POSITION_X, touch_dev->cap_info.MinY, touch_dev->cap_info.MaxY, 0, 0);
	}
	else
	{
		input_set_abs_params(touch_dev->input_dev, ABS_MT_POSITION_X, touch_dev->cap_info.MinX, touch_dev->cap_info.MaxX, 0, 0);		
		input_set_abs_params(touch_dev->input_dev, ABS_MT_POSITION_Y, touch_dev->cap_info.MinY, touch_dev->cap_info.MaxY, 0, 0);
	}

	input_set_abs_params(touch_dev->input_dev, ABS_TOOL_WIDTH, 0, 255, 0, 0);				
	input_set_abs_params(touch_dev->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(touch_dev->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);

	sain_debug_c("register %s input device \r\n", touch_dev->input_dev->name);
	ret = input_register_device(touch_dev->input_dev);
	if(ret) {
		sain_debug(KERN_ERR "unable to register %s input device\r\n", touch_dev->input_dev->name);
		goto err_input_register_device;
	}

	touch_dev->int_gpio_num = GPIO_TOUCH_PIN_NUMn;

	sain_debug_c("request irq (pin = %d) \r\n", touch_dev->int_gpio_num);

	if(touch_dev->client->irq != GPIO_TOUCH_IRQ)
	{
		sain_debug(KERN_ERR "why irq doesn't match\n");
		touch_dev->client->irq = GPIO_TOUCH_IRQ;		
	}

	if (touch_dev->client->irq) {
		ret = request_irq(touch_dev->client->irq, ts_int_handler, IRQF_TRIGGER_LOW|IRQF_DISABLED, SAIN_DRIVER_NAME, touch_dev);
		if (ret) {
			sain_debug(KERN_ERR "unable to register irq.(%s)\r\n", touch_dev->input_dev->name);
			goto err_request_irq;
		}	
	}
	touch_dev->irq = touch_dev->client->irq;
	//enable_irq(touch_dev->client->irq);	// need debug
	//enable_irq(touch_dev->client->irq);	// need Debug
	dev_info(&client->dev, "sain touch probe.\r\n");

#ifdef CONFIG_HAS_EARLYSUSPEND
	//touch_dev->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN - 1;
	touch_dev->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 1;
	touch_dev->early_suspend.suspend = sain_early_suspend;
	touch_dev->early_suspend.resume = sain_late_resume;
	register_early_suspend(&touch_dev->early_suspend);
	sema_init(&touch_dev->suspend_lock, 1);
	wake_lock_init(&touch_dev->touch_wake_lock, WAKE_LOCK_SUSPEND, "touch");
	touch_dev->is_calibrated= 0; 
#endif

#if USE_THREAD_METHOD
	wake_up_process( touch_dev->task );
#endif

#if USE_UPDATE_SYSFS
	ret = sysfs_create_group(&client->dev.kobj, &sain_attr_group);
	if (ret)
	{
		sain_debug(KERN_ERR "cannot create sysfs\n");
		goto err_create_sysfs;
	}
#endif
	return 0;

err_request_irq:
	input_unregister_device(touch_dev->input_dev);
err_input_register_device:
	input_free_device(touch_dev->input_dev);
err_kthread_create_failed:	
err_input_allocate_device:	
	kfree(touch_dev);
err_alloc_dev_data:	
err_check_functionality:

#if USE_UPDATE_SYSFS
err_create_sysfs:
#endif
	return ret;
}
#ifdef FILE_CONFIG

static int ts_fops_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int ts_fops_close(struct inode *inode, struct file *file)
{
	return 0;
}

static long ts_fops_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int i;
	//int regVal;
	int returnVal = -1;
	//s16	stmp;

	sain_debug_c("sain_touch ioctl\r\n");

	if(touch_dev==NULL)
	{
		sain_debug("Null Device\n");			
		return 0;
	}

//	sain_debug_c("cmd = %d, argp = 0x%x\n", cmd, argp);	

	switch(cmd)
	{
		case TOUCH_IOCTL_CHARGER_MODE:
			{
				//sain_debug("charger mode\n");	
				//charger_mode = arg;
			}
			break;
		case TOUCH_IOCTL_READ_LASTKEY:
			{
			}
			break;
		case TOUCH_IOCTL_DO_KEY:
			{

				input_report_key(touch_dev->input_dev, BTN_TOUCH, 1);
			
				if ( (int)argp == 0x20a )
					input_report_key(touch_dev->input_dev, 0xe3, 1);
				else if ( (int)argp == 0x20b )
					input_report_key(touch_dev->input_dev, 0xe4, 1);
				else
					input_report_key(touch_dev->input_dev, (int)argp, 1);

				input_sync(touch_dev->input_dev);


				if((int)argp == KEY_4)
				{
					sain_debug("Enable IRQ!!\n");		
					enable_irq(touch_dev->client->irq);
				}
				else if((int)argp == KEY_5)
				{
					sain_debug("Disable IRQ!!\n");		
					disable_irq(touch_dev->client->irq);
				}
				else if((int)argp == KEY_6)
				{
					struct regulator *vreg_power_1_8;
					sain_debug("Enable Regulator!!\n");		
					vreg_power_1_8 = regulator_get(NULL, "8058_lvs0");
					regulator_enable(vreg_power_1_8);
				}
				else if((int)argp == KEY_7)
				{
					struct regulator *vreg_power_1_8;
					sain_debug("Disable Regulator!!\n");		
					vreg_power_1_8 = regulator_get(NULL, "8058_lvs0");
					regulator_disable(vreg_power_1_8);
				}
				else if((int)argp == KEY_8)
				{
					sain_debug("Disable Touch Debug!!\n");		
					TOUCH_DBG_ENABLE = false;
				}
				else if((int)argp == KEY_9)
				{
					sain_debug("Enable Touch Debug!!\n");
					TOUCH_DBG_ENABLE = true;
				}
			}
			break;
		case TOUCH_IOCTL_RELEASE_KEY:
			{
				input_report_key(touch_dev->input_dev, BTN_TOUCH, 0);
				
				if ( (int)argp == 0x20a )
					input_report_key(touch_dev->input_dev, 0xe3, 0);
				else if ( (int)argp == 0x20b )
					input_report_key(touch_dev->input_dev, 0xe4, 0);
				else
					input_report_key(touch_dev->input_dev, (int)argp, 0);
			}
			break;
		case TOUCH_IOCTL_PRESS_TOUCH:
			{
				int touchX = arg & 0x0000FFFF;
				int touchY = (arg >> 16) & 0x0000FFFF;	

				input_report_abs(touch_dev->input_dev, ABS_MT_TOUCH_MAJOR, 1);
				input_report_abs(touch_dev->input_dev, ABS_MT_WIDTH_MAJOR, 0);	
				input_report_abs(touch_dev->input_dev, ABS_MT_POSITION_X, touchX);
				input_report_abs(touch_dev->input_dev, ABS_MT_POSITION_Y, touchY);

				input_mt_sync(touch_dev->input_dev);	
#ifdef TOUCH_ICS_UPDATE
			input_report_key(touch_dev->input_dev, BTN_TOUCH, SAIN_TOUCH);						
#endif
				input_sync(touch_dev->input_dev);
			}
			break;
		case TOUCH_IOCTL_RELEASE_TOUCH:
			{
				int touchX = arg & 0x0000FFFF;
				int touchY = (arg >> 16) & 0x0000FFFF;	

				input_report_abs(touch_dev->input_dev, ABS_MT_TOUCH_MAJOR, 0);
				input_report_abs(touch_dev->input_dev, ABS_MT_WIDTH_MAJOR, 0);	
				input_report_abs(touch_dev->input_dev, ABS_MT_POSITION_X, touchX);
				input_report_abs(touch_dev->input_dev, ABS_MT_POSITION_Y, touchY);

				input_mt_sync(touch_dev->input_dev);	
#ifdef TOUCH_ICS_UPDATE			
			input_report_key(touch_dev->input_dev, BTN_TOUCH, SAIN_NOTOUCH);			
#endif				
				input_sync(touch_dev->input_dev);
			}
			break;
		case TOUCH_IOCTL_DISABLE_DEBUG:
			{
				sain_debug("Disable Touch Debug!!\n");		
				TOUCH_DBG_ENABLE = false;
			}
			break;
		case TOUCH_IOCTL_ENABLE_DEBUG:
			{
				sain_debug("Enable Touch Debug!!\n");
				TOUCH_DBG_ENABLE = true;
			}
			break;
		case TOUCH_IOCTL_READ_CHIP_REVISION:
			{
				int ret = 0;
				ret = touch_dev->cap_info.chip_revision;
				sain_debug("chip rev : %d \n", ret);

				if (copy_to_user(argp, &ret, sizeof(ret)))
					return false;
			}
			break;

		case TOUCH_IOCTL_READ_FW_VERSION:
			{
				int ret = 0;
				ret = touch_dev->cap_info.chip_firmware_version;
				sain_debug("firmware ver : %d \n", ret);

				if (copy_to_user(argp, &ret, sizeof(ret)))
					return false;
			}
			break;
		case TOUCH_IOCTL_READ_REG_DATA_VERSION:
			{
				int ret = 0;
				ret = touch_dev->cap_info.chip_reg_data_version;
				sain_debug("reg data ver : %d \n", ret);

				if (copy_to_user(argp, &ret, sizeof(ret)))
					return false;
			}
			break;
		case TOUCH_IOCTL_START_UPDATE:
			{
				int ret = 0;
				u16	chip_eeprom_info;

				sain_debug(KERN_INFO "start upgrade firmware\n");
				disable_irq(touch_dev->client->irq);
				flush_workqueue(sain_workqueue);
#if	SAIN_ESD_TIMER_INTERVAL	
				if(touch_dev->use_esd_timer)
				{
					ts_esd_timer_stop(touch_dev);
				}
#endif

				// h/w calibration
				if (ts_write_reg(touch_dev->client, SAIN_TOUCH_MODE, 0x07)!=I2C_SUCCESS) goto fail_init;
				if (ts_write_cmd(touch_dev->client, SAIN_CALIBRATE_CMD)!=I2C_SUCCESS)	goto fail_init;
				if (ts_write_cmd(touch_dev->client, SAIN_REST_CMD)!=I2C_SUCCESS)	goto fail_init;
				mdelay(1); 		
				ts_write_cmd(touch_dev->client, SAIN_CLEAR_INT_STATUS_CMD);
				// wait for h/w calibration
				do{
					msleep(3000);  //needs more than 3s 
					if (ts_read_data(touch_dev->client, SAIN_EEPROM_INFO_REG, (u8*)&chip_eeprom_info, 2)<0) goto fail_init;
					sain_debug("touch eeprom info = 0x%04X\r\n", chip_eeprom_info);
					if(!sain_bit_test(chip_eeprom_info, 0))	break;
				}while(1);
				//<----------------------------------------------------------
				if (ts_write_reg(touch_dev->client, SAIN_TOUCH_MODE, 0x00)!=I2C_SUCCESS) goto fail_init;
				if (ts_write_cmd(touch_dev->client, SAIN_REST_CMD)!=I2C_SUCCESS) goto fail_init;
				if (ts_write_cmd(touch_dev->client, SAIN_SAVE_STATUS_CMD)!=I2C_SUCCESS) goto fail_init;
				mdelay(1000);	// for fusing eeprom
				if (ts_write_cmd(touch_dev->client, SAIN_REST_CMD)!=I2C_SUCCESS) goto fail_init;			

				sain_debug("firmware upgrade complete.\n");	

				sain_debug("initialize touch chip, do not touch.\n");	
				
				ts_init_touch(touch_dev);

				enable_irq(touch_dev->client->irq);

				if (copy_to_user(argp, &ret, sizeof(ret)))
				{
					return false;
				}

				return true;

				fail_init:
				sain_debug("failed initiallize\r\n");
				if (ts_write_cmd(touch_dev->client, SAIN_REST_CMD)!=I2C_SUCCESS);
				
				ts_init_touch(touch_dev);
				enable_irq(touch_dev->client->irq);

				return false;

			}
			break;
		default:
			break;
	}
	return true;

	// Write Mode
	if (cmd >= 0xF000){ 
		//i = cmd - 0xF000;
		//regVal = arg;
		//if(i > MAX_REG_COUNT || m_reg_data[i].valid != 1){
		//	return -1;
		//}
		//sain_debug_msg("write new reg data( %d < %d)\r\n", chip_reg_data_version, m_reg_data[SAIN_DATA_VERSION_REG].reg_val);
		//if(ts_write_reg(_client, (u16)i, (u16)regVal)!=I2C_SUCCESS) goto fail_init;
		//if(ts_read_data(_client, (u16)i, (u8*)&stmp, 2)<0) goto fail_init;
		//if(memcmp((char*)&m_reg_data[i].reg_val, (char*)&stmp, 2)!=0)		//if(m_reg_data[i].reg_val != stmp)
		//sain_debug(KERN_WARNING "register data is different. (addr = 0x%02X , %d != %d)\r\n", i, m_reg_data[i].reg_val, stmp);
		//sain_debug_msg("done new reg data( %d < %d)\r\n", chip_reg_data_version, m_reg_data[SAIN_DATA_VERSION_REG].reg_val);		
		//if (ts_write_cmd(_client, SAIN_SAVE_STATUS_CMD)!=I2C_SUCCESS) goto fail_init;	
		//returnVal = 0;
		//return returnVal;
	}
	// Read Mode
	else {
		i = cmd;
		if(i > MAX_REG_COUNT){
			//returnVal = ts_read_reg(_client, (u16)i);

			//returnVal = ts_read_data(_client, (u16)i, (u8*)&stmp, 2);
		}
	}	
	return returnVal;
	//fail_init:
	//	sain_debug("failed ioctl\r\n");
	//if (ts_write_cmd(_client, SAIN_REST_CMD)!=I2C_SUCCESS);
}

#endif //FILE_CONFIG


static int sain_touch_remove(struct i2c_client *client)
{
	sain_touch_dev *touch_dev = i2c_get_clientdata(client);

	sain_debug_c("sain_touch_remove+ \r\n");	
#if	SAIN_ESD_TIMER_INTERVAL	
	if(touch_dev->use_esd_timer)
	{				
		down(&touch_dev->esd_lock);	
		ts_write_reg(touch_dev->client, SAIN_PERIODICAL_INTERRUPT_INTERVAL, 0);
		ts_esd_timer_stop(touch_dev);
		up(&touch_dev->esd_lock);		
		sain_debug(KERN_INFO " ts_esd_timer_stop\n");			
	}
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&touch_dev->early_suspend);
#else 	
#endif

	destroy_workqueue(sain_workqueue);
	destroy_workqueue(sain_tmr_workqueue);


	misc_deregister(&touch_event);
	if (touch_dev->client->irq) {
		free_irq(touch_dev->client->irq, touch_dev);
	}

	input_unregister_device(touch_dev->input_dev);
	input_free_device(touch_dev->input_dev);
	kfree(touch_dev);

	return 0;
}

static int __devinit sain_touch_init(void)
{
	sain_debug("Sain Touch Init \n");
	init_hw();
	return i2c_add_driver(&sain_touch_driver);    
}

static void __exit sain_touch_exit(void)
{
	i2c_del_driver(&sain_touch_driver);
}

module_init(sain_touch_init);
module_exit(sain_touch_exit);

MODULE_DESCRIPTION("touch-screen device driver using i2c interface");
MODULE_AUTHOR("sohnet <swjang@sain.co.kr>");
MODULE_LICENSE("GPL");

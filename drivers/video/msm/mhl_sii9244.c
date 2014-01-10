/* Copyright (c) 2002-2009, Silicon Image, Inc.  All rights reserved.    
 *
 * No part of this work may be reproduced, modified, distributed, transmitted,    
 * transcribed, or translated into any language or computer format, in any form   
 * or by any means without written permission of: Silicon Image, Inc.,            
 * 1060 East Arques Avenue, Sunnyvale, California 94085       
 *
 */
 
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/bitops.h>
#include <linux/time.h>
#include <linux/completion.h>

#include <linux/irq.h>
//#include <asm/irq.h>

#include <linux/i2c.h>
#include <asm/irq.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/syscalls.h> 
#include <linux/fcntl.h> 
#include <asm/uaccess.h> 
#include <mach/gpio.h>
#include <asm/system.h>
#include <mach/gpiomux.h>
#include "msm_fb.h"
#include "mhl_sii9244_driver.h"
#include "hdmi_msm.h"

#ifdef CONFIG_PANTECH_LCD_MHL_CABLE_DETECT
#include <linux/jiffies.h>
#include <linux/interrupt.h>
#include <mach/irqs.h>
extern void sii9244_cfg_power(bool on);
#define MHL_CABLE_CONNCET			1
#define MHL_CABLE_DISCONNCET	       0
extern int pantech_hdmi_cable_detect(int on);
void sii9244_cfg_power_init(void);
extern int is_mhl_mode(void);
extern void set_flag_mhl_mode(int val);
extern void change_hpd_state(boolean on);  
extern void change_mhl_state(boolean online);	
#ifdef PANTECH_FUSION2_MHL_DETECT_2ND
extern void SwitchToD3( void );
#endif
#endif

#define DEVICE_NAME "sii9244"
#ifdef MHL_DEBUG
#define MHL_DEV_INFO(format,arg...) printk(format, ##arg)
#else
#define MHL_DEV_INFO(format,arg...) (void)0
#endif
#define SII_DEV_DBG(format,...)\
    printk(KERN_ERR "[SKY_MHL] +%s, %d\n", __FUNCTION__,__LINE__);

#define GPIO_HIGH_VALUE 1
#define GPIO_LOW_VALUE  0

/*
  Description: Setting and define the GPIO pins related to MHL transmitter.

  The below function is example function in Dempsey platform.
  
  MHL_CSDA_MSM (GPIO_AP_SDA) : CSDA pin
  MHL_CSCL_MSM (GPIO_AP_SCL) : CSCL pin
  MHL_WAKE_UP (GPIO_MHL_WAKE_UP) : Wake up pin
  MHL_RST_N (GPIO_MHL_RST) : Rset pin
  MHL_EN (GPIO_MHL_SEL):  FSA3200 USB switch 
*/
extern int mhl_power_ctrl(int on);

#define MHL_INT_IRQ 	gpio_to_irq(MHL_INT)	

bool mhl_hdmi_ready=false;
 static atomic_t mhl_connect_status;

#ifdef PANTECH_FUSION2_MHL_DETECT_2ND
 static atomic_t mhl_intr_status;
#endif 

 static struct workqueue_struct *mhl_detect_work_queue;
 static struct workqueue_struct *mhl_ctrl_connect_work_queue;

 struct delayed_work mhl_detect_work;
 struct delayed_work mhl_ctrl_connect_work;
 struct delayed_work mhl_detect_again_work;
 struct delayed_work mhl_boot_work;
 static ssize_t hdmid_ready_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
 void handle_mhl_at_boot(struct work_struct *work);
 static DEVICE_ATTR(hdmid_ready, S_IWUSR | S_IWGRP , NULL, hdmid_ready_write);
 static DECLARE_WAIT_QUEUE_HEAD(mhl_disconnect_wait_queue);
 static DECLARE_WAIT_QUEUE_HEAD(mhl_connect_wait_queue);

   
 static void start_handle_mhl(unsigned long data);
 static DECLARE_TASKLET(mhl_handle_det, start_handle_mhl, (unsigned long)NULL);
 bool mhl_handle_tasklet_processed;
 extern void do_valid_irq_handler(void);
 extern void do_hdmi_hpd_feature_on(void);
bool get_uv_handler_occur(void);
void mhl_wake_disconnect_queue(void)
{
	wake_up(&mhl_disconnect_wait_queue);
}
void mhl_disconnect_queuework(int time)
{
	int rc = 0;
#ifdef PANTECH_FUSION2_MHL_DETECT_2ND
	if (atomic_read(&mhl_intr_status)){
		disable_irq(MHL_INT_IRQ);
		atomic_set(&mhl_intr_status,0);
	}
#endif
	atomic_set(&mhl_connect_status,1);
	do{
		rc = queue_delayed_work(mhl_ctrl_connect_work_queue, &mhl_ctrl_connect_work,msecs_to_jiffies(time));
		wait_event_timeout(mhl_connect_wait_queue, 0 ,msecs_to_jiffies(10));			
	}while(!rc);
}

 void set_hdmi_ready_start(bool mode)
 {
 	mhl_hdmi_ready=mode;
 }
bool get_hdmi_ready_value(void){
	return mhl_hdmi_ready;
}

//+ sleep current problem
	static struct gpiomux_setting mhl_suspend_pullup_cfg = {
		.func = GPIOMUX_FUNC_GPIO,
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_UP,
	};
//- sleep current problem


	static struct gpiomux_setting mhl_suspend_cfg = {
		.func = GPIOMUX_FUNC_GPIO,
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_DOWN,
	};
	
	static struct gpiomux_setting mhl_active_1_cfg = {
		.func = GPIOMUX_FUNC_GPIO,
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_NONE,
	};

	static struct gpiomux_setting mhl_active_2_cfg = {
		.func = GPIOMUX_FUNC_GPIO,
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_DOWN,
	};
		
	static struct gpiomux_setting mhl_active_3_cfg = {
		.func = GPIOMUX_FUNC_GPIO,
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_UP,
	};

		static struct gpiomux_setting mhl_active_4_cfg = {
		.func = GPIOMUX_FUNC_GPIO,
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_UP,
		.dir = GPIOMUX_IN,
	};


static struct msm_gpiomux_config msm8x60_mhl_configs[] = {
	{
		.gpio = MHL_RST_N,
		.settings = {
			[GPIOMUX_ACTIVE]    = &mhl_active_1_cfg,
			[GPIOMUX_SUSPENDED] = &mhl_suspend_cfg,
		},
	},
	{
		.gpio = MHL_EN,
		.settings = {
			[GPIOMUX_ACTIVE]    = &mhl_active_1_cfg,
			[GPIOMUX_SUSPENDED] = &mhl_suspend_cfg,
		},
	},
	{
		.gpio = MHL_SHDN,
		.settings = {
			[GPIOMUX_ACTIVE]    = &mhl_active_2_cfg,
			[GPIOMUX_SUSPENDED] = &mhl_suspend_cfg,
		},
	},
	{
		.gpio = MHL_CSDA_MSM,
		.settings = {
			[GPIOMUX_ACTIVE]    = &mhl_active_3_cfg,
			[GPIOMUX_SUSPENDED] = &mhl_suspend_pullup_cfg, //sleep current problem
		},
	},
	{
		.gpio = MHL_CSCL_MSM,
		.settings = {
			[GPIOMUX_ACTIVE]    = &mhl_active_3_cfg,
			[GPIOMUX_SUSPENDED] = &mhl_suspend_pullup_cfg, //sleep current problem
		},
	},
	{
		.gpio = MHL_WAKE_UP,
		.settings = {
			[GPIOMUX_ACTIVE]    = &mhl_active_1_cfg,
			[GPIOMUX_SUSPENDED] = &mhl_suspend_cfg,
		},
	},
	
	{
		.gpio = MHL_INT,
		.settings = {
			[GPIOMUX_ACTIVE]	= &mhl_active_4_cfg,
			[GPIOMUX_SUSPENDED] = &mhl_suspend_pullup_cfg, //sleep current problem
		},
	},
};


struct work_struct sii9244_int_work;
#ifdef CONFIG_PANTECH_LCD_MHL_CABLE_DETECT

struct sii9244_cable_detect{
	struct delayed_work work;
};

struct sii9244_cable_detect sii9244_cable_detect_work;
struct sii9244_cable_detect sii9244_cable_connect_work;



#endif
struct workqueue_struct *sii9244_wq = NULL;

struct i2c_driver sii9244_i2c_driver;
struct i2c_client *sii9244_i2c_client = NULL;

struct i2c_driver sii9244A_i2c_driver;
struct i2c_client *sii9244A_i2c_client = NULL;

struct i2c_driver sii9244B_i2c_driver;
struct i2c_client *sii9244B_i2c_client = NULL;

struct i2c_driver sii9244C_i2c_driver;
struct i2c_client *sii9244C_i2c_client = NULL;

static struct i2c_device_id sii9244_id[] = {
	{"sii9244", 0},
	{}
};

static struct i2c_device_id sii9244A_id[] = {
	{"sii9244A", 0},
	{}
};

static struct i2c_device_id sii9244B_id[] = {
	{"sii9244B", 0},
	{}
};

static struct i2c_device_id sii9244C_id[] = {
	{"sii9244C", 0},
	{}
};

int MHL_i2c_init = 0;

struct sii9244_state {
	struct i2c_client *client;
};

struct i2c_client* get_sii9244_client(u8 device_id)
{
	struct i2c_client* client_ptr;

	if(device_id == 0x72)
		client_ptr = sii9244_i2c_client;
	else if(device_id == 0x7A)
		client_ptr = sii9244A_i2c_client;
	else if(device_id == 0x92)
		client_ptr = sii9244B_i2c_client;
	else if(device_id == 0xC8)
		client_ptr = sii9244C_i2c_client;
	else
		client_ptr = NULL;

	return client_ptr;
}
EXPORT_SYMBOL(get_sii9244_client);

static ssize_t MHD_check_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	int count;
	int res = 0;
	count = sprintf(buf,"%d\n", res );
	return count;
}

static ssize_t MHD_check_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	MHL_DEV_INFO(KERN_ERR "input data --> %s\n", buf);
	return size;
}

static DEVICE_ATTR(MHD_file, S_IRUGO , MHD_check_read, MHD_check_write);

u8 sii9244_i2c_read(struct i2c_client *client, u8 reg)
{
	u8 ret;	
	
	if(!MHL_i2c_init)
	{
		SII_DEV_DBG("I2C not ready");
		return 0;
	}
	
	i2c_smbus_write_byte(client, reg);	

	ret = i2c_smbus_read_byte(client);

	if (ret < 0)
	{
		SII_DEV_DBG("i2c read fail");
		return -EIO;
	}
	return ret;

}
EXPORT_SYMBOL(sii9244_i2c_read);

int sii9244_i2c_write(struct i2c_client *client, u8 reg, u8 data)
{
	if(!MHL_i2c_init)
	{
		SII_DEV_DBG("I2C not ready");
		return 0;
	}

	return i2c_smbus_write_byte_data(client, reg, data);
}
EXPORT_SYMBOL(sii9244_i2c_write);

void sii9244_interrupt_event_work(struct work_struct *p)
{
	MHL_DEV_INFO("[SKY_MHL] sii9244_interrupt_event_work() is called\n");
	sii9244_interrupt_event();
}

void mhl_int_irq_handler_sched(void)
{
	MHL_DEV_INFO("mhl_int_irq_handler_sched() is called\n");
	queue_work(sii9244_wq,&sii9244_int_work);		
}

irqreturn_t mhl_int_irq_handler(int irq, void *dev_id)
{
	MHL_DEV_INFO("mhl_int_irq_handler() is called\n");

	mhl_int_irq_handler_sched();
	return IRQ_HANDLED;
}


#ifdef CONFIG_PANTECH_FB_MSM_MHL_SII9244
void start_mhl_handler(void)
{
	MHL_DEV_INFO("%s : is called\n",__func__);
	tasklet_schedule(&mhl_handle_det);

	return ;
}

void start_handle_mhl(unsigned long data)
{
	int rc = 0;
	MHL_DEV_INFO("%s : is called\n", __func__);

	if (!MHL_Get_Cable_State()){			
		MHL_DEV_INFO("%s : queue detect work..\n", __func__);	
		rc = queue_delayed_work(mhl_detect_work_queue, &mhl_detect_work, msecs_to_jiffies(600));
		if (!rc){
			MHL_DEV_INFO("%s: mhl_detect_work is already on!!! let's try again", __func__);
			rc =  schedule_delayed_work(&mhl_detect_again_work, msecs_to_jiffies(0));
			if (!rc) 
				pr_err("%s: mhl_detect_again_work is already on a queue", __func__);
		}
	}

	return;
}

void mhl_cable_detect_handler(void)
{
	MHL_DEV_INFO("mhl_cable_detect_handler() is called\n");
	if (get_hdmi_ready_value())
		start_mhl_handler();

}
void is_mhl_cable(struct work_struct *work)
{
	int rc=0;
	MHL_DEV_INFO("%s: MHL_Get_Cable_State  %d\n ", __func__ , MHL_Get_Cable_State());
	if (!MHL_Get_Cable_State()){	
		if (is_mhl_mode()){
			atomic_set(&mhl_connect_status,0);
			do{
				rc = queue_delayed_work(mhl_ctrl_connect_work_queue, &mhl_ctrl_connect_work,0);
				wait_event_timeout(mhl_connect_wait_queue, 0 ,msecs_to_jiffies(10));
				if (!rc)
					MHL_DEV_INFO("%s: mhl_ctrl_connect_work_queue is already on, queue again\n ", __func__);
			}while(!rc);
			set_flag_mhl_mode(0);
		}
	}
	else if (MHL_Get_Cable_State()){
		if (!is_mhl_mode()){
			atomic_set(&mhl_connect_status,1);
			MHL_DEV_INFO("%s: cable_mv is_mhl_mode\n ", __func__);
			do{
				rc = queue_delayed_work(mhl_ctrl_connect_work_queue, &mhl_ctrl_connect_work,0);
				wait_event_timeout(mhl_connect_wait_queue, 0 ,msecs_to_jiffies(10));
				if (!rc)
					MHL_DEV_INFO("%s: mhl_ctrl_connect_work_queue is already on, queue again\n ", __func__);
			}while(!rc);
			set_flag_mhl_mode(0);
		}
	}
	return ;
}

void mhl_cable_connect_ctrl(struct work_struct *work)
{
	bool connected;
	MHL_DEV_INFO("%s: in\n ", __func__);

	if (!atomic_read(&mhl_connect_status)){
		connected = mhl_cable_connect();
		if(connected) atomic_set(&mhl_connect_status,1);
		else atomic_set(&mhl_connect_status,0);
	}
	else if (atomic_read(&mhl_connect_status)){
		mhl_cable_disconnect();
		atomic_set(&mhl_connect_status,0);
	}
}

bool mhl_cable_connect(void)
{
	int rc;
	
	MHL_DEV_INFO("%s: in\n ", __func__);
//		pantech_hdmi_cable_detect(0);/*definitely off hdmi */
	MHL_Set_Cable_State(MHL_CABLE_CONNCET);
#ifdef PANTECH_FUSION2_MHL_DETECT_2ND
	if (!atomic_read(&mhl_intr_status)){
		enable_irq(MHL_INT_IRQ);
		atomic_set(&mhl_intr_status,1);
	}
#endif
	//change_mhl_state(false);  // kkcho_0909
	mhl_power_ctrl(1);	
	msleep(10);				

	//change_hpd_state(1);  // [repeat-cable onoff] Check-point , 20130703,  kkcho
	
	MHL_On(1);
//		MHL_Cable_On(1);

	MHL_En_Control(1) ;// switch-MHL		

	MHL_DEV_INFO( "!!!!!!!![SKY_MHL]%s MHL cable Connect\n",__func__);

	rc = wait_event_timeout(mhl_disconnect_wait_queue, get_mhl_status()==MHL_CABLE_CONNECT ,msecs_to_jiffies(200));
	MHL_DEV_INFO("get_mhl_status = %d", get_mhl_status());
	MHL_DEV_INFO("get_mhl_rgnd_status=%d\n", get_mhl_rgnd_status());
	MHL_DEV_INFO("get_mhl_power_mode=%d\n", get_mhl_power_mode());
	if (!rc && get_mhl_status() != MHL_CABLE_CONNECT){
		wait_event_timeout(mhl_disconnect_wait_queue, 0 ,msecs_to_jiffies(100));
		pr_err("get_mhl_status = %d", get_mhl_status());
		if (!get_mhl_power_mode()){
			pr_err("mhl is not ready to be operated\n");
			MHL_DEV_INFO(" get_mhl_rgnd_status=%d", get_mhl_rgnd_status());
			mhl_cable_disconnect();
			return false;
		}
		
	}
	
	rc= wait_event_timeout(mhl_disconnect_wait_queue, get_mhl_status()==MHL_DISCOVERY_SUCCESS ,msecs_to_jiffies(1500));

	if (!rc && get_mhl_status()!= MHL_DISCOVERY_SUCCESS ){
		pr_err("get_mhl_status = %d", get_mhl_status()); /* for debug */		
		queue_delayed_work(mhl_ctrl_connect_work_queue, &mhl_ctrl_connect_work,10);
		return false;
	}

	 return true;
}
#endif  /*CONFIG_PANTECH_LCD_MHL_CABLE_DETECT*/

void mhl_cable_disconnect(void)
{
	MHL_DEV_INFO("%s: in\n ", __func__);	

	if(MHL_Get_Cable_State()){		
		MHL_DEV_INFO("%s: really in!!!!!! in\n ", __func__);	
		MHL_Set_Cable_State(MHL_CABLE_DISCONNCET);	
		set_mhl_ctrled_hpd_state(false);
#ifdef PANTECH_FUSION2_MHL_DETECT_2ND
		if (atomic_read(&mhl_intr_status)){
			disable_irq(MHL_INT_IRQ);
			atomic_set(&mhl_intr_status,0);
		}
#endif
		MHL_Cable_On(0);
		MHL_On(0);
#ifdef PANTECH_FUSION2_MHL_DETECT
		change_mhl_state(false);
#endif
#ifndef PANTECH_FUSION2_MHL_DETECT_2ND
		mhl_power_ctrl(0);
#endif
		MHL_En_Control(0) ;// switch-MHL
		//change_hpd_state(0);  // [repeat-cable onoff] Check-point , 20130703 kkcho
		//			pantech_hdmi_cable_detect(0);
		//change_mhl_state(false);  // kkcho_0909
#ifdef PANTECH_FUSION2_MHL_DETECT_2ND
		SwitchToD3();
#endif
		MHL_DEV_INFO( "[SKY_MHL]%s MHL cable disConnect \n",__func__);
	}
}

void handle_pm_irq_again(struct work_struct *work)
{
	MHL_DEV_INFO("%s: in\n ", __func__);
	cancel_delayed_work(&mhl_detect_work);
	flush_workqueue(mhl_detect_work_queue);
	queue_delayed_work(mhl_detect_work_queue, &mhl_detect_work, msecs_to_jiffies(600));	
	return;
}

void handle_mhl_at_boot(struct work_struct *work)
{
	MHL_DEV_INFO("%s: in\n ", __func__);
	wait_event_interruptible(mhl_connect_wait_queue, mhl_hdmi_ready);
	MHL_DEV_INFO("%s: wait_event_interruptible out !!\n ", __func__);
	queue_delayed_work(mhl_detect_work_queue, &mhl_detect_work, msecs_to_jiffies(600));
	return;
}

//xsemiyas_debug
static ssize_t hdmid_ready_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int value;

	sscanf(buf, "%d", &value);
	MHL_DEV_INFO(KERN_ERR "hdmid ready value[%d]\n", value);
	if(value){
		set_hdmi_ready_start(true);
		start_mhl_handler();
	}
	return size;
}

#ifdef CONFIG_PANTECH_FB_MSM_MHL_SII9244
void hdmi_ready_func(int value)
{
	if(value){
		set_hdmi_ready_start(true);
		start_mhl_handler();
	}
}
EXPORT_SYMBOL(hdmi_ready_func);
#endif

irqreturn_t mhl_wake_up_irq_handler(int irq, void *dev_id)
{

	MHL_DEV_INFO(KERN_ERR "mhl_wake_up_irq_handler() is called\n");

	//if (gpio_get_value(GPIO_MHL_SEL))	
		mhl_int_irq_handler_sched();
	
	return IRQ_HANDLED;
}
 
static int sii9244_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct sii9244_state *state;

	struct class *mhl_class;
	struct device *mhl_dev;
	//int ret;
	
       SII_DEV_DBG("");
	   
	state = kzalloc(sizeof(struct sii9244_state), GFP_KERNEL);
	if (state == NULL) {	
		MHL_DEV_INFO("failed to allocate memory \n");	
		return -ENOMEM;
	}
	
	state->client = client;
	i2c_set_clientdata(client, state);
	
	/* rest of the initialisation goes here. */
	
	MHL_DEV_INFO("sii9244 attach success!!!\n");
	
	sii9244_i2c_client = client;

	MHL_i2c_init = 1;

	mhl_class = class_create(THIS_MODULE, "mhl");
	if (IS_ERR(mhl_class))
		pr_err("Failed to create class(mhl)!\n");

	mhl_dev = device_create(mhl_class, NULL, 0, NULL, "mhl_dev");
	
	if (IS_ERR(mhl_dev))
		pr_err("Failed to create device(mhl_dev)!\n");

	if (device_create_file(mhl_dev, &dev_attr_MHD_file) < 0)		
		MHL_DEV_INFO("Failed to create device file(%s)!\n", dev_attr_MHD_file.attr.name);
	
	mhl_detect_work_queue = create_singlethread_workqueue("mhl_detect_work_queue");
	if( mhl_detect_work_queue == NULL)  																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																	  
		pr_err(KERN_ERR "[SKY_MHL]+%s mhl_detect_work_queue is NULL \n", __FUNCTION__);

	mhl_ctrl_connect_work_queue = create_singlethread_workqueue("mhl_ctrl_connect_work_queue");
	if( mhl_ctrl_connect_work_queue == NULL)																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																		   
		pr_err(KERN_ERR "[SKY_MHL]+%s mhl_ctrl_connect_work_queue is NULL \n", __FUNCTION__);
	
	INIT_DELAYED_WORK_DEFERRABLE(&mhl_ctrl_connect_work, mhl_cable_connect_ctrl);
	INIT_DELAYED_WORK_DEFERRABLE(&mhl_detect_work, is_mhl_cable);	
	INIT_DELAYED_WORK_DEFERRABLE(&mhl_detect_again_work, handle_pm_irq_again);	
	INIT_DELAYED_WORK_DEFERRABLE(&mhl_boot_work, handle_mhl_at_boot);
		//xsemiyas_debug
	if (device_create_file(mhl_dev, &dev_attr_hdmid_ready) < 0)
		MHL_DEV_INFO("Failed to create device file(%s)!\n", dev_attr_hdmid_ready.attr.name);

	return 0;
}

static int __devexit sii9244_remove(struct i2c_client *client)
{
	struct sii9244_state *state = i2c_get_clientdata(client);
	sii9244_remote_control_remove();
	kfree(state);
	destroy_workqueue(mhl_ctrl_connect_work_queue); 
	destroy_workqueue(mhl_detect_work_queue); 
	return 0;
}

static int sii9244A_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct sii9244_state *state;
	SII_DEV_DBG("");
	state = kzalloc(sizeof(struct sii9244_state), GFP_KERNEL);
	if (state == NULL) {			
		MHL_DEV_INFO("failed to allocate memory \n");
		return -ENOMEM;
	}
	
	state->client = client;
	i2c_set_clientdata(client, state);
	
	/* rest of the initialisation goes here. */
	
	MHL_DEV_INFO("sii9244A attach success!!!\n");

	sii9244A_i2c_client = client;

	return 0;

}

static int __devexit sii9244A_remove(struct i2c_client *client)
{
	struct sii9244_state *state = i2c_get_clientdata(client);
	kfree(state);
	return 0;
}

static int sii9244B_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct sii9244_state *state;
	SII_DEV_DBG("");
	state = kzalloc(sizeof(struct sii9244_state), GFP_KERNEL);
	if (state == NULL) {			
		MHL_DEV_INFO("failed to allocate memory \n");
		return -ENOMEM;
	}
	
	state->client = client;
	i2c_set_clientdata(client, state);
	
	/* rest of the initialisation goes here. */

	MHL_DEV_INFO("sii9244B attach success!!!\n");

	sii9244B_i2c_client = client;
	
	return 0;

}

static int __devexit sii9244B_remove(struct i2c_client *client)
{
	struct sii9244_state *state = i2c_get_clientdata(client);
	kfree(state);
	return 0;
}

static int sii9244C_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{ 
	int ret;    
	struct sii9244_state *state;

	SII_DEV_DBG("");

	state = kzalloc(sizeof(struct sii9244_state), GFP_KERNEL);
	if (state == NULL) {   
		MHL_DEV_INFO("failed to allocate memory \n");
		return -ENOMEM;
	}

	state->client = client;
	i2c_set_clientdata(client, state);

	/* rest of the initialisation goes here. */

	MHL_DEV_INFO("sii9244C attach success!!!\n");


	sii9244C_i2c_client = client;      

	msleep(100);    

	sii9244_wq = create_singlethread_workqueue("sii9244_wq");
	INIT_WORK(&sii9244_int_work, sii9244_interrupt_event_work);
	
/*
	ret = request_threaded_irq(MHL_INT_IRQ, NULL, mhl_int_irq_handler,
				IRQF_SHARED , "mhl_int", (void *) state); 
		*/		
   	ret = request_irq(MHL_INT_IRQ,mhl_int_irq_handler,IRQF_TRIGGER_FALLING,
				 "mhl_int", (void *) state); 

        if (ret){ 
            MHL_DEV_INFO("[SKY_MHL] unable to request irq mhl_int err:: %d\n", ret);		
            return ret;
        }    
	
        MHL_DEV_INFO("[SKY_MHL] MHL int reques successful %d\n", ret);

#ifdef PANTECH_FUSION2_MHL_DETECT_2ND
	atomic_set(&mhl_intr_status,1);
#endif
		
        return 0;    
    }

static int __devexit sii9244C_remove(struct i2c_client *client)
{
	struct sii9244_state *state = i2c_get_clientdata(client);
	kfree(state);
	return 0;
}

struct i2c_driver sii9244_i2c_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "sii9244",
	},
	.id_table	= sii9244_id,
	.probe	= sii9244_i2c_probe,
	.remove	= __devexit_p(sii9244_remove),
	.command = NULL,
};

struct i2c_driver sii9244A_i2c_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "sii9244A",
	},
	.id_table	= sii9244A_id,
	.probe	= sii9244A_i2c_probe,
	.remove	= __devexit_p(sii9244A_remove),
	.command = NULL,
};

struct i2c_driver sii9244B_i2c_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "sii9244B",
	},
	.id_table	= sii9244B_id,
	.probe	= sii9244B_i2c_probe,
	.remove	= __devexit_p(sii9244B_remove),
	.command = NULL,
};

struct i2c_driver sii9244C_i2c_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "sii9244C",
	},
	.id_table	= sii9244C_id,
	.probe	= sii9244C_i2c_probe,
	.remove	= __devexit_p(sii9244C_remove),
	.command = NULL,
};

void sii9244_cfg_power_init(void)
{
	gpio_direction_output(MHL_RST_N, GPIO_LOW_VALUE);
	msleep(10);	
	gpio_direction_output(MHL_RST_N, GPIO_HIGH_VALUE);    


	sii9244_driver_init();
	gpio_direction_output(MHL_RST_N, GPIO_LOW_VALUE);
  
  	MHL_DEV_INFO(KERN_ERR "[SKY_MHL]%s \n",__func__);
}

void sii9244_cfg_power(bool on)
{
	if(on){
		gpio_direction_output(MHL_RST_N, GPIO_LOW_VALUE);
		msleep(10);	
		gpio_direction_output(MHL_RST_N, GPIO_HIGH_VALUE);    

		sii9244_driver_init();
	}
	else{
		gpio_direction_output(MHL_RST_N, GPIO_LOW_VALUE);
		msleep(10);
		gpio_direction_output(MHL_RST_N, GPIO_HIGH_VALUE);
		gpio_direction_output(MHL_RST_N, GPIO_LOW_VALUE);
	}

	MHL_DEV_INFO(KERN_ERR "[SKY_MHL]%s : %d \n",__func__,on);
}
EXPORT_SYMBOL(sii9244_cfg_power);

void MHL_On(bool on)
{
	MHL_DEV_INFO("[SKY_MHL] USB path change : %d\n", on);

	if (on == 1) {
		//if(gpio_get_value(MHL_EN))  // USB_SWITCH Check
		//	MHL_DEV_INFO("[MHL] MHL_EN : already 1\n");
		//else {
		//	gpio_direction_output(MHL_EN, GPIO_HIGH_VALUE); // switch-MHL			
			sii9244_cfg_power(1);
		//}	
	} else {
		//if(!gpio_get_value(MHL_EN))
		//	MHL_DEV_INFO("[MHL] MHL_EN : already 0\n");
		//else {	
			sii9244_cfg_power(0);
		//	gpio_direction_output(MHL_EN, GPIO_LOW_VALUE); // switch-USB	
		//}
	}
}
EXPORT_SYMBOL(MHL_On);
/*
static void mhl_gpio_init(uint32_t *table, int len, unsigned enable)
{
	int n, rc;
	for (n = 0; n < len; n++) {
		rc = gpio_tlmm_config(table[n],
				enable ? GPIO_CFG_ENABLE : GPIO_CFG_DISABLE);
		if (rc) {
			MHL_DEV_INFO(KERN_ERR "%s: gpio_tlmm_config(%#x)=%d\n",
					__func__, table[n], rc);
			break;
		}
	}
}
*/

/*
  Description: Setting and define the GPIO pins related to MHL transmitter.

  The below function is example function in Dempsey platform.
  
  MHL_CSDA_MSM (GPIO_AP_SDA) : CSDA pin
  MHL_CSCL_MSM (GPIO_AP_SCL) : CSCL pin
  MHL_WAKE_UP (GPIO_MHL_WAKE_UP) : Wake up pin
  MHL_RST_N (GPIO_MHL_RST) : Rset pin
  MHL_EN (GPIO_MHL_SEL):  FSA3200 USB switch   
*/

int mhl_gpio_request(void)
{
	int rc = 0;
	gpio_request(MHL_WAKE_UP, "mhl_wake_up");
	if (rc) {
		MHL_DEV_INFO("request gpio MHL_WAKE_UP failed, rc=%d\n", rc);
		return -EINVAL;
	}

	gpio_request(MHL_RST_N, "mhl_rst_n"); 
	if (rc) {
		MHL_DEV_INFO("request gpio MHL_RST_N failed, rc=%d\n", rc);
		return -EINVAL;
	}

	gpio_request(MHL_EN,"mhl_en"); 
	if (rc) {
		MHL_DEV_INFO("request gpio MHL_EN failed, rc=%d\n", rc);
		return -EINVAL;
	}

	gpio_request(MHL_SHDN, "mhl_shdn"); 
	if (rc) {
		MHL_DEV_INFO("request gpio MHL_SHDN failed, rc=%d\n", rc);
		return -EINVAL;
	}
	
	return rc;
}

static void sii9244_cfg_gpio(void)
{
	int rc=0;

	MHL_DEV_INFO(KERN_ERR "[SKY_MHL]+%s 2nd needed gpio_init\n", __FUNCTION__);
	// need to define the GPIO configuration
	//  mhl_gpio_init(mhl_sii9244_gpio_init_table, ARRAY_SIZE(mhl_sii9244_gpio_init_table), 1);
	msm_gpiomux_install(msm8x60_mhl_configs,
	ARRAY_SIZE(msm8x60_mhl_configs));

	rc = mhl_gpio_request();
	if (rc) 
		MHL_DEV_INFO("gpio request error rc=%d\n", rc);

	//set_irq_type(MHL_WAKEUP_IRQ, IRQ_TYPE_EDGE_RISING);
	//	 set_irq_type(MHL_INT_IRQ, IRQ_TYPE_EDGE_FALLING);

	/* USB switch*/
	/* LOW : USB, HIGH : MHL */	
	//gpio_direction_output(MHL_EN, GPIO_HIGH_VALUE);
	//msleep(5);
	gpio_direction_output(MHL_EN, GPIO_LOW_VALUE); // USB to MSM
	//msleep(5);
	gpio_direction_output(MHL_SHDN, GPIO_LOW_VALUE);  		

	gpio_direction_output(MHL_WAKE_UP, GPIO_LOW_VALUE);  // HPD pin이 제대로 동작 안되는 상황이므로.. 보완을 위해...		
}

static int __init sii9244_init(void)
{
	int ret;

	sii9244_cfg_gpio();	

	MHL_DEV_INFO(KERN_ERR "[SKY_MHL]+%s 3rd i2c_add_driver\n", __FUNCTION__);

	ret = i2c_add_driver(&sii9244_i2c_driver);
	if (ret != 0)
		pr_err("[MHL sii9244] can't add i2c driver\n");
	else
		MHL_DEV_INFO("[MHL sii9244] add i2c driver\n");
	
	ret = i2c_add_driver(&sii9244A_i2c_driver);
	if (ret != 0)
		pr_err("[MHL sii9244A] can't add i2c driver\n");
	else
		MHL_DEV_INFO("[MHL sii9244A] add i2c driver\n");
	
	ret = i2c_add_driver(&sii9244B_i2c_driver);
	if (ret != 0)
		pr_err("[MHL sii9244B] can't add i2c driver\n");
	else
		MHL_DEV_INFO("[MHL sii9244B] add i2c driver\n");
	
	ret = i2c_add_driver(&sii9244C_i2c_driver);
	if (ret != 0)
		pr_err("[MHL sii9244C] can't add i2c driver\n");
	else
		MHL_DEV_INFO("[MHL sii9244C] add i2c driver\n");

	//mhl_power_ctrl(1); //sleep problem 
	sii9244_remote_control_init();
	sii9244_cfg_power_init();	//Turn On power to sii9244 

	return ret;	
}
module_init(sii9244_init);		

static void __exit sii9244_exit(void)
{
	i2c_del_driver(&sii9244_i2c_driver);
	i2c_del_driver(&sii9244A_i2c_driver);
	i2c_del_driver(&sii9244B_i2c_driver);	
	i2c_del_driver(&sii9244C_i2c_driver);
	
};
module_exit(sii9244_exit);

MODULE_DESCRIPTION("sii9244 MHL driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Qualcomm Innovation Center, Inc.");

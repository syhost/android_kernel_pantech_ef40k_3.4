/*
 *  max17040_battery.c
 *  fuel-gauge systems for lithium-ion (Li+) batteries
 *
 *  Copyright (C) 2009 Samsung Electronics
 *  Minkyu Kang <mk7.kang@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/max17040_battery.h>
#include <linux/slab.h>
#include <linux/i2c/smb137b.h>

//test code
#include <linux/acct.h>
#include <linux/msm_adc.h>
#include <mach/msm_xo.h>
#include <mach/msm_hsusb.h>
#include <linux/android_alarm.h>
#include <linux/earlysuspend.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/syscalls.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/fcntl.h>
#include <asm/uaccess.h>
#include <linux/rtc.h>
#include <linux/input.h>

#ifdef CONFIG_SKY_SMB136S_CHARGER
#include <linux/msm-charger.h>
#endif

#define MAX17040_VCELL_MSB	0x02
#define MAX17040_VCELL_LSB	0x03
#define MAX17040_SOC_MSB	0x04
#define MAX17040_SOC_LSB	0x05
#define MAX17040_MODE_MSB	0x06
#define MAX17040_MODE_LSB	0x07
#define MAX17040_VER_MSB	0x08
#define MAX17040_VER_LSB	0x09
#define MAX17040_RCOMP_MSB	0x0C
#define MAX17040_RCOMP_LSB	0x0D
#define MAX17040_CMD_MSB	0xFE
#define MAX17040_CMD_LSB	0xFF

#define MAX17040_DELAY		1000
#define MAX17040_BATTERY_FULL	95


#define MAX17040_BATT_ID_MAX_MV  800
#define MAX17040_BATT_ID_MIN_MV  600


#define FAST_POLL	(1 * 60) //50 seconds    //60
#define SLOW_POLL	(1 * 130) //120 seconds

#define MAX_READ	10

//#define MAX17040_ALARM_RTC_ENABLE //RTC ENABLE
//#define MAX17040_DEG_ENABLE	//normal debug
//#define MAX17040_SLEEP_DEBUG //sleep time debug
//#define MAX17040_DEBUG_QUICK
//#define MAX17040_TIME_DEBUG

//#define USE_MAX17040_QUICKSTART

#ifdef MAX17040_DEG_ENABLE
#define dbg(fmt, args...)   printk("##################[MAX17040] " fmt, ##args)
#else
#define dbg(fmt, args...)
#endif
#ifdef MAX17040_SLEEP_DEBUG
#define sleep_dbg(fmt, args...)   printk("[MAX17040 SLEEP] " fmt, ##args)
#else
#define sleep_dbg(fmt, args...) 
#endif
#define dbg_func_in()       dbg("[FUNC_IN] %s\n", __func__)
#define dbg_func_out()      dbg("[FUNC_OUT] %s\n", __func__)
#define dbg_line()          dbg("[LINE] %d(%s)\n", __LINE__, __func__)

#define ABS_WAKE                            (ABS_MISC)

//Compensation Table
#define COMPENSATION_MAX 8

//depend on quick_start
#define SKY_SOC_LSB	256
#define SKY_MULT_100(x)	(x*100)	
#define SKY_MULT_1000(x)	(x*1000)	
#define SKY_MULT_10000(x)	(x*10000)	

#define SKY_MULT_1000000(x)	(x*1000000)	
#define SKY_DIV_1000000(x)	(x/1000000)	

#define SKY_SOC_FULL	1159 // p11220 20130717 : 1074 -> 1159
#define SKY_SOC_EMPTY	33 // p11220 20130717 : 15 -> 33

#define CHARGING_DONE_THRESHOLD	1182 // 4.333V
#define RECHARGING_THRESHOLD 	1165 // p11220 20130717 : 1144 -> 1165  4.318V

#define FINISHED_CHARGING		5
//sleep config
#define SLEEP_ONE_MINUTE 60 // 1 minute
#define SLEEP_THREE_MINUTE 180 // 3 minute
#define SLEEP_FIVE_MINUTE 300 // 5 minute
#define	SLEEP_ONE_HOUR 3600 // 20 minute

#if defined(CONFIG_SKY_SMB136S_CHARGER)
extern int sky_get_plug_state(void);
extern int sky_is_batt_status(void);
extern void msm_pm_set_max_sleep_time(int64_t sleep_time_ns);
#ifdef MAX17040_DEBUG_QUICK	
extern int pm8058_chg_nobattery_factory_cable(void);
extern int pmic8058_tz_get_temp_charging(unsigned long *pm_temp);
#endif

extern atomic_t smb_charger_state;

//usb ac display
static int charge_state=0;

// rcomp change condition
int high_current_rcomp_mode = 0;

int max17040_raw_pre_soc=0;
int max17040_raw_soc=0;

/*depend on wake lock */
static int max17040_wake_state;
#endif

typedef enum {
	NoEvents= 0,
	Events,
}MAX17040_EVENT;


/*ps2 team shs 
 Early_resume : 40 seconds
 Early_suspend : 120 seconds
*/

typedef enum {
	Early_resume=0,  
	Early_suspend,

}MAX17040_STATE;
struct max17040_quick_data{
	unsigned int vcell_msb;
	unsigned int vcell_lsb;
	unsigned int soc_msb;
	unsigned int soc_lsb;

	unsigned long refrence_soc;
	/* battery quick voltage*/
	unsigned long quick_vcell;
	/* battery quick soc*/
	unsigned long quick_soc;
	/* battery quick start*/
	int quick_state;

};

struct max17040_chip {

	struct mutex	data_mutex;
	struct mutex	i2c_mutex;
	struct mutex 	quick_mutex;
	struct i2c_client		*client;
	struct delayed_work		work;
	struct power_supply		battery;
	struct max17040_platform_data	*pdata;

	//ps2 team shs : workqueue_struct
	struct workqueue_struct *monitor_wqueue;
	//ps2 team shs : work_struct
	struct work_struct monitor_work;
	//ps2 team shs : alarm
	struct alarm alarm;
	//ps2 team shs : wake up lock
	struct wake_lock work_wake_lock;
	//ps2 team shs : TEST MODE : wake up lock 
	
	unsigned long timestamp;		/* units of time */
	
	//PS2 team shs : 
	ktime_t last_poll;
	

	/* State Of Connect */
	int online;
	/* battery voltage */
	int vcell;
	/* battery capacity */
	int soc;

	/* battery brefore soc*/
	int prev_soc;

	/* i2c error filed*/

	int i2c_state;

	/* i2c voltage_error*/
	int i2c_state_vol;

	int prev_voltage;
	/*battery quick data
*/
	struct max17040_quick_data quick_data;

	/* test code states*/
	atomic_t set_test;

	/* State Of update*/
	MAX17040_EVENT event;

	/* State Of update*/
	MAX17040_EVENT suspend_event;

	
	MAX17040_STATE slow_poll;	
	/* test code : input device driver*/
	struct input_dev *max17040_input_data;

	/*ealry suspend */
	struct early_suspend early_suspend;

	/* rcomp_mode_set */
	int set_rcomp_mode;
};
 
typedef struct{
  unsigned long volt;   // read voltage
  unsigned long soc;	// read soc
  unsigned long slop;   // ?????????? ???? ???? ???? ?????? ???? x1000
  unsigned long offset; // mV
}fuelgauge_linearlize_type;

struct max17040_chip max17040_data;

/* check state
struct max17040_platform_data {
	int (*battery_online)(void);
	int (*charger_online)(void);
	int (*charger_enable)(void);

	1. battery_online : check battery connection
	2. charger_online : check charger connection
	3. charger_enable : check charger state 
};
*/

#ifdef USE_MAX17040_QUICKSTART
static fuelgauge_linearlize_type sky_fuelgauge_linearlize_table[1][COMPENSATION_MAX]={
	// volt , soc,  slop, offset
  { // Discharging table ( 250mA 4.2V )
    { 42440,  981,  228, 19980},
    { 39555,  725,  112,  31398},
    { 38009,  541,  84,  33449},
    { 37087,  340,  45,  35527},
    { 36589,  177,  31,  36046},
    { 35948,  84,   69,  35368},
    { 35389,  25,   96, 35140},
    { 33994,  0,    541, 33994},
  }
};

static int max17040_check_restart( unsigned long avoltage, int soc )
{
	int i=0;
	unsigned long sky_low_soc=0;
	unsigned long sky_fuelgauge_ref_soc=0;
	unsigned long sky_high_soc=0;
	int high_soc=0;
	int low_soc=0;
	  for( i = 0; i < COMPENSATION_MAX; i++ )
	  {
	    if( avoltage >= sky_fuelgauge_linearlize_table[0][i].volt )
	    {
		  mutex_lock(&max17040_data.quick_mutex); 			
	      sky_high_soc = (avoltage - sky_fuelgauge_linearlize_table[0][i].offset)/sky_fuelgauge_linearlize_table[0][i].slop;
		  sky_low_soc=(avoltage - sky_fuelgauge_linearlize_table[0][i].offset)%sky_fuelgauge_linearlize_table[0][i].slop;	  
		  sky_fuelgauge_ref_soc = SKY_MULT_1000(sky_high_soc)+(SKY_MULT_1000(sky_low_soc)/sky_fuelgauge_linearlize_table[0][i].slop);	  
		  high_soc=sky_fuelgauge_ref_soc + 30000;	// tolerance +30%
		  low_soc=sky_fuelgauge_ref_soc  - 30000;	// tolerance -30%
		  mutex_unlock(&max17040_data.quick_mutex);		  
		  
	      if( (soc > high_soc) || (soc < low_soc)|| ( (soc <= 1000) && (sky_fuelgauge_ref_soc > 4000) ) )
	      {
			max17040_data.quick_data.quick_state=i;	      
//			sleep_dbg("[QUICK START] voltage : [%u]mv, soc : [%d], index [%d], sky_high_soc [%u] ,sky_low_soc [%u]\n",avoltage,soc,i,sky_high_soc,sky_low_soc);	      
//			sleep_dbg("[QUICK START] ref_soc : [%u], high_soc : [%u], low_soc : [%d]\n",sky_fuelgauge_ref_soc,high_soc,low_soc);	      			
	        return 1;
	      }
	      break;
	    }
	  }
	max17040_data.quick_data.quick_state=-1;	      	  
//	sleep_dbg("[QUICK DISABLE] voltage : [%u]mv, soc : [%d], index [%d], sky_high_soc [%u], sky_low_soc [%u] \n",avoltage,soc,i,sky_high_soc,sky_low_soc);	      
//	sleep_dbg("[QUICK DISABLE] ref_soc : [%u], high_soc : [%u], low_soc : [%d]\n",sky_fuelgauge_ref_soc,high_soc,low_soc);	      			
return 0;
}
#endif

void max17040_prevent_suspend(void)
{
	dbg_func_in();
	if(!max17040_wake_state)
		{
		wake_lock(&max17040_data.work_wake_lock);
		max17040_wake_state=1;
		}
	dbg_func_out();
}
void max17040_allow_suspend(void)
{

	dbg_func_in();
	if(max17040_wake_state)
		{
		wake_unlock(&max17040_data.work_wake_lock);
		max17040_wake_state=0;		
		}
	dbg_func_out();
}

int max17040_get_voltage(void)
{
	dbg_func_in();
	if(max17040_data.i2c_state_vol)	
	return 	max17040_data.prev_voltage;
	else
	return max17040_data.vcell;
	dbg_func_out();
}

int max17040_get_charge_state(void)
{
	dbg_func_in();
	return charge_state;
	dbg_func_out();	
}

int is_max17040_capable_of_charging(void)
{
	if(max17040_raw_soc >= (CHARGING_DONE_THRESHOLD-5))
		return 0;
	return 1;	
}

static enum power_supply_property max17040_battery_props[] = {
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
};

static int max17040_get_property(struct power_supply *psy,
			    enum power_supply_property psp,
			    union power_supply_propval *val)
{
	dbg_func_in();
	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		if(max17040_data.i2c_state_vol)
		val->intval = max17040_data.prev_voltage*1000; //2011.05.16 leecy add for battery Info
		else
		val->intval = max17040_data.vcell*1000; //2011.05.16 leecy add for battery Info

		//pr_info("[SKY CHG] %s max17040_get_property= %d V\n",
		//			__func__,val->intval);		
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		if(max17040_data.i2c_state)
		val->intval = max17040_data.prev_soc;		
		else
		val->intval = max17040_data.soc;

		//pr_info("[SKY CHG] %s max17040_get_property= %d Pro\n",
		//			__func__,val->intval);
		break;
	default:
		return -EINVAL;
	}
	dbg_func_out();		
	return 0;
}
static void max17040_bat_external_power_changed(struct power_supply *psy)
{
}

static int max17040_write_reg(u8 reg, u8 ahigh, u8 alow)
{
	int ret;
	u8 buf[20];
	buf[0]=reg;
	buf[1]=ahigh;
	buf[2]=alow;
	if ( max17040_data.client == NULL ) {
		sleep_dbg("%s : max17040.client is NULL\n", __func__);
		return -ENODEV;
	}
	mutex_lock(&max17040_data.i2c_mutex); 
	ret = i2c_master_send(max17040_data.client, buf, 3);		
	mutex_unlock(&max17040_data.i2c_mutex);
	if (ret<0)
        {   
                dev_err(&max17040_data.client->dev,
                "%s couldn't write reg %d\n", __func__, ret);
	return -1;	
        }
	else if (ret!=3)
        {   
                dev_err(&max17040_data.client->dev,
                "%s couldn't write reg %d\n", __func__, ret);
	return -1;
        }
	else
	return ret;
}

static u8 max17040_read_reg(u8 reg)
{
	u8 ret_s,ret_r ,ret;
	u8 buf[20];

	buf[0]=reg;
	mutex_lock(&max17040_data.i2c_mutex); 
	ret_s = i2c_master_send(max17040_data.client,  buf, 1);
	ret_r = i2c_master_recv(max17040_data.client, buf, 1);
	mutex_unlock(&max17040_data.i2c_mutex);	
	if(ret_s<0 || ret_r<0)
	{
//	sleep_dbg("max17040 I2C FILED ret [%d] [%d]\n",ret_s,ret_r);		
                dev_err(&max17040_data.client->dev,
                "%s couldn't read reg ret_s:%d, ret_r:%d\n", __func__, ret_s,ret_r);

	return -1;
	}
	else if(ret_s !=1 || ret_r !=1)
	{
//	sleep_dbg("max17040 I2C FILED [%d] [%d] bytes transferred (expected 1)\n",ret_s,ret_r);				
                dev_err(&max17040_data.client->dev,
                "%s couldn't read reg ret_s:%d, ret_r:%d\n", __func__, ret_s,ret_r);

	return -1;
	}
	else
	ret=buf[0];
	return ret;
}

static void max17040_restart(void)
{
	int ret=0,i=0;
	ret=max17040_write_reg(MAX17040_MODE_MSB, 0x40,0x00);
	if(ret<0)
	{
		for(i=0;i<MAX_READ;i++)
		{
			ret=max17040_write_reg(MAX17040_MODE_MSB, 0x40,0x00);	
			if(ret<0)
			continue;
			else
			break;
		}
	}
	sleep_dbg("MAX17040 QUICK START TRY : [%d]\n",i);
}

static void max17040_set_rcomp(void)
{
	int ret=0,i=0;	
	int chg_type;

#ifdef MAX17040_DEBUG_QUICK	
	chg_type = atomic_read(&smb_charger_state);
#else
	chg_type = 0;
#endif
	
	if((high_current_rcomp_mode == 1) && (chg_type == 0)){
		ret = max17040_write_reg(MAX17040_RCOMP_MSB, 0x25,0x00);
		ret=max17040_write_reg(MAX17040_RCOMP_LSB, 0x1F,0x00);
		max17040_data.set_rcomp_mode = 1;
	}
	else {
		ret=max17040_write_reg(MAX17040_RCOMP_MSB, 0xC0,0x00);
		ret=max17040_write_reg(MAX17040_RCOMP_LSB, 0x1F,0x00);
		max17040_data.set_rcomp_mode = 0;
	}
	
	if(ret<0)
	{
		for(i=0;i<MAX_READ;i++)
		{
			if((high_current_rcomp_mode == 1) && (chg_type == 0)) {
                        	ret=max17040_write_reg(MAX17040_RCOMP_MSB, 0x65,0x00);
				ret=max17040_write_reg(MAX17040_RCOMP_LSB, 0x1F,0x00);
			}
			else {
				ret=max17040_write_reg(MAX17040_RCOMP_MSB, 0xC0,0x00);	
				ret=max17040_write_reg(MAX17040_RCOMP_LSB, 0x1F,0x00);
			}
			if(ret<0)
			continue;
			else
			break;
		}
	}

	sleep_dbg("MAX17040 SET_RCOMP TRY : [%d]\n",i);			
}

static void max17040_quick_get_vcell(void)
{
	u8 msb=0;
	u8 lsb=0;
	unsigned long quick_avalue;
	unsigned long voltage=0;
	int i=0;	
	msb = max17040_read_reg(MAX17040_VCELL_MSB);
	lsb = max17040_read_reg(MAX17040_VCELL_LSB);
	if(msb < 0 || lsb < 0)
	{
		for(i=0;i<MAX_READ;i++)
		{
		msb = max17040_read_reg(MAX17040_VCELL_MSB);
		lsb = max17040_read_reg(MAX17040_VCELL_LSB);
		if(msb < 0 || lsb <0)
		{
		continue;
		}
		else
		break;
		}
	}
	voltage=(msb<<4)|((lsb&0xf0)>>4);
	quick_avalue=(voltage*1250)/100;
	//sleep_dbg("MAX17040_QUICK  LOW MSB [%d] : LSB [%d] : LOW VOLTAGE [%d]\n",(int)msb,(int)lsb,voltage);
	//sleep_dbg("MAX17040_QUICK  Adjusted [%d] : I2C Error Count [%d]\n",(int)quick_avalue,(int)i);
	mutex_lock(&max17040_data.data_mutex); 	
	max17040_data.quick_data.vcell_msb = msb;	
	max17040_data.quick_data.vcell_lsb = lsb;		
	if(i==MAX_READ)
	max17040_data.quick_data.quick_vcell = 33975;	
	else
	max17040_data.quick_data.quick_vcell = quick_avalue;	
	mutex_unlock(&max17040_data.data_mutex);	

}

#ifdef MAX17040_DEBUG_QUICK	
static void max17040_quick_get_value(void)
{
	u8 rcomp=0, rcomp_low;
	sleep_dbg("\n=======================================================================\n");	
	sleep_dbg("[INFORMATION] QUICK START STATE [%d]\n",max17040_data.quick_data.quick_state);	      
	sleep_dbg("[INFORMATION] QUICK START SOC_MSB [%d]\n",max17040_data.quick_data.soc_msb);	
	sleep_dbg("[INFORMATION] QUICK START SOC_LSB [%d]\n",max17040_data.quick_data.soc_lsb);		
	sleep_dbg("[INFORMATION] QUICK START SOC [%d]\n",max17040_data.quick_data.quick_soc);	
	sleep_dbg("[INFORMATION] QUICK START REFERENSE SOC [%d]\n",sky_fuelgauge_ref_soc);				
	sleep_dbg("[INFORMATION] QUICK START VCELL_MSB [%d]\n",max17040_data.quick_data.vcell_msb);		
	sleep_dbg("[INFORMATION] QUICK START VCELL_LSB [%d]\n",max17040_data.quick_data.vcell_lsb);	
	sleep_dbg("[INFORMATION] QUICK START VOLTAGE [%d]\n",max17040_data.quick_data.quick_vcell);	
	rcomp=max17040_read_reg(MAX17040_RCOMP_MSB);
	rcomp_low=max17040_read_reg(MAX17040_RCOMP_LSB);	
	sleep_dbg("[INFORMATION] RCOMP 0x[%x][%x]\n",rcomp,rcomp_low);			
	sleep_dbg("\n=======================================================================\n");		
}
#endif

static void max17040_get_vcell(void)
{
	u8 msb;
	u8 lsb;
	int avalue=0;
	int voltage=0;
	dbg_func_in();
	msb = max17040_read_reg(MAX17040_VCELL_MSB);
	lsb = max17040_read_reg(MAX17040_VCELL_LSB);

	//check i2c error
	if(msb<0 ||lsb <0)
	{
	max17040_data.i2c_state_vol =1;
	}
	else
	{
	max17040_data.i2c_state_vol =0;
	max17040_data.prev_voltage=max17040_data.vcell;
	}
	
	voltage=(msb<<4)|((lsb&0xf0)>>4);
	avalue=(voltage*125)/100;
	
//	sleep_dbg(" MSB [%d] : LSB [%d] : LOW VOLTAGE [%d] : VOLTAGE_NOW [%d]\n",msb,lsb,voltage,avalue);

	/* voltage changes but the event is not sent.
	//temp code
	if(avalue!=max17040_data.vcell)
	max17040_data.event=Events;
	*/
	
	mutex_lock(&max17040_data.data_mutex); 	
	max17040_data.vcell = avalue;
	mutex_unlock(&max17040_data.data_mutex);	
	
#ifdef MAX17040_DEBUG_QUICK	
//Factory cable debug
#ifdef CONFIG_SKY_SMB136S_CHARGER
//printk("Max17040_data.vcell = %d\n", max17040_data.vcell);
	if(pm8058_chg_nobattery_factory_cable())
		max17040_data.vcell = 4100;
#endif
#endif

	dbg_func_out();	
}

static void max17040_check_power(int msb)
{
	if(max17040_data.slow_poll) //do not call early resume state.
		{
			if(msb<=15) //Battery level is 15% less
			max17040_data.suspend_event=Events;
		}
}

static void max17040_get_soc(void)
{
	u8 msb;
	u8 lsb;
	int avalue=0;
	int soc=0;
//	int sky_state=0;

	dbg_func_in();
	msb = max17040_read_reg(MAX17040_SOC_MSB);
	lsb = max17040_read_reg(MAX17040_SOC_LSB);

	//check i2c error
	if(msb<0 ||lsb <0)
	{
	max17040_data.i2c_state =1;
	}
	else
	{
	max17040_data.i2c_state =0;
	max17040_data.prev_soc=max17040_data.soc;
	}

#ifdef MAX17040_DEBUG_QUICK		
	//quick start code
	soc=SKY_MULT_1000(msb)+(SKY_MULT_1000(lsb)/SKY_SOC_LSB);	
	soc=soc/1000;
#else

	/*//description
	read i2c data [msb=20,lsb=10]
	avalue=20*1000+(10*1000)/256
	*/

	// Empty level change
	avalue=SKY_MULT_1000(msb)+(SKY_MULT_1000(lsb)/SKY_SOC_LSB);	

	max17040_raw_pre_soc = max17040_raw_soc;
	max17040_raw_soc = (avalue/100);
	
	//Ajdusted soc%=(SOC%-EMPTY)/(FULL-EMPTY)*100
	//if(avalue>1200) 
	if(avalue>1500)

		soc=(((avalue-SKY_MULT_100(SKY_SOC_EMPTY))*100)/(SKY_MULT_100(SKY_SOC_FULL)-SKY_MULT_100(SKY_SOC_EMPTY)));
	else 
		soc=0;
       // if(avalue >1000 && avalue <1200)
       //         soc=1;

#endif	

	//logic code
	if(soc>100) //soc>100
		soc=100;
	
	if(soc==100)
		charge_state=1;
	else
		charge_state=0;	

#ifdef MAX17040_DEBUG_QUICK	
	// if present factory cable, soc is 50%
	//pr_err("%s : soc = %d\n", __func__, soc); 
	if(pm8058_chg_nobattery_factory_cable())
		soc = 50;
#endif

	if(max17040_data.event) // soc is changed 
	{
//		sleep_dbg("CONFIG CAPACITY [%d] : BATTERY STATS  : [%d]\n",soc,sky_state);
		sleep_dbg("SOC MSB [%d] : LSB [%d] : Lower SOC [%d] : Adjusted SOC [%d] : charge_state [%d] \n",msb,lsb,avalue,soc,charge_state);
	}
	
	if(soc!=max17040_data.soc)
		max17040_data.event=Events;
	if(soc==0)	//0% is occured events
		max17040_data.event=Events;
	
	max17040_check_power(soc);
	mutex_lock(&max17040_data.data_mutex); 	
	max17040_data.soc = soc;	
	mutex_unlock(&max17040_data.data_mutex);
	dbg_func_out();		
}

static void max17040_get_version(void)
{
	u8 msb;
	u8 lsb;

	msb = max17040_read_reg(MAX17040_VER_MSB);
	lsb = max17040_read_reg(MAX17040_VER_LSB);
	pr_info("[MAX17040] Fuel-Gauge Ver %d%d\n", msb, lsb);
}

static ssize_t max17040_show_flag(struct device *dev, struct device_attribute *attr, char *buf)
{
	int enable;
	dbg_func_in();		
	enable = atomic_read(&max17040_data.set_test);
	return sprintf(buf, "%d\n", enable);
}
static ssize_t max17040_store_flag(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u8 scale = (u8)simple_strtoul(buf, NULL, 10);
	dbg_func_in();			
	dbg("max17040_store_flag => [%d]\n",scale);
	atomic_set(&max17040_data.set_test, scale);	
	dbg_func_out();					
	return count;
}
static ssize_t max17040_start_quickstart(struct device *dev, struct device_attribute *attr, char *buf)
{
	int enable;
	sleep_dbg("QUICK START\n");
	max17040_restart();
	msleep(300); //quick start update time
	enable=max17040_data.quick_data.quick_state;
	return sprintf(buf, "%d\n", enable);
}

#ifdef MAX17040_DEBUG_QUICK			
static ssize_t max17040_get_low_voltage(struct device *dev, struct device_attribute *attr, char *buf)
{
	int enable;
	sleep_dbg("max17040_get_low_voltage START\n");
	max17040_restart();
	msleep(300); //quick start update time
	enable=max17040_data.vcell;
	return sprintf(buf, "%d\n", enable);
}
static ssize_t max17040_get_low_soc(struct device *dev, struct device_attribute *attr, char *buf)
{
	int enable;
	sleep_dbg("max17040_get_low_soc START\n");
	enable=max17040_data.soc;
	return sprintf(buf, "%d\n", enable);
}

#endif


static DEVICE_ATTR(setflag, S_IWUSR | S_IRUGO, max17040_show_flag, max17040_store_flag);
static DEVICE_ATTR(quickstart, S_IWUSR | S_IRUGO, max17040_start_quickstart, NULL);
#ifdef MAX17040_DEBUG_QUICK			
static DEVICE_ATTR(voltage, S_IWUSR | S_IRUGO, max17040_get_low_voltage, NULL);
static DEVICE_ATTR(soc, S_IWUSR | S_IRUGO, max17040_get_low_soc, NULL);
#endif


static struct attribute *max17040_attrs[] = {
	&dev_attr_setflag.attr, // add  test filed
	&dev_attr_quickstart.attr, // quickstart test filed
#ifdef MAX17040_DEBUG_QUICK			
	&dev_attr_voltage.attr, // quickstart test filed
	&dev_attr_soc.attr, // quickstart test filed
#endif
	NULL,
};

static struct attribute_group max17040_attr_group = {
	.attrs = max17040_attrs,
};

#ifdef MAX17040_ALARM_RTC_ENABLE
static void max17040_program_alarm_set(struct max17040_chip *di, int seconds)
{
	ktime_t low_interval = ktime_set(seconds - 10, 0);
	ktime_t slack = ktime_set(20, 0);
	ktime_t next;
	ktime_t finish;	
	dbg_func_in();
	next = ktime_add(di->last_poll, low_interval);
	finish=ktime_add(next, slack);
	alarm_start_range(&di->alarm, next, finish);
	dbg_func_out();	
}

static void max17040_schedule(void)
{
	switch(max17040_data.online)
		{
		case 0 : //usb not connected
			if(max17040_data.slow_poll) //usb not connected and suspend
			{
			sleep_dbg("CONFIG SLOW_POLL [The alarm is set for [%d] seconds ]\n",SLOW_POLL);
			max17040_program_alarm_set(&max17040_data, SLOW_POLL);		
			}
			else
			{
			sleep_dbg("CONFIG FAST_POLL [The alarm is set for [%d] seconds]\n",FAST_POLL);	
			max17040_program_alarm_set(&max17040_data, FAST_POLL);
			}			
			break;
		case 1 : //usb connection
			sleep_dbg("CONFIG FAST_POLL [The alarm is set for [%d] seconds]\n",FAST_POLL);	
			max17040_program_alarm_set(&max17040_data, FAST_POLL);			
			break;
		default:	
			sleep_dbg("CONFIG [Time setting is not]\n");				
			break;			
		}
}
#endif

static void max17040_work(struct work_struct *work)
{
#ifdef MAX17040_ALARM_RTC_ENABLE
	struct max17040_chip *di =
		container_of(work, struct max17040_chip, monitor_work);
#else
	struct max17040_chip *di = container_of(work, struct max17040_chip, work.work);
#endif //MAX17040_ALARM_RTC_ENABLE

#ifdef CONFIG_SKY_SMB136S_CHARGER
	int thermal_time_check_count = 0;
	unsigned long pm_temp = 0;
	unsigned long pm_temp_sum = 0;
	unsigned long current_batt_delta = 0;
	unsigned long pre_batt_delta = 0;
	int batt_delta_sum = 0;
	int batt_delta = 0;
	int chg_type;
#endif

//	unsigned long flags;	
	int enable;	
	dbg_func_in();
	max17040_data.event=NoEvents;
	max17040_data.suspend_event=NoEvents;
	
#ifndef MAX17040_ALARM_RTC_ENABLE	
//	sleep_dbg("MAX17040_WORK CALL.\n");				
	//prevent suspend
	max17040_prevent_suspend();
#else
//	sleep_dbg("MAX17040_WORK RTC CALL.\n");				
#endif //MAX17040_ALARM_RTC_ENABLE

	//read voltage now
	max17040_get_vcell();

	//read soc 
	max17040_get_soc();
#ifdef MAX17040_DEBUG_QUICK	
	max17040_quick_get_value();	
#endif
	enable = atomic_read(&max17040_data.set_test);
	//save volate now and soc [TEST APPLICATION]
	if(enable)
	{	
		if(max17040_data.event) // soc is changed 
		{	
			pr_err("MAX17040 SET TEST.\n");						
			sleep_dbg("MAX17040_WORK SOC [%d] prev[%d] : voltage [%d] : charger_state [%d] : i2c state [%d]\n", 
				max17040_data.soc,max17040_data.prev_soc, max17040_data.vcell,charge_state,max17040_data.i2c_state);							
			input_report_abs(max17040_data.max17040_input_data, ABS_X, max17040_data.vcell);
			input_report_abs(max17040_data.max17040_input_data, ABS_Y, max17040_data.soc);
		    	input_report_abs(max17040_data.max17040_input_data, ABS_WAKE, enable);		
			input_sync(max17040_data.max17040_input_data);	
		}

	}
	
	//After you determine the value of voltage and soc If there are changes to the event generates.
	if(max17040_data.slow_poll) //do not call early resume state.
	{
		if(max17040_data.suspend_event)// 15 percent below where they were soc.
			{
				pr_err("low battery [%d] percent!!!!!!!\n",max17040_data.soc);				
				power_supply_changed(&di->battery);	
				pr_err("[SLEEP_EVENT] 15 percent below Send Event [%d].\n",max17040_data.soc );				
			}
		else	//15 percent up where were soc.
			{	
				power_supply_changed(&di->battery);					
				pr_err("[SLEEP_EVENT] 15 percent up Send Event [%d].\n",max17040_data.soc );				
			}

	}
	else //call early resume state.
	{
		if(max17040_data.event) {	//Different values soc.
			pr_err("MAX17040_WORK SOC [%d] prev[%d] : voltage [%d] : charger_state [%d] : i2c state [%d]\n", 
				max17040_data.soc,max17040_data.prev_soc, max17040_data.vcell,charge_state,max17040_data.i2c_state);	
			power_supply_changed(&di->battery);
		}
		else	 {	//same values soc.
			//pr_err("[EVENT] Stop Event.\n");			
		}
	}

#ifdef NEED_FIX_BUILD_ERROR
	if(max17040_raw_soc != max17040_raw_pre_soc) {
		pr_err("[SOC:%d->%d]\n", max17040_raw_pre_soc, max17040_raw_soc);
		/* if battery is fully charged, charging done*/
		if(sky_get_plug_state() == 1 && 
			(max17040_raw_pre_soc >= (CHARGING_DONE_THRESHOLD-5) 
			&& max17040_raw_soc >= CHARGING_DONE_THRESHOLD)) {
			notify_event_charging_done();
		}

		/* if gauge falls below a threshold, recharging */
		if(sky_get_plug_state() == 0 && 
			(max17040_raw_pre_soc >= (RECHARGING_THRESHOLD-5) 
			&& max17040_raw_soc <= RECHARGING_THRESHOLD)) {
			notify_event_recharging();
		}
	}
#endif

#ifdef MAX17040_ALARM_RTC_ENABLE
	di->last_poll=alarm_get_elapsed_realtime();		
	/* prevent suspend before starting the alarm */
	local_irq_save(flags);	
	max17040_schedule();
	max17040_allow_suspend();			
	local_irq_restore(flags);	
	dbg_func_out();	
#else
#ifdef CONFIG_SKY_SMB136S_CHARGER
	chg_type = atomic_read(&smb_charger_state);
#ifdef MAX17040_DEBUG_QUICK	
        pmic8058_tz_get_temp_charging(&pm_temp);
#endif
	if(thermal_time_check_count != 3) { 
		pm_temp_sum = pm_temp_sum + pm_temp;
		current_batt_delta = pm_temp;
		if (pm_temp > pre_batt_delta) {
			batt_delta = pm_temp - pre_batt_delta;
			batt_delta_sum = batt_delta_sum + batt_delta;
		}
		else {
			batt_delta = 0;
			batt_delta_sum = batt_delta_sum + batt_delta;
		}
		pre_batt_delta = current_batt_delta;
		thermal_time_check_count ++;
	}
	else {
		if(chg_type == 0) {
			if(((pm_temp_sum > 114000) || (batt_delta_sum > 600 && pm_temp_sum > 84000)) && max17040_data.set_rcomp_mode == 0) {
				high_current_rcomp_mode = 1;
				max17040_set_rcomp(); 
			}
			else {
			    if(max17040_data.set_rcomp_mode == 1) {
				high_current_rcomp_mode = 0;
				max17040_set_rcomp();
			    }
			}
		}	
		else {
				high_current_rcomp_mode = 0;
				max17040_set_rcomp();
		}
		thermal_time_check_count = 0;
		pm_temp_sum = 0;
		batt_delta_sum = 0;
			
	}
#endif
	max17040_data.slow_poll = Early_resume;
	max17040_allow_suspend();			
	schedule_delayed_work(&max17040_data.work, MAX17040_DELAY);	
#endif // MAX17040_ALARM_RTC_ENABLE
}

#ifdef MAX17040_ALARM_RTC_ENABLE
static void max_battery_alarm_callback(struct alarm *alarm)
{

	struct max17040_chip *di =
		container_of(alarm, struct max17040_chip, alarm);
	dbg_func_in();
	sleep_dbg("MAX17040_ALARM_CALLBACK CALL.\n");					
	/*enable wake_lock*/
	max17040_prevent_suspend();
	/*schdule workqueue*/
	queue_work(di->monitor_wqueue, &di->monitor_work);
	dbg_func_out();	
}

#ifdef CONFIG_HAS_EARLYSUSPEND
void max17040_batt_early_suspend(struct early_suspend *h)
{
	dbg_func_in();
	/* If we are on battery, reduce our update rate until
	 * we next resume.
	 */
	/* Runs only when the cable is not connected.*/
	max17040_data.online=sky_get_plug_state();
	max17040_data.slow_poll = Early_suspend;	
	if(!max17040_data.online)
	{
	sleep_dbg("[IMPORT]USB CABLE is not connected\n");				
	max17040_program_alarm_set(&max17040_data, SLOW_POLL);
	sleep_dbg("CONFIG SLOW_POLL [The alarm is set for [%d] seconds ]\n",SLOW_POLL);	
	}
	else
	{
	sleep_dbg("[IMPORT]USB CABLE is connected\n");			
	max17040_program_alarm_set(&max17040_data, FAST_POLL);
	sleep_dbg("CONFIG SLOW_POLL [The alarm is set for [%d] seconds ]\n",FAST_POLL);	
	}
	dbg_func_out();

}

void max17040_batt_late_resume(struct early_suspend *h)
{
	dbg_func_in();
	/* We might be on a slow sample cycle.  If we're
	 * resuming we should resample the battery state
	 * if it's been over a minute since we last did
	 * so, and move back to sampling every minute until
	 * we suspend again.
	 */
	if (max17040_data.slow_poll) {
		max17040_program_alarm_set(&max17040_data, FAST_POLL);
		max17040_data.slow_poll = Early_resume;
		sleep_dbg("CONFIG FAST_POLL [The alarm is set for [%d] seconds]\n",FAST_POLL);	
	}
	dbg_func_out();	
}
#endif// CONFIG_HAS_EARLYSUSPEND
#endif// MAX17040_ALARM_RTC_ENABLE

#ifdef CONFIG_SKY_SMB136S_CHARGER
static int max17040_get_capacity(void)
{
	if(max17040_data.i2c_state)
		return  max17040_data.prev_soc;		

	return  max17040_data.soc;
}

static int max17040_get_mvolts(void)
{
	if(max17040_data.i2c_state_vol)
		return  max17040_data.prev_voltage*1000;		

	return  max17040_data.vcell*1000;
}

struct msm_battery_func max17040_ms_batt_func = {
	.get_batt_mvolts = max17040_get_mvolts,
	.get_batt_capacity = max17040_get_capacity,
};
#endif

static int __devinit max17040_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
    struct input_dev *input_data = NULL;	
	int ret;
#ifdef USE_MAX17040_QUICKSTART
	int aflag=0;
#endif
	printk("[MAX17040] max17040_probe [IN]\n");	
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE))
		return -EIO;

	max17040_data.client = client;
	i2c_set_clientdata(client, &max17040_data);

#ifdef CONFIG_SKY_SMB136S_CHARGER
	msm_battery_func_register(&max17040_ms_batt_func);
#endif

	//sys file system are registered
	max17040_data.battery.name		= "batterys";
	max17040_data.battery.type		= POWER_SUPPLY_TYPE_BATTERY;
	max17040_data.battery.get_property	= max17040_get_property;
	max17040_data.battery.properties	= max17040_battery_props;
	max17040_data.battery.external_power_changed = max17040_bat_external_power_changed;
	max17040_data.battery.num_properties	= ARRAY_SIZE(max17040_battery_props);
	
	ret = power_supply_register(&client->dev, &max17040_data.battery);
	if (ret) {
		pr_err("[MAX17040] failed: power supply register [ERROR]\n");			
		i2c_set_clientdata(client, NULL);
		return ret;
	}
	
	//The code used in the test mode [TEST MODE]	
	ret = sysfs_create_group(&client->dev.kobj, &max17040_attr_group);	
	if (ret) {
		pr_err("[MAX17040] failed: sysfs_create_group  [ERROR]\n");					
	}	
	
	//mutex is init
	mutex_init(&max17040_data.data_mutex);
	mutex_init(&max17040_data.i2c_mutex);	
	mutex_init(&max17040_data.quick_mutex);		
	
	//rcomp is set
	max17040_set_rcomp();
	
	//Version of reading
	max17040_get_version();
	
	//read vell and soc 
	max17040_quick_get_vcell();

#ifdef USE_MAX17040_QUICKSTART
	//check quick start
	aflag=max17040_check_restart(max17040_data.quick_data.quick_vcell,max17040_data.quick_data.quick_soc);
	if(aflag)
	{
	max17040_restart();	
	msleep(300); //quick start update time
	}
#endif

	//The code used in the test mode [TEST MODE]
	atomic_set(&max17040_data.set_test, 0);	

    input_data = input_allocate_device();
  if (!input_data) {
        pr_err("[MAX17040]: Unable to input_allocate_device \n");  	
		return -1;
    }
	
    set_bit(EV_ABS,input_data->evbit);
    input_set_capability(input_data, EV_ABS, ABS_X);	
    input_set_capability(input_data, EV_ABS, ABS_Y); /* wake */	
    input_set_capability(input_data, EV_ABS, ABS_WAKE); /* wake */
	input_data->name="max17040";
	
    ret =input_register_device(input_data);
    if (ret) {
        pr_err("[MAX17040]: Unable to register input_data device\n");
		return -1;
    }
	
    input_set_drvdata(input_data, &max17040_data);	
	max17040_data.max17040_input_data=input_data;
	//initialize workqueue and alarm setting
	wake_lock_init(&max17040_data.work_wake_lock, WAKE_LOCK_SUSPEND,
			"max17040-battery");


#ifdef MAX17040_ALARM_RTC_ENABLE
	max17040_data.last_poll=alarm_get_elapsed_realtime();	
	INIT_WORK(&max17040_data.monitor_work, max17040_work);
	max17040_data.monitor_wqueue = create_freezeable_workqueue("max17040");
	
	/* init to something sane */
	if (!max17040_data.monitor_wqueue) {
		sleep_dbg("fail_workqueue Error [PROBE FUNCTION]");
		return -1;
	}
	
	alarm_init(&max17040_data.alarm, ANDROID_ALARM_ELAPSED_REALTIME_WAKEUP,
			max_battery_alarm_callback);
	
	//prevent suspend
	max17040_prevent_suspend();
#ifdef CONFIG_HAS_EARLYSUSPEND
	max17040_data.early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	max17040_data.early_suspend.suspend = max17040_batt_early_suspend;
	max17040_data.early_suspend.resume = max17040_batt_late_resume;
	dbg("set max17040 EARLY_SUSPEND\n");	
	register_early_suspend(&max17040_data.early_suspend);
#endif	//CONFIG_HAS_EARLYSUSPEND
	queue_work(max17040_data.monitor_wqueue, &max17040_data.monitor_work);
	
#else
	INIT_DELAYED_WORK_DEFERRABLE(&max17040_data.work, max17040_work);
	schedule_delayed_work(&max17040_data.work, 0);
#endif //MAX17040_ALARM_RTC_ENABLE
//	sleep_dbg("[MAX17040] max17040_probe [OUT]\n");	
        pr_info("[MAX17040]max17040_probe...\n");
	return 0;
}

static int __devexit max17040_remove(struct i2c_client *client)
{
	struct max17040_chip *chip = i2c_get_clientdata(client);
	power_supply_unregister(&chip->battery);
	i2c_set_clientdata(client, NULL);
	return 0;
}

static int max17040_sleep_time(int soc)
{
	int time =0;
	if(soc>5)
	time=(soc-5)*SLEEP_ONE_HOUR;		
	else
	time=SLEEP_THREE_MINUTE;
	return time;
}

#ifdef CONFIG_PM
static int max17040_suspend(struct i2c_client *client,
		pm_message_t state)
{
	int time =0;
	int soc=max17040_data.soc;
	dbg_func_in();

	/* If we are on battery, reduce our update rate until
	 * we next resume.
	 */
	/*ps2 team shs : Runs only when the cable is not connected.*/
	max17040_data.online=sky_get_plug_state();
	max17040_data.slow_poll = Early_suspend;	
	//ps1 team shs : cancel schedule_delayer_work
	cancel_delayed_work(&max17040_data.work);	
	//ps1 team shs : set time
	time=max17040_sleep_time(soc);
	msm_pm_set_max_sleep_time((int64_t)((int64_t) time * NSEC_PER_SEC)); 	
	sleep_dbg("[SUSPEND] set time [%d] seconds : soc [%d] ]\n",time,soc);		
	dbg_func_out();
	return 0;
}

static int max17040_resume(struct i2c_client *client)
{
	dbg_func_in();
	if (max17040_data.slow_poll) {
		schedule_delayed_work(&max17040_data.work, 0);		
		sleep_dbg("[RESUME] CONFIG FAST_POLL [The alarm is set for [%d] seconds]\n",FAST_POLL);	
	}
	dbg_func_out();
	return 0;
}

#else

#define max17040_suspend NULL
#define max17040_resume NULL

#endif /* CONFIG_PM */

static const struct i2c_device_id max17040_id[] = {
	{ "max17040", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, max17040_id);

static struct i2c_driver max17040_i2c_driver = {
	.driver	= {
		.name	= "max17040",	
	},
	.probe		= max17040_probe,
	#ifndef MAX17040_ALARM_RTC_ENABLE
	.resume = max17040_resume,
	.suspend = max17040_suspend,
	#endif
	.remove		= __devexit_p(max17040_remove),
	.id_table	= max17040_id,
};

static int __init max17040_init(void)
{
	return i2c_add_driver(&max17040_i2c_driver);
}
module_init(max17040_init);

static void __exit max17040_exit(void)
{
	i2c_del_driver(&max17040_i2c_driver);
}
module_exit(max17040_exit);

//MODULE_AUTHOR("Minkyu Kang <mk7.kang@samsung.com>");
MODULE_DESCRIPTION("MAX17040 Fuel Gauge");
MODULE_LICENSE("GPL");



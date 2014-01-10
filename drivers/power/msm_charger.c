/* Copyright (c) 2010-2012, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/mfd/pmic8058.h>
#include <linux/interrupt.h>
#include <linux/power_supply.h>
#include <linux/delay.h>
#include <linux/bitops.h>
#include <linux/debugfs.h>
#include <linux/msm-charger.h>
#include <linux/time.h>
#include <linux/slab.h>
#include <linux/wakelock.h>

#include <asm/atomic.h>

#include <mach/msm_hsusb.h>
#ifdef CONFIG_SKY_CHARGING
#include <linux/reboot.h>
#endif

#if defined(CONFIG_SKY_SMB136S_CHARGER) || defined(CONFIG_SKY_SMB137B_CHARGER)
#include <mach/restart.h>
#endif 

#if defined(CONFIG_SKY_CHARGING) || defined(CONFIG_SKY_SMB136S_CHARGER) || defined(CONFIG_SKY_SMB137B_CHARGER)
#include <linux/sky_pm_attr.h>
#endif

#define PANTECH_CHARGER_TEST_MENU

#if defined(PANTECH_CHARGER_TEST_MENU)
#include <linux/proc_fs.h>
#include <linux/input.h>
#include <linux/types.h>
#include <linux/ioctl.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/rtc.h>
#define CHARER_IOCTL_MAGIC 'p'
#define CHARGER_MONITOR_TEST_SET_MSM	_IOR(CHARER_IOCTL_MAGIC, 1, int[3])//#4648 charging
#define CHARGER_DISCHARGING_TEST_SET_MSM _IOWR(CHARER_IOCTL_MAGIC, 2, int)//#8378522 Charging/Discharging Test
#define CHARGER_DISCHARGING_TEST_GET_MSM _IOR(CHARER_IOCTL_MAGIC, 3, int)//#8378522 Charging/Discharging Test
#define CHARGER_CHARGING_TEST_SET_MSM _IOR(CHARER_IOCTL_MAGIC, 4, int[4])//#8378522 Charging/Discharging Test
#endif



#ifdef CONFIG_PANTECH_FB_MSM_MHL_SII9244 // MHL_KKCHO
#define F_MHL_AUTO_CONNECT
#endif

#ifdef F_MHL_AUTO_CONNECT
extern void MHL_Set_cable_detect_handler(void);

#define MHL_CABLE_CONNCET			1
#define MHL_CABLE_DISCONNCET	       0
#endif

#ifdef CONFIG_PANTECH_LCD_MHL_CABLE_DETECT
static int is_mhl;
#endif

#define MSM_CHG_MAX_EVENTS		16
#define CHARGING_TEOC_MS		9000000
#define UPDATE_TIME_MS			60000
#define RESUME_CHECK_PERIOD_MS		60000

#if defined(CONFIG_SKY_SMB136S_CHARGER) || defined(CONFIG_SKY_SMB137B_CHARGER)
#define DEFAULT_BATT_MAX_V		4350
#else
#define DEFAULT_BATT_MAX_V		4200
#endif
#define DEFAULT_BATT_MIN_V		3200

#define MSM_CHARGER_GAUGE_MISSING_VOLTS 3500
#define MSM_CHARGER_GAUGE_MISSING_TEMP  35

#ifdef CONFIG_SKY_CHARGING
#define MSM_CHARGER_TA_CURRENT_INCALL 400
#define MSM_CHARGER_TA_CURRENT_IDLE  900
#endif


//#define FEATURE_FACTORY_CABLE_DETECT	
//#define FEATURE_POWEROFF_CHARGING

/**
 * enum msm_battery_status
 * @BATT_STATUS_ABSENT: battery not present
 * @BATT_STATUS_ID_INVALID: battery present but the id is invalid
 * @BATT_STATUS_DISCHARGING: battery is present and is discharging
 * @BATT_STATUS_TRKL_CHARGING: battery is being trickle charged
 * @BATT_STATUS_FAST_CHARGING: battery is being fast charged
 * @BATT_STATUS_JUST_FINISHED_CHARGING: just finished charging,
 *		battery is fully charged. Do not begin charging untill the
 *		voltage falls below a threshold to avoid overcharging
 * @BATT_STATUS_TEMPERATURE_OUT_OF_RANGE: battery present,
					no charging, temp is hot/cold
 */
enum msm_battery_status {
	BATT_STATUS_ABSENT,
	BATT_STATUS_ID_INVALID,
	BATT_STATUS_DISCHARGING,
	BATT_STATUS_TRKL_CHARGING,
	BATT_STATUS_FAST_CHARGING,
	BATT_STATUS_JUST_FINISHED_CHARGING,
	BATT_STATUS_TEMPERATURE_OUT_OF_RANGE,
};

struct msm_hardware_charger_priv {
	struct list_head list;
	struct msm_hardware_charger *hw_chg;
	enum msm_hardware_charger_state hw_chg_state;
	unsigned int max_source_current;
	struct power_supply psy;
#if defined(CONFIG_SKY_CHARGING) || defined(CONFIG_SKY_SMB136S_CHARGER) || defined(CONFIG_SKY_SMB137B_CHARGER)
	struct power_supply psy_ac;
#endif
};

struct msm_charger_event {
	enum msm_hardware_charger_event event;
	struct msm_hardware_charger *hw_chg;
};

struct msm_charger_mux {
	int inited;
	struct list_head msm_hardware_chargers;
	int count_chargers;
	struct mutex msm_hardware_chargers_lock;

	struct device *dev;

	unsigned int max_voltage;
	unsigned int min_voltage;

	unsigned int safety_time;
#if 0 // P11220 20130910 : disable safety timer
	struct delayed_work teoc_work;
#endif

	unsigned int update_time;
	int stop_update;
	struct delayed_work update_heartbeat_work;

	struct mutex status_lock;
	enum msm_battery_status batt_status;
	struct msm_hardware_charger_priv *current_chg_priv;
	struct msm_hardware_charger_priv *current_mon_priv;

	unsigned int (*get_batt_capacity_percent) (void);

	struct msm_charger_event *queue;
	int tail;
	int head;
	spinlock_t queue_lock;
	int queue_count;
	struct work_struct queue_work;
	struct workqueue_struct *event_wq_thread;
	struct wake_lock wl;

#if defined(PANTECH_CHARGER_TEST_MENU)	
	int batt_id;
	int cable_id;
	struct rtc_device *rtc;
	struct rtc_wkalrm	alm;
#endif


#ifdef CONFIG_SKY_CHARGING
    unsigned int charger_type;
#endif  

#if defined(CONFIG_SKY_CHARGING) || defined(CONFIG_SKY_SMB136S_CHARGER) || defined(CONFIG_SKY_SMB137B_CHARGER)
    int is_factory_cable;
    int is_verylow_battery;
#endif

#if defined(CONFIG_SKY_CHARGING) || defined(CONFIG_SKY_SMB136S_CHARGER) || defined(CONFIG_SKY_SMB137B_CHARGER)
    int init_completed;
    struct delayed_work init_completed_work;
#endif
};

#if defined(CONFIG_SKY_CHARGING) || defined(CONFIG_SKY_SMB136S_CHARGER) || defined(CONFIG_SKY_SMB137B_CHARGER)
extern atomic_t smb_charger_state;
extern unsigned int pantech_charging_status(void);
#endif

#if defined(PANTECH_CHARGER_TEST_MENU)
static struct input_dev *bms_input_dev;
static atomic_t bms_input_flag;
#endif

static struct msm_charger_mux msm_chg;

static struct msm_battery_gauge *msm_batt_gauge;

#if defined(CONFIG_SKY_CHARGING) || defined(CONFIG_SKY_SMB136S_CHARGER) || defined(CONFIG_SKY_SMB137B_CHARGER)
struct msm_battery_func *msm_batt_func = NULL;
#endif

#if defined(CONFIG_SKY_CHARGING) || defined(CONFIG_SKY_SMB136S_CHARGER) || defined(CONFIG_SKY_SMB137B_CHARGER)

/*****************************
   MSM Charger Sysfile
******************************/

SKY_ATTR_SHOW_FUNC(init_completed)
{
	static struct msm_charger_mux *chg = &msm_chg;
	return sprintf(buf, "%d\n", chg->init_completed);
}

SKY_ATTR_STORE_FUNC(init_completed)
{
	static struct msm_charger_mux *chg = &msm_chg;
	ssize_t ret;
	int val;

	ret = kstrtoint(buf, 0, &val);

	chg->init_completed = val;
	printk("set attr init_completed:%d, size=%d\n", chg->init_completed, size);		
	return size;
}


static struct device_attribute msm_charger_sysfs_attrs[] = {
	SKY_ATTR(init_completed),	
};

/*****************************
   End of MSM Charger Sysfile
******************************/

static void init_completed_work_func(struct work_struct *work)
{
    // check init completed
    if (msm_batt_gauge
	&& msm_chg.inited == 2
#ifdef CONFIG_SKY_CHARGING
         && msm_batt_func
#endif         
         ) 
    {
       msm_chg.init_completed = 1;
	dev_info(msm_chg.dev, "%s  : init_completed !!!! = %d\n", __func__, msm_chg.init_completed);
    }
    else
    {
       schedule_delayed_work(&msm_chg.init_completed_work, msecs_to_jiffies(100));
    }
}
#endif


#ifdef CONFIG_SKY_CHARGING
//p1s team shs : check state
int sky_is_batt_status(void)
{
	int ret=msm_chg.batt_status;
	return ret;
}
//p1s team shs : read soc
#endif


#ifdef CONFIG_SKY_CHARGING
int g_prev_charger_type = CHG_TYPE_NONE;
static int msm_get_prev_charger_type(void)
{
	return g_prev_charger_type;
}

static void msm_set_charger_type(int type)
{
        pr_info("[SKY CHG] msm_set_charger_type charger_type =%d\n", type);
        g_prev_charger_type = msm_chg.charger_type;
        msm_chg.charger_type = type;
}

static int msm_get_charger_type(void)
{
        pr_info("[SKY CHG] msm_get_charger_type charger_type =%d\n", msm_chg.charger_type);
        return msm_chg.charger_type;
}
#endif

#if defined(CONFIG_SKY_BATTERY_MAX17040) || defined(CONFIG_SKY_BATTERY_MAX17043)
extern int max17040_get_charge_state(void);
#endif

#ifdef CONFIG_SKY_CHARGING
extern void pm8058_chg_set_current_temperature(int chg_current);
//extern int pmic8058_tz_get_temp_charging(unsigned long *temp);
extern void pm8058_chg_set_current_incall(int chg_current);
static unsigned int old_status = 0;

unsigned int msm_charger_is_incall(void)
{
        return old_status;
}

void msm_charger_set_current_incall(unsigned int in_call)
{
        int chg_current;

        if(old_status == in_call)
                return;
        else
                old_status = in_call;

        if(in_call)
                chg_current = MSM_CHARGER_TA_CURRENT_INCALL;
        else
                chg_current = MSM_CHARGER_TA_CURRENT_IDLE;
            
	if (msm_chg.current_chg_priv
		&& (msm_chg.current_chg_priv->hw_chg_state
			== CHG_READY_STATE || msm_chg.current_chg_priv->hw_chg_state
			== CHG_CHARGING_STATE) && msm_get_charger_type() == CHG_TYPE_AC) {
			pm8058_chg_set_current_incall(chg_current);
	}
}
EXPORT_SYMBOL(msm_charger_set_current_incall);

void msm_charger_set_lcd_onoff(unsigned int onoff)
{
        int chg_current;

        if(msm_charger_is_incall())
                return;
            
        if(onoff)
                chg_current = 700;
        else
                chg_current = 900;
            
	if (msm_chg.current_chg_priv
		&& (msm_chg.current_chg_priv->hw_chg_state
			== CHG_READY_STATE || msm_chg.current_chg_priv->hw_chg_state
			== CHG_CHARGING_STATE) && msm_get_charger_type() == CHG_TYPE_AC) {
			pm8058_chg_set_current_temperature(chg_current);
	}
}
EXPORT_SYMBOL(msm_charger_set_lcd_onoff);
#endif

static int is_chg_capable_of_charging(struct msm_hardware_charger_priv *priv)
{
	if (priv->hw_chg_state == CHG_READY_STATE
	    || priv->hw_chg_state == CHG_CHARGING_STATE)
		return 1;

	return 0;
}

static int is_batt_status_capable_of_charging(void)
{
	pr_info("%s, msm_chg.batt_status[%d]", __func__, msm_chg.batt_status);

	if (msm_chg.batt_status == BATT_STATUS_ABSENT
	    || msm_chg.batt_status == BATT_STATUS_TEMPERATURE_OUT_OF_RANGE
	    || msm_chg.batt_status == BATT_STATUS_ID_INVALID
	    || msm_chg.batt_status == BATT_STATUS_JUST_FINISHED_CHARGING)
		return 0;
	return 1;
}

static int is_batt_status_charging(void)
{
	if (msm_chg.batt_status == BATT_STATUS_TRKL_CHARGING
	    || msm_chg.batt_status == BATT_STATUS_FAST_CHARGING)
		return 1;
	return 0;
}

#ifndef CONFIG_SKY_CHARGING
#if defined(CONFIG_SKY_BATTERY_MAX17040) || defined(CONFIG_SKY_BATTERY_MAX17043)
int sky_get_plug_state(void)
{
        return is_batt_status_charging();
}
#endif  //CONFIG_SKY_BATTERY_MAX17040
#endif

static int is_battery_present(void)
{
	if (msm_batt_gauge && msm_batt_gauge->is_battery_present)
		return msm_batt_gauge->is_battery_present();
	else {
		pr_err("msm-charger: no batt gauge batt=absent\n");
		return 0;
	}
}

#ifndef CONFIG_SKY_CHARGING
static int is_battery_temp_within_range(void)
{
	if (msm_batt_gauge && msm_batt_gauge->is_battery_temp_within_range)
		return msm_batt_gauge->is_battery_temp_within_range();
	else {
		pr_err("msm-charger no batt gauge batt=out_of_temperatur\n");
		return 0;
	}
}
#endif

static int is_battery_id_valid(void)
{
	if (msm_batt_gauge && msm_batt_gauge->is_battery_id_valid)
		return msm_batt_gauge->is_battery_id_valid();
	else {
		pr_err("msm-charger no batt gauge batt=id_invalid\n");
		return 0;
	}
}

#ifdef F_MHL_AUTO_CONNECT
static int is_mhl_cable(void)
{
	if (msm_batt_gauge && msm_batt_gauge->is_mhl_cable)
		return msm_batt_gauge->is_mhl_cable();
	else {
		pr_err("msm-charger no is_mhl_cable\n");
		return 0;
	}
}
#endif

#ifdef CONFIG_PANTECH_LCD_MHL_CABLE_DETECT
int is_mhl_mode(void)
{
	if(is_mhl){
		is_mhl = 1;
		return 1;
	}else{
		is_mhl = 0;
		return 0;
	}
}
void set_flag_mhl_mode(int val)
{
	is_mhl = val;
}
#endif

#if defined(CONFIG_SKY_CHARGING) || defined(CONFIG_SKY_SMB136S_CHARGER) || defined(CONFIG_SKY_SMB137B_CHARGER)
/*
static int is_cable_present(void)
{
	struct msm_hardware_charger_priv *priv = msm_chg.current_chg_priv;

	if (priv)
		return (priv->hw_chg_state != CHG_ABSENT_STATE);
	return 0;
}
*/
static int get_battery_id(void)
{
	if (msm_batt_gauge && msm_batt_gauge->get_battery_id)
		return msm_batt_gauge->get_battery_id();
	pr_err("msm-charger read battery id error \n");
	return 0;
}

static int get_cable_id(void)
{
	if (msm_batt_gauge && msm_batt_gauge->get_cable_id)
		return msm_batt_gauge->get_cable_id();
	pr_err("msm-charger read cable id error \n");
	return 0;
}

static int is_factory_cable(void)
{
	if (msm_batt_gauge && msm_batt_gauge->is_factory_cable)
		return msm_batt_gauge->is_factory_cable();
		pr_err("msm-charger no is_factory_cable\n");
		return 0;
	}

#endif

static int get_prop_battery_mvolts(void)
{
#if defined(CONFIG_SKY_CHARGING) || defined(CONFIG_SKY_SMB136S_CHARGER) || defined(CONFIG_SKY_SMB137B_CHARGER)
	if (msm_batt_func && msm_batt_func->get_batt_mvolts)
		return msm_batt_func->get_batt_mvolts();
#else	
	if (msm_batt_gauge && msm_batt_gauge->get_battery_mvolts)
		return msm_batt_gauge->get_battery_mvolts();
#endif
	else {
		pr_err("msm-charger no batt gauge assuming 3.5V\n");
		return MSM_CHARGER_GAUGE_MISSING_VOLTS;
	}
}

#ifndef CONFIG_SKY_CHARGING
static int get_battery_temperature(void)
{
	if (msm_batt_gauge && msm_batt_gauge->get_battery_temperature)
		return msm_batt_gauge->get_battery_temperature();
	else {
		pr_err("msm-charger no batt gauge assuming 35 deg G\n");
		return MSM_CHARGER_GAUGE_MISSING_TEMP;
	}
}
#endif

static int get_prop_batt_capacity(void)
{
	int capacity = 0;

#if defined(CONFIG_SKY_CHARGING) || defined(CONFIG_SKY_SMB136S_CHARGER) || defined(CONFIG_SKY_SMB137B_CHARGER)
	if (msm_batt_func && msm_batt_func->get_batt_capacity)
		capacity = msm_batt_func->get_batt_capacity();
#else	
	if (msm_batt_gauge && msm_batt_gauge->get_batt_remaining_capacity)
		capacity = msm_batt_gauge->get_batt_remaining_capacity();
	else
		capacity = msm_chg.get_batt_capacity_percent();
#endif
	if (capacity <= 10)
		pr_err("battery capacity very low = %d\n", capacity);

	return capacity;
}

static int get_prop_batt_health(void)
{
	int status = 0;

	if (msm_chg.batt_status == BATT_STATUS_TEMPERATURE_OUT_OF_RANGE)
		status = POWER_SUPPLY_HEALTH_OVERHEAT;
	else
		status = POWER_SUPPLY_HEALTH_GOOD;

	return status;
}

static int get_prop_charge_type(void)
{
	int status = 0;

	if (msm_chg.batt_status == BATT_STATUS_TRKL_CHARGING)
		status = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
	else if (msm_chg.batt_status == BATT_STATUS_FAST_CHARGING)
		status = POWER_SUPPLY_CHARGE_TYPE_FAST;
	else
		status = POWER_SUPPLY_CHARGE_TYPE_NONE;

	return status;
}

static int get_prop_batt_status(void)
{
	int status = 0;
#if defined(CONFIG_SKY_BATTERY_MAX17040) || defined(CONFIG_SKY_BATTERY_MAX17043)
	int state = 0;
#endif
	if (msm_batt_gauge && msm_batt_gauge->get_battery_status) {
		status = msm_batt_gauge->get_battery_status();
		if (status == POWER_SUPPLY_STATUS_CHARGING ||
			status == POWER_SUPPLY_STATUS_FULL ||
			status == POWER_SUPPLY_STATUS_DISCHARGING)
			return status;
	}

	if (is_batt_status_charging())
	{
#if defined(CONFIG_SKY_BATTERY_MAX17040) || defined(CONFIG_SKY_BATTERY_MAX17043)
		//ps1 team shs : modify code
		state=max17040_get_charge_state();
		if(state)
			status = POWER_SUPPLY_STATUS_FULL;
		else
			status = POWER_SUPPLY_STATUS_CHARGING;				
#else
		status = POWER_SUPPLY_STATUS_CHARGING;
#endif
	}
	else if (msm_chg.batt_status ==
		 BATT_STATUS_JUST_FINISHED_CHARGING
			 && msm_chg.current_chg_priv != NULL)
	{
#if defined(CONFIG_SKY_BATTERY_MAX17040) || defined(CONFIG_SKY_BATTERY_MAX17043)
		//ps1 team shs : modify code
		state=max17040_get_charge_state();
		if(state)
			status = POWER_SUPPLY_STATUS_FULL;
		else
			status = POWER_SUPPLY_STATUS_CHARGING;				
#else
		status = POWER_SUPPLY_STATUS_FULL;
#endif
	}
	else
		status = POWER_SUPPLY_STATUS_DISCHARGING;
	return status;
}

 /* This function should only be called within handle_event or resume */
#define BATT_VERYLOW_MV    3000000
static void update_batt_status(void)
{
	if (is_battery_present()) {
#ifdef CONFIG_SKY_CHARGING
	// check very low battery !!
	msm_chg.is_verylow_battery = (get_prop_battery_mvolts() < BATT_VERYLOW_MV); // 3000mV
	
    	if (msm_chg.batt_status == BATT_STATUS_ABSENT
    		|| msm_chg.batt_status
    			== BATT_STATUS_ID_INVALID) {
    		//msm_chg.stop_resume_check = 0;
    		msm_chg.batt_status = BATT_STATUS_DISCHARGING;
            }
#else
		if (is_battery_id_valid()) {
			if (msm_chg.batt_status == BATT_STATUS_ABSENT
				|| msm_chg.batt_status
					== BATT_STATUS_ID_INVALID) {
				msm_chg.batt_status = BATT_STATUS_DISCHARGING;
			}
		} else
			msm_chg.batt_status = BATT_STATUS_ID_INVALID;
#endif
	 } else
		msm_chg.batt_status = BATT_STATUS_ABSENT;
}

#if defined(CONFIG_SKY_CHARGING) || defined(CONFIG_SKY_SMB136S_CHARGER) || defined(CONFIG_SKY_SMB137B_CHARGER)
static void update_cable_status(int is_connected)
{
	if (is_connected)
	{
		msm_chg.cable_id= get_cable_id();
		msm_chg.is_factory_cable = is_factory_cable();
	}
	else
	{
		msm_chg.cable_id= 0;
		msm_chg.is_factory_cable = 0;
	}		
}
#endif

static enum power_supply_property msm_power_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
};

static char *msm_power_supplied_to[] = {
	"battery",
};

static int msm_power_get_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	struct msm_hardware_charger_priv *priv;
#if defined(CONFIG_SKY_SMB136S_CHARGER) || defined(CONFIG_SKY_SMB137B_CHARGER)
	int chg_type;
#endif
	priv = container_of(psy, struct msm_hardware_charger_priv, psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = !(priv->hw_chg_state == CHG_ABSENT_STATE);
		break;
	case POWER_SUPPLY_PROP_ONLINE:
	val->intval = 0;
#ifdef CONFIG_SKY_CHARGING
	if (psy->type == POWER_SUPPLY_TYPE_USB ||
		psy->type == POWER_SUPPLY_TYPE_USB_DCP ||
		psy->type == POWER_SUPPLY_TYPE_USB_CDP ||
		psy->type == POWER_SUPPLY_TYPE_USB_ACA) {
			val->intval = ((msm_get_charger_type() == CHG_TYPE_USB)  || (msm_get_charger_type() == CHG_TYPE_FACTORY));
	}

	if(psy->type == POWER_SUPPLY_TYPE_MAINS) {
			val->intval = (msm_get_charger_type() == CHG_TYPE_AC);
	}
#elif defined(CONFIG_SKY_SMB136S_CHARGER) || defined(CONFIG_SKY_SMB137B_CHARGER)
	chg_type = atomic_read(&smb_charger_state);

	if (psy->type == POWER_SUPPLY_TYPE_USB ||
		psy->type == POWER_SUPPLY_TYPE_USB_DCP ||
		psy->type == POWER_SUPPLY_TYPE_USB_CDP ||
		psy->type == POWER_SUPPLY_TYPE_USB_ACA) {

		if(chg_type == CHG_TYPE_USB)
			val->intval = 1;
	}

	if(psy->type == POWER_SUPPLY_TYPE_MAINS) {
		if(chg_type == CHG_TYPE_AC)
			val->intval = 1;
	}
#else
		val->intval = (priv->hw_chg_state == CHG_READY_STATE)
			|| (priv->hw_chg_state == CHG_CHARGING_STATE);
#endif
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static enum power_supply_property msm_batt_power_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
};

static int msm_batt_power_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = get_prop_batt_status();
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = get_prop_charge_type();
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = get_prop_batt_health();
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = !(msm_chg.batt_status == BATT_STATUS_ABSENT);
		break;
#if defined(CONFIG_SKY_CHARGING) || defined(CONFIG_SKY_SMB136S_CHARGER) || defined(CONFIG_SKY_SMB137B_CHARGER)
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
#else
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_NiMH;
		break;
#endif
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = msm_chg.max_voltage * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = msm_chg.min_voltage * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = get_prop_battery_mvolts();
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
#if defined(CONFIG_SKY_CHARGING) || defined(CONFIG_SKY_SMB136S_CHARGER) || defined(CONFIG_SKY_SMB137B_CHARGER)
		if (msm_chg.batt_status == BATT_STATUS_ABSENT && msm_chg.is_factory_cable) // no battery factory cable state
			val->intval = 10;
		else	 if (get_prop_battery_mvolts() < 3000000)
			val->intval = 0;
		else
#endif		
		val->intval = get_prop_batt_capacity();
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static struct power_supply msm_psy_batt = {
	.name = "battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = msm_batt_power_props,
	.num_properties = ARRAY_SIZE(msm_batt_power_props),
	.get_property = msm_batt_power_get_property,
};

static int usb_chg_current;
static struct msm_hardware_charger_priv *usb_hw_chg_priv;
static void (*notify_vbus_state_func_ptr)(int);
static int usb_notified_of_insertion;

/* this is passed to the hsusb via platform_data msm_otg_pdata */
int msm_charger_register_vbus_sn(void (*callback)(int))
{
	pr_debug(KERN_INFO "%s\n", __func__);
	notify_vbus_state_func_ptr = callback;
	return 0;
}

/* this is passed to the hsusb via platform_data msm_otg_pdata */
void msm_charger_unregister_vbus_sn(void (*callback)(int))
{
	pr_debug(KERN_INFO "%s\n", __func__);
	notify_vbus_state_func_ptr = NULL;
}

#ifdef CONFIG_SKY_CHARGING
unsigned int hold_mA ;

static void msm_charge_called_before_init(unsigned int mA)
{
        pr_info("[SKY CHG] msm_charge_called_before_init with mA =%d\n",hold_mA);

        if(hold_mA == 500)
                msm_set_charger_type(CHG_TYPE_USB);
        else if(hold_mA == 900)
                msm_set_charger_type(CHG_TYPE_AC);
        else
                return;

        usb_hw_chg_priv->max_source_current = hold_mA;
        msm_charger_notify_event(usb_hw_chg_priv->hw_chg,
                CHG_ENUMERATED_EVENT);
}
#endif

static void notify_usb_of_the_plugin_event(struct msm_hardware_charger_priv
					   *hw_chg, int plugin)
{
	plugin = !!plugin;
	if (plugin == 1 && usb_notified_of_insertion == 0) {
		usb_notified_of_insertion = 1;
		if (notify_vbus_state_func_ptr) {
			dev_dbg(msm_chg.dev, "%s notifying plugin\n", __func__);
			(*notify_vbus_state_func_ptr) (plugin);
		} else
			dev_dbg(msm_chg.dev, "%s unable to notify plugin\n",
				__func__);
		usb_hw_chg_priv = hw_chg;
	}
	if (plugin == 0 && usb_notified_of_insertion == 1) {
		if (notify_vbus_state_func_ptr) {
			dev_dbg(msm_chg.dev, "%s notifying unplugin\n",
				__func__);
			(*notify_vbus_state_func_ptr) (plugin);
		} else
			dev_dbg(msm_chg.dev, "%s unable to notify unplugin\n",
				__func__);
		usb_notified_of_insertion = 0;
		usb_hw_chg_priv = NULL;
	}

#ifdef CONFIG_SKY_CHARGING
        if(hold_mA && plugin)
        {
                msm_charge_called_before_init(hold_mA);
                hold_mA = 0;
        }
#endif    
}

static unsigned int msm_chg_get_batt_capacity_percent(void)
{
	unsigned int current_voltage = get_prop_battery_mvolts();
	unsigned int low_voltage = msm_chg.min_voltage;
	unsigned int high_voltage = msm_chg.max_voltage;

	if (current_voltage <= low_voltage)
		return 0;
	else if (current_voltage >= high_voltage)
		return 100;
	else
		return (current_voltage - low_voltage) * 100
		    / (high_voltage - low_voltage);
}

#ifdef DEBUG
static inline void debug_print(const char *func,
			       struct msm_hardware_charger_priv *hw_chg_priv)
{
	dev_dbg(msm_chg.dev,
		"%s current=(%s)(s=%d)(r=%d) new=(%s)(s=%d)(r=%d) batt=%d En\n",
		func,
		msm_chg.current_chg_priv ? msm_chg.current_chg_priv->
		hw_chg->name : "none",
		msm_chg.current_chg_priv ? msm_chg.
		current_chg_priv->hw_chg_state : -1,
		msm_chg.current_chg_priv ? msm_chg.current_chg_priv->
		hw_chg->rating : -1,
		hw_chg_priv ? hw_chg_priv->hw_chg->name : "none",
		hw_chg_priv ? hw_chg_priv->hw_chg_state : -1,
		hw_chg_priv ? hw_chg_priv->hw_chg->rating : -1,
		msm_chg.batt_status);
}
#else
static inline void debug_print(const char *func,
			       struct msm_hardware_charger_priv *hw_chg_priv)
{
}
#endif

static struct msm_hardware_charger_priv *find_best_charger(void)
{
	struct msm_hardware_charger_priv *hw_chg_priv;
	struct msm_hardware_charger_priv *better;
	int rating;

	better = NULL;
	rating = 0;

	list_for_each_entry(hw_chg_priv, &msm_chg.msm_hardware_chargers, list) {
		if (is_chg_capable_of_charging(hw_chg_priv)) {
			if (hw_chg_priv->hw_chg->rating > rating) {
				rating = hw_chg_priv->hw_chg->rating;
				better = hw_chg_priv;
			}
		}
	}

	return better;
}

static int msm_charging_switched(struct msm_hardware_charger_priv *priv)
{
	int ret = 0;

	if (priv->hw_chg->charging_switched)
		ret = priv->hw_chg->charging_switched(priv->hw_chg);
	return ret;
}

static int msm_stop_charging(struct msm_hardware_charger_priv *priv)
{
	int ret;

	ret = priv->hw_chg->stop_charging(priv->hw_chg);

	dev_info(msm_chg.dev, "%s ## STOP CHARGING ## : ret [%d]", __func__, ret);

#if !defined(CONFIG_SKY_SMB136S_CHARGER)
	if (!ret)
		wake_unlock(&msm_chg.wl);
#endif

	return ret;
}

static void msm_enable_system_current(struct msm_hardware_charger_priv *priv)
{
	if (priv->hw_chg->start_system_current)
#if defined(CONFIG_SKY_CHARGING) || defined(CONFIG_SKY_SMB136S_CHARGER) || defined(CONFIG_SKY_SMB137B_CHARGER)
	{
		dev_info(msm_chg.dev, "%s: battery mvolts=%dmV\n", __func__, get_prop_battery_mvolts());

		if (msm_chg.batt_status == BATT_STATUS_ABSENT && msm_chg.is_factory_cable) // no battery factory cable state
//		if (!is_battery_present() && is_factory_cable())
		{
		priv->hw_chg->start_system_current(priv->hw_chg,
						 1500);
		}
		else if (msm_chg.is_verylow_battery) // very low battery state
//		else if (get_prop_battery_mvolts() < 3000000)
		{
			priv->hw_chg->start_system_current(priv->hw_chg,
						 1500);
		}else{
			priv->hw_chg->start_system_current(priv->hw_chg,
					 priv->max_source_current);
}
	}
#else
		priv->hw_chg->start_system_current(priv->hw_chg,
					 priv->max_source_current);
#endif
}

static void msm_disable_system_current(struct msm_hardware_charger_priv *priv)
{
	if (priv->hw_chg->stop_system_current)
		priv->hw_chg->stop_system_current(priv->hw_chg);
}

/* the best charger has been selected -start charging from current_chg_priv */
static int msm_start_charging(void)
{
	int ret = 0;
	struct msm_hardware_charger_priv *priv;

	priv = msm_chg.current_chg_priv;

#if !defined(CONFIG_SKY_SMB136S_CHARGER)
	wake_lock(&msm_chg.wl);
#endif

	ret = priv->hw_chg->start_charging(priv->hw_chg, msm_chg.max_voltage,
					 priv->max_source_current);

	dev_info(msm_chg.dev, "%s ## START CHARGING ## : ret [%d]\n", __func__, ret);

	if (ret) {
#if !defined(CONFIG_SKY_SMB136S_CHARGER)
		wake_unlock(&msm_chg.wl);
#endif
		dev_err(msm_chg.dev, "%s couldnt start chg error = %d\n",
			priv->hw_chg->name, ret);
	} else {
		priv->hw_chg_state = CHG_CHARGING_STATE;
#if defined(CONFIG_SKY_SMB136S_CHARGER) || defined(CONFIG_SKY_SMB137B_CHARGER)
		if(priv->max_source_current == 500)
		{
			priv->psy.type = POWER_SUPPLY_TYPE_USB;
		}
		else
		{
			priv->psy.type = POWER_SUPPLY_TYPE_MAINS;
		}
#endif
	}
	return ret;
}

static void handle_charging_done(struct msm_hardware_charger_priv *priv)
{
	if (msm_chg.current_chg_priv == priv) {
		if (msm_chg.current_chg_priv->hw_chg_state ==
		    CHG_CHARGING_STATE)
			if (msm_stop_charging(msm_chg.current_chg_priv)) {
				dev_err(msm_chg.dev, "%s couldnt stop chg\n",
					msm_chg.current_chg_priv->hw_chg->name);
			}
		msm_chg.current_chg_priv->hw_chg_state = CHG_READY_STATE;

		msm_chg.batt_status = BATT_STATUS_JUST_FINISHED_CHARGING;
		dev_info(msm_chg.dev, "%s: stopping safety timer work\n",
				__func__);
#if 0 // P11220 20130910 : disable safety timer
		cancel_delayed_work(&msm_chg.teoc_work);
#endif
		if (msm_batt_gauge && msm_batt_gauge->monitor_for_recharging)
			msm_batt_gauge->monitor_for_recharging();
		else
			dev_err(msm_chg.dev,
			      "%s: no batt gauge recharge monitor\n", __func__);
	}
}

#if 0 // P11220 20130910 : disable safety timer
static void teoc(struct work_struct *work)
{
	/* we have been charging too long - stop charging */
	dev_info(msm_chg.dev, "%s: safety timer work expired\n", __func__);

	mutex_lock(&msm_chg.status_lock);
	if (msm_chg.current_chg_priv != NULL
	    && msm_chg.current_chg_priv->hw_chg_state == CHG_CHARGING_STATE) {
		handle_charging_done(msm_chg.current_chg_priv);
	}
	mutex_unlock(&msm_chg.status_lock);
}
#endif

static void handle_battery_inserted(void)
{
	/* if a charger is already present start charging */
	if (msm_chg.current_chg_priv != NULL &&
	    is_batt_status_capable_of_charging() &&
	    !is_batt_status_charging()) {
		if (msm_start_charging()) {
			dev_err(msm_chg.dev, "%s couldnt start chg\n",
				msm_chg.current_chg_priv->hw_chg->name);
			return;
		}
		msm_chg.batt_status = BATT_STATUS_TRKL_CHARGING;

		dev_info(msm_chg.dev, "%s: starting safety timer work\n",
				__func__);
#if 0 // P11220 20130910 : disable safety timer
		queue_delayed_work(msm_chg.event_wq_thread,
					&msm_chg.teoc_work,
				      round_jiffies_relative(msecs_to_jiffies
							     (msm_chg.
							      safety_time)));
#endif
	}
#ifdef CONFIG_SKY_CHARGING
	if (msm_chg.current_chg_priv == NULL) {
                msm_chg.batt_status = BATT_STATUS_DISCHARGING;
    }
#endif
#if defined(CONFIG_SKY_CHARGING) || defined(CONFIG_SKY_SMB136S_CHARGER) || defined(CONFIG_SKY_SMB137B_CHARGER)
	if (is_battery_present())
		msm_chg.batt_id = get_battery_id();
#endif
}

static void handle_battery_removed(void)
{
	/* if a charger is charging the battery stop it */
	if (msm_chg.current_chg_priv != NULL
	    && msm_chg.current_chg_priv->hw_chg_state == CHG_CHARGING_STATE) {
		if (msm_stop_charging(msm_chg.current_chg_priv)) {
			dev_err(msm_chg.dev, "%s couldnt stop chg\n",
				msm_chg.current_chg_priv->hw_chg->name);
		}
		msm_chg.current_chg_priv->hw_chg_state = CHG_READY_STATE;

		dev_info(msm_chg.dev, "%s: stopping safety timer work\n",
				__func__);
#if 0 // P11220 20130910 : disable safety timer
		cancel_delayed_work(&msm_chg.teoc_work);
#endif
	}
#if defined(CONFIG_SKY_CHARGING) || defined(CONFIG_SKY_SMB136S_CHARGER) || defined(CONFIG_SKY_SMB137B_CHARGER)
	msm_chg.batt_id = 0;
#endif
}

static void update_heartbeat(struct work_struct *work)
{
#ifndef CONFIG_SKY_CHARGING
	int temperature;
#endif

#if defined(PANTECH_CHARGER_TEST_MENU)
  int enable;
#endif

	if (msm_chg.batt_status == BATT_STATUS_ABSENT
		|| msm_chg.batt_status == BATT_STATUS_ID_INVALID) {
		if (is_battery_present())
#ifndef CONFIG_SKY_CHARGING
			if (is_battery_id_valid()) 
#endif
			{
				msm_chg.batt_status = BATT_STATUS_DISCHARGING;
				handle_battery_inserted();
			}
	} else {
		if (!is_battery_present()) {
			msm_chg.batt_status = BATT_STATUS_ABSENT;
			handle_battery_removed();
		}
		/*
		 * check battery id because a good battery could be removed
		 * and replaced with a invalid battery.
		 */
		if (!is_battery_id_valid()) {
			msm_chg.batt_status = BATT_STATUS_ID_INVALID;
			handle_battery_removed();
		}
	}
	pr_debug("msm-charger %s batt_status= %d\n",
				__func__, msm_chg.batt_status);

	if (msm_chg.current_chg_priv
		&& msm_chg.current_chg_priv->hw_chg_state
			== CHG_CHARGING_STATE) {
#ifndef CONFIG_SKY_CHARGING
		temperature = get_battery_temperature();
#endif
		/* TODO implement JEITA SPEC*/
	}

#if defined(PANTECH_CHARGER_TEST_MENU)
	enable = atomic_read(&bms_input_flag);
	if(enable)
	{
		input_report_rel(bms_input_dev, REL_RX, get_prop_batt_capacity());
		input_report_rel(bms_input_dev, REL_RY, get_prop_battery_mvolts());                
		input_report_rel(bms_input_dev, REL_RZ, 0);	// not avilavle in this chip
		input_report_rel(bms_input_dev, REL_X, 0);	// not avilavle in this chip
		input_sync(bms_input_dev);
	}
#endif     

	/* notify that the voltage has changed
	 * the read of the capacity will trigger a
	 * voltage read*/
	power_supply_changed(&msm_psy_batt);

	if (msm_chg.stop_update) {
		msm_chg.stop_update = 0;
		return;
	}
	queue_delayed_work(msm_chg.event_wq_thread,
				&msm_chg.update_heartbeat_work,
			      round_jiffies_relative(msecs_to_jiffies
						     (msm_chg.update_time)));
}

/* set the charger state to READY before calling this */
static void handle_charger_ready(struct msm_hardware_charger_priv *hw_chg_priv)
{
	struct msm_hardware_charger_priv *old_chg_priv = NULL;

	debug_print(__func__, hw_chg_priv);

	if (msm_chg.current_chg_priv != NULL
	    && hw_chg_priv->hw_chg->rating >
	    msm_chg.current_chg_priv->hw_chg->rating) {
		/*
		 * a better charger was found, ask the current charger
		 * to stop charging if it was charging
		 */
		if (msm_chg.current_chg_priv->hw_chg_state ==
		    CHG_CHARGING_STATE) {
			if (msm_stop_charging(msm_chg.current_chg_priv)) {
				dev_err(msm_chg.dev, "%s couldnt stop chg\n",
					msm_chg.current_chg_priv->hw_chg->name);
				return;
			}
			if (msm_charging_switched(msm_chg.current_chg_priv)) {
				dev_err(msm_chg.dev, "%s couldnt stop chg\n",
					msm_chg.current_chg_priv->hw_chg->name);
				return;
			}
		}
		msm_chg.current_chg_priv->hw_chg_state = CHG_READY_STATE;
		
		old_chg_priv = msm_chg.current_chg_priv;
		msm_chg.current_chg_priv = NULL;
	}

	if (msm_chg.current_chg_priv == NULL) {
		msm_chg.current_chg_priv = hw_chg_priv;
		dev_info(msm_chg.dev,
			 "%s: best charger = %s\n", __func__,
			 msm_chg.current_chg_priv->hw_chg->name);

		msm_enable_system_current(msm_chg.current_chg_priv);
		/*
		 * since a better charger was chosen, ask the old
		 * charger to stop providing system current
		 */
		if (old_chg_priv != NULL)
			msm_disable_system_current(old_chg_priv);

		if (!is_batt_status_capable_of_charging())
			return;

		/* start charging from the new charger */
		if (!msm_start_charging()) {
			/* if we simply switched chg continue with teoc timer
			 * else we update the batt state and set the teoc
			 * timer */
			if (!is_batt_status_charging()) {
				dev_info(msm_chg.dev,
				       "%s: starting safety timer\n", __func__);
#if 0 // P11220 20130910 : disable safety timer
				queue_delayed_work(msm_chg.event_wq_thread,
							&msm_chg.teoc_work,
						      round_jiffies_relative
						      (msecs_to_jiffies
						       (msm_chg.safety_time)));
#endif
				msm_chg.batt_status = BATT_STATUS_TRKL_CHARGING;
			}
		} else {
			/* we couldnt start charging from the new readied
			 * charger */
			if (is_batt_status_charging())
				msm_chg.batt_status = BATT_STATUS_DISCHARGING;
		}
	}
}

static void handle_charger_removed(struct msm_hardware_charger_priv
				   *hw_chg_removed, int new_state)
{
	struct msm_hardware_charger_priv *hw_chg_priv;

	debug_print(__func__, hw_chg_removed);

	if (msm_chg.current_chg_priv == hw_chg_removed) {
		msm_disable_system_current(hw_chg_removed);
		if (msm_chg.current_chg_priv->hw_chg_state
						== CHG_CHARGING_STATE) {
			if (msm_stop_charging(hw_chg_removed)) {
				dev_err(msm_chg.dev, "%s couldnt stop chg\n",
					msm_chg.current_chg_priv->hw_chg->name);
			}
		}
		msm_chg.current_chg_priv = NULL;
	}

	hw_chg_removed->hw_chg_state = new_state;

	if (msm_chg.current_chg_priv == NULL) {
		hw_chg_priv = find_best_charger();
		if (hw_chg_priv == NULL) {
			dev_info(msm_chg.dev, "%s: no chargers\n", __func__);
			/* if the battery was Just finished charging
			 * we keep that state as is so that we dont rush
			 * in to charging the battery when a charger is
			 * plugged in shortly. */
			if (is_batt_status_charging())
				msm_chg.batt_status = BATT_STATUS_DISCHARGING;
		} else {
			msm_chg.current_chg_priv = hw_chg_priv;
			msm_enable_system_current(hw_chg_priv);
			dev_info(msm_chg.dev,
				 "%s: best charger = %s\n", __func__,
				 msm_chg.current_chg_priv->hw_chg->name);

			if (!is_batt_status_capable_of_charging())
				return;

			if (msm_start_charging()) {
				/* we couldnt start charging for some reason */
				msm_chg.batt_status = BATT_STATUS_DISCHARGING;
			}
		}
	}

	/* if we arent charging stop the safety timer */
	if (!is_batt_status_charging()) {
		dev_info(msm_chg.dev, "%s: stopping safety timer work\n",
				__func__);
#if 0 // P11220 20130910 : disable safety timer
		cancel_delayed_work(&msm_chg.teoc_work);
#endif
	}
}

static void handle_event(struct msm_hardware_charger *hw_chg, int event)
{
	struct msm_hardware_charger_priv *priv = NULL;

	/*
	 * if hw_chg is NULL then this event comes from non-charger
	 * parties like battery gauge
	 */
	if (hw_chg)
		priv = hw_chg->charger_private;

	mutex_lock(&msm_chg.status_lock);

	dev_info(msm_chg.dev, "%s  : event [%d]", __func__, event);

	switch (event) {
	case CHG_INSERTED_EVENT:
		if (priv->hw_chg_state != CHG_ABSENT_STATE) {
			dev_info(msm_chg.dev,
				 "%s insertion detected when cbl present",
				 hw_chg->name);
			break;
		}

#if defined(CONFIG_SKY_SMB136S_CHARGER)
		wake_lock(&msm_chg.wl);
#endif

		update_batt_status();
		update_cable_status(1);
		if (hw_chg->type == CHG_TYPE_USB) {
#ifdef CONFIG_SKY_CHARGING /* default setting */
                msm_set_charger_type(CHG_TYPE_USB);
#endif
			priv->hw_chg_state = CHG_PRESENT_STATE;
			notify_usb_of_the_plugin_event(priv, 1);
			if (usb_chg_current) {
				priv->max_source_current = usb_chg_current;
				usb_chg_current = 0;
				/* usb has already indicated us to charge */
				priv->hw_chg_state = CHG_READY_STATE;
				handle_charger_ready(priv);
			}
		} else {
			priv->hw_chg_state = CHG_READY_STATE;
			handle_charger_ready(priv);
		}
		break;
	case CHG_ENUMERATED_EVENT:	/* only in USB types */
		if (priv->hw_chg_state == CHG_ABSENT_STATE) {
			dev_info(msm_chg.dev, "%s enum withuot presence\n",
				 hw_chg->name);
			break;
		}

#ifdef F_MHL_AUTO_CONNECT
		if(is_mhl_cable())
		{
    #ifdef CONFIG_PANTECH_LCD_MHL_CABLE_DETECT
   		    is_mhl = 1;
    #endif
            MHL_Set_cable_detect_handler();
			printk(KERN_ERR "[SKY_MHL]%s MHL cable Connect \n",__func__);			
		}
    #ifdef CONFIG_PANTECH_LCD_MHL_CABLE_DETECT
		else
		{
		    is_mhl = 0;
		}
    #endif
#endif

#ifdef CONFIG_SKY_CHARGING
        if(msm_chg.is_factory_cable)
            msm_set_charger_type(CHG_TYPE_FACTORY);
#endif        
		update_batt_status();
		dev_dbg(msm_chg.dev, "%s enum with %dmA to draw\n",
			 hw_chg->name, priv->max_source_current);
		if (priv->max_source_current == 0) {
			/* usb subsystem doesnt want us to draw
			 * charging current */
			/* act as if the charge is removed */
			if (priv->hw_chg_state != CHG_PRESENT_STATE)
				handle_charger_removed(priv, CHG_PRESENT_STATE);
		} else {
			if (priv->hw_chg_state != CHG_READY_STATE) {
				priv->hw_chg_state = CHG_READY_STATE;
				handle_charger_ready(priv);
			}
		}
		break;
	case CHG_REMOVED_EVENT:
		if (priv->hw_chg_state == CHG_ABSENT_STATE) {
			dev_info(msm_chg.dev, "%s cable already removed\n",
				 hw_chg->name);
			break;
		}
		update_batt_status();
		update_cable_status(0);
		if (hw_chg->type == CHG_TYPE_USB) {
			usb_chg_current = 0;
			notify_usb_of_the_plugin_event(priv, 0);
		}
		handle_charger_removed(priv, CHG_ABSENT_STATE);
#ifdef CONFIG_SKY_CHARGING /* default setting */
             msm_set_charger_type(CHG_TYPE_NONE);
#endif

#if defined(CONFIG_SKY_SMB136S_CHARGER)
		wake_unlock(&msm_chg.wl);
#endif
		break;
	case CHG_DONE_EVENT:
		if (priv->hw_chg_state == CHG_CHARGING_STATE)
			handle_charging_done(priv);
		break;
	case CHG_BATT_BEGIN_FAST_CHARGING:
		/* only update if we are TRKL charging */
		if (msm_chg.batt_status == BATT_STATUS_TRKL_CHARGING)
			msm_chg.batt_status = BATT_STATUS_FAST_CHARGING;
		break;
	case CHG_BATT_NEEDS_RECHARGING:
		msm_chg.batt_status = BATT_STATUS_DISCHARGING;
		handle_battery_inserted();
		priv = msm_chg.current_chg_priv;
		break;
	case CHG_BATT_TEMP_OUTOFRANGE:
		/* the batt_temp out of range can trigger
		 * when the battery is absent */
		if (!is_battery_present()
		    && msm_chg.batt_status != BATT_STATUS_ABSENT) {
			msm_chg.batt_status = BATT_STATUS_ABSENT;
			handle_battery_removed();
			break;
		}
		if (msm_chg.batt_status == BATT_STATUS_TEMPERATURE_OUT_OF_RANGE)
			break;
		msm_chg.batt_status = BATT_STATUS_TEMPERATURE_OUT_OF_RANGE;
		handle_battery_removed();
		break;
	case CHG_BATT_TEMP_INRANGE:
		if (msm_chg.batt_status != BATT_STATUS_TEMPERATURE_OUT_OF_RANGE)
			break;
		msm_chg.batt_status = BATT_STATUS_ID_INVALID;
		/* check id */
		if (!is_battery_id_valid())
			break;
		/* assume that we are discharging from the battery
		 * and act as if the battery was inserted
		 * if a charger is present charging will be resumed */
		msm_chg.batt_status = BATT_STATUS_DISCHARGING;
		handle_battery_inserted();
		break;
	case CHG_BATT_INSERTED:
		if (msm_chg.batt_status != BATT_STATUS_ABSENT)
			break;
		/* debounce */
		if (!is_battery_present())
			break;
		msm_chg.batt_status = BATT_STATUS_ID_INVALID;
		if (!is_battery_id_valid())
			break;
		/* assume that we are discharging from the battery */
		msm_chg.batt_status = BATT_STATUS_DISCHARGING;
		/* check if a charger is present */
		handle_battery_inserted();
		break;
	case CHG_BATT_REMOVED:
		if (msm_chg.batt_status == BATT_STATUS_ABSENT)
			break;
		/* debounce */
		if (is_battery_present())
			break;
#ifdef CONFIG_SKY_CHARGING
		if (is_factory_cable() ) 
			break;
		
		if (msm_chg.batt_status == BATT_STATUS_DISCHARGING)
			break;
                mdelay(10);
		if (is_battery_present())
			break;
                mdelay(10);
		if (is_battery_present())
			break;
        	machine_power_off();
#endif
		msm_chg.batt_status = BATT_STATUS_ABSENT;
		handle_battery_removed();
#if defined(CONFIG_SKY_SMB136S_CHARGER) || defined(CONFIG_SKY_SMB137B_CHARGER)	
		if (msm_chg.current_chg_priv != NULL
	    		&& msm_chg.current_chg_priv->hw_chg_state != CHG_ABSENT_STATE) {
			if(is_factory_cable() != 1) {
				if(!is_battery_present()) {
#if defined(CONFIG_MACH_MSM8X60_EF65L)
					if(msm_chg.current_chg_priv->hw_chg->get_chg_hw_state 
						&& (msm_chg.current_chg_priv->hw_chg->get_chg_hw_state() == 1)) {
						pr_err("Battery removed, kernel power off ...\n");
						//orderly_poweroff(true);
//						arch_reset(0, "oem-34");
						msm_restart(0, "oem-34");
					}
#else
					pr_err("Battery removed, kernel power off ...\n");
					//orderly_poweroff(true);
//					arch_reset(0, "oem-34");
					msm_restart(0, "oem-34");
#endif
				}
			}
		}
#endif		
		break;
	case CHG_BATT_STATUS_CHANGE:
		/* TODO  battery SOC like battery-alarm/charging-full features
		can be added here for future improvement */
		break;
	}
	dev_dbg(msm_chg.dev, "%s %d done batt_status=%d\n", __func__,
		event, msm_chg.batt_status);

	/* update userspace */
	if (msm_batt_gauge)
		power_supply_changed(&msm_psy_batt);
	if (priv) {
#ifdef CONFIG_SKY_CHARGING
		dev_info(msm_chg.dev, "%s  : old_charger_type = %d, charger_type=%d\n", __func__, msm_get_prev_charger_type(), msm_get_charger_type());

		// change old
		if (msm_get_prev_charger_type() != msm_get_charger_type())
		{
			if (msm_get_charger_type() == CHG_TYPE_NONE) //removed
			{
				//old
				if (msm_get_prev_charger_type() == CHG_TYPE_AC)
					power_supply_changed(&priv->psy_ac);
				else
					power_supply_changed(&priv->psy);
			}
			else if (msm_get_prev_charger_type() == CHG_TYPE_NONE) //inserted
			{
				if (msm_get_charger_type() == CHG_TYPE_AC)
					power_supply_changed(&priv->psy_ac);
				else
					power_supply_changed(&priv->psy);
			}
			else
			{  // changed
				//old
				if (msm_get_prev_charger_type() == CHG_TYPE_AC)
					power_supply_changed(&priv->psy_ac);
				else
					power_supply_changed(&priv->psy);

				// new
				if (msm_get_charger_type() == CHG_TYPE_AC)
					power_supply_changed(&priv->psy_ac);
				else
					power_supply_changed(&priv->psy);
			}
		}
		else
		{
			// new
			if (msm_get_charger_type() == CHG_TYPE_AC)
				power_supply_changed(&priv->psy_ac);
			else
				power_supply_changed(&priv->psy);
		}
#else			
		power_supply_changed(&priv->psy);
		power_supply_changed(&priv->psy_ac);
#endif
	}
	mutex_unlock(&msm_chg.status_lock);
}

static int msm_chg_dequeue_event(struct msm_charger_event **event)
{
	unsigned long flags;

	spin_lock_irqsave(&msm_chg.queue_lock, flags);
	if (msm_chg.queue_count == 0) {
		spin_unlock_irqrestore(&msm_chg.queue_lock, flags);
		return -EINVAL;
	}
	*event = &msm_chg.queue[msm_chg.head];
	msm_chg.head = (msm_chg.head + 1) % MSM_CHG_MAX_EVENTS;
	pr_debug("%s dequeueing %d\n", __func__, (*event)->event);
	msm_chg.queue_count--;
	spin_unlock_irqrestore(&msm_chg.queue_lock, flags);
	return 0;
}

static int msm_chg_enqueue_event(struct msm_hardware_charger *hw_chg,
			enum msm_hardware_charger_event event)
{
	unsigned long flags;

	spin_lock_irqsave(&msm_chg.queue_lock, flags);
	if (msm_chg.queue_count == MSM_CHG_MAX_EVENTS) {
		spin_unlock_irqrestore(&msm_chg.queue_lock, flags);
		pr_err("%s: queue full cannot enqueue %d\n",
				__func__, event);
		return -EAGAIN;
	}
	pr_debug("%s queueing %d\n", __func__, event);
	msm_chg.queue[msm_chg.tail].event = event;
	msm_chg.queue[msm_chg.tail].hw_chg = hw_chg;
	msm_chg.tail = (msm_chg.tail + 1)%MSM_CHG_MAX_EVENTS;
	msm_chg.queue_count++;
	spin_unlock_irqrestore(&msm_chg.queue_lock, flags);
	return 0;
}

static void process_events(struct work_struct *work)
{
	struct msm_charger_event *event;
	int rc;

	do {
		rc = msm_chg_dequeue_event(&event);
		if (!rc)
			handle_event(event->hw_chg, event->event);
	} while (!rc);
}

/* USB calls these to tell us how much charging current we should draw */
void msm_charger_vbus_draw(unsigned int mA)
{
	if (usb_hw_chg_priv) {
#ifdef CONFIG_SKY_CHARGING
        if(mA == 500) // usb
            msm_set_charger_type(CHG_TYPE_USB);
        else if(mA == 900) // ac
            msm_set_charger_type(CHG_TYPE_AC);
        else if(mA > 500 && mA < 900) // unknown
        {
            mA = 500; 
            msm_set_charger_type(CHG_TYPE_USB);
        }
        else if(mA >= 900)  // unknown
        {
            mA = 900;  //
            msm_set_charger_type(CHG_TYPE_AC);
        }
        else
                return;
		
	if (msm_chg.is_verylow_battery) // very low battery state
	{
            mA = 1500;  //
	}		
#endif                    
		usb_hw_chg_priv->max_source_current = mA;
		msm_charger_notify_event(usb_hw_chg_priv->hw_chg,
						CHG_ENUMERATED_EVENT);
	} else
		/* remember the current, to be used when charger is ready */
//		pr_err("%s called early;charger isnt initialized\n", __func__);
		usb_chg_current = mA;

#ifdef CONFIG_SKY_CHARGING
    if(!usb_hw_chg_priv && mA)
    {
        hold_mA = mA;
    }
	pr_info("[SKY CHG]%s:mA=%d, max_source_current=%d, charger_type=%d\n", __func__,mA,  usb_hw_chg_priv->max_source_current, msm_get_charger_type());
#endif

}

static int determine_initial_batt_status(void)
{
#ifdef CONFIG_SKY_CHARGING
	if (is_battery_present())
			msm_chg.batt_status = BATT_STATUS_DISCHARGING;
#else
	if (is_battery_present())
		if (is_battery_id_valid())
			if (is_battery_temp_within_range())
				msm_chg.batt_status = BATT_STATUS_DISCHARGING;
			else
				msm_chg.batt_status
				    = BATT_STATUS_TEMPERATURE_OUT_OF_RANGE;
		else
			msm_chg.batt_status = BATT_STATUS_ID_INVALID;
#endif
	else
		msm_chg.batt_status = BATT_STATUS_ABSENT;

#if defined(CONFIG_SKY_CHARGING) || defined(CONFIG_SKY_SMB136S_CHARGER) || defined(CONFIG_SKY_SMB137B_CHARGER)
	if (is_battery_present())
		msm_chg.batt_id = get_battery_id();
#endif

	if (is_batt_status_capable_of_charging())
		handle_battery_inserted();

	/* start updaing the battery powersupply every msm_chg.update_time
	 * milliseconds */
	queue_delayed_work(msm_chg.event_wq_thread,
				&msm_chg.update_heartbeat_work,
			      round_jiffies_relative(msecs_to_jiffies
						     (msm_chg.update_time)));

	pr_debug("%s:OK batt_status=%d\n", __func__, msm_chg.batt_status);
	return 0;
}

#if defined(PANTECH_CHARGER_TEST_MENU)
#define UART_DEBUG 0
#define DISCHARGING_TEST_RESUME_TIME 	300	// 5 min

enum pantech_cable_type {
	PANTECH_CABLE_NONE=0,
	PANTECH_OTG,
	PANTECH_USB,
	PANTECH_AC,
	PANTECH_FACTORY,
	PANTECH_AUDIO_DOCKING_STATION,
	PANTECH_CABLE_MAX,
};

static void create_testmenu_entries(struct msm_charger_mux *msm_chg)
{
  struct proc_dir_entry *msm_charger_dir;
	msm_charger_dir = proc_mkdir("msm_charger_dir", NULL);

	if (msm_charger_dir == NULL) {
		pr_err("Unable to create /proc/%s directory\n", "msm_charger_dir");
	}
}

static void set_alarm_for_battery_discharging_test(struct msm_charger_mux *msm_chg, int enable)
{
	int test_enable;
	unsigned long now_tm_sec = 0;
	struct rtc_time tm;
#if UART_DEBUG	
	struct timespec ts;
#endif

	test_enable = atomic_read(&bms_input_flag);
	if(test_enable == 0 || msm_chg->rtc == NULL) 
		return;

	if(enable) {
		rtc_read_time(msm_chg->rtc, &tm);
		rtc_valid_tm(&tm);
		rtc_tm_to_time(&tm, &now_tm_sec);
		
		memset(&msm_chg->alm, 0, sizeof(msm_chg->alm));
		
		rtc_time_to_tm(now_tm_sec + DISCHARGING_TEST_RESUME_TIME, &msm_chg->alm.time);
		
		msm_chg->alm.enabled = true;
		rtc_set_alarm(msm_chg->rtc, &msm_chg->alm);
#if UART_DEBUG
		getnstimeofday(&ts);
		rtc_time_to_tm(ts.tv_sec + DISCHARGING_TEST_RESUME_TIME, &tm);
		
		printk("[Battery Discharging Test] Device will be resumed at %02d:%02d:%02d\n", 
			tm.tm_hour, tm.tm_min, tm.tm_sec);
#endif		
	}
	else {
		memset(&msm_chg->alm, 0, sizeof(msm_chg->alm));
		
		msm_chg->alm.enabled = false;
		rtc_set_alarm(msm_chg->rtc, &msm_chg->alm);
	}
}

static int msm_charger_test_misc_open(struct inode *inode, struct file *file)
{
		return 0;
}

static long msm_charger_test_misc_ioctl(struct file *file,
		    unsigned int cmd, unsigned long arg)
{
	int rc;
	//u8 rdData = 0;
	int mval[2];
	int cval[4];
	int enb=0;
	int chg_type = 0;

	switch (cmd) {
		case CHARGER_MONITOR_TEST_SET_MSM:
#ifdef CONFIG_SKY_CHARGING			
			chg_type = msm_get_charger_type();
#elif defined(CONFIG_SKY_SMB136S_CHARGER) || defined(CONFIG_SKY_SMB137B_CHARGER)
			chg_type = atomic_read(&smb_charger_state);
#else
			chg_type = CHG_TYPE_NONE;
#endif
 			if (chg_type == CHG_TYPE_USB)
				mval[0] = PANTECH_USB;
 			else if (chg_type == CHG_TYPE_AC)
				mval[0] = PANTECH_AC;
 			else if (chg_type == CHG_TYPE_FACTORY)
				mval[0] = PANTECH_FACTORY;
			else
				mval[0] = PANTECH_CABLE_NONE;

			rc = copy_to_user((void *)arg, mval, sizeof(mval));
			break;

 		case CHARGER_CHARGING_TEST_SET_MSM:
#ifdef CONFIG_SKY_CHARGING			
			chg_type = msm_get_charger_type();
#elif defined(CONFIG_SKY_SMB136S_CHARGER) || defined(CONFIG_SKY_SMB137B_CHARGER)
			chg_type = atomic_read(&smb_charger_state);
#else
			chg_type = CHG_TYPE_NONE;
#endif
 			if (chg_type == CHG_TYPE_USB)
				cval[0] = PANTECH_USB;
 			else if (chg_type == CHG_TYPE_AC)
				cval[0] = PANTECH_AC;
 			else if (chg_type == CHG_TYPE_FACTORY)
				cval[0] = PANTECH_FACTORY;
			else
				cval[0] = PANTECH_CABLE_NONE;
			
			// read pantech cable type, adc, batttery ID
			cval[1] = msm_chg.cable_id;
			cval[2] = msm_chg.batt_id; 			

//			printk("[hglim]batt_id:%d,cable_id:%d,cval=%d,%d,%d\n", msm_chg.batt_id, msm_chg.cable_id, cval[0], cval[1], cval[2]);

			rc = copy_to_user((void *)arg, cval, sizeof(cval));
			break;		

		case CHARGER_DISCHARGING_TEST_SET_MSM:
			
			// write input flag
			rc = copy_from_user((void *)&enb, (void *)arg, sizeof(int));
			atomic_set(&bms_input_flag, enb);

		break;

		case CHARGER_DISCHARGING_TEST_GET_MSM:
			
			// read input flag
			enb = atomic_read(&bms_input_flag);
			rc = copy_to_user((void *)arg, &enb, sizeof(int));
			
			break;
			
	default:
		rc = -EINVAL;
	}
	return rc;
}

static int msm_charger_battery_test_misc_release(struct inode *inode, struct file *file)
{
		return 0;
}
	
static struct file_operations msm_charger_test_dev_fops = {
	.owner = THIS_MODULE,
	.open = msm_charger_test_misc_open,
	.unlocked_ioctl	= msm_charger_test_misc_ioctl,
	.release	= msm_charger_battery_test_misc_release,
};
	
struct miscdevice msm_charger_test_misc_device = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "msm_charger_fops",
	.fops	= &msm_charger_test_dev_fops,
};
	
int msm_charger_battery_charging_test_init(struct msm_charger_mux *msm_chg)
{	
	msm_chg->rtc = rtc_class_open(CONFIG_RTC_HCTOSYS_DEVICE);
	if (msm_chg->rtc == NULL) {
		pr_err("%s: unable to open rtc device (%s)\n",
			__FILE__, CONFIG_RTC_HCTOSYS_DEVICE);
	}
	return misc_register(&msm_charger_test_misc_device);
}
static int test_subdevices_register(void)
{
	int rc = 0;

#if defined(PANTECH_CHARGER_TEST_MENU)
	atomic_set(&bms_input_flag, 0);

	bms_input_dev = input_allocate_device();
	if (!bms_input_dev) {
	    	pr_err("BMS: Unable to input_allocate_device \n");
	    	rc = -ENXIO;
	    	return rc;
	}

	set_bit(EV_REL, bms_input_dev->evbit);
	input_set_capability(bms_input_dev, EV_REL, REL_RX);	// SOC
	input_set_capability(bms_input_dev, EV_REL, REL_RY); 	// Volt
	input_set_capability(bms_input_dev, EV_REL, REL_RZ);    // TEMP
	input_set_capability(bms_input_dev, EV_REL, REL_X);	// VCHG
	bms_input_dev->name="bms_app";
	rc =input_register_device(bms_input_dev);
	if (rc) {
	    	pr_err("BMS: Unable to register input_register_device device\n");
	    	return rc;
	}
#endif
	return rc;
}
#endif

static int __devinit msm_charger_probe(struct platform_device *pdev)
{
	msm_chg.dev = &pdev->dev;
	if (pdev->dev.platform_data) {
		unsigned int milli_secs;

		struct msm_charger_platform_data *pdata
		    =
		    (struct msm_charger_platform_data *)pdev->dev.platform_data;

		milli_secs = pdata->safety_time * 60 * MSEC_PER_SEC;
		if (milli_secs > jiffies_to_msecs(MAX_JIFFY_OFFSET)) {
			dev_warn(&pdev->dev, "%s: safety time too large"
				 "%dms\n", __func__, milli_secs);
			milli_secs = jiffies_to_msecs(MAX_JIFFY_OFFSET);
		}
		msm_chg.safety_time = milli_secs;

		milli_secs = pdata->update_time * 60 * MSEC_PER_SEC;
		if (milli_secs > jiffies_to_msecs(MAX_JIFFY_OFFSET)) {
			dev_warn(&pdev->dev, "%s: safety time too large"
				 "%dms\n", __func__, milli_secs);
			milli_secs = jiffies_to_msecs(MAX_JIFFY_OFFSET);
		}
		msm_chg.update_time = milli_secs;

		msm_chg.max_voltage = pdata->max_voltage;
		msm_chg.min_voltage = pdata->min_voltage;
		msm_chg.get_batt_capacity_percent =
		    pdata->get_batt_capacity_percent;
	}
	if (msm_chg.safety_time == 0)
		msm_chg.safety_time = CHARGING_TEOC_MS;
	if (msm_chg.update_time == 0)
		msm_chg.update_time = UPDATE_TIME_MS;
	if (msm_chg.max_voltage == 0)
		msm_chg.max_voltage = DEFAULT_BATT_MAX_V;
	if (msm_chg.min_voltage == 0)
		msm_chg.min_voltage = DEFAULT_BATT_MIN_V;
	if (msm_chg.get_batt_capacity_percent == NULL)
		msm_chg.get_batt_capacity_percent =
		    msm_chg_get_batt_capacity_percent;

#if defined(CONFIG_SKY_CHARGING) || defined(CONFIG_SKY_SMB136S_CHARGER) || defined(CONFIG_SKY_SMB137B_CHARGER)
	msm_chg.batt_id = 0; 
	msm_chg.cable_id = 0;
	msm_chg.is_factory_cable = 0;
	msm_chg.is_verylow_battery = 0;
#endif

	mutex_init(&msm_chg.status_lock);
#if 0 // P11220 20130910 : disable safety timer
	INIT_DELAYED_WORK(&msm_chg.teoc_work, teoc);
#endif
	INIT_DELAYED_WORK(&msm_chg.update_heartbeat_work, update_heartbeat);

#if defined(CONFIG_SKY_CHARGING) || defined(CONFIG_SKY_SMB136S_CHARGER) || defined(CONFIG_SKY_SMB137B_CHARGER)
       msm_chg.init_completed = 0;

	// init device attr
	init_sky_attr(&pdev->dev, msm_charger_sysfs_attrs, ARRAY_SIZE(msm_charger_sysfs_attrs));

	INIT_DELAYED_WORK(&msm_chg.init_completed_work, init_completed_work_func);

	schedule_delayed_work(&msm_chg.init_completed_work, msecs_to_jiffies(100));
#endif

#if defined(PANTECH_CHARGER_TEST_MENU)
	test_subdevices_register(); //for testcode #8378522
	create_testmenu_entries(&msm_chg);
	msm_charger_battery_charging_test_init(&msm_chg);
#endif	
		
	wake_lock_init(&msm_chg.wl, WAKE_LOCK_SUSPEND, "msm_charger");
	return 0;
}

static int __devexit msm_charger_remove(struct platform_device *pdev)
{

#if defined(PANTECH_CHARGER_TEST_MENU)
  input_unregister_device(bms_input_dev);
  input_free_device(bms_input_dev);
#endif

	wake_lock_destroy(&msm_chg.wl);
	mutex_destroy(&msm_chg.status_lock);
	power_supply_unregister(&msm_psy_batt);
	return 0;
}

int msm_charger_notify_event(struct msm_hardware_charger *hw_chg,
			     enum msm_hardware_charger_event event)
{
	msm_chg_enqueue_event(hw_chg, event);
	queue_work(msm_chg.event_wq_thread, &msm_chg.queue_work);
	return 0;
}
EXPORT_SYMBOL(msm_charger_notify_event);

int msm_charger_register(struct msm_hardware_charger *hw_chg)
{
	struct msm_hardware_charger_priv *priv;
	int rc = 0;

	if (!msm_chg.inited) {
		pr_err("%s: msm_chg is NULL,Too early to register\n", __func__);
		return -EAGAIN;
	}

	if (hw_chg->start_charging == NULL
		|| hw_chg->stop_charging == NULL
		|| hw_chg->name == NULL
		|| hw_chg->rating == 0) {
		pr_err("%s: invalid hw_chg\n", __func__);
		return -EINVAL;
	}

	priv = kzalloc(sizeof *priv, GFP_KERNEL);
	if (priv == NULL) {
		dev_err(msm_chg.dev, "%s kzalloc failed\n", __func__);
		return -ENOMEM;
	}

	priv->psy.name = hw_chg->name;
	if (hw_chg->type == CHG_TYPE_USB)
		priv->psy.type = POWER_SUPPLY_TYPE_USB;
	else
		priv->psy.type = POWER_SUPPLY_TYPE_MAINS;

	priv->psy.supplied_to = msm_power_supplied_to;
	priv->psy.num_supplicants = ARRAY_SIZE(msm_power_supplied_to);
	priv->psy.properties = msm_power_props;
	priv->psy.num_properties = ARRAY_SIZE(msm_power_props);
	priv->psy.get_property = msm_power_get_property;

	rc = power_supply_register(NULL, &priv->psy);
	if (rc) {
		dev_err(msm_chg.dev, "%s power_supply_register failed\n",
			__func__);
		goto out;
	}

#if defined(CONFIG_SKY_CHARGING) || defined(CONFIG_SKY_SMB136S_CHARGER) || defined(CONFIG_SKY_SMB137B_CHARGER)
	priv->psy_ac.name = "ac";
	priv->psy_ac.type = POWER_SUPPLY_TYPE_MAINS;

	priv->psy_ac.supplied_to = msm_power_supplied_to;
	priv->psy_ac.num_supplicants = ARRAY_SIZE(msm_power_supplied_to);
	priv->psy_ac.properties = msm_power_props;
	priv->psy_ac.num_properties = ARRAY_SIZE(msm_power_props);
	priv->psy_ac.get_property = msm_power_get_property;

	rc = power_supply_register(NULL, &priv->psy_ac);
	if (rc) {
		dev_err(msm_chg.dev, "%s power_supply_register failed\n",
			__func__);
		goto out;
	}
#endif

	priv->hw_chg = hw_chg;
	priv->hw_chg_state = CHG_ABSENT_STATE;
	INIT_LIST_HEAD(&priv->list);
	mutex_lock(&msm_chg.msm_hardware_chargers_lock);
	list_add_tail(&priv->list, &msm_chg.msm_hardware_chargers);
	mutex_unlock(&msm_chg.msm_hardware_chargers_lock);
	hw_chg->charger_private = (void *)priv;

	msm_chg.inited = 2;
	return 0;

out:
	kfree(priv);
	return rc;
}
EXPORT_SYMBOL(msm_charger_register);

void msm_battery_gauge_register(struct msm_battery_gauge *batt_gauge)
{
	int rc;

	if (msm_batt_gauge) {
		msm_batt_gauge = batt_gauge;
		pr_err("msm-charger %s multiple battery gauge called\n",
								__func__);
	} else {
		rc = power_supply_register(msm_chg.dev, &msm_psy_batt);
		if (rc < 0) {
			dev_err(msm_chg.dev, "%s: power_supply_register failed"
					" rc=%d\n", __func__, rc);
			return;
		}

		msm_batt_gauge = batt_gauge;
		determine_initial_batt_status();
	}
}
EXPORT_SYMBOL(msm_battery_gauge_register);

void msm_battery_gauge_unregister(struct msm_battery_gauge *batt_gauge)
{
	msm_batt_gauge = NULL;
}
EXPORT_SYMBOL(msm_battery_gauge_unregister);

int msm_charger_unregister(struct msm_hardware_charger *hw_chg)
{
	struct msm_hardware_charger_priv *priv;

	priv = (struct msm_hardware_charger_priv *)(hw_chg->charger_private);
	mutex_lock(&msm_chg.msm_hardware_chargers_lock);
	list_del(&priv->list);
	mutex_unlock(&msm_chg.msm_hardware_chargers_lock);
	power_supply_unregister(&priv->psy);
#if defined(CONFIG_SKY_CHARGING) || defined(CONFIG_SKY_SMB136S_CHARGER) || defined(CONFIG_SKY_SMB137B_CHARGER)
	power_supply_unregister(&priv->psy_ac);
#endif
	kfree(priv);
	return 0;
}
EXPORT_SYMBOL(msm_charger_unregister);

#if defined(CONFIG_SKY_CHARGING) || defined(CONFIG_SKY_SMB136S_CHARGER) || defined(CONFIG_SKY_SMB137B_CHARGER)
void msm_battery_func_register(struct msm_battery_func *batt_func)
{
	if (msm_batt_func) {
		msm_batt_func = batt_func;
		pr_err("msm-charger %s multiple battery func called\n", __func__);
	} else {
		msm_batt_func = batt_func;
	}
}
EXPORT_SYMBOL(msm_battery_func_register);
#endif

static int msm_charger_suspend(struct device *dev)
{
	dev_dbg(msm_chg.dev, "%s suspended\n", __func__);

#if defined(PANTECH_CHARGER_TEST_MENU)
	set_alarm_for_battery_discharging_test(&msm_chg, 1);
#endif
	
	msm_chg.stop_update = 1;
	cancel_delayed_work(&msm_chg.update_heartbeat_work);
	mutex_lock(&msm_chg.status_lock);
	handle_battery_removed();
	mutex_unlock(&msm_chg.status_lock);
	return 0;
}

static int msm_charger_resume(struct device *dev)
{
	dev_dbg(msm_chg.dev, "%s resumed\n", __func__);

#if defined(PANTECH_CHARGER_TEST_MENU)
	set_alarm_for_battery_discharging_test(&msm_chg, 0);
#endif

	msm_chg.stop_update = 0;
	/* start updaing the battery powersupply every msm_chg.update_time
	 * milliseconds */
	queue_delayed_work(msm_chg.event_wq_thread,
				&msm_chg.update_heartbeat_work,
			      round_jiffies_relative(msecs_to_jiffies
						     (msm_chg.update_time)));
	mutex_lock(&msm_chg.status_lock);
	handle_battery_inserted();
	mutex_unlock(&msm_chg.status_lock);
	return 0;
}

static SIMPLE_DEV_PM_OPS(msm_charger_pm_ops,
		msm_charger_suspend, msm_charger_resume);

static struct platform_driver msm_charger_driver = {
	.probe = msm_charger_probe,
	.remove = __devexit_p(msm_charger_remove),
	.driver = {
		   .name = "msm-charger",
		   .owner = THIS_MODULE,
		   .pm = &msm_charger_pm_ops,
	},
};

static int __init msm_charger_init(void)
{
	int rc;

	INIT_LIST_HEAD(&msm_chg.msm_hardware_chargers);
	msm_chg.count_chargers = 0;
	mutex_init(&msm_chg.msm_hardware_chargers_lock);

	msm_chg.queue = kzalloc(sizeof(struct msm_charger_event)
				* MSM_CHG_MAX_EVENTS,
				GFP_KERNEL);
	if (!msm_chg.queue) {
		rc = -ENOMEM;
		goto out;
	}
	msm_chg.tail = 0;
	msm_chg.head = 0;
	spin_lock_init(&msm_chg.queue_lock);
	msm_chg.queue_count = 0;
	INIT_WORK(&msm_chg.queue_work, process_events);
	msm_chg.event_wq_thread = create_workqueue("msm_charger_eventd");
	if (!msm_chg.event_wq_thread) {
		rc = -ENOMEM;
		goto free_queue;
	}
	rc = platform_driver_register(&msm_charger_driver);
	if (rc < 0) {
		pr_err("%s: FAIL: platform_driver_register. rc = %d\n",
		       __func__, rc);
		goto destroy_wq_thread;
	}
	msm_chg.inited = 1;
	return 0;

destroy_wq_thread:
	destroy_workqueue(msm_chg.event_wq_thread);
free_queue:
	kfree(msm_chg.queue);
out:
	return rc;
}

static void __exit msm_charger_exit(void)
{
	flush_workqueue(msm_chg.event_wq_thread);
	destroy_workqueue(msm_chg.event_wq_thread);
	kfree(msm_chg.queue);
	platform_driver_unregister(&msm_charger_driver);
}

module_init(msm_charger_init);
module_exit(msm_charger_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Abhijeet Dharmapurikar <adharmap@codeaurora.org>");
MODULE_DESCRIPTION("Battery driver for Qualcomm MSM chipsets.");
MODULE_VERSION("1.0");

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
#include <linux/sky_charger_battery.h>
#include <linux/time.h>
#include <linux/slab.h>
#include <linux/wakelock.h>

#include <asm/atomic.h>

#include <mach/msm_hsusb.h>
#include <linux/reboot.h>
#include <linux/sky_pm_tools.h>

/////////////////////////
// Feature define

// debug message
#define DEBUG_ME
#define DEBUG_ME_REL

// sleep operation
#define USE_ALARM_RTC


/////////////////////////
// Definition

#define INIT_HEARTBEAT_WORK_TIME_MS 1000
#define UPDATE_HEARTBEAT_WORK_TIME_MS 10000
#define INIT_WORK_TIME_MS 2000
#define CABLE_DETECT_WORK_TIME_MS 5000

/////////////////////////
// Mecro function

#ifdef DEBUG_ME
#define dbgme(fmt, args...)   printk("[SKYCHARGER]L:%d(%s)" fmt, __LINE__, __func__, ##args)
#else
#define dbgme(fmt, args...)
#endif

#ifdef DEBUG_ME_REL
#define dbgme_rel(fmt, args...)   printk("[SKYCHARGER]L:%d(%s)" fmt, __LINE__, __func__, ##args)
#else
#define dbgme_rel(fmt, args...)
#endif


/////////////////////////
// Data structure

struct sky_charger_battery_chip {
	int inited;

	struct device *dev;

	// base info
	struct sky_battery_info batt_info;
	struct sky_charger_info chg_info;

	//status info
	enum sky_cable_state cable_status;
	enum sky_charger_state charger_status;
	enum sky_battery_state battery_status;
	enum sky_temp_state temp_status;

	//batt data
	int battery_capacity;
	int battery_valtage;

	//type
	enum sky_cable_type cable_type;
	enum sky_battery_id battery_id;
	
	//mutex
	struct mutex status_lock;

	//operator
	struct sky_charger * charger_ops;
	struct sky_cable * cable_ops;
	struct sky_battery * battery_ops;
	struct sky_temp * temp_ops;

	//event
	struct sky_event_queue * event_queue;

	//power supply
	struct power_supply psy_batt;
	struct power_supply psy_ac;
	struct power_supply psy_usb;

	//work queue
	struct delayed_work update_heartbeat_work;
	struct delayed_work init_completed_work;
	struct delayed_work cable_detect_work;

	//wake lock
	struct wake_lock chg_wake_lock;
	struct wake_lock heart_bit_wake_lock;

};

static struct sky_charger_battery_chip sky_chg_chip;


////////////////////////////
// Interface function

//charger interface function
static int sky_start_charging(void)
{
	if (sky_chg_chip.charger_ops && sky_chg_chip.charger_ops->start_charging)
		return sky_chg_chip.charger_ops->start_charging (sky_chg_chip.charger_ops);
	return 0;
}

static int sky_stop_charging(void)
{
	if (sky_chg_chip.charger_ops && sky_chg_chip.charger_ops->stop_charging)
		return sky_chg_chip.charger_ops->stop_charging (sky_chg_chip.charger_ops);
	return 0;
}

static int sky_set_current_limit(int current_ma)
{
	if (sky_chg_chip.charger_ops && sky_chg_chip.charger_ops->set_current_limit)
		return sky_chg_chip.charger_ops->set_current_limit (sky_chg_chip.charger_ops, current_ma);
	return 0;
}

static int sky_get_current_limit(void)
{
	if (sky_chg_chip.charger_ops && sky_chg_chip.charger_ops->get_current_limit)
		return sky_chg_chip.charger_ops->get_current_limit (sky_chg_chip.charger_ops);
	return 0;
}

static int sky_set_charging_current(int current_ma)
{
	if (sky_chg_chip.charger_ops && sky_chg_chip.charger_ops->set_charging_current)
		return sky_chg_chip.charger_ops->set_charging_current (sky_chg_chip.charger_ops, current_ma);
	return 0;
}

static int sky_get_charging_current(void)
{
	if (sky_chg_chip.charger_ops && sky_chg_chip.charger_ops->get_charging_current)
		return sky_chg_chip.charger_ops->get_charging_current (sky_chg_chip.charger_ops);
	return 0;
}

static int sky_set_recharging_valtage(int current_ma)
{
	if (sky_chg_chip.charger_ops && sky_chg_chip.charger_ops->set_recharging_valtage)
		return sky_chg_chip.charger_ops->set_recharging_valtage (sky_chg_chip.charger_ops, current_ma);
	return 0;
}

static int sky_get_recharging_valtage(void)
{
	if (sky_chg_chip.charger_ops && sky_chg_chip.charger_ops->get_recharging_valtage)
		return sky_chg_chip.charger_ops->get_recharging_valtage (sky_chg_chip.charger_ops);
	return 0;
}
	
static int sky_get_charging_status(void)
{
	if (sky_chg_chip.charger_ops && sky_chg_chip.charger_ops->get_charging_status)
		return sky_chg_chip.charger_ops->get_charging_status (sky_chg_chip.charger_ops);
	return 0;
}


//cable interface function
static int sky_get_cable_type(void)
{
	if (sky_chg_chip.cable_ops && sky_chg_chip.cable_ops->get_cable_type)
		return sky_chg_chip.cable_ops->get_cable_type (sky_chg_chip.cable_ops);
	return 0;
}

static int sky_get_cable_status(void)
{
	if (sky_chg_chip.cable_ops && sky_chg_chip.cable_ops->get_cable_status)
		return sky_chg_chip.cable_ops->get_cable_status (sky_chg_chip.cable_ops);
	return 0;
}

//battery interface function
static int sky_get_battery_status(void)
{
	if (sky_chg_chip.battery_ops && sky_chg_chip.battery_ops->get_battery_status)
		return sky_chg_chip.battery_ops->get_battery_status (sky_chg_chip.battery_ops);
	return 0;
}

static int sky_get_battery_id(void)
{
	if (sky_chg_chip.battery_ops && sky_chg_chip.battery_ops->get_battery_id)
		return sky_chg_chip.battery_ops->get_battery_id (sky_chg_chip.battery_ops);
	return 0;
}

static int sky_get_battery_capacity(void)
{
	if (sky_chg_chip.battery_ops && sky_chg_chip.battery_ops->get_battery_capacity)
		return sky_chg_chip.battery_ops->get_battery_capacity (sky_chg_chip.battery_ops);
	return 0;
}

static int sky_get_battery_voltage(void)
{
	if (sky_chg_chip.battery_ops && sky_chg_chip.battery_ops->get_battery_voltage)
		return sky_chg_chip.battery_ops->get_battery_voltage (sky_chg_chip.battery_ops);
	return 0;
}

//temp interface function
static int sky_get_temp_status(void)
{
	if (sky_chg_chip.temp_ops && sky_chg_chip.temp_ops->get_temp_status)
		return sky_chg_chip.temp_ops->get_temp_status (sky_chg_chip.temp_ops);
	return 0;
}

///////////////////////////



////////////////////////////
// Event function

static int handle_init_completed(void *data)
{
	// check all status
	sky_chg_chip.battery_status = sky_get_battery_status();
	sky_chg_chip.battery_id = sky_get_battery_id();
	sky_chg_chip.charger_status = sky_get_charging_status();
	sky_chg_chip.cable_status = sky_get_cable_status();
	sky_chg_chip.cable_type = sky_get_cable_type();

	if (sky_chg_chip.cable_status == CABLE_STATE_PRESENT)
	{
		sky_charger_battery_notify_event(EVENT_CABLE_INSERTED, NULL);			
	}		
	
	
	//now start battery work
	schedule_delayed_work(&sky_chg_chip->update_heartbeat_work, msecs_to_jiffies(INIT_HEARTBEAT_WORK_TIME_MS));
	return 1;
}

static int handle_cable_inserted(void *data)
{
	if (sky_chg_chip.cable_status != CABLE_STATE_PRESENT)
	{
		dbgme("Enter: cable_status:%d\n", sky_chg_chip.cable_status);

		sky_chg_chip.cable_status = CABLE_STATE_PRESENT;
		sky_chg_chip.cable_type = CABLE_TYPE_UNKNOWN;		

		// wakelock
		wake_lock(&sky_chg_chip.chg_wake_lock);

		//setup default value (default cable type usb)

		sky_set_current_limit(sky_chg_chip.chg_info_usb.max_current_limit);
		sky_set_charging_current(sky_chg_chip.chg_info_usb.charging_current);
		sky_set_recharging_valtage(sky_chg_chip.chg_info_usb.recharging_voltage);
		
		//start charging
		sky_start_charging();
		
		//start check cable detect work
		schedule_delayed_work(&sky_chg_chip->cable_detect_work, msecs_to_jiffies(CABLE_DETECT_WORK_TIME_MS));	
		return 1;
	}
	return 0;
}

static int handle_cable_detected(void *data)
{
	enum sky_cable_type cable_type;
	
	if (sky_chg_chip.cable_status == CABLE_STATE_PRESENT)
	{
		dbgme("Enter: cable_status:%d\n", sky_chg_chip.cable_status);

		//cancel check cable detect work
		cancel_delayed_work_sync(&sky_chg_chip->cable_detect_work);

		cable_type = sky_get_cable_type();

		if (sky_chg_chip.cable_type != cable_type)
		{
			sky_chg_chip.cable_type = cable_type;

			dbgme("cable_type:%d\n", sky_chg_chip.cable_type);

			if (sky_chg_chip.cable_type == CABLE_TYPE_AC)
			{
				sky_set_current_limit(sky_chg_chip.chg_info_ac.max_current_limit);
				sky_set_charging_current(sky_chg_chip.chg_info_ac.charging_current);
				sky_set_recharging_valtage(sky_chg_chip.chg_info_ac.recharging_voltage);
			}			
			return 1;
		}
	}
	return 0;
}

static int handle_cable_removed(void *data)
{
	dbgme("Enter: cable_status:%d\n", sky_chg_chip.cable_status);

	//cancel check cable detect work
	cancel_delayed_work_sync(&sky_chg_chip->cable_detect_work);

	sky_chg_chip.cable_status = CABLE_STATE_ABSENT;
	sky_chg_chip.cable_type = CABLE_TYPE_NONE;		

	//stop charging
	sky_stop_charging();

	// wake unlock
	wake_unlock(&sky_chg_chip.chg_wake_lock);
	return 1;
}

static int handle_batt_inserted(void *data)
{
	return 0;
}

static int handle_batt_removed(void *data)
{
	return 0;
}

static int handle_chg_status_change(void *data)
{
	enum sky_charger_state charger_status;
	
	charger_status = sky_get_charging_status();

	if (sky_chg_chip.battery_status == BATT_STATE_PRESENT && 
		sky_chg_chip.cable_status == CABLE_STATE_PRESENT &&
		charger_status != sky_chg_chip.charger_status)
	{
		sky_chg_chip.charger_status = charger_status;
		return 1;
	}
	return 0;
}

static int handle_batt_status_change(void *data)
{
	enum sky_battery_state battery_status;

	battery_status = sky_get_battery_status();

	if (battery_status != sky_chg_chip.battery_status)
	{
		sky_chg_chip.battery_status = battery_status;
		return 1;
	}
	return 0;
}

static int handle_batt_capacity_change(void *data)
{
	int battery_capacity;

	battery_capacity = sky_get_battery_capacity();

	if (sky_chg_chip.battery_status == BATT_STATE_PRESENT && 
		battery_capacity != sky_chg_chip.battery_capacity)
	{
		sky_chg_chip.battery_capacity = battery_capacity;
		return 1;
	}
	return 0;
}

static int handle_temp_status_change(void *data)
{
	enum sky_temp_state temp_status;

	temp_status = sky_get_temp_status();

	if (temp_status != sky_chg_chip.temp_status)
	{
		sky_chg_chip.temp_status = temp_status;
		return 1;
	}
	return 0;
}


static void sky_handle_event(int event, void *data)
{
	int need_psy_change_batt = 0;
	int need_psy_change_usb = 0;
	int need_psy_change_ac = 0;
	
	mutex_lock(&sky_chg_chip.status_lock);

	dbgme("Enter event=%d\n", event);

	switch (event) {
	case EVENT_INIT_COMPLETED:
		dbgme("Event : EVENT_INIT_COMPLETED\n");
		handle_init_completed(data);
		break;
	case EVENT_CABLE_INSERTED:
		dbgme("Event : EVENT_CABLE_INSERTED\n");
		if (handle_cable_inserted(data))
		{
			if (sky_chg_chip.cable_type == CABLE_TYPE_USB || 
				sky_chg_chip.cable_type == CABLE_TYPE_FACTORY ||
				sky_chg_chip.cable_type == CABLE_TYPE_UNKNOWN
				)
				need_psy_change_usb = 1;
			else if (sky_chg_chip.cable_type == CABLE_TYPE_AC)
				need_psy_change_ac = 1;
		}
		break;
	case EVENT_CABLE_DETECTED:
		dbgme("Event : EVENT_CABLE_DETECTED\n");
		if (handle_cable_detected(data))
		{
			if (sky_chg_chip.cable_type == CABLE_TYPE_USB || 
				sky_chg_chip.cable_type == CABLE_TYPE_FACTORY ||
				sky_chg_chip.cable_type == CABLE_TYPE_UNKNOWN
				)
				need_psy_change_usb = 1;
			else if (sky_chg_chip.cable_type == CABLE_TYPE_AC)
				need_psy_change_ac = 1;
		}
		break;
	case EVENT_CABLE_REMOVED:
		dbgme("Event : EVENT_CABLE_REMOVED\n");
		if (handle_cable_removed(data))
		{
			if (sky_chg_chip.cable_type == CABLE_TYPE_USB || 
				sky_chg_chip.cable_type == CABLE_TYPE_FACTORY ||
				sky_chg_chip.cable_type == CABLE_TYPE_UNKNOWN
				)
				need_psy_change_usb = 1;
			else if (sky_chg_chip.cable_type == CABLE_TYPE_AC)
				need_psy_change_ac = 1;
		}
		break;
	case EVENT_BATT_INSERTED:
		dbgme("Event : EVENT_BATT_INSERTED\n");
		if (handle_batt_inserted(data))
			need_psy_change_batt = 1;
		break;
	case EVENT_BATT_REMOVED:
		dbgme("Event : EVENT_BATT_REMOVED\n");
		if (handle_batt_removed(data))
			need_psy_change_batt = 1;
		break;
	case EVENT_CHG_STATUS_CHANGE:
		dbgme("Event : EVENT_CHG_STATUS_CHANGE\n");
		if (handle_chg_status_change(data))
		{
			if (sky_chg_chip.cable_type == CABLE_TYPE_USB || 
				sky_chg_chip.cable_type == CABLE_TYPE_FACTORY ||
				sky_chg_chip.cable_type == CABLE_TYPE_UNKNOWN
				)
				need_psy_change_usb = 1;
			else if (sky_chg_chip.cable_type == CABLE_TYPE_AC)
				need_psy_change_ac = 1;
		}
		break;
	case EVENT_BATT_STATUS_CHANGE:
		dbgme("Event : EVENT_BATT_STATUS_CHANGE\n");
		if (handle_batt_status_change(data))
			need_psy_change_batt = 1;
		break;
	case EVENT_BATT_CAPACITY_CHANGE:
		dbgme("Event : EVENT_BATT_STATUS_CHANGE\n");
		if (handle_batt_capacity_change(data))
			need_psy_change_batt = 1;
		break;
	case EVENT_TEMP_STATUS_CHANGE:
		dbgme("Event : EVENT_TEMP_STATUS_CHANGE\n");
		handle_temp_status_change(data);
		break;
	}

	if (need_psy_change_batt)
		power_supply_changed(&sky_chg_chip.psy_batt);
	if (need_psy_change_usb)
		power_supply_changed(&sky_chg_chip.psy_usb);
	if (need_psy_change_ac)
		power_supply_changed(&sky_chg_chip.psy_ac);

	
	mutex_unlock(&sky_chg_chip.status_lock);
}

int sky_charger_battery_notify_event(enum sky_charger_battery_event event, void  *data)
{
	sky_add_event(sky_chg_chip.event_queue ,(int)event, data);
	return 0;
}
EXPORT_SYMBOL(sky_charger_battery_notify_event);

///////////////////////////


///////////////////////////
//Registor functions

void sky_charger_register(struct sky_charger *chg)
{
	sky_chg_chip.charger_ops = chg;
}	
EXPORT_SYMBOL(sky_charger_register);

void sky_charger_unregister(struct sky_charger *chg)
{
	sky_chg_chip.charger_ops = NULL;
}	
EXPORT_SYMBOL(sky_charger_unregister);

void sky_battery_register(struct sky_battery *batt)
{
	sky_chg_chip.battery_ops = batt;
}	
EXPORT_SYMBOL(sky_battery_register);

void sky_battery_unregister(struct sky_battery *batt)
{
	sky_chg_chip.battery_ops = NULL;
}	
EXPORT_SYMBOL(sky_battery_unregister);

void sky_cable_register(struct sky_cable *cable)
{
	sky_chg_chip.cable_ops = cable;
}	
EXPORT_SYMBOL(sky_cable_register);

void sky_cable_unregister(struct sky_cable *cable)
{
	sky_chg_chip.cable_ops = NULL;
}	
EXPORT_SYMBOL(sky_cable_unregister);

void sky_temp_register(struct sky_temp *temp)
{
	sky_chg_chip.temp_ops = temp;
}	
EXPORT_SYMBOL(sky_temp_register);

void sky_temp_unregister(struct sky_temp *temp)
{
	sky_chg_chip.temp_ops = NULL;
}	
EXPORT_SYMBOL(sky_temp_unregister);

////////////////////
// power supply class

//battery
static enum power_supply_property sky_batt_power_props[] = {
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

static int sky_batt_power_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		if (sky_chg_chip.battery_capacity >= 100)
			val->intval = POWER_SUPPLY_STATUS_FULL;
		else if (sky_chg_chip.charger_status == CHG_STATE_CHARGING)
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else if (sky_chg_chip.charger_status == CHG_STATE_DISCHARGING)
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		else 
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		if (sky_chg_chip.charger_status == CHG_STATE_CHARGING)
			val->intval = POWER_SUPPLY_CHARGE_TYPE_FAST;
		else
			val->intval = POWER_SUPPLY_CHARGE_TYPE_NONE;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = !(sky_chg_chip.battery_status == BATT_STATE_ABSENT);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = sky_chg_chip.batt_info.max_voltage * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = sky_chg_chip.batt_info.min_voltage * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = sky_chg_chip.battery_valtage;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = sky_chg_chip.battery_capacity;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static enum power_supply_property sky_power_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
};


//charger
static char *sky_power_supplied_to[] = {
	"battery",
};

static int sky_power_get_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = CHG_PRESENT_STATE;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		if (psy->type == POWER_SUPPLY_TYPE_MAINS) {
			val->intval = (sky_chg_chip.cable_status == CABLE_STATE_PRESENT)? 1:0;
		}
		if (psy->type == POWER_SUPPLY_TYPE_USB) {
			val->intval = (sky_chg_chip.cable_status == CABLE_STATE_PRESENT)? 1:0;
		}
		dbgme(" POWER_SUPPLY_PROP_ONLINE=%d\n", val->intval);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}
///////////////////

///////////////////
// work queue
static void sky_init_completed_work(struct work_struct *work)
{
	dbgme(" Enter\n");

	// check battery status
	if (sky_chg_chip.battery_status == BATT_STATE_NONE)
		sky_chg_chip.battery_status = sky_get_battery_status();

	// check battery id
	if (sky_chg_chip.battery_status == BATT_STATE_PRESENT && sky_chg_chip.battery_id == BATT_ID_NONE)
		sky_chg_chip.battery_id = sky_get_battery_id();

	// check charger status
	if (sky_chg_chip.charger_status == CHG_STATE_NONE)
		sky_chg_chip.charger_status = sky_get_charging_status();

	// check charger status
	if (sky_chg_chip.cable_status == CHG_STATE_NONE)
		sky_chg_chip.cable_status = sky_get_cable_status();

	if (sky_chg_chip.cable_status  == CABLE_STATE_PRESENT && sky_chg_chip.cable_type == CABLE_TYPE_NONE)
		sky_chg_chip.cable_type = sky_get_cable_type();

	if (sky_chg_chip.cable_status == CABLE_STATE_NONE ||
		sky_chg_chip.charger_status == CHG_STATE_NONE ||
		sky_chg_chip.battery_status == BATT_STATE_NONE ||
		sky_chg_chip.temp_status == TEMP_STATE_NONE )
	{// init complete yet!!
		schedule_delayed_work(&sky_chg_chip->init_completed_work, msecs_to_jiffies(INIT_WORK_TIME_MS));
	}
	else
	{
		sky_charger_battery_notify_event(EVENT_INIT_COMPLETED, NULL);
	}
}

static void sky_update_heartbeat_work(struct work_struct *work)
{
	int need_psy_change_batt = 0;
	int need_psy_change_usb = 0;
	int need_psy_change_ac = 0;

	dbgme(" Enter\n");

	wake_lock(&sky_chg_chip.heart_bit_wake_lock);

	// check battey status
	if (sky_chg_chip.battery_status == BATT_STATE_PRESENT)
	{
		//check battery capacity
		need_psy_change_batt = handle_batt_capacity_change(NULL);

		handle_temp_status_change(NULL);
	}
	else if (sky_chg_chip.cable_status == CABLE_STATE_PRESENT && 
		sky_chg_chip.cable_type == CABLE_TYPE_FACTORY)
	{ // Factory cable
		//check battery capacity
		need_psy_change_batt = handle_batt_capacity_change(NULL);
	}
	else
	{ // no cable and no battery !!??
		// power off
	}

	if (need_psy_change_batt)
		power_supply_changed(&sky_chg_chip.psy_batt);
	if (need_psy_change_usb)
		power_supply_changed(&sky_chg_chip.psy_usb);
	if (need_psy_change_ac)
		power_supply_changed(&sky_chg_chip.psy_ac);

	wake_unlock(&sky_chg_chip.heart_bit_wake_lock);

	schedule_delayed_work(&sky_chg_chip->update_heartbeat_work, msecs_to_jiffies(UPDATE_HEARTBEAT_WORK_TIME_MS));	
}

static void sky_cable_detect_work(struct work_struct *work)
{
	dbgme(" Enter\n");
	
	sky_charger_battery_notify_event(EVENT_CABLE_DETECTED, NULL);
}

static int __devinit sky_charger_battery_probe(struct platform_device *pdev)
{
	sky_chg_chip.dev = &pdev->dev;
	if (pdev->dev.platform_data) {
		struct sky_charger_battery_platform_data *pdata = (struct sky_charger_battery_platform_data *)pdev->dev.platform_data;

		memcpy(&sky_chg_chip.batt_info, &pdata->batt_info,  sizeof(struct sky_battery_info)) ;
		memcpy(&sky_chg_chip.chg_info, &pdata->chg_info,  sizeof(struct sky_charger_info)) ;

	}

	// power supply
	// battery
	sky_chg_chip.psy_batt.name = "battery";
	sky_chg_chip.psy_batt.type = POWER_SUPPLY_TYPE_BATTERY;
	sky_chg_chip.psy_batt.properties = sky_batt_power_props;
	sky_chg_chip.psy_batt.num_properties = ARRAY_SIZE(sky_batt_power_props);
	sky_chg_chip.psy_batt.get_property = sky_batt_power_get_property;
	power_supply_register(sky_chg_chip.dev, &sky_chg_chip.psy_batt);

	//ac
	sky_chg_chip.psy_ac.name = "ac",
	sky_chg_chip.psy_ac.type = POWER_SUPPLY_TYPE_MAINS;
	sky_chg_chip.psy_ac.supplied_to = sky_power_supplied_to;
	sky_chg_chip.psy_ac.num_supplicants = ARRAY_SIZE(sky_power_supplied_to);
	sky_chg_chip.psy_ac.properties = sky_power_props;
	sky_chg_chip.psy_ac.num_properties = ARRAY_SIZE(sky_power_props);
	sky_chg_chip.psy_ac.get_property = sky_power_get_property;
	power_supply_register(sky_chg_chip.dev, &sky_chg_chip.psy_ac);

	//usb
	sky_chg_chip.psy_usb.name = "usb";
	sky_chg_chip.psy_usb.type = POWER_SUPPLY_TYPE_USB;
	sky_chg_chip.psy_usb.supplied_to = sky_power_supplied_to;
	sky_chg_chip.psy_usb.num_supplicants = ARRAY_SIZE(sky_power_supplied_to);
	sky_chg_chip.psy_usb.properties = sky_power_props;
	sky_chg_chip.psy_usb.num_properties = ARRAY_SIZE(sky_power_props);
	sky_chg_chip.psy_usb.get_property = sky_power_get_property;
	power_supply_register(sky_chg_chip.dev, &sky_chg_chip.psy_usb);


	//status info
	sky_chg_chip.cable_status = CABLE_STATE_NONE;
	sky_chg_chip.charger_status = CHG_STATE_NONE;
	sky_chg_chip.battery_status = BATT_STATE_NONE;
	sky_chg_chip.temp_status = TEMP_STATE_NONE;

	sky_chg_chip.cable_type = CABLE_TYPE_NONE;
	sky_chg_chip.battery_id = BATT_ID_NONE;

	//operator
	sky_chg_chip.charger_ops = NULL;
	sky_chg_chip.cable_ops = NULL;
	sky_chg_chip.battery_ops = NULL;
	sky_chg_chip.temp_ops = NULL;

	mutex_init(&sky_chg_chip.status_lock);

	// init work queue
	INIT_DELAYED_WORK(&sky_chg_chip.update_heartbeat_work, sky_update_heartbeat_work);
	INIT_DELAYED_WORK(&sky_chg_chip.init_completed_work, sky_init_completed_work);
	INIT_DELAYED_WORK(&sky_chg_chip.cable_detect_work, sky_cable_detect_work);

	// init wakelock
	wake_lock_init(&sky_chg_chip.chg_wake_lock, WAKE_LOCK_SUSPEND, "chg_wake_lock");
	wake_lock_init(&sky_chg_chip.heart_bit_wake_lock, WAKE_LOCK_SUSPEND, "heart_bit_wake_lock");

	schedule_delayed_work(&sky_chg_chip.init_completed_work, msecs_to_jiffies(INIT_WORK_TIME_MS));
	
	return 0;
}

static int __devexit sky_charger_battery_remove(struct platform_device *pdev)
{
	mutex_destroy(&sky_chg_chip.status_lock);
	power_supply_unregister(&sky_chg_chip.psy_batt);
	power_supply_unregister(&sky_chg_chip.psy_ac);
	power_supply_unregister(&sky_chg_chip.psy_usb);
	return 0;
}

static int sky_charger_battery_suspend(struct device *dev)
{
	return 0;
}

static int sky_charger_battery_resume(struct device *dev)
{
	return 0;
}

static SIMPLE_DEV_PM_OPS(sky_charger_battery_pm_ops,
		sky_charger_battery_suspend, sky_charger_battery_resume);

static struct platform_driver sky_charger_battery_driver = {
	.probe = sky_charger_battery_probe,
	.remove = __devexit_p(sky_charger_battery_remove),
	.driver = {
		   .name = "sky_charger_battery",
		   .owner = THIS_MODULE,
		   .pm = &sky_charger_battery_pm_ops,
	},
};

static int __init sky_charger_battery_init(void)
{
	int rc;

	sky_chg_chip.event_queue = sky_create_event_queue(16, sky_handle_event);


	rc = platform_driver_register(&sky_charger_battery_driver);
	if (rc < 0) {
		pr_err("%s: FAIL: platform_driver_register. rc = %d\n",
		       __func__, rc);
		goto out;
	}
	sky_chg_chip.inited = 1;
	return 0;

out:
	return rc;
}

static void __exit sky_charger_battery_exit(void)
{
	platform_driver_unregister(&sky_charger_battery_driver);
}

module_init(sky_charger_battery_init);
module_exit(sky_charger_battery_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("lim hyungun <lim.hyungun@pantech.com>");
MODULE_DESCRIPTION("Charger Battery driver for SKY devices.");
MODULE_VERSION("1.0");

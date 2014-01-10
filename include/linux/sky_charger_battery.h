/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
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
#ifndef __SKY_CHARGER_BATTERY_H__
#define __SKY_CHARGER_BATTERY_H__

#include <linux/power_supply.h>

enum sky_charger_battery_event {
	EVENT_INIT_COMPLETED,
	EVENT_CABLE_INSERTED,
	EVENT_CABLE_DETECTED,
	EVENT_CABLE_REMOVED,
	EVENT_BATT_INSERTED,
	EVENT_BATT_REMOVED,
	EVENT_CHG_STATUS_CHANGE,
	EVENT_BATT_STATUS_CHANGE,
	EVENT_BATT_CAPACITY_CHANGE,
	EVENT_TEMP_STATUS_CHANGE,
};

enum sky_cable_type{
	CABLE_TYPE_NONE,
	CABLE_TYPE_UNKNOWN,
	CABLE_TYPE_USB,
	CABLE_TYPE_AC,
	CABLE_TYPE_FACTORY,
	CABLE_TYPE_OTG,
};

enum sky_cable_state{
	CABLE_STATE_NONE,
	CABLE_STATE_ABSENT,
	CABLE_STATE_PRESENT,
};

enum sky_charger_state {
	CHG_STATE_NONE,
	CHG_STATE_DISCHARGING,
	CHG_STATE_CHARGING,
};

enum sky_battery_state {
	BATT_STATE_NONE,
	BATT_STATE_ABSENT,
	BATT_STATE_PRESENT,
};

enum sky_battery_id {
	BATT_ID_NONE,
	BATT_ID_DUMMY,
	BATT_ID_NORMAL,	
	BATT_ID_SAMSUNG,
	BATT_ID_LG,
};

enum sky_temp_state {
	TEMP_STATE_NONE,
	TEMP_STATE_NORMAL,
	TEMP_STATE_SOFT_HIGH,
	TEMP_STATE_HARD_HIGH,
	TEMP_STATE_SOFT_LOW,
	TEMP_STATE_HARD_LOW,
	TEMP_STATE_OUT_OF_RANGE,
};

enum sky_system_load_state {
	SYS_STATE_HIGH,
	SYS_STATE_MID,
	SYS_STATE_LOW,
};

enum sky_charger_battery_data_type {
	SKY_CHARGER,
	SKY_CABLE,
	SKY_BATTERY,
	SKY_TEMP,
};

struct sky_charger {
	int data_type;
	const char *name;
	void *data;

	int (*start_charging) (struct sky_charger *chg);
	int (*stop_charging) (struct sky_charger *chg);
	int (*set_current_limit) (struct sky_charger *chg, int current_ma);
	int (*set_charging_current) (struct sky_charger *chg, int current_ma);
	int (*set_recharging_valtage) (struct sky_charger *chg, int current_ma);
	int (*get_current_limit) (struct sky_charger *chg);
	int (*get_charging_current) (struct sky_charger *chg);
	int (*get_recharging_valtage) (struct sky_charger *chg);
	int (*get_charging_status) (struct sky_charger *chg);
};

struct sky_cable{
	int data_type;
	const char *name;
	void *data;

	int (*get_cable_type) (struct sky_cable *cable);
	int (*get_cable_status) (struct sky_cable *cable);
};

struct sky_battery{
	int data_type;
	const char *name;
	void *data;

	int (*get_battery_status) (struct sky_battery *batt);
	int (*get_battery_id) (struct sky_battery *batt);
	int (*get_battery_capacity) (struct sky_battery *batt);
	int (*get_battery_voltage) (struct sky_battery *batt);	
};

struct sky_temp{
	int data_type;
	const char *name;
	void *data;

	int (*get_temp_status) (struct sky_temp *temp);
};

struct sky_battery_info {
	unsigned int max_current;
	unsigned int min_current;
	unsigned int max_voltage;
	unsigned int min_voltage;
	unsigned int min_percent;
	unsigned int max_percent;
};

struct sky_charger_info {
	unsigned int max_current_limit;
	unsigned int charging_current;
	unsigned int recharging_voltage;	
};

struct sky_charger_battery_platform_data {
	struct sky_battery_info batt_info;
	struct sky_charger_info chg_info_usb;
	struct sky_charger_info chg_info_ac;
};

extern void sky_charger_register(struct sky_charger *chg);
extern void sky_charger_unregister(struct sky_charger *chg);
extern void sky_battery_register(struct sky_battery *batt);
extern void sky_battery_unregister(struct sky_battery *batt);
extern void sky_cable_register(struct sky_cable *cable);
extern void sky_cable_unregister(struct sky_cable *cable);
extern void sky_temp_register(struct sky_temp *temp);
extern void sky_temp_unregister(struct sky_temp *temp);

extern int sky_charger_battery_notify_event(enum sky_charger_battery_event event, void  *data);

#endif /* __SKY_CHARGER_BATTERY_H__ */

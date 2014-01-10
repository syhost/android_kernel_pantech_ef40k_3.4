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
#include <linux/max17040_battery_new.h>
#include <linux/slab.h>
#ifdef CONFIG_SKY_CHARGER_BATTERY
#include <linux/sky_charger_battery.h>
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

#define MAX_READ	10

// debug message
#define DEBUG_ME
#define DEBUG_ME_REL

#ifdef DEBUG_ME
#define dbgme(fmt, args...)   printk("[MAX17040_NEW]L:%d(%s)" fmt, __LINE__, __func__, ##args)
#else
#define dbgme(fmt, args...)
#endif

#ifdef DEBUG_ME_REL
#define dbgme_rel(fmt, args...)   printk("[MAX17040_NEW]L:%d(%s)" fmt, __LINE__, __func__, ##args)
#else
#define dbgme_rel(fmt, args...)
#endif

#define SKY_SOC_LSB	256
#define SKY_MULT_100(x)	(x*100)	
#define SKY_MULT_1000(x)	(x*1000)	
#define SKY_MULT_10000(x)	(x*10000)	

#define SKY_MULT_1000000(x)	(x*1000000)	
#define SKY_DIV_1000000(x)	(x/1000000)	

struct max17040_chip {
	struct mutex	i2c_mutex;
	struct i2c_client		*client;
	struct max17040_platform_data	*pdata;

#ifdef CONFIG_SKY_CHARGER_BATTERY
	int status;
#endif	
	int vcell;
	int soc;
};
 
struct max17040_chip max17040_data;


static int max17040_write_reg(u8 reg, u8 ahigh, u8 alow)
{
	int ret;
	u8 buf[20];
	buf[0]=reg;
	buf[1]=ahigh;
	buf[2]=alow;
	if ( max17040_data.client == NULL ) {
		dbgme_rel("%s : max17040.client is NULL\n", __func__);
		return -ENODEV;
	}
	mutex_lock(&max17040_data.i2c_mutex); 
	ret = i2c_master_send(max17040_data.client, buf, 3);		
	mutex_unlock(&max17040_data.i2c_mutex);
	if (ret<0)
		return -1;	
	else if (ret!=3)
		return -1;
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
		dbgme_rel("max17040 I2C FILED ret [%d] [%d]\n",ret_s,ret_r);		
		return -1;
	}
	else if(ret_s !=1 || ret_r !=1)
	{
		dbgme_rel("max17040 I2C FILED [%d] [%d] bytes transferred (expected 1)\n",ret_s,ret_r);				
		return -1;
	}
	else
		ret=buf[0];
	return ret;
}


static void max17040_set_rcomp(void)
{
	int ret=0,i=0;	
	ret=max17040_write_reg(MAX17040_RCOMP_MSB, 0xc0,0x00);
	if(ret<0)
	{
		for(i=0;i<MAX_READ;i++)
		{
			ret=max17040_write_reg(MAX17040_RCOMP_MSB, 0xc0,0x00);			
			if(ret<0)
				continue;
			else
				break;
		}
	}
	dbgme_rel("MAX17040 SET_RCOMP TRY : [%d]\n",i);			
}

static int max17040_get_vcell(void)
{
	u8 msb;
	u8 lsb;
	int voltage=0;
	int vcell=0;

	msb = max17040_read_reg(MAX17040_VCELL_MSB);
	lsb = max17040_read_reg(MAX17040_VCELL_LSB);

	//check i2c error
	if(msb<0 ||lsb <0)
	{
		vcell = max17040_data.vcell;
	}
	else
	{
		voltage=(msb<<4)|((lsb&0xf0)>>4);
		vcell=(voltage*125)/100;
		max17040_data.vcell = vcell;
	}
	return vcell;
}

static int max17040_get_soc(void)
{
	u8 msb;
	u8 lsb;
	int soc;

	msb = max17040_read_reg(MAX17040_SOC_MSB);
	lsb = max17040_read_reg(MAX17040_SOC_LSB);

	//check i2c error
	if(msb<0 ||lsb <0)
	{
		soc = max17040_data.soc;
	}
	else
	{
		soc = SKY_MULT_1000(msb)+(SKY_MULT_1000(lsb)/SKY_SOC_LSB);	
		max17040_data.soc = soc;
	}

	return soc;
}

static void max17040_get_version(void)
{
	u8 msb;
	u8 lsb;

	msb = max17040_read_reg(MAX17040_VER_MSB);
	lsb = max17040_read_reg(MAX17040_VER_LSB);
	dbgme("MAX17040 Fuel-Gauge Ver %d%d\n", msb, lsb);
}

#ifdef CONFIG_SKY_CHARGER_BATTERY
//-------------------------
//battery interface function

static int max17040_get_battery_status(struct sky_battery *batt)
{
	return  max17040_data.status;
}

static int max17040_get_battery_id(struct sky_battery *batt)
{
	return BATT_ID_NORMAL;
}

static int max17040_get_battery_capacity(struct sky_battery *batt)
{
	return max17040_get_soc();
}

static int max17040_get_battery_voltage(struct sky_battery *batt)
{
	return max17040_get_vcell();
}

static struct sky_battery battery_ops = {
	.data_type = SKY_BATTERY,
	.name = "max17040_battery",
	.data = &max17040_data,

	.get_battery_status = max17040_get_battery_status,
	.get_battery_id = max17040_get_battery_id,
	.get_battery_capacity = max17040_get_battery_capacity,
	.get_battery_voltage = max17040_get_battery_voltage,

};
#endif

static int __devinit max17040_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
    struct input_dev *input_data = NULL;	
	int ret;

#ifdef CONFIG_SKY_CHARGER_BATTERY
	max17040_data.status = BATT_STATE_NONE;
#endif

	dbgme_rel("[MAX17040] max17040_probe [IN]\n");	
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE))
		return -EIO;

	max17040_data.client = client;

	i2c_set_clientdata(client, &max17040_data);

	max17040_data.vcell = 0;
	max17040_data.soc = 0;

#ifdef CONFIG_SKY_CHARGER_BATTERY
	//registor
	sky_battery_register(&battery_ops);
#endif

	//mutex is init
	mutex_init(&max17040_data.i2c_mutex);	

	//rcomp is set
	max17040_set_rcomp();

	//Version of reading
	max17040_get_version();

#ifdef CONFIG_SKY_CHARGER_BATTERY
	if (max17040_get_battery_id() != BATT_ID_NONE)
		max17040_data.status = BATT_STATE_PRESENT;
#endif

	return 0;
}

static int __devexit max17040_remove(struct i2c_client *client)
{
	struct max17040_chip *chip = i2c_get_clientdata(client);
	i2c_set_clientdata(client, NULL);
	return 0;
}

MODULE_DEVICE_TABLE(i2c, max17040_id);

static struct i2c_driver max17040_i2c_driver = {
	.driver	= {
		.name	= "max17040_new",	
	},
	.probe		= max17040_probe,
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

MODULE_AUTHOR("Minkyu Kang <mk7.kang@samsung.com>");
MODULE_DESCRIPTION("MAX17040 Fuel Gauge");
MODULE_LICENSE("GPL");

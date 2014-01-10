#ifndef _MSM8X60_SKY_SENSOR_H_
#define _MSM8X60_SKY_SENSOR_H_

#include <linux/gpio.h>
#include <linux/i2c-gpio.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <mach/vreg.h>

// SENSOR KEY WORD : CONFIG_INPUT_SENSOR

//I2C NUMBER
#define I2C_DEV_INDEX_PROXIMITY			13 //  /dev/i2c-13

//#if defined(CONFIG_MACH_MSM8X60_EF39S) || defined(CONFIG_MACH_MSM8X60_EF40K) || defined(CONFIG_MACH_MSM8X60_EF40S) || defined(CONFIG_MACH_MSM8X60_EF65L)
#define GYROSCOPE_UDELAY 2 // 250 kHz
#define GYROSCOPE_INT_PIN       30
//#endif

// PROXIMITY
#if defined(CONFIG_MACH_MSM8X60_EF39S) || defined(CONFIG_MACH_MSM8X60_EF40K) || defined(CONFIG_MACH_MSM8X60_EF40S) || defined(CONFIG_MACH_MSM8X60_EF65L)
#define PROXIMITY_SCL_PIN 36
#define PROXIMITY_SDA_PIN 35
#define PROXIMITY_UDELAY 2 // 250 kHz
#if defined(CONFIG_MACH_MSM8X60_EF65L)
#define PROXIMITY_INT_PIN       46
#else
#define PROXIMITY_INT_PIN       34
#endif
#else //defined(CONFIG_MACH_MSM8X60_EF33S) || defined(CONFIG_MACH_MSM8X60_EF34K) 
#define PROXIMITY_SCL_PIN 52
#define PROXIMITY_SDA_PIN 51
#define PROXIMITY_UDELAY 2 // 250 kHz
#define PROXIMITY_INT_PIN       124
#endif

static struct i2c_gpio_platform_data gpio_i2c_proximity_data = {
#if defined(PROXIMITY_SCL_PIN) && defined(PROXIMITY_SDA_PIN) && defined(PROXIMITY_UDELAY)
	.scl_pin = PROXIMITY_SCL_PIN,
	.sda_pin = PROXIMITY_SDA_PIN,
	.udelay = PROXIMITY_UDELAY
#endif
};

struct platform_device msm_device_sensor_proximity = {
	.name = "i2c-gpio",
	.id = I2C_DEV_INDEX_PROXIMITY,
	.dev = {
		.platform_data = &gpio_i2c_proximity_data,
	}
};


static struct i2c_board_info __initdata proximity_i2c_info[] __initdata = {
	{
		I2C_BOARD_INFO("apds9900", 0x39),
	},
};

static struct i2c_board_info __initdata gyroscope_i2c_info[] __initdata = {
	{
		I2C_BOARD_INFO("mpu3050", 0x68),
		.irq = MSM_GPIO_TO_INT(GYROSCOPE_INT_PIN), //gpio_to_irq
	},
	{
		I2C_BOARD_INFO("geomagnetic", 0x2e),
	},
};


static int sensors_hw_init(void)
{
	int rc = 0;
	
#ifdef GYROSCOPE_INT_PIN
	rc = gpio_tlmm_config(GPIO_CFG(GYROSCOPE_INT_PIN, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	if (rc) {
		pr_err("%s: Could not configure gpio %d\n", __func__, GYROSCOPE_INT_PIN);
		return rc;
	}
#endif

#ifdef PROXIMITY_INT_PIN
	rc = gpio_tlmm_config(GPIO_CFG(PROXIMITY_INT_PIN, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	if (rc) {
		pr_err("%s: Could not configure gpio %d\n", __func__, PROXIMITY_INT_PIN);
		return rc;
	}
#endif

#ifdef PROXIMITY_SCL_PIN
	rc = gpio_tlmm_config(GPIO_CFG(PROXIMITY_SCL_PIN, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	if (rc) {
		pr_err("%s: Could not configure gpio %d\n", __func__, PROXIMITY_SCL_PIN);
		return rc;
	}
#endif

#ifdef PROXIMITY_SDA_PIN
	rc = gpio_tlmm_config(GPIO_CFG(PROXIMITY_SDA_PIN, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	if (rc) {
		pr_err("%s: Could not configure gpio %d\n", __func__, PROXIMITY_SDA_PIN);
		return rc;
	}
#endif
	return rc;
}
#endif // _MSM8X60_SKY_SENSOR_H_

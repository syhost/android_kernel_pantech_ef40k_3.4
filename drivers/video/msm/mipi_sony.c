/* Copyright (c) 2008-2010, Code Aurora Forum. All rights reserved.
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include "msm_fb.h"
#include "mipi_dsi.h"
#include "mipi_sony.h"
#include <mach/gpio.h>
#include <asm/irq.h>
#include <asm/system.h>

/* Defines for Debug Messages */
#define LCD_DEBUG_MSG

#ifdef LCD_DEBUG_MSG
#define ENTER_FUNC()        printk(KERN_INFO "[SKY_LCD] +%s \n", __FUNCTION__);
#define EXIT_FUNC()         printk(KERN_INFO "[SKY_LCD] -%s \n", __FUNCTION__);
#define ENTER_FUNC2()       printk(KERN_ERR "[SKY_LCD] +%s\n", __FUNCTION__);
#define EXIT_FUNC2()        printk(KERN_ERR "[SKY_LCD] -%s\n", __FUNCTION__);
#define PRINT(fmt, args...) printk(KERN_INFO fmt, ##args);
#define DEBUG_EN 1
#else
#define PRINT(fmt, args...)
#define ENTER_FUNC2()
#define EXIT_FUNC2()
#define ENTER_FUNC()
#define EXIT_FUNC()
#define DEBUG_EN 0
#endif

#define GPIO_HIGH_VALUE 1
#define GPIO_LOW_VALUE  0

#define NOP()	do {asm volatile ("NOP");} while(0);
#define DELAY_3NS() do { \
    asm volatile ("NOP"); \
    asm volatile ("NOP"); \
    asm volatile ("NOP");} while(0);


#ifdef CONFIG_PANTECH_EF65L_BOARD
#ifdef GPIO_I2C_LED_CONTROL
const uint8 MAX8831_I2C_ID = 0x9A;
const uint8 MAX8831_I2C_DELAY = 5;
#endif /*GPIO_I2C_LED_CONTROL*/

//#define FEATURE_SKY_BACKLIGHT_MAX8831
#define FEATURE_SKY_BACKLIGHT_AAT1403 

#ifdef FEATURE_SKY_BACKLIGHT_MAX8831
#define MAX8831_ONOFF_CNTL  0x00
#define MAX8831_LED1_RAMP_CNTL  0x03
#define MAX8831_LED2_RAMP_CNTL  0x04
#define MAX8831_ILED1_CNTL  0x0B
#define MAX8831_ILED2_CNTL  0x0C
#define LCD_BL_SCL     21
#define LCD_BL_SDA     22
#endif /*FEATURE_SKY_BACKLIGHT_MAX8831*/

#ifdef FEATURE_SKY_BACKLIGHT_AAT1403
#define LCD_BL_EN      21
#define BL_MAX         32
#endif

#else/*CONFIG_PANTECH_EF65_BOARD*/
#define LCD_BL_EN 		13
#endif/*CONFIG_PANTECH_EF65_BOARD*/


#define LCD_RESET		50
//#define LCD_BL_EN		13
#define LCD_POWER		49
#define LCD_DET			130

/*
 * 2011.11.11 lived
 * TP30: comment out this define
 * ~TP20: define this define
 * 2011.11.20 lived
 * TP20->TP10
 */
#if 0//(BOARD_REV <= TP10)
#define USE_INITIAL_SEQ
#endif

static struct msm_panel_common_pdata *mipi_sony_pdata;

static struct dsi_buf sony_tx_buf;
static struct dsi_buf sony_rx_buf;
#ifdef CONFIG_PANTECH_EF65L_BOARD
static uint32_t lcd_gpio_init_table[] = {
	GPIO_CFG(LCD_RESET, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(LCD_POWER, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
#ifdef 	FEATURE_SKY_BACKLIGHT_MAX8831
	GPIO_CFG(LCD_BL_SCL, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(LCD_BL_SDA, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
#endif
#ifdef 	FEATURE_SKY_BACKLIGHT_AAT1403
	GPIO_CFG(LCD_BL_EN, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
#endif
};
#else
static uint32_t lcd_gpio_init_table[] = {
	GPIO_CFG(LCD_RESET, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(LCD_BL_EN, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(LCD_DET, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};
#endif
struct lcd_state_type {
    boolean disp_powered_up;
    boolean disp_initialized;
    boolean disp_on;
};

static struct lcd_state_type sony_state = { 0, };

static void lcd_gpio_init(uint32_t *table, int len, unsigned enable)
{
	int n, rc;
	for (n = 0; n < len; n++) {
		rc = gpio_tlmm_config(table[n],
				enable ? GPIO_CFG_ENABLE : GPIO_CFG_DISABLE);
		if (rc) {
			printk(KERN_ERR "%s: gpio_tlmm_config(%#x)=%d\n",
					__func__, table[n], rc);
			break;
		}
	}
}

#ifdef FEATURE_SKY_BACKLIGHT_MAX8831
static struct i2c_client *led_i2c_client = NULL;

static int led_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int rc = 0;

	ENTER_FUNC2(); 
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		led_i2c_client = NULL;
		rc = -1;
		if(DEBUG_EN) printk(KERN_ERR "[led_i2c_probe] failed!!!\n");
	}
	else
	{
		led_i2c_client = client;
	}
	EXIT_FUNC2();
	if(DEBUG_EN) printk(KERN_ERR "[led_i2c_probe] successed!!!\n");

	return rc;
}


static int led_i2c_remove(struct i2c_client *client)
{
	led_i2c_client = NULL;

	if(DEBUG_EN) printk(KERN_ERR "[led_i2c_probe] removed!!!\n");
	return 0;
}

static const struct i2c_device_id led_id[] = {
	{ "MAX8831", 0 },  { }
};

static struct i2c_driver led_i2c_driver = {
	.driver = {
		.name = "MAX8831",
		.owner = THIS_MODULE,
	},
	.probe = led_i2c_probe,
	.remove = __devexit_p(led_i2c_remove),
	.id_table = led_id,
};

static void led_i2c_api_Init(void)
{
	int result = 0;

	ENTER_FUNC2();
	result = i2c_add_driver(&led_i2c_driver);
	if(result){
		if(DEBUG_EN) printk(KERN_ERR "[led_i2c_api_Init] error!!!\n");
	}
	EXIT_FUNC2();
}

#ifndef GPIO_I2C_LED_CONTROL
static uint8 led_i2c_master_send(uint8 reg, uint8 data)
{
	static int ret = 0;
	unsigned char buf[4];
	struct i2c_msg msg[2];

	if(!led_i2c_client)
	{
		if(DEBUG_EN) printk(KERN_ERR "[led_i2c_master_send] led_i2c_client is null!!!\n");
			return -1;
	}
	buf[0] = reg;
	buf[1] = data;

	msg[0].addr = led_i2c_client->addr;  
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = buf;

	ret = i2c_transfer(led_i2c_client->adapter, msg, 1);
	if (ret < 0)
	{
		if(DEBUG_EN) printk(KERN_ERR "[led_i2c_master_send] write error!!!\n");
			return FALSE;
	}
	else
	{
		return TRUE;
	}
}
#endif

#if defined(GPIO_I2C_LED_CONTROL)
#define LCD_BL_SDA      22//72
#define LCD_BL_SCL      21//73

#if 0
static uint32_t backlight_gpio_init_table[] = {
	GPIO_CFG(LCD_BL_SDA, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(LCD_BL_SCL, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};
#endif
static void sda_write (unsigned char onoff)
{
	if(onoff)
	{
		gpio_set_value(LCD_BL_SDA, GPIO_HIGH_VALUE);
	}
	else
	{
		gpio_set_value(LCD_BL_SDA, GPIO_LOW_VALUE);
	}
} 

static void scl_write (unsigned char onoff)
{
	if(onoff)
	{
		gpio_set_value(LCD_BL_SCL, GPIO_HIGH_VALUE);
	}
	else
	{
		gpio_set_value(LCD_BL_SCL, GPIO_LOW_VALUE);
	}
}

static void SCL_DELAY (void)
{
	clk_busy_wait(MAX8831_I2C_DELAY);
}

static void i2c_wait (void)
{
	clk_busy_wait(MAX8831_I2C_DELAY);
}


static void i2c_start_condition (void)
{
	sda_write (1);
	scl_write (1);  
	sda_write(0);
	SCL_DELAY();
	scl_write (0);
	i2c_wait();
}

static void i2c_stop_condition (void)
{
	sda_write (0);
	scl_write(1);
	SCL_DELAY();
	sda_write (1);
	i2c_wait();  
}

static void i2c_ack (void)
{
	sda_write (0);
	scl_write (1);
	SCL_DELAY();
	scl_write(0);
	i2c_wait();
}

static void i2c_write_byte(unsigned char data)
{
	signed char  bit_idx;

	for (bit_idx = 7; bit_idx >= 0; bit_idx--)
	{
		sda_write (data>>bit_idx & 0x01);
		SCL_DELAY();
		scl_write (1);
		SCL_DELAY();
		scl_write (0);
		//BOOT_SCL_DELAY();
	}
	//boot_i2c_ack();
}

static void led_gpio_i2c_set(uint8 reg, uint8 data)
{
	i2c_start_condition();
	i2c_write_byte(MAX8831_I2C_ID);
	i2c_ack();
	i2c_write_byte(reg);
	i2c_ack();
	i2c_write_byte(data);
	i2c_ack();
	i2c_stop_condition();
}

//static int once_backlight_on=0;
#endif

#if (0)
static void backlight_off(void)
{
	ENTER_FUNC2();   
	led_i2c_master_send(MAX8831_ONOFF_CNTL, 0x00);     
	mdelay(5);
	//gpio_set_value(LCD_BL_EN ,GPIO_LOW_VALUE);
	flag_lcd_bl_off = TRUE;
	EXIT_FUNC2();  
}
#endif
#endif /*FEATURE_SKY_BACLIGHT_SC620*/
#ifdef USE_INITIAL_SEQ
/* Initial sequence for SONY LCD */
char passwd1[3]     = {0xF0, 0x5A, 0x5A};
char passwd2[3]     = {0xF1, 0x5A, 0x5A};

#if defined (MIPI_CLOCK_400MBPS)
char id1wr[2]       = {0xB1, 0x53};
char id2wr[2]       = {0xB2, 0xa9};
char id3wr[2]       = {0xB3, 0x60};
#elif defined (MIPI_CLOCK_250MBPS) || defined (MIPI_CLOCK_450MBPS) || defined (MIPI_CLOCK_500MBPS)
char id1wr[2]       = {0xB1, 0x00};
char id2wr[2]       = {0xB2, 0x00};
char id3wr[2]       = {0xB3, 0x00};
#endif

#if defined (MIPI_CLOCK_400MBPS) || defined (MIPI_CLOCK_450MBPS) || defined (MIPI_CLOCK_500MBPS)
char miectl_1[4]    = {0xC0, 0x80, 0x80, 0x10};
#elif defined (MIPI_CLOCK_250MBPS)
char miectl_1[4]    = {0xC0, 0x80, 0x90, 0x10};
#endif

char bcmode[2]      = {0xC1, 0x01};
char miectl_2[2]    = {0xC2, 0x08};

char wrblctl[4]     = {0xC3, 0x00, 0x00, 0x20};
#if defined (MIPI_CLOCK_400MBPS) || defined (MIPI_CLOCK_450MBPS) || defined (MIPI_CLOCK_500MBPS)
char source_ctl[5]  = {0xF2, 0x03, 0x33, 0xA1, 0x85};
char power_ctl[13]	= {0xF4, 0x0A, 0x0B, 0x07, 0x07, 0x10, 0x14, 0x0D, 0x0C, 0xAD, 0x00, 0x33, 0x33};
#elif defined (MIPI_CLOCK_250MBPS)
char source_ctl[4]	= {0xF2, 0x03, 0x33, 0x81};
char power_ctl[13]	= {0xF4, 0x0A, 0x0B, 0x07, 0x07, 0x20, 0x14, 0x0D, 0x0C, 0xB9, 0x00, 0x33, 0x33};
#endif

#if defined (MIPI_CLOCK_400MBPS)
char panel_ctl[21]	= {0xF6, 0x02, 0x0F, 0x0E, 0x22, 0x09, 0x00, 0x11, 0x1F, 0x18, 0x03,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x02, 0x09, 0x10, 0x51};
#elif defined (MIPI_CLOCK_450MBPS)
char panel_ctl[21]	= {0xF6, 0x02, 0x0E, 0x0C, 0x1E, 0x08, 0x00, 0x0F, 0x1B, 0x16, 0x03,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x02, 0x09, 0x10, 0x51};
#elif defined (MIPI_CLOCK_500MBPS)
char panel_ctl[21]	= {0xF6, 0x02, 0x11, 0x0F, 0x25, 0x0A, 0x00, 0x13, 0x22, 0x1B, 0x03,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x02, 0x09, 0x10, 0x51};
#elif defined (MIPI_CLOCK_250MBPS)
char panel_ctl[21]	= {0xF6, 0x02, 0x11, 0x0F, 0x25, 0x0A, 0x00, 0x13, 0x22, 0x1B, 0x03,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x08, 0x0C, 0x08, 0x51};
#endif

#if 0
char positive_gamma[64]= {0xFA, 0x00, 0x3F, 0x28, 0x27, 0x33, 0x32, 0x2B, 0x2A, 0x2B, 0x27,
	0x23, 0x28, 0x2B, 0x2E, 0x32, 0x37, 0x3C, 0x3F, 0x3F, 0x3F,
	0x26, 0x00, 0x3F, 0x29, 0x27, 0x34, 0x34, 0x2D, 0x2C, 0x2C,
	0x28, 0x24, 0x28, 0x2B, 0x2E, 0x32, 0x36, 0x3A, 0x3C, 0x3A,
	0x3C, 0x1E, 0x00, 0x3F, 0x28, 0x27, 0x33, 0x32, 0x2B, 0x2A,
	0x2B, 0x27, 0x23, 0x28, 0x2B, 0x2E, 0x32, 0x36, 0x3A, 0x3D,
	0x3C, 0x3E, 0x1F};
char negative_gamma[64]= {0xFB, 0x00, 0x3F, 0x28, 0x27, 0x33, 0x32, 0x2B, 0x2A, 0x2B, 0x27,
	0x23, 0x28, 0x2B, 0x2E, 0x32, 0x37, 0x3C, 0x3F, 0x3F, 0x3F,
	0x26, 0x00, 0x3F, 0x29, 0x27, 0x34, 0x34, 0x2D, 0x2C, 0x2C,
	0x28, 0x24, 0x28, 0x2B, 0x2E, 0x32, 0x36, 0x3A, 0x3C, 0x3A,
	0x3C, 0x1E, 0x00, 0x3F, 0x28, 0x27, 0x33, 0x32, 0x2B, 0x2A,
	0x2B, 0x27, 0x23, 0x28, 0x2B, 0x2E, 0x32, 0x36, 0x3A, 0x3D,
	0x3C, 0x3E, 0x1F};
#endif

char amp_type_1[3]     = {0xFC, 0x5A, 0x5A};
#if defined (MIPI_CLOCK_400MBPS) || defined (MIPI_CLOCK_450MBPS) || defined (MIPI_CLOCK_500MBPS)
char amp_type_2[4]     = {0xF5, 0x5A, 0x55, 0x38};
#elif defined (MIPI_CLOCK_250MBPS)
char amp_type_2[5]     = {0xF5, 0x5A, 0x55, 0x38, 0x00};
#endif

#endif // end of USE_INITIAL_SEQ

char sleep_out[2]   = {0x11, 0x00};
#ifdef CONFIG_PANTECH_EF65L_BOARD
char display_ctl[2] = {0x36, 0x80};
#else
char display_ctl[2]  = {0x36, 0x40};
#endif
char disp_on[2]     = {0x29, 0x00};
char sleep_in[2]    = {0x10, 0x00};
char disp_off[2]    = {0x28, 0x00};

static struct dsi_cmd_desc sony_display_off_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 0, sizeof(disp_off), disp_off},
	{DTYPE_DCS_WRITE, 1, 0, 0, 120, sizeof(sleep_in), sleep_in}
};

static struct dsi_cmd_desc sony_display_on_cmds[] = {
#if defined (MIPI_CLOCK_250MBPS) || !defined(USE_INITIAL_SEQ)
	{DTYPE_DCS_WRITE, 1, 0, 0, 140, sizeof(sleep_out), sleep_out},
#endif
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(display_ctl), display_ctl},
	{DTYPE_DCS_WRITE, 1, 0, 0, 0, sizeof(disp_on), disp_on}
};

#ifdef USE_INITIAL_SEQ
static struct dsi_cmd_desc sony_display_init_cmds[] = {
#if !defined (MIPI_CLOCK_250MBPS)
    {DTYPE_DCS_WRITE, 1, 0, 0, 140, sizeof(sleep_out), sleep_out},
#endif
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(passwd1), passwd1},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(passwd2), passwd2},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(id1wr), id1wr},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(id2wr), id2wr},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(id3wr), id3wr},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(miectl_1), miectl_1},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(bcmode), bcmode},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(miectl_2), miectl_2},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(wrblctl), wrblctl},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(source_ctl), source_ctl},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(power_ctl), power_ctl},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(panel_ctl), panel_ctl},
#if 0
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(positive_gamma), positive_gamma},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(negative_gamma), negative_gamma},
#endif
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(amp_type_1), amp_type_1},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 10, sizeof(amp_type_2), amp_type_2},
};
#endif

/*
 0. sony_display_init_cmds
 1. sony_display_veil_init0_cmds
 2. sony_display_veil_lut_cmds
 3. sony_display_veil_init1_cmds
 4. sony_display_veil_tex_cmds
 5. sony_display_veil_colormap_cmds
 6. sony_display_veil_init2_cmds
 7. dsi_cmd_desc sony_display_on_cmds
 */

static int mipi_sony_lcd_on(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;

    ENTER_FUNC2();

	mfd = platform_get_drvdata(pdev);

	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;

	//mutex_lock(&mfd->dma->ov_mutex);
	if (sony_state.disp_initialized == false) {
		//PRINT("[LIVED] LCD RESET!!\n");
		gpio_set_value(LCD_RESET, GPIO_LOW_VALUE);
		usleep(10);//msleep(1);
		gpio_set_value(LCD_RESET, GPIO_HIGH_VALUE);
		usleep(10);//msleep(120);
#ifdef USE_INITIAL_SEQ
		mipi_dsi_cmds_tx(&sony_tx_buf, sony_display_init_cmds,
				ARRAY_SIZE(sony_display_init_cmds));
#endif
		sony_state.disp_initialized = true;
	}
	mipi_dsi_cmds_tx(&sony_tx_buf, sony_display_on_cmds,
			ARRAY_SIZE(sony_display_on_cmds));
	sony_state.disp_on = true;
	//mutex_unlock(&mfd->dma->ov_mutex);

 	EXIT_FUNC2();
	return 0;
}

#if 0
int mipi_samsung_lcd_off_running=0;
#endif
static int mipi_sony_lcd_off(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;

    ENTER_FUNC2();

	mfd = platform_get_drvdata(pdev);

	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;

    //mutex_lock(&mfd->dma->ov_mutex); 
	if (sony_state.disp_on == true) {
		gpio_set_value(LCD_RESET, GPIO_LOW_VALUE);
		usleep(10);//msleep(1);
		gpio_set_value(LCD_RESET, GPIO_HIGH_VALUE);
		usleep(10);//msleep(120);

#if 0
		mipi_samsung_lcd_off_running=1;
#endif
		mipi_dsi_cmds_tx(&sony_tx_buf, sony_display_off_cmds,
				ARRAY_SIZE(sony_display_off_cmds));
#if 0
		mipi_samsung_lcd_off_running=0;
#endif
		sony_state.disp_on = false;
		sony_state.disp_initialized = false;
	}
    //mutex_unlock(&mfd->dma->ov_mutex);    

    EXIT_FUNC2();
	return 0;
}

#ifndef CONFIG_PANTECH_EF65L_BOARD
static int first_enable = 1;
#endif
#if 0//(BOARD_REV < WS20)
static int prev_bl_level = 8;
#else
static int prev_bl_level = 10;
#endif

void mipi_sony_set_prev_bl_level(int bl_level)
{
	    prev_bl_level = bl_level;
}

#ifdef CONFIG_PANTECH_EF65L_BOARD
static void mipi_sony_set_backlight(struct msm_fb_data_type *mfd)
{
#ifdef FEATURE_SKY_BACKLIGHT_MAX8831
#ifndef GPIO_I2C_LED_CONTROL
	boolean ret = TRUE;
#endif
#ifdef GPIO_I2C_LED_CONTROL
	if (mfd->bl_level ==0) {
		led_gpio_i2c_set(0x00, 0x00);
	} else {
		//lcd_gpio_init(backlight_gpio_init_table, ARRAY_SIZE(backlight_gpio_init_table), 1);
		//gpio_set_value(LCD_BL_SDA, GPIO_LOW_VALUE);
		//gpio_set_value(LCD_BL_SCL, GPIO_LOW_VALUE);   
		led_gpio_i2c_set(0x00, 0x03);
		led_gpio_i2c_set(0x0B, mfd->bl_level);
		led_gpio_i2c_set(0x0C, mfd->bl_level);
		//gpio_free(LCD_BL_SDA);
		//gpio_free(LCD_BL_SCL);
	}
#else
	if (mfd->bl_level == 0) {
	/* ON/OFF Control Register*/
		ret = led_i2c_master_send(MAX8831_ONOFF_CNTL, 0x00); 
		if (ret == 0)   printk(KERN_ERR "[Error : led_i2c_master_send]l!!!\n");
		first_enable = 0;
	} else {
		if (first_enable == 0) {
		/* ON/OFF Control Register*/
			ret = led_i2c_master_send(MAX8831_ONOFF_CNTL, 0x03); 
			if (ret == 0)   printk(KERN_ERR "[Error : led_i2c_master_send]l!!!\n");
			first_enable = 1;
	}

	/* LED_1 Current Control Register*/ 
	//ret = led_i2c_master_send(MAX8831_ILED1_CNTL, mfd->bl_level); 
	//ret = led_i2c_master_send(MAX8831_ILED1_CNTL, 0x19); //3.05
	//ret = led_i2c_master_send(MAX8831_ILED1_CNTL, 0x1f); //4.0
	//ret = led_i2c_master_send(MAX8831_ILED1_CNTL, 0x25); //5.05
	//ret = led_i2c_master_send(MAX8831_ILED1_CNTL, 0x2B); //6.15
	ret = led_i2c_master_send(MAX8831_ILED1_CNTL, 0x32); //7.45
	if (ret == 0)   printk(KERN_ERR "[Error : led_i2c_master_send]l!!!\n");

	/* LED_2 Current Control Register*/
	//ret = led_i2c_master_send(MAX8831_ILED2_CNTL, mfd->bl_level); 
	//ret = led_i2c_master_send(MAX8831_ILED1_CNTL, 0x19); //3.05
	//ret = led_i2c_master_send(MAX8831_ILED1_CNTL, 0x1f); //4.0
	//ret = led_i2c_master_send(MAX8831_ILED1_CNTL, 0x25); //5.05
	//ret = led_i2c_master_send(MAX8831_ILED1_CNTL, 0x2B); //6.15
	ret = led_i2c_master_send(MAX8831_ILED1_CNTL, 0x32); //7.45
	if (ret == 0)   printk(KERN_ERR "[Error : led_i2c_master_send]l!!!\n");
}
	//printk(KERN_INFO "[SKY_LCD] -%s:%d\n", __FUNCTION__, mfd->bl_level);       

#endif /*GPIO_I2C_LED_CONTROL*/	   
#elif defined(FEATURE_SKY_BACKLIGHT_AAT1403)
	//static int first_enable = 0;
	static int prev_bl_level = 0;
	int cnt, bl_level;
	//int count = 0;
	unsigned long flags;
	bl_level = mfd->bl_level;

    PRINT("%s() bl_level %d prev_bl_level %d\n", __func__, mfd->bl_level, prev_bl_level);

	if (bl_level == prev_bl_level || sony_state.disp_on == 0) {
		PRINT("[LIVED] same! or not disp_on\n");
	} else {
		if (bl_level == 0) {
			gpio_set_value(LCD_BL_EN ,GPIO_LOW_VALUE);
			usleep(500);
    	} else {
    		cnt = BL_MAX - bl_level;

//PRINT("[LIVED] prev_bl_level =%d, bl_level =%d, cnt=%d\n", prev_bl_level, bl_level, cnt);
        	do {
        		local_save_flags(flags);
        		local_irq_disable();
        		gpio_set_value(LCD_BL_EN ,GPIO_LOW_VALUE);
        		udelay(1);	// T LO
        		//count++;
        		gpio_set_value(LCD_BL_EN ,GPIO_HIGH_VALUE);
        		udelay(1);	// T HI
        		local_irq_restore(flags);
        	} while (cnt--);

	        udelay(500);      // latch

//PRINT("[LIVED] count=%d\n", count);
        }
        prev_bl_level = bl_level;
    }

#endif /*FEATURE_SKY_BACKLIGHT_MAX8831*/   
}

#else/**CONFIG_PANTECH_EF65L_BOARD*/
static void mipi_sony_set_backlight(struct msm_fb_data_type *mfd)
{
	int cnt, bl_level;
	//int count = 0;
	unsigned long flags;

    PRINT("%s() bl_level %d prev_bl_level %d\n", __func__, mfd->bl_level, prev_bl_level);

	bl_level = mfd->bl_level;

	if (bl_level == prev_bl_level || sony_state.disp_on == 0) {
		PRINT("[LIVED] same! or not disp_on\n");
	} else {
		if (bl_level == 0) {
			gpio_set_value(LCD_BL_EN, GPIO_LOW_VALUE);
			usleep(300);      // Disable hold time
			first_enable = 0;
		} else {
			if (prev_bl_level == 0) {
				//count++;
				gpio_set_value(LCD_BL_EN, GPIO_HIGH_VALUE);
				if (first_enable == 0) {
					first_enable = 1;
					local_save_flags(flags);
					local_irq_disable();
					udelay(10);	// T_EN
					mdelay(3);	// T_SS
					local_irq_restore(flags);
				} else {
					udelay(300);      // Turn on time
				}
				//PRINT("[LIVED] (0) init!\n");
			}
#if 0//(BOARD_REV < WS20)
			if (prev_bl_level < bl_level) {
				cnt = BL_MAX - bl_level;
				cnt += prev_bl_level;
			} else {
				cnt = prev_bl_level - bl_level;
			}
			//PRINT("[LIVED] cnt=%d\n", cnt);
			while (cnt) {
				local_save_flags(flags);
				local_irq_disable();
				gpio_set_value(LCD_BL_EN, GPIO_LOW_VALUE);
				DELAY_3NS();//udelay(3);      // Turn off time
				//count++;
				gpio_set_value(LCD_BL_EN, GPIO_HIGH_VALUE);
			    local_irq_restore(flags);
				udelay(10);      // Turn on time
				//PRINT("[LIVED] (2) cnt=%d!\n", cnt);
				cnt--;
			}
			//PRINT("[LIVED] count=%d\n", count);
#else
			if (prev_bl_level < bl_level) {
				cnt = BL_MAX - bl_level;
				cnt += prev_bl_level;
				if (cnt > (BL_MAX - bl_level + 6)) {
					gpio_set_value(LCD_BL_EN, GPIO_LOW_VALUE);
					local_save_flags(flags);
					local_irq_disable();
					udelay(50);// TDIS
					local_irq_restore(flags);
					cnt = BL_MAX - bl_level + 1;
				}
			} else {
				cnt = prev_bl_level - bl_level;
			}
			//pr_info("[LIVED] cnt=%d, prev_bl_level=%d, bl_level=%d\n",
			//		cnt, prev_bl_level, bl_level);
			while (cnt) {
				local_save_flags(flags);
				local_irq_disable();
				gpio_set_value(LCD_BL_EN, GPIO_LOW_VALUE);
				DELAY_3NS();//udelay(3);      // Turn off time
				gpio_set_value(LCD_BL_EN, GPIO_HIGH_VALUE);
				local_irq_restore(flags);
				udelay(10);      // Turn on time
				cnt--;
			}
#endif
		}
		prev_bl_level = bl_level;
	}
}
#endif/*CONFIG_PANTECH_EF65L_BOARD*/

static int __devinit mipi_sony_lcd_probe(struct platform_device *pdev)
{
    if (pdev->id == 0) {
        mipi_sony_pdata = pdev->dev.platform_data;
		return 0;
	}

	msm_fb_add_device(pdev);

	return 0;
}

static struct platform_driver this_driver = {
	.probe  = mipi_sony_lcd_probe,
	.driver = {
		.name   = "mipi_sony",
	},
};

static struct msm_fb_panel_data sony_panel_data = {
       .on             = mipi_sony_lcd_on,
       .off            = mipi_sony_lcd_off,
       .set_backlight  = mipi_sony_set_backlight,
};

static int ch_used[3];

int mipi_sony_device_register(struct msm_panel_info *pinfo,
					u32 channel, u32 panel)
{
	struct platform_device *pdev = NULL;
	int ret;

	if ((channel >= 3) || ch_used[channel])
		return -ENODEV;

	ch_used[channel] = TRUE;

	pdev = platform_device_alloc("mipi_sony", (panel << 8)|channel);
	if (!pdev)
		return -ENOMEM;

	sony_panel_data.panel_info = *pinfo;

	ret = platform_device_add_data(pdev, &sony_panel_data,
		sizeof(sony_panel_data));
	if (ret) {
		printk(KERN_ERR
		  "%s: platform_device_add_data failed!\n", __func__);
		goto err_device_put;
	}

	ret = platform_device_add(pdev);
	if (ret) {
		printk(KERN_ERR
		  "%s: platform_device_register failed!\n", __func__);
		goto err_device_put;
	}

	return 0;

err_device_put:
	platform_device_put(pdev);
	return ret;
}

#ifdef CONFIG_SW_RESET
extern int msm_reset_reason_read_only(void);
#endif
static int __init mipi_sony_lcd_init(void)
{
    ENTER_FUNC2();

    lcd_gpio_init(lcd_gpio_init_table, ARRAY_SIZE(lcd_gpio_init_table), 1);

#ifdef FEATURE_SKY_BACKLIGHT_MAX8831
    led_i2c_api_Init();
#endif

	gpio_set_value(LCD_DET, GPIO_LOW_VALUE);
    sony_state.disp_powered_up = true;

    mipi_dsi_buf_alloc(&sony_tx_buf, DSI_BUF_SIZE);
    mipi_dsi_buf_alloc(&sony_rx_buf, DSI_BUF_SIZE);

#if 0//def CONFIG_SW_RESET
	if (msm_reset_reason_read_only()) {
		first_enable = 0;
		prev_bl_level = 0;
	}
#endif

    EXIT_FUNC2();

    return platform_driver_register(&this_driver);
}

module_init(mipi_sony_lcd_init);

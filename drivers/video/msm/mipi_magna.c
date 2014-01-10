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
#include "mipi_magna.h"
#include <mach/gpio.h>
#include <asm/irq.h>
#include <asm/system.h>

#ifdef CONFIG_PANTECH_LCD_VEIL_VIEW
#include "veil_view_cmds.h"
#endif
#ifdef F_SKYDISP_LCD_GAMMA_TEST
#include "gamma.h"
#endif

#if 0 // 130527 p14198 ADDED 
void gpio_set_value(unsigned gpio, int value);
#endif

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

#define LCD_RESET       94
#define LCD_BL_EN      140
#define BL_MAX          16

static struct msm_panel_common_pdata *mipi_magna_pdata;

static struct dsi_buf magna_tx_buf;
static struct dsi_buf magna_rx_buf;

static uint32_t lcd_gpio_init_table[] = {
	GPIO_CFG(LCD_RESET, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	//GPIO_CFG(LCD_BL_EN, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(LCD_BL_EN, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
};

struct lcd_state_type {
	boolean disp_powered_up;
	boolean disp_initialized;
	boolean disp_on;
#ifdef CONFIG_PANTECH_LCD_VEIL_VIEW
	boolean disp_veil_initialized;
	boolean disp_veil_enable;
#endif
#ifdef F_SKYDISP_LCD_GAMMA_TEST
	int gamma;
#endif
};

static struct lcd_state_type magna_state = { 0, };
#ifdef CONFIG_PANTECH_LCD_VEIL_VIEW
struct msmfb_veil_view veil_state = {0, };
#endif

#ifdef CONFIG_SKY_CHARGING
extern void msm_charger_set_lcd_onoff(unsigned int onff);
#endif

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

char sleep_out[2]   = {0x11, 0x00};
char bank0[2]       = {0xB0, 0x00};
//char trvcolmap6[11] = {0xE6, 0x12, 0x04, 0x00, 0x05, 0x07, 0x03, 0xFF, 0x10, 0xFF, 0xFF};
char trvcolmap6[11] = {0xE6, 0x52, 0x05, 0x02, 0x05, 0x07, 0x03, 0xFF, 0x10, 0xFF, 0xFF};
char bank2[2]       = {0xB0, 0x02};
char datah_dly[2]   = {0xC6, 0x05};
char disp_on[2]     = {0x29, 0x00};
char disp_off[2]    = {0x28, 0x00};
char sleep_in[2]    = {0x10, 0x00};
char deep_stb[2]    = {0x70, 0x01};

static struct dsi_cmd_desc magna_display_off_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 0, sizeof(disp_off), disp_off},
	{DTYPE_DCS_WRITE, 1, 0, 0, 120, sizeof(sleep_in), sleep_in},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 120,sizeof(deep_stb), deep_stb},
};

static struct dsi_cmd_desc magna_display_init_cmds[] = {
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(bank0), bank0},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(trvcolmap6), trvcolmap6},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(bank2), bank2},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(datah_dly), datah_dly},
};

static struct dsi_cmd_desc magna_sleep_out_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 120, sizeof(sleep_out), sleep_out},
};

static struct dsi_cmd_desc magna_display_on_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 0, sizeof(disp_on), disp_on}
};

/*
   0. magna_display_init_cmds
   1. magna_display_veil_init0_cmds
   2. magna_display_veil_lut_cmds
   3. magna_display_veil_init1_cmds
   4. magna_display_veil_tex_cmds
   5. magna_display_veil_colormap_cmds
   6. magna_display_veil_init2_cmds
   7. dsi_cmd_desc magna_display_on_cmds
   */

static int mipi_magna_lcd_on(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;
    ENTER_FUNC2();
    
	mfd = platform_get_drvdata(pdev);

	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;

	//mutex_lock(&mfd->dma->ov_mutex);

	if (magna_state.disp_initialized == false) {
		printk(KERN_INFO"[LIVED] LCD RESET!!\n");
		gpio_set_value(LCD_RESET, GPIO_LOW_VALUE);
		//usleep(50);
		msleep(1);
		gpio_set_value(LCD_RESET, GPIO_HIGH_VALUE);
		msleep(5);

		mipi_dsi_cmds_tx(&magna_tx_buf, magna_display_init_cmds,
				ARRAY_SIZE(magna_display_init_cmds));

		magna_state.disp_initialized = true;
	}

#ifdef CONFIG_PANTECH_LCD_VEIL_VIEW
	if (magna_state.disp_veil_enable == true) {
		MIPI_OUTP(MIPI_DSI_BASE + 0x38, 0x10000000);    // High Speed Mode
		printk(KERN_INFO"[LIVED] Veil View Setting!\n");
		mipi_dsi_cmds_tx(&magna_tx_buf, magna_display_veil_init0_cmds,
				ARRAY_SIZE(magna_display_veil_init0_cmds));
		mipi_dsi_cmds_tx(&magna_tx_buf, magna_display_veil_lut_cmds[veil_state.lut],
				ARRAY_SIZE(magna_display_veil_lut_cmds[veil_state.lut]));
		mipi_dsi_cmds_tx(&magna_tx_buf, magna_display_veil_init1_cmds,
				ARRAY_SIZE(magna_display_veil_init1_cmds));
		mipi_dsi_cmds_tx(&magna_tx_buf, magna_display_veil_tex_cmds[veil_state.tex],
				ARRAY_SIZE(magna_display_veil_tex_cmds[veil_state.tex]));
		mipi_dsi_cmds_tx(&magna_tx_buf, magna_display_veil_colormap_cmds[veil_state.map],
				ARRAY_SIZE(magna_display_veil_colormap_cmds[veil_state.map]));
		mipi_dsi_cmds_tx(&magna_tx_buf, magna_display_veil_init2_cmds,
				ARRAY_SIZE(magna_display_veil_init2_cmds));
		mipi_dsi_cmds_tx(&magna_tx_buf, magna_display_veil_on_cmds,
				ARRAY_SIZE(magna_display_veil_on_cmds));
		MIPI_OUTP(MIPI_DSI_BASE + 0x38, 0x14000000);    // Low Power Mode
	}
#endif
#ifdef F_SKYDISP_LCD_GAMMA_TEST
	if (magna_state.disp_veil_enable == false && magna_state.gamma != 0) {
		/* when Veil view enable, use Gamma 2.2 (default) */
		mipi_dsi_cmds_tx(&magna_tx_buf, magna_otp_non_load_cmds,
				ARRAY_SIZE(magna_otp_non_load_cmds));
    }
#endif
	if (magna_state.disp_on == false) {
		mipi_dsi_cmds_tx(&magna_tx_buf, magna_sleep_out_cmds,
				ARRAY_SIZE(magna_sleep_out_cmds));
#ifdef F_SKYDISP_LCD_GAMMA_TEST
		if (magna_state.disp_veil_enable == false && magna_state.gamma != 0) {
			/* when Veil view enable, use Gamma 2.2 (default) */
			printk(KERN_INFO"[LIVED] GAMMA =%d\n", magna_state.gamma);

			mipi_dsi_cmds_tx(&magna_tx_buf, magna_otp_non_init_cmds,
					ARRAY_SIZE(magna_otp_non_init_cmds));

			mipi_dsi_cmds_tx(&magna_tx_buf, magna_display_gamma_cmds[magna_state.gamma],
					ARRAY_SIZE(magna_display_gamma_cmds[magna_state.gamma]));
		}
#endif
		mipi_dsi_cmds_tx(&magna_tx_buf, magna_display_on_cmds,
				ARRAY_SIZE(magna_display_on_cmds));

		magna_state.disp_on = true;
#ifdef CONFIG_SKY_CHARGING
        msm_charger_set_lcd_onoff(true);
#endif
	}
	//mutex_unlock(&mfd->dma->ov_mutex);

    EXIT_FUNC2();
	return 0;
}

#if 0
int mipi_magna_lcd_off_running=0;
#endif
static int mipi_magna_lcd_off(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;
    ENTER_FUNC2();

	mfd = platform_get_drvdata(pdev);

	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;

	if (magna_state.disp_on == true) {
		gpio_set_value(LCD_RESET, GPIO_LOW_VALUE);
		msleep(1);
		//msleep(15);
		gpio_set_value(LCD_RESET, GPIO_HIGH_VALUE);
	    msleep(5);

		//mutex_lock(&mfd->dma->ov_mutex);
#if 0
		mipi_magna_lcd_off_running=1;
#endif
		mipi_dsi_cmds_tx(&magna_tx_buf, magna_display_off_cmds,
				ARRAY_SIZE(magna_display_off_cmds));
#if 0
		mipi_magna_lcd_off_running=0;
#endif
		//mutex_unlock(&mfd->dma->ov_mutex);
        //msleep(120);
		magna_state.disp_on = false;
#ifdef CONFIG_SKY_CHARGING
        msm_charger_set_lcd_onoff(false);
#endif
	}

#ifdef CONFIG_PANTECH_LCD_VEIL_VIEW
	// for command..
	//gpio_set_value(LCD_RESET, GPIO_LOW_VALUE);
	magna_state.disp_initialized = false;
#endif

    EXIT_FUNC2();
	return 0;
}

#ifdef CONFIG_PANTECH_LCD_VEIL_VIEW
void mipi_magna_veil_mode(boolean enable)
{
	printk(KERN_INFO"[LIVED] Veil View enable=%d\n", enable);
	magna_state.disp_veil_enable = enable;
}
EXPORT_SYMBOL(mipi_magna_veil_mode);

int mipi_magna_veil_set(struct msmfb_veil_view veil)
{
	//uint32_t i = 0;
	struct dsi_cmd_desc dsi_t;

	dsi_t.dtype = DTYPE_GEN_WRITE1;
	dsi_t.last = 1;
	dsi_t.vc = 0;
	dsi_t.ack = 0;
	dsi_t.wait = 0;

	if (veil.lut >= NUM_LUT || veil.tex >= NUM_TEX
			|| veil.map >= NUM_MAP || veil.size > MAX_SIZE 
			|| veil.rot > MAX_ROT ) {
		printk(KERN_ERR "[LIVED] invalid veil view argument\n");
		return -EINVAL;
	}

	printk(KERN_INFO"[LIVED] current lut=%d, tex=%d, map=%d, size=%d, rot=%d\n",
			veil_state.lut, veil_state.tex, veil_state.map, veil_state.size, veil_state.rot);
	printk(KERN_INFO"[LIVED] set lut=%d, tex=%d, map=%d, size=%d, rot=%d\n",
			veil.lut, veil.tex, veil.map, veil.size, veil.rot);

	if (veil.lut != veil_state.lut) {
		printk(KERN_INFO"[LIVED] lut=%d\n", veil.lut);
		veil_state.lut = veil.lut;
	}
	if (veil.tex != veil_state.tex) {
		printk(KERN_INFO"[LIVED] tex=%d\n", veil.tex);
		veil_state.tex = veil.tex;
	}
	if (veil.map != veil_state.map) {
		printk(KERN_INFO"[LIVED] map=%d\n", veil.map);
		veil_state.map = veil.map;
	}
	if (veil.size != veil_state.size || veil.rot != veil_state.rot) {
		printk(KERN_INFO"[LIVED] size=%d, rot=%d\n", veil.size, veil.rot);
		veil_init[NUM_ROT_SIZE][1] = ((veil.rot & 0x0F) << 4) | (veil.size & 0x0F);
		printk(KERN_INFO"[LIVED] size_rot=%x\n", veil_init[NUM_ROT_SIZE][1]);
		dsi_t.dlen = sizeof(veil_init[NUM_ROT_SIZE]);
		dsi_t.payload = veil_init[NUM_ROT_SIZE];
		magna_display_veil_init2_cmds[NUM_ROT_SIZE_IN_CMDS] = dsi_t;

		veil_state.size = veil.size;
		veil_state.rot= veil.rot;
	}

	return magna_state.disp_veil_enable;
}
EXPORT_SYMBOL(mipi_magna_veil_set);
#endif

#ifdef F_SKYDISP_LCD_GAMMA_TEST
void mipi_magna_gamma_set(int gamma)
{
	printk(KERN_INFO"[LIVED] gamma_set gamma=%d\n", gamma);
	if (gamma >= 0 && gamma < NUM_GAMMA)
		magna_state.gamma = gamma;
}
EXPORT_SYMBOL(mipi_magna_gamma_set);
#endif

static int first_enable = 1;
static int prev_bl_level = BL_MAX;

static void mipi_magna_set_backlight(struct msm_fb_data_type *mfd)
{
	int cnt, bl_level;
	int count = 0;
	unsigned long flags;
	bl_level = mfd->bl_level;

 	PRINT("%s() bl_level %d prev_bl_level %d\n", __func__, mfd->bl_level, prev_bl_level);

	if (bl_level == prev_bl_level || magna_state.disp_on == 0) {
		//PRINT("[LIVED] same! or not disp_on\n");
	} else {
		if (bl_level == 0) {
			gpio_set_value(LCD_BL_EN ,GPIO_LOW_VALUE);
			usleep(250);      // Disable hold time
		} else {
//			local_save_flags(flags); //2011.05.30 leecy  ������ �̵�..
//			local_irq_disable();
			if (prev_bl_level == 0) {
				count++;
				gpio_set_value(LCD_BL_EN ,GPIO_HIGH_VALUE);
				if (first_enable == 0) {
					first_enable = 1;
					msleep(25); // Initial enable time
				} else {
					udelay(300);      // Turn on time
				}
				//PRINT("[LIVED] (0) init!\n");
			}
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
				gpio_set_value(LCD_BL_EN ,GPIO_LOW_VALUE);
				DELAY_3NS();//udelay(3);      // Turn off time
				count++;
				gpio_set_value(LCD_BL_EN ,GPIO_HIGH_VALUE);
				local_irq_restore(flags);
				udelay(300);      // Turn on time
				//PRINT("[LIVED] (2) cnt=%d!\n", cnt);
				cnt--;
			}
			//PRINT("[LIVED] count=%d\n", count);
//			local_irq_restore(flags); //2011.05.30 leecy..
		}
		prev_bl_level = bl_level;
	}
}

static int __devinit mipi_magna_lcd_probe(struct platform_device *pdev)
{
	if (pdev->id == 0) {
		mipi_magna_pdata = pdev->dev.platform_data;
		return 0;
	}

	msm_fb_add_device(pdev);

	return 0;
}

static struct platform_driver this_driver = {
	.probe  = mipi_magna_lcd_probe,
	.driver = {
		.name   = "mipi_magna",
	},
};

static struct msm_fb_panel_data magna_panel_data = {
	.on             = mipi_magna_lcd_on,
	.off            = mipi_magna_lcd_off,
	.set_backlight  = mipi_magna_set_backlight,
};

static int ch_used[3];

int mipi_magna_device_register(struct msm_panel_info *pinfo,
		u32 channel, u32 panel)
{
	struct platform_device *pdev = NULL;
	int ret;

	if ((channel >= 3) || ch_used[channel])
		return -ENODEV;

	ch_used[channel] = TRUE;

	pdev = platform_device_alloc("mipi_magna", (panel << 8)|channel);
	if (!pdev)
		return -ENOMEM;

	magna_panel_data.panel_info = *pinfo;

	ret = platform_device_add_data(pdev, &magna_panel_data,
			sizeof(magna_panel_data));
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
extern int msm_reset_get_bl(void);
extern int msm_reset_reason_read_only(void);
#endif
static int __init mipi_magna_lcd_init(void)
{
	//ENTER_FUNC();

	lcd_gpio_init(lcd_gpio_init_table, ARRAY_SIZE(lcd_gpio_init_table), 1);

	magna_state.disp_powered_up = true;

	mipi_dsi_buf_alloc(&magna_tx_buf, DSI_BUF_SIZE);
	mipi_dsi_buf_alloc(&magna_rx_buf, DSI_BUF_SIZE);

#ifdef CONFIG_PANTECH_LCD_VEIL_VIEW
	veil_state.lut = 0;
	veil_state.tex = 0;
	veil_state.map = 0;
	veil_state.size = 1;
	veil_state.rot = 0;
#endif
#ifdef F_SKYDISP_LCD_GAMMA_TEST
	//magna_state.gamma = 0;
    //gamma_pattern_2(2.8)
	magna_state.gamma = 3;   
#endif
#ifdef CONFIG_SW_RESET
	if (msm_reset_reason_read_only()) {
		first_enable = 0;
		prev_bl_level = 0;
	}
#endif

 	//EXIT_FUNC();

	return platform_driver_register(&this_driver);
}

module_init(mipi_magna_lcd_init);

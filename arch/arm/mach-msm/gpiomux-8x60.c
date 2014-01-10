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
 */
#include <linux/module.h>
#include <mach/irqs.h>
#include <asm/mach-types.h>
#include <mach/gpiomux.h>
#include "gpiomux-8x60.h"

static struct gpiomux_setting console_uart = {
	.func = GPIOMUX_FUNC_2,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};

/* The SPI configurations apply to GSBI1 and GSBI10 */
static struct gpiomux_setting spi_active = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting spi_suspended_config = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting spi_suspended_cs_config = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};

#ifdef CONFIG_SKY_DMB_SPI_HW
static struct gpiomux_setting gsbi2 = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};
#endif

/* This I2C active configuration applies to GSBI3 and GSBI4 */
static struct gpiomux_setting i2c_active = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};
#if defined (CONFIG_INPUT_SENSOR)
#if defined(CONFIG_MACH_MSM8X60_EF39S) || defined(CONFIG_MACH_MSM8X60_EF40K) || defined(CONFIG_MACH_MSM8X60_EF40S) || defined(CONFIG_MACH_MSM8X60_EF65L)
#if defined(CONFIG_SENSORS_MPU3050)
static struct gpiomux_setting gsbi5 = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};
#endif
#endif
#endif

static struct gpiomux_setting i2c_active_gsbi7 = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_12MA,
	.pull = GPIOMUX_PULL_NONE,
};

/* This I2C suspended configuration applies to GSBI3, GSBI4 and GSBI7 */
static struct gpiomux_setting i2c_suspended_config = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting gsbi8 = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting ps_hold = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_12MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting msm_snddev_active_config = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting msm_snddev_suspend_config = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting ebi2_a_d = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_UP,
};

static struct gpiomux_setting ebi2_oe = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_UP,
};

static struct gpiomux_setting ebi2_we = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_UP,
};

#ifndef CONFIG_SKY_DMB_SPI_HW
static struct gpiomux_setting ebi2_cs2 = {
	.func = GPIOMUX_FUNC_2,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_UP,
};
#endif

static struct gpiomux_setting ebi2_cs3 = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_UP,
};

#if defined(CONFIG_USB_PEHCI_HCD) || defined(CONFIG_USB_PEHCI_HCD_MODULE)
static struct gpiomux_setting ebi2_cs4 = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_UP,
};
#endif

#if !(defined(CONFIG_PN544) && (defined(CONFIG_MACH_MSM8X60_EF39S) ||defined(CONFIG_MACH_MSM8X60_EF40S) ||defined(CONFIG_MACH_MSM8X60_EF40K)))
static struct gpiomux_setting ebi2_adv = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_UP,
};
#endif

#if defined(CONFIG_USB_PEHCI_HCD) || defined(CONFIG_USB_PEHCI_HCD_MODULE)
static struct gpiomux_setting usb_isp1763_actv_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting usb_isp1763_susp_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};
#endif

static struct gpiomux_setting sdcc1_dat_0_3_cmd_actv_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_10MA,
	.pull = GPIOMUX_PULL_UP,
};

static struct gpiomux_setting sdcc1_dat_4_7_cmd_actv_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_10MA,
	.pull = GPIOMUX_PULL_UP,
};

static struct gpiomux_setting sdcc1_clk_actv_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_16MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting sdcc1_suspend_config = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
};

static struct gpiomux_setting sdcc2_dat_0_3_cmd_actv_cfg = {
	.func = GPIOMUX_FUNC_2,
	.drv = GPIOMUX_DRV_10MA,
	.pull = GPIOMUX_PULL_UP,
};

static struct gpiomux_setting sdcc2_dat_4_7_cmd_actv_cfg = {
	.func = GPIOMUX_FUNC_2,
	.drv = GPIOMUX_DRV_10MA,
	.pull = GPIOMUX_PULL_UP,
};

static struct gpiomux_setting sdcc2_clk_actv_cfg = {
	.func = GPIOMUX_FUNC_2,
	.drv = GPIOMUX_DRV_16MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting sdcc2_suspend_config = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
};

static struct gpiomux_setting sdcc5_dat_0_3_cmd_actv_cfg = {
	.func = GPIOMUX_FUNC_2,
	.drv = GPIOMUX_DRV_10MA,
	.pull = GPIOMUX_PULL_UP,
};

static struct gpiomux_setting sdcc5_clk_actv_cfg = {
	.func = GPIOMUX_FUNC_2,
	.drv = GPIOMUX_DRV_16MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting sdcc5_suspend_config = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
};

static struct gpiomux_setting aux_pcm_active_config = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting aux_pcm_suspend_config = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting uart1dm_active = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting uart1dm_suspended = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting mi2s_active_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting mi2s_suspend_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting lcdc_active_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_16MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting lcdc_suspend_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting mdp_vsync_suspend_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting hdmi_suspend_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting mdm2ap_status_active_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting mdm2ap_status_suspend_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting cam_suspend_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting mdm2ap_sync_active_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting mdm2ap_sync_suspend_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting tm_active = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
};

static struct gpiomux_setting tm_suspended = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting tma_active = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_6MA,
	.pull = GPIOMUX_PULL_UP,
};

static struct gpiomux_setting ts_suspended = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting mdp_vsync_active_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting hdmi_active_1_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
};

static struct gpiomux_setting hdmi_active_2_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_16MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting hdmi_active_3_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_16MA,
	.pull = GPIOMUX_PULL_DOWN,
};

#ifdef CONFIG_PANTECH_FB_MSM_MHL_SII9244
#include "../../../drivers/video/msm/mhl_sii9244_driver.h"

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

/*static struct gpiomux_setting mhl_active_3_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};*/

static struct gpiomux_setting mhl_active_4_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
	.dir = GPIOMUX_IN,
};
#endif


static struct gpiomux_setting pmic_suspended_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};

#ifndef CONFIG_PANTECH_CAMERA_HW
static struct gpiomux_setting cam_active_1_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};
#endif

static struct gpiomux_setting cam_active_2_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting cam_active_3_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_UP,
};

#ifndef CONFIG_PANTECH_CAMERA_HW
static struct gpiomux_setting cam_active_4_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};
#endif

static struct gpiomux_setting cam_active_5_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_4MA,
	.pull = GPIOMUX_PULL_NONE,
};

#ifdef CONFIG_MSM_GSBI9_UART
static struct gpiomux_setting uart9dm_active = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_8MA ,
	.pull = GPIOMUX_PULL_DOWN,
};
#if defined (CONFIG_INPUT_SENSOR)
#if defined(CONFIG_MACH_MSM8X60_EF33S) || defined(CONFIG_MACH_MSM8X60_EF34K) || defined(CONFIG_MACH_MSM8X60_EF35L)
#if defined(CONFIG_SENSORS_MPU3050)
static struct gpiomux_setting gsbi9 = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};
#endif
#else
static struct gpiomux_setting gsbi9 = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};
#endif
#endif
#endif

#if defined(CONFIG_SKY_BATTERY_MAX17040) || defined(CONFIG_SKY_BATTERY_MAX17043)
static struct gpiomux_setting GSBI11 = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};
#endif

static struct gpiomux_setting ap2mdm_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting mdm2ap_status_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting mdm2ap_vfr_active_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_UP,
};

static struct gpiomux_setting mdm2ap_vfr_suspend_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting mdm2ap_errfatal_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_16MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting ap2mdm_kpdpwr_n_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};


#ifndef CONFIG_FB_MSM_MIPI_DSI_MAGNA
static struct gpiomux_setting mdm2ap_vddmin_active_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting mdm2ap_vddmin_suspend_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};
#endif

#if defined(CONFIG_PN544) 
static struct gpiomux_setting  PN544_suspend_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
//	.dir = GPIOMUX_OUT_LOW,
};
#endif

static struct msm_gpiomux_config msm8x60_gsbi_configs[] __initdata = {
	{
		.gpio      = 33,
		.settings = {
			[GPIOMUX_SUSPENDED] = &spi_suspended_config,
			[GPIOMUX_ACTIVE]    = &spi_active,
		},
	},
#if defined (CONFIG_INPUT_SENSOR) && (defined(CONFIG_MACH_MSM8X60_EF39S) || defined(CONFIG_MACH_MSM8X60_EF40K) || defined(CONFIG_MACH_MSM8X60_EF40S) || defined(CONFIG_MACH_MSM8X60_EF65L))
#if !defined(CONFIG_SENSORS_APDS9900)
	{
		.gpio      = 34,
		.settings = {
			[GPIOMUX_SUSPENDED] = &spi_suspended_config,
			[GPIOMUX_ACTIVE]    = &spi_active,
		},
	},
	{
		.gpio      = 35,
		.settings = {
			[GPIOMUX_SUSPENDED] = &spi_suspended_cs_config,
			[GPIOMUX_ACTIVE]    = &spi_active,
		},
	},
	{
		.gpio      = 36,
		.settings = {
			[GPIOMUX_SUSPENDED] = &spi_suspended_config,
			[GPIOMUX_ACTIVE]    = &spi_active,
		},
	},
#endif

#elif defined(CONFIG_PN544) && (defined(CONFIG_MACH_MSM8X60_EF33S) || defined(CONFIG_MACH_MSM8X60_EF34K) || defined(CONFIG_MACH_MSM8X60_EF34C))
	{
		.gpio      = 35,
		.settings = {
			[GPIOMUX_SUSPENDED] = &i2c_suspended_config,
			[GPIOMUX_ACTIVE]    = &i2c_active,
		},
	},
	{
		.gpio      = 36,
		.settings = {
			[GPIOMUX_SUSPENDED] = &i2c_suspended_config,
			[GPIOMUX_ACTIVE]    = &i2c_active,
		},
	},
#else
	{
		.gpio      = 34,
		.settings = {
			[GPIOMUX_SUSPENDED] = &spi_suspended_config,
			[GPIOMUX_ACTIVE]    = &spi_active,
		},
	},
	{
		.gpio      = 35,
		.settings = {
			[GPIOMUX_SUSPENDED] = &spi_suspended_cs_config,
			[GPIOMUX_ACTIVE]    = &spi_active,
		},
	},
	{
		.gpio      = 36,
		.settings = {
			[GPIOMUX_SUSPENDED] = &spi_suspended_config,
			[GPIOMUX_ACTIVE]    = &spi_active,
		},
	},
#endif //CONFIG_INPUT_SENSOR

#ifdef CONFIG_SKY_DMB_SPI_HW
	{
      .gpio      = 37, // MOSI
      .settings = {
        [GPIOMUX_SUSPENDED] = &gsbi2,
      },
    },
    {
      .gpio      = 38, // MISO
      .settings = {
        [GPIOMUX_SUSPENDED] = &gsbi2,
      },
    },
      {
      .gpio      = 39, // CS
      .settings = {
        [GPIOMUX_SUSPENDED] = &gsbi2,
      },
    },
    {
      .gpio      = 40, // CLK
      .settings = {
        [GPIOMUX_SUSPENDED] = &gsbi2,
      },
    },
#endif
	{
		.gpio      = 43,
		.settings = {
			[GPIOMUX_SUSPENDED] = &i2c_suspended_config,
			[GPIOMUX_ACTIVE]    = &i2c_active,
		},
	},
	{
		.gpio      = 44,
		.settings = {
			[GPIOMUX_SUSPENDED] = &i2c_suspended_config,
			[GPIOMUX_ACTIVE]    = &i2c_active,
		},
	},
	{
		.gpio      = 47,
		.settings = {
			[GPIOMUX_SUSPENDED] = &i2c_suspended_config,
			[GPIOMUX_ACTIVE]    = &i2c_active,
		},
	},
	{
		.gpio      = 48,
		.settings = {
			[GPIOMUX_SUSPENDED] = &i2c_suspended_config,
			[GPIOMUX_ACTIVE]    = &i2c_active,
		},
	},
#if defined (CONFIG_INPUT_SENSOR)
#if defined(CONFIG_MACH_MSM8X60_EF39S) || defined(CONFIG_MACH_MSM8X60_EF40K) || defined(CONFIG_MACH_MSM8X60_EF40S) || defined(CONFIG_MACH_MSM8X60_EF65L)
#if defined(CONFIG_SENSORS_MPU3050)
	{
                        .gpio      = 51,
                        .settings = {
                            [GPIOMUX_SUSPENDED] = &gsbi5,
                        },
                    },
                    {
                        .gpio      = 52,
                        .settings = {
                            [GPIOMUX_SUSPENDED] = &gsbi5,
                        },
                    },
#endif
#endif
#endif					
	{
		.gpio      = 59,
		.settings = {
			[GPIOMUX_SUSPENDED] = &i2c_suspended_config,
			[GPIOMUX_ACTIVE]    = &i2c_active_gsbi7,
		},
	},
	{
		.gpio      = 60,
		.settings = {
			[GPIOMUX_SUSPENDED] = &i2c_suspended_config,
			[GPIOMUX_ACTIVE]    = &i2c_active_gsbi7,
		},
	},
	{
		.gpio      = 64,
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi8,
		},
	},
	{
		.gpio      = 65,
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi8,
		},
	},
#if defined (CONFIG_INPUT_SENSOR)
#if defined(CONFIG_MACH_MSM8X60_EF33S) || defined(CONFIG_MACH_MSM8X60_EF34K) || defined(CONFIG_MACH_MSM8X60_EF35L)
#if defined(CONFIG_SENSORS_MPU3050)

	{
		.gpio      = 68, // sda
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi9,
		},
	},
	{
		.gpio      = 69, // scl
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi9,
		},
	},
#endif
#endif
#endif
#if defined(CONFIG_PN544) && (defined(CONFIG_MACH_MSM8X60_EF39S) ||defined(CONFIG_MACH_MSM8X60_EF40S) ||defined(CONFIG_MACH_MSM8X60_EF40K) || defined(CONFIG_MACH_MSM8X60_EF65L))
	{
		.gpio      = 72,
		.settings = {
			[GPIOMUX_SUSPENDED] = &i2c_suspended_config,
			[GPIOMUX_ACTIVE]    = &i2c_active,
		},
	},
	{
		.gpio      = 73,
		.settings = {
			[GPIOMUX_SUSPENDED] = &i2c_suspended_config,
			[GPIOMUX_ACTIVE]    = &i2c_active,
		},
	},
#endif
#if defined(CONFIG_SKY_BATTERY_MAX17040) || defined(CONFIG_SKY_BATTERY_MAX17043)
	{
		.gpio = 104, // sda
		.settings = {
			[GPIOMUX_SUSPENDED] = &GSBI11,
		},
	},
	{
		.gpio = 103, // scl
		.settings = {
			[GPIOMUX_SUSPENDED] = &GSBI11,
		},
	},
#endif
};

static struct msm_gpiomux_config msm8x60_fluid_gsbi_configs[] __initdata = {
	{
		.gpio      = 70,
		.settings = {
			[GPIOMUX_SUSPENDED] = &spi_suspended_config,
			[GPIOMUX_ACTIVE]    = &spi_active,
		},
	},
	{
		.gpio      = 72,
		.settings = {
			[GPIOMUX_SUSPENDED] = &spi_suspended_cs_config,
			[GPIOMUX_ACTIVE]    = &spi_active,
		},
	},
	{
		.gpio      = 73,
		.settings = {
			[GPIOMUX_SUSPENDED] = &spi_suspended_config,
			[GPIOMUX_ACTIVE]    = &spi_active,
		},
	},
};

static struct msm_gpiomux_config msm8x60_ebi2_configs[] __initdata = {
#ifndef CONFIG_SKY_DMB_SPI_HW
	{
		.gpio      = 40,
		.settings = {
			[GPIOMUX_SUSPENDED] = &ebi2_cs2,
		},
	},
#endif
	{
		.gpio      = 123,
		.settings = {
			[GPIOMUX_SUSPENDED] = &ebi2_a_d,
		},
	},
#if defined(CONFIG_INPUT_SENSOR) //EF33S PROXIMITY INTERRUPT PIN
#if defined(CONFIG_MACH_MSM8X60_EF39S) || defined(CONFIG_MACH_MSM8X60_EF40K) || defined(CONFIG_MACH_MSM8X60_EF40S) || defined(CONFIG_MACH_MSM8X60_EF65L)
	{
		.gpio      = 124,
		.settings = {
			[GPIOMUX_SUSPENDED] = &ebi2_a_d,
		},
	},
#endif
#endif
	{
		.gpio      = 125,
		.settings = {
			[GPIOMUX_SUSPENDED] = &ebi2_a_d,
		},
	},
	{
		.gpio      = 126,
		.settings = {
			[GPIOMUX_SUSPENDED] = &ebi2_a_d,
		},
	},
	{
		.gpio      = 127,
		.settings = {
			[GPIOMUX_SUSPENDED] = &ebi2_a_d,
		},
	},
	{
		.gpio      = 128,
		.settings = {
			[GPIOMUX_SUSPENDED] = &ebi2_a_d,
		},
	},
	{
		.gpio      = 129,
		.settings = {
			[GPIOMUX_SUSPENDED] = &ebi2_a_d,
		},
	},
	{
		.gpio      = 130,
		.settings = {
			[GPIOMUX_SUSPENDED] = &ebi2_a_d,
		},
	},
#if defined(CONFIG_USB_PEHCI_HCD) || defined(CONFIG_USB_PEHCI_HCD_MODULE)
	/* ISP VDD_3V3_EN */
	{
		.gpio      = 132,
		.settings = {
			[GPIOMUX_SUSPENDED] = &ebi2_cs4,
		},
	},
#endif
	{
		.gpio      = 133,
		.settings = {
			[GPIOMUX_SUSPENDED] = &ebi2_cs3,
		},
	},
	{
		.gpio      = 135,
		.settings = {
			[GPIOMUX_SUSPENDED] = &ebi2_a_d,
		},
	},
	{
		.gpio      = 136,
		.settings = {
			[GPIOMUX_SUSPENDED] = &ebi2_a_d,
		},
	},
	{
		.gpio      = 137,
		.settings = {
			[GPIOMUX_SUSPENDED] = &ebi2_a_d,
		},
	},
	{
		.gpio      = 138,
		.settings = {
			[GPIOMUX_SUSPENDED] = &ebi2_a_d,
		},
	},
	{
		.gpio      = 139,
		.settings = {
			[GPIOMUX_SUSPENDED] = &ebi2_a_d,
		},
	},
#ifndef CONFIG_FB_MSM_MIPI_DSI_MAGNA
	{
		.gpio      = 140,
		.settings = {
			[GPIOMUX_SUSPENDED] = &ebi2_a_d,
		},
	},
#endif

#if !(defined(CONFIG_PN544) && (defined(CONFIG_MACH_MSM8X60_EF33S) || defined(CONFIG_MACH_MSM8X60_EF34K) || defined(CONFIG_MACH_MSM8X60_EF34C)))
	{
		.gpio      = 141,
		.settings = {
			[GPIOMUX_SUSPENDED] = &ebi2_a_d,
		},
	},
#endif
#if !(defined(CONFIG_PN544) && (defined(CONFIG_MACH_MSM8X60_EF33S) || defined(CONFIG_MACH_MSM8X60_EF34K) || defined(CONFIG_MACH_MSM8X60_EF34C) \
        || defined(CONFIG_MACH_MSM8X60_EF39S) ||defined(CONFIG_MACH_MSM8X60_EF40S) ||defined(CONFIG_MACH_MSM8X60_EF40K) ))
	{
		.gpio      = 142,
		.settings = {
			[GPIOMUX_SUSPENDED] = &ebi2_a_d,
		},
	},
#endif
	{
		.gpio      = 143,
		.settings = {
			[GPIOMUX_SUSPENDED] = &ebi2_a_d,
		},
	},
	{
		.gpio      = 144,
		.settings = {
			[GPIOMUX_SUSPENDED] = &ebi2_a_d,
		},
	},
	{
		.gpio      = 145,
		.settings = {
			[GPIOMUX_SUSPENDED] = &ebi2_a_d,
		},
	},
	{
		.gpio      = 146,
		.settings = {
			[GPIOMUX_SUSPENDED] = &ebi2_a_d,
		},
	},
	{
		.gpio      = 147,
		.settings = {
			[GPIOMUX_SUSPENDED] = &ebi2_a_d,
		},
	},
	{
		.gpio      = 148,
		.settings = {
			[GPIOMUX_SUSPENDED] = &ebi2_a_d,
		},
	},
	{
		.gpio      = 149,
		.settings = {
			[GPIOMUX_SUSPENDED] = &ebi2_a_d,
		},
	},
	{
		.gpio      = 150,
		.settings = {
			[GPIOMUX_SUSPENDED] = &ebi2_a_d,
		},
	},
	{
		.gpio      = 151,
		.settings = {
			[GPIOMUX_SUSPENDED] = &ebi2_oe,
		},
	},

#if !(defined(CONFIG_PN544) && (defined(CONFIG_MACH_MSM8X60_EF39S) ||defined(CONFIG_MACH_MSM8X60_EF40S) ||defined(CONFIG_MACH_MSM8X60_EF40K)))
	{
		.gpio      = 153,
		.settings = {
			[GPIOMUX_SUSPENDED] = &ebi2_adv,
		},
	},
#endif
	{
		.gpio      = 157,
		.settings = {
			[GPIOMUX_SUSPENDED] = &ebi2_we,
		},
	},
};

#if defined(CONFIG_USB_PEHCI_HCD) || defined(CONFIG_USB_PEHCI_HCD_MODULE)
static struct msm_gpiomux_config msm8x60_isp_usb_configs[] __initdata = {
	{
		.gpio      = 117,
		.settings = {
			[GPIOMUX_ACTIVE]    = &usb_isp1763_actv_cfg,
			[GPIOMUX_SUSPENDED] = &usb_isp1763_susp_cfg,
		},
	},
	{
		.gpio      = 152,
		.settings = {
			[GPIOMUX_ACTIVE]    = &usb_isp1763_actv_cfg,
			[GPIOMUX_SUSPENDED] = &usb_isp1763_susp_cfg,
		},
	},

};
#endif

static struct msm_gpiomux_config msm8x60_uart_configs[] __initdata = {
	{ /* UARTDM_TX */
		.gpio      = 53,
		.settings = {
			[GPIOMUX_ACTIVE]    = &uart1dm_active,
			[GPIOMUX_SUSPENDED] = &uart1dm_suspended,
		},
	},
	{ /* UARTDM_RX */
		.gpio      = 54,
		.settings = {
			[GPIOMUX_ACTIVE]    = &uart1dm_active,
			[GPIOMUX_SUSPENDED] = &uart1dm_suspended,
		},
	},
	{ /* UARTDM_CTS */
		.gpio      = 55,
		.settings = {
			[GPIOMUX_ACTIVE]    = &uart1dm_active,
			[GPIOMUX_SUSPENDED] = &uart1dm_suspended,
		},
	},
	{ /* UARTDM_RFR */
		.gpio      = 56,
		.settings = {
			[GPIOMUX_ACTIVE]    = &uart1dm_active,
			[GPIOMUX_SUSPENDED] = &uart1dm_suspended,
		},
	},
	{
		.gpio      = 115,
		.settings = {
			[GPIOMUX_SUSPENDED] = &console_uart,
		},
	},
	{
		.gpio      = 116,
		.settings = {
			[GPIOMUX_SUSPENDED] = &console_uart,
		},
	},
#if !defined(CONFIG_USB_PEHCI_HCD) && !defined(CONFIG_USB_PEHCI_HCD_MODULE)
	/* USB ISP1763 may also use 117 GPIO */
	{
		.gpio      = 117,
		.settings = {
			[GPIOMUX_SUSPENDED] = &console_uart,
		},
	},
#endif
	{
		.gpio      = 118,
		.settings = {
			[GPIOMUX_SUSPENDED] = &console_uart,
		},
	},
};

#ifdef CONFIG_MSM_GSBI9_UART
static struct msm_gpiomux_config msm8x60_charm_uart_configs[] __initdata = {
	{ /* UART9DM  RX */
		.gpio	   = 66,
		.settings = {
			[GPIOMUX_ACTIVE]    = &uart9dm_active,
			[GPIOMUX_SUSPENDED] = &gsbi9,
		},
	},
	{ /* UART9DM TX */
		.gpio	   = 67,
		.settings = {
			[GPIOMUX_ACTIVE]    = &uart9dm_active,
			[GPIOMUX_SUSPENDED] = &gsbi9,
		},
	},
};
#endif

static struct msm_gpiomux_config msm8x60_ts_configs[] __initdata = {
	{
		/* TS_ATTN */
		.gpio = 58,
		.settings = {
			[GPIOMUX_SUSPENDED] = &ts_suspended,
		},
	},
};

static struct msm_gpiomux_config msm8x60_tmg200_configs[] __initdata = {
	{
		.gpio = 61,
		.settings = {
			[GPIOMUX_ACTIVE] = &tm_active,
			[GPIOMUX_SUSPENDED] = &tm_suspended,
		},
	},
};

static struct msm_gpiomux_config msm8x60_tma300_configs[] __initdata = {
	{
		.gpio = 61,
		.settings = {
			[GPIOMUX_ACTIVE]    = &tma_active,
			[GPIOMUX_SUSPENDED] = &tm_suspended,
		},
	},
};

static struct msm_gpiomux_config msm8x60_aux_pcm_configs[] __initdata = {
	{
		.gpio = 111,
		.settings = {
			[GPIOMUX_ACTIVE]    = &aux_pcm_active_config,
			[GPIOMUX_SUSPENDED] = &aux_pcm_suspend_config,
		},
	},
	{
		.gpio = 112,
		.settings = {
			[GPIOMUX_ACTIVE]    = &aux_pcm_active_config,
			[GPIOMUX_SUSPENDED] = &aux_pcm_suspend_config,
		},
	},
	{
		.gpio = 113,
		.settings = {
			[GPIOMUX_ACTIVE]    = &aux_pcm_active_config,
			[GPIOMUX_SUSPENDED] = &aux_pcm_suspend_config,
		},
	},
	{
		.gpio = 114,
		.settings = {
			[GPIOMUX_ACTIVE]    = &aux_pcm_active_config,
			[GPIOMUX_SUSPENDED] = &aux_pcm_suspend_config,
		},
	},
};

static struct msm_gpiomux_config msm8x60_sdc_configs[] __initdata = {
	/* SDCC1 data[0] */
	{
		.gpio = 159,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdcc1_dat_0_3_cmd_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdcc1_suspend_config,
		},
	},
	/* SDCC1 data[1] */
	{
		.gpio = 160,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdcc1_dat_0_3_cmd_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdcc1_suspend_config,
		},
	},
	/* SDCC1 data[2] */
	{
		.gpio = 161,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdcc1_dat_0_3_cmd_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdcc1_suspend_config,
		},
	},
	/* SDCC1 data[3] */
	{
		.gpio = 162,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdcc1_dat_0_3_cmd_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdcc1_suspend_config,
		},
	},
	/* SDCC1 data[4] */
	{
		.gpio = 163,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdcc1_dat_4_7_cmd_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdcc1_suspend_config,
		},
	},
	/* SDCC1 data[5] */
	{
		.gpio = 164,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdcc1_dat_4_7_cmd_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdcc1_suspend_config,
		},
	},
	/* SDCC1 data[6] */
	{
		.gpio = 165,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdcc1_dat_4_7_cmd_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdcc1_suspend_config,
		},
	},
	/* SDCC1 data[7] */
	{
		.gpio = 166,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdcc1_dat_4_7_cmd_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdcc1_suspend_config,
		},
	},
	/* SDCC1 CLK */
	{
		.gpio = 167,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdcc1_clk_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdcc1_suspend_config,
		},
	},
	/* SDCC1 CMD */
	{
		.gpio = 168,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdcc1_dat_0_3_cmd_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdcc1_suspend_config,
		},
	},
};

static struct msm_gpiomux_config msm8x60_charm_sdc_configs[] __initdata = {
	/* SDCC5 cmd */
	{
		.gpio = 95,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdcc5_dat_0_3_cmd_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdcc5_suspend_config,
		},
	},
#if !(defined(CONFIG_PN544) && (defined(CONFIG_MACH_MSM8X60_EF33S) || defined(CONFIG_MACH_MSM8X60_EF34K) || defined(CONFIG_MACH_MSM8X60_EF34C)))
	/* SDCC5 data[3]*/
	{
		.gpio = 96,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdcc5_dat_0_3_cmd_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdcc5_suspend_config,
		},
	},
#endif
	/* SDCC5 clk */
	{
		.gpio = 97,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdcc5_clk_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdcc5_suspend_config,
		},
	},
	/* SDCC5 data[2]*/
	{
		.gpio = 98,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdcc5_dat_0_3_cmd_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdcc5_suspend_config,
		},
	},
	/* SDCC5 data[1]*/
	{
		.gpio = 99,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdcc5_dat_0_3_cmd_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdcc5_suspend_config,
		},
	},
	/* SDCC5 data[0]*/
	{
		.gpio = 100,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdcc5_dat_0_3_cmd_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdcc5_suspend_config,
		},
	},
	/* MDM2AP_SYNC */
	{
		.gpio = 129,
		.settings = {
			[GPIOMUX_ACTIVE]    = &mdm2ap_sync_active_cfg,
			[GPIOMUX_SUSPENDED] = &mdm2ap_sync_suspend_cfg,
		},
	},

#ifndef CONFIG_FB_MSM_MIPI_DSI_MAGNA
	/* MDM2AP_VDDMIN */
	{
		.gpio = 140,
		.settings = {
			[GPIOMUX_ACTIVE]    = &mdm2ap_vddmin_active_cfg,
			[GPIOMUX_SUSPENDED] = &mdm2ap_vddmin_suspend_cfg,
		},
	},
#endif

	/* SDCC2 data[0] */
	{
		.gpio = 143,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdcc2_dat_0_3_cmd_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdcc2_suspend_config,
		},
	},
	/* SDCC2 data[1] */
	{
		.gpio = 144,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdcc2_dat_0_3_cmd_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdcc2_suspend_config,
		},
	},
	/* SDCC2 data[2] */
	{
		.gpio = 145,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdcc2_dat_0_3_cmd_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdcc2_suspend_config,
		},
	},
	/* SDCC2 data[3] */
	{
		.gpio = 146,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdcc2_dat_0_3_cmd_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdcc2_suspend_config,
		},
	},
	/* SDCC2 data[4] */
	{
		.gpio = 147,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdcc2_dat_4_7_cmd_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdcc2_suspend_config,
		},
	},
	/* SDCC2 data[5] */
	{
		.gpio = 148,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdcc2_dat_4_7_cmd_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdcc2_suspend_config,
		},
	},
	/* SDCC2 data[6] */
	{
		.gpio = 149,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdcc2_dat_4_7_cmd_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdcc2_suspend_config,
		},
	},
	/* SDCC2 data[7] */
	{
		.gpio = 150,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdcc2_dat_4_7_cmd_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdcc2_suspend_config,
		},
	},
	/* SDCC2 CMD */
	{
		.gpio = 151,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdcc2_dat_0_3_cmd_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdcc2_suspend_config,
		},
	},
	/* SDCC2 CLK */
	{
		.gpio = 152,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdcc2_clk_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdcc2_suspend_config,
		},
	},
};

static struct msm_gpiomux_config msm8x60_snd_configs[] __initdata = {
	{
		.gpio = 108,
		.settings = {
			[GPIOMUX_ACTIVE]    = &msm_snddev_active_config,
			[GPIOMUX_SUSPENDED] = &msm_snddev_suspend_config,
		},
	},
	{
		.gpio = 109,
		.settings = {
			[GPIOMUX_ACTIVE]    = &msm_snddev_active_config,
			[GPIOMUX_SUSPENDED] = &msm_snddev_suspend_config,
		},
	},
};

static struct msm_gpiomux_config msm8x60_mi2s_configs[] __initdata = {
	/* MI2S WS */
	{
		.gpio = 101,
		.settings = {
			[GPIOMUX_ACTIVE]    = &mi2s_active_cfg,
			[GPIOMUX_SUSPENDED] = &mi2s_suspend_cfg,
		},
	},
	/* MI2S SCLK */
	{
		.gpio = 102,
		.settings = {
			[GPIOMUX_ACTIVE]    = &mi2s_active_cfg,
			[GPIOMUX_SUSPENDED] = &mi2s_suspend_cfg,
		},
	},
	/* MI2S MCLK */
	{
		.gpio = 103,
		.settings = {
			[GPIOMUX_ACTIVE]    = &mi2s_active_cfg,
			[GPIOMUX_SUSPENDED] = &mi2s_suspend_cfg,
		},
	},
	/* MI2S SD3 */
	{
		.gpio = 107,
		.settings = {
			[GPIOMUX_ACTIVE]    = &mi2s_active_cfg,
			[GPIOMUX_SUSPENDED] = &mi2s_suspend_cfg,
		},
	},
};


static struct msm_gpiomux_config msm8x60_lcdc_configs[] __initdata = {
	/* lcdc_pclk */
	{
		.gpio = 0,
		.settings = {
			[GPIOMUX_ACTIVE]    = &lcdc_active_cfg,
			[GPIOMUX_SUSPENDED] = &lcdc_suspend_cfg,
		},
	},
	/* lcdc_hsync */
	{
		.gpio = 1,
		.settings = {
			[GPIOMUX_ACTIVE]    = &lcdc_active_cfg,
			[GPIOMUX_SUSPENDED] = &lcdc_suspend_cfg,
		},
	},
	/* lcdc_vsync */
	{
		.gpio = 2,
		.settings = {
			[GPIOMUX_ACTIVE]    = &lcdc_active_cfg,
			[GPIOMUX_SUSPENDED] = &lcdc_suspend_cfg,
		},
	},
	/* lcdc_den */
	{
		.gpio = 3,
		.settings = {
			[GPIOMUX_ACTIVE]    = &lcdc_active_cfg,
			[GPIOMUX_SUSPENDED] = &lcdc_suspend_cfg,
		},
	},
	/* lcdc_red7 */
	{
		.gpio = 4,
		.settings = {
			[GPIOMUX_ACTIVE]    = &lcdc_active_cfg,
			[GPIOMUX_SUSPENDED] = &lcdc_suspend_cfg,
		},
	},
	/* lcdc_red6 */
	{
		.gpio = 5,
		.settings = {
			[GPIOMUX_ACTIVE]    = &lcdc_active_cfg,
			[GPIOMUX_SUSPENDED] = &lcdc_suspend_cfg,
		},
	},
	/* lcdc_red5 */
	{
		.gpio = 6,
		.settings = {
			[GPIOMUX_ACTIVE]    = &lcdc_active_cfg,
			[GPIOMUX_SUSPENDED] = &lcdc_suspend_cfg,
		},
	},
	/* lcdc_red4 */
	{
		.gpio = 7,
		.settings = {
			[GPIOMUX_ACTIVE]    = &lcdc_active_cfg,
			[GPIOMUX_SUSPENDED] = &lcdc_suspend_cfg,
		},
	},
	/* lcdc_red3 */
	{
		.gpio = 8,
		.settings = {
			[GPIOMUX_ACTIVE]    = &lcdc_active_cfg,
			[GPIOMUX_SUSPENDED] = &lcdc_suspend_cfg,
		},
	},
#if !(defined(CONFIG_SKY_SMB136S_CHARGER) && defined(CONFIG_MACH_MSM8X60_EF65L))
	/* lcdc_red2 */
	{
		.gpio = 9,
		.settings = {
			[GPIOMUX_ACTIVE]    = &lcdc_active_cfg,
			[GPIOMUX_SUSPENDED] = &lcdc_suspend_cfg,
		},
	},
	/* lcdc_red1 */
	{
		.gpio = 10,
		.settings = {
			[GPIOMUX_ACTIVE]    = &lcdc_active_cfg,
			[GPIOMUX_SUSPENDED] = &lcdc_suspend_cfg,
		},
	},
#endif
	/* lcdc_red0 */
	{
		.gpio = 11,
		.settings = {
			[GPIOMUX_ACTIVE]    = &lcdc_active_cfg,
			[GPIOMUX_SUSPENDED] = &lcdc_suspend_cfg,
		},
	},
	/* lcdc_grn7 */
	{
		.gpio = 12,
		.settings = {
			[GPIOMUX_ACTIVE]    = &lcdc_active_cfg,
			[GPIOMUX_SUSPENDED] = &lcdc_suspend_cfg,
		},
	},
#ifndef CONFIG_FB_MSM_MIPI_DSI_SONY
	/* lcdc_grn6 */
	{
		.gpio = 13,
		.settings = {
			[GPIOMUX_ACTIVE]    = &lcdc_active_cfg,
			[GPIOMUX_SUSPENDED] = &lcdc_suspend_cfg,
		},
	},
#endif
	/* lcdc_grn5 */
	{
		.gpio = 14,
		.settings = {
			[GPIOMUX_ACTIVE]    = &lcdc_active_cfg,
			[GPIOMUX_SUSPENDED] = &lcdc_suspend_cfg,
		},
	},
	/* lcdc_grn4 */
	{
		.gpio = 15,
		.settings = {
			[GPIOMUX_ACTIVE]    = &lcdc_active_cfg,
			[GPIOMUX_SUSPENDED] = &lcdc_suspend_cfg,
		},
	},
#if !(defined(CONFIG_PN544) && defined(CONFIG_MACH_MSM8X60_EF65L))
	/* lcdc_grn3 */
	{
		.gpio = 16,
		.settings = {
			[GPIOMUX_ACTIVE]    = &lcdc_active_cfg,
			[GPIOMUX_SUSPENDED] = &lcdc_suspend_cfg,
		},
	},
#if !defined(CONFIG_MACH_MSM8X60_EF39S) && !defined(CONFIG_MACH_MSM8X60_EF40S) && !defined(CONFIG_MACH_MSM8X60_EF40K)
	/* lcdc_grn2 */
	{
		.gpio = 17,
		.settings = {
			[GPIOMUX_ACTIVE]    = &lcdc_active_cfg,
			[GPIOMUX_SUSPENDED] = &lcdc_suspend_cfg,
		},
	},
#endif
#endif
#if !defined(CONFIG_MACH_MSM8X60_EF39S) && !defined(CONFIG_MACH_MSM8X60_EF40S) && !defined(CONFIG_MACH_MSM8X60_EF40K)
	/* lcdc_grn1 */
	{
		.gpio = 18,
		.settings = {
			[GPIOMUX_ACTIVE]    = &lcdc_active_cfg,
			[GPIOMUX_SUSPENDED] = &lcdc_suspend_cfg,
		},
	},
#endif
	/* lcdc_grn0 */
	{
		.gpio = 19,
		.settings = {
			[GPIOMUX_ACTIVE]    = &lcdc_active_cfg,
			[GPIOMUX_SUSPENDED] = &lcdc_suspend_cfg,
		},
	},
	/* lcdc_blu7 */
	{
		.gpio = 20,
		.settings = {
			[GPIOMUX_ACTIVE]    = &lcdc_active_cfg,
			[GPIOMUX_SUSPENDED] = &lcdc_suspend_cfg,
		},
	},
	/* lcdc_blu6 */
	{
		.gpio = 21,
		.settings = {
			[GPIOMUX_ACTIVE]    = &lcdc_active_cfg,
			[GPIOMUX_SUSPENDED] = &lcdc_suspend_cfg,
		},
	},
	/* lcdc_blu5 */
	{
		.gpio = 22,
		.settings = {
			[GPIOMUX_ACTIVE]    = &lcdc_active_cfg,
			[GPIOMUX_SUSPENDED] = &lcdc_suspend_cfg,
		},
	},
	/* lcdc_blu4 */
	{
		.gpio = 23,
		.settings = {
			[GPIOMUX_ACTIVE]    = &lcdc_active_cfg,
			[GPIOMUX_SUSPENDED] = &lcdc_suspend_cfg,
		},
	},
	/* lcdc_blu3 */
	{
		.gpio = 24,
		.settings = {
			[GPIOMUX_ACTIVE]    = &lcdc_active_cfg,
			[GPIOMUX_SUSPENDED] = &lcdc_suspend_cfg,
		},
	},
	/* lcdc_blu2 */
	{
		.gpio = 25,
		.settings = {
			[GPIOMUX_ACTIVE]    = &lcdc_active_cfg,
			[GPIOMUX_SUSPENDED] = &lcdc_suspend_cfg,
		},
	},
	/* lcdc_blu1 */
	{
		.gpio = 26,
		.settings = {
			[GPIOMUX_ACTIVE]    = &lcdc_active_cfg,
			[GPIOMUX_SUSPENDED] = &lcdc_suspend_cfg,
		},
	},
	/* lcdc_blu0 */
	{
		.gpio = 27,
		.settings = {
			[GPIOMUX_ACTIVE]    = &lcdc_active_cfg,
			[GPIOMUX_SUSPENDED] = &lcdc_suspend_cfg,
		},
	},
};

static struct msm_gpiomux_config msm8x60_mdp_vsync_configs[] __initdata = {
	{
		.gpio = 28,
		.settings = {
			[GPIOMUX_ACTIVE]    = &mdp_vsync_active_cfg,
			[GPIOMUX_SUSPENDED] = &mdp_vsync_suspend_cfg,
		},
	},
};

static struct msm_gpiomux_config msm8x60_hdmi_configs[] __initdata = {
	{
		.gpio = 169,
		.settings = {
			[GPIOMUX_ACTIVE]    = &hdmi_active_1_cfg,
			[GPIOMUX_SUSPENDED] = &hdmi_suspend_cfg,
		},
	},
	{
		.gpio = 170,
		.settings = {
			[GPIOMUX_ACTIVE]    = &hdmi_active_2_cfg,
			[GPIOMUX_SUSPENDED] = &hdmi_suspend_cfg,
		},
	},
	{
		.gpio = 171,
		.settings = {
			[GPIOMUX_ACTIVE]    = &hdmi_active_2_cfg,
			[GPIOMUX_SUSPENDED] = &hdmi_suspend_cfg,
		},
	},
	{
		.gpio = 172,
		.settings = {
			[GPIOMUX_ACTIVE]    = &hdmi_active_3_cfg,
			[GPIOMUX_SUSPENDED] = &hdmi_suspend_cfg,
		},
	},
#ifdef CONFIG_PANTECH_FB_MSM_MHL_SII9244
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
			[GPIOMUX_ACTIVE]    = &mhl_active_2_cfg,
			[GPIOMUX_SUSPENDED] = &mhl_suspend_cfg,
		},
	},
	{
		.gpio = MHL_SHDN,
		.settings = {
			[GPIOMUX_ACTIVE]    = &mhl_active_1_cfg,
			[GPIOMUX_SUSPENDED] = &mhl_suspend_cfg,
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
			[GPIOMUX_SUSPENDED] = &mhl_suspend_pullup_cfg,
		},
	},
#endif
};

/* Because PMIC drivers do not use gpio-management routines and PMIC
 * gpios must never sleep, a "good enough" config is obtained by placing
 * the active config in the 'suspended' slot and leaving the active
 * config invalid: the suspended config will be installed at boot
 * and never replaced.
 */

static struct msm_gpiomux_config msm8x60_pmic_configs[] __initdata = {
	{
		.gpio = 88,
		.settings = {
			[GPIOMUX_SUSPENDED] = &pmic_suspended_cfg,
		},
	},
	{
		.gpio = 91,
		.settings = {
			[GPIOMUX_SUSPENDED] = &pmic_suspended_cfg,
		},
	},
};

static struct msm_gpiomux_config msm8x60_common_configs[] __initdata = {
	/* MDM2AP_STATUS */
	{
		.gpio = 77,
		.settings = {
			[GPIOMUX_ACTIVE]    = &mdm2ap_status_active_cfg,
			[GPIOMUX_SUSPENDED] = &mdm2ap_status_suspend_cfg,
		},
	},
	/* PS_HOLD */
	{
		.gpio      = 92,
		.settings = {
			[GPIOMUX_SUSPENDED] = &ps_hold,
		},
	},
};

#if defined(CONFIG_PN544)
static struct msm_gpiomux_config msm8x60_nfc_configs[] __initdata = {
#if defined(CONFIG_MACH_MSM8X60_EF33S) || defined(CONFIG_MACH_MSM8X60_EF34K) || defined(CONFIG_MACH_MSM8X60_EF34C)
    // ven
	{
		.gpio      = 141,
		.settings = {
			[GPIOMUX_SUSPENDED] = &PN544_suspend_cfg,
		},
	},
    // Firmware download
	{
		.gpio      = 142,
		.settings = {
			[GPIOMUX_SUSPENDED] = &PN544_suspend_cfg,
		},
	},
    // irq
	{
		.gpio      = 96,
		.settings = {
			[GPIOMUX_SUSPENDED] = &PN544_suspend_cfg,
		},
	},

#elif defined (CONFIG_MACH_MSM8X60_EF39S) || defined(CONFIG_MACH_MSM8X60_EF40S) || defined(CONFIG_MACH_MSM8X60_EF40K)
    // ven
	{
		.gpio      = 153,
		.settings = {
			[GPIOMUX_SUSPENDED] = &PN544_suspend_cfg,
		},
	},
    // Firmware download
	{
		.gpio      = 142,
		.settings = {
			[GPIOMUX_SUSPENDED] = &PN544_suspend_cfg,
		},
	},
    // irq
	{
		.gpio      = 105,
		.settings = {
			[GPIOMUX_SUSPENDED] = &PN544_suspend_cfg,
		},
	},
#elif defined(CONFIG_MACH_MSM8X60_EF65L)
    // ven
	{
		.gpio      = 16,
		.settings = {
			[GPIOMUX_SUSPENDED] = &PN544_suspend_cfg,
		},
	},
    // Firmware download
	{
		.gpio      = 17,
		.settings = {
			[GPIOMUX_SUSPENDED] = &PN544_suspend_cfg,
		},
	},
    // irq
	{
		.gpio      = 105,
		.settings = {
			[GPIOMUX_SUSPENDED] = &PN544_suspend_cfg,
		},
	},
#endif
};
#endif /* defined(CONFIG_PN544) */


static struct msm_gpiomux_config msm8x60_cam_configs[] __initdata = {
#ifdef CONFIG_PANTECH_CAMERA_HW
        {   // CAMIO_MCLK
            .gpio = 32,
            .settings = {
                [GPIOMUX_ACTIVE]    = &cam_active_5_cfg,
                [GPIOMUX_SUSPENDED] = &cam_suspend_cfg,
            },
        },
        {   // CAMIO_I2C_SDA
            .gpio = 47,
            .settings = {
                [GPIOMUX_ACTIVE]    = &cam_active_3_cfg,
                [GPIOMUX_SUSPENDED] = &cam_suspend_cfg,
            },
        },
        {   // CAMIO_I2C_SCL
            .gpio = 48,
            .settings = {
                [GPIOMUX_ACTIVE]    = &cam_active_3_cfg,
                [GPIOMUX_SUSPENDED] = &cam_suspend_cfg,
            },
        },
#if defined(CONFIG_MACH_MSM8X60_EF39S) || \
        defined(CONFIG_MACH_MSM8X60_EF40S) || \
        defined(CONFIG_MACH_MSM8X60_EF40K)
        {   // CAMIO_R_STBY(_N)
            .gpio = 46,
            .settings = {
                [GPIOMUX_ACTIVE]    = &cam_active_2_cfg,
                [GPIOMUX_SUSPENDED] = &cam_suspend_cfg,
            },
        },
#endif        
#if defined(CONFIG_MACH_MSM8X60_EF65L)  
        {   // CAMIO_R_STBY(_N) for65L
            .gpio = 62,
            .settings = {
                [GPIOMUX_ACTIVE]    = &cam_active_2_cfg,
                [GPIOMUX_SUSPENDED] = &cam_suspend_cfg,
            },
        },
#endif        
        {   // CAMIO_R_RST(_N)
            .gpio = 106,
            .settings = {
                [GPIOMUX_ACTIVE]    = &cam_active_2_cfg,
                [GPIOMUX_SUSPENDED] = &cam_suspend_cfg,
            },
        },
        {   // CAMIO_F_STBY(_N)
            .gpio = 139,
            .settings = {
                [GPIOMUX_ACTIVE]    = &cam_active_2_cfg,
                [GPIOMUX_SUSPENDED] = &cam_suspend_cfg,
            },
        },
        {   // CAMIO_F_RST(_N)
            .gpio = 137,
            .settings = {
                [GPIOMUX_ACTIVE]    = &cam_active_2_cfg,
                [GPIOMUX_SUSPENDED] = &cam_suspend_cfg,
            },
        },
        {   // CAMIO_FL_DRV_EN
            .gpio = 31,
            .settings = {
                [GPIOMUX_ACTIVE]    = &cam_active_2_cfg,
                [GPIOMUX_SUSPENDED] = &cam_suspend_cfg,
            },
        },
#if defined(CONFIG_MACH_MSM8X60_EF39S) || \
        defined(CONFIG_MACH_MSM8X60_EF40S) || \
        defined(CONFIG_MACH_MSM8X60_EF40K) || \
        defined(CONFIG_MACH_MSM8X60_EF65L)  
        {   // CAMIO_R_CORE_1P1V_EN1
            .gpio = 124,
            .settings = {
                [GPIOMUX_ACTIVE]    = &cam_active_2_cfg,
                [GPIOMUX_SUSPENDED] = &cam_suspend_cfg,
            },
        },
        {   // CAMIO_R_CORE_1P1V_EN2
            .gpio = 15,
            .settings = {
                [GPIOMUX_ACTIVE]    = &cam_active_2_cfg,
                [GPIOMUX_SUSPENDED] = &cam_suspend_cfg,
            },
        },
#endif
#else
	{
		.gpio = 29,
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_active_2_cfg,
			[GPIOMUX_SUSPENDED] = &cam_suspend_cfg,
		},
	},
	{
		.gpio = 30,
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_active_1_cfg,
			[GPIOMUX_SUSPENDED] = &cam_suspend_cfg,
		},
	},
	{
		.gpio = 31,
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_active_2_cfg,
			[GPIOMUX_SUSPENDED] = &cam_suspend_cfg,
		},
	},
	{
		.gpio = 32,
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_active_5_cfg,
			[GPIOMUX_SUSPENDED] = &cam_suspend_cfg,
		},
	},
	{
		.gpio = 42,
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_active_2_cfg,
			[GPIOMUX_SUSPENDED] = &cam_suspend_cfg,
		},
	},
	{
		.gpio = 47,
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_active_3_cfg,
			[GPIOMUX_SUSPENDED] = &cam_suspend_cfg,
		},
	},
	{
		.gpio = 48,
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_active_3_cfg,
			[GPIOMUX_SUSPENDED] = &cam_suspend_cfg,
		},
	},
#if !(defined(CONFIG_PN544) && (defined(CONFIG_MACH_MSM8X60_EF39S) ||defined(CONFIG_MACH_MSM8X60_EF40S) ||defined(CONFIG_MACH_MSM8X60_EF40K) || defined(CONFIG_MACH_MSM8X60_EF65L)))
	{
		.gpio = 105,
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_active_4_cfg,
			[GPIOMUX_SUSPENDED] = &cam_suspend_cfg,
		},
	},
#endif
	{
		.gpio = 106,
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_active_4_cfg,
			[GPIOMUX_SUSPENDED] = &cam_suspend_cfg,
		},
	},
#endif
};

static struct msm_gpiomux_config msm8x60_charm_configs[] __initdata = {
	/* AP2MDM_WAKEUP */
	{
		.gpio = 135,
		.settings = {
			[GPIOMUX_SUSPENDED] = &ap2mdm_cfg,
		}
	},
	/* MDM2AP_VFR */
	{
		.gpio = 94,
		.settings = {
			[GPIOMUX_ACTIVE] = &mdm2ap_vfr_active_cfg,
			[GPIOMUX_SUSPENDED] = &mdm2ap_vfr_suspend_cfg,
		}
	},
	/* AP2MDM_STATUS */
	{
		.gpio = 136,
		.settings = {
			[GPIOMUX_SUSPENDED] = &ap2mdm_cfg,
		}
	},
	/* MDM2AP_STATUS */
	{
		.gpio = 134,
		.settings = {
			[GPIOMUX_SUSPENDED] = &mdm2ap_status_cfg,
		}
	},
	/* MDM2AP_WAKEUP */
	{
#if defined(CONFIG_MACH_MSM8X60_EF39S) || defined(CONFIG_MACH_MSM8X60_EF40S) || defined(CONFIG_MACH_MSM8X60_EF40K) || defined(CONFIG_MACH_MSM8X60_EF65L)
		.gpio = 45,
#else
		.gpio = 40,
#endif
		.settings = {
			[GPIOMUX_SUSPENDED] = &ap2mdm_cfg,
		}
	},
	/* MDM2AP_ERRFATAL */
	{
		.gpio = 133,
		.settings = {
			[GPIOMUX_SUSPENDED] = &mdm2ap_errfatal_cfg,
		}
	},
	/* AP2MDM_ERRFATAL */
	{
		.gpio = 93,
		.settings = {
			[GPIOMUX_SUSPENDED] = &ap2mdm_cfg,
		}
	},
	/* AP2MDM_KPDPWR_N */
	{
		.gpio = 132,
		.settings = {
			[GPIOMUX_SUSPENDED] = &ap2mdm_kpdpwr_n_cfg,
		}
	},
	/* AP2MDM_PMIC_RESET_N */
	{
		.gpio = 131,
		.settings = {
			[GPIOMUX_SUSPENDED] = &ap2mdm_kpdpwr_n_cfg,
		}
	}
};
static struct gpiomux_setting unused_gpio_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
	.dir = GPIOMUX_IN,
};

#if defined(CONFIG_MACH_MSM8X60_EF33S) || defined(CONFIG_MACH_MSM8X60_EF34K)
static struct msm_gpiomux_config msm8x60_ef33S34K_unused_gpio_configs[] __initdata = {
	{
		.gpio      = 0,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,	/* unused gpio pin */
		},
	},	
	{
		.gpio      = 1,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 2,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 3,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 4,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 5,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 6,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 7,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 8,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 9,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 10,		
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 11,	
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 12,	
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 13,	
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 14,		
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 15,		
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 16,		
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 17,		
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 18,		
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 19,
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 20,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 21,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 22,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 23,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 24,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 25,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 26,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 27,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 28,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 33,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 34,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 39,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 40,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 41,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 42,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 45,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 46,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 49,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 50,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 58,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 62,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 63,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 66,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 67,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 70,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 71,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 75,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 83,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 85,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 86,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 100,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 101,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 102,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 105,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 107,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 115,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 116,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 123,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 131,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 133,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 134,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 143,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 144,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 145,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 146,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 147,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 148,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 149,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 150,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 151,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 153,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 154,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 155,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 156,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 157,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 158,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 169,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 170,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 171,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 172,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	

};
#endif

#if defined(CONFIG_MACH_MSM8X60_EF35L)
static struct msm_gpiomux_config msm8x60_ef35L_unused_gpio_configs[] __initdata = {
	{
		.gpio      = 0,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,	/* unused gpio pin */
		},
	},	
	{
		.gpio      = 1,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 2,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 3,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 4,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 5,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 6,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 7,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 8,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 9,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 10,		
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 11,	
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 12,	
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 13,	
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 14,		
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 15,		
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 16,		
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 17,		
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 18,		
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 19,
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 20,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 21,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 22,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 23,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 24,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 25,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 26,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 27,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 28,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 33,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 34,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 35,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 36,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 37,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 38,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 39,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 40,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 41,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 45,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 46,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 49,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 50,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 58,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 62,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 63,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 66,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 67,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 70,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 71,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 74,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 75,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 77,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 82,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 85,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 86,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 96,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 100,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 101,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 102,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 105,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 107,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 115,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 116,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 123,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 130,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 131,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 133,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 134,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 141,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 142,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 143,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 144,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 146,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 147,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 148,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 149,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 150,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 151,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 153,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 154,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 155,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 156,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 157,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 158,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 169,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 170,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 171,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 172,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
};
#endif

#if defined(CONFIG_MACH_MSM8X60_EF39S)
static struct msm_gpiomux_config msm8x60_ef39S_unused_gpio_configs[] __initdata = {
	{
		.gpio      = 0,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,	/* unused gpio pin */
		},
	},	
	{
		.gpio      = 1,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 2,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 3,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 4,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 5,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 6,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 7,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 8,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 9,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 10,		
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 11,		
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 12,		
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 13,		
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 14,	
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 20,	
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 24,	
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 28,	
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 33,	
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 41,	
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 62,		
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 70,		
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 71,		
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 74,		
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 75,		
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 77,		
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 78,		
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 79,		
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 83,		
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 85,		
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 115,		
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 116,		
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 123,		
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 130,		
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
};
#endif

#if defined(CONFIG_MACH_MSM8X60_EF40S) || defined(CONFIG_MACH_MSM8X60_EF40K)
static struct msm_gpiomux_config msm8x60_ef40S40K_unused_gpio_configs[] __initdata = {
	{
		.gpio      = 0,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,	/* unused gpio pin */
		},
	},	
	{
		.gpio      = 1,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 2,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 3,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 4,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 5,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 6,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 7,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 8,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 9,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 10,		
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 11,	
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 12,	
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 14,	
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 20,		
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 21,		
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 22,		
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 24,		
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 28,		
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 41,		
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 70,
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 71,
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 74,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 75,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 77,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 78,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 79,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 83,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 85,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 115,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 116,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 123,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
};
#endif
#if defined(CONFIG_MACH_MSM8X60_EF65L)
static struct msm_gpiomux_config msm8x60_ef65L_unused_gpio_configs[] __initdata = {
	{
		.gpio      = 0,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,	/* unused gpio pin */
		},
	},	
	{
		.gpio      = 1,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 2,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 3,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 4,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 5,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 6,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 7,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 18,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 22,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 24,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 31,		
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 33,	
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 34,	
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 71,		
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 82,		
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 83,		
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 116,		
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 123,
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 130,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 141,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 142,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 153,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
	{
		.gpio      = 154,			
		.settings = {
			[GPIOMUX_ACTIVE]    = &unused_gpio_cfg,
			[GPIOMUX_SUSPENDED] = &unused_gpio_cfg,
		},
	},
};
#endif



struct msm_gpiomux_configs
msm8x60_surf_ffa_gpiomux_cfgs[] __initdata = {
	{msm8x60_gsbi_configs, ARRAY_SIZE(msm8x60_gsbi_configs)},
	{msm8x60_ebi2_configs, ARRAY_SIZE(msm8x60_ebi2_configs)},
	{msm8x60_uart_configs, ARRAY_SIZE(msm8x60_uart_configs)},
#if defined(CONFIG_USB_PEHCI_HCD) || defined(CONFIG_USB_PEHCI_HCD_MODULE)
	{msm8x60_isp_usb_configs, ARRAY_SIZE(msm8x60_isp_usb_configs)},
#endif
	{msm8x60_ts_configs, ARRAY_SIZE(msm8x60_ts_configs)},
	{msm8x60_aux_pcm_configs, ARRAY_SIZE(msm8x60_aux_pcm_configs)},
	{msm8x60_sdc_configs, ARRAY_SIZE(msm8x60_sdc_configs)},
	{msm8x60_snd_configs, ARRAY_SIZE(msm8x60_snd_configs)},
	{msm8x60_mi2s_configs, ARRAY_SIZE(msm8x60_mi2s_configs)},
	{msm8x60_lcdc_configs, ARRAY_SIZE(msm8x60_lcdc_configs)},
	{msm8x60_mdp_vsync_configs, ARRAY_SIZE(msm8x60_mdp_vsync_configs)},
	{msm8x60_hdmi_configs, ARRAY_SIZE(msm8x60_hdmi_configs)},
	{msm8x60_pmic_configs, ARRAY_SIZE(msm8x60_pmic_configs)},
	{msm8x60_common_configs, ARRAY_SIZE(msm8x60_common_configs)},
	{msm8x60_cam_configs, ARRAY_SIZE(msm8x60_cam_configs)},
	{msm8x60_tmg200_configs, ARRAY_SIZE(msm8x60_tmg200_configs)},
#if defined(CONFIG_PN544)
	{msm8x60_nfc_configs, ARRAY_SIZE(msm8x60_nfc_configs)},
#endif
	{NULL, 0},
};

struct msm_gpiomux_configs
msm8x60_fluid_gpiomux_cfgs[] __initdata = {
	{msm8x60_gsbi_configs, ARRAY_SIZE(msm8x60_gsbi_configs)},
	{msm8x60_fluid_gsbi_configs, ARRAY_SIZE(msm8x60_fluid_gsbi_configs)},
	{msm8x60_ebi2_configs, ARRAY_SIZE(msm8x60_ebi2_configs)},
	{msm8x60_uart_configs, ARRAY_SIZE(msm8x60_uart_configs)},
	{msm8x60_ts_configs, ARRAY_SIZE(msm8x60_ts_configs)},
	{msm8x60_aux_pcm_configs, ARRAY_SIZE(msm8x60_aux_pcm_configs)},
	{msm8x60_sdc_configs, ARRAY_SIZE(msm8x60_sdc_configs)},
	{msm8x60_snd_configs, ARRAY_SIZE(msm8x60_snd_configs)},
	{msm8x60_mi2s_configs, ARRAY_SIZE(msm8x60_mi2s_configs)},
	{msm8x60_lcdc_configs, ARRAY_SIZE(msm8x60_lcdc_configs)},
	{msm8x60_mdp_vsync_configs, ARRAY_SIZE(msm8x60_mdp_vsync_configs)},
	{msm8x60_hdmi_configs, ARRAY_SIZE(msm8x60_hdmi_configs)},
	{msm8x60_pmic_configs, ARRAY_SIZE(msm8x60_pmic_configs)},
	{msm8x60_common_configs, ARRAY_SIZE(msm8x60_common_configs)},
	{msm8x60_cam_configs, ARRAY_SIZE(msm8x60_cam_configs)},
	{msm8x60_tma300_configs, ARRAY_SIZE(msm8x60_tma300_configs)},
	{NULL, 0},
};

struct msm_gpiomux_configs
msm8x60_charm_gpiomux_cfgs[] __initdata = {
	{msm8x60_gsbi_configs, ARRAY_SIZE(msm8x60_gsbi_configs)},
	{msm8x60_uart_configs, ARRAY_SIZE(msm8x60_uart_configs)},
#ifdef CONFIG_MSM_GSBI9_UART
	{msm8x60_charm_uart_configs, ARRAY_SIZE(msm8x60_charm_uart_configs)},
#endif
	{msm8x60_ts_configs, ARRAY_SIZE(msm8x60_ts_configs)},
	{msm8x60_aux_pcm_configs, ARRAY_SIZE(msm8x60_aux_pcm_configs)},
	{msm8x60_sdc_configs, ARRAY_SIZE(msm8x60_sdc_configs)},
	{msm8x60_snd_configs, ARRAY_SIZE(msm8x60_snd_configs)},
	{msm8x60_mi2s_configs, ARRAY_SIZE(msm8x60_mi2s_configs)},
	{msm8x60_lcdc_configs, ARRAY_SIZE(msm8x60_lcdc_configs)},
	{msm8x60_mdp_vsync_configs, ARRAY_SIZE(msm8x60_mdp_vsync_configs)},
	{msm8x60_hdmi_configs, ARRAY_SIZE(msm8x60_hdmi_configs)},
	{msm8x60_pmic_configs, ARRAY_SIZE(msm8x60_pmic_configs)},
	{msm8x60_common_configs, ARRAY_SIZE(msm8x60_common_configs)},
	{msm8x60_cam_configs, ARRAY_SIZE(msm8x60_cam_configs)},
	{msm8x60_tmg200_configs, ARRAY_SIZE(msm8x60_tmg200_configs)},
	{msm8x60_charm_sdc_configs, ARRAY_SIZE(msm8x60_charm_sdc_configs)},
	{msm8x60_charm_configs, ARRAY_SIZE(msm8x60_charm_configs)},
#if defined(CONFIG_PN544)
	{msm8x60_nfc_configs, ARRAY_SIZE(msm8x60_nfc_configs)},
#endif
	{NULL, 0},
};

struct msm_gpiomux_configs
msm8x60_dragon_gpiomux_cfgs[] __initdata = {
	{msm8x60_gsbi_configs, ARRAY_SIZE(msm8x60_gsbi_configs)},
	{msm8x60_ebi2_configs, ARRAY_SIZE(msm8x60_ebi2_configs)},
	{msm8x60_uart_configs, ARRAY_SIZE(msm8x60_uart_configs)},
#if defined(CONFIG_USB_PEHCI_HCD) || defined(CONFIG_USB_PEHCI_HCD_MODULE)
	{msm8x60_isp_usb_configs, ARRAY_SIZE(msm8x60_isp_usb_configs)},
#endif
	{msm8x60_ts_configs, ARRAY_SIZE(msm8x60_ts_configs)},
	{msm8x60_aux_pcm_configs, ARRAY_SIZE(msm8x60_aux_pcm_configs)},
	{msm8x60_sdc_configs, ARRAY_SIZE(msm8x60_sdc_configs)},
	{msm8x60_snd_configs, ARRAY_SIZE(msm8x60_snd_configs)},
	{msm8x60_mi2s_configs, ARRAY_SIZE(msm8x60_mi2s_configs)},
	{msm8x60_lcdc_configs, ARRAY_SIZE(msm8x60_lcdc_configs)},
	{msm8x60_mdp_vsync_configs, ARRAY_SIZE(msm8x60_mdp_vsync_configs)},
	{msm8x60_hdmi_configs, ARRAY_SIZE(msm8x60_hdmi_configs)},
	{msm8x60_pmic_configs, ARRAY_SIZE(msm8x60_pmic_configs)},
	{msm8x60_common_configs, ARRAY_SIZE(msm8x60_common_configs)},
	{msm8x60_cam_configs, ARRAY_SIZE(msm8x60_cam_configs)},
	{msm8x60_tmg200_configs, ARRAY_SIZE(msm8x60_tmg200_configs)},
	{NULL, 0},
};

void __init msm8x60_init_gpiomux(struct msm_gpiomux_configs *cfgs)
{
	int rc;

	rc = msm_gpiomux_init(NR_GPIO_IRQS);
	if (rc) {
		pr_err("%s failure: %d\n", __func__, rc);
		return;
	}

	while (cfgs->cfg) {
		msm_gpiomux_install(cfgs->cfg, cfgs->ncfg);
		++cfgs;
	}
#if defined(CONFIG_MACH_MSM8X60_EF33S) || defined(CONFIG_MACH_MSM8X60_EF34K)	
	msm_gpiomux_install(msm8x60_ef33S34K_unused_gpio_configs, ARRAY_SIZE(msm8x60_ef33S34K_unused_gpio_configs));
#endif
#if defined(CONFIG_MACH_MSM8X60_EF40S) || defined(CONFIG_MACH_MSM8X60_EF40K)
	msm_gpiomux_install(msm8x60_ef40S40K_unused_gpio_configs, ARRAY_SIZE(msm8x60_ef40S40K_unused_gpio_configs));
#endif
#if defined(CONFIG_MACH_MSM8X60_EF35L)
	msm_gpiomux_install(msm8x60_ef35L_unused_gpio_configs, ARRAY_SIZE(msm8x60_ef35L_unused_gpio_configs));
#endif
#if defined(CONFIG_MACH_MSM8X60_EF39S)
	msm_gpiomux_install(msm8x60_ef39S_unused_gpio_configs, ARRAY_SIZE(msm8x60_ef39S_unused_gpio_configs));
#endif
#if defined(CONFIG_MACH_MSM8X60_EF65L)
	msm_gpiomux_install(msm8x60_ef65L_unused_gpio_configs, ARRAY_SIZE(msm8x60_ef65L_unused_gpio_configs));
#endif


}



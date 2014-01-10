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
#ifndef __ARCH_ARM_MACH_MSM_DEVICES_MSM8X60_H
#define __ARCH_ARM_MACH_MSM_DEVICES_MSM8X60_H

#define MSM_GSBI3_QUP_I2C_BUS_ID 0
#define MSM_GSBI4_QUP_I2C_BUS_ID 1
#define MSM_GSBI9_QUP_I2C_BUS_ID 2
#define MSM_GSBI8_QUP_I2C_BUS_ID 3
#define MSM_GSBI7_QUP_I2C_BUS_ID 4
#define MSM_GSBI12_QUP_I2C_BUS_ID 5
#define MSM_SSBI1_I2C_BUS_ID     6
#define MSM_SSBI2_I2C_BUS_ID     7
#define MSM_SSBI3_I2C_BUS_ID     8

#ifdef CONFIG_FB_MSM_MIPI_DSI_SAMSUNG // EF39S PANEL
#define MSM_LED_I2C_BUS_ID 10
#endif

#ifdef CONFIG_PANTECH_FB_MSM_MHL_SII9244
#define MSM_MHL_I2C_BUS_ID 11
#endif

#if defined(CONFIG_PN544) 
#if defined(CONFIG_MACH_MSM8X60_EF33S) || defined(CONFIG_MACH_MSM8X60_EF34K) || defined(CONFIG_MACH_MSM8X60_EF34C)
#define MSM_GSBI1_QUP_I2C_BUS_ID 11
#elif defined(CONFIG_MACH_MSM8X60_EF39S) ||defined(CONFIG_MACH_MSM8X60_EF40S) ||defined(CONFIG_MACH_MSM8X60_EF40K) || defined(CONFIG_MACH_MSM8X60_EF65L)
#define MSM_GSBI10_QUP_I2C_BUS_ID 21
#endif
#endif


#if defined (CONFIG_INPUT_SENSOR)
#if defined(CONFIG_MACH_MSM8X60_EF39S) || defined(CONFIG_MACH_MSM8X60_EF40K) || defined(CONFIG_MACH_MSM8X60_EF40S) || defined(CONFIG_MACH_MSM8X60_EF65L)
#if defined(CONFIG_SENSORS_MPU3050)
#define MSM_GSBI5_QUP_I2C_BUS_ID 33
#endif
#endif
#endif

#ifdef CONFIG_SKY_SMB136S_CHARGER
#define MSM_SMB_I2C_BUS_ID 39
#endif

#if defined(CONFIG_SKY_BATTERY_MAX17040) || defined(CONFIG_SKY_BATTERY_MAX17043)
#if defined(CONFIG_MACH_MSM8X60_EF39S) || defined(CONFIG_MACH_MSM8X60_EF40K) || defined(CONFIG_MACH_MSM8X60_EF40S) || defined(CONFIG_MACH_MSM8X60_EF65L)
#define MSM_GSBI11_QUP_I2C_BUS_ID	16
#else
#define MSM_GSBI11_QUP_I2C_BUS_ID	10 // ps2 team shs : fuel gauge porting
#endif
#endif

#ifdef CONFIG_SND_SOC_MSM8660_APQ
extern struct platform_device msm_pcm;
extern struct platform_device msm_pcm_routing;
extern struct platform_device msm_cpudai0;
extern struct platform_device msm_cpudai1;
extern struct platform_device msm_cpudai_hdmi_rx;
extern struct platform_device msm_cpudai_bt_rx;
extern struct platform_device msm_cpudai_bt_tx;
extern struct platform_device msm_cpudai_fm_rx;
extern struct platform_device msm_cpudai_fm_tx;
extern struct platform_device msm_cpu_fe;
extern struct platform_device msm_stub_codec;
extern struct platform_device msm_voice;
extern struct platform_device msm_voip;
extern struct platform_device msm_lpa_pcm;
extern struct platform_device msm_pcm_hostless;
#endif

#ifdef CONFIG_SPI_QUP
#if !(defined(CONFIG_PN544) && (defined(CONFIG_MACH_MSM8X60_EF33S) || defined(CONFIG_MACH_MSM8X60_EF34K) || defined(CONFIG_MACH_MSM8X60_EF34C)))
extern struct platform_device msm_gsbi1_qup_spi_device;
#endif
#if !(defined(CONFIG_PN544) && (defined(CONFIG_MACH_MSM8X60_EF39S) ||defined(CONFIG_MACH_MSM8X60_EF40S) ||defined(CONFIG_MACH_MSM8X60_EF40K) || defined(CONFIG_MACH_MSM8X60_EF65L)))
extern struct platform_device msm_gsbi10_qup_spi_device;
#endif
#ifdef CONFIG_SKY_DMB_SPI_HW
extern struct platform_device msm_gsbi2_qup_spi_device;
#endif
#endif

extern struct platform_device msm_bus_apps_fabric;
extern struct platform_device msm_bus_sys_fabric;
extern struct platform_device msm_bus_mm_fabric;
extern struct platform_device msm_bus_sys_fpb;
extern struct platform_device msm_bus_cpss_fpb;
extern struct platform_device msm_bus_def_fab;

extern struct platform_device msm_device_smd;
extern struct platform_device msm_device_gpio;
extern struct platform_device msm_device_vidc;
extern struct platform_device apq8064_msm_device_vidc;

extern struct platform_device msm_charm_modem;
extern struct platform_device msm_device_tz_log;
#ifdef CONFIG_HW_RANDOM_MSM
extern struct platform_device msm_device_rng;
#endif

void __init msm8x60_init_irq(void);
void __init msm8x60_check_2d_hardware(void);

#ifdef CONFIG_PANTECH_FB_MSM_MHL_SII9244  // MHL_KKCHO
int mhl_power_ctrl(int on);
#endif

#ifdef CONFIG_MSM_DSPS
extern struct platform_device msm_dsps_device;
#endif

#if defined(CONFIG_MSM_RPM_STATS_LOG)
extern struct platform_device msm_rpm_stat_device;
#endif
#endif

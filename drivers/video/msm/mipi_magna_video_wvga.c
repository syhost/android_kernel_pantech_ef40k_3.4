/* Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
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

static struct msm_panel_info pinfo;

// 325Mhz Own version.
static struct mipi_dsi_phy_ctrl dsi_video_mode_phy_db = {
	{0x03, 0x01, 0x01, 0x00},  /* regulator */
	/* timing */
	{0x49, 0x13, 0x18, 0x00, 0x20, 0x28, 0x0E, 0x18,
	0x08, 0x03, 0x04},
	{0x7f, 0x00, 0x00, 0x00},   /* phy ctrl */
	{0xee, 0x03, 0x86, 0x03},  /* strength */
	/* pll control */

//#define DSI_BIT_CLK_325MHZ
#define DSI_BIT_CLK_326MHZ

#if defined(DSI_BIT_CLK_325MHZ) 
	{0x41, 0x44, 0x31, 0xDA, 0x00, 0x50, 0x48, 0x63,
	0x31, 0x0f, 0x07,
	0x05, 0x14, 0x03, 0x03, 0x03, 0x54, 0x06, 0x10, 0x04, 0x03 },
#elif defined(DSI_BIT_CLK_326MHZ)
	{0x41, 0x45, 0x31, 0xDA, 0x00, 0x50, 0x48, 0x63,
	0x31, 0x0f, 0x07,
	0x05, 0x14, 0x03, 0x03, 0x03, 0x54, 0x06, 0x10, 0x04, 0x03 },
#else
#error DSI CLOCK is not define!
#endif
};


static int __init mipi_video_magna_wvga_init(void)
{
	int ret;

#ifdef CONFIG_FB_MSM_MIPI_PANEL_DETECT
	if (msm_fb_detect_client("mipi_video_magna_wvga"))
		return 0;
#endif

	pinfo.xres = 480;
	pinfo.yres = 800;
	pinfo.type = MIPI_VIDEO_PANEL;
	pinfo.pdest = DISPLAY_1;
	pinfo.wait_cycle = 0;
	pinfo.bpp = 24;
	pinfo.lcdc.h_back_porch = 12;
	pinfo.lcdc.h_front_porch = 34;
	pinfo.lcdc.h_pulse_width = 6;
	pinfo.lcdc.v_back_porch = 4;
	pinfo.lcdc.v_front_porch = 6;
	pinfo.lcdc.v_pulse_width = 2;
	pinfo.lcdc.border_clr = 0;	/* blk */
	pinfo.lcdc.underflow_clr = 0xff;	/* blue */
	pinfo.lcdc.hsync_skew = 0;
	pinfo.bl_max = 16;
	pinfo.bl_min = 1;
	pinfo.fb_num = 2;
#if defined(DSI_BIT_CLK_325MHZ) 
	pinfo.clk_rate = 325000000;
#elif defined(DSI_BIT_CLK_326MHZ)
#ifdef F_SKYDISP_BUG_FIX_CTS_FRAME_RATE
	pinfo.clk_rate = 318000000;
#else
	pinfo.clk_rate = 326000000;
#endif
#endif
	pinfo.lcd.vsync_enable = FALSE;
	pinfo.lcd.hw_vsync_mode = FALSE;

	pinfo.mipi.mode = DSI_VIDEO_MODE;
	pinfo.mipi.pulse_mode_hsa_he = TRUE;
	pinfo.mipi.hfp_power_stop = TRUE;
	pinfo.mipi.hbp_power_stop = FALSE;//TRUE;
	pinfo.mipi.hsa_power_stop = TRUE;
	pinfo.mipi.eof_bllp_power_stop = TRUE;
	pinfo.mipi.bllp_power_stop = FALSE;//TRUE;
	//pinfo.mipi.traffic_mode = DSI_NON_BURST_SYNCH_PULSE;
	pinfo.mipi.traffic_mode = DSI_NON_BURST_SYNCH_EVENT;
	pinfo.mipi.dst_format = DSI_VIDEO_DST_FORMAT_RGB888;
	pinfo.mipi.vc = 0;
	pinfo.mipi.rgb_swap = DSI_RGB_SWAP_BGR;
	pinfo.mipi.data_lane0 = TRUE;
	pinfo.mipi.data_lane1 = TRUE;
	pinfo.mipi.data_lane2 = FALSE;
	pinfo.mipi.data_lane3 = FALSE;
#if defined(DSI_BIT_CLK_325MHZ) 
	// 325Mhz
	pinfo.mipi.t_clk_post = 0x01;
	pinfo.mipi.t_clk_pre = 0x10;
#elif defined(DSI_BIT_CLK_326MHZ)
	// 326Mhz
	pinfo.mipi.t_clk_post = 0x01;
	pinfo.mipi.t_clk_pre = 0x10;
#endif
	pinfo.mipi.stream = 0; /* dma_p */
	pinfo.mipi.mdp_trigger = DSI_CMD_TRIGGER_SW;
	pinfo.mipi.dma_trigger = DSI_CMD_TRIGGER_SW;
	pinfo.mipi.frame_rate = 60;
	pinfo.mipi.dsi_phy_db = &dsi_video_mode_phy_db;

	ret = mipi_magna_device_register(&pinfo, MIPI_DSI_PRIM,
			MIPI_DSI_PANEL_WVGA);
	if (ret)
		printk(KERN_ERR "%s: failed to register device!\n", __func__);

	return ret;
}

module_init(mipi_video_magna_wvga_init);

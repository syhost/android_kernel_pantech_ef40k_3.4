/* Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora Forum nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * Alternatively, provided that this notice is retained in full, this software
 * may be relicensed by the recipient under the terms of the GNU General Public
 * License version 2 ("GPL") and only version 2, in which case the provisions of
 * the GPL apply INSTEAD OF those given above.  If the recipient relicenses the
 * software under the GPL, then the identification text in the MODULE_LICENSE
 * macro must be changed to reflect "GPLv2" instead of "Dual BSD/GPL".  Once a
 * recipient changes the license terms to the GPL, subsequent recipients shall
 * not relicense under alternate licensing terms, including the BSD or dual
 * BSD/GPL terms.  In addition, the following license statement immediately
 * below and between the words START and END shall also then apply when this
 * software is relicensed under the GPL:
 *
 * START
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 and only version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * END
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */
/*
 * MSM architecture driver to reset the modem
 */

#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/io.h>

#include "sky_sys_reset.h"
#include "smd_private.h"
#include <mach/msm_rpcrouter.h>
#include <mach/oem_rapi_client.h>
#include <mach/msm_smsm.h>
#include <mach/msm_iomap.h>
#if defined(CONFIG_PANTECH_DEBUG)
#ifdef CONFIG_PANTECH_DEBUG_SCHED_LOG  //p14291_pantech_dbg
#include <mach/pantech_debug.h> 
#endif
#endif

#define DEBUG
/* #undef DEBUG */
#ifdef DEBUG
#define D(x...) printk(x)
#else
#define D(x...) do {} while (0)
#endif

typedef enum { 
  USER_RESET = 0x00000000,
  SW_RESET = 0x00000001,
  PDL_RESET = 0x00000002,
}SYS_RST_RESET_TYPE_E;

typedef enum{
  MAIN_LCD_BL_OFF = 0x00000000,
  MAIN_LCD_BL_ON  = 0x0A0F090F,
}SYS_RST_LCD_BL_STATE_E;
 
typedef enum{
  RST_LCD_BL_OFF=0x00000000,
  RST_LCD_BL_ON =0x00000001,
  RST_LCD_BL_USER=0x00000002, 
}SYS_RST_LCD_BL_E;

#ifdef CONFIG_PANTECH_RESET_REASON
int sky_reset_reason = SYS_RESET_REASON_ABNORMAL;
int sky_prev_reset_reason = SYS_RESET_REASON_ABNORMAL;
static int silent_boot_mode = 0;
static int backlight_mode_before_reset = 0;

#ifdef CONFIG_PANTECH_LCD_SILENT_BOOT
static int backlight_mode = 0;
#endif
static int inited_reboot_info_flag = 0;
#endif


/*
* ** FUNCTION DEFINATION ***
*/
#ifdef CONFIG_PANTECH_RESET_REASON
void sky_sys_rst_init_reboot_info(void)
{ 
    void *restart_addr = NULL;
    unsigned int reboot_mode = 0;

    if( inited_reboot_info_flag != 0 )
        return;
    
    restart_addr = PANTECH_RESTART_REASON_ADDR;
    reboot_mode = __raw_readl(restart_addr);
    printk(KERN_INFO "[%s] reboot_mode: 0x%x, \n",__func__,reboot_mode);

    //get flag clear reset reason
    sky_prev_reset_reason = WHAT_SYS_RESET;
    printk(KERN_INFO "[%s] sky_prev_reset_reason: 0x%x, \n",__func__,sky_prev_reset_reason);

    //1. backlight flag set
    if( IS_BACKLIGHT_OFF_FLAG )
        backlight_mode_before_reset = 0;
    else
        backlight_mode_before_reset = 1;

    //2. silent boot mode set
    silent_boot_mode = IS_SYS_RESET;

#if defined(CONFIG_PANTECH_DEBUG)
    //3. pantech dbg mode set
    if( IS_RAMDUMP_FLAG )
        pantech_debug_enable = 1;
    else
        pantech_debug_enable = 0;
#endif

#if defined(CONFIG_PANTECH_DEBUG)
    printk(KERN_INFO "[%s] backlight_mode_before_reset: %i, silent_boot_mode: %i, pantech_debug_enable: %i\n", __func__, backlight_mode_before_reset, silent_boot_mode, pantech_debug_enable);
#else
    printk(KERN_INFO "[%s] backlight_mode_before_reset: %i, silent_boot_mode: %i\n", __func__, backlight_mode_before_reset, silent_boot_mode);
#endif

	//write reset reason default abnormal
    __raw_writel(SYS_RESET_REASON_ABNORMAL, restart_addr);
    sky_reset_reason = SYS_RESET_REASON_ABNORMAL;
    
    inited_reboot_info_flag = 1;
}
EXPORT_SYMBOL(sky_sys_rst_init_reboot_info);
#endif

#if defined(CONFIG_PANTECH_LCD_SILENT_BOOT) && defined(CONFIG_PANTECH_RESET_REASON)
uint8_t sky_sys_rst_get_silent_boot_mode(void)
{
  if( inited_reboot_info_flag == 0 )
    sky_sys_rst_init_reboot_info();

  return silent_boot_mode;
}
EXPORT_SYMBOL(sky_sys_rst_get_silent_boot_mode);

uint8_t sky_sys_rst_get_silent_boot_backlight(void)
{
  if( inited_reboot_info_flag == 0 )
    sky_sys_rst_init_reboot_info();

  printk(KERN_INFO "[%s] backlight show= %d\n",__func__,backlight_mode_before_reset);
  return backlight_mode_before_reset;
}
EXPORT_SYMBOL(sky_sys_rst_get_silent_boot_backlight);

void sky_sys_rst_set_silent_boot_backlight(int backlight)
{
  void *restart_addr = NULL;
  unsigned reboot_mode = 0;

  if( inited_reboot_info_flag == 0 )
    sky_sys_rst_init_reboot_info();

  backlight_mode = backlight;

  restart_addr = PANTECH_RESTART_REASON_ADDR;
  reboot_mode = __raw_readl(restart_addr);
  if( backlight == 0 )
  {
    reboot_mode |= SYS_RESET_BACKLIGHT_OFF_FLAG;
  }
  else
  {
    reboot_mode &= ~SYS_RESET_BACKLIGHT_OFF_FLAG;
  }
  __raw_writel(reboot_mode, restart_addr);

  printk(KERN_INFO "[%s] backlight store= %d, 0x%x\n",__func__,backlight,reboot_mode);
}
EXPORT_SYMBOL(sky_sys_rst_set_silent_boot_backlight);
#endif /* CONFIG_PANTECH_LCD_SILENT_BOOT */

#ifdef CONFIG_PANTECH_RESET_REASON
void sky_sys_rst_set_reboot_info(int reset_reason)
{
  void *restart_addr = NULL;

  if(sky_reset_reason != SYS_RESET_REASON_ABNORMAL){
      printk(KERN_INFO "[%s] skip set : previous reason = 0x%x\n",__func__, sky_reset_reason);
      return;
  }
  
  sky_reset_reason = reset_reason;
  
  restart_addr = PANTECH_RESTART_REASON_ADDR;

#ifdef CONFIG_PANTECH_LCD_SILENT_BOOT
  if( backlight_mode == 0 )
  {
    reset_reason |= SYS_RESET_BACKLIGHT_OFF_FLAG;
  }
  else
  {
    reset_reason &= ~SYS_RESET_BACKLIGHT_OFF_FLAG;
  }
#endif

  __raw_writel(reset_reason, restart_addr);  

  printk(KERN_INFO "[%s] reset_reason:0x%x\n",__func__,reset_reason);
}
EXPORT_SYMBOL(sky_sys_rst_set_reboot_info);
#endif

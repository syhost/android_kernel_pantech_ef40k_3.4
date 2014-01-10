#ifndef __ARCH_ARM_MACH_MSM_SKY_SYS_RESET_H
#define __ARCH_ARM_MACH_MSM_SKY_SYS_RESET_H

#ifdef CONFIG_PANTECH_RESET_REASON

#include <mach/msm_iomap.h>

#define QCOM_RESTART_REASON_ADDR   (MSM_IMEM_BASE + 0x65C)
#define PANTECH_RESTART_REASON_OFFSET 0x04
#define PANTECH_RESTART_REASON_ADDR   (QCOM_RESTART_REASON_ADDR + PANTECH_RESTART_REASON_OFFSET) //physical address : 0x2A05F660


/*******************************************************************************
**  RESET REASON DEFINE (Must Have vender cust_pantech.h == kernel sky_sys_reset) START
*******************************************************************************/
#define SYS_RESET_REASON_MASK                      0xBAAB0000
#define SYS_RESET_BACKLIGHT_OFF_FLAG               0x40000000
#define SYS_RESET_RAMDUMP_FLAG                     0x04000000
    
#define SYS_RESET_REASON_LINUX_MASK                0xBAAB1100
#define SYS_RESET_REASON_LINUX                     0xBAAB11E1
#define SYS_RESET_REASON_USERDATA_FS               0xBAAB11E2
    
#define SYS_RESET_REASON_WATCHDOG_MASK             0xBAAB2200
#define SYS_RESET_REASON_WATCHDOG                  0xBAAB22E1
#define SYS_RESET_REASON_WATCHDOG_XPU              0xBAAB22E2
    
#define SYS_RESET_REASON_ABNORMAL_MASK             0xBAAB3300
#define SYS_RESET_REASON_ABNORMAL                  0xBAAB33E1
    
#define SYS_RESET_REASON_MDM_MASK                  0xBAAB4400
#define SYS_RESET_REASON_MDM                       0xBAAB44E1
    
#define SYS_RESET_REASON_LPASS_MASK                0xBAAB5500
#define SYS_RESET_REASON_LPASS                     0xBAAB55E1
    
#define SYS_RESET_REASON_DSPS_MASK                 0xBAAB6600
#define SYS_RESET_REASON_DSPS                      0xBAAB66E1
    
#define SYS_RESET_REASON_RIVA_MASK                 0xBAAB7700
#define SYS_RESET_REASON_RIVA                      0xBAAB77E1
    
#define SYS_RESET_REASON_RPM_MASK                  0xBAAB8800
#define SYS_RESET_REASON_RPM_DOGBARK               0xBAAB88E1
#define SYS_RESET_REASON_RPM_ERRFATAL              0xBAAB88E2
    
#define SYS_RESET_REASON_AMSS_MASK                 0xBAAB9900
#define SYS_RESET_REASON_AMSS                      0xBAAB99E1
    
#define SYS_RESET_REASON_NORMAL_MASK               0xBAABCD00
#define SYS_RESET_REASON_NORMAL                    0xBAABCDDC
    
#define SYS_FLAG_SUM                               (SYS_RESET_BACKLIGHT_OFF_FLAG | SYS_RESET_RAMDUMP_FLAG)
    
#define IS_SYS_RESET_N_REBOOT                      (((*(unsigned int *)PANTECH_RESTART_REASON_ADDR)& ~SYS_FLAG_SUM) & ~(0xFFFF)) == (SYS_RESET_REASON_MASK)
#define IS_SYS_RESET                               ((((*(unsigned int *)PANTECH_RESTART_REASON_ADDR)& ~SYS_FLAG_SUM) & ~(0xFFFF)) == (SYS_RESET_REASON_MASK)) && \
                                                       ((((*(unsigned int *)PANTECH_RESTART_REASON_ADDR)& ~SYS_FLAG_SUM) & ~(0xFF)) != (SYS_RESET_REASON_NORMAL_MASK))
#define WHAT_SYS_RESET_GROUP                       ((*(unsigned int *)PANTECH_RESTART_REASON_ADDR)& ~SYS_FLAG_SUM) & ~(0xFF)
#define WHAT_SYS_RESET                             ((*(unsigned int *)PANTECH_RESTART_REASON_ADDR)& ~SYS_FLAG_SUM)
    
#define IS_BACKLIGHT_OFF_FLAG                      (((*(unsigned int *)PANTECH_RESTART_REASON_ADDR)& SYS_RESET_BACKLIGHT_OFF_FLAG)) == (SYS_RESET_BACKLIGHT_OFF_FLAG)
#define IS_RAMDUMP_FLAG                            (((*(unsigned int *)PANTECH_RESTART_REASON_ADDR)& SYS_RESET_RAMDUMP_FLAG)) == (SYS_RESET_RAMDUMP_FLAG)
/*******************************************************************************
**  RESET REASON DEFINE (Must Have vender cust_pantech.h == kernel sky_sys_reset) END
*******************************************************************************/

extern int sky_reset_reason;
extern int sky_prev_reset_reason;

extern void sky_sys_rst_init_reboot_info(void);
#ifdef CONFIG_PANTECH_LCD_SILENT_BOOT
extern uint8_t sky_sys_rst_get_silent_boot_mode(void);
extern uint8_t sky_sys_rst_get_silent_boot_backlight(void);
extern void  sky_sys_rst_set_silent_boot_backlight(int backlight);
#endif
extern void sky_sys_rst_set_reboot_info(int reset_reason);

#endif
#endif
// CONFIG_PANTECH_RESET_REASON


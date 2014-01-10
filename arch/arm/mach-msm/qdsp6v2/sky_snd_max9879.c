/************************************************************************************************
**
**    AUDIO EXTERNAL AMP
**
**    FILE
**        Sky_snd_max9879.c
**
**    DESCRIPTION
**        This file contains audio external amp api
**          
**          void snd_extamp_api_Init()
**          void snd_extamp_api_SetPath()
**          void snd_extamp_api_SetVolume()
**          void snd_extamp_api_Sleep()
**
**    Copyright (c) 2010 by PANTECH Incorporated.  All Rights Reserved.
*************************************************************************************************/


/************************************************************************************************
** Includes
*************************************************************************************************/
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <mach/gpio.h>
//#include "dal.h"
//#include "dal_audio.h"
#include "sky_snd_max9879.h"
#include "sky_snd_audio.h"

#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/msm_audio.h>
#include <mach/board.h>

#ifdef FEATURE_SKY_SND_AUDIO_TEST_COMMAND
#include <asm/ioctls.h>
#include <linux/uaccess.h>
#endif

/************************************************************************************************
** Definition
*************************************************************************************************/
/* Default Register Value */
#define DEFAULT_INPUT_MODE_CONTROL_REG_VAL  0x00
#define DEFAULT_SPEAKER_VOLUME_CONTROL_REG_VAL  0x00
#define DEFAULT_LEFT_HEADPHONE_VOLUME_CONTROL_REG_VAL 0x00
#define DEFAULT_RIGHT_HEADPHONE_VOLUME_CONTROL_REG_VAL 0x00
#define DEFAULT_OUTPUT_MODE_CONTROL_REG_VAL OUTPUT_SHDN_POWER_DOWN | OUTPUT_BYPASS_ENABLE
/************************************************************************************************
** Variables
*************************************************************************************************/
static extamp_info_t tExtampInfo;
static unsigned char bHeadphonePath = 0;
static struct i2c_client *extamp_i2c_client = NULL;
static int CurrDeviceId = -1;
static int CurrDevicePwrStatus = 0;
static int ExtAmpBlock = 0;

#ifdef FEATURE_SKY_SND_AUDIO_TEST_COMMAND
#define SND_AMP_IOCTL_MAGIC 'a'
#define SND_SKY_EXTAMP_WRITE  _IOW(SND_AMP_IOCTL_MAGIC, 0, unsigned)
#define SND_SKY_EXTAMP_READ  _IOW(SND_AMP_IOCTL_MAGIC, 1, unsigned)
#define SND_SKY_EXTAMP_SPK_CH_CTRL  _IOW(SND_AMP_IOCTL_MAGIC, 2, unsigned)
#define SND_SKY_EXTAMP_SET_MODE  _IOW(SND_AMP_IOCTL_MAGIC, 3, unsigned)
#define SND_SKY_SET_CALLMODE _IOW(SND_AMP_IOCTL_MAGIC, 4, unsigned)
#define SND_SKY_GET_CALLMODE _IOW(SND_AMP_IOCTL_MAGIC, 5, unsigned)
#define SND_SKY_SET_MIC_MUTE_CTRL _IOW(SND_AMP_IOCTL_MAGIC, 6, unsigned)
#define SND_SKY_SET_AUTOANSWER _IOW(SND_AMP_IOCTL_MAGIC, 7, unsigned)
#if 1//SangwonLee 110330 Right Speaker is near by handset mic. Disable it to prevent recording noise.
#define SND_SKY_SET_RSPK_OFF _IOW(SND_AMP_IOCTL_MAGIC, 9, unsigned)
#endif
static int ex_exception_spk_ch;
static int exception_spk_ch;
static int current_reg;
static int current_regval;

static int curMode;
#ifdef MVS_AMR_DUMP
#define SND_SKY_DUMP_ENABLE _IOW(SND_AMP_IOCTL_MAGIC, 8, unsigned)
#endif
#endif

static call_mode_param  curCallMode;
#ifdef CONFIG_SKY_SND_MVS
call_mode_e g_CallMode;  //20130711 jhsong : check vt / voip other srcs in kernel
#endif
extern int get_pcm_in_status(void); // SangwonLee 110330 Do not turn on right spk when rec.
#if 1//SangwonLee 110330 Right Speaker is near by handset mic. Disable it to prevent recording noise.
extern void set_pcm_in_status(void); // SangwonLee 110330 Do not turn on right spk when rec.
#endif

extern int get_pcm_in_status(void); // SangwonLee 110330 Do not turn on right spk when rec.
extern void set_pcm_in_exception(int value);    // SangwonLee 110330 To prevent record Enforced Audible Sound.
/************************************************************************************************
** Declaration
*************************************************************************************************/
static int snd_extamp_i2c_write(u8 reg, u8 data);
/*static int snd_extamp_i2c_read(u8 reg, u8 *data);*/
static int max9879_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int __exit max9879_remove(struct i2c_client *client);
static void max9879_shutdown(struct i2c_client *client);

#if !defined(CONFIG_MACH_MSM8X60_EF34K) && !defined(CONFIG_MACH_MSM8X60_EF35L) && !defined (CONFIG_MACH_MSM8X60_EF33S)
/* static */ void snd_extamp_setpath(extamp_inmode_e inpath, extamp_outmode_e outpath, extamp_outfmt_e outfmt);
static void snd_extamp_setvolume(extamp_outmode_e outpath, uint32_t volume);
static void snd_extamp_setpreampgain(extamp_inmode_e inpath, u8 val);
static void snd_extamp_sleep(u8 enable);
#endif

static int snd_extamp_write_all_reg(extamp_info_t tCurrentExtampInfo);

#ifdef FEATURE_SKY_SND_MISC_DEVICE
static int snd_misc_device_init(void);
static int snd_misc_device_deinit(void);
static int ext_amp_open(struct inode *inode, struct file *file);
static int ext_amp_release(struct inode *inode, struct file *file);
static long ext_amp_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
#endif

#ifdef FEATURE_SKY_SND_AUDIO_TEST_COMMAND
static int snd_extamp_read_single_reg(int reg);
static void snd_extamp_write_single_reg(int reg, int regval);
static bool snd_extamp_exception_check(void);
#endif

/*==========================================================================
** max9879_probe
**=========================================================================*/
static int max9879_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
 	int rc = 0;
	int status = 0;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		extamp_i2c_client = NULL;
		rc = -1;
	}
	else 
	{
		extamp_i2c_client = client;
		
		tExtampInfo.in_mode_ctrl = DEFAULT_INPUT_MODE_CONTROL_REG_VAL;
		tExtampInfo.spk_vol_ctrl = DEFAULT_SPEAKER_VOLUME_CONTROL_REG_VAL;
		tExtampInfo.hpl_vol_ctrl = DEFAULT_LEFT_HEADPHONE_VOLUME_CONTROL_REG_VAL;
		tExtampInfo.hpr_vol_ctrl = DEFAULT_RIGHT_HEADPHONE_VOLUME_CONTROL_REG_VAL; 
		tExtampInfo.out_mode_ctrl = DEFAULT_OUTPUT_MODE_CONTROL_REG_VAL; 

		/* Init MAX9877 register */
              status = snd_extamp_write_all_reg(tExtampInfo);
              if(status!=0) rc = -1;
	}
	AMP_DBG_HIGH("External AMP status:%d -> rc:%d", status, rc);
	return rc;
    
}

/*==========================================================================
** max9879_remove
**=========================================================================*/
static int __exit max9879_remove(struct i2c_client *client)
{
	int rc = 0;
	extamp_i2c_client = NULL;
	/*rc = i2c_detach_client(client);*/
	return rc;
}

static int max9879_suspend(struct device *dev)
{
      unsigned char SuspendOk = 0;

      if((tExtampInfo.spk_vol_ctrl == VOL_M_MUTE) &&
        (tExtampInfo.hpl_vol_ctrl == VOL_M_MUTE) &&
        (tExtampInfo.hpr_vol_ctrl == VOL_M_MUTE))   SuspendOk = 1;

       if(SuspendOk) {
           tExtampInfo.out_mode_ctrl |= OUTPUT_BYPASS_ENABLE;
   	   snd_extamp_i2c_write(OUTPUT_MODE_CTRL_REG, tExtampInfo.out_mode_ctrl);
       }
	AMP_DBG_MED("AMP suspend(%d)", SuspendOk);
	
	return 0;
}

static int max9879_resume(struct device *dev)
{
	AMP_DBG_MED("AMP resume()");
	return 0;
}

static void max9879_shutdown(struct i2c_client *client)
{
	tExtampInfo.out_mode_ctrl &= ~OUTPUT_SHDN_POWER_ON;
	
	snd_extamp_i2c_write(OUTPUT_MODE_CTRL_REG, tExtampInfo.out_mode_ctrl);
	AMP_DBG_MED("AMP Shutdown for power-off");
}

static const struct i2c_device_id max9879_id[] = {
	{ "max9879-amp", MAX9879_SLAVE_ADDR},
};

static struct dev_pm_ops i2c_device_max9879_pm_ops = {
             .suspend = max9879_suspend, 
             .resume = max9879_resume,
};

static struct i2c_driver max9879_driver = {
	.id_table = max9879_id,
	.probe  = max9879_probe,
	.remove = __exit_p(max9879_remove),
	.shutdown = max9879_shutdown,
	.driver = {
		.name = "max9879-amp",
                .pm = &i2c_device_max9879_pm_ops,
	},
};

/*==========================================================================
** snd_extamp_get_current_callmode
**=========================================================================*/
int snd_extamp_get_current_callmode(void)
{
       int result=curCallMode.mode;
       return result;
}
/*==========================================================================
** snd_extamp_get_mode
**=========================================================================*/
int snd_extamp_get_mode(void)
{
       int result=curMode;
       return result;
}
/*==========================================================================
** snd_extamp_api_reset
**=========================================================================*/
void snd_extamp_api_reset(void)
{
       curCallMode.mode= MODE_DEFAULT;
       curCallMode.param=0;
       curMode=0;
#ifdef CONFIG_SKY_SND_MVS
	g_CallMode = MODE_DEFAULT;
#endif
       AMP_DBG_HIGH("Rest max9879 parameters.");
}
/*==========================================================================
** snd_extamp_api_Init
**=========================================================================*/
void snd_extamp_api_Init(void)
{
	int result = 0;

	AMP_DBG_HIGH("Init max9879");

#ifdef FEATURE_SKY_SND_AUDIO_TEST_COMMAND
       ex_exception_spk_ch=-1;
       exception_spk_ch=0;
       current_reg=0;
       current_regval=0;
#endif
       
#ifdef FEATURE_SKY_SND_MISC_DEVICE
       snd_misc_device_init();
#endif
        
	result = i2c_add_driver(&max9879_driver);
	if(result){
		AMP_ERR("Init max9879 Fail");
	}
}

/*==========================================================================
** snd_extamp_api_DeInit
**=========================================================================*/
void snd_extamp_api_DeInit(void)
{
#ifdef FEATURE_SKY_SND_MISC_DEVICE
       snd_misc_device_deinit();
#endif
	i2c_del_driver(&max9879_driver);
	AMP_DBG_HIGH("DeInit max9879");
}

/*==========================================================================
** snd_extamp_exception_check
**=========================================================================*/
static bool snd_extamp_exception_check(void)
{
    if(ex_exception_spk_ch!=exception_spk_ch)   {
        AMP_DBG_HIGH("snd_extamp_exception_check (ex:%d, cur:%d)", ex_exception_spk_ch, exception_spk_ch);
        return true;
    }
    return false;
}
/*==========================================================================
** snd_extamp_exception_check
**=========================================================================*/
#if !defined(CONFIG_MACH_MSM8X60_EF34K) && !defined(CONFIG_MACH_MSM8X60_EF35L) && !defined (CONFIG_MACH_MSM8X60_EF33S)
static bool snd_extamp_skip_exception_check(int wParam)
{
    if(!wParam && curMode==MODE_IN_CALL)        {
        AMP_DBG_HIGH("snd_extamp_skip_exception_check - forbid in-call amp off");
        return true;
    }
    return false;
}
#endif
/*==========================================================================
** snd_extamp_write_all_reg
**=========================================================================*/
static void snd_extamp_exception_ctrl(extamp_info_t *tCurrentExtampInfo)
{
    if(exception_spk_ch)
    {
        AMP_DBG_HIGH("snd_extamp_exception_ctrl - exception_spk_ch:%d", exception_spk_ch);
        tCurrentExtampInfo->out_mode_ctrl&=~(OUTPUT_RIGHT_SPEAKER_ENABLE|OUTPUT_LEFT_SPEAKER_ENABLE);

        if(exception_spk_ch&0x1)   tCurrentExtampInfo->out_mode_ctrl|=OUTPUT_RIGHT_SPEAKER_ENABLE;
        if(exception_spk_ch&0x2)   tCurrentExtampInfo->out_mode_ctrl|=OUTPUT_LEFT_SPEAKER_ENABLE;
    }
    ex_exception_spk_ch=exception_spk_ch;
}
/*==========================================================================
** snd_extamp_write_all_reg
**=========================================================================*/
static int snd_extamp_write_all_reg(extamp_info_t tCurrentExtampInfo)
{
    int reg;
    int status[MAX9879_REG_NUM] = {0,0,0,0,0};
    int write_status=0;

    snd_extamp_exception_ctrl(&tCurrentExtampInfo);
    
    if(tExtampInfo.in_mode_ctrl != tCurrentExtampInfo.in_mode_ctrl)	{
        status[INPUT_MODE_CTRL_REG] = snd_extamp_i2c_write(INPUT_MODE_CTRL_REG, tCurrentExtampInfo.in_mode_ctrl);
        AMP_DBG_LOW("snd_extamp_write_all_reg (INPUT_MODE_CTRL_REG : 0x%x )", tCurrentExtampInfo.in_mode_ctrl);
        write_status |= 0x1<<INPUT_MODE_CTRL_REG;
    }
    if(tExtampInfo.spk_vol_ctrl != tCurrentExtampInfo.spk_vol_ctrl)	{
        status[SPK_VOL_CTRL_REG] = snd_extamp_i2c_write(SPK_VOL_CTRL_REG, tCurrentExtampInfo.spk_vol_ctrl );
        AMP_DBG_LOW("snd_extamp_write_all_reg (SPK_VOL_CTRL_REG : 0x%x )", tCurrentExtampInfo.spk_vol_ctrl);
        write_status |= 0x1<<SPK_VOL_CTRL_REG;
    }
    if(tExtampInfo.hpl_vol_ctrl != tCurrentExtampInfo.hpl_vol_ctrl)	{
        status[LHP_VOL_CTRL_REG] = snd_extamp_i2c_write(LHP_VOL_CTRL_REG, tCurrentExtampInfo.hpl_vol_ctrl);
        AMP_DBG_LOW("snd_extamp_write_all_reg (LHP_VOL_CTRL_REG : 0x%x )", tCurrentExtampInfo.hpl_vol_ctrl);
        write_status |= 0x1<<LHP_VOL_CTRL_REG;
    }
    if(tExtampInfo.hpr_vol_ctrl != tCurrentExtampInfo.hpr_vol_ctrl)	{
        status[RHP_VOL_CTRL_REG] = snd_extamp_i2c_write(RHP_VOL_CTRL_REG, tCurrentExtampInfo.hpr_vol_ctrl);
        AMP_DBG_LOW("snd_extamp_write_all_reg (RHP_VOL_CTRL_REG : 0x%x )", tCurrentExtampInfo.hpr_vol_ctrl);        
        write_status |= 0x1<<RHP_VOL_CTRL_REG;
    }
    if(tExtampInfo.out_mode_ctrl != tCurrentExtampInfo.out_mode_ctrl)	{
        status[OUTPUT_MODE_CTRL_REG] = snd_extamp_i2c_write(OUTPUT_MODE_CTRL_REG, tCurrentExtampInfo.out_mode_ctrl);
        AMP_DBG_LOW("snd_extamp_write_all_reg (OUTPUT_MODE_CTRL_REG : 0x%x )", tCurrentExtampInfo.out_mode_ctrl);        
        write_status |= 0x1<<OUTPUT_MODE_CTRL_REG;
    }

    memcpy(&tExtampInfo, &tCurrentExtampInfo, sizeof(extamp_info_t));

    for(reg=0;reg<MAX9879_REG_NUM;reg++)	{
        if(status[reg])	{
            AMP_ERR("snd_extamp_write_all_reg Fail (%d Reg.)", reg);
            return -1;
        }
    }

    if(write_status!=0)        AMP_DBG_MED("snd_extamp_write_all_reg (Write Status:%x)", write_status);
    else        AMP_DBG_MED("snd_extamp_write_all_reg (No change)  ");

    return 0;
}

/*==========================================================================
** snd_extamp_make_current
**=========================================================================*/
void snd_extamp_make_current(extamp_info_t* tCurrentExtampInfo)
{
    memcpy(tCurrentExtampInfo, &tExtampInfo, sizeof(extamp_info_t));
}

/*==========================================================================
** snd_extamp_api_SetDevice
**=========================================================================*/
void snd_extamp_api_SetDevice(int on, uint32_t cad_device)
{
    extamp_info_t tCurrentExtampInfo;
    int result;

    if(CurrDeviceId == cad_device && CurrDevicePwrStatus == on && !snd_extamp_exception_check())        return;
    if(ExtAmpBlock) return;

    //if(snd_extamp_skip_exception_check(on)) return;

    #if 0
    CurrDeviceId = cad_device;
    CurrDevicePwrStatus = on;
    AMP_DBG_MED("Set max9879 on(%d), path(%d)", on, cad_device);
    #else
    //20110425-jhpark : Rx mute ...in auto-answer mode
    if(on && curCallMode.mode == MODE_AUTOANSWER)
    {
        if(curCallMode.param)
        {
            CurrDeviceId = cad_device;
            CurrDevicePwrStatus = on;
            cad_device = SND_DEVICE_MUTE_RX;
            AMP_DBG_MED("MODE_AUTOANSWER  ON      CurrDeviceId(%d),  cad_device(%d)", CurrDeviceId, cad_device);
        }
        else
        {
            cad_device = CurrDeviceId;
            curCallMode.mode = MODE_DEFAULT;
#ifdef CONFIG_SKY_SND_MVS
	     g_CallMode = MODE_DEFAULT;
#endif
            AMP_DBG_MED("MODE_AUTOANSWER  OFF      CurrDeviceId(%d)", CurrDeviceId);
        }
    }else {
        CurrDeviceId = cad_device;
        CurrDevicePwrStatus = on;
        AMP_DBG_MED("Set max9879 on(%d), path(%d)", on, cad_device);
    }
    #endif
   
    snd_extamp_make_current(&tCurrentExtampInfo);

    switch(cad_device)
    {
        case SND_DEVICE_HANDSET_RX:
            tCurrentExtampInfo.out_mode_ctrl = OUTPUT_SHDN_POWER_DOWN | OUTPUT_BYPASS_ENABLE;
        break;
        case SND_DEVICE_HEADSET_RX:
        if (on)
        {
            tCurrentExtampInfo.in_mode_ctrl  = IN_INB_STEREO_SINGLE | IN_PGAINB_0_DB;
            tCurrentExtampInfo.spk_vol_ctrl = VOL_M_MUTE;
#if defined (CONFIG_MACH_MSM8X60_EF35L)
            tCurrentExtampInfo.hpl_vol_ctrl = tCurrentExtampInfo.hpr_vol_ctrl = VOL_M_9_DB;
#else
            tCurrentExtampInfo.hpl_vol_ctrl = tCurrentExtampInfo.hpr_vol_ctrl = VOL_M_11_DB;
#endif
            tCurrentExtampInfo.out_mode_ctrl = OUTPUT_SHDN_POWER_ON | OUTPUT_INB_ENABLE | OUTPUT_HEADPHONE_AMP_ENABLE;
        }
        else
        {
            tCurrentExtampInfo.in_mode_ctrl  = IN_INB_STEREO_SINGLE | IN_PGAINB_0_DB;
            tCurrentExtampInfo.spk_vol_ctrl = VOL_M_MUTE;
            tCurrentExtampInfo.hpl_vol_ctrl = tCurrentExtampInfo.hpr_vol_ctrl = VOL_M_MUTE;
            tCurrentExtampInfo.out_mode_ctrl = OUTPUT_SHDN_POWER_DOWN | OUTPUT_INB_DISABLE | OUTPUT_HEADPHONE_AMP_DISABLE;
        }
        break;

        case SND_DEVICE_SPEAKER_RX:
        if (on)
        {
            if(curMode==MODE_IN_CALL)     tCurrentExtampInfo.in_mode_ctrl  = IN_INA_STEREO_SINGLE | IN_PGAINA_5P_5_DB;
            else tCurrentExtampInfo.in_mode_ctrl  = IN_INA_STEREO_SINGLE | IN_PGAINA_0_DB;
#if defined (CONFIG_MACH_MSM8X60_EF35L)
            tCurrentExtampInfo.spk_vol_ctrl = VOL_M_5_DB;
#else
            tCurrentExtampInfo.spk_vol_ctrl = VOL_M_3_DB;
#endif
            tCurrentExtampInfo.hpl_vol_ctrl = tCurrentExtampInfo.hpr_vol_ctrl = VOL_M_MUTE;
            tCurrentExtampInfo.out_mode_ctrl = OUTPUT_SHDN_POWER_ON | OUTPUT_INA_ENABLE | OUTPUT_LEFT_SPEAKER_ENABLE | OUTPUT_RIGHT_SPEAKER_ENABLE;
            if(curMode==MODE_IN_CALL || get_pcm_in_status()) {  // SangwonLee 110319 Right Speaker is near by handset mic. Disable it to prevent echo and noise.
                tCurrentExtampInfo.out_mode_ctrl&=~OUTPUT_RIGHT_SPEAKER_ENABLE;
                AMP_DBG_HIGH("Turn off R-SPK");
            }
        }
        else
        {
            tCurrentExtampInfo.in_mode_ctrl  = IN_INA_STEREO_SINGLE | IN_PGAINA_0_DB;
            tCurrentExtampInfo.spk_vol_ctrl = VOL_M_MUTE;
            tCurrentExtampInfo.hpl_vol_ctrl = tCurrentExtampInfo.hpr_vol_ctrl = VOL_M_MUTE;
            tCurrentExtampInfo.out_mode_ctrl = OUTPUT_SHDN_POWER_DOWN | OUTPUT_INA_DISABLE | OUTPUT_LEFT_SPEAKER_DISABLE | OUTPUT_RIGHT_SPEAKER_DISABLE;
        }
        break;
        
        case SND_DEVICE_SPEAKER_HEADSET_RX:
        if (on)
        {
            tCurrentExtampInfo.in_mode_ctrl  = IN_INA_STEREO_SINGLE | IN_INB_STEREO_SINGLE | IN_PGAINA_0_DB | IN_PGAINB_0_DB;
#if defined (CONFIG_MACH_MSM8X60_EF35L)
            tCurrentExtampInfo.spk_vol_ctrl = VOL_M_5_DB;
            tCurrentExtampInfo.hpl_vol_ctrl = tCurrentExtampInfo.hpr_vol_ctrl = VOL_M_9_DB;
#else
            tCurrentExtampInfo.spk_vol_ctrl = VOL_M_3_DB;
            tCurrentExtampInfo.hpl_vol_ctrl = tCurrentExtampInfo.hpr_vol_ctrl = VOL_M_11_DB;
#endif
            tCurrentExtampInfo.out_mode_ctrl = OUTPUT_SHDN_POWER_ON | OUTPUT_INA_ENABLE | OUTPUT_INB_ENABLE
                                                                    | OUTPUT_HEADPHONE_AMP_ENABLE
                                                                    | OUTPUT_LEFT_SPEAKER_ENABLE | OUTPUT_RIGHT_SPEAKER_ENABLE;
            if(get_pcm_in_status()) {  // SangwonLee 110330 Right Speaker is near by handset mic. Disable it to prevent recording noise.
                tCurrentExtampInfo.out_mode_ctrl&=~OUTPUT_RIGHT_SPEAKER_ENABLE;
                AMP_DBG_HIGH("Turn off R-SPK");
            }
        }
        else
        {
            tCurrentExtampInfo.in_mode_ctrl  = IN_PGAINA_0_DB | IN_PGAINB_0_DB;
            tCurrentExtampInfo.spk_vol_ctrl = VOL_M_MUTE;
            tCurrentExtampInfo.hpl_vol_ctrl = tCurrentExtampInfo.hpr_vol_ctrl = VOL_M_MUTE;
            tCurrentExtampInfo.out_mode_ctrl = OUTPUT_SHDN_POWER_DOWN | OUTPUT_INA_DISABLE | OUTPUT_INB_DISABLE
                                                                    | OUTPUT_HEADPHONE_AMP_DISABLE
                                                                    | OUTPUT_LEFT_SPEAKER_DISABLE | OUTPUT_RIGHT_SPEAKER_DISABLE;
        }
        break;
        
        case SND_DEVICE_MUTE_RX:
        if (on)
        {
            tCurrentExtampInfo.in_mode_ctrl  = IN_PGAINA_0_DB | IN_PGAINB_0_DB;
            tCurrentExtampInfo.spk_vol_ctrl = VOL_M_MUTE;
            tCurrentExtampInfo.hpl_vol_ctrl = tCurrentExtampInfo.hpr_vol_ctrl = VOL_M_MUTE;
            tCurrentExtampInfo.out_mode_ctrl = tExtampInfo.out_mode_ctrl;
            tCurrentExtampInfo.out_mode_ctrl &= ~(OUTPUT_BYPASS_ENABLE);
        }
        break;
    
    default:
    break;
    }

    result = snd_extamp_write_all_reg(tCurrentExtampInfo); 

}

/*==========================================================================
** snd_extamp_api_SetVolume
**=========================================================================*/
void snd_extamp_api_SetVolume(unsigned char hp_vol, unsigned char sp_vol)
{
    extamp_info_t tCurrentExtampInfo;
    int result;

    AMP_DBG_MED("Set Volume hp_vol(%d), sp_vol(%d)", hp_vol, sp_vol);

    snd_extamp_make_current(&tCurrentExtampInfo);

    switch(CurrDeviceId){
        case SND_DEVICE_HANDSET_RX:
        break;
        case SND_DEVICE_HEADSET_RX:
            tCurrentExtampInfo.hpl_vol_ctrl  = hp_vol;
            tCurrentExtampInfo.hpr_vol_ctrl  = hp_vol;
        break;
        case SND_DEVICE_SPEAKER_RX:
            tCurrentExtampInfo.spk_vol_ctrl  = sp_vol;
        break;
        case SND_DEVICE_SPEAKER_HEADSET_RX:
            tCurrentExtampInfo.hpl_vol_ctrl  = hp_vol;
            tCurrentExtampInfo.hpr_vol_ctrl  = hp_vol;
            tCurrentExtampInfo.spk_vol_ctrl  = sp_vol;
        break;

        default:
        break;
    }
    result = snd_extamp_write_all_reg(tCurrentExtampInfo); 	
}

/*==========================================================================
** snd_extamp_api_HeadsetConnected
**=========================================================================*/
void snd_extamp_api_HeadsetConnected(uint32_t connect)
{
	bHeadphonePath = connect;
}

/*==========================================================================
** snd_extamp_api_SetPreAmpGain
**=========================================================================*/
void snd_extamp_api_SetPreAmpGain(uint32_t gain)
{
    #if 0   // Temporary.
    extamp_info_t tCurrentExtampInfo;
    int result;

    AUD_ERR("Set PreAmpGain gain(%d)", gain);

    snd_extamp_make_current(&tCurrentExtampInfo);

    switch(CurrDeviceId){
        case SND_DEVICE_HANDSET_RX:
        break;
        case SND_DEVICE_HEADSET_RX:
            tCurrentExtampInfo.inp_gain_ctl_reg_val  &= ~IN_PGAINA_MASK;
            tCurrentExtampInfo.inp_gain_ctl_reg_val  = tCurrentExtampInfo.inp_gain_ctl_reg_val |(gain<<2);

        break;
        case SND_DEVICE_SPEAKER_RX:
            tCurrentExtampInfo.inp_gain_ctl_reg_val  &= ~IN_PGAINB_MASK;
            tCurrentExtampInfo.inp_gain_ctl_reg_val  = tCurrentExtampInfo.inp_gain_ctl_reg_val |gain;

        break;
        case SND_DEVICE_SPEAKER_HEADSET_RX:
            tCurrentExtampInfo.inp_gain_ctl_reg_val  &= ~(IN_PGAINA_MASK|IN_PGAINB_MASK);
            tCurrentExtampInfo.inp_gain_ctl_reg_val  = tCurrentExtampInfo.inp_gain_ctl_reg_val |(gain<<2 |gain);

        break;
        default:
        break;
    }
    result = snd_extamp_write_all_reg(tCurrentExtampInfo); 
    #endif
}

/*==========================================================================
** snd_extamp_api_SetPath
**=========================================================================*/
#if !defined(CONFIG_MACH_MSM8X60_EF34K) && !defined(CONFIG_MACH_MSM8X60_EF35L) && !defined (CONFIG_MACH_MSM8X60_EF33S)
/*static*/ void snd_extamp_setpath(extamp_inmode_e inpath, extamp_outmode_e outpath, extamp_outfmt_e outfmt)
{
	u8 inregval, outregval = 0;

	switch(outfmt){
	case EXTAMP_MONO:
		inregval = IN_INA_MONO_DIFFERENTIAL | IN_INB_MONO_DIFFERENTIAL;
		break;
	case EXTAMP_STEREO:
		inregval = IN_INA_STEREO_SINGLE | IN_INB_STEREO_SINGLE;
		break;
	default:
		break;
	}

	switch(inpath){
	case EXTAMP_IN_INA:
		switch(outpath){
		case EXTAMP_OUT_SPK:
			outregval = OUTPUT_SHDN_POWER_ON |OUTPUT_BYPASS_DISABLE |OUTPUT_INA_ENABLE |OUTPUT_LEFT_SPEAKER_ENABLE |OUTPUT_RIGHT_SPEAKER_ENABLE;
			break;
		case EXTAMP_OUT_HPH_L:
		case EXTAMP_OUT_HPH_R:
		case EXTAMP_OUT_HPH_LR:
			outregval = OUTPUT_SHDN_POWER_ON |OUTPUT_BYPASS_DISABLE |OUTPUT_INA_ENABLE  |OUTPUT_HEADPHONE_AMP_ENABLE;
			break;
		case EXTAMP_OUT_SPK_HPH:
			outregval = OUTPUT_SHDN_POWER_ON |OUTPUT_BYPASS_DISABLE |OUTPUT_INA_ENABLE  |OUTPUT_HEADPHONE_AMP_ENABLE 
                                     |OUTPUT_LEFT_SPEAKER_ENABLE |OUTPUT_RIGHT_SPEAKER_ENABLE;           
                     bHeadphonePath = 1;
			break;
		default:
			return;
		}
		break;
	case EXTAMP_IN_INB:
		switch(outpath){
		case EXTAMP_OUT_SPK:
			outregval = OUTPUT_SHDN_POWER_ON |OUTPUT_BYPASS_DISABLE |OUTPUT_INB_ENABLE |OUTPUT_LEFT_SPEAKER_ENABLE |OUTPUT_RIGHT_SPEAKER_ENABLE;			break;
		case EXTAMP_OUT_HPH_L:
		case EXTAMP_OUT_HPH_R:
		case EXTAMP_OUT_HPH_LR:
                     outregval = OUTPUT_SHDN_POWER_ON |OUTPUT_BYPASS_DISABLE |OUTPUT_INB_ENABLE |OUTPUT_HEADPHONE_AMP_ENABLE;
			break;
		case EXTAMP_OUT_SPK_HPH:
                     outregval = OUTPUT_SHDN_POWER_ON |OUTPUT_BYPASS_DISABLE |OUTPUT_INB_ENABLE |OUTPUT_HEADPHONE_AMP_ENABLE
                                     |OUTPUT_LEFT_SPEAKER_ENABLE |OUTPUT_RIGHT_SPEAKER_ENABLE;           
			break;
		default:
			return;
		}
		break;

	case EXTAMP_IN_INAB:
		switch(outpath){
		case EXTAMP_OUT_SPK:
			outregval = OUTPUT_SHDN_POWER_ON |OUTPUT_BYPASS_DISABLE |OUTPUT_INA_ENABLE |OUTPUT_INB_ENABLE 
                                        |OUTPUT_LEFT_SPEAKER_ENABLE |OUTPUT_RIGHT_SPEAKER_ENABLE;
			break;
		case EXTAMP_OUT_HPH_L:
		case EXTAMP_OUT_HPH_R:
		case EXTAMP_OUT_HPH_LR:
			outregval = OUTPUT_SHDN_POWER_ON |OUTPUT_BYPASS_DISABLE |OUTPUT_INA_ENABLE |OUTPUT_INB_ENABLE 
                                        |OUTPUT_HEADPHONE_AMP_ENABLE;
			break;
		case EXTAMP_OUT_SPK_HPH:
			outregval = OUTPUT_SHDN_POWER_ON |OUTPUT_BYPASS_DISABLE |OUTPUT_INA_ENABLE |OUTPUT_INB_ENABLE 
                                        |OUTPUT_HEADPHONE_AMP_ENABLE |OUTPUT_LEFT_SPEAKER_ENABLE |OUTPUT_RIGHT_SPEAKER_ENABLE;
			break;
		default:
			return;
        	}
		break;
	case EXTAMP_IN_RXIN:
			outregval = OUTPUT_SHDN_POWER_DOWN |OUTPUT_BYPASS_ENABLE;
              break;
	default:
		return;
	}

	tExtampInfo.in_mode_ctrl = (tExtampInfo.in_mode_ctrl & 0x30) | inregval;
	tExtampInfo.out_mode_ctrl = outregval;
	
	snd_extamp_i2c_write(INPUT_MODE_CTRL_REG, tExtampInfo.in_mode_ctrl);
	snd_extamp_i2c_write(OUTPUT_MODE_CTRL_REG , tExtampInfo.out_mode_ctrl);
			
}
#endif

/*==========================================================================
** snd_extamp_i2c_write
**=========================================================================*/
static int snd_extamp_i2c_write(u8 reg, u8 data)
{
	static int ret = 0;
	unsigned char buf[2];
	struct i2c_msg msg[1];

	if(!extamp_i2c_client){
		return -1;
	}

	buf[0] = (unsigned char)reg;
	buf[1] = (unsigned char)data;

	msg[0].addr = extamp_i2c_client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = buf;

	ret = i2c_transfer(extamp_i2c_client->adapter, msg, 1);

	if (ret < 0) {
              AMP_ERR("snd_extamp_i2c_write fail ret=%x", ret);
		return -1;
	}

	return 0;
}
#if 1//SangwonLee 110330 Right Speaker is near by handset mic. Disable it to prevent recording noise.
static void snd_R_speaker_ctrl_add(void)
{
    extamp_info_t tCurrentExtampInfo;
    int result;	
    snd_extamp_make_current(&tCurrentExtampInfo);
    // SangwonLee 110330 Right Speaker is near by handset mic. Disable it to prevent recording noise.
    if(tCurrentExtampInfo.out_mode_ctrl&OUTPUT_RIGHT_SPEAKER_ENABLE){
		tCurrentExtampInfo.out_mode_ctrl&=~OUTPUT_RIGHT_SPEAKER_ENABLE;
		AMP_DBG_HIGH("IOCTL - Turn off R-SPK");
		result = snd_extamp_write_all_reg(tCurrentExtampInfo);
    }
    AMP_DBG_HIGH("snd_R_speaker_ctrl_add - set_pcm_in_status");
    set_pcm_in_status();
}
#endif
/*==========================================================================
** miscellaneous device
**=========================================================================*/
#ifdef FEATURE_SKY_SND_MISC_DEVICE
static struct file_operations snd_fops = {
	.owner		= THIS_MODULE,
	.open		= ext_amp_open,
	.release	= ext_amp_release,
	.unlocked_ioctl	= ext_amp_ioctl,
};
static struct miscdevice snd_miscdev = {
	.minor =    MISC_DYNAMIC_MINOR,
	.name =     "ext_amp",
	.fops =     &snd_fops
};
static int snd_misc_device_init(void)
{
    int result=0;
    result = misc_register(&snd_miscdev);
    if(result<0)    AMP_ERR("snd_misc_device_init ret= %d ", result);
    return result;
}
static int snd_misc_device_deinit(void)
{
    int result=0;
    result = misc_deregister(&snd_miscdev);
    if(result<0)    AMP_ERR("snd_misc_device_deinit ret= %d ", result);
    return result;
}
static int ext_amp_open(struct inode *inode, struct file *file)
{
	AMP_DBG_LOW("ext_amp_open ");
	return 0;
}

static int ext_amp_release(struct inode *inode, struct file *file)
{
	AMP_DBG_LOW("ext_amp_release ");
	return 0;	
}

static long ext_amp_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
       uint32_t value;
	AMP_DBG_LOW("ext_amp_ioctl cmd:0x%x, arg:0x%x ", cmd, arg);
    
    	switch (cmd) {
#ifdef FEATURE_SKY_SND_AUDIO_TEST_COMMAND
            case SND_SKY_EXTAMP_WRITE:
                if (get_user(value, (uint32_t __user *) arg)) {
                    AMP_ERR("Err:aud_sub_ioctl cmd SND_SKY_EXTAMP_WRITE");
                    return -EPERM;
                }
                current_reg= (value&0xff00)>>8;
                current_regval = (value&0xff);
                AMP_DBG_HIGH("SND_SKY_EXTAMP_WRITE (%d, %d)", current_reg, current_regval);
                snd_extamp_write_single_reg(current_reg, current_regval);
            break;

            case SND_SKY_EXTAMP_READ:
                if (get_user(value, (uint32_t __user *) arg)) {
                    AMP_ERR("Err:aud_sub_ioctl cmd SND_SKY_EXTAMP_READ");
                    return -EPERM;
                }
                current_reg = value;
                current_regval = snd_extamp_read_single_reg(value);
                AMP_DBG_HIGH("SND_SKY_EXTAMP_READ (%d, %d)", current_reg, current_regval);
                value = current_regval&0xff;
                if (copy_to_user((void*) arg, &value, sizeof(int))) {
                    AMP_ERR("aud_sub_ioctl cmd SND_SKY_EXTAMP_READ ERROR!!!");
                }
            break;
            case SND_SKY_EXTAMP_SPK_CH_CTRL:
                if (get_user(value, (uint32_t __user *) arg)) {
                    AMP_ERR("Err:aud_sub_ioctl cmd SND_SKY_EXTAMP_SPK_CH_CTRL");
                    return -EPERM;
                }
                exception_spk_ch=value; 
                AMP_DBG_HIGH("SND_SKY_EXTAMP_SPK_CH_CTRL (%d)", value);

                if(value)   {
                    snd_extamp_write_single_reg(DEFAULT_OUTPUT_MODE_CONTROL_REG_VAL, current_regval);
                }
            break;
#endif
            case SND_SKY_EXTAMP_SET_MODE:
                if (get_user(value, (uint32_t __user *) arg)) {
                    AMP_ERR("Err:aud_sub_ioctl cmd SND_SKY_EXTAMP_SET_MODE");
                    return -EPERM;
                }
                curMode=value; 
                AMP_DBG_MED("SND_SKY_EXTAMP_SET_MODE (%d)", value);
                break;
           case SND_SKY_SET_CALLMODE:
                  if (get_user(value, (uint32_t __user *) arg)) {
                        AMP_ERR("SND_SKY_SET_CALLMODE fail!\n");
			   return -EPERM;
                  }
                      curCallMode.mode = (value&0xff00)>>8;
                      curCallMode.param = (value&0xff);

#ifdef CONFIG_SKY_SND_MVS
			g_CallMode = curCallMode.mode;
#endif
                  AMP_DBG_MED("SND_SKY_SET_CALLMODE mode=%d, param=%d", curCallMode.mode, curCallMode.param);
              break;
          case SND_SKY_GET_CALLMODE:
              AMP_DBG_MED("SND_SKY_GET_CALLMODE mode=%d, param=%d", curCallMode.mode, curCallMode.param);
		if (copy_to_user((void*) arg, &curCallMode.mode, sizeof(int))) {
			AMP_ERR("SND_SKY_GET_CALLMODE fail!\n");
		}
            break;
        case SND_SKY_SET_MIC_MUTE_CTRL:    {
            	int value;
		if (get_user(value, (uint32_t __user *) arg)) {
                    AMP_ERR("SND_SKY_SET_MIC_MUTE_CTRL fail!\n");
		      return -EPERM;
		}
              if(value) {
                #define skip_buffer_num 42
                set_pcm_in_exception(skip_buffer_num);
              }
              AMP_DBG_MED("SND_SKY_SET_MIC_MUTE_CTRL skip_buffer_num=%d", skip_buffer_num);
            break;
        }
        case SND_SKY_SET_AUTOANSWER:
                  if (get_user(value, (uint32_t __user *) arg)) {
                        AMP_ERR("SND_SKY_SET_AUTOANSWER fail!\n");
                        return -EPERM;
                  }
                      curCallMode.mode = (value&0xff00)>>8;
                      curCallMode.param = (value&0xff);

#ifdef CONFIG_SKY_SND_MVS
			g_CallMode = curCallMode.mode ;
#endif
                  AMP_DBG_MED("SND_SKY_SET_AUTOANSWER  mode=%d, param=%d", curCallMode.mode, curCallMode.param);
		  if (curCallMode.param == 0)
                      snd_extamp_api_SetDevice(1, 1);           
              break;
#if 1//SangwonLee 110330 Right Speaker is near by handset mic. Disable it to prevent recording noise.
		case SND_SKY_SET_RSPK_OFF:
			AMP_DBG_HIGH("SND_SKY_SET_RSPK_OFF - snd_R_speaker_ctrl_add");
			snd_R_speaker_ctrl_add();
			break;
#endif
#ifdef MVS_AMR_DUMP
        case SND_SKY_DUMP_ENABLE:
            {
                  extern void set_enable_dump(int value);
                  if (get_user(value, (uint32_t __user *) arg)) {
                        AMP_ERR("SND_SKY_SET_AUTOANSWER fail!\n");
                        return -EPERM;
                  }
                  else                  {
                    AMP_DBG_MED("SND_SKY_DUMP_ENABLE  (%d)", value);
                    set_enable_dump(value);
                  }
            }
            break;
#endif		
            default:
                  break;
    	}
       return 0;
}
#endif

#ifdef FEATURE_SKY_SND_AUDIO_TEST_COMMAND
/*==========================================================================
** snd_extamp_read_single_reg
**=========================================================================*/
static int snd_extamp_read_single_reg(int reg)
{
    int regval=0;

    switch(reg)
    {
         case 0:
             regval=tExtampInfo.in_mode_ctrl;
             break;
         case 1:
             regval=tExtampInfo.spk_vol_ctrl;
             break;
         case 2:
             regval=tExtampInfo.hpl_vol_ctrl;
             break;
         case 3:
             regval=tExtampInfo.hpr_vol_ctrl;
             break;
         case 4:
             regval=tExtampInfo.out_mode_ctrl;
             break;
            default:
                break;
     }    
    return regval;
}
/*==========================================================================
** snd_extamp_write_single_reg
**=========================================================================*/
static void snd_extamp_write_single_reg(int reg, int regval)
{
	extamp_info_t tCurrentExtampInfo;
	snd_extamp_make_current(&tCurrentExtampInfo);

       switch(reg)
       {
            case 0:
                tCurrentExtampInfo.in_mode_ctrl=regval;
                break;
            case 1:
                tCurrentExtampInfo.spk_vol_ctrl=regval;
                break;
            case 2:
                tCurrentExtampInfo.hpl_vol_ctrl=regval;
                break;
            case 3:
                tCurrentExtampInfo.hpr_vol_ctrl=regval;
                break;
            case 4:
                tCurrentExtampInfo.out_mode_ctrl=regval;
                break;
            default:
                break;
        }
       snd_extamp_write_all_reg(tCurrentExtampInfo);
}
#endif

/*==========================================================================
** snd_extamp_i2c_read
**=========================================================================*/
/*static int snd_extamp_i2c_read(u8 reg, u8 *data)
{
	static int ret = 0;
	unsigned char buf[1];
	struct i2c_msg msgs[2];

	if(!extamp_i2c_client){
		return -1;
	}

	buf[0] = reg;

	msgs[0].addr = extamp_i2c_client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = buf;

	msgs[1].addr = extamp_i2c_client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = 1;
	msgs[1].buf = buf;

	ret = i2c_transfer(extamp_i2c_client->adapter, msgs, 2);
	if ( ret < 0) {
		return -1;
	}

	*data = (u8)buf[0];
	return 0;
}*/

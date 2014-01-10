#ifndef SKY_SND_EXT_AMP_MAX9879_H
#define SKY_SND_EXT_AMP_MAX9879_H
/************************************************************************************************
**
**    AUDIO EXTERNAL AMP
**
**    FILE
**        Sky_snd_ext_amp_max97001.h
**
**    DESCRIPTION
**        This file contains audio external amp api
**
**    Copyright (c) 2010 by PANTECH Incorporated.  All Rights Reserved.
*************************************************************************************************/


/************************************************************************************************
** Includes
*************************************************************************************************/
#include <linux/kernel.h>
/************************************************************************************************
** FEATURE Definition
*************************************************************************************************/
#define FEATURE_SKY_SND_MISC_DEVICE
#define FEATURE_SKY_SND_AUDIO_TEST_COMMAND		// SangwonLee 110104

/************************************************************************************************
** Message Definition
*************************************************************************************************/
//#define MSG_LEVEL_LOW   1
#define MSG_LEVEL_MED   2
#define MSG_LEVEL_HIGH   4

#ifdef MSG_LEVEL_LOW
#define AMP_DBG_LOW(fmt, arg...) printk(KERN_INFO "%s: " fmt "\n", __func__, ## arg)
#else
#define AMP_DBG_LOW(fmt, arg...) do {} while (0)
#endif

#ifdef MSG_LEVEL_MED
#define AMP_DBG_MED(fmt, arg...) printk(KERN_INFO "%s: " fmt "\n", __func__, ## arg)
#else
#define AMP_DBG_MED(fmt, arg...) do {} while (0)
#endif

#ifdef MSG_LEVEL_HIGH
#define AMP_DBG_HIGH(fmt, arg...) printk(KERN_INFO "%s: " fmt "\n", __func__, ## arg)
#else
#define AMP_DBG_HIGH(fmt, arg...) do {} while (0)
#endif

#define AMP_ERR(fmt, arg...) printk(KERN_ERR "%s: " fmt "\n", __func__, ## arg)
/************************************************************************************************
** Definition
*************************************************************************************************/
#define MAX9879_SLAVE_ADDR	0x4D	


/* MAX9879 Control Registers */
#define INPUT_MODE_CTRL_REG  0x00
#define SPK_VOL_CTRL_REG  0x01
#define LHP_VOL_CTRL_REG 0x02
#define RHP_VOL_CTRL_REG 0x03
#define OUTPUT_MODE_CTRL_REG 0x04
#define MAX9879_REG_NUM 0x05

/* Input Register */
#define IN_ZCD_ENABLE   0x40
#define IN_ZCD_DISABLE   0x00
#define IN_INA_MONO_DIFFERENTIAL    0x20
#define IN_INA_STEREO_SINGLE    0x00
#define IN_INB_MONO_DIFFERENTIAL    0x10
#define IN_INB_STEREO_SINGLE    0x00
#define IN_PGAINA_0_DB  0x00
#define IN_PGAINA_5P_5_DB  0x04
#define IN_PGAINA_10_DB  0x08
#define IN_PGAINA_MASK  0x0c
#define IN_PGAINB_0_DB  0x00
#define IN_PGAINB_5P_5_DB  0x01
#define IN_PGAINB_10_DB  0x02
#define IN_PGAINB_MASK  0x03

/* Volume Control Register */
#define VOL_MASK 0x1f
#define VOL_M_MUTE     0x00
#define VOL_M_75_DB     0x01
#define VOL_M_71_DB     0x02
#define VOL_M_67_DB     0x03
#define VOL_M_63_DB     0x04
#define VOL_M_59_DB     0x05
#define VOL_M_55_DB     0x06
#define VOL_M_51_DB     0x07
#define VOL_M_47_DB     0x08
#define VOL_M_41_DB     0x09
#define VOL_M_38_DB     0x0a
#define VOL_M_35_DB     0x0b
#define VOL_M_32_DB     0x0c
#define VOL_M_29_DB     0x0d
#define VOL_M_26_DB     0x0e
#define VOL_M_23_DB     0x0f
#define VOL_M_21_DB     0x11
#define VOL_M_19_DB     0x12
#define VOL_M_17_DB     0x13
#define VOL_M_15_DB     0x14
#define VOL_M_13_DB     0x15
#define VOL_M_11_DB     0x16
#define VOL_M_9_DB     0x17
#define VOL_M_7_DB     0x18
#define VOL_M_6_DB     0x19
#define VOL_M_5_DB     0x1a
#define VOL_M_4_DB     0x1b
#define VOL_M_3_DB     0x1c
#define VOL_M_2_DB     0x1d
#define VOL_M_1_DB     0x1e
#define VOL_P_0_DB     0x1f

/* Output Register */
#define OUTPUT_SHDN_POWER_ON  0x80
#define OUTPUT_SHDN_POWER_DOWN  0x0
#define OUTPUT_BYPASS_ENABLE    0x40
#define OUTPUT_BYPASS_DISABLE    0x0
#define OUTPUT_INA_ENABLE   0x08
#define OUTPUT_INA_DISABLE   0x0
#define OUTPUT_INB_ENABLE   0x10
#define OUTPUT_INB_DISABLE   0x0
#define OUTPUT_LEFT_SPEAKER_ENABLE   0x04
#define OUTPUT_LEFT_SPEAKER_DISABLE   0x0
#define OUTPUT_RIGHT_SPEAKER_ENABLE   0x02
#define OUTPUT_RIGHT_SPEAKER_DISABLE   0x0
#define OUTPUT_HEADPHONE_AMP_ENABLE 0x01
#define OUTPUT_HEADPHONE_AMP_DISABLE 0x0

#define SND_DEVICE_HANDSET_RX  0 // handset_rx
#define SND_DEVICE_HANDSET_TX  1//handset_tx
#define SND_DEVICE_SPEAKER_RX  2 //speaker_stereo_rx
#define SND_DEVICE_SPEAKER_TX  3//speaker_mono_tx
#define SND_DEVICE_HEADSET_RX  4 //headset_stereo_rx
#define SND_DEVICE_HEADSET_TX  5 //headset_mono_tx
#define SND_DEVICE_FMRADIO_HANDSET_RX 6 //fmradio_handset_rx
#define SND_DEVICE_FMRADIO_HEADSET_RX 7 //fmradio_headset_rx
#define SND_DEVICE_FMRADIO_SPEAKER_RX 8 //fmradio_speaker_rx
#define SND_DEVICE_DUALMIC_HANDSET_TX  9 //handset_dual_mic_endfire_tx
#define SND_DEVICE_DUALMIC_SPEAKER_TX  10 //speaker_dual_mic_endfire_tx
#define SND_DEVICE_TTY_HEADSET_MONO_RX  11 //tty_headset_mono_rx
#define SND_DEVICE_TTY_HEADSET_MONO_TX  12 //tty_headset_mono_tx
#define SND_DEVICE_BT_SCO_RX  17 //bt_sco_rx
#define SND_DEVICE_BT_SCO_TX  18 //bt_sco_tx
#define SND_DEVICE_SPEAKER_HEADSET_RX  13 //headset_stereo_speaker_stereo_rx
#define SND_DEVICE_MUTE_RX  22     //AutoAnswer
#define SND_DEVICE_CURRENT     32   // Current Device 

typedef enum 
{
  EXTAMP_OUT_SPK,
  EXTAMP_OUT_HPH_L,
  EXTAMP_OUT_HPH_R,
  EXTAMP_OUT_HPH_LR,
  EXTAMP_OUT_SPK_HPH,
  EXTAMP_OUT_BYPASS
}extamp_outmode_e;

typedef enum
{
  EXTAMP_IN_INA,
  EXTAMP_IN_INB,
  EXTAMP_IN_INAB,
  EXTAMP_IN_RXIN
}extamp_inmode_e;

typedef enum
{
  EXTAMP_MONO,
  EXTAMP_STEREO
}extamp_outfmt_e;

typedef struct
{
  u8	in_mode_ctrl;
  u8	spk_vol_ctrl;
  u8	hpl_vol_ctrl;
  u8	hpr_vol_ctrl;
  u8	out_mode_ctrl;
}extamp_info_t;

typedef enum
{
  MODE_NORMAL,
  MODE_RINGTONE,
  MODE_IN_CALL
}phone_state_e;

typedef enum
{
  MODE_DEFAULT,
  MODE_VT,
  MODE_VOIP,
  MODE_TDMB,
  MODE_AUTOANSWER,
}call_mode_e;

typedef struct
{
    call_mode_e mode;
    int     param;
}call_mode_param;

/************************************************************************************************
** Variables
*************************************************************************************************/

/************************************************************************************************
** Declaration
*************************************************************************************************/
extern void snd_extamp_api_Init(void);
extern void snd_extamp_api_DeInit(void);
extern void snd_extamp_api_SetDevice(int on, uint32_t cad_device);
extern void snd_extamp_api_SetVolume(unsigned char hp_vol, unsigned char sp_vol);
extern void snd_extamp_api_SetPreAmpGain(uint32_t gain);
extern void snd_extamp_api_HeadsetConnected(uint32_t connect);
extern int snd_extamp_get_current_callmode(void);
extern int snd_extamp_get_mode(void);
extern void snd_extamp_api_reset(void);
#endif /*SKY_SND_EXT_AMP_MAX9879_H*/


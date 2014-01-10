
/***************************************************************************
* 
*   SiI9244 - MHL Transmitter Driver
*
* Copyright (C) (2011, Silicon Image Inc)
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation version 2.
*
* This program is distributed "as is" WITHOUT ANY WARRANTY of any
* kind, whether express or implied; without even the implied warranty
* of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
*****************************************************************************/

/* ========================================================
Revision history
     Date     :          who                                            : Why
2011.11.07 :          Cho.kyoungku@pantech.com        : Update for MHL CTS 1.1
========================================================*/

#include <linux/delay.h>
#include <linux/interrupt.h>
#include <asm/irq.h>
#include <linux/i2c.h>
#include <asm/irq.h>
#include <linux/delay.h>
#include <linux/syscalls.h> 
#include <linux/fcntl.h> 
#include <asm/uaccess.h> 
#include <mach/gpio.h>
#include "mhl_sii9244_driver.h"
#include "msm_fb.h"
#include <linux/input.h>
#ifdef CONFIG_PANTECH_F_CHECK_INT
#include <linux/mutex.h>
#endif

#define GENERIC_FAST_FORWARD 208
#define GENERIC_REWIND 168
#define GENERIC_PREVIOUS 165
#define GENERIC_NEXT 163
#define GENERIC_STOP 166
#define GENERIC_PLAY_PAUSE 164
#define GENERIC_ENTER 66
#define MEDIA_PAUSE 201

#ifdef MHL_DEBUG
#define TX_DEBUG_PRINT(x) printk x
#else
#define TX_DEBUG_PRINT(x) 
#endif
/*==============================================================================*/
#ifdef MHL_REMOCON_CONTROL 
static struct input_dev *ipdev;
#endif

#ifdef F_MHL_AUTO_CONNECT
unsigned int mhl_cable_state = 0;
#define MHL_DEVICE_ON			1
#define MHL_DEVICE_OFF			0
#define MHL_CABLE_CONNCET			1
#define MHL_CABLE_DISCONNCET	       0
#endif
extern int mhl_power_ctrl(int on);
extern int pantech_hdmi_cable_detect(int on);
extern void sii9244_cfg_power_init(void);
#ifdef CONFIG_PANTECH_LCD_MHL_CABLE_DETECT
extern void change_mhl_state(boolean online);
extern void mhl_cable_detect_handler(void);
#ifdef PANTECH_FUSION2_MHL_DETECT
extern void mhl_connect_api(boolean on);
#endif
#endif


//fix cbus_cmd_tail's prev & next pointer break
#define FIX_CBUS_BIND_POINTER_BREAK
//fix sound problem when first connection
#define FIX_SOUND_PROBLEM_WHEN_FIRST_CONNTECTION

#define	MHL_MAX_RCP_KEY_CODE	(0x7F + 1)	// inclusive
u8 rcpSupportTable [MHL_MAX_RCP_KEY_CODE] = {
	(MHL_DEV_LD_GUI),		// 0x00 = Select
	(MHL_DEV_LD_GUI),		// 0x01 = Up
	(MHL_DEV_LD_GUI),		// 0x02 = Down
	(MHL_DEV_LD_GUI),		// 0x03 = Left
	(MHL_DEV_LD_GUI),		// 0x04 = Right
	0, 0, 0, 0,				// 05-08 Reserved
	(MHL_DEV_LD_GUI),		// 0x09 = Root Menu
	0, 0, 0,				// 0A-0C Reserved
	(MHL_DEV_LD_GUI),		// 0x0D = Select
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	// 0E-1F Reserved
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_MEDIA | MHL_DEV_LD_TUNER),	// Numeric keys 0x20-0x29
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_MEDIA | MHL_DEV_LD_TUNER),
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_MEDIA | MHL_DEV_LD_TUNER),
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_MEDIA | MHL_DEV_LD_TUNER),
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_MEDIA | MHL_DEV_LD_TUNER),
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_MEDIA | MHL_DEV_LD_TUNER),
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_MEDIA | MHL_DEV_LD_TUNER),
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_MEDIA | MHL_DEV_LD_TUNER),
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_MEDIA | MHL_DEV_LD_TUNER),
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_MEDIA | MHL_DEV_LD_TUNER),
	0,						// 0x2A = Dot
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_MEDIA | MHL_DEV_LD_TUNER),	// Enter key = 0x2B
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_MEDIA | MHL_DEV_LD_TUNER),	// Clear key = 0x2C
	0, 0, 0,				// 2D-2F Reserved
	(MHL_DEV_LD_TUNER),		// 0x30 = Channel Up
	(MHL_DEV_LD_TUNER),		// 0x31 = Channel Dn
	(MHL_DEV_LD_TUNER),		// 0x32 = Previous Channel
	(MHL_DEV_LD_AUDIO),		// 0x33 = Sound Select
	0,						// 0x34 = Input Select
	0,						// 0x35 = Show Information
	0,						// 0x36 = Help
	0,						// 0x37 = Page Up
	0,						// 0x38 = Page Down
	0, 0, 0, 0, 0, 0, 0,	// 0x39-0x3F Reserved
	0,						// 0x40 = Undefined

	(MHL_DEV_LD_SPEAKER),	// 0x41 = Volume Up
	(MHL_DEV_LD_SPEAKER),	// 0x42 = Volume Down
	(MHL_DEV_LD_SPEAKER),	// 0x43 = Mute
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO),	// 0x44 = Play
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_RECORD),	// 0x45 = Stop
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_RECORD),	// 0x46 = Pause
	(MHL_DEV_LD_RECORD),	// 0x47 = Record
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO),	// 0x48 = Rewind
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO),	// 0x49 = Fast Forward
	(MHL_DEV_LD_MEDIA),		// 0x4A = Eject
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_MEDIA),	// 0x4B = Forward
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_MEDIA),	// 0x4C = Backward
	0, 0, 0,				// 4D-4F Reserved
	0,						// 0x50 = Angle
	0,						// 0x51 = Subpicture
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 52-5F Reserved
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO),	// 0x60 = Play Function
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO),	// 0x61 = Pause the Play Function
	(MHL_DEV_LD_RECORD),	// 0x62 = Record Function
	(MHL_DEV_LD_RECORD),	// 0x63 = Pause the Record Function
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_RECORD),	// 0x64 = Stop Function

	(MHL_DEV_LD_SPEAKER),	// 0x65 = Mute Function
	(MHL_DEV_LD_SPEAKER),	// 0x66 = Restore Mute Function
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 	// Undefined or reserved
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 		// Undefined or reserved
};

mhl_rx_cap_type mhl_reciever_cap;
mhl_tx_status_type mhl_status_value;
cbus_cmd_node *cbus_cmd_head, *cbus_cmd_tail;
/*==============================================================================*/
void WriteByteTPI(u8 RegOffset, u8 Data);
u8 ReadByteTPI(u8 RegOffset);
u8 ReadByteCBUS (u8 Offset);
void WriteByteCBUS(u8 Offset, u8 Data);
void ReadModifyWriteTPI(u8 Offset, u8 Mask, u8 Value);
void sii9244_driver_init(void);
void sii9244_remote_control_init(void);
void SiiMhlTxInitialize(void);
void	CbusReset(void);
void	SwitchToD3( void );
static void ReleaseUsbIdSwitchOpen ( void );
void WriteInitialRegisterValues ( void );
static void InitCBusRegs( void );
void MhlTxDrvProcessConnection ( void );
void	SwitchToD0( void );
void CbusWakeUpPulseGenerator(void);
static u8 CBusProcessErrors( u8 intStatus );
static void ForceUsbIdSwitchOpen ( void );
static void ApplyDdcAbortSafety(void);
void	SiiMhlTxDrvTmdsControl( bool enable );
void sii9244_interrupt_event(void);
void ReadModifyWriteCBUS(u8 Offset, u8 Mask, u8 Value); 
int MHL_Cable_On(int on);
void HDMI_Control_On(bool on);
#ifdef MHL_REMOCON_CONTROL 
void rcp_cbus_uevent(u8);
#endif 
static void sii9244_mhl_tx_int(void);
int I2C_WriteByte(u8 deviceID, u8 offset, u8 value);
u8 I2C_ReadByte(u8 deviceID, u8 offset);
void simg_mhl_tx_handler(void);
void process_waiting_rgnd(void);
void process_mhl_discovery_start(void);
u8 TX_RGND_check (void);
void process_mhl_discovery_success(void);
void cbus_cmd_bind(u8 command, u8 offset, u8* data, u16 data_lenth, bool priority);
static void cbus_cmd_send(struct work_struct *cbus_work);
void mhltx_got_status(u8 status0, u8 status1);
void mhltx_set_status(u8 command, u8 offset, u8 value);
void cbus_cmd_done(void);
void cbus_cmd_put_workqueue(cbus_cmd_node* node_p);
void mhltx_got_mhlintr(u8 data0, u8 data1);
void SiiMhlTxRapkSend(void);
void SiiMhlTxRcpeSend(u8 erro_code, u8 key_code);
void SiiMhlTxRcpkSend(u8 keycode);
void proccess_msc_msg_handle(void);
void MhlTxSetInt( u8 regToWrite,u8  mask, u8 priority);
void delete_all_cbus_cmd(void);
void ReadModifyWritePage0(u8 Offset, u8 Mask, u8 Data);
void WriteBytePage0 (u8 Offset, u8 Data);
u8 ReadBytePage0 (u8 Offset);
void process_cbus_interrrupt(void);
void simg_timer_handler(unsigned long timer_type);
void simg_init_timer(u8 data, unsigned int msecs);
//void simg_timer_hanler_cb(void* _data);

/*==============================================================================*/
#define	I2C_READ_MODIFY_WRITE(saddr,offset,mask)	I2C_WriteByte(saddr, offset, I2C_ReadByte(saddr, offset) | (mask));
#define ReadModifyWriteByteCBUS(offset,andMask,orMask)  WriteByteCBUS(offset,(ReadByteCBUS(offset)&andMask) | orMask)

#define	SET_BIT(saddr,offset,bitnumber)		I2C_READ_MODIFY_WRITE(saddr,offset, (1<<bitnumber))
#define	CLR_BIT(saddr,offset,bitnumber)		I2C_WriteByte(saddr, offset, I2C_ReadByte(saddr, offset) & ~(1<<bitnumber))


#define	DISABLE_DISCOVERY				CLR_BIT(PAGE_0_0X72, 0x90, 0);
#define	ENABLE_DISCOVERY				SET_BIT(PAGE_0_0X72, 0x90, 0);

#define STROBE_POWER_ON                    CLR_BIT(PAGE_0_0X72, 0x90, 1);

#define RSEN_HIGH BIT2
/*========================================================================*/
//
//	Look for interrupts on INTR4 (Register 0x74)
//		7 = RSVD		(reserved)
//		6 = RGND Rdy	(interested)
//		5 = VBUS Low	(ignore)	
//		4 = CBUS LKOUT	(interested)
//		3 = USB EST		(interested)
//		2 = MHL EST		(interested)
//		1 = RPWR5V Change	(ignore)
//		0 = SCDT Change	(only if necessary)
//
#define	INTR_4_DESIRED_MASK				(BIT0 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6) 
#define USB_EST_INTR4 BIT3
#define MHL_EST_INTR4 BIT2
#define CBUS_LOCKOUT BIT4
#define RGND_RDY BIT6
#define SCDT_CHANGE BIT0
#define	MASK_INTR_4_INTERRUPTS		I2C_WriteByte(PAGE_0_0X72, 0x78, INTR_4_DESIRED_MASK)
#define	UNMASK_INTR_4_INTERRUPTS	I2C_WriteByte(PAGE_0_0X72, 0x78, 0x00)
#define RGND_INTR4_INTERRUPT      I2C_WriteByte(PAGE_0_0X72, 0x78, BIT6)
#define WRITE_INTR4_VALUE(x)      I2C_WriteByte(PAGE_0_0X72, 0x78, x)  

/*========================================================================*/
//	Look for interrupts on INTR_2 (Register 0x72)
//		7 = bcap done			(ignore)
//		6 = parity error		(ignore)
//		5 = ENC_EN changed		(ignore)
//		4 = no premable			(ignore)
//		3 = ACR CTS changed		(ignore)
//		2 = ACR Pkt Ovrwrt		(ignore)
//		1 = TCLK_STBL changed	(interested)
//		0 = Vsync				(ignore)

#define	INTR_2_DESIRED_MASK				(BIT1)
#define	MASK_INTR_2_INTERRUPTS		I2C_WriteByte(PAGE_0_0X72, 0x76, INTR_2_DESIRED_MASK)
#define	UNMASK_INTR_2_INTERRUPTS			I2C_WriteByte(PAGE_0_0X72, 0x76, 0x00)

/*========================================================================*/
//	Look for interrupts on INTR_1 (Register 0x71)
//		7 = RSVD		(reserved)
//		6 = MDI_HPD		(interested)
//		5 = RSEN CHANGED(interested)	
//		4 = RSVD		(reserved)
//		3 = RSVD		(reserved)
//		2 = RSVD		(reserved)
//		1 = RSVD		(reserved)
//		0 = RSVD		(reserved)

#define	INTR_1_DESIRED_MASK				(BIT5|BIT6) 
#define MDI_HPD (BIT6)
#define RSEN_CHANG (BIT5) 
#define	MASK_INTR_1_INTERRUPTS		    I2C_WriteByte(PAGE_0_0X72, 0x75, INTR_1_DESIRED_MASK)
#define	UNMASK_INTR_1_INTERRUPTS			I2C_WriteByte(PAGE_0_0X72, 0x75, 0x00)
#define WRITE_INTR1_VALUE(x)          I2C_WriteByte(PAGE_0_0X72, 0x75, x)  

/*========================================================================*/
//	Look for interrupts on CBUS:CBUS_INTR_STATUS [0xC8:0x08]
//		7 = RSVD			(reserved)
//		6 = MSC_RESP_ABORT	(interested)
//		5 = MSC_REQ_ABORT	(interested)	
//		4 = MSC_REQ_DONE	(interested)
//		3 = MSC_MSG_RCVD	(interested)
//		2 = DDC_ABORT		(interested)
//		1 = RSVD			(reserved)
//		0 = rsvd			(reserved)
#define	INTR_CBUS1_DESIRED_MASK			(BIT2 | BIT3 | BIT4 | BIT5 | BIT6)
#define MSC_RESP_ABORT BIT6
#define MSC_REQ_ABORT BIT5
#define MSC_REQ_DONE BIT4
#define MSC_MSG_RCVD BIT3 
#define DDC_ABORT BIT2
#define	MASK_CBUS1_INTERRUPTS			I2C_WriteByte(PAGE_CBUS_0XC8, 0x09, INTR_CBUS1_DESIRED_MASK)
#define	UNMASK_CBUS1_INTERRUPTS			I2C_WriteByte(PAGE_CBUS_0XC8, 0x09, 0x00)
#define WRITE_CBUS1_VALUE(x)  I2C_WriteByte(PAGE_CBUS_0XC8, 0x09, x) 

/*========================================================================*/
#define	INTR_CBUS2_DESIRED_MASK			(BIT0| BIT2 | BIT3)
#define	MASK_CBUS2_INTERRUPTS			I2C_WriteByte(PAGE_CBUS_0XC8, 0x1F, INTR_CBUS2_DESIRED_MASK)
#define	UNMASK_CBUS2_INTERRUPTS			I2C_WriteByte(PAGE_CBUS_0XC8, 0x1F, 0x00)
#define WRITE_CBUS2_VALUE(x)  I2C_WriteByte(PAGE_CBUS_0XC8, 0x1F, x) 

/*========================================================================*/

boolean f_delay_first_init = FALSE;
boolean f_schedule_ing = FALSE;
static void mhl_tx_init(void);
#ifdef CONFIG_PANTECH_F_CHECK_INT
boolean init_done = FALSE;

static int init_workqueue_flag = 0;
void sii9244_interrupt_schdule(void);

static DEFINE_MUTEX(mutx);

static struct workqueue_struct *intr_queue_work;
static struct work_struct intr_work;
void intr_check_thread(struct work_struct *work);
static DECLARE_WORK(intr_work,intr_check_thread); 
extern void do_hdmi_hpd_feature_on(void);
static bool mhl_ctrled_hpd_state=false;
#endif
struct f_mhl_cable_check_delayed_work  {
	struct delayed_work		work;
};

struct f_mhl_cable_check_delayed_work f_mhl_cable_check_control;
#ifdef CONFIG_PANTECH_F_CHECK_INT
void intr_check_thread(struct work_struct *work)
{
	if(!gpio_get_value(MHL_INT)){ 
		mutex_lock(&mutx);
		sii9244_interrupt_schdule();
		mutex_unlock(&mutx);
	}
}
#endif

void set_mhl_ctrled_hpd_state(bool hpd)
{
	mhl_ctrled_hpd_state = hpd;
}

bool get_mhl_ctrled_hpd_state(void)
{
	return mhl_ctrled_hpd_state;
}
void sii9244_driver_init(void)
{
#ifdef MHL_DEBUG
	printk(KERN_ERR "[SKY_MHL]+%s 4th i2c-reg \n", __FUNCTION__);
#endif
	SiiMhlTxInitialize(); 
	if (!init_workqueue_flag){
		intr_queue_work = create_singlethread_workqueue("intr_work");
		INIT_WORK(&intr_work, intr_check_thread);
		init_workqueue_flag++;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                
		                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   
		if( intr_queue_work == NULL)	{																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																	   
#ifdef MHL_DEBUG
		printk(KERN_ERR "[SKY_MHL]+%s intr_queue_work is NULL \n", __FUNCTION__);
#endif	
		}
	}																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																												  
	init_done =TRUE;
}
EXPORT_SYMBOL(sii9244_driver_init);

#ifdef MHL_REMOCON_CONTROL 
void Remote_control_init(void)
{
	int ret;
	
	ipdev =  input_allocate_device();
	ipdev->name = "MHL_REMOCON";

	input_set_capability(ipdev, EV_KEY, KEY_0);
	input_set_capability(ipdev, EV_KEY, KEY_1);
	input_set_capability(ipdev, EV_KEY, KEY_2);
	input_set_capability(ipdev, EV_KEY, KEY_3);
	input_set_capability(ipdev, EV_KEY, KEY_4);
	input_set_capability(ipdev, EV_KEY, KEY_5);
	input_set_capability(ipdev, EV_KEY, KEY_6);
	input_set_capability(ipdev, EV_KEY, KEY_7);
	input_set_capability(ipdev, EV_KEY, KEY_8);
	input_set_capability(ipdev, EV_KEY, KEY_9);
	input_set_capability(ipdev, EV_KEY, KEY_LEFT);
	input_set_capability(ipdev, EV_KEY, KEY_RIGHT);
	input_set_capability(ipdev, EV_KEY, KEY_UP);
	input_set_capability(ipdev, EV_KEY, KEY_DOWN);
	input_set_capability(ipdev, EV_KEY, KEY_REPLY/*KEY_ENTER*/);
	input_set_capability(ipdev, EV_KEY, KEY_F1);
	input_set_capability(ipdev, EV_KEY, KEY_F2);
	input_set_capability(ipdev, EV_KEY, KEY_END);
	input_set_capability(ipdev, EV_KEY, KEY_F3);
	input_set_capability(ipdev, EV_KEY, KEY_F4);

	input_set_capability(ipdev, EV_KEY, KEY_OK);
	input_set_capability(ipdev, EV_KEY, KEY_SEND);	
	
	input_set_capability(ipdev, EV_KEY, KEY_BACK);
	input_set_capability(ipdev, EV_KEY, KEY_PLAY);	
	input_set_capability(ipdev, EV_KEY, KEY_STOP);
	input_set_capability(ipdev, EV_KEY, KEY_PAUSE);
	input_set_capability(ipdev, EV_KEY, KEY_VIDEO_NEXT);
	input_set_capability(ipdev, EV_KEY, KEY_VIDEO_PREV);	
	input_set_capability(ipdev, EV_KEY, KEY_PLAYCD);
	input_set_capability(ipdev, EV_KEY, KEY_STOPCD);
	input_set_capability(ipdev, EV_KEY, KEY_PAUSECD);
	input_set_capability(ipdev, EV_KEY, KEY_NEXTSONG);
	input_set_capability(ipdev, EV_KEY, KEY_PREVIOUSSONG);	
	input_set_capability(ipdev, EV_KEY, KEY_VOLUMEUP);
	input_set_capability(ipdev, EV_KEY, KEY_VOLUMEDOWN);
	input_set_capability(ipdev, EV_KEY, KEY_PLAYPAUSE);
	input_set_capability(ipdev, EV_KEY, KEY_REWIND);
	input_set_capability(ipdev, EV_KEY, KEY_FASTFORWARD);
	input_set_capability(ipdev, EV_KEY, KEY_FORWARD);
	input_set_capability(ipdev, EV_KEY, KEY_MEDIA);
	input_set_capability(ipdev, EV_KEY, KEY_NEXT);
	input_set_capability(ipdev, EV_KEY, KEY_PREVIOUS);
	input_set_capability(ipdev, EV_KEY, KEY_MEDIA_REPEAT);
	input_set_capability(ipdev, EV_KEY, KEY_FORWARD);
	input_set_capability(ipdev, EV_KEY, KEY_CHANNELDOWN);
	input_set_capability(ipdev, EV_KEY, KEY_CHANNELUP);

#ifdef F_MHL_AUTO_CONNECT
	input_set_capability(ipdev, EV_KEY, KEY_F21);  // hdmi_contorl_on
	input_set_capability(ipdev, EV_KEY, KEY_F22);  // hdmi_control_off
	input_set_capability(ipdev, EV_KEY, KEY_F23);  // mhl_connect_on
	input_set_capability(ipdev, EV_KEY, KEY_F24);  // mhl_connect_off
#endif
	
	ret = input_register_device(ipdev);	
	if (ret<0)
	{
		printk("MHL_REMOCON: input_device_init_fail\n");
	}	
}

void sii9244_remote_control_init(void)
{
#ifdef MHL_DEBUG
       printk(KERN_ERR "[SKY_MHL]+%s  \n", __FUNCTION__);
#endif
	Remote_control_init();
}
EXPORT_SYMBOL(sii9244_remote_control_init);
void sii9244_remote_control_remove(void)	
{
#ifdef MHL_DEBUG
       printk(KERN_ERR "[SKY_MHL]+%s  \n", __FUNCTION__);
#endif	
	input_unregister_device(ipdev);
	input_free_device(ipdev);
}
#endif

#define MSECS_TO_JIFFIES(ms) (((ms)*HZ+999)/1000)

signed long simg_schedule_timeout_uninterruptible(signed long timeout)
{
  	__set_current_state(TASK_UNINTERRUPTIBLE);
  	return schedule_timeout(timeout);
}

void simg_msleep(unsigned int msecs)
{
	unsigned long timeout = MSECS_TO_JIFFIES(msecs) + 1;

	while (timeout){
		timeout = simg_schedule_timeout_uninterruptible(timeout);
	}
		
}

void SiiMhlTxInitialize( void )
{
#ifdef MHL_DEBUG
	printk(KERN_INFO "(Line:%d) sii9244_mhl_tx_int\n", (int)__LINE__ );
#endif
	mhl_status_value.mhl_status = NO_MHL_STATUS;
	mhl_status_value.cbus_connected = FALSE;
	mhl_status_value.sink_hpd = FALSE;
	mhl_status_value.linkmode = 0x03; 
	mhl_status_value.connected_ready = 0;
	mhl_status_value.rsen_check_available = FALSE;

	mhl_status_value.intr4_mask_value = RGND_RDY;
	mhl_status_value.intr1_mask_value = 0x00;
	mhl_status_value.intr_cbus1_mask_value = 0x00;
	mhl_status_value.intr_cbus2_mask_value = 0x00;
	mhl_status_value.rgnd_1k = FALSE;

	memset(&mhl_reciever_cap, 0x00, sizeof(mhl_rx_cap_type));

#ifdef FIX_CBUS_BIND_POINTER_BREAK
	if(cbus_cmd_head == NULL)
		cbus_cmd_head = (cbus_cmd_node*)kmalloc(sizeof(cbus_cmd_node), GFP_KERNEL);
	if(cbus_cmd_tail == NULL)
		cbus_cmd_tail = (cbus_cmd_node*)kmalloc(sizeof(cbus_cmd_node), GFP_KERNEL);
#else
	cbus_cmd_head = (cbus_cmd_node*)kmalloc(sizeof(cbus_cmd_node), GFP_KERNEL);
	cbus_cmd_tail = (cbus_cmd_node*)kmalloc(sizeof(cbus_cmd_node), GFP_KERNEL);
#endif

	cbus_cmd_head->next = cbus_cmd_tail;
	cbus_cmd_head->prev = cbus_cmd_head;
	cbus_cmd_tail->next = cbus_cmd_tail;
	cbus_cmd_tail->prev = cbus_cmd_head;

	cbus_cmd_head->cmd_send = 0;
	cbus_cmd_tail->cmd_send = 0;

	WriteInitialRegisterValues();

	WRITE_INTR4_VALUE(mhl_status_value.intr4_mask_value);
	WRITE_INTR1_VALUE(mhl_status_value.intr1_mask_value);
	WRITE_CBUS1_VALUE(mhl_status_value.intr_cbus1_mask_value);
	WRITE_CBUS2_VALUE(mhl_status_value.intr_cbus2_mask_value);

	SwitchToD3();
	mhl_status_value.mhl_status =  MHL_WAITING_RGND_DETECT;  
}

void WriteInitialRegisterValues ( void )
{
      TX_DEBUG_PRINT(("Drv: WriteInitialRegisterValues\n"));
	// Power Up
	I2C_WriteByte(PAGE_1_0x7A, 0x3D, 0x3F);	// Power up CVCC 1.2V core
	I2C_WriteByte(PAGE_2_0x92, 0x11, 0x01);	// Enable TxPLL Clock
	I2C_WriteByte(PAGE_2_0x92, 0x12, 0x15);	// Enable Tx Clock Path & Equalizer
	I2C_WriteByte(PAGE_0_0X72, 0x08, 0x35);	// Power Up TMDS Tx Core

	CbusReset();

	// Analog PLL Control
	I2C_WriteByte(PAGE_2_0x92, 0x10, 0xC1);	// bits 5:4 = 2b00 as per characterization team.
	I2C_WriteByte(PAGE_2_0x92, 0x17, 0x03);	// PLL Calrefsel
	I2C_WriteByte(PAGE_2_0x92, 0x1A, 0x20);	// VCO Cal
	I2C_WriteByte(PAGE_2_0x92, 0x22, 0x8A);	// Auto EQ
	I2C_WriteByte(PAGE_2_0x92, 0x23, 0x6A);	// Auto EQ
	I2C_WriteByte(PAGE_2_0x92, 0x24, 0xAA);	// Auto EQ
	I2C_WriteByte(PAGE_2_0x92, 0x25, 0xCA);	// Auto EQ
	I2C_WriteByte(PAGE_2_0x92, 0x26, 0xEA);	// Auto EQ
	I2C_WriteByte(PAGE_2_0x92, 0x4C, 0xA0);	// Manual zone control
	I2C_WriteByte(PAGE_2_0x92, 0x4D, 0x00);	// PLL Mode Value

	I2C_WriteByte(PAGE_0_0X72, 0x80, 0x24);	// Enable Rx PLL Clock Value	
	I2C_WriteByte(PAGE_2_0x92, 0x45, 0x44);	// Rx PLL BW value from I2C
	I2C_WriteByte(PAGE_2_0x92, 0x31, 0x0A);	// Rx PLL BW ~ 4MHz
	I2C_WriteByte(PAGE_0_0X72, 0xA0, 0xD0);
	I2C_WriteByte(PAGE_0_0X72, 0xA1, 0xFC);	// Disable internal MHL driver
	I2C_WriteByte(PAGE_0_0X72, 0xA3, 0xEB);
	I2C_WriteByte(PAGE_0_0X72, 0xA6, 0x0C);
	I2C_WriteByte(PAGE_0_0X72, 0x2B, 0x01);	// Enable HDCP Compliance safety

	//
	// CBUS & Discovery
	// CBUS discovery cycle time for each drive and float = 150us
	//

	ReadModifyWriteTPI(0x90, BIT_3 | BIT_2, BIT_2);
	I2C_WriteByte(PAGE_0_0X72, 0x91, 0xA5);		// Clear bit 6 (reg_skip_rgnd)

	// Changed from 66 to 65 for 94[1:0] = 01 = 5k reg_cbusmhl_pup_sel
       I2C_WriteByte(PAGE_0_0X72, 0x94, 0x77);			// 1.8V CBUS VTH & GND threshold
       //I2C_WriteByte(PAGE_0_0X72, 0x94, 0x75);			// 1.8V CBUS VTH & GND threshold

	//set bit 2 and 3, which is Initiator Timeout
	I2C_WriteByte(PAGE_CBUS_0XC8, 0x31, I2C_ReadByte(PAGE_CBUS_0XC8, 0x31) | 0x0c);

	//I2C_WriteByte(PAGE_0_0X72, 0xA5, 0xAC);			// RGND Hysterisis.
       I2C_WriteByte(PAGE_0_0X72, 0xA5, 0xA0);			

	// RGND & single discovery attempt (RGND blocking)
	I2C_WriteByte(PAGE_0_0X72, 0x95, 0x71);

	// use 1K and 2K setting
	//I2C_WriteByte(PAGE_0_0X72, 0x96, 0x22);

	// Use VBUS path of discovery state machine
	I2C_WriteByte(PAGE_0_0X72, 0x97, 0x00);

	WriteBytePage0(0x92, 0xA6); // kkcho_1212, change 1119	  0x86->0xA6			
	WriteBytePage0(0x93, 0x8C);				

	ReadModifyWritePage0(0x79, BIT5 | BIT4, BIT4);	// Force upstream HPD to 0 when not in MHL mode.
#ifdef MHL_DEBUG	
	printk("Drv: Upstream HPD Acquired - driven low.\n");
#endif
	set_mhl_ctrled_hpd_state(false);
#ifdef ACTIV_HIGH_INT
 	ReadModifyWritePage0(0x79, BIT_1 | BIT_2, 0); //Set interrupt active high
#endif/*ACTIV_HIGH_INT*/

	simg_msleep(25);
	ReadModifyWritePage0(0x95, BIT6, 0x00);		// Release USB ID switch
	
	I2C_WriteByte(PAGE_0_0X72, 0x90, 0x27);			// Enable CBUS discovery

       //CbusReset();
       InitCBusRegs();

	// Enable Auto soft reset on SCDT = 0
	I2C_WriteByte(PAGE_0_0X72, 0x05, 0x04);

	// HDMI Transcode mode enable
	I2C_WriteByte(PAGE_0_0X72, 0x0D, 0x1C);
}

void CbusReset (void)
{
	u8	idx;
	SET_BIT(PAGE_0_0X72, 0x05, 3);
	simg_msleep(2);
	CLR_BIT(PAGE_0_0X72, 0x05, 3);
  
	for(idx=0; idx < 4; idx++){
		// Enable WRITE_STAT interrupt for writes to all 4 MSC Status registers.
		I2C_WriteByte(PAGE_CBUS_0XC8, 0xE0 + idx, 0xFF);

		// Enable SET_INT interrupt for writes to all 4 MSC Interrupt registers.
		I2C_WriteByte(PAGE_CBUS_0XC8, 0xF0 + idx, 0xFF);
	}
}

static void InitCBusRegs (void)
{
	byte		regval;
	// Increase DDC translation layer timer

	TX_DEBUG_PRINT(("Drv: InitCBusRegs\n"));

	I2C_WriteByte(PAGE_CBUS_0XC8, 0x07, 0xF2);//for default DDC byte mode	
	
	I2C_WriteByte(PAGE_CBUS_0XC8, 0x40, 0x03); 			// CBUS Drive Strength
	I2C_WriteByte(PAGE_CBUS_0XC8, 0x42, 0x06); 			// CBUS DDC interface ignore segment pointer
	I2C_WriteByte(PAGE_CBUS_0XC8, 0x36, 0x0C);
	I2C_WriteByte(PAGE_CBUS_0XC8, 0x3D, 0xFD);

	I2C_WriteByte(PAGE_CBUS_0XC8, 0x1C, 0x01);
	I2C_WriteByte(PAGE_CBUS_0XC8, 0x1D, 0x0F);	

	I2C_WriteByte(PAGE_CBUS_0XC8, 0x44, 0x02);

	// Setup our devcap
	I2C_WriteByte(PAGE_CBUS_0XC8, 0x80, 0x00);

	I2C_WriteByte(PAGE_CBUS_0XC8, 0x81, 0x10);
	I2C_WriteByte(PAGE_CBUS_0XC8, 0x82, 0x02);
	I2C_WriteByte(PAGE_CBUS_0XC8, 0x83, 0);  						
	I2C_WriteByte(PAGE_CBUS_0XC8, 0x84, 0);						
	I2C_WriteByte(PAGE_CBUS_0XC8, 0x85, MHL_DEV_VID_LINK_SUPPRGB444);
	I2C_WriteByte(PAGE_CBUS_0XC8, 0x86, MHL_DEV_AUD_LINK_2CH);
	I2C_WriteByte(PAGE_CBUS_0XC8, 0x87, 0);										// not for source
	I2C_WriteByte(PAGE_CBUS_0XC8, 0x88, 0x8E);
	I2C_WriteByte(PAGE_CBUS_0XC8, 0x89, 0x0F);										// not for source
	I2C_WriteByte(PAGE_CBUS_0XC8, 0x8A, 0x07);                                                            // Only RCP // Change
	I2C_WriteByte(PAGE_CBUS_0XC8, 0x8B, 0);
	I2C_WriteByte(PAGE_CBUS_0XC8, 0x8C, 0);										// reserved
	I2C_WriteByte(PAGE_CBUS_0XC8, 0x8D, MHL_SCRATCHPAD_SIZE);

	I2C_WriteByte(PAGE_CBUS_0XC8, 0x8E, 0x33); //MHL_INT_AND_STATUS_SIZE);

	I2C_WriteByte(PAGE_CBUS_0XC8, 0x8F, 0);										//reserved

	// Make bits 2,3 (initiator timeout) to 1,1 for register CBUS_LINK_CONTROL_2
	regval = I2C_ReadByte(PAGE_CBUS_0XC8, REG_CBUS_LINK_CONTROL_2 );
	regval = (regval | 0x0C);
	I2C_WriteByte(PAGE_CBUS_0XC8,REG_CBUS_LINK_CONTROL_2, regval);

	// Set NMax to 1
	I2C_WriteByte(PAGE_CBUS_0XC8, REG_CBUS_LINK_CONTROL_1, 0x01);

	ReadModifyWriteCBUS(REG_CBUS_LINK_CONTROL_11, 0x38, 0x30);
	ReadModifyWriteCBUS(REG_MSC_TIMEOUT_LIMIT, 0x0F, 0x0D); 
	ReadModifyWriteCBUS(0x2E, 0x15, 0x15);
}

static void ReleaseUsbIdSwitchOpen ( void )
{
	//mdelay(50);
	simg_msleep(50); // per spec	

	// Release USB ID switch
	//ReadModifyWriteTPI(0x95, BIT_6, 0x00);
	ReadModifyWritePage0(0x95, BIT6, 0x00);	
	ENABLE_DISCOVERY;
}

void read_cbus_data(u8 offset, unsigned char cnt, unsigned char *data)
{
	unsigned char i;

	for(i=0; i<cnt; i++){
		*data++ = ReadByteCBUS(offset+i);
	}
}

u8 ReadBytePage0 (u8 Offset)
{
	return I2C_ReadByte(PAGE_0_0X72, Offset);
}

void WriteBytePage0 (u8 Offset, u8 Data)
{
	I2C_WriteByte(PAGE_0_0X72, Offset, Data);
}

void ReadModifyWritePage0(u8 Offset, u8 Mask, u8 Data)
{
	u8 Temp;

	Temp = ReadBytePage0(Offset);		// Read the current value of the register.
	Temp &= ~Mask;					// Clear the bits that are set in Mask.
	Temp |= (Data & Mask);			// OR in new value. Apply Mask to Value for safety.
	WriteBytePage0(Offset, Temp);		// Write new value back to register.
}

void SwitchToD3( void )
{
#ifdef MHL_DEBUG
	printk(KERN_INFO "(Line:%d) TX_GO2D3 \n", (int)__LINE__ );
#endif
	ForceUsbIdSwitchOpen();

	ReadModifyWritePage0(0x93, BIT7 | BIT6 | BIT5 | BIT4, 0);
	ReadModifyWritePage0(0x94, BIT1 | BIT0, 0);
	ReleaseUsbIdSwitchOpen();

	ReadModifyWritePage0(0x79, BIT5 | BIT4, BIT4);	//Force HPD to LOW
	set_mhl_ctrled_hpd_state(false);
	
	I2C_WriteByte(PAGE_2_0x92, 0x01, 0x03);
	CLR_BIT(PAGE_1_0x7A, 0x3D, 0);  
}

void	SwitchToD0( void )
{
#ifdef MHL_DEBUG
	printk(KERN_INFO"SwitchToD0 \n");  
#endif
	WriteInitialRegisterValues();

	// Force Power State to ON
	STROBE_POWER_ON
	mhl_status_value.mhl_status = MHL_DISCOVERY_START;    
}

#ifdef MHL_WAKEUP_TIMER_MODIFY 
/* To use hrtimer*/
#define	MS_TO_NS(x)	(x * 1000000)

DECLARE_WAIT_QUEUE_HEAD(wake_wq);

static struct hrtimer hr_wake_timer;

static bool wakeup_time_expired;

static bool hrtimer_initialized;
static bool first_timer;

enum hrtimer_restart hrtimer_wakeup_callback(struct hrtimer *timer)
{
	wake_up(&wake_wq);
	wakeup_time_expired = true;
//	hrtimer_cancel(&hr_wake_timer);
	return HRTIMER_NORESTART;
}


void start_hrtimer_ms(unsigned long delay_in_ms)
{
	ktime_t ktime;
	ktime = ktime_set(0, MS_TO_NS(delay_in_ms));

	wakeup_time_expired = false;
//	hrtimer_init(&hr_wake_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	if (first_timer)
		first_timer = false;
	else
		hrtimer_cancel(&hr_wake_timer);

//	hr_wake_timer.function = &hrtimer_wakeup_callback;
	hrtimer_start(&hr_wake_timer, ktime, 0x1);
}
#endif

void CbusWakeUpPulseGenerator(void)
{	
	TX_DEBUG_PRINT(("Drv: CbusWakeUpPulseGenerator\n"));

#ifdef MHL_WAKEUP_TIMER_MODIFY 
	if (!hrtimer_initialized) {
		hrtimer_init(&hr_wake_timer, 1, 0x1);
		hr_wake_timer.function = &hrtimer_wakeup_callback;
		hrtimer_initialized = true;
		first_timer = true;
	}
		
	//
	// I2C method
	//
	//I2C_WriteByte(SA_TX_Page0_Primary, 0x92, (I2C_ReadByte(SA_TX_Page0_Primary, 0x92) | 0x10));

	// Start the pulse
	I2C_WriteByte(PAGE_0_0X72, 0x96, (I2C_ReadByte(PAGE_0_0X72, 0x96) | 0xC0));
	start_hrtimer_ms(19);
	wait_event_interruptible(wake_wq, wakeup_time_expired);

	I2C_WriteByte(PAGE_0_0X72, 0x96, (I2C_ReadByte(PAGE_0_0X72, 0x96) & 0x3F));
	start_hrtimer_ms(19);
	wait_event_interruptible(wake_wq, wakeup_time_expired);

	I2C_WriteByte(PAGE_0_0X72, 0x96, (I2C_ReadByte(PAGE_0_0X72, 0x96) | 0xC0));
	start_hrtimer_ms(19);
	wait_event_interruptible(wake_wq, wakeup_time_expired);

	I2C_WriteByte(PAGE_0_0X72, 0x96, (I2C_ReadByte(PAGE_0_0X72, 0x96) & 0x3F));
	start_hrtimer_ms(60);
	wait_event_interruptible(wake_wq, wakeup_time_expired);

	I2C_WriteByte(PAGE_0_0X72, 0x96, (I2C_ReadByte(PAGE_0_0X72, 0x96) | 0xC0));
	start_hrtimer_ms(19);
	wait_event_interruptible(wake_wq, wakeup_time_expired);

	I2C_WriteByte(PAGE_0_0X72, 0x96, (I2C_ReadByte(PAGE_0_0X72, 0x96) & 0x3F));
	start_hrtimer_ms(19);
	wait_event_interruptible(wake_wq, wakeup_time_expired);

	I2C_WriteByte(PAGE_0_0X72, 0x96, (I2C_ReadByte(PAGE_0_0X72, 0x96) | 0xC0));
	start_hrtimer_ms(19);
	wait_event_interruptible(wake_wq, wakeup_time_expired);

	I2C_WriteByte(PAGE_0_0X72, 0x96, (I2C_ReadByte(PAGE_0_0X72, 0x96) & 0x3F));

	start_hrtimer_ms(T_SRC_WAKE_TO_DISCOVER);
	wait_event_interruptible(wake_wq, wakeup_time_expired);

#else
	// Start the pulse
	I2C_WriteByte(PAGE_0_0X72, 0x96, (I2C_ReadByte(PAGE_0_0X72, 0x96) | 0xC0));
       mdelay(T_SRC_WAKE_PULSE_WIDTH_1);	// adjust for code path
       I2C_WriteByte(PAGE_0_0X72, 0x96, (I2C_ReadByte(PAGE_0_0X72, 0x96) & 0x3F));
       mdelay(T_SRC_WAKE_PULSE_WIDTH_1);	// adjust for code path
       I2C_WriteByte(PAGE_0_0X72, 0x96, (I2C_ReadByte(PAGE_0_0X72, 0x96) | 0xC0));
       mdelay(T_SRC_WAKE_PULSE_WIDTH_1);	// adjust for code path
       I2C_WriteByte(PAGE_0_0X72, 0x96, (I2C_ReadByte(PAGE_0_0X72, 0x96) & 0x3F));
       mdelay(T_SRC_WAKE_PULSE_WIDTH_2);	// adjust for code path
       I2C_WriteByte(PAGE_0_0X72, 0x96, (I2C_ReadByte(PAGE_0_0X72, 0x96) | 0xC0));
       mdelay(T_SRC_WAKE_PULSE_WIDTH_1);	// adjust for code path
       I2C_WriteByte(PAGE_0_0X72, 0x96, (I2C_ReadByte(PAGE_0_0X72, 0x96) & 0x3F));
       mdelay(T_SRC_WAKE_PULSE_WIDTH_1);	// adjust for code path
       I2C_WriteByte(PAGE_0_0X72, 0x96, (I2C_ReadByte(PAGE_0_0X72, 0x96) | 0xC0));
       mdelay(20);
       I2C_WriteByte(PAGE_0_0X72, 0x96, (I2C_ReadByte(PAGE_0_0X72, 0x96) & 0x3F));
       mdelay(T_SRC_WAKE_TO_DISCOVER);
#endif
       //
       // Toggle MHL discovery bit
       // 
       //I2C_WriteByte(PAGE_0_0X72, 0x92, (I2C_ReadByte(PAGE_0_0X72, 0x92) & 0xEF));
   
       //DISABLE_DISCOVERY;
       //ENABLE_DISCOVERY;
}

static u8 CBusProcessErrors( u8 intStatus )
{
	u8 result          = 0;
	u8 mscAbortReason  = 0;
//	u8 ddcAbortReason  = 0;

	/* At this point, we only need to look at the abort interrupts. */
	intStatus &=  (BIT_MSC_ABORT | BIT_MSC_XFR_ABORT);

	if ( intStatus ){
	        //      result = ERROR_CBUS_ABORT;		// No Retry will help
	          /* If transfer abort or MSC abort, clear the abort reason register. */
#if (0) // 20111117, kkcho_temp, for Auth
		if( intStatus & BIT_DDC_ABORT )
		{
			result = ddcAbortReason = ReadByteCBUS( REG_DDC_ABORT_REASON );
			TX_DEBUG_PRINT( ("CBUS DDC ABORT happened, reason:: %02X\n", (int)(ddcAbortReason)));
		}
#endif

		if ( intStatus & BIT_MSC_XFR_ABORT ){
			result = mscAbortReason = ReadByteCBUS( REG_PRI_XFR_ABORT_REASON );
			TX_DEBUG_PRINT( ("CBUS:: MSC Transfer ABORTED. Clearing 0x0D\n"));
			WriteByteCBUS( REG_PRI_XFR_ABORT_REASON, 0xFF );
		}

		if ( intStatus & BIT_MSC_ABORT ){
			TX_DEBUG_PRINT( ("CBUS:: MSC Peer sent an ABORT. Clearing 0x0E\n"));
			WriteByteCBUS( REG_CBUS_PRI_FWR_ABORT_REASON, 0xFF );
		}

		// Now display the abort reason.
		if ( mscAbortReason != 0 ){
			TX_DEBUG_PRINT( ("CBUS:: Reason for ABORT is ....0x%02X = ", (int)mscAbortReason ));
			if ( mscAbortReason & CBUSABORT_BIT_REQ_MAXFAIL)
				TX_DEBUG_PRINT( ("Requestor MAXFAIL - retry threshold exceeded\n"));

			if ( mscAbortReason & CBUSABORT_BIT_PROTOCOL_ERROR)
				TX_DEBUG_PRINT( ("Protocol Error\n"));

			if ( mscAbortReason & CBUSABORT_BIT_REQ_TIMEOUT)
				TX_DEBUG_PRINT( ("Requestor translation layer timeout\n"));

			if ( mscAbortReason & CBUSABORT_BIT_PEER_ABORTED)
				TX_DEBUG_PRINT( ("Peer sent an abort\n"));

			if ( mscAbortReason & CBUSABORT_BIT_UNDEFINED_OPCODE)
				TX_DEBUG_PRINT( ("Undefined opcode\n"));
		}
	}
	return( result );
}

static void ForceUsbIdSwitchOpen ( void )
{
	DISABLE_DISCOVERY 		// Disable CBUS discovery

	ReadModifyWritePage0(0x95, BIT6, BIT6);	// Force USB ID switch to open
	WriteBytePage0(0x92, 0xA6); // kkcho_1212, change 1119	  0x86->0xA6	
	ReadModifyWritePage0(0x79, BIT5 | BIT4, BIT4);	// Force upstream HPD to 0 when not in MHL mode.
	set_mhl_ctrled_hpd_state(false);
}

static void ApplyDdcAbortSafety(void)
{
	u8 bTemp, bPost;

	WriteByteCBUS(0x29, 0xFF);  // clear the ddc abort counter
	bTemp = ReadByteCBUS(0x29);  // get the counter
	simg_msleep(3);
	bPost = ReadByteCBUS(0x29);  // get another value of the counter

	if (bPost > (bTemp + 50)){
		CbusReset();
		InitCBusRegs();
		ForceUsbIdSwitchOpen();
		ReleaseUsbIdSwitchOpen();

		I2C_WriteByte(PAGE_0_0X72, 0xA0, 0xD0);
		SiiMhlTxDrvTmdsControl(FALSE);

		sii9244_mhl_tx_int(); 
	}
}

void	SiiMhlTxDrvTmdsControl( bool enable )
{
	if( enable ){
		SET_BIT(PAGE_0_0X72, 0x80, 4);
		TX_DEBUG_PRINT(("Drv: TMDS Output Enabled\n"));
		CLR_BIT(PAGE_0_0X72, 0x79, 4);  // kkcho_1212
		//MHL_Cable_On(1);  // kkcho_1212_temp
	}
	else{
		CLR_BIT(PAGE_0_0X72, 0x80, 4);   
		TX_DEBUG_PRINT(("Drv: TMDS Ouput Disabled\n"));
		//MHL_Cable_On(0); // kkcho_1212_temp
	}
}

static void cbus_cmd_send(struct work_struct * work)
{
	struct cbus_send_cmd_type *dev = container_of(work,struct cbus_send_cmd_type, work);  
	u8 startbit = 0;
	bool result = TRUE;
#ifdef MHL_DEBUG
	printk(KERN_INFO "\n ***cbus_cmd_send: cmd:%x  offset:%x Data[0]:%x\n", 
	 dev->command,dev->offsetdata, dev->payload_u.msgdata[0]);
#endif
	WriteByteCBUS( (REG_CBUS_PRI_ADDR_CMD&0xFF), dev->offsetdata); 	// set offset  
	WriteByteCBUS( (REG_CBUS_PRI_WR_DATA_1ST & 0xFF), dev->payload_u.msgdata[0]);

	switch(dev->command)
	{
		case MHL_SET_INT:	// Set one interrupt register = 0x60
			startbit = MSC_START_BIT_WRITE_REG;
			break;

		case MHL_WRITE_STAT:	// Write one status register = 0x60 | 0x80
			startbit = MSC_START_BIT_WRITE_REG;
			break;

		case MHL_READ_DEVCAP:	// Read one device capability register = 0x61
			startbit = MSC_START_BIT_READ_REG;
			break;

		case MHL_GET_STATE:			// 0x62 -
		case MHL_GET_VENDOR_ID:		// 0x63 - for vendor id	
		case MHL_SET_HPD:			// 0x64	- Set Hot Plug Detect in follower
		case MHL_CLR_HPD:			// 0x65	- Clear Hot Plug Detect in follower
		case MHL_GET_SC1_ERRORCODE:		// 0x69	- Get channel 1 command error code
		case MHL_GET_DDC_ERRORCODE:		// 0x6A	- Get DDC channel command error code.
		case MHL_GET_MSC_ERRORCODE:		// 0x6B	- Get MSC command error code.
		case MHL_GET_SC3_ERRORCODE:		// 0x6D	- Get channel 3 command error code.
			WriteByteCBUS( (REG_CBUS_PRI_ADDR_CMD & 0xFF), dev->command );
			startbit = MSC_START_BIT_MSC_CMD;
			break;

		case MHL_MSC_MSG:
			WriteByteCBUS( (REG_CBUS_PRI_WR_DATA_2ND & 0xFF), dev->payload_u.msgdata[1]);
			WriteByteCBUS( (REG_CBUS_PRI_ADDR_CMD & 0xFF), dev->command );
			startbit = MSC_START_BIT_VS_CMD;
			break;

		case MHL_WRITE_BURST:
			ReadModifyWriteCBUS((REG_MSC_WRITE_BURST_LEN & 0xFF),0x0F, dev->length -1 );

			// Now copy all bytes from array to local scratchpad
			if (NULL == dev->payload_u.pdata)
			{
#ifdef MHL_DEBUG			
			printk("\nDrv:%d Put pointer to WRITE_BURST data in req.pdatabytes!!!\n\n",(int)__LINE__);
#endif
			}
			else
			{
			int i;
#ifdef MHL_DEBUG
			printk("\nDrv:%d Writing data into scratchpad (Lenth: %d)\n\n",(int)__LINE__, dev->length);
#endif
			for ( i = 0; i < dev->length; i++ )
			{
			u8 tmp=0;
			tmp = dev->payload_u.msgdata[i];
#ifdef MHL_DEBUG
			printk(KERN_INFO "(Line:%d)  BURST DATA:%x \n", (int)__LINE__ , tmp); 
#endif
			WriteByteCBUS( (REG_CBUS_SCRATCHPAD_0 & 0xFF) + i, tmp );
			}
			}
			startbit = MSC_START_BIT_WRITE_BURST;
			break;

		default:
			result = FALSE;
			break;
	}

	if ( result ){
		WriteByteCBUS( REG_CBUS_PRI_START & 0xFF, startbit );
	}
	else	{
#ifdef MHL_DEBUG	
		printk("\nDrv:%d SiiMhlTxDrvSendCbusCommand failed\n\n",(int)__LINE__);
#endif
	}

	kfree(dev);
}

void MhlTxSetInt( u8 regToWrite,u8  mask, u8 priority)
{
	u8 data[1];

	data[0] = mask;
	cbus_cmd_bind(MHL_SET_INT,regToWrite,data,0x01,priority );    
}

void cbus_cmd_put_workqueue(cbus_cmd_node* node_p)
{
	struct cbus_send_cmd_type *dev;

	if(node_p->cmd_send == TRUE){
#ifdef MHL_DEBUG	
		printk(KERN_INFO "\n Line:%d COMMAND already was sent \n",(int)__LINE__ ); 
#endif
		return ;
	}

	dev = (struct cbus_send_cmd_type*)kmalloc(sizeof(struct cbus_send_cmd_type), GFP_KERNEL);

	if (dev == NULL){
#ifdef MHL_DEBUG	
		printk(KERN_INFO "\n Line:%d (dev == NULL) \n",(int)__LINE__ ); 
#endif
		return ;
	}
#ifdef MHL_DEBUG
	printk(KERN_INFO "\n Line:%d, send:%x, cmd:%x, offset:%x data[0]:%x data[1]:%x \n",(int)__LINE__, 
	node_p->cmd_send,node_p->command, node_p->offset, node_p->payload.data[0],node_p->payload.data[1]); 
#endif
	memset(dev, 0x00, sizeof(struct cbus_send_cmd_type));

	node_p->cmd_send = TRUE;
	dev->command = node_p->command;
	dev->offsetdata = node_p->offset;
	dev->length = node_p->lenght;
	memset(dev->payload_u.msgdata, 0x00, 16);
	memcpy(dev->payload_u.msgdata, node_p->payload.data, node_p->lenght);  

	INIT_WORK (&dev->work, cbus_cmd_send);
	schedule_work(&dev->work);    
}

void cbus_cmd_bind(u8 command, u8 offset, u8* data, u16 data_lenth, bool priority)
{
	cbus_cmd_node *new_node;
	cbus_cmd_node *s;

	new_node = (cbus_cmd_node*)kmalloc(sizeof(cbus_cmd_node), GFP_KERNEL);

	if (new_node == NULL){
#ifdef MHL_DEBUG	
		printk(KERN_INFO "\n Line:%d (new_node == NULL) \n",(int)__LINE__ ); 
#endif
		return;
	}

	if(priority){
#ifdef MHL_DEBUG	
		printk(KERN_INFO "\n CBUS command urgent priority \n");
#endif
		s = cbus_cmd_head->next;

		if(s == cbus_cmd_tail){
			new_node->next = cbus_cmd_tail;
			new_node->prev = cbus_cmd_head;
			cbus_cmd_tail->prev = new_node;
			cbus_cmd_head->next = new_node;
		}
		else	{
			if(s->cmd_send == TRUE){
				new_node->next = s->next;
				new_node->prev = s;
				s->next->prev = new_node;
				s->next = new_node;
			}
			else	{
				new_node->next = s;
				new_node->prev = cbus_cmd_head;
				s->prev = new_node;
				cbus_cmd_head->next = new_node;
			}
		}
	}
	else	{
#ifdef MHL_DEBUG	
		printk(KERN_INFO "\n CBUS command Normal priority \n");
#endif
		s = cbus_cmd_tail->prev;
		new_node->next = cbus_cmd_tail;
		new_node->prev = s; 
		cbus_cmd_tail->prev = new_node;      
		s->next = new_node; 
	}
  
	new_node->command = command;  
	new_node->offset = offset;

	if(data_lenth){
		memcpy(new_node->payload.data, data, data_lenth);
	}

	if(MHL_MSC_MSG_RCPE == data[0]){
		data_lenth = (data_lenth -1);
	}

	new_node->lenght = data_lenth;  
#ifdef MHL_DEBUG	
	printk(KERN_INFO "\n new_node->lenght: %d, %d \n", new_node->lenght, data_lenth);
#endif
	new_node->cmd_send = FALSE;
	new_node->cmd_done = FALSE; 

	if(cbus_cmd_head->next->cmd_send == TRUE){
#ifdef MHL_DEBUG	
		printk(KERN_INFO "\n Already command sending~~~ \n");
#endif
		return;
	}  

	cbus_cmd_put_workqueue(new_node);
	return;
}

void SiiMhlTxRapkSend(void)
{
	u8 data[2];

	data[0] = MHL_MSC_MSG_RAPK;
	data[1] = 0x00;
	cbus_cmd_bind(MHL_MSC_MSG,0x00,data,0x02, FALSE );
}

void SiiMhlTxRcpeSend(u8 erro_code, u8 key_code)
{
	u8 data[3];

	data[0] = MHL_MSC_MSG_RCPE;
	data[1] = erro_code;
	data[2] = key_code;

	cbus_cmd_bind(MHL_MSC_MSG,0x00,data,0x03, FALSE ); //don't copy keycode data.
}

#ifdef MHL_REMOCON_CONTROL 
void rcp_cbus_uevent(u8 rcpCode)	
{
	char env_buf[120];
	int key = 0;

	memset(env_buf, 0, sizeof(env_buf));
#ifdef MHL_DEBUG	
	printk("[SKY_MHL]%s : RCP Message Recvd , rcpCode = 0x%x\n",__func__,rcpCode);
#endif

       switch(rcpCode)
   	{

		case 0x01:  key = KEY_UP; break;// UP-key	
		case 0x02:  key = KEY_DOWN; break;// DOWN-key;
		case 0x03:  key = KEY_LEFT; break;// LEFT-key
		case 0x04:  key = KEY_RIGHT; break;// RIGHT-key
		case 0x00:  key = GENERIC_ENTER/*KEY_ENTER*/; break;// select-key //KEY_OK  //KEY_SEND	
		case 0x20 : key = KEY_0; break;
		case 0x21 : key = KEY_1; break;
		case 0x22 : key = KEY_2; break;
		case 0x23 : key = KEY_3; break;
		case 0x24 : key = KEY_4; break;
		case 0x25 : key = KEY_5; break;
		case 0x26 : key = KEY_6; break;
		case 0x27 : key = KEY_7; break;
		case 0x28 : key = KEY_8; break;
		case 0x29 : key = KEY_9; break;
		//case 0x71 : key = KEY_F1; break;
		//case 0x72 : key = KEY_F2; break;
		//case 0x74 : key = KEY_F4; break;	
		//case 0x73 : key = KEY_F3; break;
		case 0x0D : key = KEY_BACK; break;	  // KEY_END는 종료키다.
		/*
		MHD_RCP_CMD_VOL_UP	        = 0x41,
		MHD_RCP_CMD_VOL_DOWN        = 0x42,
		MHD_RCP_CMD_MUTE            = 0x43,
		MHD_RCP_CMD_PLAY            = 0x44,
		MHD_RCP_CMD_STOP            = 0x45,
		MHD_RCP_CMD_PAUSE           = 0x46,
		MHD_RCP_CMD_RECORD          = 0x47,
		MHD_RCP_CMD_REWIND          = 0x48,
		MHD_RCP_CMD_FAST_FWD        = 0x49,
		MHD_RCP_CMD_EJECT           = 0x4A,
		MHD_RCP_CMD_FWD             = 0x4B,
		MHD_RCP_CMD_BKWD            = 0x4C,
		*/
  		case 0x41 : key = KEY_VOLUMEUP; break;//SYS_KEY_MMI_VOL_UP; break;//Vol Up Key
		case 0x42 : key = KEY_VOLUMEDOWN; break; //SYS_KEY_MMI_VOL_DOWN; break;// Vol Down Key	
		case 0x44:  case 0x60: key = GENERIC_PLAY_PAUSE; break;// 207, java->MEDIA_PLAY_PAUSE  //gcan KEY_PLAY->GENERIC_PLAY_PAUSE
		case 0x45:  case 0x64: key = GENERIC_STOP; break;// STOP-key  //KEY_STOPCD  //gcan KEY_STOP->GENERIC_STOP
		case 0x46:  case 0x61: key = GENERIC_PLAY_PAUSE; break;// 119, java->MEDIA_PLAY_PAUSE //gcan KEY_PAUSE->GENERIC_PLAY_PAUSE
		case 0x48:  key = GENERIC_PREVIOUS ; break;// <<  FORWARD로 동작 되어야 함.. //gcan KEY_VIDEO_PREV->GENERIC_PREVIOUS
		case 0x49:  key = GENERIC_NEXT ; break;// >>	  REWIND로 동작 되어야 함. 	 //gcan KEY_VIDEO_NEXT->GENERIC_NEXT
		case 0x4B:  key = GENERIC_FAST_FORWARD ; break;// <<ㅇ //gcan KEY_FASTFORWARD->GENERIC_FAST_FORWARD
		case 0x4C:  key = GENERIC_REWIND ; break;// >> //gcan KEY_REWIND->GENERIC_REWIND
		
		default:
			// Any freak value here would continue with no event to app
			break;	
   	}

	input_report_key(ipdev, key, 1);
	input_sync(ipdev);
	input_report_key(ipdev, key, 0);
	input_sync(ipdev);
	return;
}
#endif

#ifdef F_MHL_AUTO_CONNECT
void MHL_Set_Cable_State(bool connect)
{
#ifdef MHL_DEBUG
	printk("[SKY_MHL] MHL_Set_Cable_State connect : %d\n", connect);
#endif

       if (connect){
		mhl_cable_state =MHL_CABLE_CONNCET;  //(connect)
   	}
	else	{
		mhl_cable_state =MHL_CABLE_DISCONNCET;  //(disconnect)   	
   	}	   	
}
EXPORT_SYMBOL(MHL_Set_Cable_State);

void MHL_En_Control(bool on)
{
#ifdef MHL_DEBUG
	printk("[SKY_MHL] MHL_En_Control  : %d\n", on);
#endif
       if (on){
#if (0) //   	
	 /* txs0102 control : HDMI-siganl level shift*/
        /* MHL_SHDW : active_low */
        gpio_direction_output(MHL_SHDN, GPIO_HIGH_VALUE);
        msleep(5);
        gpio_direction_output(MHL_SHDN, GPIO_LOW_VALUE);
        msleep(5);
        gpio_direction_output(MHL_SHDN, GPIO_HIGH_VALUE);  
#endif

#ifdef F_MHL_USB_SW_CONTROL_BUG_FIX
   	       gpio_direction_output(MHL_WAKE_UP, GPIO_HIGH_VALUE);  //20110811, kkcho,  HPD pin이 제대로 동작 안되는 상황이므로.. 보완을 위해...	
#endif    	 
	
		gpio_direction_output(MHL_EN, GPIO_HIGH_VALUE); // switch-MHL
   	}
	else	{
		gpio_direction_output(MHL_EN, GPIO_LOW_VALUE); // switch-USB	  	
#ifdef F_MHL_USB_SW_CONTROL_BUG_FIX   	
   		gpio_direction_output(MHL_WAKE_UP, GPIO_LOW_VALUE);  //20110811, kkcho,  HPD pin이 제대로 동작 안되는 상황이므로.. 보완을 위해...	
#endif 	
#if (0)
	        /* txs0102 control : HDMI-siganl level shift*/
	        gpio_direction_output(MHL_SHDN, GPIO_LOW_VALUE);	
#endif
#ifdef CONFIG_PANTECH_F_CHECK_INT
		 init_done = FALSE;
#endif		
   	}	   	
}
EXPORT_SYMBOL(MHL_En_Control);

uint32_t MHL_Get_Cable_State(void)
{
	uint32_t cabel_state = 0;
#ifdef MHL_DEBUG
	printk("[SKY_MHL] MHL_Get_Cable_State : %d\n", mhl_cable_state);
#endif
	if (mhl_cable_state == MHL_CABLE_CONNCET)
		return 1;

       return cabel_state;	   	
}
EXPORT_SYMBOL(MHL_Get_Cable_State);

/*
* 20130708, kkcho, media_pause for caption
*/
void media_pause_cmd(void)
{
	int key = 0;
	key = MEDIA_PAUSE;    	
	
	input_report_key(ipdev, key, 1);
	input_sync(ipdev);
	input_report_key(ipdev, key, 0);
	input_sync(ipdev);
}

int MHL_Cable_On(int on)
{
#ifdef FIX_SOUND_PROBLEM_WHEN_FIRST_CONNTECTION
	static bool isFirst = true;
#endif
	
//	int key = 0;
//	static int prev_on;	
#ifdef MHL_DEBUG
	printk("[SKY_MHL] MHL_Cable_On path change : %d\n", on);
#endif
//	if (on == prev_on)
//		return 0;

       if (on){
		media_pause_cmd(); // 20130708, kkcho, media_pause for caption
		
		/* txs0102 control : HDMI-siganl level shift*/
		/* MHL_SHDW : active_low */
		gpio_direction_output(MHL_SHDN, GPIO_HIGH_VALUE);
		msleep(5);
		gpio_direction_output(MHL_SHDN, GPIO_LOW_VALUE);
		msleep(5);
		gpio_direction_output(MHL_SHDN, GPIO_HIGH_VALUE);  

		   //MHL_On(1);      
	//	key = KEY_F23;  //KEYCODE_BUTTON_THUMBL  <= ON (connect)
		//msleep(10);	
		//pantech_hdmi_cable_detect(1);	
#ifdef PANTECH_FUSION2_MHL_DETECT		
		//change_mhl_state(true);	
		mhl_connect_api(1); 
#endif

#ifdef FIX_SOUND_PROBLEM_WHEN_FIRST_CONNTECTION
		if(isFirst) {
			delete_all_cbus_cmd(); 
			mhl_tx_init();

			isFirst = false;
		}
#endif		
   	}
	else	{	
		//pantech_hdmi_cable_detect(0);
#ifdef PANTECH_FUSION2_MHL_DETECT		
		change_mhl_state(false);
#endif		
		/* txs0102 control : HDMI-siganl level shift*/
		gpio_direction_output(MHL_SHDN, GPIO_LOW_VALUE);		
//		key = KEY_F24;  //KEYCODE_BUTTON_THUMBR  <= OFF (disconnect) 	
   	}	   	
	
//	input_report_key(ipdev, key, 1);
//	input_sync(ipdev);
//	input_report_key(ipdev, key, 0);
//	input_sync(ipdev);

//	prev_on = on;
	return 0;
}
EXPORT_SYMBOL(MHL_Cable_On);

void HDMI_Control_On(bool on)
{
	int key = 0;
#ifdef MHL_DEBUG
	printk("[SKY_MHL] HDMI_On path change : %d\n", on);
#endif
       if (on){
		key = KEY_F21;  //KEYCODE_BUTTON_THUMBL  <= ON (connect)
   	}
	else	{
		key = KEY_F22;  //KEYCODE_BUTTON_THUMBR  <= OFF (disconnect)   	
   	}	   	
	
	input_report_key(ipdev, key, 1);
	input_sync(ipdev);
	input_report_key(ipdev, key, 0);
	input_sync(ipdev);
}
EXPORT_SYMBOL(HDMI_Control_On);

void MHL_Set_cable_detect_handler(void)
{
#ifdef MHL_DEBUG
	printk("[SKY_MHL] MHL_Set_cable_detect_handler \n");
#endif
	mhl_cable_detect_handler();
}
EXPORT_SYMBOL(MHL_Set_cable_detect_handler);
#endif

void SiiMhlTxRcpkSend(u8 keycode)
{
	u8 data[2];

	data[0] = MHL_MSC_MSG_RCPK;
	data[1] = keycode;
	cbus_cmd_bind(MHL_MSC_MSG,0x00,data,0x02, FALSE );
}

void sii9244_interrupt_schdule(void)
{
#if (1) 
	simg_mhl_tx_handler(); 
#else
	static int function_start = 0;   //for test....

	if(function_start == 0)
	{
		function_start = 1;
		// test code start....
		//simg_interrupt_enable(FALSE);
		//simg_msleep(1000);
		printk(KERN_INFO "SiI9244 function start ## **\n");
		//mdelay(1000);

		//simg_msleep(1000);      
		//simg_chip_rst();
		//simg_interrupt_enable(TRUE);
		sii9244_mhl_tx_int();    
		//simg_interrupt_enable(TRUE);
		simg_mhl_tx_handler(); 
	}
	else
	{
		simg_mhl_tx_handler(); 
	}
#endif  
}

void mhl_int_schdule(struct work_struct *work)
{
	//f_schedule_ing = TRUE;
#ifdef MHL_DEBUG       
	// printk("[SKY_MHL] sii9244_interrupt_schdule\n");
#endif
#ifdef CONFIG_PANTECH_F_CHECK_INT
	mutex_lock(&mutx);
	sii9244_interrupt_schdule();
	mutex_unlock(&mutx);
#else
	sii9244_interrupt_schdule();
#endif
	   
       if (MHL_Get_Cable_State() == FALSE)
		cancel_delayed_work(&f_mhl_cable_check_control.work);	
#ifdef CONFIG_PANTECH_F_CHECK_INT
	if ( intr_queue_work != NULL && init_done)
		queue_work(intr_queue_work, &intr_work); 
#endif	
}

/*===========================================================================

  FUNCTION SiI9244_interrupt_event

  DESCRIPTION
   When SII9244 H/W interrupt happen, call this event function
===========================================================================*/
void sii9244_interrupt_event(void)
{
	if (/*!f_schedule_ing &&*/ MHL_Get_Cable_State()){
		//f_schedule_ing = TRUE;
		if (!f_delay_first_init){
			INIT_DELAYED_WORK_DEFERRABLE(&f_mhl_cable_check_control.work, mhl_int_schdule);
			f_delay_first_init = TRUE;
		}

		schedule_delayed_work(&f_mhl_cable_check_control.work, 0);  
		//printk("RETURN (0xFA == reg74)\n");
		//if (MHL_Get_Cable_State() == FALSE)
		//{
		//f_schedule_ing = FALSE;
		//	cancel_delayed_work(&f_mhl_cable_check_control.work);	
		//}	   
		//I2C_WriteByte(PAGE_0_0X72, (0x74), reg74);	// clear all interrupts
	}
}

EXPORT_SYMBOL(sii9244_interrupt_event);

/*===========================================================================
// kkcho_1212
=========================================================================*/
u8 TX_RGND_check (void)
{
	u8 rgndImpedance;

	rgndImpedance = I2C_ReadByte(PAGE_0_0X72, 0x99) & 0x03;
#ifdef MHL_DEBUG	
	printk(KERN_INFO "Drv: RGND = %02X \n ", (int)rgndImpedance);
#endif
	if (0x02 == rgndImpedance)	{
#ifdef MHL_DEBUG	
		printk(KERN_INFO "(MHL Device)\n");
#endif
		mhl_status_value.rgnd_1k = TRUE;

		// Select CBUS drive float.
		SET_BIT(PAGE_0_0X72, 0x95, 5);    

		simg_msleep(T_SRC_VBUS_CBUS_TO_STABLE);

		CbusWakeUpPulseGenerator();   
	}
	else	{
#ifdef MHL_DEBUG	
		printk(KERN_INFO "(Non-MHL Device)\n");
#endif
		mhl_status_value.rgnd_1k = FALSE;
	}

	return rgndImpedance;  
}

void delete_all_cbus_cmd(void)
{
	cbus_cmd_node *p;
	cbus_cmd_node *s;

	p = cbus_cmd_head->next;

	while(p != cbus_cmd_tail)	{
		s = p;
		p = p->next;
		p ->prev = cbus_cmd_head;
		kfree(s);
	}

	cbus_cmd_head->next = cbus_cmd_tail;
#ifdef FIX_CBUS_BIND_POINTER_BREAK
	cbus_cmd_head->prev = cbus_cmd_head;	/// add
	cbus_cmd_tail->next = cbus_cmd_tail;		/// add
#endif
	cbus_cmd_tail->prev = cbus_cmd_head;

	return;
}

static void mhl_tx_init(void)
{
	//simg_interrupt_enable(FALSE);
	//simg_chip_rst();
	sii9244_mhl_tx_int(); 
	//simg_interrupt_enable(TRUE);
}

////////////////////////////////////////////////////////////////////
// Int4Isr
//
//
//	Look for interrupts on INTR4 (Register 0x74)
//		7 = RSVD		(reserved)
//		6 = RGND Rdy	(interested)
//		5 = VBUS Low	(ignore)	
//		4 = CBUS LKOUT	(interested)
//		3 = USB EST		(interested)
//		2 = MHL EST		(interested)
//		1 = RPWR5V Change	(ignore)
//		0 = SCDT Change	(interested during D0)
////////////////////////////////////////////////////////////////////
void process_waiting_rgnd(void)
{
	u8 int4Status;

	int4Status = I2C_ReadByte(PAGE_0_0X72, 0x74);	// read status
#ifdef MHL_DEBUG
	printk (KERN_INFO "** Int4Isr : %x \n", (int)int4Status);
#endif
	if((0xFF == int4Status)||(0xFA == int4Status))	{
		simg_msleep(2);

		int4Status = I2C_ReadByte(PAGE_0_0X72, (0x74));	// read status
#ifdef MHL_DEBUG		
		printk (KERN_INFO "** Int4Isr 2 : %x \n", (int)int4Status);
#endif
		if((0xFF == int4Status)||(0xFA == int4Status))		{
			sii9244_mhl_tx_int();
			return ;
		}    
	}
  
	if(int4Status & RGND_RDY){
		mhl_status_value.mhl_status = MHL_CABLE_CONNECT;

		SwitchToD0(); 

		if(TX_RGND_check()!= 0x02){
			delete_all_cbus_cmd(); 

			//change to USB path.... customer...
			{
			//	MHL_On(0);
			//	msleep(10);
			//disconnect
				mhl_disconnect_queuework(0);
				
			}	
	//		sii9244_mhl_tx_int();
			return ;
		}

		I2C_WriteByte(PAGE_0_0X72, 0x74, int4Status);	// clear all interrupts        

		mhl_status_value.intr4_mask_value = (USB_EST_INTR4|MHL_EST_INTR4);
		mhl_status_value.intr1_mask_value = RSEN_CHANG;
		mhl_status_value.intr_cbus1_mask_value = 0x00;
		mhl_status_value.intr_cbus2_mask_value = 0x00;

		WRITE_INTR4_VALUE(mhl_status_value.intr4_mask_value);
		WRITE_INTR1_VALUE(mhl_status_value.intr1_mask_value);
		WRITE_CBUS1_VALUE(mhl_status_value.intr_cbus1_mask_value);
		WRITE_CBUS2_VALUE(mhl_status_value.intr_cbus2_mask_value);    
	} 
	else	{
		delete_all_cbus_cmd(); 
	//	mhl_tx_init();
		//sii9244_mhl_tx_int();  // kkcho_1212_temp

		//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!disconnect
		/*the status of intr4's masking is only RGND RDY, so disconnecting work is needed.*/
		mhl_disconnect_queuework(0);
	}
}

void proccess_msc_msg_handle(void)
{
	u8 subcmd, msg_data;

	subcmd = ReadByteCBUS( 0x18 );
	msg_data = ReadByteCBUS( 0x19 );  

	switch(subcmd)
	{
		case MHL_MSC_MSG_RAP:
			if( MHL_RAP_CONTENT_ON == msg_data){
				//simg_TmdsControl(TRUE);
				SiiMhlTxDrvTmdsControl(TRUE);
			}
			else if( MHL_RAP_CONTENT_OFF == msg_data)
				SiiMhlTxDrvTmdsControl( FALSE );

			SiiMhlTxRapkSend();
			break;

		case MHL_MSC_MSG_RCP:
#ifdef MHL_DEBUG			
			printk(KERN_INFO "\n MhlTx:%d MHL_MSC_MSG_RCP Key:%x \n",(int)__LINE__, msg_data );
#endif
			if(MHL_LOGICAL_DEVICE_MAP & rcpSupportTable [msg_data & 0x7F] ){
				SiiMhlTxRcpkSend(msg_data);
				// need to send RCP keycode...
#ifdef MHL_REMOCON_CONTROL 
				rcp_cbus_uevent(msg_data);	
#endif
			}
			else	{
				SiiMhlTxRcpeSend( RCPE_INEEFECTIVE_KEY_CODE, msg_data );
			}
			break;

		case MHL_MSC_MSG_RCPK:
#ifdef MHL_DEBUG			
			printk(KERN_INFO "MhlTx:%d SiiMhlTxGetEvents RCPK\n",(int)__LINE__ );
#endif
			break;

		case MHL_MSC_MSG_RCPE:
#ifdef MHL_DEBUG			
			printk(KERN_INFO "MhlTx:%d SiiMhlTxGetEvents MHL_MSC_MSG_RCPE \n",(int)__LINE__ );
#endif
			break;

		case MHL_MSC_MSG_RAPK:
#ifdef MHL_DEBUG			
			printk(KERN_INFO"MhlTx:%d SiiMhlTxGetEvents RAPK\n",(int)__LINE__ );
#endif
			break;

		default:
			// Any freak value here would continue with no event to app
			break;
	}
}

void cbus_cmd_done(void)
{
	cbus_cmd_node *current_p;
	cbus_cmd_node *next_p;

	current_p = cbus_cmd_head->next;
	current_p->cmd_send = FALSE;
	current_p->cmd_done = TRUE;

	switch( current_p->command)
	{
		case MHL_WRITE_STAT:
			//read data..
			current_p->lenght = 0x01;
			current_p->payload.data[0] = ReadByteCBUS(0x16);
#ifdef MHL_DEBUG			
			printk(KERN_INFO "\n cbus_cmd_done Line:%d CMD:%x, Data: %x \n", (int)__LINE__ , current_p->command,current_p->payload.data[0] );
#endif
			break;

		case MHL_READ_DEVCAP:  
			switch(current_p->offset)
			{

				case MHL_CAP_MHL_VERSION:
#ifdef MHL_DEBUG					
					printk(KERN_INFO "\n Line:%d MHL_CAP_MHL_VERSION \n", (int)__LINE__ );          
#endif
					mhl_reciever_cap.mhl_ver = ReadByteCBUS(0x16);
					break;

				case MHL_DEV_CATEGORY_OFFSET:
					{
						u8 cap_data =0;
						cap_data = (0x1F & ReadByteCBUS(0x16));

						switch(cap_data)
						{
							case MHL_DEV_CAT_SOURCE:
#ifdef MHL_DEBUG								
								printk(KERN_INFO "\n Line:%d MHL_DEV_CAT_SOURCE \n", (int)__LINE__ );          
#endif
								mhl_reciever_cap.mhl_ver = MHL_DEV_CAT_SOURCE;
								break;

							case MHL_SINK_W_POW:
#ifdef MHL_DEBUG								
								printk(KERN_INFO "\n Line:%d MHL_SINK_W_POW \n", (int)__LINE__ );          
#endif
								mhl_reciever_cap.mhl_ver = MHL_SINK_W_POW;
								//mhl tx doesn't need power out
								break;

							case MHL_SINK_WO_POW:
#ifdef MHL_DEBUG								
								printk(KERN_INFO "\n Line:%d MHL_SINK_WO_POW \n", (int)__LINE__ );       
#endif
								mhl_reciever_cap.mhl_ver = MHL_SINK_WO_POW;
								//this sink capability is wrong... Sink should have a POW bit....
								break;

							case MHL_DONGLE_W_POW:
#ifdef MHL_DEBUG								
								printk(KERN_INFO "\n Line:%d MHL_DONGLE_W_POW \n", (int)__LINE__ );    
#endif
								mhl_reciever_cap.mhl_ver = MHL_DONGLE_W_POW;
								//mhl tx doesn't need power out
								break;

							case MHL_DONGLE_WO_POW:
#ifdef MHL_DEBUG								
								printk(KERN_INFO "\n Line:%d MHL_DONGLE_WO_POW \n", (int)__LINE__ ); 
#endif
								mhl_reciever_cap.mhl_ver = MHL_DONGLE_WO_POW;
								// No dongle power ...
								break;

							default:
#ifdef MHL_DEBUG								
								printk(KERN_INFO "\n Line:%d DEFAULT \n", (int)__LINE__ );                
#endif
								mhl_reciever_cap.mhl_ver = cap_data;
								break;           
						}
					}
					break;

				case MHL_CAP_ADOPTER_ID_H:
#ifdef MHL_DEBUG					
					printk(KERN_INFO "\n Line:%d MHL_CAP_ADOPTER_ID_H \n", (int)__LINE__ );    
#endif
					mhl_reciever_cap.adopter_id = ReadByteCBUS(0x16);;
					mhl_reciever_cap.adopter_id <<= 8;
					break;

				case MHL_CAP_ADOPTER_ID_L:
#ifdef MHL_DEBUG					
					printk(KERN_INFO "\n Line:%d MHL_CAP_ADOPTER_ID_L \n", (int)__LINE__ );    
#endif
					mhl_reciever_cap.adopter_id |= ReadByteCBUS(0x16);;
					break;

				case MHL_CAP_VID_LINK_MODE:
#ifdef MHL_DEBUG					
					printk(KERN_INFO "\n Line:%d MHL_CAP_VID_LINK_MODE \n", (int)__LINE__ );     
#endif
					mhl_reciever_cap.vid_link_mode = (0x3F & ReadByteCBUS(0x16));
					break;

				case MHL_CAP_AUD_LINK_MODE:
#ifdef MHL_DEBUG					
					printk(KERN_INFO "\n Line:%d MHL_CAP_AUD_LINK_MODE \n", (int)__LINE__ );             
#endif
					mhl_reciever_cap.aud_link_mode = (0x03 & ReadByteCBUS(0x16));
					break;

				case MHL_CAP_VIDEO_TYPE:
#ifdef MHL_DEBUG					
					printk(KERN_INFO "\n Line:%d MHL_CAP_VIDEO_TYPE \n", (int)__LINE__ );         
#endif
					mhl_reciever_cap.video_type = (0x8F & ReadByteCBUS(0x16));
					break;

				case MHL_CAP_LOG_DEV_MAP:
#ifdef MHL_DEBUG					
					printk(KERN_INFO "\n Line:%d MHL_CAP_LOG_DEV_MAP \n", (int)__LINE__ );      
#endif
					mhl_reciever_cap.log_dev_map = ReadByteCBUS(0x16);
					break;

				case MHL_CAP_BANDWIDTH:
#ifdef MHL_DEBUG					
					printk(KERN_INFO "\n Line:%d MHL_CAP_BANDWIDTH \n", (int)__LINE__ );        
#endif
					mhl_reciever_cap.bandwidth = ReadByteCBUS(0x16);
					break;

				case MHL_CAP_FEATURE_FLAG:                 
					mhl_reciever_cap.feature_flag = (0x07 & ReadByteCBUS(0x16));
#ifdef MHL_DEBUG					
					printk(KERN_INFO "\n Line:%d MHL_CAP_FEATURE_FLAG:%x \n", (int)__LINE__, mhl_reciever_cap.feature_flag );          
#endif
					mhl_reciever_cap.rcp_support = (mhl_reciever_cap.feature_flag & MHL_FEATURE_RCP_SUPPORT)?TRUE:FALSE;
					mhl_reciever_cap.rap_support = (mhl_reciever_cap.feature_flag & MHL_FEATURE_RAP_SUPPORT)?TRUE:FALSE;
					mhl_reciever_cap.sp_support = (mhl_reciever_cap.feature_flag & MHL_FEATURE_SP_SUPPORT)?TRUE:FALSE;
					break;

				case MHL_CAP_DEVICE_ID_H:
#ifdef MHL_DEBUG					
					printk(KERN_INFO "\n Line:%d MHL_CAP_DEVICE_ID_H \n", (int)__LINE__ );           
#endif
					mhl_reciever_cap.device_id = ReadByteCBUS(0x16);
					mhl_reciever_cap.device_id <<= 8;
					break;

				case MHL_CAP_DEVICE_ID_L:
#ifdef MHL_DEBUG					
					printk(KERN_INFO "\n Line:%d MHL_CAP_DEVICE_ID_L \n", (int)__LINE__ );         
#endif
					mhl_reciever_cap.device_id |= ReadByteCBUS(0x16);
					break;

				case MHL_CAP_SCRATCHPAD_SIZE:
#ifdef MHL_DEBUG					
					printk(KERN_INFO "\n Line:%d MHL_CAP_SCRATCHPAD_SIZE \n", (int)__LINE__ );       
#endif
					mhl_reciever_cap.scratchpad_size = ReadByteCBUS(0x16);
					break;

				case MHL_CAP_INT_STAT_SIZE:
#ifdef MHL_DEBUG					
					printk(KERN_INFO "\n Line:%d MHL_CAP_INT_STAT_SIZE \n", (int)__LINE__ );                
#endif
					mhl_reciever_cap.int_stat_size = ReadByteCBUS(0x16);
					break;

				case MHL_CAP_DEV_STATE:
				case MHL_CAP_RESERVED:          
					default:
#ifdef MHL_DEBUG						
					printk(KERN_INFO "\n Line:%d DEFAULT \n", (int)__LINE__ );          
#endif
					break;
			}
		break;

	case MHL_MSC_MSG:  
#ifdef MHL_DEBUG		
		printk(KERN_INFO "\n cbus_cmd_done MHL_MSC_MSG \n");
#endif
		// check the previous data
		if(current_p->payload.data[0] == MHL_MSC_MSG_RCPE)
		{
			next_p = current_p->next;
#ifdef FIX_CBUS_BIND_POINTER_BREAK
			if(next_p==NULL)
				next_p=cbus_cmd_tail;
#endif
			cbus_cmd_head->next = next_p;
			next_p->prev = cbus_cmd_head;

#ifdef FIX_CBUS_BIND_POINTER_BREAK
			if(current_p != cbus_cmd_tail)
#endif
				kfree(current_p);

			if(next_p == cbus_cmd_tail)
			{
#ifdef MHL_DEBUG			
				printk(KERN_INFO "\n Line:%d (next_p == cbus_cmd_tail) \n", (int)__LINE__ );         
#endif
			}   
			else
			{
				cbus_cmd_put_workqueue(next_p); 
			}
#ifdef MHL_DEBUG
			printk(KERN_INFO "\n Line:%d MHL_MSC_MSG \n (current_p->payload.data[0] == MHL_MSC_MSG_RCPE)\n", (int)__LINE__ );
#endif
			SiiMhlTxRcpkSend(current_p->payload.data[2]);
			return ;
		}
		break;

	case MHL_WRITE_BURST:
		next_p = current_p->next;
#ifdef FIX_CBUS_BIND_POINTER_BREAK
			if(next_p==NULL)
				next_p=cbus_cmd_tail;
#endif
		
		cbus_cmd_head->next = next_p;
		next_p->prev = cbus_cmd_head;

#ifdef FIX_CBUS_BIND_POINTER_BREAK
		if(current_p != cbus_cmd_tail)
#endif
			kfree(current_p);

		if(next_p == cbus_cmd_tail)
		{
#ifdef MHL_DEBUG		
			printk(KERN_INFO "\n Line:%d (next_p == cbus_cmd_tail) \n", (int)__LINE__ );            
#endif
		}   
		else
		{
			cbus_cmd_put_workqueue(next_p); 
		}
#ifdef MHL_DEBUG
		printk(KERN_INFO "\n cbus_cmd_done MHL_WRITE_BURST \n");   
#endif
		MhlTxSetInt( MHL_RCHANGE_INT,MHL_INT_DSCR_CHG,FALSE);         
		return ;

		break;

	case MHL_SET_INT:  
#ifdef MHL_DEBUG		
		printk(KERN_INFO "\n cbus_cmd_done MHL_SET_INT \n");
#endif
		if((current_p->offset ==MHL_RCHANGE_INT )&&(current_p->payload.data[0]==MHL_INT_DSCR_CHG ))
		{
			//Write burst final step... Req->GRT->Write->DSCR
#ifdef MHL_DEBUG			
			printk(KERN_INFO "\n MHL_RCHANGE_INT&MHL_INT_DSCR_CHG  \n");
#endif
		}
		else if((current_p->offset ==MHL_RCHANGE_INT )&&(current_p->payload.data[0]==MHL_INT_DCAP_CHG ))
		{
			next_p = current_p->next;
#ifdef FIX_CBUS_BIND_POINTER_BREAK
			if(next_p==NULL)
				next_p=cbus_cmd_tail;
#endif
			
			cbus_cmd_head->next = next_p;
			next_p->prev = cbus_cmd_head;
			
#ifdef FIX_CBUS_BIND_POINTER_BREAK
			if(current_p != cbus_cmd_tail)
#endif
				kfree(current_p);

			if(next_p == cbus_cmd_tail)
			{
#ifdef MHL_DEBUG			
				printk(KERN_INFO "\n Line:%d (next_p == cbus_cmd_tail) \n", (int)__LINE__ );      
#endif
			}   
			else
			{
				cbus_cmd_put_workqueue(next_p); 
			}          

			mhl_status_value.connected_ready |= MHL_STATUS_DCAP_RDY;
			mhltx_set_status(MHL_WRITE_STAT, MHL_STATUS_REG_CONNECTED_RDY,mhl_status_value.connected_ready );

			return ;
		}
		break;   

	default:
#ifdef MHL_DEBUG		
		printk(KERN_INFO "\n cbus_cmd_done default \n");
#endif
		break;
	}

	next_p = current_p->next;
#ifdef FIX_CBUS_BIND_POINTER_BREAK
			if(next_p==NULL)
				next_p=cbus_cmd_tail;
#endif
	
	cbus_cmd_head->next = next_p;
	next_p->prev = cbus_cmd_head;

#ifdef FIX_CBUS_BIND_POINTER_BREAK
	if(current_p != cbus_cmd_tail)
#endif		
		kfree(current_p);

	if(next_p == cbus_cmd_tail){
#ifdef MHL_DEBUG	
		printk(KERN_INFO "\n Line:%d (next_p == cbus_cmd_tail) \n", (int)__LINE__ );
#endif
		return;
	}   

	cbus_cmd_put_workqueue(next_p);    
}


void mhltx_got_mhlintr(u8 data0, u8 data1)
{
	u8 tmp[0];

	if(MHL_INT_DCAP_CHG & data0)	{
#ifdef MHL_DEBUG	
		printk(KERN_INFO "(Line:%d)  MHL_INT_DCAP_CHG \n", (int)__LINE__ );      
#endif
		cbus_cmd_bind(MHL_READ_DEVCAP, MHL_DEV_CATEGORY_OFFSET,tmp,0,FALSE);	
		cbus_cmd_bind(MHL_READ_DEVCAP,MHL_CAP_FEATURE_FLAG,tmp, 0, FALSE);
	}

	if( MHL_INT_DSCR_CHG & data0){
		printk(KERN_INFO "(Line:%d)  MHL_INT_DSCR_CHG \n", (int)__LINE__ ); 
		//c_DSCR_CHG_handle_func();
	}

	if( MHL_INT_REQ_WRT  & data0)	{
#ifdef MHL_DEBUG	
		printk(KERN_INFO "(Line:%d)  MHL_INT_REQ_WRT \n", (int)__LINE__ ); 
#endif
		MhlTxSetInt( MHL_RCHANGE_INT, MHL_INT_GRT_WRT, FALSE);
	}

	if( MHL_INT_GRT_WRT  & data0)	{
#ifdef MHL_DEBUG	
		printk(KERN_INFO "(Line:%d)  MHL_INT_GRT_WRT \n", (int)__LINE__ ); 
#endif
		//c_GRT_WRT_handle_func();
	}

	if(MHL_INT_EDID_CHG & data1)	{
#ifdef MHL_DEBUG	
		printk(KERN_INFO "(Line:%d)  MHL_INT_EDID_CHG \n", (int)__LINE__ ); 
#endif
		ReadModifyWritePage0(0x79, BIT5 | BIT4, BIT4);	// Force upstream HPD to 0 when not in MHL mode.
		set_mhl_ctrled_hpd_state(false);
		simg_msleep(110);
		SET_BIT(PAGE_0_0X72, 0x79,  5);
		CLR_BIT(PAGE_0_0X72, 0x79, 4);
	}  
}

void mhl_path_enable(bool path_en)
{
	u8 data[16];
	u16 data_size;
#ifdef MHL_DEBUG
	printk(KERN_INFO "(Line:%d)  MHL_STATUS_PATH_ENABLED !!! \n", (int)__LINE__ ); 
#endif
	if(path_en)
		mhl_status_value.linkmode |= MHL_STATUS_PATH_ENABLED;
	else
		mhl_status_value.linkmode &= ~MHL_STATUS_PATH_ENABLED;


	data[0] = mhl_status_value.linkmode;
	data_size = 1;
	cbus_cmd_bind(MHL_WRITE_STAT, MHL_STATUS_REG_LINK_MODE,data,data_size, FALSE);
}

void mhltx_got_status(u8 status0, u8 status1)
{
	u8 data[16];
	u16 data_size;

	memset(data, 0x00, (sizeof(u8)*16));
	data_size = 0;

	if((MHL_STATUS_PATH_ENABLED & status1)&&
		!(mhl_status_value.linkmode & MHL_STATUS_PATH_ENABLED)){
		mhl_path_enable(TRUE);
	}
	else if(!(MHL_STATUS_PATH_ENABLED & status1)&&
		(mhl_status_value.linkmode & MHL_STATUS_PATH_ENABLED)){
		mhl_path_enable(FALSE);
	}

	if(MHL_STATUS_DCAP_RDY & status0){
#ifdef MHL_DEBUG	
		printk(KERN_INFO "(Line:%d)  MHL_STATUS_DCAP_RDY !!! \n", (int)__LINE__ ); 
#endif
		cbus_cmd_bind(MHL_READ_DEVCAP,MHL_DEV_CATEGORY_OFFSET,data, data_size, FALSE);
		cbus_cmd_bind(MHL_READ_DEVCAP,MHL_CAP_FEATURE_FLAG,data, data_size, FALSE);
	}
}

void process_cbus_interrrupt(void)
{
	u8 intr_cbus1, intr_cbus2;

	intr_cbus1 = ReadByteCBUS(0x08);
	WriteByteCBUS(0x08, intr_cbus1); //clear interrupt.

	if (intr_cbus1 & BIT2){
		ApplyDdcAbortSafety();
	}

	if((intr_cbus1 & BIT3)){
#ifdef MHL_DEBUG	
	  printk(KERN_INFO "Drv: MSC_MSG/RAP Received\n");
#endif
	  proccess_msc_msg_handle();
	}
  
	if((intr_cbus1 & BIT5) || (intr_cbus1 & BIT6)) // MSC_REQ_ABORT or MSC_RESP_ABORT
	{
		u8 cbus_error = 0;
#ifdef MHL_DEBUG		
		printk (KERN_INFO "((cbusInt & BIT5) || (cbusInt & BIT6)) \n");	
#endif
		cbus_error = CBusProcessErrors(intr_cbus1);

#if (0) // 20111117, kkcho_temp, for Auth
		if(intr_cbus1){
			delete_all_cbus_cmd(); 
			mhl_tx_init();
			//sii9244_mhl_tx_int();	  
			return;
		}
#else
		if(cbus_error)	{
			delete_all_cbus_cmd(); 
			mhl_tx_init();
			//sii9244_mhl_tx_int();	  
			return;
		}
#endif
	}
 
	// MSC_REQ_DONE received.
	if(intr_cbus1 & BIT4){
		cbus_cmd_done();
	}

	intr_cbus2 = ReadByteCBUS(0x1E);
	WriteByteCBUS(0x1E, intr_cbus2); //clear interrupt.  
    
	if(intr_cbus2 & INTR_CBUS2_DESIRED_MASK){
		if(intr_cbus2& BIT0){
#ifdef MHL_DEBUG		
			printk(KERN_INFO "WRITE BURST event from sink \n");
#endif
			//c_DSCR_CHG_handle_func();
		}

		if(intr_cbus2& BIT2){
			u8 cbus_intr_set0, cbus_intr_set1;
#ifdef MHL_DEBUG			
			printk(KERN_INFO "Drv: MSC RESPONDER STATUS CHANGED \n");
#endif
			cbus_intr_set0 = ReadByteCBUS(0xA0);
			WriteByteCBUS( 0xA0, cbus_intr_set0);

			cbus_intr_set1 = ReadByteCBUS(0xA1);
			WriteByteCBUS( 0xA1, cbus_intr_set1);

			mhltx_got_mhlintr( cbus_intr_set0, cbus_intr_set1 );      
		}

		if(intr_cbus2& BIT3){
			u8 status[2];
#ifdef MHL_DEBUG
			printk(KERN_INFO "Drv: MSC RESPONDER INTERRUPT POSTED \n");
#endif
			status[0] = ReadByteCBUS(0xB0);
			WriteByteCBUS( 0xB0, 0xFF);

			status[1] = ReadByteCBUS(0xB1);
			WriteByteCBUS( 0xB1, 0xFF);      
			mhltx_got_status(status[0], status[1]);  		
		}
	}  
}

struct timer_list simg_timer;

boolean resen_check_init = FALSE;
boolean rsen_timer = FALSE;  // kkcho_1118

struct rsen_check_delayed_work  {
	struct delayed_work		work;
};

struct rsen_check_delayed_work rsen_check_control;

void simg_timer_schdule(struct work_struct *work)
{
	//for CTS 3.3.14.3
	u8 sys_stat =0;

	sys_stat = I2C_ReadByte(PAGE_0_0X72, 0x09);

	if(((sys_stat & RSEN_HIGH) == 0)&&(mhl_status_value.rsen_check_available == TRUE))	{
#ifdef MHL_DEBUG	
		printk(KERN_INFO "RSEN LOW_1 ~ \n");
#endif
		// kkcho_1118 
		// mhl_status_value.mhl_status = MHL_RSEN_LOW;
		simg_msleep(110);

		sys_stat = I2C_ReadByte(PAGE_0_0X72, 0x09);
#ifdef MHL_DEBUG		
		printk(KERN_INFO "sys_stat_1: %x ~ \n", sys_stat);
#endif
		if((sys_stat & RSEN_HIGH) == 0)	{
#ifdef MHL_DEBUG		
			printk(KERN_INFO "RSEN Really LOW_1 ~ \n");
#endif
			delete_all_cbus_cmd(); 
			ForceUsbIdSwitchOpen();
			simg_msleep(50);
			ReleaseUsbIdSwitchOpen();

			//customer need to change the USB PATH
			//		MHL_On(0);
			//	msleep(10);
			mhl_disconnect_queuework(0);
			//sii9244_mhl_tx_int();
		}
	}      
}

void simg_timer_handler(unsigned long timer_type)
{
	del_timer(&simg_timer);
	rsen_timer = FALSE;	// kkcho_1118
#ifdef MHL_DEBUG
	printk(KERN_INFO "\n#### simg_timer_handler %d ####\n", (int) timer_type);
#endif
	//simg_timer_work.data = &timer_type;
	//simg_timer_work.routine = simg_timer_hanler_cb;
	//schedule_work(&simg_timer_work);
	if (!resen_check_init)	{
		INIT_DELAYED_WORK_DEFERRABLE(&rsen_check_control.work, simg_timer_schdule);
		resen_check_init = TRUE;
	}
	
	schedule_delayed_work(&rsen_check_control.work, 0);  
}

void simg_init_timer(u8 data, unsigned int msecs)
{
	init_timer(&simg_timer);
	simg_timer.function = simg_timer_handler;
	simg_timer.data = data;
	simg_timer.expires = jiffies + (MSECS_TO_JIFFIES(msecs) + 1); // 2 * HZ;
	add_timer(&simg_timer);
}

void process_mhl_discovery_start(void)
{
	u8 int4Status, intr1;

	intr1 = I2C_ReadByte(PAGE_0_0X72, 0x71);
	// kkcho_1212 I2C_WriteByte(PAGE_0_0X72, 0x71, intr1); //clear interrupt.
#ifdef MHL_DEBUG	
	printk(KERN_INFO "intr1:%x , intr1_mask_value:%x \n", intr1 , mhl_status_value.intr1_mask_value); 
#endif
	I2C_WriteByte(PAGE_0_0X72, 0x71, intr1); //clear interrupt.
	if(intr1&RSEN_CHANG){
		u8 sys_stat =0;

		sys_stat = I2C_ReadByte(PAGE_0_0X72, 0x09);
#ifdef MHL_DEBUG		
		printk(KERN_INFO "sys_stat_2: %x ~ \n", sys_stat);
#endif
		if(((sys_stat & RSEN_HIGH) == 0)&&(mhl_status_value.rsen_check_available == TRUE))	{
#ifdef MHL_DEBUG		
			printk(KERN_INFO "RSEN LOW ~ \n");
#endif
			mhl_status_value.mhl_status = MHL_RSEN_LOW;

			simg_msleep(110);

			sys_stat = I2C_ReadByte(PAGE_0_0X72, 0x09);
#ifdef MHL_DEBUG
			printk(KERN_INFO "sys_stat_3: %x ~ \n", sys_stat);
#endif
			if((sys_stat & RSEN_HIGH) == 0)	{
#ifdef MHL_DEBUG			
				printk(KERN_INFO "RSEN Really LOW_2 ~ \n");
#endif
				delete_all_cbus_cmd(); 
				ForceUsbIdSwitchOpen();
				simg_msleep(50);
				ReleaseUsbIdSwitchOpen();
				sii9244_mhl_tx_int();
				return;
			}		
			else	{
				if(rsen_timer){
					del_timer(&simg_timer);
					rsen_timer = FALSE;
				}
#ifdef MHL_DEBUG				
				printk(KERN_INFO "RSEN HIGH ~ \n");
#endif
				mhl_status_value.mhl_status = MHL_DISCOVERY_SUCCESS;				
			}	
		}
		else	{
			if(rsen_timer){
				del_timer(&simg_timer);
				rsen_timer = FALSE;
			}	
#ifdef MHL_DEBUG
			printk(KERN_INFO "RSEN HIGH ~ \n");
#endif
		}    
	}

	int4Status = I2C_ReadByte(PAGE_0_0X72, (0x74));	// read status
#ifdef MHL_DEBUG
	printk(KERN_INFO "int4Status = %02X \n ", (int)int4Status);
#endif
	I2C_WriteByte(PAGE_0_0X72, (0x74), int4Status);	// clear all interrupts

	//printk("INT PAGE_0_0X72:0x71 :%x \n" ,I2C_ReadByte(PAGE_0_0X72, 0x71));
	//printk("INT ReadByteCBUS:0x08 :%x \n" ,ReadByteCBUS(0x08));
	//printk("INT ReadByteCBUS:0x1E :%x \n" ,ReadByteCBUS(0x1E));  

	mhl_status_value.cbus_connected = ReadByteCBUS(0x0a);
#ifdef MHL_DEBUG
	printk(KERN_INFO "MHL CBUS Connected STATUS:%d \n", mhl_status_value.cbus_connected );
#endif
 
	if((int4Status & MHL_EST_INTR4)&&(mhl_status_value.cbus_connected)){
#ifdef MHL_DEBUG	
		printk(KERN_INFO "MHL EST SUCCESS ~ \n");
#endif

		mhl_status_value.mhl_status = MHL_DISCOVERY_SUCCESS;


		I2C_WriteByte(PAGE_0_0X72, 0xA0, 0x10);
		WriteByteCBUS(0x07, 0xF2);  // CBUS DDC byte handshake mode    
		// Enable segment pointer safety
		SET_BIT(PAGE_CBUS_0XC8, 0x44, 1);    
		ENABLE_DISCOVERY;    

		mhl_status_value.rsen_check_available = TRUE;

		MhlTxSetInt( MHL_RCHANGE_INT, MHL_INT_DCAP_CHG, FALSE);

		if(rsen_timer == FALSE){
			rsen_timer = TRUE;
			simg_init_timer(RSEN_400M_TIMER,T_SRC_RXSENSE_CHK+T_SRC_RSEN_DEGLITCH); //Do. add +T_SRC_RSEN_DEGLITCH for prevent MHL disconnect total 550ms // kkcho. T_SRC_RXSENSE_CHK(200) for CTS 3.3.14.3
		}	
		 
		mhl_status_value.intr1_mask_value = (RSEN_CHANG|MDI_HPD); //RSEN_CHANG; 
		mhl_status_value.intr4_mask_value = (CBUS_LOCKOUT);
		mhl_status_value.intr_cbus1_mask_value = INTR_CBUS1_DESIRED_MASK;
		mhl_status_value.intr_cbus2_mask_value = INTR_CBUS2_DESIRED_MASK;

		WRITE_INTR4_VALUE(mhl_status_value.intr4_mask_value);
		WRITE_INTR1_VALUE(mhl_status_value.intr1_mask_value);
		WRITE_CBUS1_VALUE(mhl_status_value.intr_cbus1_mask_value);
		WRITE_CBUS2_VALUE(mhl_status_value.intr_cbus2_mask_value); 

		//printk("INT PAGE_0_0X72:0x74 :%x \n" ,I2C_ReadByte(PAGE_0_0X72, 0x74));
		//printk("INT PAGE_0_0X72:0x71 :%x \n" ,I2C_ReadByte(PAGE_0_0X72, 0x71));
		//printk("INT ReadByteCBUS:0x08 :%x \n" ,ReadByteCBUS(0x08));
		//printk("INT ReadByteCBUS:0x1E :%x \n" ,ReadByteCBUS(0x1E));  

		process_cbus_interrrupt();
	}
	else if((int4Status & USB_EST_INTR4)&&(mhl_status_value.rgnd_1k == TRUE)){
#ifdef MHL_DEBUG	
		printk(KERN_INFO "MHL EST FAIL #1 ~ \n");
#endif
		mhl_status_value.mhl_status = MHL_DISCOVERY_FAIL;
		delete_all_cbus_cmd(); 
//		sii9244_mhl_tx_int();

		//!!!!!!!! disconnect
		mhl_disconnect_queuework(0);
	}
	else if(mhl_status_value.cbus_connected == 0x01){
#ifdef MHL_DEBUG	
		printk(KERN_INFO "MHL EST Not interrupt ~ \n");
#endif
		mhl_status_value.intr_cbus1_mask_value = INTR_CBUS1_DESIRED_MASK;
		mhl_status_value.intr_cbus2_mask_value = INTR_CBUS2_DESIRED_MASK;
		WRITE_CBUS1_VALUE(mhl_status_value.intr_cbus1_mask_value);
		WRITE_CBUS2_VALUE(mhl_status_value.intr_cbus2_mask_value);    
		process_cbus_interrrupt();
	} 
}

void mhltx_set_status(u8 command, u8 offset, u8 value)
{
	u8 data[16];
	u16 data_size;

	memset(data, 0x00, (sizeof(u8)*16));
	data[0] = value;
	data_size = 1;  

	cbus_cmd_bind(command,offset,data,data_size, FALSE);
}

void process_mhl_discovery_success(void)
{
	u8 intr4, intr1;

	intr1 = I2C_ReadByte(PAGE_0_0X72, 0x71);
	I2C_WriteByte(PAGE_0_0X72, 0x71, intr1); //clear interrupt.

#ifdef MHL_DEBUG
	printk(KERN_INFO "intr1:%x , intr1_mask_value:%x \n", intr1 , mhl_status_value.intr1_mask_value); 
#endif
  
	if(intr1&RSEN_CHANG){
		u8 sys_stat =0;

		sys_stat = I2C_ReadByte(PAGE_0_0X72, 0x09);
#ifdef MHL_DEBUG		
		printk(KERN_INFO "sys_stat_4: %x ~ \n", sys_stat);
#endif

		if(((sys_stat & RSEN_HIGH) == 0)&&(mhl_status_value.rsen_check_available == TRUE))	{
#ifdef MHL_DEBUG		
			printk(KERN_INFO "RSEN LOW ~ \n");
#endif
			mhl_status_value.mhl_status = MHL_RSEN_LOW;

			simg_msleep(110);

			sys_stat = I2C_ReadByte(PAGE_0_0X72, 0x09);
#ifdef MHL_DEBUG			
			printk(KERN_INFO "sys_stat_5: %x ~ \n", sys_stat);
#endif

			if((sys_stat & RSEN_HIGH) == 0)	{
#ifdef MHL_DEBUG			
				printk(KERN_INFO "RSEN Really LOW_3 ~ \n");
#endif
				delete_all_cbus_cmd(); 
				//ForceUsbIdSwitchOpen();
				//simg_msleep(50);
				//ReleaseUsbIdSwitchOpen();
				//customer need to change the USB PATH
				//	MHL_On(0);
				//	msleep(10);
				mhl_disconnect_queuework(0);

				//sii9244_mhl_tx_int();
				goto exit_func;
			}
		      else{
				if(rsen_timer){
					del_timer(&simg_timer);
					rsen_timer = FALSE;
				}
#ifdef MHL_DEBUG				
				printk(KERN_INFO "RSEN HIGH ~ \n");
#endif
				mhl_status_value.mhl_status = MHL_DISCOVERY_SUCCESS;
		      }	
		}
		else	{
			if(rsen_timer){
				del_timer(&simg_timer);
				rsen_timer = FALSE;
			}			
#ifdef MHL_DEBUG
			printk(KERN_INFO "RSEN HIGH ~ \n");
#endif
		}
	}

	if(intr1&MDI_HPD){
		u8 tmp;

		tmp = ReadByteCBUS(0x0D);
#ifdef MHL_DEBUG		
		printk(KERN_INFO "HPD :%x  ~ \n", tmp);
#endif

		if(tmp & 0x40){		
			mhl_status_value.sink_hpd = TRUE;
			set_mhl_ctrled_hpd_state(true);
			mhltx_set_status(MHL_WRITE_STAT, MHL_STATUS_REG_LINK_MODE,mhl_status_value.linkmode );
			SiiMhlTxDrvTmdsControl(TRUE); 
			MHL_Cable_On(1);  // kkcho_1212_temp		
		//pantech_hdmi_cable_detect(1);
		}
		else	{
			delete_all_cbus_cmd(); 
			mhl_status_value.sink_hpd = FALSE;
			SiiMhlTxDrvTmdsControl(FALSE);  	
			//MHL_Cable_On(0);  // kkcho_1212_temp
			//pantech_hdmi_cable_detect(0);
			//change_mhl_state(0);
		}
	} 
	//kkcho_1212_3 I2C_WriteByte(PAGE_0_0X72, 0x71, intr1); //clear interrupt.

	intr4 = I2C_ReadByte(PAGE_0_0X72, 0x74);
	I2C_WriteByte(PAGE_0_0X72, 0x74, intr4); //clear interrupt.

	//(CBUS_LOCKOUT|SCDT_CHANGE)
	if(intr4&SCDT_CHANGE){
#ifdef MHL_DEBUG	
		printk(KERN_INFO "\n SCDT_CHANGE !!!\n");  //currently not handle...
#endif
	}

	if(intr4&CBUS_LOCKOUT){
#ifdef MHL_DEBUG	
		printk(KERN_INFO "\n CBUS_LOCKOUT !!!\n"); 
#endif
		ForceUsbIdSwitchOpen();
		ReleaseUsbIdSwitchOpen();

		return;
	}

	process_cbus_interrrupt();
#ifdef MHL_DEBUG
	printk(KERN_INFO "************** Normal finish...**************\n" );
#endif
	return;

exit_func:
	printk(KERN_INFO "Goto exit_func \n" );  
}

void simg_mhl_tx_handler(void)
{
	switch (mhl_status_value.mhl_status)
	{
		case MHL_WAITING_RGND_DETECT:
#ifdef MHL_DEBUG			
			printk(KERN_INFO "(Line:%d) MHL_INIT_DONE \n", (int)__LINE__ );
#endif
			process_waiting_rgnd();
			break;

		case MHL_DISCOVERY_FAIL:
#ifdef MHL_DEBUG			
			printk(KERN_INFO "(Line:%d) MHL_DISCOVERY_FAIL \n", (int)__LINE__ );        
#endif
			break;

		case MHL_CABLE_CONNECT:
#ifdef MHL_DEBUG			
			printk(KERN_INFO "(Line:%d) MHL_RGND_DETECT \n", (int)__LINE__ );
#endif
			break;

		case MHL_DISCOVERY_START:
#ifdef MHL_DEBUG
			printk(KERN_INFO "(Line:%d) MHL_DISCOVERY_START \n", (int)__LINE__ );
#endif
			process_mhl_discovery_start();
			break;

		case MHL_DISCOVERY_SUCCESS:
		case MHL_RSEN_LOW:
#ifdef MHL_DEBUG			
			printk(KERN_INFO "(Line:%d) MHL_DISCOVERY_SUCCESS \n", (int)__LINE__ );          
#endif
			process_mhl_discovery_success();
			break;

		default:
#ifdef MHL_DEBUG			
			printk(KERN_INFO "(Line:%d) DEAFAULT:%x \n", (int)__LINE__, mhl_status_value.mhl_status );      
#endif
			break;
	}
}

static void sii9244_mhl_tx_int(void)
{
#ifdef MHL_DEBUG
	printk(KERN_INFO "(Line:%d) sii9244_mhl_tx_int\n", (int)__LINE__ );
#endif

	mhl_status_value.mhl_status = NO_MHL_STATUS;
	mhl_status_value.cbus_connected = FALSE;
	mhl_status_value.sink_hpd = FALSE;
	mhl_status_value.linkmode = 0x03; 
	mhl_status_value.connected_ready = 0;
	mhl_status_value.rsen_check_available = FALSE;

	mhl_status_value.intr4_mask_value = RGND_RDY;
	mhl_status_value.intr1_mask_value = 0x00;
	mhl_status_value.intr_cbus1_mask_value = 0x00;
	mhl_status_value.intr_cbus2_mask_value = 0x00;
	mhl_status_value.rgnd_1k = FALSE;

	memset(&mhl_reciever_cap, 0x00, sizeof(mhl_rx_cap_type));

#ifdef FIX_CBUS_BIND_POINTER_BREAK
	if(cbus_cmd_head == NULL)
		cbus_cmd_head = (cbus_cmd_node*)kmalloc(sizeof(cbus_cmd_node), GFP_KERNEL);
	if(cbus_cmd_tail == NULL)
		cbus_cmd_tail = (cbus_cmd_node*)kmalloc(sizeof(cbus_cmd_node), GFP_KERNEL);
#else
	cbus_cmd_head = (cbus_cmd_node*)kmalloc(sizeof(cbus_cmd_node), GFP_KERNEL);
	cbus_cmd_tail = (cbus_cmd_node*)kmalloc(sizeof(cbus_cmd_node), GFP_KERNEL);
#endif



	cbus_cmd_head->next = cbus_cmd_tail;
	cbus_cmd_head->prev = cbus_cmd_head;
	cbus_cmd_tail->next = cbus_cmd_tail;
	cbus_cmd_tail->prev = cbus_cmd_head;

	cbus_cmd_head->cmd_send = 0;
	cbus_cmd_tail->cmd_send = 0;

	WriteInitialRegisterValues();

	WRITE_INTR4_VALUE(mhl_status_value.intr4_mask_value);
	WRITE_INTR1_VALUE(mhl_status_value.intr1_mask_value);
	WRITE_CBUS1_VALUE(mhl_status_value.intr_cbus1_mask_value);
	WRITE_CBUS2_VALUE(mhl_status_value.intr_cbus2_mask_value);

	SwitchToD3();
	mhl_status_value.mhl_status =  MHL_WAITING_RGND_DETECT;  
}

/*===========================================================================

===========================================================================*/
//------------------------------------------------------------------------------
// Function: I2C_WriteByte
// Description:
//------------------------------------------------------------------------------
int I2C_WriteByte(u8 deviceID, u8 offset, u8 value)
{
	int err;
	struct i2c_msg msg[1];
	unsigned char data[2];
	struct i2c_client* client_ptr = get_sii9244_client(deviceID);

	msg->addr = client_ptr->addr;
	msg->flags = 0;
	msg->len = 2;
	msg->buf = data;
	data[0] = offset;
	data[1] = value;

	err = i2c_transfer(client_ptr->adapter, msg, 1);

	if (err >= 0)
		return 0; 

	return err;
}
#if 0
u8 I2C_ReadByte(u8 deviceID, u8 offset)
{
	struct i2c_msg msg[2];
	u8 reg_buf[] = { offset };
	u8 data_buf[] = { 0 };
	int err;

	struct i2c_client* client_ptr = get_sii9244_client(deviceID);

	if(!client_ptr)
	{
#ifdef MHL_DEBUG	
		printk("[MHL]I2C_ReadByte error %x\n",deviceID); 
#endif
		return 0;	
	}

	msg[0].addr = client_ptr->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = reg_buf;

	msg[1].addr = client_ptr->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = data_buf;

	err = i2c_transfer(client_ptr->adapter, msg, 2);

	if (err < 0) {
#ifdef MHL_DEBUG		
		printk(KERN_INFO "%s: I2C err: %d\n", __func__, err);
#endif
		return 0xFF; //err;
	}  

	return *data_buf;
}
#else
byte I2C_ReadByte(byte deviceID, byte offset)
{
        byte number = 0;
        struct i2c_client* client_ptr = get_sii9244_client(deviceID);
        if(!client_ptr){
        	printk("[MHL]I2C_ReadByte error %x\n",deviceID); 
        	return 0;	
        }


        if(deviceID == 0x72)
              number = sii9244_i2c_read(client_ptr,offset);
        else if(deviceID == 0x7A)
        	number = sii9244_i2c_read(client_ptr,offset);
        else if(deviceID == 0x92)
        	number = sii9244_i2c_read(client_ptr,offset);
        else if(deviceID == 0xC8)
        	number = sii9244_i2c_read(client_ptr,offset);

        return (number);

}
#endif
u8 ReadByteTPI (u8 Offset) 
{
	return I2C_ReadByte(PAGE_0_0X72, Offset);
}

void WriteByteTPI (u8 Offset, u8 Data) 
{
	I2C_WriteByte(PAGE_0_0X72, Offset, Data);
}

void ReadModifyWriteTPI(u8 Offset, u8 Mask, u8 Data) 
{
	u8 Temp;

	Temp = ReadByteTPI(Offset);		// Read the current value of the register.
	Temp &= ~Mask;					// Clear the bits that are set in Mask.
	Temp |= (Data & Mask);			// OR in new value. Apply Mask to Value for safety.
	WriteByteTPI(Offset, Temp);		// Write new value back to register.
}

u8 ReadByteCBUS (u8 Offset) 
{
	return I2C_ReadByte(PAGE_CBUS_0XC8, Offset);
}

void WriteByteCBUS(u8 Offset, u8 Data) 
{
	I2C_WriteByte(PAGE_CBUS_0XC8, Offset, Data);
}

void ReadModifyWriteCBUS(u8 Offset, u8 Mask, u8 Value) 
{
        u8 Temp;

        Temp = ReadByteCBUS(Offset);
        Temp &= ~Mask;
        Temp |= (Value & Mask);
        WriteByteCBUS(Offset, Temp);
}
int get_mhl_status(void)
{
	return mhl_status_value.mhl_status;
}
int get_mhl_power_mode(void){
	return I2C_ReadByte(PAGE_1_0x7A, 0x3D)&0x01;
}
int get_mhl_rgnd_status(void){
	return mhl_status_value.cbus_connected;
}

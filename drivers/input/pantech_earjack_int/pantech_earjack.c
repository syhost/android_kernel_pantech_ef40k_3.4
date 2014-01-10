/*
 *
 *  Pantech Earjack Driver
 *
 *  Copyright (C) 2011 Pantech, Inc.
 *
 *  Author: Sangwoo Kim, Yoonkoo Kang
 *
 */


#include <asm/atomic.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/switch.h>
#include <mach/mpp.h>                                

#include <linux/gpio.h>
#include <linux/interrupt.h>

#include <linux/regulator/consumer.h>                   
#include <linux/regulator/pmic8058-regulator.h>         
#include <linux/pmic8058-xoadc.h>

#include <linux/workqueue.h>
#include <linux/errno.h>
#include <linux/msm_adc.h>

#include <linux/spinlock.h>
#include <linux/wakelock.h>

#include <linux/irq.h>
#include <mach/irqs.h>
#include <linux/mfd/pmic8058.h>

#include <linux/timer.h>
#include <linux/delay.h>

/* -------------------------------------------------------------------- */
/* debug option */
/* -------------------------------------------------------------------- */
#define EARJACK_DBG
#ifdef EARJACK_DBG
#define dbg(fmt, args...)   pr_debug("[earjack] " fmt, ##args)
#define dbg_without_label(fmt, args...)   pr_debug(fmt, ##args)
#else
#define dbg(fmt, args...)
#define dbg_without_label(fmt, args...) 
#endif

#define EARJACK_FUNC_DBG
#ifdef EARJACK_FUNC_DBG
#define dbg_func_in()       dbg("++ %s\n", __func__)
#define dbg_func_out()      dbg("-- %s\n", __func__)
#define dbg_line()          dbg("line:%d(%s)\n", __LINE__, __func__)
#else
#define dbg_func_in()  
#define dbg_func_out() 
#define dbg_line()     
#endif
/* -------------------------------------------------------------------- */
#if defined (CONFIG_MACH_MSM8X60_EF33S)||defined (CONFIG_MACH_MSM8X60_EF34K)||defined (CONFIG_MACH_MSM8X60_EF34C)||defined (CONFIG_MACH_MSM8X60_EF35L)
#define EARJACK_POLL_MODE_FEATURE //p12911 : Rc Bad FET Devices
#endif

#if defined(CONFIG_PANTECH_EF40K_BOARD) || defined(CONFIG_PANTECH_EF40S_BOARD) || defined(CONFIG_PANTECH_EF65L_BOARD)
#define FEATURE_TOUCH_EARJACK_TEST //TOUCH issue 20111202_kmh_sensor	
#endif

#ifdef EARJACK_POLL_MODE_FEATURE
typedef enum{
	EARJACK_FET_LOW = 0, //dont insert the earjack and the detect pin is low.
	EARJACK_FET_HIGH = 1, //insert the earjack and the detect pin is high.
}earjack_state;

//check the earjack 
typedef enum{ 
	NONE_EARJACK_INSERTED = 0,
	POLAR3_EARJACK_INSERTED = 1,
	POLAR4_EARJACK_INSERTED = 2,
} earjack_current_type;

typedef struct{
	earjack_current_type type;
	uint16_t min;
	uint16_t max;   
} Earjack_current_state;

static Earjack_current_state current_earjack_type[] ={                       
	{ NONE_EARJACK_INSERTED, 2110,	2300},
	{ POLAR4_EARJACK_INSERTED,1600,  2070},
	{ POLAR3_EARJACK_INSERTED, 0,  40},    
};
earjack_current_type pantech_earjack_type; //3 // 3 and 4 pole check

earjack_state earjack_states=EARJACK_FET_LOW;
static int earjack_fet_success=0;

#define EARJACK_DETECT_POLLING_TIME					50	// check the earjack timming
#define EARJACK_CARKIT_EVENT_POLLING_TIME			30	// check the carkit time
#define EARJACK_REMOTE_KEY_EVENT_POLLING_TIME		3	// 10 sec

static earjack_state earjack_check_det_pin(void);
static int earjack_poll_check_inserted(void);
static int is_earjack_insert_by_mpp(void);
static int check_analog_mpp(int channel,int *mv_reading);
#endif //EARJACK_POLL_MODE_FEATURE
/* -------------------------------------------------------------------- */

#define DRIVER_NAME	"pantech_earjack"

#define EARJACK_DET     125 	/* Earjack sensing interrupt pin No. */
#define REMOTEKEY_DET   215     /* IRQ's:173 + PM GPIOs:40 + MPP No.:3 + INDEX:-1 = 215 */

#define 	BIT_HEADSET 					1
#define	BIT_HEADSET_SPEAKER_ONLY 	2
#define	BIT_HEADSET_MIC_ONLY 			4

#define MAX_ADC_READ_COUNT		3
#define MAX_ADC_VALUE				1000

#define REMOTEKEY_DET_ACTIVE_LOW // select if remoteky_det pin is active low


#ifdef EARJACK_DET_ACTIVE_LOW
	#define EARJACK_INSERTED !gpio_get_value_cansleep(EARJACK_DET)
#else /*EARJACK_ACTIVE_HIGH*/
	#define EARJACK_INSERTED  gpio_get_value_cansleep(EARJACK_DET)
#endif /*EARJACK_DET_ACTIVE_LOW*/

#define EARJACK_RELEASED !EARJACK_INSERTED

#ifdef REMOTEKEY_DET_ACTIVE_LOW 
#define REMOTEKEY_PRESSED !gpio_get_value_cansleep(REMOTEKEY_DET)
#else
#define REMOTEKEY_PRESSED  gpio_get_value_cansleep(REMOTEKEY_DET)
#endif
#define REMOTEKEY_RELEASED !REMOTEKEY_PRESSED

#define PM8058_GPIO_BASE						NR_MSM_GPIOS
#define PM8058_MPP_BASE						(PM8058_GPIO_BASE + PM8058_GPIOS)
#define PM8058_MPP_PM_TO_SYS(pm_gpio)		(pm_gpio + PM8058_MPP_BASE)
#define  ARR_SIZE( a )  ( sizeof( (a) ) / sizeof( (a[0]) ) )

typedef enum{
	EARJACK_STATE_OFF,
	EARJACK_STATE_ON_3POLE_CHECK,
	EARJACK_STATE_ON,
	EARJACK_STATE_CHECK,
}earjack_type;

static struct regulator *hs_jack_l8;

//static struct regulator *hs_jack_s3;

// Pantech Earjack Structure
struct pantech_earjack {
	struct input_dev *ipdev;
    	struct switch_dev sdev;
	earjack_type type;
	bool mic_on;
	bool hs_on;
	bool remotekey_pressed;
	int remotekey_index;
	bool car_kit;
};
static struct pantech_earjack *earjack;

// Interrupt Handlers
static irqreturn_t Earjack_Det_handler(int irq, void *dev_id);
static irqreturn_t Remotekey_Det_handler(int irq, void *dev_id);

// Locks
static struct wake_lock earjack_wake_lock;
static struct wake_lock remotekey_wake_lock;

// Remote Key 
typedef struct{
	char* key_name;
	int key_index;
	int min;
	int max;
} remotekey;

#if defined (CONFIG_MACH_MSM8X60_EF33S) || defined(CONFIG_MACH_MSM8X60_EF34K)||defined (CONFIG_MACH_MSM8X60_EF35L)
// Default : 2.4V
static remotekey remotekey_type[] ={ 
	{ "NO_KEY",		0,			2400,	2600},
	{ "KEY_MEDIA",		KEY_MEDIA,		45,	130},
	{ "KEY_VOLUMEDOWN",	KEY_VOLUMEDOWN,		420,	620},
	{ "KEY_VOLUMEUP",	KEY_VOLUMEUP,		230,	390},
	{ "KEY_MEDIA_CARKIT",	KEY_MEDIA,		0,	40}, // car_kit - should be on the last position 
};
#else
// Default : 2.7V
static remotekey remotekey_type[] ={ 
	{ "NO_KEY",		0,			2400,	2600},
	{ "KEY_MEDIA",		KEY_MEDIA,		60,	130},
	{ "KEY_VOLUMEDOWN",	KEY_VOLUMEDOWN,		500,	700},
	{ "KEY_VOLUMEUP",	KEY_VOLUMEUP,		250,	420},
	{ "KEY_MEDIA_CARKIT",	KEY_MEDIA,		0,	50}, // car_kit - should be on the last position 
};
#endif

static struct delayed_work earjack_work;
static struct delayed_work remotekey_work;

int irq_detect=0;
int irq_remote=0;
int wake_state=0;
int remotekey_wake=0;
static void earjack_detect_func(struct work_struct * earjack_work);
static void remotekey_detect_func(struct work_struct * remotekey_work);
#ifdef FEATURE_TOUCH_EARJACK_TEST 
extern void pantech_touch_earjack(int flag);
#endif
static void  pm8058_mpp_config_AMUX(void)
{
	int ret;

       struct pm8xxx_mpp_config_data sky_handset_analog_adc = {
			.type	= PM8XXX_MPP_TYPE_A_INPUT,
			.level	= PM8XXX_MPP_AIN_AMUX_CH5,
			.control = PM8XXX_MPP_AOUT_CTRL_DISABLE,	
		};	
     
	ret = pm8xxx_mpp_config(PM8058_MPP_PM_TO_SYS(XOADC_MPP_3), &sky_handset_analog_adc);	
       
    	if (ret)
   		pr_err("%s: pm8058_mpp_config_AMUX ret=%d\n",__func__, ret);	
}
static void  pm8058_mpp_config_DIG(void)
{
	int ret;

       struct pm8xxx_mpp_config_data sky_handset_digital_adc = {
			.type	= PM8XXX_MPP_TYPE_D_INPUT,
			.level	= PM8058_MPP_DIG_LEVEL_S3,
			.control = PM8XXX_MPP_DIN_TO_INT,	
		};

	ret = pm8xxx_mpp_config(PM8058_MPP_PM_TO_SYS(XOADC_MPP_3), &sky_handset_digital_adc);

	if (ret < 0) 
		pr_err("%s: pm8058_mpp_config_DIG ret=%d\n",__func__, ret);
}
static int disable_irq_remote(void)
{
	if(irq_remote == 0) {
		disable_irq_nosync(gpio_to_irq(REMOTEKEY_DET));
		irq_remote = 1;
	}else{
		pr_debug("[EARJACK REMOTE] Already disable irq\n");
	}	
	return irq_remote;
}
static int enable_irq_remote(void)
{
	dbg_func_in();	
	if(irq_remote == 1) {
		enable_irq(gpio_to_irq(REMOTEKEY_DET));
		irq_remote = 0;
	}else{
		pr_debug("[EARJACK REMOTE] Already enable irq\n");
	}	
	return irq_remote;
}


static void disable_irq_detect(void)
{
	dbg_func_in();	
	if(irq_detect == 0) {
		disable_irq_nosync(gpio_to_irq(EARJACK_DET));
		irq_detect = 1;
	}else{
		pr_debug("[EARJACK DETECT] Already disable irq\n");
	}	
}
static void enable_irq_detect(void)
{
	dbg_func_in();	
	if(irq_detect == 1) {
		enable_irq(gpio_to_irq(EARJACK_DET));
		irq_detect = 0;
	}else{
		pr_debug("[EARJACK DETECT] Already enable irq\n");
	}	
}
void earjack_prevent_suspend(void)
{
	dbg_func_in();
	if(!wake_state)
		{
		wake_lock(&earjack_wake_lock);
		wake_state=1;
		}
	dbg_func_out();
}
void earjack_allow_suspend(void)
{

	dbg_func_in();
	if(wake_state)
		{
		wake_unlock(&earjack_wake_lock);
		wake_state=0;		
		}
	dbg_func_out();
}
void remotekey_prevent_suspend(void)
{
	dbg_func_in();
	if(!remotekey_wake)
		{
		wake_lock(&remotekey_wake_lock);
		remotekey_wake=1;
		}
	dbg_func_out();
}
void remotekey_allow_suspend(void)
{

	dbg_func_in();
	if(remotekey_wake)
		{
		wake_unlock(&remotekey_wake_lock);
		remotekey_wake=0;		
		}
	dbg_func_out();
}


// Switch Device Functions
static ssize_t msm_headset_print_name(struct switch_dev *sdev, char *buf)
{
	switch (switch_get_state(&earjack->sdev)) {
	case 0: //NO_DEVICE:
		return sprintf(buf, "No Device\n");
	case 1: // MSM_HEADSET:
		return sprintf(buf, "Headset\n");
  default : //  MSM_HEADSET for 3pole earjack //kyk
    return sprintf(buf, "Headset\n");
	}
	return -EINVAL;
}

int switch_state(void)
{
	int state;
	if (earjack->mic_on && earjack->hs_on)
		state = BIT_HEADSET;
	else if (earjack->hs_on)
		state = BIT_HEADSET_SPEAKER_ONLY;
	else if (earjack->mic_on)
		state = BIT_HEADSET_MIC_ONLY;
	else
		state = 0;

	return state;
}

// Sysfs 
static ssize_t show_headset(struct device *dev, struct device_attribute *attr
, char *buf)
{
	int conn_headset_type = switch_state();
	return snprintf(buf, PAGE_SIZE, "%d\n", conn_headset_type);
}

static ssize_t set_headset(struct device *dev, struct device_attribute *attr, 
const char *buf, size_t count)
{
    return 0;
}

static DEVICE_ATTR(headset, S_IRUGO | S_IWUSR, show_headset, set_headset);

static struct attribute *dev_attrs[] = {
    &dev_attr_headset.attr,
    NULL,
};

static struct attribute_group dev_attr_grp = {
    .attrs = dev_attrs,
};


// Pantech Earjack Probe Function
static int __devinit pantech_earjack_probe(struct platform_device *pdev)
{
	int rc = 0;
	struct input_dev *ipdev;

	dbg_func_in();


	// Alloc Devices
	earjack = kzalloc(sizeof(struct pantech_earjack), GFP_KERNEL);
	if (!earjack)
		return -ENOMEM;

	earjack->sdev.name	= "hw2";     
	earjack->sdev.print_name = msm_headset_print_name;
	rc = switch_dev_register(&earjack->sdev);
	if (rc)
		goto err_switch_dev_register;

	ipdev = input_allocate_device();
	if (!ipdev) {
		rc = -ENOMEM;
		goto err_alloc_input_dev;
	}
	input_set_drvdata(ipdev, earjack);


	// Init Status Flags
	earjack->ipdev = ipdev;
	earjack->car_kit = 0;
	earjack->type=EARJACK_STATE_OFF;
	earjack->remotekey_pressed = 0;
	earjack->remotekey_index = 0;

	// Initialize Work Queue
	INIT_DELAYED_WORK(&earjack_work,earjack_detect_func);          // INIT WORK
	INIT_DELAYED_WORK( &remotekey_work, remotekey_detect_func );

	// Get Power Source
	hs_jack_l8 = regulator_get(NULL, "8058_l8");
	
	if (IS_ERR(hs_jack_l8)) {
		rc = PTR_ERR(hs_jack_l8);
		printk(KERN_ERR "%s: regulator get of %s failed (%d)\n",
			__func__, "hs_jack_l8", rc);
	}
	
	rc = regulator_set_voltage( hs_jack_l8, 2700000, 2700000 );
	// Initialize Wakelocks
	wake_lock_init(&earjack_wake_lock, WAKE_LOCK_SUSPEND, "earjack_wake_lock_init");
	wake_lock_init(&remotekey_wake_lock, WAKE_LOCK_SUSPEND, "remotekey_wake_lock_init");

	// Get GPIO's
	gpio_request(EARJACK_DET, "earjack_det");
	gpio_request(REMOTEKEY_DET, "remotekey_det");
	gpio_tlmm_config(GPIO_CFG(EARJACK_DET, 0, GPIO_CFG_INPUT,GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
//	gpio_tlmm_config(GPIO_CFG(EARJACK_DET, 0, GPIO_CFG_INPUT,GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
#ifdef EARJACK_POLL_MODE_FEATURE
	earjack_check_det_pin();
#endif

	rc = request_irq(gpio_to_irq(EARJACK_DET), Earjack_Det_handler, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING| IRQF_NO_SUSPEND, "earjack_det-irq", earjack);

	// Init Mutex
	irq_set_irq_wake(gpio_to_irq(EARJACK_DET), 1);
	irq_set_irq_wake(gpio_to_irq(REMOTEKEY_DET), 1);

	// Init Input Device
	ipdev->name	= DRIVER_NAME;
	ipdev->id.vendor    = 0x0001;
	ipdev->id.product   = 1;
	ipdev->id.version   = 1;

	input_set_capability(ipdev, EV_KEY, KEY_MEDIA);
	input_set_capability(ipdev, EV_KEY, KEY_VOLUMEUP);
	input_set_capability(ipdev, EV_KEY, KEY_VOLUMEDOWN);
	input_set_capability(ipdev, EV_KEY, KEY_POWER);    
	input_set_capability(ipdev, EV_KEY, KEY_END);
	input_set_capability(ipdev, EV_SW,  SW_HEADPHONE_INSERT);
	input_set_capability(ipdev, EV_SW,  SW_MICROPHONE_INSERT);

	rc = input_register_device(ipdev);
	if (rc) {
		dev_err(&ipdev->dev,
				"hs_probe: input_register_device rc=%d\n", rc);
		goto err_reg_input_dev;
	}
	platform_set_drvdata(pdev, earjack);

	rc = sysfs_create_group(&pdev->dev.kobj, &dev_attr_grp);
	if (rc) {
		dev_err(&ipdev->dev,
				"hs_probe: sysfs_create_group rc=%d\n", rc);
		goto err_earjack_init;
	}


	// Scehdule earjack_detect_func for initial detect
	disable_irq_detect(); //disable_irq_nosync(gpio_to_irq(EARJACK_DET));
	earjack_prevent_suspend();
	schedule_delayed_work(&earjack_work,10);    // after 100ms
	
	dbg_func_out();
	return 0;

err_earjack_init:

err_reg_input_dev:
	input_unregister_device(ipdev);
	ipdev = NULL;
err_alloc_input_dev:
	input_free_device(ipdev);
err_switch_dev_register:
	kfree(earjack);
	return 0;
}

bool is_4pole_earjack (void) 
{
	bool retVal;
	int err;
	if (gpio_cansleep(REMOTEKEY_DET)){
		err=gpio_get_value_cansleep(REMOTEKEY_DET);
		dbg("gpio_get_value_cansleep(REMOTEKEY_DET) start\n");                    
	}else{
		err=gpio_get_value(REMOTEKEY_DET);
		dbg("gpio_get_value(REMOTEKEY_DET) start\n");
	}
	retVal = (err>0) ? 1:0;
	return retVal;
}
#ifdef EARJACK_POLL_MODE_FEATURE
bool is_remote_key_low(void) 
{
	bool retVal;
	int err;
	if (gpio_cansleep(REMOTEKEY_DET)){
		err=gpio_get_value_cansleep(REMOTEKEY_DET);
		dbg("gpio_get_value_cansleep(REMOTEKEY_DET) start\n");                    
	}else{
		err=gpio_get_value(REMOTEKEY_DET);
		dbg("gpio_get_value(REMOTEKEY_DET) start\n");
	}
	retVal = (err>0) ? 0:1;
	return retVal;
}
static earjack_state earjack_check_det_pin(void)
{
	int ret; //check the gpio 
	if (gpio_cansleep(EARJACK_DET)){
		ret=gpio_get_value_cansleep(EARJACK_DET);
		dbg("gpio_get_value_cansleep(EARJACK_DET) start\n");                    
	}else{
		ret=gpio_get_value(EARJACK_DET);
		dbg("gpio_get_value(EARJACK_DET) start\n");
	}	
	if(ret) //inserted the pin and If earjack detect pin is high, FET IC is failed or .
	{				
		if(earjack_fet_success) //check the interrupt occured,
			earjack_states=EARJACK_FET_LOW;				
		else
			earjack_states=EARJACK_FET_HIGH;		
	}
	else
	{
		earjack_states=EARJACK_FET_LOW;
	}
	return earjack_states;
}

static int earjack_poll_check_inserted(void)  // detect key sensing                                 
{
	int nAdcValue = 0;
	int i;
	int ret=-1;
	dbg_func_in();
	check_analog_mpp(CHANNEL_ADC_HDSET,&nAdcValue);             // read analog input mpp_3    
	for( i = 0; i< ARR_SIZE( current_earjack_type ); i++ ) {																			// set current earjack state
  	if( nAdcValue >= current_earjack_type[i].min && nAdcValue <= current_earjack_type[i].max ){
			ret=i;
			break;
		}
	}
	pr_debug("[EARJACK_POLL_MODE_FEATURE] earjack_poll_check_inserted nAdcValue : %d ret : %d\n", nAdcValue, ret);	
	dbg_func_out();
	return ret;
}

     
static int is_earjack_insert_by_mpp(void)
{
	int state=0;
	int err=0;
	int mmp_value=0;
	//enable the l8 regulator
	state=regulator_is_enabled(hs_jack_l8);
	if(state<=0)  err = regulator_enable(hs_jack_l8);	
	pm8058_mpp_config_AMUX();
	// read adc value twice 
	mmp_value = earjack_poll_check_inserted();
	pm8058_mpp_config_DIG();
	//disable the l8 regulator	
	dbg("[EARJACK_POLL_MODE_FEATURE] is_earjack_insert_by_mpp [%d] \n",mmp_value);	
	if(state<=0)
	{
	err=regulator_is_enabled(hs_jack_l8);
	dbg("[EARJACK_POLL_MODE_FEATURE] REGULATOR (DISABLED)=> %d\n",err);
	if(err>0) regulator_disable(hs_jack_l8);	
	}
	else
	dbg("[EARJACK_POLL_MODE_FEATURE] REGULATOR [ENABLED] => %d\n",err);		
	return mmp_value;
}
#endif
static irqreturn_t Earjack_Det_handler(int irq, void *dev_id)
{
	//dump_stack();
	disable_irq_detect(); //disable_irq_nosync(gpio_to_irq(EARJACK_DET));
#ifdef EARJACK_POLL_MODE_FEATURE	
	earjack_fet_success++;
#endif	

	earjack_prevent_suspend();
	schedule_delayed_work(&earjack_work, 30);    // after 100ms start function of earjack_detect_func
	dbg_func_out();
	return IRQ_HANDLED;
}
static void earjack_detect_func( struct work_struct *test_earjack)         
{
	int err;

#ifdef EARJACK_POLL_MODE_FEATURE					
	int aflag=0;
	int need_schdule=0;
#endif
	dbg_func_in();
	dbg("currnet earjack->type: ");
	switch(earjack->type){
		case    EARJACK_STATE_OFF : 
			{
				dbg_without_label("EARJACK_STATE_OFF\n");
#ifdef EARJACK_POLL_MODE_FEATURE						
				if((EARJACK_FET_HIGH == earjack_check_det_pin()) && (earjack_fet_success == 0))
				{	
					if(EARJACK_INSERTED){
						if(is_earjack_insert_by_mpp()) //is insert the 3 pole or 4pole earjack 
						{
							need_schdule=0;
						}else{
							need_schdule=1;
							goto poll_end;
						}							
					}
				}
#endif	
				if(EARJACK_INSERTED){
					earjack->type = EARJACK_STATE_CHECK;
					err=regulator_is_enabled(hs_jack_l8);
					if(err<=0)  err = regulator_enable(hs_jack_l8);
					pm8058_mpp_config_DIG();					
					schedule_delayed_work(&earjack_work, 5); //50ms
					// return without enable IRQ, wake_unlock.
					return;
				}
				break;
			}
		case    EARJACK_STATE_CHECK  :   
			{
				dbg_without_label("EARJACK_STATE_CHECK\n");   
				if(EARJACK_INSERTED)
				{
					// 3pole
					if(!is_4pole_earjack()){
						dbg("3pole headset inserted.\n");
						earjack->type= EARJACK_STATE_ON_3POLE_CHECK;
						earjack->mic_on = 0;
						earjack->hs_on = 1;
						input_report_switch(earjack->ipdev, SW_HEADPHONE_INSERT,earjack->hs_on);						
						switch_set_state(&earjack->sdev, switch_state());
#ifdef EARJACK_POLL_MODE_FEATURE												
						pantech_earjack_type=POLAR3_EARJACK_INSERTED;
#endif
						schedule_delayed_work(&earjack_work, 60);   // check if 4pole 600ms
						// return without enable IRQ, wake_unlock.
						return;
					}
					//4pole
					else {
						dbg("4pole headset inserted.\n");
						earjack->type= EARJACK_STATE_ON;
#ifdef EARJACK_POLL_MODE_FEATURE						
						pantech_earjack_type=POLAR4_EARJACK_INSERTED;
						if((EARJACK_FET_HIGH == earjack_check_det_pin()) && (earjack_fet_success == 0))
						{
							err=request_threaded_irq(gpio_to_irq(REMOTEKEY_DET),NULL,Remotekey_Det_handler,IRQF_TRIGGER_FALLING | IRQF_NO_SUSPEND, "remote_det-irq", earjack);
						}else{
							err=request_threaded_irq(gpio_to_irq(REMOTEKEY_DET),NULL,Remotekey_Det_handler,IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_NO_SUSPEND, "remote_det-irq", earjack);
						}
#else
						err=request_threaded_irq(gpio_to_irq(REMOTEKEY_DET),NULL,Remotekey_Det_handler,IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_NO_SUSPEND, "remote_det-irq", earjack);
						
#endif
						if(err) 
							dbg("request_threaded_irq failed\n");
						earjack->mic_on = 1;
						earjack->hs_on = 1;
						input_report_switch(earjack->ipdev, SW_MICROPHONE_INSERT,earjack->mic_on); //TODO: NO USE?
						input_report_switch(earjack->ipdev, SW_HEADPHONE_INSERT,earjack->hs_on);
						switch_set_state(&earjack->sdev, switch_state());
#ifdef FEATURE_TOUCH_EARJACK_TEST
					      pantech_touch_earjack(1); // earjack insert
#endif					      
#ifdef EARJACK_POLL_MODE_FEATURE	
						if((EARJACK_FET_HIGH == earjack_check_det_pin()) && (earjack_fet_success == 0))
						{	
								need_schdule=1;
								goto poll_end;
							}
#endif
					}
				}
				else // if EARJACK_RELEASED
				{
					earjack->type = EARJACK_STATE_OFF;                
					dbg("earjack_type: -> EARJACK_STATE_OFF");
					#ifdef FEATURE_TOUCH_EARJACK_TEST
					   pantech_touch_earjack(0); // earjack release
					#endif
				}

				break; // case EARJACK_STATE_CHECK 
			}

		case    EARJACK_STATE_ON_3POLE_CHECK  :   
			{                        
				// CHECKING IF 4POLE EARJACK IS INSERTIND?
				dbg_without_label("EARJACK_STATE_ON_3POLE_CHECK\n");   
				if(EARJACK_INSERTED){
					earjack->type= EARJACK_STATE_ON;                   
						#ifdef FEATURE_TOUCH_EARJACK_TEST
					      pantech_touch_earjack(1); // earjack insert
					    #endif
					if(!is_4pole_earjack()){
						dbg("3pole earjack insert.\n");
#ifdef EARJACK_POLL_MODE_FEATURE												
						pantech_earjack_type=POLAR3_EARJACK_INSERTED;
#endif
            			err=regulator_is_enabled(hs_jack_l8);
						dbg("regulator_is_enabled(hs_jack_l8) value => %d\n",err);
						if(err>0) regulator_disable(hs_jack_l8);
					}
					else {
						dbg("4pole earjack insert.\n");
						earjack->mic_on =1;
						input_report_switch(earjack->ipdev, SW_MICROPHONE_INSERT,earjack->mic_on);
    						switch_set_state(&earjack->sdev, switch_state());
#ifdef EARJACK_POLL_MODE_FEATURE						
						pantech_earjack_type=POLAR4_EARJACK_INSERTED;
						if((EARJACK_FET_HIGH == earjack_check_det_pin()) && (earjack_fet_success == 0))
						{
							err=request_threaded_irq(gpio_to_irq(REMOTEKEY_DET),NULL,Remotekey_Det_handler,IRQF_TRIGGER_FALLING | IRQF_NO_SUSPEND, "remote_det-irq", earjack);
						}else{
							err=request_threaded_irq(gpio_to_irq(REMOTEKEY_DET),NULL,Remotekey_Det_handler,IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_NO_SUSPEND, "remote_det-irq", earjack);
						}
#else
						err=request_threaded_irq(gpio_to_irq(REMOTEKEY_DET),NULL,Remotekey_Det_handler,IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_NO_SUSPEND, "remote_det-irq", earjack);
						
#endif
						if(err) dbg("request_threaded_irq failed\n");
					}
#ifdef EARJACK_POLL_MODE_FEATURE								
					if((EARJACK_FET_HIGH == earjack_check_det_pin()) && (earjack_fet_success == 0)){
						need_schdule=1;			
					}
#endif	
				}
				else{
					err=regulator_is_enabled(hs_jack_l8);
					dbg("regulator_is_enabled(hs_jack_l8) value => %d\n",err);
					if(err>0) regulator_disable(hs_jack_l8);
					earjack->type = EARJACK_STATE_OFF; 
					earjack->hs_on=0;
					input_report_switch(earjack->ipdev, SW_HEADPHONE_INSERT,earjack->hs_on);
    					switch_set_state(&earjack->sdev, switch_state());
				}
				break;   
			}

		case EARJACK_STATE_ON:   
			{
				dbg_without_label("EARJACK_STATE_ON\n");
#ifdef EARJACK_POLL_MODE_FEATURE								
				if((EARJACK_FET_HIGH == earjack_check_det_pin()) && (earjack_fet_success == 0)){
					if(EARJACK_INSERTED){
						need_schdule=1;			
						if(pantech_earjack_type==POLAR3_EARJACK_INSERTED){ //insert 3POLE
							dbg("[EARJACK_POLL_MODE_FEATURE] 3 POLE EARJACK_CHECK\n");						
							if( is_earjack_insert_by_mpp()==0)//disconnect the earjack. 
							{
								aflag=1;
								dbg("3 POLE EARJACK_STATE_RELEASE\n");
							}
						}
						else
						{
							if(pantech_earjack_type==POLAR4_EARJACK_INSERTED) //insert 4POLE
							{
								dbg("[EARJACK_POLL_MODE_FEATURE] 4 POLE EARJACK_CHECK\n");														
								disable_irq_nosync(gpio_to_irq(REMOTEKEY_DET));		
								if(!irq_remote) //Key is pressed, the state does not need to read the adc value.
								if( is_earjack_insert_by_mpp()==0)//disconnect the earjack. 
								{
									aflag=1;
									dbg("[EARJACK_POLL_MODE_FEATURE] 4 POLE EARJACK_STATE_RELEASE\n");
								}
								enable_irq(gpio_to_irq(REMOTEKEY_DET));								
							}
						}
					}
				}
				if(EARJACK_RELEASED || aflag){
#else
				if(EARJACK_RELEASED){			
#endif
#ifdef EARJACK_POLL_MODE_FEATURE								
					if(earjack_fet_success == 0) {
						cancel_delayed_work(&remotekey_work);
					}
#endif
					earjack->type = EARJACK_STATE_OFF;
					#ifdef FEATURE_TOUCH_EARJACK_TEST
					      pantech_touch_earjack(0); // earjack insert
					#endif					
					// if 4pole
					if (earjack->mic_on) {
						earjack->mic_on = 0;
						input_report_switch(earjack->ipdev, SW_MICROPHONE_INSERT,earjack->mic_on);
						// free remote key irq and turn off mic power.
						free_irq(gpio_to_irq(REMOTEKEY_DET), earjack);
            			err=regulator_is_enabled(hs_jack_l8);
						dbg("regulator_is_enabled(hs_jack_l8) value => %d\n",err);
						if(err>0){
							regulator_disable(hs_jack_l8);
						}

						// release remote key if pressed	
						if(earjack->remotekey_pressed){
							earjack->remotekey_pressed = 0;
							if (earjack->remotekey_index != 0) 
								input_report_key(earjack->ipdev, remotekey_type[earjack->remotekey_index].key_index, earjack->remotekey_pressed);
							dbg("remote key: %s : %d->%d \n", remotekey_type[earjack->remotekey_index].key_name, !earjack->remotekey_pressed, earjack->remotekey_pressed);
							input_sync(earjack->ipdev);
						}                            


					}
					dbg("[EARJACK] EARJACK 3.5 PIPE RELEASE \n");					
					earjack->hs_on = 0;
					input_report_switch(earjack->ipdev, SW_HEADPHONE_INSERT,earjack->hs_on);
    				switch_set_state(&earjack->sdev, switch_state());
					dbg("[EARJACK] EARJACK 3.5 PIPE RELEASE END\n");															
				}
				break;
			}                    
		default :   
			printk("Error : P12911 earjack_detect_func default.\n");
		break;
	}
#ifdef EARJACK_POLL_MODE_FEATURE	
	poll_end :
	if(need_schdule)
	schedule_delayed_work(&earjack_work, EARJACK_DETECT_POLLING_TIME); //50ms			
#endif
	enable_irq_detect(); //enable_irq(gpio_to_irq(EARJACK_DET));
	earjack_allow_suspend(); 
	dbg_func_out();
	return;
}

static int check_analog_mpp(int channel,int *mv_reading)                   // read adc value 
{
	int ret;
	void *h;
	struct adc_chan_result adc_chan_result;
	struct completion  conv_complete_evt;
	dbg_func_in();
	ret = adc_channel_open(channel, &h);
	if (ret) {
		pr_err("%s: couldnt open channel %d ret=%d\n",
				__func__, channel, ret);
		goto out;
	}
	init_completion(&conv_complete_evt);
	ret = adc_channel_request_conv(h, &conv_complete_evt);
	if (ret) {
		pr_err("%s: couldnt request conv channel %d ret=%d\n",
				__func__, channel, ret);
		goto out;
	}
	wait_for_completion(&conv_complete_evt);
	ret = adc_channel_read_result(h, &adc_chan_result);
	if (ret) {
		pr_err("%s: couldnt read result channel %d ret=%d\n",
				__func__, channel, ret);
		goto out;
	}
	ret = adc_channel_close(h);
	if (ret) {
		pr_err("%s: couldnt close channel %d ret=%d\n",
				__func__, channel, ret);
	}
	if (mv_reading)
		*mv_reading = adc_chan_result.measurement;

	pr_debug("%s: done for %d\n", __func__, channel);
	dbg_func_out();
	return adc_chan_result.physical;
out:
	pr_debug("%s: done for %d\n", __func__, channel);
	return -EINVAL;
}

static int adc_value_to_key(void)  // detect key sensing                                 
{
	int nAdcValue = 0;
	int i;
	dbg_func_in();
	check_analog_mpp(CHANNEL_ADC_HDSET,&nAdcValue);             // read analog input mpp_3    
	dbg("analog intput mpp_03 value : %d\n", nAdcValue);
	for( i = 1; i< ARR_SIZE( remotekey_type); i++ ) {
		if( nAdcValue >= remotekey_type[i].min && nAdcValue <= remotekey_type[i].max ) {
			// Check if car_kit
			earjack->car_kit = 0;
			if(i == ARR_SIZE(remotekey_type)-1 )
				earjack->car_kit = 1;
			// Return key index
			dbg_func_out();  	
			return i;
		}
	}
	// No Key, return 0
	dbg_func_out();  	
	return 0;
}

static irqreturn_t Remotekey_Det_handler(int irq, void *dev_id)                 // isr_Remotekey_Det_handler
{
	dbg_func_in();
	// if 3pole headset, return. 
	if(!earjack->mic_on) {
		return IRQ_HANDLED;
	}
	// disable_irq, wake_lock
	//dump_stack();
	disable_irq_remote();
	remotekey_prevent_suspend();
	schedule_delayed_work(&remotekey_work, 0);
//	schedule_delayed_work( &remotekey_work, 50 );
	dbg_func_out();
	return IRQ_HANDLED;        
}

static void remotekey_detect_func(struct work_struct * test_remotekey_work)
{
	int key_first=0;
	int key_second=0;    
	//	int key_third=0;    
	dbg_func_in();
	// car kit
	if(earjack->car_kit ){
		if(EARJACK_INSERTED){
			dbg("report KEY_MEDIA input from car_kit");
			input_report_key(earjack->ipdev, KEY_MEDIA, earjack->remotekey_pressed);		// car kit button down
			earjack->car_kit = 0; // reset car_kit flag
			if(REMOTEKEY_RELEASED) {
				earjack->remotekey_pressed = 0;
				input_report_key(earjack->ipdev, KEY_MEDIA, earjack->remotekey_pressed);	// car kit button up
			}
		}		
		earjack->car_kit = 0; // reset car_kit flag.
		enable_irq_remote();
		remotekey_allow_suspend();
		dbg_func_out();
		return ;
	}
	// if current state: key not pressed
	if(!earjack->remotekey_pressed){ 
		dbg("[remotekey_detect_func] current state: remotekey not pressed, read adc value. \n");
		pm8058_mpp_config_AMUX();
		// read adc value twice 
		key_first = adc_value_to_key();
		key_second = adc_value_to_key(); // after 20ms (adc reading delay)
		pm8058_mpp_config_DIG();

		// is valid key && earjack inserted 
		if( (key_first==key_second) && EARJACK_INSERTED){
			earjack->remotekey_index = key_first;
			earjack->remotekey_pressed = 1; 
			if (earjack->remotekey_index != 0 && !earjack->car_kit)
			{
				input_report_key(earjack->ipdev, remotekey_type[earjack->remotekey_index].key_index, earjack->remotekey_pressed);
				printk("remote key: %s : %d->%d \n", remotekey_type[earjack->remotekey_index].key_name, !earjack->remotekey_pressed, earjack->remotekey_pressed);
			}	
			else if (earjack->car_kit)
			{
				schedule_delayed_work(&remotekey_work, 20);	// for car kit : delayed work after 200ms for __  TODO
				return;
			}
			
			// Defense code :key pressed but released during processing key. 
			if(REMOTEKEY_RELEASED){
				earjack->remotekey_pressed=0;
				if (earjack->remotekey_index != 0) 
					input_report_key(earjack->ipdev, remotekey_type[earjack->remotekey_index].key_index, earjack->remotekey_pressed);
				dbg("remote key: %s : %d->%d \n", remotekey_type[earjack->remotekey_index].key_name, !earjack->remotekey_pressed, earjack->remotekey_pressed);
			}

		}
		// else, ignore key
		else {
			dbg("igrnore key.\n");
			enable_irq_remote();
			remotekey_allow_suspend();
			dbg_func_out();
			return ;
		}	
	}
	// if current state : key pressed
	else
	{ 
#ifdef EARJACK_POLL_MODE_FEATURE		
		if(earjack_fet_success == 0 && (REMOTEKEY_PRESSED) ){			
				schedule_delayed_work(&remotekey_work, EARJACK_REMOTE_KEY_EVENT_POLLING_TIME);
				return;										
		}
#endif
		earjack->remotekey_pressed=0;
		if (earjack->remotekey_index != 0) 
			input_report_key(earjack->ipdev, remotekey_type[earjack->remotekey_index].key_index, earjack->remotekey_pressed);
		dbg("remote key: %s : %d->%d \n", remotekey_type[earjack->remotekey_index].key_name, !earjack->remotekey_pressed, earjack->remotekey_pressed);
	}        
#ifdef EARJACK_POLL_MODE_FEATURE								
	if((earjack_fet_success == 0) && (earjack->remotekey_pressed == 1)){
		input_sync(earjack->ipdev);	
		schedule_delayed_work(&remotekey_work, EARJACK_REMOTE_KEY_EVENT_POLLING_TIME);
		return;
	}
#endif
	input_sync(earjack->ipdev);
	enable_irq_remote();
	remotekey_allow_suspend();	
	dbg_func_out();
	return;
}

// Remove Driver
static int __devexit pantech_earjack_remove(struct platform_device *pdev)
{
	struct pantech_earjack *earjack = platform_get_drvdata(pdev);

	dbg_func_in();
	input_unregister_device(earjack->ipdev);
	switch_dev_unregister(&earjack->sdev);
	kfree(earjack);
	wake_lock_destroy(&earjack_wake_lock);
	wake_lock_destroy(&remotekey_wake_lock);
	dbg_func_out();
	return 0;
}

// Suspend, Resume
static int pantech_earjack_suspend(struct platform_device * pdev, pm_message_t state)
{

#ifdef EARJACK_POLL_MODE_FEATURE	
   	int err;
	dbg_func_in();
	if(earjack_states==EARJACK_FET_HIGH &&earjack_fet_success == 0) {
		cancel_delayed_work_sync(&earjack_work);
		cancel_delayed_work_sync(&remotekey_work);
		
		if(earjack->mic_on != 1){
			err=regulator_is_enabled(hs_jack_l8);
			if(err>0){
				regulator_disable(hs_jack_l8);
			}
		}	
	}	
#endif
	return 0;
}
static int pantech_earjack_resume(struct platform_device * pdev)
{
#ifdef EARJACK_POLL_MODE_FEATURE											
	int err;
	dbg_func_in();
	if(earjack_states==EARJACK_FET_HIGH &&earjack_fet_success == 0) {
		
		err = regulator_is_enabled(hs_jack_l8); 
		if(err<=0) {
			regulator_enable(hs_jack_l8);
		}
		
		schedule_delayed_work(&earjack_work,10);		
	}	
#endif
	return 0;
}

// Driver Structure
static struct platform_driver pantech_earjack_driver = {
	.probe	= pantech_earjack_probe,
	.remove	= pantech_earjack_remove,
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
	},
	.resume = pantech_earjack_resume,
	.suspend = pantech_earjack_suspend,
};

// Module Init
static int __init pantech_earjack_init(void)
{
	printk("[earjack] pantech_earjack module init.\n");
	return platform_driver_register(&pantech_earjack_driver);
}
late_initcall(pantech_earjack_init);

// Module Exit 
static void __exit pantech_earjack_exit(void)
{
	printk("[earjack] pantech_earjack module exit.\n");
	platform_driver_unregister(&pantech_earjack_driver);
}
module_exit(pantech_earjack_exit);

MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:pantech_earjack");

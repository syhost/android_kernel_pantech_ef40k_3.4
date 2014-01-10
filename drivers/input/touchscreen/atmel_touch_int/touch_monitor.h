/* 
 * Touch Monitor Interface 
 * Ver 0.1
 */

#include "touch_log.h"
#include "../touch_ioctl.h"

static int monitor_open(struct inode *inode, struct file *file);
static ssize_t monitor_read(struct file *file, char *buf, size_t count, loff_t *ppos);
static ssize_t monitor_write(struct file *file, const char *buf, size_t count, loff_t *ppos);
static int monitor_release(struct inode *inode, struct file *file);
static long monitor_ioctl(struct file *file, unsigned int cmd, unsigned long arg);

static void set_touch_config(int data, int object_type, int field_index);
static int get_touch_config(int object_type, int field_index);
static void apply_touch_config(void);
static void reset_touch_config(void);
static int ioctl_debug(unsigned long arg);
static uint8_t qt_charger_mode_config(unsigned long mode);
static int ioctl_diag_debug(unsigned long arg);
static int send_reference_data(unsigned long arg);
uint8_t calibrate_chip(void);

/*
 * vendor_id : 
 * ateml(1) cypress(2)
 * model_id : 
 * ef39s(0390) ef40s(0400) ef40k (0401)
 * presto(9000) kelly(9100)
 * type : 
 * model manager would manage ito or color type.

 * return vendor_id*100*10000 + model_id*100 + type;
 */
static int vendor_id = 1;
static int model_id = 400;
static int type_id = 0;

static struct file_operations monitor_fops = 
{
	.owner =    THIS_MODULE,
	.unlocked_ioctl =    monitor_ioctl,
	.read =     monitor_read,
	.write =    monitor_write,
	.open =     monitor_open,
	.release =  monitor_release
};

static struct miscdevice touch_monitor = 
{
	.minor =    MISC_DYNAMIC_MINOR,
	.name =     "touch_monitor",
	.fops =     &monitor_fops
};

typedef struct
{
	int touch_count; 

} touch_monitor_info_t;
//static touch_monitor_info_t *touch_monitor_info; 

static long monitor_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int data, object_type, field_index;
	// Read Command 
	// Write, Etc.
	switch (cmd)
	{
		case GET_TOUCH_ID:
			return vendor_id*100*10000 + model_id*100 + type_id;
			break;
		case SET_TOUCH_CONFIG:
			data		= (int)((arg & 0xFFFF0000) >> 16);
			object_type 	= (int)((arg & 0x0000FF00) >> 8);
			field_index 	= (int)((arg & 0x000000FF) >> 0);

			set_touch_config(data, object_type, field_index);
			break;
		case GET_TOUCH_CONFIG:
			object_type 	= (int)((arg & 0x0000FF00) >> 8);
			field_index 	= (int)((arg & 0x000000FF) >> 0);
			return get_touch_config(object_type, field_index);
			break;
		case APPLY_TOUCH_CONFIG:
			apply_touch_config();
			break;
		case RESET_TOUCH_CONFIG:
			reset_touch_config();
			break;
#ifdef ITO_TYPE_CHECK 
			// TO BE DEPRECIATED
		case READ_ITO_TYPE:
			return 0; 
			break;
#endif
		case TOUCH_IOCTL_DEBUG:
			return ioctl_debug(arg);
			break;
		case TOUCH_IOCTL_CHARGER_MODE:
			qt_charger_mode_config(arg);
			break;
		case DIAG_DEBUG:
			return ioctl_diag_debug(arg);
			break;
		case GET_REFERENCE_DATA:
			return send_reference_data(arg);
			break;
		case CALIBRATE:
			return calibrate_chip();
			break;

		default:
			return 0;
			break;
	}
	return 0;
}




    //todo



// call in driver init function
void touch_monitor_init(void) {
	int rc;
	rc = misc_register(&touch_monitor);
	if (rc) {
		pr_err("::::::::: can''t register touch_monitor\n");
	}
	init_proc();
}

// call in driver remove function
void touch_monitor_exit(void) {
	misc_deregister(&touch_monitor);
	remove_proc();
}

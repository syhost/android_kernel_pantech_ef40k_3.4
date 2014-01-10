#ifndef PANTECH_APANIC_H
#define PANTECH_APANIC_H

/* ==========================================================================*/
/* boot_smem_alloc_logging_system (Note : SMEM buffer size is 512bytes) --[[ */
/* ==========================================================================*/
#define P_INFO_BUFFER_SIZE 0x80
#define PINFO_START_MAGIC 0xDEAD0525
#define PINFO_END_MAGIC 0x5250DAED
#define DLOAD_SET_MAGIC_TEXT 0x95857493
#define DLOAD_SET_MAGIC_USBDUMP 0x95857494
#define DLOAD_SET_MAGIC_EMMCDUMP 0x95857495
	
struct pantech_log_header{
	unsigned int magic;
	unsigned int version;
	unsigned int *klog_buf_address;
	unsigned int *klog_end_idx;
	unsigned int *logcat_buf_address;
	unsigned int *logcat_w_off;
	unsigned int logcat_size;
};

struct phone_info_log{
	unsigned int start_phone_info_log;
	char information[P_INFO_BUFFER_SIZE];
	unsigned int end_phone_info_log;
};

//MAIN STRUCTER
struct pantech_log_system_s
{
	struct pantech_log_header crash_info;
	struct phone_info_log phone_info;
	unsigned int dload_sbl3_set_magic;
};
/* ==========================================================================*/
/* ]]-- boot_smem_alloc_logging_system */
/* ==========================================================================*/


#endif


/* drivers/misc/pantech_apanic.c
 *
 * Copyright (C) 2011 PANTECH, Co. Ltd.
 * based on drivers/misc/apanic.c
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.      See the
 * GNU General Public License for more details.
 *
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/wakelock.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/notifier.h>
#include <linux/debugfs.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/preempt.h>
#include <asm/cacheflush.h>
#include <asm/system.h>
#include <linux/fb.h>
#include <linux/time.h>
#include <linux/io.h>
#include "smd_private.h"
#include "modem_notifier.h"
#include <linux/nmi.h>
#include <mach/msm_iomap.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <mach/pantech_apanic.h>

#define PANIC_MAGIC    0xDAEDDAED
#define PHDR_VERSION   0x01

struct pantech_log_system_s *pantech_log_sys = NULL;

extern struct pantech_log_header *get_pantech_klog_dump_address(void);
extern struct pantech_log_header *get_pantech_logcat_dump_address(void);

static int apanic_blk(struct notifier_block *this, unsigned long event, void *ptr)
{
      if(pantech_log_sys != NULL)
      {
          pantech_log_sys->crash_info.magic = PANIC_MAGIC;
          pantech_log_sys->crash_info.version = PHDR_VERSION;
      }
      
      flush_cache_all();

      return NOTIFY_DONE;
}

/*****************************************************
 * PNANTEH APANIC MODULE INIT
 * **************************************************/
static struct notifier_block panic_blk = {
      .notifier_call    = apanic_blk,
};

static int sbl3DloadMagic = 0;
static ssize_t sbl3DloadMagic_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int size;
	
	if (!pantech_log_sys)
		pantech_log_sys = (struct pantech_log_system_s *)smem_get_entry(SMEM_ID_VENDOR2, &size);	
	
	if(pantech_log_sys->dload_sbl3_set_magic == DLOAD_SET_MAGIC_TEXT)
		sbl3DloadMagic = 1;
	else if(pantech_log_sys->dload_sbl3_set_magic == DLOAD_SET_MAGIC_USBDUMP)
		sbl3DloadMagic = 2;
	else if(pantech_log_sys->dload_sbl3_set_magic == DLOAD_SET_MAGIC_EMMCDUMP)
		sbl3DloadMagic = 3;
	else
		sbl3DloadMagic = 0;
		
    return sprintf(buf, "%d\n", sbl3DloadMagic);
}


static struct kobj_attribute sbl3DloadMagic_attribute =
        __ATTR(sbl3DloadMagic, 0664, sbl3DloadMagic_show, NULL);

static struct attribute *attrs[] = {
    &sbl3DloadMagic_attribute.attr,
    NULL,
};

static struct attribute_group attr_group = {
    .attrs = attrs,
};

static struct kobject *pantech_apanic_kobj;

int __init pantech_apanic_init(void)
{
      unsigned size;
      struct pantech_log_header *klog_header, *logcat_log_header;
      int retval;

      pantech_apanic_kobj = kobject_create_and_add("pantech_apanic", kernel_kobj);
	  if (!pantech_apanic_kobj)
	  	return -ENOMEM;
	  retval = sysfs_create_group(pantech_apanic_kobj, &attr_group);
	  if (retval)
	  	kobject_put(pantech_apanic_kobj);

      klog_header = get_pantech_klog_dump_address();
      logcat_log_header = get_pantech_logcat_dump_address();

	  if (!pantech_log_sys)
      	pantech_log_sys = (struct pantech_log_system_s *)smem_get_entry(SMEM_ID_VENDOR2, &size);	  

      if(!pantech_log_sys){
          printk(KERN_ERR "pantech_apanic: no available crash buffer , initial failed\n");
          return 0;
      }
      else
      {
          atomic_notifier_chain_register(&panic_notifier_list, &panic_blk);
      }

      pantech_log_sys->crash_info.magic=0;
      pantech_log_sys->crash_info.version=0;
      pantech_log_sys->crash_info.klog_buf_address = klog_header->klog_buf_address;
      pantech_log_sys->crash_info.klog_end_idx = klog_header->klog_end_idx;

      pantech_log_sys->crash_info.logcat_buf_address = logcat_log_header->logcat_buf_address;
      pantech_log_sys->crash_info.logcat_w_off = logcat_log_header->logcat_w_off;
      pantech_log_sys->crash_info.logcat_size = logcat_log_header->logcat_size;

	  printk("pantech_apanic : pantech_log_header initialized success for write to SMEM\n");

      return 0;
}



module_init(pantech_apanic_init);

MODULE_AUTHOR("Pantech ls4 part1>");
MODULE_DESCRIPTION("Pantech errlogging driver");
MODULE_LICENSE("GPL v2");

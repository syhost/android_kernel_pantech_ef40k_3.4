#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/syscalls.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/fcntl.h>
#include <linux/unistd.h>
#include <linux/wakelock.h>
#include <linux/sky_pm_tools.h>

#define DEBUG_ME
#define DEBUG_ME_REL

#ifdef DEBUG_ME
#define dbgme(fmt, args...)   printk("[SKYPMTOOLS]L:%d(%s)" fmt, __LINE__, __func__, ##args)
#else
#define dbgme(fmt, args...)
#endif

#ifdef DEBUG_ME_REL
#define dbgme_rel(fmt, args...)   printk("[SKYPMTOOLS]L:%d(%s)" fmt, __LINE__, __func__, ##args)
#else
#define dbgme_rel(fmt, args...)
#endif

/************************
  Sky Slow buffer function 
*************************/

int sky_slow_buff_init(struct sky_slow_buff * sb, int buf_size, int immediate)
{
	sb->buff = kzalloc(sizeof(int)*buf_size, GFP_KERNEL);
	if (!sb->buff)
		return -2;
	
	sb->init_finish = 0;
	sb->buff_size = buf_size;
	sb->now_sum = 0;
	sb->pos = 0;
	sb->immediate = immediate;

	return 0;
}

void sky_slow_buff_reset(struct sky_slow_buff * sb)
{
	sb->init_finish = 0;
	sb->now_sum = 0;
	sb->pos = 0;
}

int sky_slow_buff_is_init_finish(struct sky_slow_buff * sb)
{
	return sb->init_finish;
}

int sky_slow_buff_add(struct sky_slow_buff * sb, int value)
{	
	int div_count;

	if (!sb->buff || sb->buff_size <= 0)
		return -2;		

	div_count = sb->buff_size;

	if (!sb->init_finish)
	{
		sb->buff[sb->pos] = value;
		sb->now_sum += value;
		sb->pos++;
		if (sb->pos >= sb->buff_size)
			sb->init_finish = 1;
		else if (sb->immediate)
			div_count = sb->pos;
		else
			return -1;
	}
	else
	{
		sb->now_sum -= sb->buff[sb->pos];
		sb->now_sum += value;
		sb->buff[sb->pos] = value;
		sb->pos++;
	}
	if (sb->pos >= sb->buff_size)
		sb->pos = 0;

	return (int)((sb->now_sum)/div_count);
}

void sky_slow_buff_free(struct sky_slow_buff * sb)
{
	kfree(sb->buff);
}

int sky_adc_scaler(
	int src_value,
	int src_min, int src_max, 
	int tgt_min, int tgt_max)
{
	int src_range = src_max - src_min;
	int trt_range = tgt_max - tgt_min;
	int src_off = ((src_value<src_min)?src_min:(src_value>src_max)?src_max:src_value) - src_min;

	return (int)(((src_off*trt_range)/src_range)+tgt_min);
}

/************************
  Sky Rescale function 
*************************/

int sky_rescale_get_b_value(int a_val, struct sky_rescale_data_list *list)
{
	int i;
	int d = 0;

	if (list->data_list[0].a_min > a_val)
	{
		d = (int)list->data_list[0].b_min;
		list->cur_index = 0;
		dbgme("get b_val=%d(list_name:%s,index:%d,a_val:%d)\n", d, list->name, list->cur_index, a_val);
		return d;
	}
	else if (list->data_list[list->list_size-1].a_max < a_val)
	{
		d = (int)list->data_list[list->list_size-1].b_max;
		list->cur_index = list->list_size-1;
		dbgme("get b_val=%d(list_name:%s,index:%d,a_val:%d)\n", d, list->name, list->cur_index, a_val);
		return d;
	}
	else 
	{
		i = list->cur_index;
		do{
			if (list->data_list[i].a_min <= a_val && 
				list->data_list[i].a_max >= a_val)
			{
				d = (int)sky_adc_scaler((int)a_val, 
																	list->data_list[i].a_min, 
																	list->data_list[i].a_max, 
																	list->data_list[i].b_min, 
																	list->data_list[i].b_max);
				list->cur_index = i;
				dbgme("get b_val=%d(list_name:%s,index:%d,a_val:%d)\n", d, list->name, list->cur_index, a_val);
				return d;
			}
			i++;
			if (i >= list->list_size) i = 0;
		}while(i != list->cur_index);
	}	
	dbgme_rel("get b_val=unknown(list_name:%s,a_val:%d)\n", list->name, a_val);

	return -1;	
}

int sky_rescale_get_a_value(int b_val, struct sky_rescale_data_list *list)
{
	int i;
	int d = 0;

	if (list->data_list[0].b_min > b_val)
	{
		d = (int)list->data_list[0].a_min;
		list->cur_index = 0;
		dbgme("get a_val=%d(list_name:%s,index:%d,b_val:%d)\n", d, list->name, list->cur_index, b_val);
		return d;
	}
	else if (list->data_list[list->list_size-1].b_max < b_val)
	{
		d = (int)list->data_list[list->list_size-1].a_max;
		list->cur_index = list->list_size-1;
		dbgme("get a_val=%d(list_name:%s,index:%d,b_val:%d)\n", d, list->name, list->cur_index, b_val);
		return d;
	}
	else 
	{
		i = list->cur_index;
		do{
			if (list->data_list[i].b_min <= b_val && 
				list->data_list[i].b_max >= b_val)
			{
				d = (int)sky_adc_scaler((int)b_val, 
																	list->data_list[i].b_min, 
																	list->data_list[i].b_max, 
																	list->data_list[i].a_min, 
																	list->data_list[i].a_max);
				list->cur_index = i;
				dbgme("get a_val=%d(list_name:%s,index:%d,b_val:%d)\n", d, list->name, list->cur_index, b_val);
				return d;
			}
			i++;
			if (i >= list->list_size) i = 0;
		}while(i != list->cur_index);
	}	
	dbgme_rel("get a_val=unknown(list_name:%s,b_val:%d)\n", list->name, b_val);

	return -1;	
}

int sky_rescale_get_cur_index(struct sky_rescale_data_list *list)
{
	return list->cur_index;
}

int sky_rescale_get_list_size(struct sky_rescale_data_list *list)
{
	return list->list_size;
}

char * sky_rescale_get_list_name(struct sky_rescale_data_list *list)
{
	return list->name;
}


/************************
  Sky range function 
*************************/

int sky_range_set_value(int val, struct sky_range_data_list *list)
{
	int i;

	i = list->cur_index;
	do{
		if (list->data_list[i].min <= val && 
			list->data_list[i].max >= val)
		{
			list->cur_index = i;
			dbgme("set value=%d(list_name:%s,range_name:%s,index:%d)\n", val, list->name, list->data_list[i].name, i);
			return i;
		}
		i++;
		if (i >= list->list_size) i = 0;
	}while(i != list->cur_index);

			dbgme("set value=%d(list_name:%s,range_name:unknown,index:unknown)\n", val, list->name);

	return -1;	
}

int sky_range_get_index(struct sky_range_data_list *list)
{
	return list->cur_index;
}

void * sky_range_get_data(struct sky_range_data_list *list)
{
	return list->data_list[list->cur_index].data;
}

char * sky_range_get_list_name(struct sky_range_data_list *list)
{
	return list->name;
}

/************************
  Sky event queue function 
*************************/

static int sky_dequeue_event(struct sky_event_queue * q, struct sky_event  **event)
{
	unsigned long flags;

	dbgme("Enter\n");

	spin_lock_irqsave(&q->queue_lock, flags);
	if (q->queue_count == 0) {
		spin_unlock_irqrestore(&q->queue_lock, flags);
		return -EINVAL;
	}
	*event = &q->queue[q->head];
	q->head = (q->head + 1) % q->max_events;
	pr_debug("%s dequeueing %d\n", __func__, (*event)->event);
	q->queue_count--;
	spin_unlock_irqrestore(&q->queue_lock, flags);

	dbgme("Exit\n");

	return 0;
}

static int sky_enqueue_event(struct sky_event_queue * q, int event, void *data)
{
	unsigned long flags;

	dbgme("Enter\n");

	spin_lock_irqsave(&q->queue_lock, flags);
	if (q->queue_count == q->max_events) {
		spin_unlock_irqrestore(&q->queue_lock, flags);
		pr_err("%s: queue full cannot enqueue %d\n",
				__func__, event);
		return -EAGAIN;
	}
	pr_debug("%s queueing %d\n", __func__, event);
	q->queue[q->tail].event = event;
	q->queue[q->tail].data = data;
	q->tail = (q->tail + 1)%q->max_events;
	q->queue_count++;
	spin_unlock_irqrestore(&q->queue_lock, flags);

	dbgme("Exit\n");

	return 0;
}

static void sky_process_events(struct work_struct *work)
{
	struct sky_event  *event;
	int rc;
	struct sky_event_queue * q = container_of(work, struct sky_event_queue, queue_work);

	do {
		rc = sky_dequeue_event(q, &event);
		if (!rc)
			if (q->event_callback)
				q->event_callback(event->event, event->data);
	} while (!rc);
}

int sky_add_event(struct sky_event_queue * q, int event, void  *data)
{
	sky_enqueue_event(q, event, data);
	queue_work(q->event_wq_thread, &q->queue_work);
	return 0;
}

int sky_clear_event(struct sky_event_queue * q)
{
	q->tail = 0;
	q->head = 0;
	q->queue_count = 0;
	return 0;
}

struct sky_event_queue * sky_create_event_queue(int max_events, void (*event_callback)(int event, void *data))
{
	struct sky_event_queue *q = kmalloc(sizeof(struct sky_event_queue), GFP_KERNEL);

	if (!q) {
		goto out3;
	}

	q->queue = kzalloc(sizeof(struct sky_event) * max_events, GFP_KERNEL);
	if (!q->queue) {
		goto out2;
	}
	q->tail = 0;
	q->head = 0;
	spin_lock_init(&q->queue_lock);
	q->queue_count = 0;
	INIT_WORK(&q->queue_work, sky_process_events);
	q->event_wq_thread = create_workqueue("sky_eventd");
	if (!q->event_wq_thread) {
		goto out1;
	}
	
	q->max_events = max_events;
	q->event_callback = event_callback;

	return q;
out1:
	kfree(q->queue);
out2:
	kfree(q);
out3:
	return NULL;
}

void sky_delete_event_queue(struct sky_event_queue *q)
{
	if (q && q->queue){
		kfree(q->queue);
		kfree(q);
	}
}


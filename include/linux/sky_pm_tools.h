/************************
  Sky bit function 
*************************/
#define sky_set_bit(v,a) ((v)|=((0x1<<(a))&0xff))
#define sky_clear_bit(v,a) ((v)&=(~(0x1<<(a))&0xff))
#define sky_is_set_bit(v,a) ((v)&((0x1<<(a))&0xff))

/************************
  Sky filter function 
*************************/
#define sky_band_pass_filter(val, min, max) (((val) >= (min) && (val) <= (max))?(val):((val) < (min))?(min):(max))


/************************
  Sky Slow buffer function 
*************************/

struct sky_slow_buff {
	int * buff;
	int buff_size;
	int pos;
	int now_sum;
	int init_finish;
	int immediate;
};

extern int sky_slow_buff_init(struct sky_slow_buff * sb, int buf_size, int immediate);
extern void sky_slow_buff_reset(struct sky_slow_buff * sb);
extern int sky_slow_buff_is_init_finish(struct sky_slow_buff * sb);
extern int sky_slow_buff_add(struct sky_slow_buff * sb, int value);
extern void sky_slow_buff_free(struct sky_slow_buff * sb);
extern int sky_adc_scaler(
	int src_value,
	int src_min, int src_max, 
	int tgt_min, int tgt_max);


/************************
  Sky Rescale function 
*************************/

struct sky_rescale_data {
	int a_min;
	int a_max;
	int b_min;
	int b_max;
};

struct sky_rescale_data_list {
	char name[20];
	struct sky_rescale_data * data_list;
	int list_size;
	int cur_index;	
};

#define SKY_RESCALE_DATA(_id, _a_min, _a_max, _b_min, _b_max)	\
	[_id]={\
	.a_min = _a_min,\
	.a_max = _a_max,\
	.b_min = _b_min,\
	.b_max = _b_max,\
	}


#define SKY_RESCALE_LIST_INIT(_name, _data) \
{\
	.name = _name,\
	.data_list = _data,\
	.list_size = ARRAY_SIZE(_data),\
	.cur_index = 0,\
}

extern int sky_rescale_get_b_value(int a_val, struct sky_rescale_data_list *list);
extern int sky_rescale_get_a_value(int b_val, struct sky_rescale_data_list *list);
extern int sky_rescale_get_cur_index(struct sky_rescale_data_list *list);
extern int sky_rescale_get_list_size(struct sky_rescale_data_list *list);
extern char * sky_rescale_get_list_name(struct sky_rescale_data_list *list);


/************************
  Sky Range function 
*************************/

struct sky_range_data {
	char name[20];
	int min;
	int max;
	void *data;
};

struct sky_range_data_list {
	char name[20];
	struct sky_range_data * data_list;
	int list_size;
	int cur_index;	
};

#define SKY_RANGE_DATA(_id, _name, _min, _max, _data)	\
	[_id]={\
	.name = _name,\
	.min = _min,\
	.max = _max,\
	.data = _data,\
	}

#define SKY_RANGE_LIST_INIT(_name, _data) \
{\
	.name = _name,\
	.data_list = _data,\
	.list_size = ARRAY_SIZE(_data),\
	.cur_index = 0,\
}

extern int sky_range_set_value(int val, struct sky_range_data_list *list);
extern int sky_range_get_index(struct sky_range_data_list *list);
extern void * sky_range_get_data(struct sky_range_data_list *list);
extern char * sky_range_get_list_name(struct sky_range_data_list *list);

/************************
  Sky event queue function 
*************************/

struct sky_event {
	int event;
	void *data;
};

struct sky_event_queue {
	struct sky_event  *queue;

	int max_events;
	
	int tail;
	int head;
	spinlock_t queue_lock;
	int queue_count;
	struct work_struct queue_work;
	struct workqueue_struct *event_wq_thread;	

	void (*event_callback)(int event, void *data);
};

extern int sky_add_event(struct sky_event_queue * q, int event, void  *data);
extern int sky_clear_event(struct sky_event_queue * q);
extern struct sky_event_queue * sky_create_event_queue(int max_events, void (*event_callback)(int event, void *data));
extern void sky_delete_event_queue(struct sky_event_queue *q);



#define SKY_ATTR(_name)					\
{									\
	.attr = { .name = #_name, .mode = 0644 },					\
	.show = sky_attr_##_name##_show,				\
	.store = sky_attr_##_name##_store,				\
}

#define SKY_ATTR_SHOW_FUNC(_name) \
static ssize_t sky_attr_##_name##_show(struct device *dev,\
		struct device_attribute *attr, char *buf)

#define SKY_ATTR_STORE_FUNC(_name) \
static ssize_t sky_attr_##_name##_store(struct device *dev,\
		struct device_attribute *attr,\
		const char *buf, size_t size)

static void init_sky_attr(struct device * dev, struct device_attribute * attrs, int count)
{
	struct device_attribute *attr;
	int i;
	int r;

	for (i = 0; i < count; i++)
	{
		attr = &attrs[i];
		r = device_create_file(dev, attr);
		if (r)
			dev_err(dev, "init_sky_attr() failed to create sysfs file (%d)\n", r);
	}
}


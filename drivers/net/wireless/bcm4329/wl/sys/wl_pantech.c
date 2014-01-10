/*
 * Linux Wireless Extensions support
 *
 * Copyright (C) 1999-2010, Pantech Corporation

 */

#include <wl_pantech.h>
#include <linux/netdevice.h>

#ifdef CONFIG_SKY_WLAN_BCM4329

static struct device *cfg80211_parent_dev = NULL;

struct device *wl_cfg80211_get_parent_dev(void)
{
	return cfg80211_parent_dev;
}

void wl_cfg80211_set_parent_dev(void *dev)
{
	cfg80211_parent_dev = dev;
}

void wl_cfg80211_clear_parent_dev(void)
{
	cfg80211_parent_dev = NULL;
}
#endif

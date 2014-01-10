/*
 * Linux Wireless Extensions support
 *
 * Copyright (C) 1999-2010, Pantech Corporation

 */

#ifdef CONFIG_SKY_WLAN_BCM4329
void wl_cfg80211_set_parent_dev(void *dev);
struct device *wl_cfg80211_get_parent_dev(void);
void wl_cfg80211_clear_parent_dev(void);
#endif

GCC_VERSION      := $(shell $(CONFIG_SHELL) $(PWD)/scripts/gcc-version.sh $(CROSS_COMPILE)gcc)
EXTRA_CFLAGS += -Idrivers/media/video/msm
EXTRA_CFLAGS += -Idrivers/media/video/msm/io
EXTRA_CFLAGS += -Idrivers/media/video/msm/eeprom
EXTRA_CFLAGS += -Idrivers/media/video/msm/csi
obj-$(CONFIG_MSM_CAMERA_SENSOR) += msm_sensor.o
#obj-$(CONFIG_OV5647) += ov5647_v4l2.o
#obj-$(CONFIG_OV8825) += ov8825_v4l2.o
#obj-$(CONFIG_IMX074) += imx074_v4l2.o
#obj-$(CONFIG_S5K3L1YX) += s5k3l1yx.o
#obj-$(CONFIG_IMX091) += imx091.o
#obj-$(CONFIG_OV2720) += ov2720.o
#obj-$(CONFIG_MT9M114) += mt9m114_v4l2.o
#obj-$(CONFIG_S5K4E1) += s5k4e1_v4l2.o
#obj-$(CONFIG_MT9E013) += mt9e013_v4l2.o
#obj-$(CONFIG_WEBCAM_OV9726) += ov9726_v4l2.o
#obj-$(CONFIG_OV7692) += ov7692_v4l2.o
#obj-$(CONFIG_VX6953) += vx6953.o
ifeq ($(CONFIG_MACH_MSM8X60_EF39S),y)
 obj-$(CONFIG_PANTECH_CAMERA_CE1612) += ce1612_v4l2.o
#+= ce1612_reg_v4l2.o
 obj-$(CONFIG_PANTECH_CAMERA_S5K6AAFX13) += s5k6aafx13_r2_v4l2.o s5k6aafx13_r2_v4l2_cfg.o
endif
#mt9d113 camera module needs 2 tunning values, for SKT and none-SKT.
ifeq ($(CONFIG_MACH_MSM8X60_EF40S),y)
 obj-$(CONFIG_PANTECH_CAMERA_CE1612) += ce1612_v4l2.o
#+= ce1612_reg_v4l2.o
 obj-$(CONFIG_PANTECH_CAMERA_MT9D113) += mt9d113_r2_v4l2.o mt9d113_cfg_40s_v4l2.o
endif
ifeq ($(CONFIG_MACH_MSM8X60_EF40K),y)
 obj-$(CONFIG_PANTECH_CAMERA_CE1612) += ce1612_v4l2.o
#+= ce1612_reg_v4l2.o
 obj-$(CONFIG_PANTECH_CAMERA_MT9D113) += mt9d113_r2_v4l2.o mt9d113_cfg_40k_v4l2.o
endif
ifeq ($(CONFIG_MACH_MSM8X60_EF65L),y)
 obj-$(CONFIG_PANTECH_CAMERA_CE1612) += ce1612_v4l2.o
#+= ce1612_reg_v4l2.o
 obj-$(CONFIG_PANTECH_CAMERA_MT9D113) += mt9d113_r2_v4l2.o mt9d113_cfg_65l_v4l2.o
endif
# obj-$(CONFIG_ICP_HD) += icp_hd.o icp_hd_reg.o camsensor_icp_hd_tuneup.o camsensor_parser.o
ifeq ($(CONFIG_MACH_MSM8X60_EF33S),y)
 obj-$(CONFIG_PANTECH_CAMERA_ICP_HD) += icp_hd_r1_v4l2.o icp_hd_r1_v4l2_reg.o
 obj-$(CONFIG_PANTECH_CAMERA_S5K6AAFX13) += s5k6aafx13_r1_v4l2.o s5k6aafx13_r1_v4l2_reg.o
endif
ifeq ($(CONFIG_MACH_MSM8X60_EF34K),y)
 obj-$(CONFIG_PANTECH_CAMERA_ICP_HD) += icp_hd_r1_v4l2.o icp_hd_r1_v4l2_reg.o
 obj-$(CONFIG_PANTECH_CAMERA_S5K6AAFX13) += s5k6aafx13_r1_v4l2.o s5k6aafx13_r1_v4l2_reg.o
endif
ifeq ($(CONFIG_MACH_MSM8X60_EF35L),y)
 obj-$(CONFIG_PANTECH_CAMERA_ICP_HD) += icp_hd_r1_v4l2.o icp_hd_r1_v4l2_reg.o
 obj-$(CONFIG_PANTECH_CAMERA_S5K6AAFX13) += s5k6aafx13_r1_v4l2.o s5k6aafx13_r1_v4l2_reg.o
endif
obj-$(CONFIG_PANTECH_CAMERA) += sensor_i2c.o sensor_ctrl.o
obj-$(CONFIG_PANTECH_CAMERA_TUNER) += ptune_parser.o
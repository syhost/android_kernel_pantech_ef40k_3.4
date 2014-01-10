/* Copyright (c) 2011-2012, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include "msm_sensor.h"
#include "sensor_ctrl.h" 
#include "sensor_i2c.h" 


#if 1//test
#include <linux/regulator/machine.h> 
#include "sensor_ctrl.h"

#include <media/v4l2-subdev.h>
#include "msm.h"
#include "msm_ispif.h"
#include "msm_camera_i2c.h"
#if 1 // jjhwang File 
#include <linux/syscalls.h>
#endif

#include <linux/gpio.h>
#endif

#include "s5k6aafx13_r1_v4l2.h"
//#include "s5k6aafx13_r2_v4l2_cfg.h"

#define SENSOR_NAME "s5k6aafx13"
#define PLATFORM_DRIVER_NAME "msm_camera_s5k6aafx13"
#define s5k6aafx13_obj s5k6aafx13_##obj

#define F_S5K6AAFX13_POWER
#define F_ICP_HD_STANDBY

/* Micron S5K6AAFX13 Registers and their values */
#define SENSOR_DEBUG 0

#define ON  1
#define OFF 0

#if 1
#undef CDBG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#endif 
#if defined(CONFIG_MACH_MSM8X60_EF33S) || defined(CONFIG_MACH_MSM8X60_EF34K) || defined(CONFIG_MACH_MSM8X60_EF35L)
/* icp_hd + S5K6AAFX13 */

#ifdef F_ICP_HD_STANDBY//BOARD_VER_G(WS10)	
#define ICP_SLAVE_ADDRESS 0x7A
//#else
//#define ICP_SLAVE_ADDRESS 0x78>>1
#endif

#define C_PANTECH_CAMERA_MIN_BRIGHTNESS 0	/* -4 */
#define C_PANTECH_CAMERA_MAX_BRIGHTNESS 8	/* +4 */

#define s5k6aafx13_delay_msecs_stream 100//50//200//500
#define S5K6AAFX13_I2C_RETRY	10
#define S5K6AAFX13_I2C_MPERIOD	200

#define CAMIO_F_RST_N	0
#define CAMIO_F_STB_N	1
#define CAMIO_R_RST_N	2
#define CAMIO_R_STB_N	3
#define CAMIO_MAX	4

static sgpio_ctrl_t sgpios[CAMIO_MAX] = {
    {CAMIO_F_RST_N, "CAMIO_F_RST_N", 137},
    {CAMIO_F_STB_N, "CAMIO_F_STB_N", 139},
#ifdef F_ICP_HD_STANDBY	
	{CAMIO_R_RST_N, "CAMIO_R_RST_N", 106},
	{CAMIO_R_STB_N, "CAMIO_R_STB_N",  57},//46},//EF39
#endif	
};

static int current_fps = 31;
static int s5k6aafx13_sensor_set_preview_fps(struct msm_sensor_ctrl_t *s_ctrl ,int8_t preview_fps);

#if 0
#ifdef F_ICP_HD_STANDBY
static struct regulator *l2b_2p8v_8m;//8901_l2
static struct regulator *mvs0b_1p8v_8m;//8901_mvs0
static struct regulator *s2b_1p2v_8m;//8901_s2
static struct regulator *lvs3b_1p8v;//8901_lvs3
static struct regulator *l3b_2p8v_8m;//8901_l3
#endif

#ifdef F_S5K6AAFX13_POWER
static struct regulator *lvs1b_1p8v;//8901_lvs1
//static struct regulator *lvs2b_1p8v;
//static struct regulator *mvs0_1p8v;
static struct regulator *l1b_2p8v;//8901_l1
static struct regulator *l15a_1p5v;//8058_l15
#endif
#endif

#define CAM2V_IO_1P8V	0
#define CAM2V_CORE_1P5V	1
#define CAM2V_A_2P8V	2
#ifdef F_ICP_HD_STANDBY
#define CAM1V_IO_1P8V   3
#define CAM1V_L2B_2P8V  4
#define CAM1V_S2_1P2V   5
#define CAM1V_LVS3_1P8V 6
#define CAM1V_L3_2P8V   7
#define CAMV_MAX	8// 3
#else
#define CAMV_MAX	    3
#endif

static bool b_snapshot_flag;

static svreg_ctrl_t svregs[CAMV_MAX] = {
	{CAM2V_IO_1P8V,   "8901_lvs1", NULL, 0},//8901_mvs0//EF39 //1800 //I2C pull-up
	{CAM2V_CORE_1P5V, "8058_l15",   NULL, 1500},//8901_s2//EF39 //1500
	{CAM2V_A_2P8V,    "8901_l1",  NULL, 2800},//8058_l19//EF39
#ifdef F_ICP_HD_STANDBY
    {CAM1V_IO_1P8V,    "8901_mvs0",  NULL, 0}, //1800
    {CAM1V_L2B_2P8V,    "8901_l2",  NULL, 2800},
    {CAM1V_S2_1P2V,    "8901_s2",  NULL, 1300},
    {CAM1V_LVS3_1P8V,    "8901_lvs3",  NULL, 0}, //1800
    {CAM1V_L3_2P8V,    "8901_l3",  NULL, 2800},
#endif
};

#else
#error "unknown machine!"
#endif

#if 1
static struct msm_cam_clk_info cam_mclk_info[] = {
	{"cam_clk", MSM_SENSOR_MCLK_24HZ},
};
#endif

#if 1//needtocheck
//extern si2c_const_param_t s5k6aafx13_const_params[SI2C_PID_MAX];
//static si2c_param_t s5k6aafx13_params[SI2C_PID_MAX];

/* I2C slave address */
#define SI2C_SA	((s5k6aafx13_client->addr) >> 1)

#if 0
typedef struct {
	struct work_struct work;
} s5k6aafx13_work_t;

typedef struct {
	const struct msm_camera_sensor_info *sinfo;
} s5k6aafx13_ctrl_t;
#endif

//extern si2c_const_param_t s5k6aafx13_const_params[SI2C_PID_MAX];
//static si2c_param_t s5k6aafx13_params[SI2C_PID_MAX];

//static mt9d113_ctrl_t *s5k6aafx13_ctrl = NULL;
static struct i2c_client *s5k6aafx13_client = NULL;
//static mt9d113_work_t *s5k6aafx13_work = NULL;
#endif

DEFINE_MUTEX(s5k6aafx13_mut);
static struct msm_sensor_ctrl_t s5k6aafx13_s_ctrl;

static int8_t sensor_mode = -1;   			// 0: full size,  1: qtr size, 2: fullhd size, 3: ZSL


#if 0//def F_STREAM_ON_OFF	
static struct msm_camera_i2c_reg_conf s5k6aafx13_start_settings[] = {
	{0x0100, 0x01},
};

static struct msm_camera_i2c_reg_conf s5k6aafx13_stop_settings[] = {
	{0x0100, 0x00},
};
#endif

#if 0
static struct msm_camera_i2c_reg_conf s5k6aafx13_groupon_settings[] = {
	{0x104, 0x01},
};

static struct msm_camera_i2c_reg_conf s5k6aafx13_groupoff_settings[] = {
	{0x104, 0x00},
};
#endif

#if 0
static struct msm_camera_i2c_reg_conf s5k6aafx13_prev_settings[] = {
};

static struct msm_camera_i2c_reg_conf s5k6aafx13_snap_settings[] = {
};

static struct msm_camera_i2c_reg_conf s5k6aafx13_recommend_settings[] = {
};
#endif

static struct v4l2_subdev_info s5k6aafx13_subdev_info[] = {
	{
	.code   =V4L2_MBUS_FMT_YUYV8_2X8,// V4L2_MBUS_FMT_SBGGR10_1X10,//for isp
	.colorspace = V4L2_COLORSPACE_JPEG,
	.fmt    = 1,
	.order    = 0,
	},
	/* more can be supported, to be added later */
};

#if 0
static struct msm_camera_i2c_conf_array s5k6aafx13_init_conf[] = {
	{&s5k6aafx13_recommend_settings[0],
	ARRAY_SIZE(s5k6aafx13_recommend_settings), 0, MSM_CAMERA_I2C_WORD_DATA}
};

static struct msm_camera_i2c_conf_array s5k6aafx13_confs[] = {
	{&s5k6aafx13_snap_settings[0],
	ARRAY_SIZE(s5k6aafx13_snap_settings), 0, MSM_CAMERA_I2C_WORD_DATA},
	{&s5k6aafx13_prev_settings[0],
	ARRAY_SIZE(s5k6aafx13_prev_settings), 0, MSM_CAMERA_I2C_WORD_DATA},
	{&s5k6aafx13_prev_settings[0],
	ARRAY_SIZE(s5k6aafx13_prev_settings), 0, MSM_CAMERA_I2C_WORD_DATA},	
	{&s5k6aafx13_snap_settings[0],
	ARRAY_SIZE(s5k6aafx13_snap_settings), 0, MSM_CAMERA_I2C_WORD_DATA},	
};
#endif



#if 0
static struct msm_sensor_output_info_t s5k6aafx13_dimensions[] = {
	{
		.x_output = 0x1070,
		.y_output = 0xC30,
		.line_length_pclk = 0x1178,
		.frame_length_lines = 0xC90,
		.vt_pixel_clk = 182400000,
		.op_pixel_clk = 182400000,
		.binning_factor = 1,
	},
	{
		.x_output = 0x838,
		.y_output = 0x618,
		.line_length_pclk = 0x1178,
		.frame_length_lines = 0x634,
		.vt_pixel_clk = 216000000,
		.op_pixel_clk = 108000000,
		.binning_factor = 2,
	},
};

/* CAMIF output resolutions */
#define S5K6AAFX13_FULL_WIDTH                 1280
#define S5K6AAFX13_FULL_HEIGHT                960
#define S5K6AAFX13_QTR_WIDTH                  640
#define S5K6AAFX13_QTR_HEIGHT                 480

/* 816x612, 24MHz MCLK 96MHz PCLK */
uint32_t S5K6AAFX13_FULL_SIZE_DUMMY_PIXELS   = 0;
uint32_t S5K6AAFX13_FULL_SIZE_DUMMY_LINES    = 0;
uint32_t S5K6AAFX13_FULL_SIZE_WIDTH          = 1280;
uint32_t S5K6AAFX13_FULL_SIZE_HEIGHT         = 960;

uint32_t S5K6AAFX13_QTR_SIZE_DUMMY_PIXELS    = 0;
uint32_t S5K6AAFX13_QTR_SIZE_DUMMY_LINES     = 0;
uint32_t  S5K6AAFX13_QTR_SIZE_WIDTH          = 640;
uint32_t S5K6AAFX13_QTR_SIZE_HEIGHT          = 480;

uint32_t S5K6AAFX13_HRZ_FULL_BLK_PIXELS      = 16;
uint32_t S5K6AAFX13_VER_FULL_BLK_LINES       = 12;
uint32_t S5K6AAFX13_HRZ_QTR_BLK_PIXELS       = 16;
uint32_t S5K6AAFX13_VER_QTR_BLK_LINES        = 12;
#else

#define C_PANTECH_CAMERA_MIN_PREVIEW_FPS	5
#define C_PANTECH_CAMERA_MAX_PREVIEW_FPS	31

#define S5K6AAFX13_FULL_SIZE_DUMMY_PIXELS   0
#define S5K6AAFX13_FULL_SIZE_DUMMY_LINES    0
#define S5K6AAFX13_FULL_SIZE_WIDTH          1280//3264
#define S5K6AAFX13_FULL_SIZE_HEIGHT         960//2448

#define S5K6AAFX13_QTR_SIZE_DUMMY_PIXELS    0
#define S5K6AAFX13_QTR_SIZE_DUMMY_LINES     0
#define S5K6AAFX13_QTR_SIZE_WIDTH           640//1280
#define S5K6AAFX13_QTR_SIZE_HEIGHT          480//960

#define S5K6AAFX13_HRZ_FULL_BLK_PIXELS      16//0
#define S5K6AAFX13_VER_FULL_BLK_LINES       12//0

#define S5K6AAFX13_HRZ_QTR_BLK_PIXELS       16//0
#define S5K6AAFX13_VER_QTR_BLK_LINES        12//0

//#define S5K6AAFX13_1080P_SIZE_WIDTH        1920
//#define S5K6AAFX13_1080P_SIZE_HEIGHT        1088

//#define S5K6AAFX13_HRZ_1080P_BLK_PIXELS      0
//#define S5K6AAFX13_VER_1080P_BLK_LINES        0

//#define S5K6AAFX13_ZSL_SIZE_WIDTH    2560 
//#define S5K6AAFX13_ZSL_SIZE_HEIGHT   1920 


static struct msm_sensor_output_info_t s5k6aafx13_dimensions[] = {
	{
		.x_output = S5K6AAFX13_FULL_SIZE_WIDTH,
		.y_output = S5K6AAFX13_FULL_SIZE_HEIGHT,
		.line_length_pclk = S5K6AAFX13_FULL_SIZE_WIDTH + S5K6AAFX13_HRZ_FULL_BLK_PIXELS ,
		.frame_length_lines = S5K6AAFX13_FULL_SIZE_HEIGHT+ S5K6AAFX13_VER_FULL_BLK_LINES ,
		.vt_pixel_clk = 21811200,//35000000,//198000000, //207000000, //276824064, 
		.op_pixel_clk = 40000000,//35000000,//198000000, //207000000, //276824064, 
		.binning_factor = 1,
	},
	{
		.x_output = S5K6AAFX13_QTR_SIZE_WIDTH,
		.y_output = S5K6AAFX13_QTR_SIZE_HEIGHT,
		.line_length_pclk = S5K6AAFX13_QTR_SIZE_WIDTH + S5K6AAFX13_HRZ_QTR_BLK_PIXELS,
		.frame_length_lines = S5K6AAFX13_QTR_SIZE_HEIGHT+ S5K6AAFX13_VER_QTR_BLK_LINES,
		.vt_pixel_clk = 21811200,//35000000, //207000000, //276824064, 
		.op_pixel_clk = 40000000,//35000000, //207000000, //276824064, 
		.binning_factor = 1,//2,
	},
	{
		.x_output = S5K6AAFX13_QTR_SIZE_WIDTH,
		.y_output = S5K6AAFX13_QTR_SIZE_HEIGHT,
		.line_length_pclk = S5K6AAFX13_QTR_SIZE_WIDTH + S5K6AAFX13_HRZ_QTR_BLK_PIXELS,
		.frame_length_lines = S5K6AAFX13_QTR_SIZE_HEIGHT+ S5K6AAFX13_VER_QTR_BLK_LINES,
		.vt_pixel_clk = 21811200,//35000000, //207000000, //276824064, 
		.op_pixel_clk = 40000000,//35000000, //207000000, //276824064, 
		.binning_factor = 1,//2,
	},
	{
		.x_output = S5K6AAFX13_FULL_SIZE_WIDTH,
		.y_output = S5K6AAFX13_FULL_SIZE_HEIGHT,
		.line_length_pclk = S5K6AAFX13_FULL_SIZE_WIDTH + S5K6AAFX13_HRZ_FULL_BLK_PIXELS ,
		.frame_length_lines = S5K6AAFX13_FULL_SIZE_HEIGHT+ S5K6AAFX13_VER_FULL_BLK_LINES ,
		.vt_pixel_clk = 21811200,//35000000, //207000000, //276824064, 
		.op_pixel_clk = 40000000,//35000000, //207000000, //276824064, 
		.binning_factor = 1,
	},	
};

#endif
static struct msm_camera_csi_params s5k6aafx13_csic_params = {
	.data_format = CSI_8BIT,//CSI_10BIT,
	.lane_cnt    = 1,//4,
	.lane_assign = 0xe4,
	.dpcm_scheme = 0,
	.settle_cnt  = 0x14,
};

static struct msm_camera_csi_params *s5k6aafx13_csic_params_array[] = {
	&s5k6aafx13_csic_params,
	&s5k6aafx13_csic_params,
	&s5k6aafx13_csic_params,
	&s5k6aafx13_csic_params,	
};

static struct msm_camera_csid_vc_cfg s5k6aafx13_cid_cfg[] = {
	{0, CSI_YUV422_8, CSI_DECODE_8BIT},//CSI_RAW10, CSI_DECODE_10BIT},
	{1, CSI_EMBED_DATA, CSI_DECODE_8BIT},
	{2, CSI_RESERVED_DATA_0, CSI_DECODE_8BIT},
	{3, CSI_RESERVED_DATA_0, CSI_DECODE_8BIT},	
};

static struct msm_camera_csi2_params s5k6aafx13_csi_params = {
	.csid_params = {
		.lane_cnt = 1,//4,
		.lut_params = {
			.num_cid = ARRAY_SIZE(s5k6aafx13_cid_cfg),
			.vc_cfg = s5k6aafx13_cid_cfg,
		},
	},
	.csiphy_params = {
		.lane_cnt = 1,//4,
		.settle_cnt = 0x14,//0x1B,
	},
};

static struct msm_camera_csi2_params *s5k6aafx13_csi_params_array[] = {
	&s5k6aafx13_csi_params,
	&s5k6aafx13_csi_params,
	&s5k6aafx13_csi_params,
	&s5k6aafx13_csi_params,	
};

#if 0 //for build
static struct msm_sensor_output_reg_addr_t s5k6aafx13_reg_addr = {
	.x_output = 0x34C,
	.y_output = 0x34E,
	.line_length_pclk = 0x342,
	.frame_length_lines = 0x340,
};
#endif

static struct msm_sensor_id_info_t s5k6aafx13_id_info = {
	.sensor_id_reg_addr = 0x0000,
	.sensor_id = 0x0,
};

#if 0 //for build
static struct msm_sensor_exp_gain_info_t s5k6aafx13_exp_gain_info = {
	.coarse_int_time_addr = 0x202,
	.global_gain_addr = 0x204,
	.vert_offset = 3,
};
#endif

int32_t s5k6aafx13_sensor_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
    int rc = 0;
#if 1    
    uint16_t chipid = 999;
#endif
	struct msm_sensor_ctrl_t *s_ctrl;
	pr_err("%s i2c_probe called\n", __func__); //build
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s %s i2c_check_functionality failed\n",
			__func__, client->name);
		rc = -EFAULT;
		return rc;
	}

    s5k6aafx13_client = client;

	s_ctrl = (struct msm_sensor_ctrl_t *)(id->driver_data);
	if (s_ctrl->sensor_i2c_client != NULL) {
		s_ctrl->sensor_i2c_client->client = client;
		if (s_ctrl->sensor_i2c_addr != 0)
			s_ctrl->sensor_i2c_client->client->addr =
				s_ctrl->sensor_i2c_addr;
	} else {
		pr_err("%s %s sensor_i2c_client NULL\n",
			__func__, client->name);
		rc = -EFAULT;
		return rc;
	}

	s_ctrl->sensordata = client->dev.platform_data;
	if (s_ctrl->sensordata == NULL) {
		pr_err("%s %s NULL sensor data\n", __func__, client->name);
		return -EFAULT;
	}
	
	rc = s_ctrl->func_tbl->sensor_power_up(s_ctrl);
	if (rc < 0) {
		pr_err("%s %s power up failed\n", __func__, client->name);
		return rc;
	}
    
#if 1//check
    pr_err("msm_sensor id: %d\n", chipid);
//    msleep(3000);
//    pr_err("msleep(3000);\n");


    rc = msm_camera_i2c_read(
            s_ctrl->sensor_i2c_client,
            s_ctrl->sensor_id_info->sensor_id_reg_addr, &chipid,
            MSM_CAMERA_I2C_WORD_DATA);
    pr_err("msm_sensor id: %d\n", chipid);

    if (rc < 0) {
        pr_err("%s: %s: read id failed\n", __func__,
        	s_ctrl->sensordata->sensor_name);
        //return rc;
        goto probe_fail;
    }

#else
	if (s_ctrl->func_tbl->sensor_match_id)
		rc = s_ctrl->func_tbl->sensor_match_id(s_ctrl);
	else
		rc = msm_sensor_match_id(s_ctrl);
	if (rc < 0)
		goto probe_fail;
#endif
	snprintf(s_ctrl->sensor_v4l2_subdev.name,
		sizeof(s_ctrl->sensor_v4l2_subdev.name), "%s", id->name);
	v4l2_i2c_subdev_init(&s_ctrl->sensor_v4l2_subdev, client,
		s_ctrl->sensor_v4l2_subdev_ops);

	msm_sensor_register(&s_ctrl->sensor_v4l2_subdev);
	goto i2c_probe_end;
    
probe_fail:
    pr_err("%s %s_i2c_probe failed\n", __func__, client->name);
    
i2c_probe_end:
#if 1
 	if (rc > 0)
		rc = 0;
		
	s_ctrl->func_tbl->sensor_power_down(s_ctrl);
#endif
    CDBG("%s_i2c_probe end\n", client->name);
	return rc; 
}

static const struct i2c_device_id s5k6aafx13_i2c_id[] = {
	{SENSOR_NAME, (kernel_ulong_t)&s5k6aafx13_s_ctrl},
	{ }
};

static struct i2c_driver s5k6aafx13_i2c_driver = {
	.id_table = s5k6aafx13_i2c_id,
    .probe  = s5k6aafx13_sensor_i2c_probe,//msm_sensor_i2c_probe,
        .driver = {
		.name = SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client s5k6aafx13_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,  //fix?
};

#ifdef F_S5K6AAFX13_POWER
/* s5k6aafx13_vreg_init */
static int s5k6aafx13_vreg_init(void)
{
    int rc = 0;
    pr_err("%s E\n", __func__);
    
    rc = sgpio_init(sgpios, CAMIO_MAX);
    pr_err("%s sgpio_init \n", __func__);
    if (rc < 0)
        goto sensor_init_fail;

    rc = svreg_init(svregs, CAMV_MAX);
    pr_err("%s svreg_init \n", __func__);
    if (rc < 0)
       goto sensor_init_fail;
    
    pr_err("%s X\n", __func__);
    return 0;
  
sensor_init_fail:
    pr_err("%s sensor_init_fail \n", __func__);
    svreg_release(svregs, CAMV_MAX); 
    sgpio_release(sgpios, CAMIO_MAX);
    return -ENODEV;
}

static int32_t s5k6aafx13_i2c_txdata(unsigned short saddr,
	 unsigned char *txdata, int length)
{
	uint32_t i = 0;
	int32_t rc = 0;
	
	struct i2c_msg msg[] = {
		{
			.addr = (saddr >> 1),
			.flags = 0,
			.len = length,
			.buf = txdata,
		},
	};

#if SENSOR_DEBUG
	if (length == 2)
		CDBG("msm_io_i2c_w: 0x%04x 0x%04x\n",
			*(u16 *) txdata, *(u16 *) (txdata + 2));
	else if (length == 4)
		CDBG("msm_io_i2c_w: 0x%04x\n", *(u16 *) txdata);
	else
		CDBG("msm_io_i2c_w: length = %d\n", length);
#endif
	
	for (i = 0; i < S5K6AAFX13_I2C_RETRY; i++) {
		rc = i2c_transfer(s5k6aafx13_client->adapter, msg, 1); 
		if (rc >= 0) {			
			return 0;
		}
		CDBG("%s: tx retry. [%02x.%02x.%02x] len=%d rc=%d\n", __func__,saddr, *txdata, *(txdata + 1), length, rc);
		msleep(S5K6AAFX13_I2C_MPERIOD);
	}
	return -EIO;	
}

#ifdef F_ICP_HD_STANDBY
static int32_t icp_hd_i2c_write_dw(unsigned short saddr,
	unsigned short waddr, unsigned short wdata1, unsigned short wdata2)
{
	int32_t rc = -EIO;
	unsigned char buf[6];

	memset(buf, 0, sizeof(buf));
	
	buf[0] = (waddr & 0xFF00)>>8;
	buf[1] = (waddr & 0x00FF);
	buf[2] = (wdata1 & 0xFF00)>>8;
	buf[3] = (wdata1 & 0x00FF);
	buf[4] = (wdata2 & 0xFF00)>>8;
	buf[5] = (wdata2 & 0x00FF);

	rc = s5k6aafx13_i2c_txdata(saddr, buf, 6);

	if (rc < 0)
		CDBG(
		"i2c_write failed, addr = 0x%x, val1 = 0x%x, val2 = 0x%x!\n",
		waddr, wdata1, wdata1);

	return rc;
}
#endif

static int32_t s5k6aafx13_i2c_write(unsigned short saddr, unsigned short waddr, unsigned short wdata, enum s5k6aafx13_width width)
{
	int32_t rc = -EIO;
	unsigned char buf[4];

	memset(buf, 0, sizeof(buf));
	switch (width) {
	case WORD_LEN: {
		buf[0] = (waddr & 0xFF00)>>8;
		buf[1] = (waddr & 0x00FF);
		buf[2] = (wdata & 0xFF00)>>8;
		buf[3] = (wdata & 0x00FF);

		rc = s5k6aafx13_i2c_txdata(saddr, buf, 4);
	}
		break;

	case TRIPLE_LEN: {
		buf[0] = (waddr & 0xFF00)>>8;
		buf[1] = (waddr & 0x00FF);
		buf[2] = wdata;
		rc = s5k6aafx13_i2c_txdata(saddr, buf, 3);
	}
		break;

	case BYTE_LEN: {
		buf[0] = waddr;
		buf[1] = wdata;
		rc = s5k6aafx13_i2c_txdata(saddr, buf, 2);
	}
		break;

	default:
		break;
	}

	if (rc < 0)
		CDBG(
		"i2c_write failed, addr = 0x%x, val = 0x%x!\n",
		waddr, wdata);

	return rc;
}

static int32_t s5k6aafx13_i2c_write_table(
	struct s5k6aafx13_i2c_reg_conf const *reg_conf_tbl,
	int num_of_items_in_table)
{
#if 1
	int i;
	int32_t rc = -EIO;

//	uint16_t poll_delay = 0;
//       uint16_t poll_retry = 0;
//       uint16_t poll_mcu_var = 0;
//       uint16_t poll_data = 0;
//       uint16_t poll_mask = 0;
//       uint16_t retry_cnt = 0;
//	uint16_t read_data = 0;
	//OTP 방어코드 추가
//	uint16_t otp_retry_cnt = 0;
//	uint16_t otp_poll_retry = 20;

	for (i = 0; i < num_of_items_in_table; i++) 
	{		
	switch(reg_conf_tbl->width )
	{
		case ZERO_LEN:
		{
			CDBG("ZERO_LEN continue ADDR = 0x%x, VALUE = 0x%x\n",reg_conf_tbl->waddr, reg_conf_tbl->wdata);
			reg_conf_tbl++;		
			rc = 0;
			continue;
		}
#ifdef BURST_MODE_INIT
		case BURST_1:
			rc = s5k6aafx13_i2c_txdata(s5k6aafx13_client->addr, 
				s5k6aafx13_regs.init_burst_settings_1, s5k6aafx13_regs.init_burst_settings_size_1);
			if (rc < 0) {
				pr_err("BURST_1 failed!\n");
				break;
			}
			reg_conf_tbl++;			
			break;
		case BURST_2:
			rc = s5k6aafx13_i2c_txdata(s5k6aafx13_client->addr, 
				s5k6aafx13_regs.init_burst_settings_2, s5k6aafx13_regs.init_burst_settings_size_2);
			if (rc < 0) {
				pr_err("BURST_2 failed!\n");
				break;
			}
			reg_conf_tbl++;			
			break;
		case BURST_3:
			rc = s5k6aafx13_i2c_txdata(s5k6aafx13_client->addr, 
				s5k6aafx13_regs.init_burst_settings_3, s5k6aafx13_regs.init_burst_settings_size_3);
			if (rc < 0) {
				pr_err("BURST_3 failed!\n");
				break;
			}
			reg_conf_tbl++;			
			break;
		case BURST_4:
			rc = s5k6aafx13_i2c_txdata(s5k6aafx13_client->addr, 
				s5k6aafx13_regs.init_burst_settings_4, s5k6aafx13_regs.init_burst_settings_size_4);
			if (rc < 0) {
				pr_err("BURST_4 failed!\n");
				break;
			}
			reg_conf_tbl++;			
			break;
		case BURST_5:
			rc = s5k6aafx13_i2c_txdata(s5k6aafx13_client->addr, 
				s5k6aafx13_regs.init_burst_settings_5, s5k6aafx13_regs.init_burst_settings_size_5);
			if (rc < 0) {
				pr_err("BURST_5 failed!\n");
				break;
			}
			reg_conf_tbl++;			
			break;
		case BURST_6:
			rc = s5k6aafx13_i2c_txdata(s5k6aafx13_client->addr, 
				s5k6aafx13_regs.init_burst_settings_6, s5k6aafx13_regs.init_burst_settings_size_6);
			if (rc < 0) {
				pr_err("BURST_6 failed!\n");
				break;
			}
			reg_conf_tbl++;			
			break;
		case BURST_7:
			rc = s5k6aafx13_i2c_txdata(s5k6aafx13_client->addr, 
				s5k6aafx13_regs.init_burst_settings_7, s5k6aafx13_regs.init_burst_settings_size_7);
			if (rc < 0) {
				pr_err("BURST_7 failed!\n");
				break;
			}
			reg_conf_tbl++;			
			break;
		case BURST_8:
			rc = s5k6aafx13_i2c_txdata(s5k6aafx13_client->addr, 
				s5k6aafx13_regs.init_burst_settings_8, s5k6aafx13_regs.init_burst_settings_size_8);
			if (rc < 0) {
				pr_err("BURST_8 failed!\n");
				break;
			}
			reg_conf_tbl++;			
			break;
		case BURST_9:
			rc = s5k6aafx13_i2c_txdata(s5k6aafx13_client->addr, 
				s5k6aafx13_regs.init_burst_settings_9, s5k6aafx13_regs.init_burst_settings_size_9);
			if (rc < 0) {
				pr_err("BURST_9 failed!\n");
				break;
			}
			reg_conf_tbl++;			
			break;
		case BURST_10:
			rc = s5k6aafx13_i2c_txdata(s5k6aafx13_client->addr, 
				s5k6aafx13_regs.init_burst_settings_10, s5k6aafx13_regs.init_burst_settings_size_10);
			if (rc < 0) {
				pr_err("BURST_10 failed!\n");
				break;
			}
			reg_conf_tbl++;			
			break;

		case BURST_11:
			rc = s5k6aafx13_i2c_txdata(s5k6aafx13_client->addr, 
				s5k6aafx13_regs.init_burst_settings_11, s5k6aafx13_regs.init_burst_settings_size_11);
			if (rc < 0) {
				pr_err("BURST_11 failed!\n");
				break;
			}
			reg_conf_tbl++;			
			break;
		case BURST_12:
			rc = s5k6aafx13_i2c_txdata(s5k6aafx13_client->addr, 
				s5k6aafx13_regs.init_burst_settings_12, s5k6aafx13_regs.init_burst_settings_size_12);
			if (rc < 0) {
				pr_err("BURST_12 failed!\n");
				break;
			}
			reg_conf_tbl++;			
			break;
		case BURST_13:
			rc = s5k6aafx13_i2c_txdata(s5k6aafx13_client->addr, 
				s5k6aafx13_regs.init_burst_settings_13, s5k6aafx13_regs.init_burst_settings_size_13);
			if (rc < 0) {
				pr_err("BURST_13 failed!\n");
				break;
			}
			reg_conf_tbl++;			
			break;
		case BURST_14:
			rc = s5k6aafx13_i2c_txdata(s5k6aafx13_client->addr, 
				s5k6aafx13_regs.init_burst_settings_14, s5k6aafx13_regs.init_burst_settings_size_14);
			if (rc < 0) {
				pr_err("BURST_14 failed!\n");
				break;
			}
			reg_conf_tbl++;			
			break;
		case BURST_15:
			rc = s5k6aafx13_i2c_txdata(s5k6aafx13_client->addr, 
				s5k6aafx13_regs.init_burst_settings_15, s5k6aafx13_regs.init_burst_settings_size_15);
			if (rc < 0) {
				pr_err("BURST_15 failed!\n");
				break;
			}
			reg_conf_tbl++;			
			break;
		case BURST_16:
			rc = s5k6aafx13_i2c_txdata(s5k6aafx13_client->addr, 
				s5k6aafx13_regs.init_burst_settings_16, s5k6aafx13_regs.init_burst_settings_size_16);
			if (rc < 0) {
				pr_err("BURST_16 failed!\n");
				break;
			}
			reg_conf_tbl++;			
			break;
		case BURST_17:
			rc = s5k6aafx13_i2c_txdata(s5k6aafx13_client->addr, 
				s5k6aafx13_regs.init_burst_settings_17, s5k6aafx13_regs.init_burst_settings_size_17);
			if (rc < 0) {
				pr_err("BURST_17 failed!\n");
				break;
			}
			reg_conf_tbl++;			
			break;
		case BURST_18:
			rc = s5k6aafx13_i2c_txdata(s5k6aafx13_client->addr, 
				s5k6aafx13_regs.init_burst_settings_18, s5k6aafx13_regs.init_burst_settings_size_18);
			if (rc < 0) {
				pr_err("BURST_18 failed!\n");
				break;
			}
			reg_conf_tbl++;			
			break;
		case BURST_19:
			rc = s5k6aafx13_i2c_txdata(s5k6aafx13_client->addr, 
				s5k6aafx13_regs.init_burst_settings_19, s5k6aafx13_regs.init_burst_settings_size_19);
			if (rc < 0) {
				pr_err("BURST_19 failed!\n");
				break;
			}
			reg_conf_tbl++;			
			break;
		case BURST_20:
			rc = s5k6aafx13_i2c_txdata(s5k6aafx13_client->addr, 
				s5k6aafx13_regs.init_burst_settings_20, s5k6aafx13_regs.init_burst_settings_size_20);
			if (rc < 0) {
				pr_err("BURST_20 failed!\n");
				break;
			}
			reg_conf_tbl++;			
			break;

		case BURST_21:
			rc = s5k6aafx13_i2c_txdata(s5k6aafx13_client->addr, 
				s5k6aafx13_regs.init_burst_settings_21, s5k6aafx13_regs.init_burst_settings_size_21);
			if (rc < 0) {
				pr_err("BURST_21 failed!\n");
				break;
			}
			reg_conf_tbl++;			
			break;
		case BURST_22:
			rc = s5k6aafx13_i2c_txdata(s5k6aafx13_client->addr, 
				s5k6aafx13_regs.init_burst_settings_22, s5k6aafx13_regs.init_burst_settings_size_22);
			if (rc < 0) {
				pr_err("BURST_22 failed!\n");
				break;
			}
			reg_conf_tbl++;			
			break;
		case BURST_23:
			rc = s5k6aafx13_i2c_txdata(s5k6aafx13_client->addr, 
				s5k6aafx13_regs.init_burst_settings_23, s5k6aafx13_regs.init_burst_settings_size_23);
			if (rc < 0) {
				pr_err("BURST_23 failed!\n");
				break;
			}
			reg_conf_tbl++;			
			break;
		case BURST_24:
			rc = s5k6aafx13_i2c_txdata(s5k6aafx13_client->addr, 
				s5k6aafx13_regs.init_burst_settings_24, s5k6aafx13_regs.init_burst_settings_size_24);
			if (rc < 0) {
				pr_err("BURST_24 failed!\n");
				break;
			}
			reg_conf_tbl++;			
			break;
		case BURST_25:
			rc = s5k6aafx13_i2c_txdata(s5k6aafx13_client->addr, 
				s5k6aafx13_regs.init_burst_settings_25, s5k6aafx13_regs.init_burst_settings_size_25);
			if (rc < 0) {
				pr_err("BURST_25 failed!\n");
				break;
			}
			reg_conf_tbl++;			
			break;
		case BURST_26:
			rc = s5k6aafx13_i2c_txdata(s5k6aafx13_client->addr, 
				s5k6aafx13_regs.init_burst_settings_26, s5k6aafx13_regs.init_burst_settings_size_26);
			if (rc < 0) {
				pr_err("BURST_26 failed!\n");
				break;
			}
			reg_conf_tbl++;			
			break;
		case BURST_27:
			rc = s5k6aafx13_i2c_txdata(s5k6aafx13_client->addr, 
				s5k6aafx13_regs.init_burst_settings_27, s5k6aafx13_regs.init_burst_settings_size_27);
			if (rc < 0) {
				pr_err("BURST_27 failed!\n");
				break;
			}
			reg_conf_tbl++;			
			break;
		case BURST_28:
			rc = s5k6aafx13_i2c_txdata(s5k6aafx13_client->addr, 
				s5k6aafx13_regs.init_burst_settings_28, s5k6aafx13_regs.init_burst_settings_size_28);
			if (rc < 0) {
				pr_err("BURST_28 failed!\n");
				break;
			}
			reg_conf_tbl++;			
			break;
		case BURST_29:
			rc = s5k6aafx13_i2c_txdata(s5k6aafx13_client->addr, 
				s5k6aafx13_regs.init_burst_settings_29, s5k6aafx13_regs.init_burst_settings_size_29);
			if (rc < 0) {
				pr_err("BURST_29 failed!\n");
				break;
			}
			reg_conf_tbl++;			
			break;
		case BURST_30:
			rc = s5k6aafx13_i2c_txdata(s5k6aafx13_client->addr, 
				s5k6aafx13_regs.init_burst_settings_30, s5k6aafx13_regs.init_burst_settings_size_30);
			if (rc < 0) {
				pr_err("BURST_30 failed!\n");
				break;
			}
			reg_conf_tbl++;			
			break;

		case BURST_31:
			rc = s5k6aafx13_i2c_txdata(s5k6aafx13_client->addr, 
				s5k6aafx13_regs.init_burst_settings_31, s5k6aafx13_regs.init_burst_settings_size_31);
			if (rc < 0) {
				pr_err("BURST_31 failed!\n");
				break;
			}
			reg_conf_tbl++;			
			break;
		case BURST_32:
			rc = s5k6aafx13_i2c_txdata(s5k6aafx13_client->addr, 
				s5k6aafx13_regs.init_burst_settings_32, s5k6aafx13_regs.init_burst_settings_size_32);
			if (rc < 0) {
				pr_err("BURST_32 failed!\n");
				break;
			}
			reg_conf_tbl++;			
			break;
		case BURST_33:
			rc = s5k6aafx13_i2c_txdata(s5k6aafx13_client->addr, 
				s5k6aafx13_regs.init_burst_settings_33, s5k6aafx13_regs.init_burst_settings_size_33);
			if (rc < 0) {
				pr_err("BURST_33 failed!\n");
				break;
			}
			reg_conf_tbl++;			
			break;
		case BURST_34:
			rc = s5k6aafx13_i2c_txdata(s5k6aafx13_client->addr, 
				s5k6aafx13_regs.init_burst_settings_34, s5k6aafx13_regs.init_burst_settings_size_34);
			if (rc < 0) {
				pr_err("BURST_34 failed!\n");
				break;
			}
			reg_conf_tbl++;			
			break;
		case BURST_35:
			rc = s5k6aafx13_i2c_txdata(s5k6aafx13_client->addr, 
				s5k6aafx13_regs.init_burst_settings_35, s5k6aafx13_regs.init_burst_settings_size_35);
			if (rc < 0) {
				pr_err("BURST_35 failed!\n");
				break;
			}
			reg_conf_tbl++;			
			break;
		case BURST_36:
			rc = s5k6aafx13_i2c_txdata(s5k6aafx13_client->addr, 
				s5k6aafx13_regs.init_burst_settings_36, s5k6aafx13_regs.init_burst_settings_size_36);
			if (rc < 0) {
				pr_err("BURST_36 failed!\n");
				break;
			}
			reg_conf_tbl++;			
			break;
		case BURST_37:
			rc = s5k6aafx13_i2c_txdata(s5k6aafx13_client->addr, 
				s5k6aafx13_regs.init_burst_settings_37, s5k6aafx13_regs.init_burst_settings_size_37);
			if (rc < 0) {
				pr_err("BURST_37 failed!\n");
				break;
			}
			reg_conf_tbl++;			
			break;
		case BURST_38:
			rc = s5k6aafx13_i2c_txdata(s5k6aafx13_client->addr, 
				s5k6aafx13_regs.init_burst_settings_38, s5k6aafx13_regs.init_burst_settings_size_38);
			if (rc < 0) {
				pr_err("BURST_38 failed!\n");
				break;
			}
			reg_conf_tbl++;			
			break;
    case BURST_39:
      rc = s5k6aafx13_i2c_txdata(s5k6aafx13_client->addr, 
        s5k6aafx13_regs.init_burst_settings_39, s5k6aafx13_regs.init_burst_settings_size_39);
      if (rc < 0) {
        pr_err("BURST_39 failed!\n");
        break;
      }
      reg_conf_tbl++;         
      break;
#endif
		default:
		{
			rc = s5k6aafx13_i2c_write(s5k6aafx13_client->addr,
								reg_conf_tbl->waddr, reg_conf_tbl->wdata,
								reg_conf_tbl->width);
			//CAM_INFO("I2C WRITE!!! ADDR = 0x%x, VALUE = 0x%x, width = %d, num_of_items_in_table=%d, i=%d\n",
			//	reg_conf_tbl->waddr, reg_conf_tbl->wdata, reg_conf_tbl->width, num_of_items_in_table, i);

			if (rc < 0)
			{
				pr_err("s5k6aafx13_i2c_write failed!\n");
				return rc;
			}
			
			if (reg_conf_tbl->mdelay_time != 0)
				mdelay(reg_conf_tbl->mdelay_time);

			reg_conf_tbl++;

			break;
		}			
	}	
	}

	return rc;
#else
	int i;
	int32_t rc = -EIO;

	for (i = 0; i < num_of_items_in_table; i++) {
		rc = s5k6aafx13_i2c_write(s5k6aafx13_client->addr,
			reg_conf_tbl->waddr, reg_conf_tbl->wdata,
			reg_conf_tbl->width);
		if (rc < 0)
			break;
		if (reg_conf_tbl->mdelay_time != 0)
			mdelay(reg_conf_tbl->mdelay_time);
		reg_conf_tbl++;
	}
#endif
	return rc;
}

#ifdef F_ICP_HD_STANDBY
/* icp_hd_vreg_init */

static int icp_hd_sensor_reset(int set)
{
	int rc = 0;

	if (sgpio_ctrl(sgpios, CAMIO_R_RST_N, 0) < 0)
	{
		rc = -EIO;
		pr_err("%s %s Failed!:%d\n", __FILE__, __func__, __LINE__);
		goto reset_fail;
	}
		
	if(set)
	{
	    if (sgpio_ctrl(sgpios, CAMIO_R_RST_N, 1) < 0)   
	    {
	    	rc = -EIO;
	    	pr_err("%s %s Failed!:%d\n", __FILE__, __func__, __LINE__);
	    	goto reset_fail;
	    }
	}

	return rc;

reset_fail:
	pr_err("%s %s Failed!:%d\n", __FILE__, __func__, __LINE__);
	return rc;
}

static int icp_hd_sensor_power(int on)
{
	int rc = 0;
	CDBG("%s %s:%d power = %d\n", __FILE__, __func__, __LINE__,on);

	if(on)
	{
	    if (sgpio_ctrl(sgpios, CAMIO_R_STB_N, 0) < 0)	rc = -EIO;

	    if (svreg_ctrl(svregs, CAM1V_S2_1P2V, 1) < 0)	rc = -EIO;
	    if (rc) 
		{
			pr_err("%s: Enable regulator CAM1V_S2_1P2V failed\n", __func__);
			goto fail;
		}
	    mdelay(1);
	    if (svreg_ctrl(svregs, CAM1V_IO_1P8V, 1) < 0)	rc = -EIO;
	    if (rc) 
		{
			pr_err("%s: Enable regulator CAM1V_IO_1P8V failed\n", __func__);
			goto fail;
		}
	    mdelay(1);
	    if (svreg_ctrl(svregs, CAM1V_LVS3_1P8V, 1) < 0)	rc = -EIO;
	    if (rc) 
		{
			pr_err("%s: Enable regulator CAM1V_LVS3_1P8V failed\n", __func__);
			goto fail;
		}
	    mdelay(1);
	    if (svreg_ctrl(svregs, CAM1V_L2B_2P8V, 1) < 0)	rc = -EIO;
	    if (rc) 
		{
			pr_err("%s: Enable regulator CAM1V_L2B_2P8V failed\n", __func__);
			goto fail;
		}
	    mdelay(1);
	    if (svreg_ctrl(svregs, CAM1V_L3_2P8V, 1) < 0)	rc = -EIO;
	    if (rc) 
		{
			pr_err("%s: Enable regulator CAM1V_L3_2P8V failed\n", __func__);
			goto fail;
		}
	    mdelay(1);
	    CDBG("%s %s ON Success:%d\n", __FILE__, __func__, __LINE__);
	}
	else
	{
		if(svregs[CAM1V_S2_1P2V].vreg)
		{
	    	if (svreg_ctrl(svregs, CAM1V_S2_1P2V, 0) < 0)   rc = -EIO;
	    	if (rc) 
			{
				pr_err("%s: Disable regulator CAM1V_S2_1P2V failed\n", __func__);
				goto fail;
			}
			regulator_put(svregs[CAM1V_S2_1P2V].vreg);
			svregs[CAM1V_S2_1P2V].vreg = NULL;
	    }
	     
	     /* MCLK will be disabled once again after this. */
	    //  (void)msm_camio_clk_disable(CAMIO_CAM_MCLK_CLK);
	    if (svreg_ctrl(svregs, CAM1V_LVS3_1P8V, 0) < 0)    rc = -EIO;
    	if (rc) 
		{
			pr_err("%s: Disable regulator CAM1V_LVS3_1P8V failed\n", __func__);
			goto fail;
		}	    
		CDBG("%s %s:%d power \n", __FILE__, __func__, __LINE__);
	    if (svreg_ctrl(svregs, CAM1V_IO_1P8V, 0) < 0)  rc = -EIO;
    	if (rc) 
		{
			pr_err("%s: Disable regulator CAM1V_IO_1P8V failed\n", __func__);
			goto fail;
		}
		CDBG("%s %s:%d power \n", __FILE__, __func__, __LINE__);
	    if (svreg_ctrl(svregs, CAM1V_L2B_2P8V, 0) < 0) rc = -EIO;
    	if (rc) 
		{
			pr_err("%s: Disable regulator CAM1V_L2B_2P8V failed\n", __func__);
			goto fail;
		}
		CDBG("%s %s:%d power \n", __FILE__, __func__, __LINE__);
	    if (svreg_ctrl(svregs, CAM1V_L3_2P8V, 0) < 0) rc = -EIO;
    	if (rc) 
		{
			pr_err("%s: Disable regulator CAM1V_L3_2P8V failed\n", __func__);
			goto fail;
		}
		CDBG("%s %s OFF Success:%d\n", __FILE__, __func__, __LINE__);
	}
	return rc;
	
fail:
	pr_err("%s %s Failed!:%d\n", __FILE__, __func__, __LINE__);
	if(svregs[CAM1V_L2B_2P8V].vreg){
		regulator_put(svregs[CAM1V_L2B_2P8V].vreg);
		svregs[CAM1V_L2B_2P8V].vreg = NULL;
	}
	if(svregs[CAM1V_S2_1P2V].vreg){
		regulator_put(svregs[CAM1V_S2_1P2V].vreg);
		svregs[CAM1V_S2_1P2V].vreg = NULL;
	}
	if(svregs[CAM1V_L3_2P8V].vreg){
		regulator_put(svregs[CAM1V_L3_2P8V].vreg);
		svregs[CAM1V_L3_2P8V].vreg = NULL;
	}

	return rc;	
}

int32_t icp_hd_sensor_standby(struct msm_sensor_ctrl_t *s_ctrl)
{
    int32_t rc = 0;
    CDBG("%s E\n", __func__);

    //rc = msm_sensor_power_up(s_ctrl);
    //CDBG(" %s : msm_sensor_power_up : rc = %d E\n",__func__, rc);  

	rc = icp_hd_sensor_power(ON);
	if (rc) 
	{
		pr_err("%s : icp_hd_power failed rc=%d\n", __func__, rc);
		return rc;
	}
 
	pr_err("%s: [wsyang_debug] msm_cam_clk_enable() / 1 \n", __func__);
	msm_cam_clk_enable(&s_ctrl->sensor_i2c_client->client->dev,
    					cam_mclk_info, &s_ctrl->cam_clk, ARRAY_SIZE(cam_mclk_info), 1);

	icp_hd_sensor_reset(ON);
    mdelay(5);

    rc = icp_hd_i2c_write_dw(ICP_SLAVE_ADDRESS, 0xf03c,  0x0009, 0x5000);
    rc = icp_hd_i2c_write_dw(ICP_SLAVE_ADDRESS, 0xe000,  0x0000, 0x0000);
    rc = icp_hd_i2c_write_dw(ICP_SLAVE_ADDRESS, 0xe004,  0x0000, 0x8fff);
	pr_err("%s: icp_hd_i2c_write_dw : %d\n", __func__, rc);
	
    if (sgpio_ctrl(sgpios, CAMIO_R_STB_N, 1) < 0)	rc = -EIO;

	if(svregs[CAM1V_S2_1P2V].vreg)
	{
		if (svreg_ctrl(svregs, CAM1V_S2_1P2V, 0) < 0)	rc = -EIO;
		if (rc)
			pr_err("%s: Disable regulator CAM1V_S2_1P2V failed\n", __func__);
		else
		{
			regulator_put(svregs[CAM1V_S2_1P2V].vreg);
			svregs[CAM1V_S2_1P2V].vreg = NULL;
		}
	}
    
    CDBG("%s X (%d)\n", __func__, rc);
    return rc;
}

#endif

/* msm_sensor_power_up */
//s5k6aafx13_sensor_power_up
static int s5k6aafx13_sensor_reset(int set)//const struct msm_camera_sensor_info *dev)
{
	int rc = 0;

    if (sgpio_ctrl(sgpios, CAMIO_F_RST_N, 0) < 0)
    {
    	rc = -EIO;
    	pr_err("%s %s Failed!:%d\n", __FILE__, __func__, __LINE__);
    	goto reset_fail;
    }
    	
    mdelay(20);

	if(set)
	{
    	if (sgpio_ctrl(sgpios, CAMIO_F_RST_N, 1) < 0)
    	{
    		rc = -EIO;		
    		pr_err("%s %s Failed!:%d\n", __FILE__, __func__, __LINE__);
    		goto reset_fail;
    	}
	}
    
    if (sgpio_ctrl(sgpios, CAMIO_F_STB_N, set) < 0)
    {
    	rc = -EIO;
    	pr_err("%s %s Failed!:%d\n", __FILE__, __func__, __LINE__);
    	goto reset_fail;
    }
    	
    mdelay(20);

    return rc;

reset_fail:
	pr_err("%s %s Failed!:%d\n", __FILE__, __func__, __LINE__);
	return rc;
}

static int s5k6aafx13_sensor_power(int on)
{
	int rc = 0;
	CDBG("%s %s:%d power = %d\n", __FILE__, __func__, __LINE__,on);
	if(on) 
	{
	    if (svreg_ctrl(svregs, CAM2V_A_2P8V, 1) < 0)	rc = -EIO;
		if (rc) 
		{
			pr_err("%s: Enable regulator l1b_2p8v failed\n", __func__);
			goto fail;
		}
		
	    msleep(1);
	    if (svreg_ctrl(svregs, CAM2V_CORE_1P5V, 1) < 0)	rc = -EIO;
		if (rc) 
		{
			pr_err("%s: Enable regulator l15a_1p5v failed\n", __func__);
			goto fail;
		}
	    
	    msleep(1);
	    if (svreg_ctrl(svregs, CAM2V_IO_1P8V, 1) < 0)	rc = -EIO;
		if (rc) 
		{
			pr_err("%s: Enable regulator lvs1b_1p8v failed\n", __func__);
			goto fail;
		}
        msleep(1);
	}
	else
	{
	    if (svreg_ctrl(svregs, CAM2V_IO_1P8V, 0) < 0)    rc = -EIO;
	    if (rc)
			pr_err("%s: Disable regulator lvs1b_1p8v failed\n", __func__);
			
	    if (svreg_ctrl(svregs, CAM2V_CORE_1P5V, 0) < 0)  rc = -EIO;
	    if (rc)
			pr_err("%s: Disable regulator l15a_1p5v failed\n", __func__);
		regulator_put(svregs[CAM2V_CORE_1P5V].vreg);
		svregs[CAM2V_CORE_1P5V].vreg = NULL;
			
	    if (svreg_ctrl(svregs, CAM2V_A_2P8V, 0) < 0) rc = -EIO;	
	    if (rc)
			pr_err("%s: Disable regulator l1b_2p8v failed\n", __func__);
		regulator_put(svregs[CAM2V_A_2P8V].vreg);
		svregs[CAM2V_A_2P8V].vreg = NULL;
	}
	CDBG("%s %s:%d\n", __FILE__, __func__, __LINE__);
	return rc;

fail:
	pr_err("%s %s:%d\n", __FILE__, __func__, __LINE__);
	if(svregs[CAM2V_A_2P8V].vreg)
	{
		regulator_put(svregs[CAM2V_A_2P8V].vreg);
		svregs[CAM2V_A_2P8V].vreg = NULL;
	}
	if(svregs[CAM2V_CORE_1P5V].vreg)
	{
		regulator_put(svregs[CAM2V_CORE_1P5V].vreg);
		svregs[CAM2V_CORE_1P5V].vreg = NULL;
	}
		
	return rc;		
}

int32_t s5k6aafx13_sensor_power_up(struct msm_sensor_ctrl_t *s_ctrl)
{
    int32_t rc = 0;
    CDBG("%s E\n", __func__);

    //config_csi = 0;
	b_snapshot_flag = false;
	 
    rc = msm_sensor_power_up(s_ctrl);
    CDBG(" %s : msm_sensor_power_up : rc = %d E\n",__func__, rc);  
    s5k6aafx13_vreg_init();
#ifdef F_ICP_HD_STANDBY
    rc = icp_hd_sensor_standby(s_ctrl);
	if (rc < 0)
	{
		pr_err("%s : icp_hd_sensor_standby FAIL!!! return~~\n", __func__);
		return rc;
	}
#endif
	rc = s5k6aafx13_sensor_power(ON);
	if (rc < 0)
	{
		pr_err("%s : s5k6aafx13_sensor_power(ON) FAIL!!! return~~\n", __func__);
		return rc;
	}
		
	CDBG(" msm_camio_clk_rate_set E\n");
#ifndef F_ICP_HD_STANDBY
    msm_cam_clk_enable(&s_ctrl->sensor_i2c_client->client->dev,
        cam_mclk_info, &s_ctrl->cam_clk, ARRAY_SIZE(cam_mclk_info), 1);
#endif
	CDBG(" msm_camio_clk_rate_set X\n");
    mdelay(5);

	rc = s5k6aafx13_sensor_reset(ON);
	if (rc < 0)
	{
		pr_err("%s : s5k6aafx13_sensor_reset(ON) FAIL!!! return~~\n", __func__);
		return rc;
	}
		
    mdelay(5);
	//CDBG("s5k6aafx13_reg_init E \n");
    //rc = s5k6aafx13_i2c_write_table(&s5k6aafx13_regs.init_parm[0], s5k6aafx13_regs.init_parm_size);
    //CDBG("s5k6aafx13_reg_init X \n");
    
	if (rc < 0)
		pr_err("%s : s5k6aafx13_i2c_write_table FAIL!!! return~~\n", __func__);

    current_fps = 31;

    CDBG("%s X (%d)\n", __func__, rc);
    return rc;
}

int32_t s5k6aafx13_sensor_power_down(struct msm_sensor_ctrl_t *s_ctrl)
{
    int32_t rc = 0;

    CDBG("%s E\n", __func__);

    msm_sensor_power_down(s_ctrl);
    CDBG(" %s : msm_sensor_power_down : rc = %d E\n",__func__, rc);

	rc = s5k6aafx13_sensor_reset(OFF);
	if (rc < 0) {
		pr_err("%s : s5k6aafx13_sensor_reset off failed!\n", __func__);		
	}
     /* MCLK will be disabled once again after this. */
    //  (void)msm_camio_clk_disable(CAMIO_CAM_MCLK_CLK);
    rc = s5k6aafx13_sensor_power(OFF);
	if (rc < 0) {
		pr_err("%s : s5k6aafx13_sensor_power off failed rc=%d\n", __func__, rc);		
	}
#ifdef F_ICP_HD_STANDBY
    if (sgpio_ctrl(sgpios, CAMIO_R_STB_N, 0) < 0)   rc = -EIO;

    rc = icp_hd_sensor_reset(OFF);
	if (rc < 0) {
		pr_err("%s : icp_hd_sensor_reset off failed rc=%d\n", __func__, rc);		
	}

    rc = icp_hd_sensor_power(OFF);
	if (rc < 0) {
		pr_err("%s : icp_hd_sensor_power off failed rc=%d\n", __func__, rc);		
	}
#endif

    svreg_release(svregs, CAMV_MAX);
    sgpio_release(sgpios, CAMIO_MAX);
    si2c_release();    

    CDBG("%s X (%d)\n", __func__, rc);
    return rc;
}
#endif

void s5k6aafx13_sensor_start_stream(struct msm_sensor_ctrl_t *s_ctrl)
{
#if 0
	int rc = 0;

	rc = s5k6aafx13_i2c_write_table(&s5k6aafx13_regs.streaming_on_settings[0],
					s5k6aafx13_regs.streaming_on_settings_size);
	if (rc < 0)
		SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);

    SKYCDBG("%s: %d\n", __func__, __LINE__);
#endif
}

void s5k6aafx13_sensor_stop_stream(struct msm_sensor_ctrl_t *s_ctrl)
{
#if 0
	int rc = 0;

	rc = s5k6aafx13_i2c_write_table(&s5k6aafx13_regs.streaming_off_settings[0],
					s5k6aafx13_regs.streaming_off_settings_size);

	if (rc < 0)
		SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);

    SKYCDBG("%s: %d\n", __func__, __LINE__);
#endif
}

int s5k6aafx13_sensor_init(struct msm_sensor_ctrl_t *s_ctrl)
{
	int rc = 0;

	SKYCDBG("%s: E\n",__func__);

	rc = s5k6aafx13_i2c_write_table(&s5k6aafx13_regs.init_parm[0], s5k6aafx13_regs.init_parm_size);
    //pr_err("%s : s5k6aafx13_i2c regs init\n", __func__);
    
    //rc = si2c_write_param(SI2C_SA, SI2C_INIT, s5k6aafx13_params);

    if (rc < 0)
        goto sensor_init_fail;


    sensor_mode = 1;

    SKYCDBG("%s: X\n",__func__);    
    return rc;
    
#if 1//needtocheck    
sensor_init_fail:

#if 0
	if (s5k6aafx13_ctrl) {
		kfree(s5k6aafx13_ctrl);
		s5k6aafx13_ctrl = NULL;
	}
	(void)s5k6aafx13_power_off();
	svreg_release(svregs, CAMV_MAX);
	sgpio_release(sgpios, CAMIO_MAX);
#endif    

    si2c_release();
    
	SKYCDBG("%s err(%d)\n", __func__, rc);
	return rc;
#endif    
}
#if 0

int32_t s5k6aafx13_sensor_write_init_settings(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc;
	rc = msm_sensor_write_all_conf_array(
		s_ctrl->sensor_i2c_client,
		s_ctrl->msm_sensor_reg->init_settings,
		s_ctrl->msm_sensor_reg->init_size);
	return rc;
}

/* msm_sensor_write_res_settings */
//s5k6aafx13_sensor_write_res_settings
int32_t s5k6aafx13_sensor_write_res_settings(struct msm_sensor_ctrl_t *s_ctrl,
	uint16_t res)
{
	int32_t rc;
	rc = msm_sensor_write_conf_array(
		s_ctrl->sensor_i2c_client,
		s_ctrl->msm_sensor_reg->mode_settings, res);
	if (rc < 0)
		return rc;

	rc = msm_sensor_write_output_settings(s_ctrl, res);
	if (rc < 0)
		return rc;

	if (s_ctrl->func_tbl->sensor_adjust_frame_lines)
		rc = s_ctrl->func_tbl->sensor_adjust_frame_lines(s_ctrl, res);

	return rc;
}
#endif

#if 1 // sangwon
static int32_t s5k6aafx13_snapshot_config(void)
{
	int32_t rc = 0;
#if 0 // Debugging point for Crash buf //wsyang_temp
    unsigned short reg_value = 0;
#endif

	/* set snapshot resolution to 1280x960 */
	SKYCDBG("%s start\n",__func__);
	rc = s5k6aafx13_i2c_write_table(&s5k6aafx13_regs.snapshot_cfg_settings[0], s5k6aafx13_regs.snapshot_cfg_settings_size);
	
	if (rc < 0)
	{
		pr_err("s5k6aafx13_i2c_write_table FAIL!!! return~~\n");
		return rc;
	}
#if 0 // Debugging point for Crash buf //wsyang_temp
    s5k6aafx13_i2c_write(s5k6aafx13_client->addr, 0xFCFC, 0xD000, WORD_LEN);
    s5k6aafx13_i2c_write(s5k6aafx13_client->addr, 0x002C, 0x7000, WORD_LEN);

    s5k6aafx13_i2c_write(s5k6aafx13_client->addr, 0x002E, 0x01CC, WORD_LEN);
    s5k6aafx13_i2c_read(s5k6aafx13_client->addr, 0x0F12, &reg_value, WORD_LEN);
    CAM_ERR("[wsyang_temp] s5k6aafx13_i2c_read [0x01CC] = 0x%x",reg_value);

    s5k6aafx13_i2c_write(s5k6aafx13_client->addr, 0x002E, 0x10EE, WORD_LEN);
    s5k6aafx13_i2c_read(s5k6aafx13_client->addr, 0x0F12, &reg_value, WORD_LEN);
    CAM_ERR("[wsyang_temp] s5k6aafx13_i2c_read [0x10EE] = 0x%x",reg_value);


    s5k6aafx13_i2c_write(s5k6aafx13_client->addr, 0x002E, 0x01E2, WORD_LEN);
    s5k6aafx13_i2c_read(s5k6aafx13_client->addr, 0x0F12, &reg_value, WORD_LEN);
    CAM_ERR("[wsyang_temp] s5k6aafx13_i2c_read [0x01E2] = 0x%x",reg_value);

    s5k6aafx13_i2c_write(s5k6aafx13_client->addr, 0x002E, 0x0222, WORD_LEN);
    s5k6aafx13_i2c_read(s5k6aafx13_client->addr, 0x0F12, &reg_value, WORD_LEN);
    CAM_ERR("[wsyang_temp] s5k6aafx13_i2c_read [0x0222] = 0x%x",reg_value);

    s5k6aafx13_i2c_write(s5k6aafx13_client->addr, 0x002E, 0x0228, WORD_LEN);
    s5k6aafx13_i2c_read(s5k6aafx13_client->addr, 0x0F12, &reg_value, WORD_LEN);
    CAM_ERR("[wsyang_temp] s5k6aafx13_i2c_read [0x0228] = 0x%x",reg_value);

    s5k6aafx13_i2c_write(s5k6aafx13_client->addr, 0x002E, 0x04B0, WORD_LEN);
    s5k6aafx13_i2c_read(s5k6aafx13_client->addr, 0x0F12, &reg_value, WORD_LEN);
    CAM_ERR("[wsyang_temp] s5k6aafx13_i2c_read [0x04B0] = 0x%x",reg_value);
#endif

	SKYCDBG("%s end rc = %d\n",__func__, rc);
	
	return rc;
}
#endif

int32_t s5k6aafx13_sensor_setting(struct msm_sensor_ctrl_t *s_ctrl,
			int update_type, int res)
{
	int32_t rc = 0;

	s_ctrl->func_tbl->sensor_stop_stream(s_ctrl);
	msleep(30);
	if (update_type == MSM_SENSOR_REG_INIT) {
		s_ctrl->curr_csi_params = NULL;
		msm_sensor_enable_debugfs(s_ctrl);
//needtocheck//        
		s5k6aafx13_sensor_init(s_ctrl);//msm_sensor_write_init_settings(s_ctrl);
		
	} else if (update_type == MSM_SENSOR_UPDATE_PERIODIC) {
		//msm_sensor_write_res_settings(s_ctrl, res);
		if (s_ctrl->curr_csi_params != s_ctrl->csi_params[res]) {
            SKYCDBG("%s: ==> MIPI setting  E %d\n", __func__, update_type);
			s_ctrl->curr_csi_params = s_ctrl->csi_params[res];
			s_ctrl->curr_csi_params->csid_params.lane_assign =
				s_ctrl->sensordata->sensor_platform_info->
				csi_lane_params->csi_lane_assign;
			s_ctrl->curr_csi_params->csiphy_params.lane_mask =
				s_ctrl->sensordata->sensor_platform_info->
				csi_lane_params->csi_lane_mask;
			v4l2_subdev_notify(&s_ctrl->sensor_v4l2_subdev,
				NOTIFY_CSID_CFG,
				&s_ctrl->curr_csi_params->csid_params);
			mb();
			v4l2_subdev_notify(&s_ctrl->sensor_v4l2_subdev,
				NOTIFY_CSIPHY_CFG,
				&s_ctrl->curr_csi_params->csiphy_params);
			mb();
			msleep(20);
            SKYCDBG("%s: ==> MIPI setting  X %d\n", __func__, update_type);		
		}

		v4l2_subdev_notify(&s_ctrl->sensor_v4l2_subdev,
			NOTIFY_PCLK_CHANGE, &s_ctrl->msm_sensor_reg->
			output_settings[res].op_pixel_clk);
		s_ctrl->func_tbl->sensor_start_stream(s_ctrl);
#if 0//needtocheck		
		switch (res) {
			case 0:
//				rc = ce1612_snapshot_config(s_ctrl);	
				break;

			case 1:
//				rc = ce1612_video_config(s_ctrl);
                si2c_write_param(SI2C_SA, SI2C_INIT, s5k6aafx13_params);

				SKYCDBG("Sensor setting : Case 1 Video config"); 
				break;

	      //case 2: 
			//	rc = ce1612_1080p_config(s_ctrl);	
			//	break;

			case 3: 
//				rc = ce1612_ZSL_config(s_ctrl);	
				SKYCDBG("Sensor setting : Case 3 ZSL config");				
				break;	
				 
			default:
//				rc = ce1612_video_config(s_ctrl);
				SKYCDBG("Sensor setting : Default Video config"); 
				break;
		}
#endif
		sensor_mode = res;
		SKYCDBG("Sensor setting : Res = %d\n", res);
        
		msleep(30);
	}
	return rc;
}

int32_t s5k6aafx13_sensor_setting1(struct msm_sensor_ctrl_t *s_ctrl,
			int update_type, int res)
{
	int32_t rc = 0;
	static int csi_config;
    SKYCDBG("%s: update_type = %d, res=%d E\n", __func__, update_type, res);

	s_ctrl->func_tbl->sensor_stop_stream(s_ctrl);
	msleep(30);
	if (update_type == MSM_SENSOR_REG_INIT) 
	{
		CDBG("Register INIT\n");
		s_ctrl->curr_csi_params = NULL;
		msm_sensor_enable_debugfs(s_ctrl);       
		s5k6aafx13_sensor_init(s_ctrl);
		csi_config = 0;
	} 
	else if (update_type == MSM_SENSOR_UPDATE_PERIODIC) 
	{
		CDBG("PERIODIC : %d\n", res);
		msleep(30);
		if (!csi_config) 
		{
            SKYCDBG("%s: ==> MIPI setting  E %d\n", __func__, update_type);
			s_ctrl->curr_csic_params = s_ctrl->csic_params[res];
			CDBG("CSI config in progress\n");
			v4l2_subdev_notify(&s_ctrl->sensor_v4l2_subdev,	NOTIFY_CSIC_CFG, s_ctrl->curr_csic_params);
			CDBG("CSI config is done\n");
			mb();
			msleep(30);
			csi_config = 1;
            SKYCDBG("%s: ==> MIPI setting  X %d\n", __func__, update_type);	
		}
		v4l2_subdev_notify(&s_ctrl->sensor_v4l2_subdev,	NOTIFY_PCLK_CHANGE,	&s_ctrl->sensordata->pdata->ioclk.vfe_clk_rate);

		s_ctrl->func_tbl->sensor_start_stream(s_ctrl);
#if 1//needtocheck		
        switch (res) {
            case 0:			// SNAPSHOT
				if(!b_snapshot_flag) 
				{
					rc = s5k6aafx13_snapshot_config();
					b_snapshot_flag = true;
					SKYCDBG("s5k6aafx13_snapshot_config, rc = %d \n", rc);
					if (rc < 0)
			        {
						pr_err("s5k6aafx13_snapshot_config FAIL!!! return~~\n");
						return rc;
					}			
				}
                break;

            case 1:				// PREVIEW
				b_snapshot_flag = false;
                SKYCDBG("%s: si2c_write_param(SI2C_SA, SI2C_PREVIEW, s5k6aafx13_params) / CALL\n",__func__);
                //rc = si2c_write_param(SI2C_SA, SI2C_PREVIEW, s5k6aafx13_params);
				rc = s5k6aafx13_i2c_write_table(&s5k6aafx13_regs.preview_cfg_settings[0], s5k6aafx13_regs.preview_cfg_settings_size);           
                if (rc < 0) {
                    pr_err("%s video config err(%d)\n", __func__, rc);
                    return rc;
                }
#if 1//for_test
                if(current_fps != 31)
                    s5k6aafx13_sensor_set_preview_fps(s_ctrl ,current_fps);
#else                  
                if ((previewFPS > C_PANTECH_CAMERA_MIN_PREVIEW_FPS) && (previewFPS < C_PANTECH_CAMERA_MAX_PREVIEW_FPS)) 
	            {  
			       SKYCDBG("%s: preview_fps=%d\n", __func__, previewFPS);
                   s5k6aafx13_sensor_set_preview_fps(NULL ,previewFPS);
                }
#endif
                SKYCDBG("Sensor setting : Case 1 Video config"); 
                break;

          //case 2: 
            //  rc = ce1612_1080p_config(s_ctrl);   
            //  break;

            case 3: 
//              rc = ce1612_ZSL_config(s_ctrl); 
                SKYCDBG("Sensor setting : Case 3 ZSL config");              
                break;  
                 
            default:
//              rc = ce1612_video_config(s_ctrl);
                SKYCDBG("Sensor setting : Default Video config"); 
                break;
        }
#endif
		sensor_mode = res;
		SKYCDBG("Sensor setting : Res = %d\n", res);

		msleep(50);
	}
	return rc;
}

/* sensor_set_function */
//s5k6aafx13_set_function 
#ifdef CONFIG_PANTECH_CAMERA
static int s5k6aafx13_sensor_set_brightness(struct msm_sensor_ctrl_t *s_ctrl ,int8_t brightness)
{
	int32_t rc = 0;
//	int i = 0;
	SKYCDBG("%s start~ receive brightness = %d\n",__func__, brightness);

	if ((brightness < C_PANTECH_CAMERA_MIN_BRIGHTNESS) || (brightness > C_PANTECH_CAMERA_MAX_BRIGHTNESS)) {
		pr_err("%s error. brightness=%d\n", __func__, brightness);
		return 0;//-EINVAL;
	}

	rc = s5k6aafx13_i2c_write_table(s5k6aafx13_regs.bright_cfg_settings[brightness],	
					s5k6aafx13_regs.bright_cfg_settings_size);
	if (rc < 0)
		pr_err("CAMERA_BRIGHTNESS I2C FAIL!!! return~~\n");

	SKYCDBG("%s end\n",__func__);
	return rc;
}

static int s5k6aafx13_sensor_set_effect(struct msm_sensor_ctrl_t *s_ctrl ,int8_t effect)
{
//	uint16_t reg_addr;
//	uint16_t reg_val;
	long rc = 0;

	SKYCDBG("%s start\n",__func__);

	if(effect < CAMERA_EFFECT_OFF || effect >= CAMERA_EFFECT_MAX /*need to check*/){
		pr_err("%s error. effect=%d\n", __func__, effect);
		return 0;//-EINVAL;
	}

	rc = s5k6aafx13_i2c_write_table(s5k6aafx13_regs.effect_cfg_settings[effect],
					s5k6aafx13_regs.effect_cfg_settings_size);
	if (rc < 0)
		pr_err("CAMERA_WB I2C FAIL!!! return~~\n");
		
	SKYCDBG("%s end\n",__func__);
	return rc;
}

static int s5k6aafx13_sensor_set_exposure_mode(struct msm_sensor_ctrl_t *s_ctrl ,int8_t exposure)
{
	int32_t rc = 0;

	SKYCDBG("%s  exposure = %d\n",__func__, exposure);

	if ((exposure < 0) || (exposure >= 4))
	{
		pr_err("%s FAIL!!! return~~  exposure = %d\n",__func__,exposure);
		return 0;//-EINVAL;
	}

	rc = s5k6aafx13_i2c_write_table(s5k6aafx13_regs.exposure_cfg_settings[exposure],
					s5k6aafx13_regs.exposure_cfg_settings_size);
	if (rc < 0)
		pr_err("CAMERA_EFFECT_SEPIA I2C FAIL!!! return~~\n");	
	
	SKYCDBG("%s end\n",__func__);

	return rc;
}

static int  s5k6aafx13_sensor_set_wb(struct msm_sensor_ctrl_t *s_ctrl ,int8_t whitebalance)
{
	
	int32_t rc = 0;
//	int8_t m_wb = 0;
		
	SKYCDBG("%s start  whitebalance=%d\n",__func__, whitebalance);

	rc = s5k6aafx13_i2c_write_table(s5k6aafx13_regs.wb_cfg_settings[whitebalance-1], s5k6aafx13_regs.wb_cfg_settings_size);
	if (rc < 0)
		pr_err("CAMERA_WB I2C FAIL!!! return~~\n");	

	SKYCDBG("%s end\n",__func__);
	return rc;
}

static int s5k6aafx13_sensor_set_preview_fps(struct msm_sensor_ctrl_t *s_ctrl ,int8_t preview_fps)
{
	/* 0 : variable 5~14fps, 1 ~ 14 : fixed fps, 31 : variable 5~14fps */
	/* default: variable 5 ~ 14fps */
#define S5K6AAFX13_MAX_PREVIEW_FPS 14
	int32_t rc = 0;	
#if 0 // Debugging point for Crash buf //wsyang_temp
    unsigned short reg_value = 0;
#endif

	if ((preview_fps < C_PANTECH_CAMERA_MIN_PREVIEW_FPS) || (preview_fps > C_PANTECH_CAMERA_MAX_PREVIEW_FPS)) 
	{
		pr_err("%s: -EINVAL, preview_fps=%d\n", 
			__func__, preview_fps);
		return -EINVAL;
	}

	//limit actually max frame rate
	if((preview_fps > S5K6AAFX13_MAX_PREVIEW_FPS) && (preview_fps < C_PANTECH_CAMERA_MAX_PREVIEW_FPS))
		preview_fps = S5K6AAFX13_MAX_PREVIEW_FPS;

	SKYCDBG("%s: preview_fps=%d\n", __func__, preview_fps);

	rc = s5k6aafx13_i2c_write_table(s5k6aafx13_regs.preview_fps_cfg_settings[preview_fps],
					s5k6aafx13_regs.preview_fps_cfg_settings_size);
	if (rc < 0)
		pr_err("CAMERA_SET_PREVIEW_FPS I2C FAIL!!! return~~\n");

    current_fps = preview_fps;
	SKYCDBG("%s end rc = %d\n",__func__, rc);

	return rc;
}

static int s5k6aafx13_sensor_set_reflect(struct msm_sensor_ctrl_t *s_ctrl ,int8_t reflect)
{
	int32_t rc = 0;
//	int32_t i = 0;
//	int8_t npolling = -1;

	SKYCDBG("%s  reflect = %d\n",__func__, reflect);

	if ((reflect < 0) || (reflect >= 4))
	{
		pr_err("%s FAIL!!! return~~  reflect = %d\n",__func__,reflect);
		return 0;//-EINVAL;
	}

#ifdef F_PANTECH_CAMERA_FIX_CFG_REFLECT
	rc = s5k6aafx13_i2c_write_table(s5k6aafx13_regs.reflect_cfg_settings[reflect],
				s5k6aafx13_regs.reflect_cfg_settings_size);
#endif

	if (rc < 0)
		pr_err("CAMERA_SET_REFLECT I2C FAIL!!! return~~\n");
	
	SKYCDBG("%s end\n",__func__);

	return rc;
}
#endif
//...

static int __init msm_sensor_init_module(void)
{
//    CDBG("%s E\n", __func__);
	return i2c_add_driver(&s5k6aafx13_i2c_driver);
}

static struct v4l2_subdev_core_ops s5k6aafx13_subdev_core_ops = {
	.ioctl = msm_sensor_subdev_ioctl,
	.s_power = msm_sensor_power,
};

static struct v4l2_subdev_video_ops s5k6aafx13_subdev_video_ops = {
	.enum_mbus_fmt = msm_sensor_v4l2_enum_fmt,
};

static struct v4l2_subdev_ops s5k6aafx13_subdev_ops = {
	.core = &s5k6aafx13_subdev_core_ops,
	.video  = &s5k6aafx13_subdev_video_ops,
};

static struct msm_sensor_fn_t s5k6aafx13_func_tbl = {
#if 1//def F_STREAM_ON_OFF
	.sensor_start_stream = s5k6aafx13_sensor_start_stream,//msm_sensor_start_stream,
	.sensor_stop_stream = s5k6aafx13_sensor_stop_stream,//msm_sensor_stop_stream,
#endif
#if 0
	.sensor_group_hold_on = msm_sensor_group_hold_on,
	.sensor_group_hold_off = msm_sensor_group_hold_off,
	.sensor_set_fps = msm_sensor_set_fps,
	.sensor_write_exp_gain = msm_sensor_write_exp_gain1,
	.sensor_write_snapshot_exp_gain = msm_sensor_write_exp_gain1,
#endif
	.sensor_setting = s5k6aafx13_sensor_setting, //s5k6aafx13_sensor_setting
	.sensor_csi_setting = s5k6aafx13_sensor_setting1,//msm_sensor_setting1,
	.sensor_set_sensor_mode = msm_sensor_set_sensor_mode,
	.sensor_mode_init = msm_sensor_mode_init,
	.sensor_get_output_info = msm_sensor_get_output_info,
	.sensor_config = msm_sensor_config,
#ifdef F_S5K6AAFX13_POWER
    .sensor_power_up = s5k6aafx13_sensor_power_up,//msm_sensor_power_up,
    .sensor_power_down = s5k6aafx13_sensor_power_down,//msm_sensor_power_down,
#else
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = msm_sensor_power_down,
#endif
//	.sensor_adjust_frame_lines = msm_sensor_adjust_frame_lines,
	.sensor_get_csi_params = msm_sensor_get_csi_params,
//s5k6aafx13_set_function
#ifdef CONFIG_PANTECH_CAMERA
    .sensor_set_brightness = s5k6aafx13_sensor_set_brightness,
    .sensor_set_effect = s5k6aafx13_sensor_set_effect,
    .sensor_set_exposure_mode = s5k6aafx13_sensor_set_exposure_mode,
    .sensor_set_wb = s5k6aafx13_sensor_set_wb,
    .sensor_set_preview_fps = s5k6aafx13_sensor_set_preview_fps,
    .sensor_set_reflect = s5k6aafx13_sensor_set_reflect,    
#endif
};

static struct msm_sensor_reg_t s5k6aafx13_r1_regs = {
	.default_data_type = MSM_CAMERA_I2C_WORD_DATA, //MSM_CAMERA_I2C_BYTE_DATA,
#if 0//def F_STREAM_ON_OFF	        
	.start_stream_conf = s5k6aafx13_start_settings,
	.start_stream_conf_size = ARRAY_SIZE(s5k6aafx13_start_settings),
	.stop_stream_conf = s5k6aafx13_stop_settings,
	.stop_stream_conf_size = ARRAY_SIZE(s5k6aafx13_stop_settings),
#endif
#if 0
	.group_hold_on_conf = s5k6aafx13_groupon_settings,
	.group_hold_on_conf_size = ARRAY_SIZE(s5k6aafx13_groupon_settings),
	.group_hold_off_conf = s5k6aafx13_groupoff_settings,
	.group_hold_off_conf_size =
		ARRAY_SIZE(s5k6aafx13_groupoff_settings),
#endif
	.init_settings = NULL,//&s5k6aafx13_init_conf[0],//NULL,//&s5k6aafx13_init_conf[0],
	.init_size = 0,//ARRAY_SIZE(s5k6aafx13_init_conf),//0,//ARRAY_SIZE(s5k6aafx13_init_conf),
	.mode_settings = NULL,//&s5k6aafx13_confs[0],//NULL,//&s5k6aafx13_confs[0],
	.output_settings = &s5k6aafx13_dimensions[0],
	.num_conf = ARRAY_SIZE(s5k6aafx13_cid_cfg),//ARRAY_SIZE(s5k6aafx13_confs),
	//include reg settings
};

static struct msm_sensor_ctrl_t s5k6aafx13_s_ctrl = {
	.msm_sensor_reg = &s5k6aafx13_r1_regs,
	.sensor_i2c_client = &s5k6aafx13_sensor_i2c_client,
	.sensor_i2c_addr = 0x5A,//0xB4,//0x5A,
//	.sensor_output_reg_addr = &s5k6aafx13_reg_addr,
	.sensor_id_info = &s5k6aafx13_id_info,  //not use matchid
//	.sensor_exp_gain_info = &s5k6aafx13_exp_gain_info,
	.cam_mode = MSM_SENSOR_MODE_INVALID,
	.csic_params = &s5k6aafx13_csic_params_array[0],
	.csi_params = &s5k6aafx13_csi_params_array[0],
	.msm_sensor_mutex = &s5k6aafx13_mut,
	.sensor_i2c_driver = &s5k6aafx13_i2c_driver,
	.sensor_v4l2_subdev_info = s5k6aafx13_subdev_info,  
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(s5k6aafx13_subdev_info),
	.sensor_v4l2_subdev_ops = &s5k6aafx13_subdev_ops,
	.func_tbl = &s5k6aafx13_func_tbl,
	.clk_rate = MSM_SENSOR_MCLK_24HZ,
};

late_initcall(msm_sensor_init_module);//module_init//late_initcall
MODULE_DESCRIPTION("Samsung 1.3MP SoC Sensor Driver");
MODULE_LICENSE("GPL v2");

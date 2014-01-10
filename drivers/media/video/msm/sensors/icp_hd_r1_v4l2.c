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

#include "icp_hd_r1_v4l2.h"

#define SENSOR_NAME "icp_hd"
#define PLATFORM_DRIVER_NAME "msm_camera_icp_hd"
#define icp_hd_obj icp_hd_##obj

#define F_ICP_HD_POWER

/* Micron ICP_HD Registers and their values */
#define SENSOR_DEBUG 0

#define icp_hd_delay_msecs_stream 100//50//200//500
#define ICP_HD_I2C_RETRY	5//10
#define ICP_HD_I2C_MPERIOD	30//200
#define ICP_HD_SNAPSHOT_RETRY 	200//30
#define ICP_HD_PREVIEW_RETRY 	30
#define ICP_HD_POLLING_RETRY	 	30
#define ICP_HD_CFG_REFLECT_MAX	 	4

#ifdef F_PANTECH_CAMERA_FIX_CFG_SCENE_MODE
#define CAMERA_BESTSHOT_MAX	21
static int icp_scenemode[CAMERA_BESTSHOT_MAX] = {
	ICP_HD_CFG_SCENE_MODE_OFF, ICP_HD_CFG_SCENE_MODE_AUTO, 
	ICP_HD_CFG_SCENE_MODE_LANDSCAPE, ICP_HD_CFG_SCENE_MODE_WINTER,
	ICP_HD_CFG_SCENE_MODE_BEACH, ICP_HD_CFG_SCENE_MODE_SUNSET,
	ICP_HD_CFG_SCENE_MODE_NIGHT, ICP_HD_CFG_SCENE_MODE_PORTRAIT,
       -1  /*BACKLIGHT*/, ICP_HD_CFG_SCENE_MODE_SPORTS,
       -1  /*ANTISHAKE*/,  -1 /*FLOWERS*/,
       -1  /*CANDLELIGHT*/,  -1  /*FIREWORKS*/,
       ICP_HD_CFG_SCENE_MODE_PARTY, -1 /*NIGHT_PORTRAIT*/,
       -1  /*THEATRE*/, -1  /*ACTION*/,
       -1  /*AR*/, ICP_HD_CFG_SCENE_MODE_INDOOR,
       ICP_HD_CFG_SCENE_MODE_TEXT
};
	
#endif
#if 1
#undef CDBG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#endif 
#if defined(CONFIG_MACH_MSM8X60_EF33S) || defined(CONFIG_MACH_MSM8X60_EF34K) || defined(CONFIG_MACH_MSM8X60_EF35L)
/* icp_hd + S5K6AAFX13 */

#define CAMIO_R_RST_N	0
#define CAMIO_R_STB_N	1
//#define CAMIO_F_RST_N	2
//#define CAMIO_F_STB_N	3
#define CAMIO_MAX	2

static sgpio_ctrl_t sgpios[CAMIO_MAX] = {
	{CAMIO_R_RST_N, "CAMIO_R_RST_N", 106},
	{CAMIO_R_STB_N, "CAMIO_R_STB_N",  57},//46},//EF39
};

#define CAM1V_IO_1P8V   0
#define CAM1V_L2B_2P8V  1
#define CAM1V_S2_1P2V   2
#define CAM1V_LVS3_1P8V 3
#define CAM1V_L3_2P8V   4
#define CAMV_MAX	5

static svreg_ctrl_t svregs[CAMV_MAX] = {
    {CAM1V_IO_1P8V,    "8901_mvs0",  NULL, 0}, //1800
    {CAM1V_L2B_2P8V,    "8901_l2",  NULL, 2800},
    {CAM1V_S2_1P2V,    "8901_s2",  NULL, 1300},
    {CAM1V_LVS3_1P8V,    "8901_lvs3",  NULL, 0}, //1800
    {CAM1V_L3_2P8V,    "8901_l3",  NULL, 2800},
};

static icp_hd_params_info_t params_info;

static int current_fps = 31;
static int icp_hd_sensor_set_preview_fps(struct msm_sensor_ctrl_t *s_ctrl ,int8_t preview_fps);

#else
#error "unknown machine!"
#endif

#if 1
static struct msm_cam_clk_info cam_mclk_info[] = {
	{"cam_clk", MSM_SENSOR_MCLK_24HZ},
};
#endif

#if 1//needtocheck
/* I2C slave address */
//#define SI2C_SA	((icp_hd_client->addr) >> 1)

#if 0
typedef struct {
	struct work_struct work;
} icp_hd_work_t;

typedef struct {
	const struct msm_camera_sensor_info *sinfo;
} icp_hd_ctrl_t;
#endif

//extern si2c_const_param_t icp_hd_const_params[SI2C_PID_MAX];
//static si2c_param_t icp_hd_params[SI2C_PID_MAX];

//static mt9d113_ctrl_t *icp_hd_ctrl = NULL;
static struct i2c_client *icp_hd_client = NULL;
//static mt9d113_work_t *icp_hd_work = NULL;
#endif

#ifdef F_PANTECH_CAMERA_TUP_LOAD_FILE
static icp_hd_tune_state_type icp_hd_tup_state = ICP_HD_TUNE_STATE_NONE;
static icp_hd_tune_mode_type icp_hd_tup_mode = ICP_HD_TUNE_STATE_TUNNING_MODE_ON;
static icp_hd_params_tbl_t icp_hd_params_tbl;
#endif

DEFINE_MUTEX(icp_hd_mut);
static struct msm_sensor_ctrl_t icp_hd_s_ctrl;

static int8_t sensor_mode = -1;   			// 0: full size,  1: qtr size, 2: fullhd size, 3: ZSL

#ifdef F_PANTECH_CAMERA_FIX_CFG_LED_MODE
int led_auto;
unsigned int nbrightness = 0;
static int8_t is_led_work = 0;
#endif

#if 1
static int32_t icp_hd_i2c_read(unsigned short saddr, unsigned short raddr, unsigned int *rdata, enum icp_hd_width width);
#endif

#if 0//def F_STREAM_ON_OFF	
static struct msm_camera_i2c_reg_conf icp_hd_start_settings[] = {
	{0x0100, 0x01},
};

static struct msm_camera_i2c_reg_conf icp_hd_stop_settings[] = {
	{0x0100, 0x00},
};
#endif

#if 0
static struct msm_camera_i2c_reg_conf icp_hd_groupon_settings[] = {
	{0x104, 0x01},
};

static struct msm_camera_i2c_reg_conf icp_hd_groupoff_settings[] = {
	{0x104, 0x00},
};
#endif

#if 0
static struct msm_camera_i2c_reg_conf icp_hd_prev_settings[] = {
};

static struct msm_camera_i2c_reg_conf icp_hd_snap_settings[] = {
};

static struct msm_camera_i2c_reg_conf icp_hd_recommend_settings[] = {
};
#endif

static struct v4l2_subdev_info icp_hd_subdev_info[] = {
	{
	.code   =V4L2_MBUS_FMT_YUYV8_2X8,// V4L2_MBUS_FMT_SBGGR10_1X10,//for isp
	.colorspace = V4L2_COLORSPACE_JPEG,
	.fmt    = 1,
	.order    = 0,
	},
	/* more can be supported, to be added later */
};

#if 0
static struct msm_camera_i2c_conf_array icp_hd_init_conf[] = {
	{&icp_hd_recommend_settings[0],
	ARRAY_SIZE(icp_hd_recommend_settings), 0, MSM_CAMERA_I2C_WORD_DATA}
};

static struct msm_camera_i2c_conf_array icp_hd_confs[] = {
	{&icp_hd_snap_settings[0],
	ARRAY_SIZE(icp_hd_snap_settings), 0, MSM_CAMERA_I2C_WORD_DATA},
	{&icp_hd_prev_settings[0],
	ARRAY_SIZE(icp_hd_prev_settings), 0, MSM_CAMERA_I2C_WORD_DATA},
	{&icp_hd_prev_settings[0],
	ARRAY_SIZE(icp_hd_prev_settings), 0, MSM_CAMERA_I2C_WORD_DATA},	
	{&icp_hd_snap_settings[0],
	ARRAY_SIZE(icp_hd_snap_settings), 0, MSM_CAMERA_I2C_WORD_DATA},	
};
#endif



#if 0
static struct msm_sensor_output_info_t icp_hd_dimensions[] = {
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
#define ICP_HD_FULL_WIDTH                 3264
#if BOARD_VER_G(WS10)
#define ICP_HD_FULL_HEIGHT                2448
#else
#define ICP_HD_FULL_HEIGHT                2449
#endif
#define ICP_HD_QTR_WIDTH                   1280
#define ICP_HD_QTR_HEIGHT                  960

/* 816x612, 24MHz MCLK 96MHz PCLK */
uint32_t ICP_HD_FULL_SIZE_DUMMY_PIXELS   = 0;
uint32_t ICP_HD_FULL_SIZE_DUMMY_LINES    = 0;
uint32_t ICP_HD_FULL_SIZE_WIDTH          = ICP_HD_FULL_WIDTH;
uint32_t ICP_HD_FULL_SIZE_HEIGHT         = ICP_HD_FULL_HEIGHT;

uint32_t ICP_HD_QTR_SIZE_DUMMY_PIXELS    = 0;
uint32_t ICP_HD_QTR_SIZE_DUMMY_LINES     = 0;
uint32_t ICP_HD_QTR_SIZE_WIDTH           = ICP_HD_QTR_WIDTH;
uint32_t ICP_HD_QTR_SIZE_HEIGHT          = ICP_HD_QTR_HEIGHT;

uint32_t ICP_HD_HRZ_FULL_BLK_PIXELS      = 16;
uint32_t ICP_HD_VER_FULL_BLK_LINES       = 12;
uint32_t ICP_HD_HRZ_QTR_BLK_PIXELS       = 16;
uint32_t ICP_HD_VER_QTR_BLK_LINES        = 12;

#else

#define C_PANTECH_CAMERA_MIN_PREVIEW_FPS	5
#define C_PANTECH_CAMERA_MAX_PREVIEW_FPS	31

#define ICP_HD_FULL_SIZE_DUMMY_PIXELS   0
#define ICP_HD_FULL_SIZE_DUMMY_LINES    0
#define ICP_HD_FULL_SIZE_WIDTH          3264//1280//3264
#define ICP_HD_FULL_SIZE_HEIGHT         2448//960//2448

#define ICP_HD_1080P_SIZE_DUMMY_PIXELS	0
#define ICP_HD_1080P_SIZE_DUMMY_LINES	0
#define ICP_HD_1080P_SIZE_WIDTH			1920
#define ICP_HD_1080P_SIZE_HEIGHT		1080

#define ICP_HD_QTR_SIZE_DUMMY_PIXELS    0
#define ICP_HD_QTR_SIZE_DUMMY_LINES     0
#define ICP_HD_QTR_SIZE_WIDTH           1280//640//1280 
#define ICP_HD_QTR_SIZE_HEIGHT          960//480//960 

#define ICP_HD_HRZ_FULL_BLK_PIXELS      16//0
#define ICP_HD_VER_FULL_BLK_LINES       12//0

#define ICP_HD_HRZ_1080P_BLK_PIXELS		16//0
#define ICP_HD_VER_1080P_BLK_LINES		12//0

#define ICP_HD_HRZ_QTR_BLK_PIXELS       16//0
#define ICP_HD_VER_QTR_BLK_LINES        12//0



//#define ICP_HD_HRZ_1080P_BLK_PIXELS      0
//#define ICP_HD_VER_1080P_BLK_LINES        0

//#define ICP_HD_ZSL_SIZE_WIDTH    2560 
//#define ICP_HD_ZSL_SIZE_HEIGHT   1920 

static struct msm_sensor_output_info_t icp_hd_dimensions[] = {
	{
		.x_output = ICP_HD_FULL_SIZE_WIDTH,
		.y_output = ICP_HD_FULL_SIZE_HEIGHT,
		.line_length_pclk = ICP_HD_FULL_SIZE_WIDTH + ICP_HD_HRZ_FULL_BLK_PIXELS ,
		.frame_length_lines = ICP_HD_FULL_SIZE_HEIGHT+ ICP_HD_VER_FULL_BLK_LINES ,
		.vt_pixel_clk = 198000000,//21811200,//35000000,//198000000, //207000000, //276824064, 
		.op_pixel_clk = 198000000,//40000000,//35000000,//198000000, //207000000, //276824064, 
		.binning_factor = 1,
	},
	{
		.x_output = ICP_HD_QTR_SIZE_WIDTH,
		.y_output = ICP_HD_QTR_SIZE_HEIGHT,
		.line_length_pclk = ICP_HD_QTR_SIZE_WIDTH + ICP_HD_HRZ_QTR_BLK_PIXELS,
		.frame_length_lines = ICP_HD_QTR_SIZE_HEIGHT+ ICP_HD_VER_QTR_BLK_LINES,
		.vt_pixel_clk = 198000000,//21811200,//35000000, //207000000, //276824064, 
		.op_pixel_clk = 198000000,//40000000,//35000000, //207000000, //276824064, 
		.binning_factor = 1,//2,
	},
	{
		.x_output = ICP_HD_1080P_SIZE_WIDTH,
		.y_output = ICP_HD_1080P_SIZE_HEIGHT,
		.line_length_pclk = ICP_HD_1080P_SIZE_WIDTH + ICP_HD_HRZ_1080P_BLK_PIXELS,
		.frame_length_lines = ICP_HD_1080P_SIZE_HEIGHT+ ICP_HD_VER_1080P_BLK_LINES,
		.vt_pixel_clk = 198000000,//21811200,//35000000, //207000000, //276824064, 
		.op_pixel_clk = 198000000,//40000000,//35000000, //207000000, //276824064, 
		.binning_factor = 1,
	},
	{
		.x_output = ICP_HD_FULL_SIZE_WIDTH,
		.y_output = ICP_HD_FULL_SIZE_HEIGHT,
		.line_length_pclk = ICP_HD_FULL_SIZE_WIDTH + ICP_HD_HRZ_FULL_BLK_PIXELS ,
		.frame_length_lines = ICP_HD_FULL_SIZE_HEIGHT+ ICP_HD_VER_FULL_BLK_LINES ,
		.vt_pixel_clk = 198000000,//21811200,//35000000, //207000000, //276824064, 
		.op_pixel_clk = 198000000,//40000000,//35000000, //207000000, //276824064, 
		.binning_factor = 1,
	},	
};

#endif
static struct msm_camera_csi_params icp_hd_csic_params = {
	.data_format = CSI_8BIT,//CSI_10BIT,
	.lane_cnt    = 2,//1,//4,
	.lane_assign = 0xe4,
	.dpcm_scheme = 0,
	.settle_cnt  = 0x14,
};

static struct msm_camera_csi_params *icp_hd_csic_params_array[] = {
	&icp_hd_csic_params,
	&icp_hd_csic_params,
	&icp_hd_csic_params,
	&icp_hd_csic_params,	
};

static struct msm_camera_csid_vc_cfg icp_hd_cid_cfg[] = {
	{0, CSI_YUV422_8, CSI_DECODE_8BIT},//CSI_RAW10, CSI_DECODE_10BIT},
	{1, CSI_EMBED_DATA, CSI_DECODE_8BIT},
	{2, CSI_RESERVED_DATA_0, CSI_DECODE_8BIT},
	{3, CSI_RESERVED_DATA_0, CSI_DECODE_8BIT},	
};

static struct msm_camera_csi2_params icp_hd_csi_params = {
	.csid_params = {
		.lane_cnt = 2,//1,//4,
		.lut_params = {
			.num_cid = ARRAY_SIZE(icp_hd_cid_cfg),
			.vc_cfg = icp_hd_cid_cfg,
		},
	},
	.csiphy_params = {
		.lane_cnt = 2,//1,//4,
		.settle_cnt = 0x14,//0x1B,
	},
};

static struct msm_camera_csi2_params *icp_hd_csi_params_array[] = {
	&icp_hd_csi_params,
	&icp_hd_csi_params,
	&icp_hd_csi_params,
	&icp_hd_csi_params,	
};

#if 0 //for build
static struct msm_sensor_output_reg_addr_t icp_hd_reg_addr = {
	.x_output = 0x34C,
	.y_output = 0x34E,
	.line_length_pclk = 0x342,
	.frame_length_lines = 0x340,
};
#endif

static struct msm_sensor_id_info_t icp_hd_id_info = {
	.sensor_id_reg_addr = 0x0000,//0xe002,//0x0000,
	.sensor_id = 0x0,
};

#if 0 //for build
static struct msm_sensor_exp_gain_info_t icp_hd_exp_gain_info = {
	.coarse_int_time_addr = 0x202,
	.global_gain_addr = 0x204,
	.vert_offset = 3,
};
#endif

int32_t icp_hd_sensor_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
    int rc = 0;
#if 0
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

    icp_hd_client = client;

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
    
#if 0//check
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

static const struct i2c_device_id icp_hd_i2c_id[] = {
	{SENSOR_NAME, (kernel_ulong_t)&icp_hd_s_ctrl},
	{ }
};

static struct i2c_driver icp_hd_i2c_driver = {
	.id_table = icp_hd_i2c_id,
    .probe  = icp_hd_sensor_i2c_probe,//msm_sensor_i2c_probe,
        .driver = {
		.name = SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client icp_hd_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,  //fix?
};

#if 1
static int32_t icp_hd_i2c_txdata(unsigned short saddr,
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
	for (i = 0; i < ICP_HD_I2C_RETRY; i++) {
		rc = i2c_transfer(icp_hd_client->adapter, msg, 1); 
		if (rc >= 0) {			
			return 0;
		}
		CDBG("%s: tx retry. [%02x.%02x.%02x] len=%d rc=%d\n", __func__,saddr, *txdata, *(txdata + 1), length, rc);
		msleep(ICP_HD_I2C_MPERIOD);
	}
	return -EIO;
}

static int32_t icp_hd_i2c_write(unsigned short saddr,
	unsigned short waddr, unsigned int dwdata, enum icp_hd_width width)
{
	int32_t rc = -EIO;
	unsigned char buf[6];

	memset(buf, 0, sizeof(buf));
	switch (width) {
	case DWORD_LEN: {
		buf[0] = (waddr & 0xFF00)>>8;
		buf[1] = (waddr & 0x00FF);
		buf[2] = (dwdata & 0xFF000000)>>24;
		buf[3] = (dwdata & 0x00FF0000)>>16;
		buf[4] = (dwdata & 0x0000FF00)>>8;
		buf[5] = (dwdata & 0x000000FF);
#ifdef DEBUG_I2C_VALUE	
		CDBG("DWORD_LEN10: dwdata  = 0x%x\n", dwdata );
		CDBG("DWORD_LEN11: buf[2] = 0x%x, buf[3] = 0x%x, buf[4] = 0x%x, buf[5] = 0x%x\n", buf[2], buf[3], buf[4], buf[5]);
#endif		
		rc = icp_hd_i2c_txdata(saddr, buf, 6);
	}
		break;
	case WORD_LEN: {
		buf[0] = (waddr & 0xFF00)>>8;
		buf[1] = (waddr & 0x00FF);
		buf[2] = (dwdata & 0x0000FF00)>>8;
		buf[3] = (dwdata & 0x000000FF);
		rc = icp_hd_i2c_txdata(saddr, buf, 4);
	}
		break;

	case TRIPLE_LEN: {
		buf[0] = (waddr & 0xFF00)>>8;
		buf[1] = (waddr & 0x00FF);
		buf[2] = (dwdata & 0x000000FF);
		rc = icp_hd_i2c_txdata(saddr, buf, 3);
	}
		break;

	case BYTE_LEN: {
		buf[0] = waddr;
		buf[1] = (dwdata & 0x000000FF);
		rc = icp_hd_i2c_txdata(saddr, buf, 2);
	}
		break;

	default:
		break;
	}

	if (rc < 0)
		pr_err(
		"i2c_write failed, addr = 0x%x, val = 0x%x!\n",
		waddr, dwdata);

	return rc;
}

#ifdef BURST_MODE_64_BYTE
static int32_t icp_hd_i2c_write_a2d1(unsigned short waddr, unsigned char wdata)
{
	int32_t rc = -EIO;
	unsigned char buf[3];

	memset(buf, 0, sizeof(buf));

	buf[0] = (waddr & 0xFF00)>>8;
	buf[1] = (waddr & 0x00FF);
	buf[2] = wdata;
	
	rc = icp_hd_i2c_txdata(icp_hd_client->addr, buf, 3);

	if (rc < 0)
		pr_err(
		"i2c_write failed, saddr= 0x%x, addr = 0x%x, val = 0x%x!\n",
		icp_hd_client->addr, waddr, wdata);

	return rc;
}

static int32_t icp_hd_i2c_write_dw(unsigned short waddr, unsigned int dwdata32)
{
	int32_t rc = -EIO;
	unsigned char buf[6];

	memset(buf, 0, sizeof(buf));
	
	buf[0] = (waddr & 0xFF00)>>8;
	buf[1] = (waddr & 0x00FF);
	buf[2] = (dwdata32 & 0xFF000000)>>24;
	buf[3] = (dwdata32 & 0x00FF0000)>>16;
	buf[4] = (dwdata32 & 0x0000FF00)>>8;
	buf[5] = (dwdata32 & 0x000000FF);

	rc = icp_hd_i2c_txdata(icp_hd_client->addr, buf, 6);
#ifdef DEBUG_I2C_VALUE	
	CDBG("icp_hd_i2c_write_dw10: dwdata32 = 0x%x\n", dwdata32);
	CDBG("icp_hd_i2c_write_dw11: buf[2] = 0x%x, buf[3] = 0x%x, buf[4] = 0x%x, buf[5] = 0x%x\n", buf[2], buf[3], buf[4], buf[5]);
#endif	
	if (rc < 0)
		pr_err("i2c_write failed, addr = 0x%x, dwdata32 = 0x%x\n", waddr, dwdata32);

	return rc;
}
#endif

static int32_t icp_hd_i2c_write_w(unsigned short waddr, unsigned short wdata)
{
	int32_t rc = -EIO;
	unsigned char buf[4];

	memset(buf, 0, sizeof(buf));
	
	buf[0] = (waddr & 0xFF00)>>8;
	buf[1] = (waddr & 0x00FF);
	buf[2] = (wdata & 0xFF00)>>8;
	buf[3] = (wdata & 0x00FF);

	rc = icp_hd_i2c_txdata(icp_hd_client->addr, buf, 4);

	if (rc < 0)
		pr_err("i2c_write failed, addr = 0x%x, dwdata32 = 0x%x\n", waddr, wdata);

	return rc;
}

static int32_t icp_hd_i2c_write_table(
	struct icp_hd_i2c_reg_conf const *reg_conf_tbl,
	int num_of_items_in_table)
{
#if 1
	int i;
	int32_t rc = -EIO;
#if defined(BURST_MODE_64_BYTE)||defined(BURST_MODE_4096_BYTE)||defined(BURST_MODE_64_TO_4096_BYTE)
	int32_t burst_current_pos = 0;
	int32_t burst_current_size = 0;
	int32_t burst_reg_size = 0;
	int32_t icp_hd_start_addr = 0;
	uint8_t * icp_hd_reg_base_addr = 0;
#if defined(BURST_MODE_64_TO_4096_BYTE)
#define BURST_DATA_LEN 2046//4078//4094//60//2046//4094
	uint8_t icp_hd_burst_data[2+BURST_DATA_LEN];
#endif
#endif	

	uint16_t poll_delay = 0;
       uint16_t poll_retry = 0;
       uint16_t poll_mcu_var = 0;
       uint16_t poll_data = 0;
       uint16_t poll_mask = 0;
       uint16_t retry_cnt = 0;
	unsigned int read_data = 0;
	//OTP defense code add
	//uint16_t otp_retry_cnt = 0;
	//uint16_t otp_poll_retry = 20;

	for (i = 0; i < num_of_items_in_table; i++) 
	{		
	switch(reg_conf_tbl->width )
	{
		case ZERO_LEN:
		{
			CDBG("ZERO_LEN continue ADDR = 0x%x, VALUE = 0x%x\n",reg_conf_tbl->waddr, reg_conf_tbl->dwdata);
			reg_conf_tbl++;	
			rc = 0;
			continue;
		}
		case POLL_MCU_VAR:
		{
			poll_mcu_var = reg_conf_tbl->waddr;
   		       poll_mask = reg_conf_tbl->dwdata;	              
	              poll_data = (reg_conf_tbl+1)->waddr;
			poll_delay = ((reg_conf_tbl+1)->dwdata & 0xff00) >> 8;
                     poll_retry = ((reg_conf_tbl+1)->dwdata & 0x00ff);              

		       CDBG("POLLING!! poll_delay=%x, poll_retry=%x, poll_mcu_var=%x, poll_data=%x, poll_mask=%x\n",poll_delay, poll_retry, poll_mcu_var, poll_data, poll_mask);
				  
			for (retry_cnt = 0; retry_cnt < poll_retry; retry_cnt++)
                	{
			            rc = icp_hd_i2c_read(icp_hd_client->addr, poll_mcu_var, &read_data, WORD_LEN);
	                    if (rc < 0)
	                    {
	                        pr_err("<<POLL_MCU_VAR icp_hd_i2c_read_word (FALSE)\n");
	                        return -EIO;
	                    }
	                    
	                    if ((read_data & poll_mask) != poll_data)
	                    {
	                        CDBG("retry polling MCU variable... after sleeping %d ms, read_data=%2x\n", poll_delay, read_data);
	                        msleep(poll_delay);
	                    }
	                    else
	                    {
	                        CDBG("stop polling MCU variable... retried %d/%d time(s) (delay = %d ms), read_data=%2x\n", retry_cnt, poll_retry, poll_delay, read_data);
	                        break;
	                    }
			}

			if (retry_cnt == poll_retry)
	              {
	                     pr_err("<<RETRY FAIL!! poll_mcu_var = %x, read_data = %x (FALSE)\n", poll_mcu_var, read_data);
#ifdef ICP_HD_MODULE_ABNORMAL_OPERATION_DEFENCE			      
				if((poll_mcu_var == 0x6002) && (poll_data == 0xffff))
				return ICP_HD_REG_POLLING_ERROR;
				else
					return 0;
#else
				return 0;
#endif
	              }

			//  2���� ���� �̿��ϹǷ� +2�� ���ش�
			reg_conf_tbl++;
			reg_conf_tbl++;
			i++;

			break;
		}
#ifdef BURST_MODE_64_BYTE
		case BURST_WORD_1:
			burst_current_pos = 0;
 			burst_current_size = 0;
			burst_reg_size = icp_hd_tune_regs.init_burst_settings_size_1;
			icp_hd_reg_base_addr = icp_hd_tune_regs.init_burst_settings_1;
			icp_hd_start_addr = 0x0000644C;		
			CDBG("BURST_WORD_1 START~~~ icp_hd_start_addr= 0x%x, icp_hd_reg_base_addr = 0x%x, burst_reg_size = 0x%x\n", icp_hd_start_addr, icp_hd_reg_base_addr, burst_reg_size);
							
			while(burst_current_pos < burst_reg_size){
				burst_current_size = ((burst_reg_size - burst_current_pos)>=62 )? 62 : (burst_reg_size - burst_current_pos);
#ifdef DEBUG_I2C_VALUE		
#if 0
				CDBG("BURST_WORD_10~~ icp_hd_start_addr= 0x%x, icp_hd_reg_base_addr = 0x%x, burst_current_size\n", icp_hd_start_addr, icp_hd_reg_base_addr);
				CDBG("BURST_WORD_11~~ burst_current_size= 0x%x, burst_current_pos = 0x%x, burst_reg_size = 0x%x\n", burst_current_size, burst_current_pos, burst_reg_size);
#else
				CDBG("BURST_1 ADDR~~ icp_hd_start_addr= 0x%x, icp_hd_reg_base_addr = 0x%x, burst_current_size = %d\n", icp_hd_start_addr, icp_hd_reg_base_addr, burst_current_size);
				CDBG("BURST_1 DATA~~\n");
				CDBG("%2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x\n", *icp_hd_reg_base_addr, *(icp_hd_reg_base_addr+1), *(icp_hd_reg_base_addr+2),*(icp_hd_reg_base_addr+3),*(icp_hd_reg_base_addr+4),*(icp_hd_reg_base_addr+5),*(icp_hd_reg_base_addr+6),*(icp_hd_reg_base_addr+7),*(icp_hd_reg_base_addr+8),*(icp_hd_reg_base_addr+9),*(icp_hd_reg_base_addr+10),*(icp_hd_reg_base_addr+11),*(icp_hd_reg_base_addr+12),*(icp_hd_reg_base_addr+13));
				CDBG("%2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x\n", *(icp_hd_reg_base_addr+14), *(icp_hd_reg_base_addr+15), *(icp_hd_reg_base_addr+16),*(icp_hd_reg_base_addr+17),*(icp_hd_reg_base_addr+18),*(icp_hd_reg_base_addr+19),*(icp_hd_reg_base_addr+20),*(icp_hd_reg_base_addr+21),*(icp_hd_reg_base_addr+22),*(icp_hd_reg_base_addr+23),*(icp_hd_reg_base_addr+24),*(icp_hd_reg_base_addr+25),*(icp_hd_reg_base_addr+26),*(icp_hd_reg_base_addr+27),*(icp_hd_reg_base_addr+28),*(icp_hd_reg_base_addr+29));
				CDBG("%2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x\n", *(icp_hd_reg_base_addr+30), *(icp_hd_reg_base_addr+31), *(icp_hd_reg_base_addr+32),*(icp_hd_reg_base_addr+33),*(icp_hd_reg_base_addr+34),*(icp_hd_reg_base_addr+35),*(icp_hd_reg_base_addr+36),*(icp_hd_reg_base_addr+37),*(icp_hd_reg_base_addr+38),*(icp_hd_reg_base_addr+39),*(icp_hd_reg_base_addr+40),*(icp_hd_reg_base_addr+41),*(icp_hd_reg_base_addr+42),*(icp_hd_reg_base_addr+43),*(icp_hd_reg_base_addr+44),*(icp_hd_reg_base_addr+45));
				CDBG("%2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x\n", *(icp_hd_reg_base_addr+46), *(icp_hd_reg_base_addr+47), *(icp_hd_reg_base_addr+48),*(icp_hd_reg_base_addr+49),*(icp_hd_reg_base_addr+50),*(icp_hd_reg_base_addr+51),*(icp_hd_reg_base_addr+52),*(icp_hd_reg_base_addr+53),*(icp_hd_reg_base_addr+54),*(icp_hd_reg_base_addr+55),*(icp_hd_reg_base_addr+56),*(icp_hd_reg_base_addr+57),*(icp_hd_reg_base_addr+58),*(icp_hd_reg_base_addr+59),*(icp_hd_reg_base_addr+60),*(icp_hd_reg_base_addr+61));				
#endif				
#endif				
				rc = icp_hd_i2c_write_dw(0xF03C, icp_hd_start_addr);
				rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_reg_base_addr, burst_current_size);			
				if (rc < 0)
				{
					pr_err("icp_hd_i2c_bust_1 failed!\n");
					break;
				}
				burst_current_pos += burst_current_size;
				icp_hd_start_addr = icp_hd_start_addr + burst_current_size - 2;
				icp_hd_reg_base_addr += burst_current_size;				
			}
			
			CDBG("BURST_WORD_1 END!!! icp_hd_start_addr= 0x%x, icp_hd_reg_base_addr = 0x%x, burst_reg_size = 0x%x, burst_current_pos = 0x%x\n", 
				icp_hd_start_addr, icp_hd_reg_base_addr, burst_reg_size, burst_current_pos);
			rc = icp_hd_i2c_write_w(0x601A, 0x0002);
			reg_conf_tbl++;
			//msleep(50);
			
			break;
		case BURST_WORD_2:
			burst_current_pos = 0;
 			burst_current_size = 0;
			burst_reg_size = icp_hd_tune_regs.init_burst_settings_size_2;
			icp_hd_reg_base_addr = icp_hd_tune_regs.init_burst_settings_2;
			icp_hd_start_addr = 0x0000DC40;
							
			while(burst_current_pos < burst_reg_size){
				burst_current_size = ((burst_reg_size - burst_current_pos)>=62 )? 62 : (burst_reg_size - burst_current_pos);
				rc = icp_hd_i2c_write_dw(0xF03C, icp_hd_start_addr);
				rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_reg_base_addr, burst_current_size);			
				if (rc < 0)
				{
					pr_err("icp_hd_i2c_bust_2 failed!\n");
					break;
				}

				burst_current_pos += burst_current_size;
				icp_hd_start_addr = icp_hd_start_addr + burst_current_size - 2;
				icp_hd_reg_base_addr += burst_current_size;				
			}
			CDBG("BURST_WORD_2 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_2 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_2);
			rc = icp_hd_i2c_write_w(0x601A, 0x0010);			
			reg_conf_tbl++;
			//msleep(50);
			
			break;
		case BURST_WORD_3:
			burst_current_pos = 0;
 			burst_current_size = 0;
			burst_reg_size = icp_hd_tune_regs.init_burst_settings_size_3;
			icp_hd_reg_base_addr = icp_hd_tune_regs.init_burst_settings_3;
			icp_hd_start_addr = 0x00006BAC;
							
			while(burst_current_pos < burst_reg_size){
				burst_current_size = ((burst_reg_size - burst_current_pos)>=62 )? 62 : (burst_reg_size - burst_current_pos);
				rc = icp_hd_i2c_write_dw(0xF03C, icp_hd_start_addr);
				rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_reg_base_addr, burst_current_size);			
				if (rc < 0)
				{
					pr_err("icp_hd_i2c_bust_3 failed!\n");
					break;
				}

				burst_current_pos += burst_current_size;
				icp_hd_start_addr = icp_hd_start_addr + burst_current_size - 2;
				icp_hd_reg_base_addr += burst_current_size;				
			}
			CDBG("BURST_WORD_3 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_3 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_3);
			rc = icp_hd_i2c_write_w(0x601A, 0x0002);
			reg_conf_tbl++;
			//msleep(50);
			
			break;
		case BURST_WORD_4:
			burst_current_pos = 0;
 			burst_current_size = 0;
			burst_reg_size = icp_hd_tune_regs.init_burst_settings_size_4;
			icp_hd_reg_base_addr = icp_hd_tune_regs.init_burst_settings_4;
			icp_hd_start_addr = 0x0000E0DC;
							
			while(burst_current_pos < burst_reg_size){
				burst_current_size = ((burst_reg_size - burst_current_pos)>=62 )? 62 : (burst_reg_size - burst_current_pos);
				rc = icp_hd_i2c_write_dw(0xF03C, icp_hd_start_addr);
				rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_reg_base_addr, burst_current_size);			
				if (rc < 0)
				{
					pr_err("icp_hd_i2c_bust_4 failed!\n");
					break;
				}

				burst_current_pos += burst_current_size;
				icp_hd_start_addr = icp_hd_start_addr + burst_current_size - 2;
				icp_hd_reg_base_addr += burst_current_size;				
			}
			CDBG("BURST_WORD_4 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_4 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_4);			
			reg_conf_tbl++;			
			
			break;
		case BURST_WORD_5:
			burst_current_pos = 0;
 			burst_current_size = 0;
			burst_reg_size = icp_hd_tune_regs.init_burst_settings_size_5;
			icp_hd_reg_base_addr = icp_hd_tune_regs.init_burst_settings_5;
			icp_hd_start_addr = 0x0000E574;
							
			while(burst_current_pos < burst_reg_size){
				burst_current_size = ((burst_reg_size - burst_current_pos)>=62 )? 62 : (burst_reg_size - burst_current_pos);
				rc = icp_hd_i2c_write_dw(0xF03C, icp_hd_start_addr);
				rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_reg_base_addr, burst_current_size);			
				if (rc < 0)
				{
					pr_err("icp_hd_i2c_bust_5 failed!\n");
					break;
				}

				burst_current_pos += burst_current_size;
				icp_hd_start_addr = icp_hd_start_addr + burst_current_size - 2;
				icp_hd_reg_base_addr += burst_current_size;				
			}
			CDBG("BURST_WORD_5 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_5 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_5);			
			reg_conf_tbl++;			
			
			break;
#elif defined(BURST_MODE_64_TO_4096_BYTE)
		case BURST_WORD_1:
			burst_current_pos = 0;
 			burst_current_size = 0;
			burst_reg_size = icp_hd_tune_regs.init_burst_settings_size_1;
			icp_hd_reg_base_addr = icp_hd_tune_regs.init_burst_settings_1;
			icp_hd_start_addr = 0x0000644C;		

			icp_hd_burst_data[0] = 0xE0;
			icp_hd_burst_data[1] = 0x00;			
			CDBG("BURST_WORD_1 START~~~ icp_hd_start_addr= 0x%x, icp_hd_reg_base_addr = 0x%x, burst_reg_size = 0x%x\n", icp_hd_start_addr, icp_hd_reg_base_addr, burst_reg_size);
							
			while(burst_current_pos < burst_reg_size){
				burst_current_size = ((burst_reg_size - burst_current_pos)>=BURST_DATA_LEN )? BURST_DATA_LEN : (burst_reg_size - burst_current_pos);
				CDBG("MEMCPY~ START~~~ &icp_hd_reg_base_addr[burst_current_pos]= 0x%x, burst_current_size = 0x%x\n", &icp_hd_reg_base_addr[burst_current_pos] , burst_current_size);				
				memcpy(&icp_hd_burst_data[2], &icp_hd_reg_base_addr[burst_current_pos] , burst_current_size);
				CDBG("MEMCPY~ END~~~ &icp_hd_reg_base_addr[burst_current_pos]= 0x%x, burst_current_size = 0x%x\n", &icp_hd_reg_base_addr[burst_current_pos] , burst_current_size);
#ifdef DEBUG_I2C_VALUE		
#if 0
				CDBG("BURST_WORD_10~~ icp_hd_start_addr= 0x%x, icp_hd_reg_base_addr = 0x%x, burst_current_size\n", icp_hd_start_addr, icp_hd_reg_base_addr);
				CDBG("BURST_WORD_11~~ burst_current_size= 0x%x, burst_current_pos = 0x%x, burst_reg_size = 0x%x\n", burst_current_size, burst_current_pos, burst_reg_size);
#else
				CDBG("BURST_1 ADDR~~ icp_hd_start_addr= 0x%x, icp_hd_reg_base_addr = 0x%x, burst_current_size = %d\n", icp_hd_start_addr, icp_hd_reg_base_addr, burst_current_size);
				CDBG("BURST_1 DATA~~\n");
				CDBG("%2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x\n", *icp_hd_burst_data, *(icp_hd_burst_data+1), *(icp_hd_burst_data+2),*(icp_hd_burst_data+3),*(icp_hd_burst_data+4),*(icp_hd_burst_data+5),*(icp_hd_burst_data+6),*(icp_hd_burst_data+7),*(icp_hd_burst_data+8),*(icp_hd_burst_data+9),*(icp_hd_burst_data+10),*(icp_hd_burst_data+11),*(icp_hd_burst_data+12),*(icp_hd_burst_data+13));
				CDBG("%2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x\n", *(icp_hd_burst_data+14), *(icp_hd_burst_data+15), *(icp_hd_burst_data+16),*(icp_hd_burst_data+17),*(icp_hd_burst_data+18),*(icp_hd_burst_data+19),*(icp_hd_burst_data+20),*(icp_hd_burst_data+21),*(icp_hd_burst_data+22),*(icp_hd_burst_data+23),*(icp_hd_burst_data+24),*(icp_hd_burst_data+25),*(icp_hd_burst_data+26),*(icp_hd_burst_data+27),*(icp_hd_burst_data+28),*(icp_hd_burst_data+29));
				CDBG("%2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x\n", *(icp_hd_burst_data+30), *(icp_hd_burst_data+31), *(icp_hd_burst_data+32),*(icp_hd_burst_data+33),*(icp_hd_burst_data+34),*(icp_hd_burst_data+35),*(icp_hd_burst_data+36),*(icp_hd_burst_data+37),*(icp_hd_burst_data+38),*(icp_hd_burst_data+39),*(icp_hd_burst_data+40),*(icp_hd_burst_data+41),*(icp_hd_burst_data+42),*(icp_hd_burst_data+43),*(icp_hd_burst_data+44),*(icp_hd_burst_data+45));
				CDBG("%2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x\n", *(icp_hd_burst_data+46), *(icp_hd_burst_data+47), *(icp_hd_burst_data+48),*(icp_hd_burst_data+49),*(icp_hd_burst_data+50),*(icp_hd_burst_data+51),*(icp_hd_burst_data+52),*(icp_hd_burst_data+53),*(icp_hd_burst_data+54),*(icp_hd_burst_data+55),*(icp_hd_burst_data+56),*(icp_hd_burst_data+57),*(icp_hd_burst_data+58),*(icp_hd_burst_data+59),*(icp_hd_burst_data+60),*(icp_hd_burst_data+61));				
#endif				
#endif							
				rc = icp_hd_i2c_write_dw(0xF03C, icp_hd_start_addr);
				rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_burst_data, burst_current_size+2);			
				if (rc < 0)
				{
					pr_err("icp_hd_i2c_burst_1 failed!\n");
					break;
				}
				burst_current_pos += burst_current_size;
				icp_hd_start_addr += burst_current_size;							
			}
			
			CDBG("BURST_WORD_1 END!!! icp_hd_start_addr= 0x%x, icp_hd_reg_base_addr = 0x%x, burst_reg_size = 0x%x, burst_current_pos = 0x%x\n", 
				icp_hd_start_addr, icp_hd_reg_base_addr, burst_reg_size, burst_current_pos);
			rc = icp_hd_i2c_write_w(0x601A, 0x0002);
			reg_conf_tbl++;
			//msleep(50);
			
			break;
		case BURST_WORD_2:
			burst_current_pos = 0;
 			burst_current_size = 0;
			burst_reg_size = icp_hd_tune_regs.init_burst_settings_size_2;
			icp_hd_reg_base_addr = icp_hd_tune_regs.init_burst_settings_2;
			icp_hd_start_addr = 0x0000DC40;

			icp_hd_burst_data[0] = 0xE0;
			icp_hd_burst_data[1] = 0x00;
			while(burst_current_pos < burst_reg_size){
				burst_current_size = ((burst_reg_size - burst_current_pos)>=BURST_DATA_LEN )? BURST_DATA_LEN : (burst_reg_size - burst_current_pos);
				memcpy(&icp_hd_burst_data[2], &icp_hd_reg_base_addr[burst_current_pos] , burst_current_size);
				rc = icp_hd_i2c_write_dw(0xF03C, icp_hd_start_addr);
				rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_burst_data, burst_current_size+2);			
				if (rc < 0)
				{
					pr_err("icp_hd_i2c_burst_2 failed!\n");
					break;
				}
				burst_current_pos += burst_current_size;
				icp_hd_start_addr += burst_current_size;		
			}
			CDBG("BURST_WORD_2 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_2 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_2);
			rc = icp_hd_i2c_write_w(0x601A, 0x0010);			
			reg_conf_tbl++;
			//msleep(50);
			
			break;
		case BURST_WORD_3:
			burst_current_pos = 0;
 			burst_current_size = 0;
			burst_reg_size = icp_hd_tune_regs.init_burst_settings_size_3;
			icp_hd_reg_base_addr = icp_hd_tune_regs.init_burst_settings_3;
			icp_hd_start_addr = 0x00006BAC;

			icp_hd_burst_data[0] = 0xE0;
			icp_hd_burst_data[1] = 0x00;
			while(burst_current_pos < burst_reg_size){
				burst_current_size = ((burst_reg_size - burst_current_pos)>=BURST_DATA_LEN )? BURST_DATA_LEN : (burst_reg_size - burst_current_pos);
				memcpy(&icp_hd_burst_data[2], &icp_hd_reg_base_addr[burst_current_pos] , burst_current_size);
				rc = icp_hd_i2c_write_dw(0xF03C, icp_hd_start_addr);
				rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_burst_data, burst_current_size+2);			
				if (rc < 0)
				{
					pr_err("icp_hd_i2c_burst_3 failed!\n");
					break;
				}
				burst_current_pos += burst_current_size;
				icp_hd_start_addr += burst_current_size;			
			}
			CDBG("BURST_WORD_3 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_3 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_3);
			rc = icp_hd_i2c_write_w(0x601A, 0x0002);
			reg_conf_tbl++;
			//msleep(50);
			
			break;
		case BURST_WORD_4:
			burst_current_pos = 0;
 			burst_current_size = 0;
			burst_reg_size = icp_hd_tune_regs.init_burst_settings_size_4;
			icp_hd_reg_base_addr = icp_hd_tune_regs.init_burst_settings_4;
			icp_hd_start_addr = 0x0000E0DC;

			icp_hd_burst_data[0] = 0xE0;
			icp_hd_burst_data[1] = 0x00;							
			while(burst_current_pos < burst_reg_size){
				burst_current_size = ((burst_reg_size - burst_current_pos)>=BURST_DATA_LEN )? BURST_DATA_LEN : (burst_reg_size - burst_current_pos);
				memcpy(&icp_hd_burst_data[2], &icp_hd_reg_base_addr[burst_current_pos] , burst_current_size);
				rc = icp_hd_i2c_write_dw(0xF03C, icp_hd_start_addr);
				rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_burst_data, burst_current_size+2);			
				if (rc < 0)
				{
					pr_err("icp_hd_i2c_burst_4 failed!\n");
					break;
				}
				burst_current_pos += burst_current_size;
				icp_hd_start_addr += burst_current_size;			
			}
			CDBG("BURST_WORD_4 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_4 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_4);			
			reg_conf_tbl++;			
			
			break;
		case BURST_WORD_5:
			burst_current_pos = 0;
 			burst_current_size = 0;
			burst_reg_size = icp_hd_tune_regs.init_burst_settings_size_5;
			icp_hd_reg_base_addr = icp_hd_tune_regs.init_burst_settings_5;
			icp_hd_start_addr = 0x0000E574;
							
			icp_hd_burst_data[0] = 0xE0;
			icp_hd_burst_data[1] = 0x00;
			while(burst_current_pos < burst_reg_size){
				burst_current_size = ((burst_reg_size - burst_current_pos)>=BURST_DATA_LEN )? BURST_DATA_LEN : (burst_reg_size - burst_current_pos);
				memcpy(&icp_hd_burst_data[2], &icp_hd_reg_base_addr[burst_current_pos] , burst_current_size);
				rc = icp_hd_i2c_write_dw(0xF03C, icp_hd_start_addr);
				rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_burst_data, burst_current_size+2);			
				if (rc < 0)
				{
					pr_err("icp_hd_i2c_burst_5 failed!\n");
					break;
				}
				burst_current_pos += burst_current_size;
				icp_hd_start_addr += burst_current_size;				
			}
			CDBG("BURST_WORD_5 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_5 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_5);			
			reg_conf_tbl++;			
			
			break;
#elif defined(BURST_MODE_4096_BYTE_CAPTURE_12FPS)
		case BURST_WORD_1:
			rc = icp_hd_i2c_write_dw(0xF03C, 0x0000644C);
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_1, icp_hd_tune_regs.init_burst_settings_size_1);			
			if (rc < 0)
			{
				pr_err("icp_hd_i2c_bust_1 failed!\n");
				break;
			}
			CDBG("BURST_WORD_1 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_1 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_1);
			rc = icp_hd_i2c_write_w(0x601A, 0x0002);
			reg_conf_tbl++;	
			break;
		case BURST_WORD_2:
			rc = icp_hd_i2c_write_dw(0xF03C, 0x0000E940);
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_2, icp_hd_tune_regs.init_burst_settings_size_2);
			if (rc < 0)
			{
				pr_err("icp_hd_i2c_bust_2 failed!\n");
				break;
			}
			CDBG("BURST_WORD_2 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_2 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_2);
			rc = icp_hd_i2c_write_w(0x601A, 0x0010);			
			reg_conf_tbl++;			
			break;
		case BURST_WORD_3:
			rc = icp_hd_i2c_write_dw(0xF03C, 0x00006B38);
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_3, icp_hd_tune_regs.init_burst_settings_size_3);
			if (rc < 0)
			{
				pr_err("icp_hd_i2c_bust_3 failed!\n");
				break;
			}
			CDBG("BURST_WORD_3 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_3 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_3);
			reg_conf_tbl++;
			break;
		case BURST_WORD_4:
			rc = icp_hd_i2c_write_dw(0xF03C, 0x00007B36);
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_4, icp_hd_tune_regs.init_burst_settings_size_4);
			if (rc < 0)
			{
				pr_err("icp_hd_i2c_bust_4 failed!\n");
				break;
			}
			CDBG("BURST_WORD_4 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_4 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_4);
			reg_conf_tbl++;
			break;
		case BURST_WORD_5:
			rc = icp_hd_i2c_write_dw(0xF03C, 0x00008B34);
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_5, icp_hd_tune_regs.init_burst_settings_size_5);
			if (rc < 0)
			{
				pr_err("icp_hd_i2c_bust_5 failed!\n");
				break;
			}
			CDBG("BURST_WORD_5 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_5 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_5);
			reg_conf_tbl++;
			break;
		case BURST_WORD_6:
			rc = icp_hd_i2c_write_dw(0xF03C, 0x00009B32);
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_6, icp_hd_tune_regs.init_burst_settings_size_6);
			if (rc < 0)
			{
				pr_err("icp_hd_i2c_bust_6 failed!\n");
				break;
			}
			CDBG("BURST_WORD_6 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_6 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_6);
			reg_conf_tbl++;
			break;
		case BURST_WORD_7:
			rc = icp_hd_i2c_write_dw(0xF03C, 0x0000AB30);
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_7, icp_hd_tune_regs.init_burst_settings_size_7);
			if (rc < 0)
			{
				pr_err("icp_hd_i2c_bust_7 failed!\n");
				break;
			}
			CDBG("BURST_WORD_7 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_7 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_7);
			reg_conf_tbl++;
			break;
		case BURST_WORD_8:
			rc = icp_hd_i2c_write_dw(0xF03C, 0x0000BB2E);
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_8, icp_hd_tune_regs.init_burst_settings_size_8);
			if (rc < 0)
			{
				pr_err("icp_hd_i2c_bust_8 failed!\n");
				break;
			}
			CDBG("BURST_WORD_8 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_8 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_8);
			reg_conf_tbl++;
			break;
		case BURST_WORD_9:
			rc = icp_hd_i2c_write_dw(0xF03C, 0x0000CB2C);
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_9, icp_hd_tune_regs.init_burst_settings_size_9);
			if (rc < 0)
			{
				pr_err("icp_hd_i2c_bust_9 failed!\n");
				break;
			}
			CDBG("BURST_WORD_9 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_9 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_9);
			reg_conf_tbl++;
			break;
		case BURST_WORD_10:
			rc = icp_hd_i2c_write_dw(0xF03C, 0x0000DB2A);
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_10, icp_hd_tune_regs.init_burst_settings_size_10);
			if (rc < 0)
			{
				pr_err("icp_hd_i2c_bust_10 failed!\n");
				break;
			}
			CDBG("BURST_WORD_10 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_10 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_10);
			rc = icp_hd_i2c_write_w(0x601A, 0x0002);
			reg_conf_tbl++;
			break;
		case BURST_WORD_11:
			rc = icp_hd_i2c_write_dw(0xF03C, 0x0000EE54);
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_11, icp_hd_tune_regs.init_burst_settings_size_11);
			if (rc < 0)
			{
				pr_err("icp_hd_i2c_bust_11 failed!\n");
				break;
			}
			CDBG("BURST_WORD_11 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_11 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_11);
			reg_conf_tbl++;
			break;
		case BURST_WORD_12:
			rc = icp_hd_i2c_write_dw(0xF03C, 0x0000F2EC);
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_12, icp_hd_tune_regs.init_burst_settings_size_12);
			if (rc < 0)
			{
				pr_err("icp_hd_i2c_bust_12 failed!\n");
				break;
			}
			CDBG("BURST_WORD_12 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_12 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_12);
			reg_conf_tbl++;
			break;
#elif defined(BURST_MODE_4096_BYTE_CAPTURE_10FPS)
		case BURST_WORD_1:
			rc = icp_hd_i2c_write_dw(0xF03C, 0x0000644C);
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_1, icp_hd_tune_regs.init_burst_settings_size_1);			
			if (rc < 0)
			{
				pr_err("icp_hd_i2c_bust_1 failed!\n");
				break;
			}
			CDBG("BURST_WORD_1 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_1 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_1);
			rc = icp_hd_i2c_write_w(0x601A, 0x0002);
			reg_conf_tbl++;	
			break;
		case BURST_WORD_2:
			rc = icp_hd_i2c_write_dw(0xF03C, 0x0000ECC8);
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_2, icp_hd_tune_regs.init_burst_settings_size_2);
			if (rc < 0)
			{
				pr_err("icp_hd_i2c_bust_2 failed!\n");
				break;
			}
			CDBG("BURST_WORD_2 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_2 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_2);
			rc = icp_hd_i2c_write_w(0x601A, 0x0010);			
			reg_conf_tbl++;			
			break;
		case BURST_WORD_3:
			rc = icp_hd_i2c_write_dw(0xF03C, 0x00006B6C);
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_3, icp_hd_tune_regs.init_burst_settings_size_3);
			if (rc < 0)
			{
				pr_err("icp_hd_i2c_bust_3 failed!\n");
				break;
			}
			CDBG("BURST_WORD_3 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_3 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_3);
			reg_conf_tbl++;
			break;
		case BURST_WORD_4:
			rc = icp_hd_i2c_write_dw(0xF03C, 0x00007B6A);
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_4, icp_hd_tune_regs.init_burst_settings_size_4);
			if (rc < 0)
			{
				pr_err("icp_hd_i2c_bust_4 failed!\n");
				break;
			}
			CDBG("BURST_WORD_4 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_4 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_4);
			reg_conf_tbl++;
			break;
		case BURST_WORD_5:
			rc = icp_hd_i2c_write_dw(0xF03C, 0x00008B68);
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_5, icp_hd_tune_regs.init_burst_settings_size_5);
			if (rc < 0)
			{
				pr_err("icp_hd_i2c_bust_5 failed!\n");
				break;
			}
			CDBG("BURST_WORD_5 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_5 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_5);
			reg_conf_tbl++;
			break;
		case BURST_WORD_6:
			rc = icp_hd_i2c_write_dw(0xF03C, 0x00009B66);
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_6, icp_hd_tune_regs.init_burst_settings_size_6);
			if (rc < 0)
			{
				pr_err("icp_hd_i2c_bust_6 failed!\n");
				break;
			}
			CDBG("BURST_WORD_6 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_6 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_6);
			reg_conf_tbl++;
			break;
		case BURST_WORD_7:
			rc = icp_hd_i2c_write_dw(0xF03C, 0x0000AB64);
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_7, icp_hd_tune_regs.init_burst_settings_size_7);
			if (rc < 0)
			{
				pr_err("icp_hd_i2c_bust_7 failed!\n");
				break;
			}
			CDBG("BURST_WORD_7 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_7 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_7);
			reg_conf_tbl++;
			break;
		case BURST_WORD_8:
			rc = icp_hd_i2c_write_dw(0xF03C, 0x0000BB62);
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_8, icp_hd_tune_regs.init_burst_settings_size_8);
			if (rc < 0)
			{
				pr_err("icp_hd_i2c_bust_8 failed!\n");
				break;
			}
			CDBG("BURST_WORD_8 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_8 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_8);
			reg_conf_tbl++;
			break;
		case BURST_WORD_9:
			rc = icp_hd_i2c_write_dw(0xF03C, 0x0000CB60);
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_9, icp_hd_tune_regs.init_burst_settings_size_9);
			if (rc < 0)
			{
				pr_err("icp_hd_i2c_bust_9 failed!\n");
				break;
			}
			CDBG("BURST_WORD_9 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_9 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_9);
			reg_conf_tbl++;
			break;
		case BURST_WORD_10:
			rc = icp_hd_i2c_write_dw(0xF03C, 0x0000DB5E);
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_10, icp_hd_tune_regs.init_burst_settings_size_10);
			if (rc < 0)
			{
				pr_err("icp_hd_i2c_bust_10 failed!\n");
				break;
			}
			CDBG("BURST_WORD_10 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_10 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_10);			
			reg_conf_tbl++;
			break;
		case BURST_WORD_11:
			rc = icp_hd_i2c_write_dw(0xF03C, 0x0000EB5C);
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_11, icp_hd_tune_regs.init_burst_settings_size_11);
			if (rc < 0)
			{
				pr_err("icp_hd_i2c_bust_11 failed!\n");
				break;
			}			
			CDBG("BURST_WORD_11 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_11 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_11);
			rc = icp_hd_i2c_write_w(0x601A, 0x0002);
			reg_conf_tbl++;
			break;
		case BURST_WORD_12:
			rc = icp_hd_i2c_write_dw(0xF03C, 0x0000F1DC);
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_12, icp_hd_tune_regs.init_burst_settings_size_12);
			if (rc < 0)
			{
				pr_err("icp_hd_i2c_bust_12 failed!\n");
				break;
			}
			CDBG("BURST_WORD_12 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_12 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_12);
			reg_conf_tbl++;
			break;
		case BURST_WORD_13:
			rc = icp_hd_i2c_write_dw(0xF03C, 0x0000F674);
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_13, icp_hd_tune_regs.init_burst_settings_size_13);
			if (rc < 0)
			{
				pr_err("icp_hd_i2c_bust_13 failed!\n");
				break;
			}
			CDBG("BURST_WORD_13 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_13 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_13);
			reg_conf_tbl++;
			break;
#elif defined(BURST_MODE_4096_BYTE_CAPTURE_11FPS)
		case BURST_WORD_1:			
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_1, icp_hd_tune_regs.init_burst_settings_size_1);			
			if (rc < 0)
			{
				pr_err("icp_hd_i2c_bust_1 failed!\n");
				break;
			}
			CDBG("BURST_WORD_1 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_1 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_1);
			reg_conf_tbl++;	
			break;
		case BURST_WORD_2:			
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_2, icp_hd_tune_regs.init_burst_settings_size_2);
			if (rc < 0)
			{
				pr_err("icp_hd_i2c_bust_2 failed!\n");
				break;
			}
			CDBG("BURST_WORD_2 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_2 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_2);
			rc = icp_hd_i2c_write_dw(0x9FFC, 0x19BE7D0E);
			reg_conf_tbl++;			
			break;
		case BURST_WORD_3:			
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_3, icp_hd_tune_regs.init_burst_settings_size_3);
			if (rc < 0)
			{
				pr_err("icp_hd_i2c_bust_3 failed!\n");
				break;
			}
			CDBG("BURST_WORD_3 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_3 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_3);
			reg_conf_tbl++;
			break;
		case BURST_WORD_4:			
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_4, icp_hd_tune_regs.init_burst_settings_size_4);
			if (rc < 0)
			{
				pr_err("icp_hd_i2c_bust_4 failed!\n");
				break;
			}
			CDBG("BURST_WORD_4 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_4 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_4);
			rc = icp_hd_i2c_write_dw(0x9FFC, 0x5F0B0F54);
			reg_conf_tbl++;
			break;
		case BURST_WORD_5:
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_5, icp_hd_tune_regs.init_burst_settings_size_5);
			if (rc < 0)
			{
				pr_err("icp_hd_i2c_bust_5 failed!\n");
				break;
			}
			CDBG("BURST_WORD_5 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_5 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_5);
			reg_conf_tbl++;
			break;
		case BURST_WORD_6:
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_6, icp_hd_tune_regs.init_burst_settings_size_6);
			if (rc < 0)
			{
				pr_err("icp_hd_i2c_bust_6 failed!\n");
				break;
			}
			CDBG("BURST_WORD_6 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_6 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_6);
			rc = icp_hd_i2c_write_dw(0x9FFC, 0x35B13B78);
			reg_conf_tbl++;
			break;
		case BURST_WORD_7:
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_7, icp_hd_tune_regs.init_burst_settings_size_7);
			if (rc < 0)
			{
				pr_err("icp_hd_i2c_bust_7 failed!\n");
				break;
			}
			CDBG("BURST_WORD_7 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_7 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_7);
			rc = icp_hd_i2c_write_w(0x6002, 0xFFFF);
			rc = icp_hd_i2c_write_dw(0x2028, 0x000186A0);
			reg_conf_tbl++;
			break;		
#elif defined(BURST_MODE_8192_BYTE_CAPTURE_11FPS)		
			case BURST_WORD_1:			
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_1, icp_hd_tune_regs.init_burst_settings_size_1);			
			if (rc < 0)
			{
				pr_err("icp_hd_i2c_bust_1 failed!\n");
				break;
			}
			CDBG("BURST_WORD_1 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_1 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_1);
			rc = icp_hd_i2c_write_w(0x9FFE, 0x7D0E);
			reg_conf_tbl++;	
			break;
		case BURST_WORD_2:			
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_2, icp_hd_tune_regs.init_burst_settings_size_2);
			if (rc < 0)
			{
				pr_err("icp_hd_i2c_bust_2 failed!\n");
				break;
			}
			CDBG("BURST_WORD_2 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_2 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_2);
			rc = icp_hd_i2c_write_w(0x9FFE, 0x0F54);
			reg_conf_tbl++;			
			break;
		case BURST_WORD_3:			
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_3, icp_hd_tune_regs.init_burst_settings_size_3);
			if (rc < 0)
			{
				pr_err("icp_hd_i2c_bust_3 failed!\n");
				break;
			}
			CDBG("BURST_WORD_3 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_3 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_3);
			icp_hd_i2c_write_w(0x9FFE, 0x3B78);
			reg_conf_tbl++;
			break;
		case BURST_WORD_4:			
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_4, icp_hd_tune_regs.init_burst_settings_size_4);
			if (rc < 0)
			{
				pr_err("icp_hd_i2c_bust_4 failed!\n");
				break;
			}
			CDBG("BURST_WORD_4 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_4 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_4);
			rc = icp_hd_i2c_write_w(0x6002, 0xFFFF);
			reg_conf_tbl++;
			break;
#elif defined(BURST_MODE_8192_BYTE_COMPRESS_INIT)
		case BURST_WORD_1:
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_1, icp_hd_tune_regs.init_burst_settings_size_1);
			if (rc < 0)
			{
				pr_err("icp_hd_i2c_bust_1 failed!\n");
				break;
			}
			CDBG("BURST_WORD_1 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_1 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_1);
			reg_conf_tbl++;
			break;
		case BURST_WORD_2:
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_2, icp_hd_tune_regs.init_burst_settings_size_2);
			if (rc < 0)
			{
				pr_err("icp_hd_i2c_bust_2 failed!\n");
				break;
			}
			CDBG("BURST_WORD_2 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_2 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_2);
			reg_conf_tbl++;
			break;
		case BURST_WORD_3:
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_3, icp_hd_tune_regs.init_burst_settings_size_3);
			if (rc < 0)
			{
				pr_err("icp_hd_i2c_bust_3 failed!\n");
				break;
			}
			CDBG("BURST_WORD_3 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_3 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_3);
			reg_conf_tbl++;
			break;
		case BURST_WORD_4:
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_4, icp_hd_tune_regs.init_burst_settings_size_4);
			if (rc < 0)
			{
				pr_err("icp_hd_i2c_bust_4 failed!\n");
				break;
			}
			CDBG("BURST_WORD_4 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_4 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_4);
			reg_conf_tbl++;
			break;
		case BURST_WORD_5:
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_5, icp_hd_tune_regs.init_burst_settings_size_5);
			if (rc < 0)
			{
				pr_err("icp_hd_i2c_bust_5 failed!\n");
				break;
			}
			CDBG("BURST_WORD_5 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_5 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_5);
			reg_conf_tbl++;
			break;
		case BURST_WORD_6:			
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_6, icp_hd_tune_regs.init_burst_settings_size_6);			
			if (rc < 0)
			{
				pr_err("icp_hd_i2c_bust_6 failed!\n");
				break;
			}
			CDBG("BURST_WORD_6 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_6 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_6);
			rc = icp_hd_i2c_write_w(0x9FFE, 0x4220);
			reg_conf_tbl++;	
			break;
		case BURST_WORD_7:			
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_7, icp_hd_tune_regs.init_burst_settings_size_7);
			if (rc < 0)
			{
				pr_err("icp_hd_i2c_bust_7 failed!\n");
				break;
			}
			CDBG("BURST_WORD_7 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_7 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_7);
			rc = icp_hd_i2c_write_w(0x9FFE, 0x62B8);
			reg_conf_tbl++;			
			break;
		case BURST_WORD_8:			
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_8, icp_hd_tune_regs.init_burst_settings_size_8);
			if (rc < 0)
			{
				pr_err("icp_hd_i2c_bust_8 failed!\n");
				break;
			}
			CDBG("BURST_WORD_8 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_8 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_8);
			rc = icp_hd_i2c_write_w(0x6002, 0xFFFF);
			rc = icp_hd_i2c_write_dw(0x2028, 0x000186A0);
			reg_conf_tbl++;
			break;
#elif defined(BURST_MODE_8192_BYTE_COMPRESS_INIT_OTP)
		case BURST_WORD_1:
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_1, icp_hd_tune_regs.init_burst_settings_size_1);
			if (rc < 0)
			{
				pr_err("icp_hd_i2c_bust_1 failed!\n");
				break;
			}
			CDBG("BURST_WORD_1 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_1 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_1);
			rc = icp_hd_i2c_write_w(0x9FFE, 0xF391);
			reg_conf_tbl++;
			break;
		case BURST_WORD_2:			
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_2, icp_hd_tune_regs.init_burst_settings_size_2);			
			if (rc < 0)
			{
				pr_err("icp_hd_i2c_bust_2 failed!\n");
				break;
			}
			CDBG("BURST_WORD_2 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_2 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_2);
			rc = icp_hd_i2c_write_w(0x9FFE, 0x0004);
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_3, icp_hd_tune_regs.init_burst_settings_size_3);
			rc = icp_hd_i2c_write_w(0x6002, 0xFFFF);
			rc = icp_hd_i2c_write_dw(0x2028, 0x000186A0);
			reg_conf_tbl++;
			break;
#elif defined(BURST_MODE_8192_BYTE_COMPRESS_INIT_OTP_TUNE_UPDATE)
        case BURST_WORD_1:
            rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_1, icp_hd_tune_regs.init_burst_settings_size_1);
            if (rc < 0)
            {
                pr_err("icp_hd_i2c_bust_1 failed!\n");
                break;
            }
            CDBG("BURST_WORD_1 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_1 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_1);
            rc = icp_hd_i2c_write_w(0x9FFE, 0x00A0);
            reg_conf_tbl++;
            break;
        case BURST_WORD_2:          
            rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_2, icp_hd_tune_regs.init_burst_settings_size_2);         
            if (rc < 0)
            {
                pr_err("icp_hd_i2c_bust_2 failed!\n");
                break;
            }
            CDBG("BURST_WORD_2 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_2 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_2);
            rc = icp_hd_i2c_write_w(0x9FFE, 0x0808);
            rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_3, icp_hd_tune_regs.init_burst_settings_size_3);
            rc = icp_hd_i2c_write_w(0x6002, 0xFFFF);
//            rc = icp_hd_i2c_write_dw(0x2028, 0x000186A0);
            reg_conf_tbl++;
            break;
#elif defined(BURST_MODE_8192_BYTE_COMPRESS_INIT_NODATA)
	  	case BURST_WORD_1:
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_1, icp_hd_tune_regs.init_burst_settings_size_1);
			if (rc < 0)
			{
				pr_err("icp_hd_i2c_bust_1 failed!\n");
				break;
			}
			CDBG("BURST_WORD_1 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_1 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_1);
			rc = icp_hd_i2c_write_w(0x9FFE, 0x4240);
			reg_conf_tbl++;
			break;
		case BURST_WORD_2:			
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_2, icp_hd_tune_regs.init_burst_settings_size_2);			
			if (rc < 0)
			{
				pr_err("icp_hd_i2c_bust_2 failed!\n");
				break;
			}
			CDBG("BURST_WORD_2 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_2 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_2);
			rc = icp_hd_i2c_write_w(0x9FFE, 0x4021);
			reg_conf_tbl++;
			break;
		case BURST_WORD_3:
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_3, icp_hd_tune_regs.init_burst_settings_size_3);
			if (rc < 0)
			{
				pr_err("icp_hd_i2c_bust_3 failed!\n");
				break;
			}
			CDBG("BURST_WORD_3 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_3 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_3);
			rc = icp_hd_i2c_write_w(0x6002, 0xFFFF);
			reg_conf_tbl++;
			break;    	
#elif defined(BURST_MODE_8192_BYTE_COMPRESS_INIT_OTP_TUNE_UPDATE_2)
			case BURST_WORD_1:
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_1, icp_hd_tune_regs.init_burst_settings_size_1);
			if (rc < 0)
			{
				pr_err("icp_hd_i2c_bust_1 failed!\n");
				break;
			}
			CDBG("OTP BURST_WORD_1 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_1 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_1);
			rc = icp_hd_i2c_write_w(0x9FFE, 0x4048);
			reg_conf_tbl++;
			break;
			case BURST_WORD_2:			
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_2, icp_hd_tune_regs.init_burst_settings_size_2);			
			if (rc < 0)
			{
				pr_err("icp_hd_i2c_bust_2 failed!\n");
				break;
			}
			CDBG("OTP BURST_WORD_2 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_2 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_2);
			rc = icp_hd_i2c_write_w(0x9FFE, 0x8181);
			reg_conf_tbl++;
			break;
		case BURST_WORD_3:
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_3, icp_hd_tune_regs.init_burst_settings_size_3);
			if (rc < 0)
			{
				pr_err("icp_hd_i2c_bust_3 failed!\n");
				break;
			}
			CDBG("OTP BURST_WORD_3 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_3 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_3);
			rc = icp_hd_i2c_write_w(0x6002, 0xFFFF);
			reg_conf_tbl++;
			break;
#elif defined(BURST_MODE_8192_BYTE_COMPRESS_INIT_OTP_TUNE_UPDATE_3)
		case BURST_WORD_1:
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_1, icp_hd_tune_regs.init_burst_settings_size_1);
			if (rc < 0)
			{
				pr_err("icp_hd_i2c_bust_1 failed!\n");
				break;
			}
			CDBG("OTP BURST_WORD_1 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_1 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_1);
			rc = icp_hd_i2c_write_w(0x9FFE, 0x5EDF);
			reg_conf_tbl++;
			break;
			case BURST_WORD_2:			
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_2, icp_hd_tune_regs.init_burst_settings_size_2);			
			if (rc < 0)
			{
				pr_err("icp_hd_i2c_bust_2 failed!\n");
				break;
			}
			CDBG("OTP BURST_WORD_2 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_2 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_2);
			rc = icp_hd_i2c_write_w(0x9FFE, 0x0073);
			reg_conf_tbl++;
			break;
		case BURST_WORD_3:
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_3, icp_hd_tune_regs.init_burst_settings_size_3);
			if (rc < 0)
			{
				pr_err("icp_hd_i2c_bust_3 failed!\n");
				break;
			}
			CDBG("OTP BURST_WORD_3 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_3 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_3);
			rc = icp_hd_i2c_write_w(0x6002, 0xFFFF);
			reg_conf_tbl++;
			break;
#elif defined(BURST_MODE_8192_BYTE_COMPRESS_INIT_OTP_TUNE_UPDATE_4)
        case BURST_WORD_1:
            rc = icp_hd_i2c_write_w(0x601A, 0x1000);// OTP uploading command �� ���ķ� ���� Initialization Writing ������.
            rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_1, icp_hd_tune_regs.init_burst_settings_size_1);
            if (rc < 0)
            {
                pr_err("icp_hd_i2c_bust_1 failed!\n");
                break;
            }
            CDBG("OTP BURST_WORD_1 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_1 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_1);
            rc = icp_hd_i2c_write_w(0x9FFE, 0x0602);
            reg_conf_tbl++;
            break;
            case BURST_WORD_2:          
            rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_2, icp_hd_tune_regs.init_burst_settings_size_2);         
            if (rc < 0)
            {
                pr_err("icp_hd_i2c_bust_2 failed!\n");
                break;
            }
            CDBG("OTP BURST_WORD_2 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_2 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_2);
            rc = icp_hd_i2c_write_w(0x9FFE, 0xECCA);
            reg_conf_tbl++;
            break;
        case BURST_WORD_3:
            rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_3, icp_hd_tune_regs.init_burst_settings_size_3);
            if (rc < 0)
            {
                pr_err("icp_hd_i2c_bust_3 failed!\n");
                break;
            }
            CDBG("OTP BURST_WORD_3 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_3 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_3);
            rc = icp_hd_i2c_write_w(0x6002, 0xFFFF);
            reg_conf_tbl++;
            break;
#elif defined(BURST_MODE_8192_BYTE_COMPRESS_INIT_OTP_TUNE_UPDATE_5)
        case BURST_WORD_1:
            rc = icp_hd_i2c_write_w(0x601A, 0x1000);// OTP uploading command �� ���ķ� ���� Initialization Writing ������.
            rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_1, icp_hd_tune_regs.init_burst_settings_size_1);
            if (rc < 0)
            {
                pr_err("icp_hd_i2c_bust_1 failed!\n");
                break;
            }
            CDBG("OTP BURST_WORD_1 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_1 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_1);
            rc = icp_hd_i2c_write_w(0x9FFE, 0x0602);
            reg_conf_tbl++;
            break;
            case BURST_WORD_2:          
            rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_2, icp_hd_tune_regs.init_burst_settings_size_2);         
            if (rc < 0)
            {
                pr_err("icp_hd_i2c_bust_2 failed!\n");
                break;
            }
            CDBG("OTP BURST_WORD_2 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_2 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_2);
            rc = icp_hd_i2c_write_w(0x9FFE, 0xECCA);
            reg_conf_tbl++;
            break;
        case BURST_WORD_3:
            rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_3, icp_hd_tune_regs.init_burst_settings_size_3);
            if (rc < 0)
            {
                pr_err("icp_hd_i2c_bust_3 failed!\n");
                break;
            }
            CDBG("OTP BURST_WORD_3 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_3 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_3);
            rc = icp_hd_i2c_write_w(0x6002, 0xFFFF);
            msleep(10);
            reg_conf_tbl++;
            break;
	case BURST_WORD_4:
	     msleep(10);
	     CDBG("OTP BURST_WORD_4 SUCCESS!!! DELAY 10ms for 0xffff polling\n");
            reg_conf_tbl++;
            break;            
#elif defined(BURST_MODE_8192_BYTE_COMPRESS_INIT_OTP_TUNE_UPDATE_6)
		case BURST_WORD_1:
			// OTP �ε� ���з� ���� preview��ȭ�� OTPĿ�ǵ� burst�κ����� �ٽ� �̵�
			//rc = icp_hd_i2c_write_w(0x601A, 0x1000);// OTP uploading command �� ���ķ� ���� Initialization Writing ������.
			//���� �ҷ��� ���� 10ms
			    msleep(10);
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_1, icp_hd_tune_regs.init_burst_settings_size_1);
			if (rc < 0)
			{
			    pr_err("icp_hd_i2c_bust_1 failed!\n");
			    break;
			}
			CDBG("OTP BURST_WORD_1 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_1 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_1);
			rc = icp_hd_i2c_write_w(0x9FFE, 0x0602);
			reg_conf_tbl++;
			break;
		case BURST_WORD_2:          
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_2, icp_hd_tune_regs.init_burst_settings_size_2);         
			if (rc < 0)
			{
			    pr_err("icp_hd_i2c_bust_2 failed!\n");
			    break;
			}
			CDBG("OTP BURST_WORD_2 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_2 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_2);
			rc = icp_hd_i2c_write_w(0x9FFE, 0xECCA);
			reg_conf_tbl++;
			break;
		case BURST_WORD_3:
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_3, icp_hd_tune_regs.init_burst_settings_size_3);
			if (rc < 0)
			{
			    pr_err("icp_hd_i2c_bust_3 failed!\n");
			    break;
			}
			CDBG("OTP BURST_WORD_3 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_3 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_3);
			rc = icp_hd_i2c_write_w(0x6002, 0xFFFF);
			reg_conf_tbl++;
			break;	
#elif defined(BURST_MODE_8192_BYTE_COMPRESS_INIT_OTP_TUNE_UPDATE_7)
		case BURST_WORD_1:
			// OTP �ε� ���з� ���� preview��ȭ�� OTPĿ�ǵ� burst�κ����� �ٽ� �̵�
			//rc = icp_hd_i2c_write_w(0x601A, 0x1000);// OTP uploading command �� ���ķ� ���� Initialization Writing ������.
			//���� �ҷ��� ���� 10ms
			    msleep(10);
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_1, icp_hd_tune_regs.init_burst_settings_size_1);
			if (rc < 0)
			{
			    pr_err("icp_hd_i2c_bust_1 failed!\n");
			    break;
			}
			CDBG("OTP BURST_WORD_1 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_1 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_1);
			rc = icp_hd_i2c_write_w(0x9FFE, 0x0602);
			reg_conf_tbl++;
			break;
		case BURST_WORD_2:          
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_2, icp_hd_tune_regs.init_burst_settings_size_2);         
			if (rc < 0)
			{
			    pr_err("icp_hd_i2c_bust_2 failed!\n");
			    break;
			}
			CDBG("OTP BURST_WORD_2 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_2 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_2);
			rc = icp_hd_i2c_write_w(0x9FFE, 0xECCA);
			reg_conf_tbl++;
			break;
		case BURST_WORD_3:
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_3, icp_hd_tune_regs.init_burst_settings_size_3);
			if (rc < 0)
			{
			    pr_err("icp_hd_i2c_bust_3 failed!\n");
			    break;
			}
			CDBG("OTP BURST_WORD_3 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_3 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_3);
			rc = icp_hd_i2c_write_w(0x6002, 0xFFFF);
			reg_conf_tbl++;
			break;            
#elif defined(BURST_MODE_4096_BYTE)
		case BURST_WORD_1:
			rc = icp_hd_i2c_write_dw(0xF03C, 0x0000644C);
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_1, icp_hd_tune_regs.init_burst_settings_size_1);			
			if (rc < 0)
			{
				pr_err("icp_hd_i2c_bust_1 failed!\n");
				break;
			}
			CDBG("BURST_WORD_1 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_1 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_1);
			rc = icp_hd_i2c_write_w(0x601A, 0x0002);
			reg_conf_tbl++;
			msleep(50);
			
			break;
		case BURST_WORD_2:
			rc = icp_hd_i2c_write_dw(0xF03C, 0x0000644C);
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_2, icp_hd_tune_regs.init_burst_settings_size_2);
			if (rc < 0)
			{
				pr_err("icp_hd_i2c_bust_2 failed!\n");
				break;
			}
			CDBG("BURST_WORD_2 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_2 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_2);
			rc = icp_hd_i2c_write_w(0x601A, 0x0010);			
			reg_conf_tbl++;
			msleep(50);
			
			break;
		case BURST_WORD_3:
			rc = icp_hd_i2c_write_dw(0xF03C, 0x00000000);
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_3, icp_hd_tune_regs.init_burst_settings_size_3);
			if (rc < 0)
			{
				pr_err("icp_hd_i2c_bust_3 failed!\n");
				break;
			}
			CDBG("BURST_WORD_3 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_3 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_3);
			reg_conf_tbl++;
			break;
		case BURST_WORD_4:
			rc = icp_hd_i2c_write_dw(0xF03C, 0x00000FFC);
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_4, icp_hd_tune_regs.init_burst_settings_size_4);
			if (rc < 0)
			{
				pr_err("icp_hd_i2c_bust_4 failed!\n");
				break;
			}
			CDBG("BURST_WORD_4 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_4 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_4);
			reg_conf_tbl++;
			break;
		case BURST_WORD_5:
			rc = icp_hd_i2c_write_dw(0xF03C, 0x00001FF8);
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_5, icp_hd_tune_regs.init_burst_settings_size_5);
			if (rc < 0)
			{
				pr_err("icp_hd_i2c_bust_5 failed!\n");
				break;
			}
			CDBG("BURST_WORD_5 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_5 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_5);
			reg_conf_tbl++;
			break;
		case BURST_WORD_6:
			rc = icp_hd_i2c_write_dw(0xF03C, 0x00002FF4);
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_6, icp_hd_tune_regs.init_burst_settings_size_6);
			if (rc < 0)
			{
				pr_err("icp_hd_i2c_bust_6 failed!\n");
				break;
			}
			CDBG("BURST_WORD_6 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_6 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_6);
			reg_conf_tbl++;
			break;
		case BURST_WORD_7:
			rc = icp_hd_i2c_write_dw(0xF03C, 0x00003FF0);
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_7, icp_hd_tune_regs.init_burst_settings_size_7);
			if (rc < 0)
			{
				pr_err("icp_hd_i2c_bust_7 failed!\n");
				break;
			}
			CDBG("BURST_WORD_7 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_7 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_7);
			reg_conf_tbl++;
			break;
		case BURST_WORD_8:
			rc = icp_hd_i2c_write_dw(0xF03C, 0x00004FEC);
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_8, icp_hd_tune_regs.init_burst_settings_size_8);
			if (rc < 0)
			{
				pr_err("icp_hd_i2c_bust_8 failed!\n");
				break;
			}
			CDBG("BURST_WORD_8 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_8 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_8);
			reg_conf_tbl++;
			break;
		case BURST_WORD_9:
			rc = icp_hd_i2c_write_dw(0xF03C, 0x00005FE8);
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_9, icp_hd_tune_regs.init_burst_settings_size_9);
			if (rc < 0)
			{
				pr_err("icp_hd_i2c_bust_9 failed!\n");
				break;
			}
			CDBG("BURST_WORD_9 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_9 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_9);
			reg_conf_tbl++;
			break;
		case BURST_WORD_10:
			rc = icp_hd_i2c_write_dw(0xF03C, 0x00006FE4);
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_10, icp_hd_tune_regs.init_burst_settings_size_10);
			if (rc < 0)
			{
				pr_err("icp_hd_i2c_bust_10 failed!\n");
				break;
			}
			CDBG("BURST_WORD_10 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_10 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_10);
			rc = icp_hd_i2c_write_w(0x601A, 0x0002);
			reg_conf_tbl++;
			msleep(50);
			break;
		case BURST_WORD_11:
			rc = icp_hd_i2c_write_dw(0xF03C, 0x000068E8);
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_11, icp_hd_tune_regs.init_burst_settings_size_11);
			if (rc < 0)
			{
				pr_err("icp_hd_i2c_bust_11 failed!\n");
				break;
			}
			CDBG("BURST_WORD_11 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_11 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_11);
			reg_conf_tbl++;
			break;
		case BURST_WORD_12:
			rc = icp_hd_i2c_write_dw(0xF03C, 0x00006D80);
			rc = icp_hd_i2c_txdata(icp_hd_client->addr, icp_hd_tune_regs.init_burst_settings_12, icp_hd_tune_regs.init_burst_settings_size_12);
			if (rc < 0)
			{
				pr_err("icp_hd_i2c_bust_12 failed!\n");
				break;
			}
			CDBG("BURST_WORD_12 SUCCESS!!! next_dwdata= 0x%x, icp_hd_tune_regs.init_burst_settings_size_12 = %d\n", (reg_conf_tbl+1)->dwdata, icp_hd_tune_regs.init_burst_settings_size_12);
			reg_conf_tbl++;
			break;		
#endif			
		default:
		{
			rc = icp_hd_i2c_write(icp_hd_client->addr,
								reg_conf_tbl->waddr, reg_conf_tbl->dwdata,
								reg_conf_tbl->width);
			//CDBG("I2C WRITE!!! ADDR = 0x%x, VALUE = 0x%x, width = %d, num_of_items_in_table=%d, i=%d\n",
				//reg_conf_tbl->waddr, reg_conf_tbl->wdata, reg_conf_tbl->width, num_of_items_in_table, i);

			if (rc < 0)
			{
				pr_err("icp_hd_i2c_write failed!\n");
				return rc;
			}
			
			if (reg_conf_tbl->mdelay_time != 0)
				msleep(reg_conf_tbl->mdelay_time);

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
		rc = icp_hd_i2c_write(icp_hd_client->addr,
			reg_conf_tbl->waddr, reg_conf_tbl->wdata,
			reg_conf_tbl->width);
		if (rc < 0)
			break;
		if (reg_conf_tbl->mdelay_time != 0)
			msleep(reg_conf_tbl->mdelay_time);
		reg_conf_tbl++;
	}
#endif
	return rc;
}

static int icp_hd_i2c_rxdata(unsigned short saddr,
	unsigned char *rxdata, int length)
{
	uint32_t i = 0;
	int32_t rc = 0;
	
	struct i2c_msg msgs[] = {
	{
		.addr   = (saddr >> 1),
		.flags = 0,
		.len   = 2,
		.buf   = rxdata,
	},
	{
		.addr   = (saddr >> 1),
		.flags = I2C_M_RD,
		.len   = length,
		.buf   = rxdata,
	},
	};

#if SENSOR_DEBUG
	if (length == 2)
		CDBG("msm_io_i2c_r: 0x%04x 0x%04x\n",
			*(u16 *) rxdata, *(u16 *) (rxdata + 2));
	else if (length == 4)
		CDBG("msm_io_i2c_r: 0x%04x\n", *(u16 *) rxdata);
	else
		CDBG("msm_io_i2c_r: length = %d\n", length);
#endif
	
	for (i = 0; i < ICP_HD_I2C_RETRY; i++) {
		rc = i2c_transfer(icp_hd_client->adapter, msgs, 2); 
		if (rc >= 0) {			
			return 0;
		}
		CDBG("%s: tx retry. [%02x.%02x.%02x] len=%d rc=%d\n", __func__,(saddr >> 1), *rxdata, *(rxdata + 1), length, rc);
		msleep(ICP_HD_I2C_MPERIOD);
	}
	return -EIO;
}

static int32_t icp_hd_i2c_read(unsigned short   saddr,
	unsigned short raddr, unsigned int *rdata, enum icp_hd_width width)
{
	int32_t rc = 0;
	unsigned char buf[4];

	if (!rdata)
		return -EIO;

	memset(buf, 0, sizeof(buf));

	switch (width) {
	case WORD_LEN: {
		buf[0] = (raddr & 0xFF00)>>8;
		buf[1] = (raddr & 0x00FF);

		rc = icp_hd_i2c_rxdata(saddr, buf, 2);
		if (rc < 0)
			return rc;

		*rdata = buf[0] << 8 | buf[1];
	}
		break;
	case DWORD_LEN: {
		buf[0] = (raddr & 0xFF00)>>8;
		buf[1] = (raddr & 0x00FF);

		rc = icp_hd_i2c_rxdata(saddr, buf, 4);
		if (rc < 0)
			return rc;

		*rdata = buf[0] << 24 | buf[1] << 16 | buf[2] << 8 | buf[3];
	}
		break;
	default:
		break;
	}

	if (rc < 0)
		pr_err("icp_hd_i2c_read failed!\n");

	return rc;
}

#endif

static void icp_hd_sensor_params_info_init(void)
{

	params_info.brightness = -1;
	params_info.effect = -1;
	params_info.exposure = -1;
	params_info.wb = -1;
	params_info.preview_fps = -1;
	params_info.reflect = -1;
	params_info.scene_mode = -1;
	params_info.antibanding = -1;
	params_info.antishake = -1;
}

void icp_hd_sensor_start_stream(struct msm_sensor_ctrl_t *s_ctrl)
{
	int rc = 0;

	rc = icp_hd_i2c_write_table(&icp_hd_tune_regs.streaming_on_settings[0],
					icp_hd_tune_regs.streaming_on_settings_size);
	if (rc < 0)
		SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);

    SKYCDBG("%s: %d\n", __func__, __LINE__);
}

void icp_hd_sensor_stop_stream(struct msm_sensor_ctrl_t *s_ctrl)
{
	int rc = 0;

	rc = icp_hd_i2c_write_table(&icp_hd_tune_regs.streaming_off_settings[0],
					icp_hd_tune_regs.streaming_off_settings_size);

	if (rc < 0)
		SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);

    SKYCDBG("%s: %d\n", __func__, __LINE__);
}

int icp_hd_sensor_init(struct msm_sensor_ctrl_t *s_ctrl)
{
	int rc = 0;
	//uint8_t data_buf[4];

	SKYCDBG("%s: E\n",__func__);

#if 0
    rc = si2c_write_param(SI2C_SA, SI2C_INIT, icp_hd_params);
    if (rc < 0)
        goto sensor_init_fail;
#endif

#if 1//needtocheck
	/* PLL Setup Start */
	rc = icp_hd_i2c_write_table(&icp_hd_tune_regs.init_parm[0],
					icp_hd_tune_regs.init_parm_size);
#ifdef ICP_HD_MODULE_ABNORMAL_OPERATION_DEFENCE
	if(rc == ICP_HD_REG_POLLING_ERROR)
	{
		CDBG("ICP_HD_REG_POLLING_ERROR !\n");
		if (sgpio_ctrl(sgpios, CAMIO_R_RST_N, 0) < 0)	rc = -EIO;//gpio_set_value(SENSOR_RESET,0);
		msleep(5);
		if (sgpio_ctrl(sgpios, CAMIO_R_RST_N, 1) < 0)	rc = -EIO;//gpio_set_value(SENSOR_RESET,1);
		msleep(5);
		icp_hd_sensor_params_info_init();
		rc = icp_hd_i2c_write_table(&icp_hd_tune_regs.init_parm[0],
					icp_hd_tune_regs.init_parm_size);

		if(rc == ICP_HD_REG_POLLING_ERROR)
		{
			CDBG("SECOND POLLING FAIL!!!OUT OF INIT~~~\n");
			return -EIO;
		}
	}
#endif	

#endif

    sensor_mode = 1;

    SKYCDBG("%s: X\n",__func__);    
    return rc;
    
#if 0//needtocheck    
sensor_init_fail:

    //si2c_release();
    
	SKYCDBG("%s err(%d)\n", __func__, rc);
	return rc;
#endif    
}
#if 0

int32_t icp_hd_sensor_write_init_settings(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc;
	rc = msm_sensor_write_all_conf_array(
		s_ctrl->sensor_i2c_client,
		s_ctrl->msm_sensor_reg->init_settings,
		s_ctrl->msm_sensor_reg->init_size);
	return rc;
}

/* msm_sensor_write_res_settings */
//icp_hd_sensor_write_res_settings
int32_t icp_hd_sensor_write_res_settings(struct msm_sensor_ctrl_t *s_ctrl,
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
/* sensor_set_function */
//icp_hd_set_function 
#ifdef CONFIG_PANTECH_CAMERA 
#ifdef F_PANTECH_CAMERA_FIX_CFG_LED_MODE
static int icp_hd_set_isp_gpio_0(bool enable)
{
	//unsigned short isp_gpio_value = 0;
	unsigned int isp_gpio_value = 0;
	int32_t rc, final_rc = 0;
		
	if( (rc = icp_hd_i2c_write(icp_hd_client->addr, 0xf03c, 0x0009,WORD_LEN)) ) final_rc = rc;
	if( (rc = icp_hd_i2c_write(icp_hd_client->addr, 0xf03e, 0x5004,WORD_LEN)) )final_rc = rc;
	if( (rc = icp_hd_i2c_read(icp_hd_client->addr,	0xE002, &isp_gpio_value,WORD_LEN)) ) final_rc = rc;
	//SKYCDBG("GPIO_0:read isp_gpio_value_11 = 0x%x\n",isp_gpio_value);
	isp_gpio_value = isp_gpio_value|0x0001;
	//SKYCDBG("GPIO_0:read isp_gpio_value_12 = 0x%x\n",isp_gpio_value);
	if( (rc = icp_hd_i2c_write(icp_hd_client->addr, 0xE002, isp_gpio_value,WORD_LEN)) ) final_rc = rc;

	if( (rc = icp_hd_i2c_write(icp_hd_client->addr, 0xf03c, 0x0009,WORD_LEN)) ) final_rc = rc;
	if( (rc = icp_hd_i2c_write(icp_hd_client->addr, 0xf03e, 0x5000,WORD_LEN)) ) final_rc = rc;
	if( (rc = icp_hd_i2c_read(icp_hd_client->addr,	0xE002, &isp_gpio_value,WORD_LEN)) ) final_rc = rc;
	//SKYCDBG("GPIO_0:read isp_gpio_value_13 = 0x%x\n",isp_gpio_value);
	if(enable)
	{
		isp_gpio_value = isp_gpio_value|0x0001;
	}
	else
	{
		isp_gpio_value = isp_gpio_value&0xfffe;
	}
	//SKYCDBG("GPIO_0:read isp_gpio_value_14 = 0x%x\n",isp_gpio_value);
	if( (rc = icp_hd_i2c_write(icp_hd_client->addr, 0xE002, isp_gpio_value,WORD_LEN)) ) final_rc = rc;

	if (final_rc < 0)
	{
		SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
		return final_rc;
	}
	return final_rc;
}

static int icp_hd_set_isp_gpio_2(bool enable)
{
	//unsigned short isp_gpio_value = 0;
	unsigned int isp_gpio_value = 0;
	int32_t rc, final_rc = 0;
		
	if( (rc = icp_hd_i2c_write(icp_hd_client->addr, 0xf03c, 0x0009,WORD_LEN)) ) final_rc = rc;
	if( (rc = icp_hd_i2c_write(icp_hd_client->addr, 0xf03e, 0x5004,WORD_LEN)) ) final_rc = rc;
	if( (rc = icp_hd_i2c_read(icp_hd_client->addr,	0xE002, &isp_gpio_value,WORD_LEN)) ) final_rc = rc;
	//SKYCDBG("GPIO_2:read isp_gpio_value_21 = 0x%x\n",isp_gpio_value);
	isp_gpio_value = isp_gpio_value|0x0004;
	//SKYCDBG("GPIO_2:read isp_gpio_value_22 = 0x%x\n",isp_gpio_value);
	if( (rc = icp_hd_i2c_write(icp_hd_client->addr, 0xE002, isp_gpio_value,WORD_LEN)) ) final_rc = rc;

	if( (rc = icp_hd_i2c_write(icp_hd_client->addr, 0xf03c, 0x0009,WORD_LEN)) ) final_rc = rc;
	if( (rc = icp_hd_i2c_write(icp_hd_client->addr, 0xf03e, 0x5000,WORD_LEN)) ) final_rc = rc;
	if( (rc = icp_hd_i2c_read(icp_hd_client->addr,	0xE002, &isp_gpio_value,WORD_LEN)) )final_rc = rc;
	//SKYCDBG("GPIO_2:read isp_gpio_value_23 = 0x%x\n",isp_gpio_value);
	if(enable)
	{
		isp_gpio_value = isp_gpio_value|0x0004;
	}
	else
	{
		isp_gpio_value = isp_gpio_value&0xfffb;
	}
	//SKYCDBG("GPIO_2:read isp_gpio_value_24 = 0x%x\n",isp_gpio_value);
	if( (rc = icp_hd_i2c_write(icp_hd_client->addr, 0xE002, isp_gpio_value,WORD_LEN)) ) final_rc = rc;

	if (final_rc < 0)
	{
		SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
		return final_rc;
	}
	return final_rc;
}
static int32_t icp_hd_sensor_set_led_mode(struct msm_sensor_ctrl_t *s_ctrl ,int8_t led_mode)
{
	/* off, auto, on, movie */	
	int enable_flash_main_gpio = 0;	
	int rc, final_rc = 0;
	
	SKYCDBG("%s: led_mode=%d\n", __func__, led_mode);

	if(led_mode)
		led_auto = ICP_HD_CFG_LED_MODE_AUTO;
	else
		led_auto = ICP_HD_CFG_LED_MODE_OFF;
			
	if ((led_mode < ICP_HD_CFG_LED_MODE_OFF) || (led_mode >= ICP_HD_CFG_LED_MODE_MAX)) {
		SKYCERR("%s: -EINVAL, led_mode=%d\n", __func__, led_mode);
		return -EINVAL;
	}

	if((led_mode != ICP_HD_CFG_LED_MODE_OFF)&&(led_mode != ICP_HD_CFG_LED_MODE_AUTO))
		enable_flash_main_gpio = ON;
	else
		enable_flash_main_gpio = OFF;

	//control icp_hd led flash main gpio
	if( (rc = gpio_tlmm_config(GPIO_CFG(ICP_HD_LED_FLASH_ENABLE_GPIO, 0,
		GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),GPIO_CFG_ENABLE)) ) final_rc = rc;
	if (!final_rc)
	{
		SKYCDBG("%s enable_flash_main_gpio = %d\n", __func__, enable_flash_main_gpio);
		gpio_set_value(ICP_HD_LED_FLASH_ENABLE_GPIO,enable_flash_main_gpio);					
	}
	else
	{
		SKYCERR("ERR:%s ENABLE FLASH GPIO FAIL!!!rc=%d return~~\n", __func__, rc);
		return final_rc;
	}

	//control icp_hd isp gpio
	switch(led_mode)
	{
		case ICP_HD_CFG_LED_MODE_OFF:
			if( (rc = icp_hd_set_isp_gpio_0(OFF)) ) final_rc = rc;
			if( (rc = icp_hd_set_isp_gpio_2(OFF)) ) final_rc = rc;
			break;
		case ICP_HD_CFG_LED_MODE_AUTO:
			if( (rc = icp_hd_set_isp_gpio_0(OFF)) ) final_rc = rc;
			if( (rc = icp_hd_set_isp_gpio_2(OFF)) ) final_rc = rc;
			SKYCDBG("PM_FLASH_LED_SET_CURRENT_PROC rc = %d\n", rc);
			break;
		case ICP_HD_CFG_LED_MODE_ON:
#if 0 //BOARD_VER_G(WS20)
			if( (rc = icp_hd_set_isp_gpio_0(ON)) )  final_rc = rc;
			if( (rc = icp_hd_set_isp_gpio_2(OFF)) ) final_rc = rc;
#else
			if( (rc = icp_hd_set_isp_gpio_0(OFF)) ) final_rc = rc;
			if( (rc = icp_hd_set_isp_gpio_2(ON)) )  final_rc = rc;
#endif
			break;
		case ICP_HD_CFG_LED_MODE_MOVIE:
#if 0 //BOARD_VER_G(WS20)			
			if( (rc = icp_hd_set_isp_gpio_0(ON)) )  final_rc = rc;
			if( (rc = icp_hd_set_isp_gpio_2(OFF)) ) final_rc = rc;
#else		
			if( (rc = icp_hd_set_isp_gpio_0(OFF)) ) final_rc = rc;
			if( (rc = icp_hd_set_isp_gpio_2(ON)) )  final_rc = rc;
#endif			
			break;
		case ICP_HD_CFG_LED_MODE_SNAP:
#if 0 //BOARD_VER_G(WS20)
			if( (rc = icp_hd_set_isp_gpio_0(ON)) ) final_rc = rc;
			if( (rc = icp_hd_set_isp_gpio_2(ON)) ) final_rc = rc;
#else
			if( (rc = icp_hd_set_isp_gpio_0(OFF)) )final_rc = rc;
			if( (rc = icp_hd_set_isp_gpio_2(ON)) ) final_rc = rc;
#endif
			break;
		default:		
			if( (rc = icp_hd_set_isp_gpio_0(OFF)) )final_rc = rc;
			if( (rc = icp_hd_set_isp_gpio_2(OFF)) )final_rc = rc;
			break;			
	}

	is_led_work = led_mode;

	if (final_rc < 0)
	{
		SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
		return final_rc;
	}			
	return final_rc;
}
#endif

#ifdef F_PANTECH_CAMERA_FIX_CFG_BRIGHTNESS
static int32_t icp_hd_sensor_set_brightness(struct msm_sensor_ctrl_t *s_ctrl ,int8_t brightness)
{
	int32_t rc = 0;
//	int i = 0;
	SKYCDBG("%s start~ receive brightness = %d\n",__func__, brightness);
	
	if ((brightness < 0) || (brightness >= 9)) {
		SKYCERR("%s error. brightness=%d\n", __func__, brightness);
		return -EINVAL;
	}
#ifdef F_PANTECH_CAMERA_TUP_LOAD_FILE
	// tunning value
	if((icp_hd_tup_state == ICP_HD_TUNE_STATE_LOAD_VALUE) && (icp_hd_params_tbl.bright[brightness].num_params > 0))	
	{
		SKYCDBG("icp_hd_tup_state = %d, icp_hd_params_tbl.bright[0].num_params=%d\n", icp_hd_tup_state, brightness);
		icp_hd_i2c_write_params(icp_hd_params_tbl.bright[brightness].params,
								icp_hd_params_tbl.bright[brightness].num_params);
	}
	// static value
	else
#endif		
	{
 		rc = icp_hd_i2c_write_table(&icp_hd_tune_regs.bright_cfg_settings[brightness][0],
						icp_hd_tune_regs.bright_cfg_settings_size);		
 	}		

	if (rc < 0)
	{
		SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
		return rc;
	}		

	params_info.brightness = brightness;
	SKYCDBG("%s end\n",__func__);

	return rc;	
}
#endif

static int32_t icp_hd_sensor_set_effect(struct msm_sensor_ctrl_t *s_ctrl ,int8_t effect)
{
//	uint16_t reg_addr;
//	uint16_t reg_val;
	int32_t rc = 0;

	SKYCDBG("%s start\n",__func__);

	if(effect < CAMERA_EFFECT_OFF || effect >= CAMERA_EFFECT_MAX){
		SKYCERR("%s error. effect=%d\n", __func__, effect);
		return -EINVAL;
	}
#ifdef F_PANTECH_CAMERA_TUP_LOAD_FILE
	// tunning value
	if((icp_hd_tup_state == ICP_HD_TUNE_STATE_LOAD_VALUE) && (icp_hd_params_tbl.effect[effect].num_params > 0))	
	{
		icp_hd_i2c_write_params(icp_hd_params_tbl.effect[effect].params,
								icp_hd_params_tbl.effect[effect].num_params);
	}
	// static value
	else
#endif		
	{
  		rc = icp_hd_i2c_write_table(&icp_hd_tune_regs.effect_cfg_settings[effect][0],
						icp_hd_tune_regs.effect_cfg_settings_size);		
	}
	
	if (rc < 0)
	{
		SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
		return rc;
	}		

	params_info.effect = effect;
 	SKYCDBG("%s end\n",__func__);
	
	return rc;
}
#ifdef F_PANTECH_CAMERA_FIX_CFG_EXPOSURE
static int32_t icp_hd_sensor_set_exposure_mode(struct msm_sensor_ctrl_t *s_ctrl ,int8_t exposure)
{
 	int32_t rc = 0;

	SKYCDBG("%s  exposure = %d\n",__func__, exposure);

	if ((exposure < 0) || (exposure >= 4/*ICP_HD_CFG_EXPOSURE_MAX*/))
	{
		SKYCERR("%s FAIL!!! return~~  exposure = %d\n",__func__,exposure);
		return 0;//-EINVAL;
	}
#ifdef F_PANTECH_CAMERA_TUP_LOAD_FILE
	if((icp_hd_tup_state == ICP_HD_TUNE_STATE_LOAD_VALUE) && (icp_hd_params_tbl.exposure[exposure].num_params > 0))	
	{
		icp_hd_i2c_write_params(icp_hd_params_tbl.exposure[exposure].params,
								icp_hd_params_tbl.exposure[exposure].num_params);
	}
	// static value
	else
#endif		
	{
  		rc = icp_hd_i2c_write_table(&icp_hd_tune_regs.exposure_cfg_settings[exposure][0],
						icp_hd_tune_regs.exposure_cfg_settings_size);		
	}
 
	if (rc < 0)
	{
		SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
		return rc;
	}		

	params_info.exposure = exposure;
	SKYCDBG("%s end\n",__func__);

	return rc;
}
#endif
 
#ifdef F_PANTECH_CAMERA_FIX_CFG_WB
static int32_t icp_hd_sensor_set_wb(struct msm_sensor_ctrl_t *s_ctrl ,int8_t wb)
{
	int32_t rc = 0;
		
	SKYCDBG("%s start, whitebalance=%d\n",__func__, wb);
#ifdef F_PANTECH_CAMERA_TUP_LOAD_FILE
	// tunning value
	if((icp_hd_tup_state == ICP_HD_TUNE_STATE_LOAD_VALUE) && (icp_hd_params_tbl.wb[wb-1].num_params > 0))	
	{
		icp_hd_i2c_write_params(icp_hd_params_tbl.wb[wb-1].params,
								icp_hd_params_tbl.wb[wb-1].num_params);
	}
	// static value
	else
#endif		
	{
  		rc = icp_hd_i2c_write_table(&icp_hd_tune_regs.wb_cfg_settings[wb-1][0],
						icp_hd_tune_regs.wb_cfg_settings_size);		
	}

	if (rc < 0)
	{
		SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
		return rc;
	}		

	params_info.wb = wb;
	SKYCDBG("%s end\n",__func__);

	return rc;
}
#endif
#ifdef F_PANTECH_CAMERA_FIX_CFG_PREVIEW_FPS
static int32_t icp_hd_sensor_set_preview_fps(struct msm_sensor_ctrl_t *s_ctrl ,int8_t preview_fps)
{
	/* 0 : variable 10~30fps, 1 ~ 30 : fixed fps, 31 : variable 10~30fps */
	/* default 31 : variable 10 ~ 30fps */
	int32_t rc = 0;

	if ((preview_fps < C_PANTECH_CAMERA_MIN_PREVIEW_FPS) || (preview_fps > C_PANTECH_CAMERA_MAX_PREVIEW_FPS)) {
		SKYCERR("%s: -EINVAL, preview_fps=%d\n", 
			__func__, preview_fps);
		return -EINVAL;
	}

	SKYCDBG("%s: preview_fps=%d\n", __func__, preview_fps);
#if 0//def F_PANTECH_CAMERA_ICP_HD
#define ICP_HD_FFPS_RATE_CONST 256
#define ICP_HD_FFPS_AE_UPPER_CONST 1000000
#define ICP_HD_FFPS_AE_MAX_CONST 3000
	int32_t icp_hd_ffps_value = 0;
	double icp_hd_ffps_ae_upper = 0;
	double icp_hd_ffps_ae_max = 0;

	if(preview_fps != 1)
		icp_hd_ffps_value = preview_fps * ICP_HD_FFPS_RATE_CONST;
	else
		icp_hd_ffps_value = 257;

	icp_hd_ffps_ae_upper =  (1/preview_fps)*ICP_HD_FFPS_AE_UPPER_CONST;
	icp_hd_ffps_ae_max = icp_hd_ffps_ae_upper - ICP_HD_FFPS_AE_MAX_CONST;
	CAM_INFO("icp_hd_ffps_value=%d, icp_hd_ffps_ae_upper=%d, icp_hd_ffps_ae_max=%d\n", icp_hd_ffps_value, icp_hd_ffps_ae_upper, icp_hd_ffps_ae_max);
	
	rc = icp_hd_i2c_write(icp_hd_client->addr, 0x2020, icp_hd_ffps_value, WORD_LEN);
	rc = icp_hd_i2c_write(icp_hd_client->addr, 0x2028, icp_hd_ffps_ae_upper, DWORD_LEN);
	rc = icp_hd_i2c_write(icp_hd_client->addr, 0x2024, icp_hd_ffps_ae_max, DWORD_LEN);
#else	
#ifdef F_PANTECH_CAMERA_TUP_LOAD_FILE
	if((icp_hd_tup_state == ICP_HD_TUNE_STATE_LOAD_VALUE) && (icp_hd_params_tbl.ffps[preview_fps].num_params > 0))	
	{
		icp_hd_i2c_write_params(icp_hd_params_tbl.ffps[preview_fps].params,
								icp_hd_params_tbl.ffps[preview_fps].num_params);
	}
	// static value
	else
#endif		
	{
  		rc = icp_hd_i2c_write_table(&icp_hd_tune_regs.preview_fps_cfg_settings[preview_fps][0],
						icp_hd_tune_regs.preview_fps_cfg_settings_size);		
	}
#endif

	if (rc < 0)
	{
		SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
		return rc;
	}

    current_fps = preview_fps;
    params_info.preview_fps = preview_fps;

	SKYCDBG("%s end rc = %d\n",__func__, rc);

	return 0;
}
#endif
#ifdef F_PANTECH_CAMERA_FIX_CFG_REFLECT
static int32_t icp_hd_sensor_set_reflect(struct msm_sensor_ctrl_t *s_ctrl ,int8_t reflect)
{
	int32_t rc = 0;
	int32_t i = 0;
	//int8_t npolling = -1;
	unsigned int npolling = -1;

	SKYCDBG("%s  reflect = %d\n",__func__, reflect);

	if ((reflect < 0) || (reflect >= ICP_HD_CFG_REFLECT_MAX))
	{
		SKYCERR("%s FAIL!!! return~~  reflect = %d\n",__func__,reflect);
		return 0;//-EINVAL;
	}
#ifdef F_PANTECH_CAMERA_TUP_LOAD_FILE
	if((icp_hd_tup_state == ICP_HD_TUNE_STATE_LOAD_VALUE) && (icp_hd_params_tbl.reflect[reflect].num_params > 0))	
	{
		icp_hd_i2c_write_params(icp_hd_params_tbl.reflect[reflect].params,
								icp_hd_params_tbl.reflect[reflect].num_params);
	}
	// static value
	else
#endif		
	{
  		rc = icp_hd_i2c_write_table(&icp_hd_tune_regs.reflect_cfg_settings[reflect][0],
						icp_hd_tune_regs.reflect_cfg_settings_size);		
	}

	for (i = 0; i < ICP_HD_POLLING_RETRY; i++) {
		rc = icp_hd_i2c_read(icp_hd_client->addr,
		0x8404, &npolling,TRIPLE_LEN);
		if (rc < 0)
		{
			SKYCERR("ERR:%s POLLING FAIL!!!rc=%d return~~\n", __func__, rc);
			//return rc;
		}
		//SKYCDBG("%s: retry npolling = %x, count = %d\n", __func__,npolling, i);
		if (npolling == 0) {
			params_info.reflect = reflect;
			return 0;
		}		
		msleep(20);
	}	
 
	if (rc < 0)
	{
		SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
		return rc;
	}		
	
	SKYCDBG("%s end\n",__func__);
	params_info.reflect = reflect;
	return rc;

}
#endif

#ifdef F_PANTECH_CAMERA_FIX_CFG_AF
#define AF_POLL_PERIOD	50
#define AF_POLL_RETRY	100
#define ICP_HD_FOCUS_AUTO_COMPLETE_REG 0x01e0
#define ICP_HD_FOCUS_RECT_START_REG 0x01F1
#define ICP_HD_FOCUS_RECT_COMPLETE_REG 0x01F0
static int32_t icp_hd_sensor_set_auto_focus(struct msm_sensor_ctrl_t *s_ctrl ,int8_t autofocus)
{
	int32_t rc = 0;
	//int32_t i = 0;
	//unsigned short read_buf = 0;
//	unsigned int read_buf = 0; 

	SKYCDBG("%s  auto_focus = %d\n",__func__, autofocus);
	if ((autofocus < 0) || (autofocus > 5/*4*/))//pangya
	{
		SKYCERR("%s FAIL!!! return~~  autofocus = %d\n",__func__,autofocus);
		return 0;//-EINVAL;
	}

	switch(autofocus)
	{
#ifdef F_PANTECH_CAMERA_FIX_CFG_AF //ICS fix autofocus == spotfocus mode
		case 2:
#endif
		case 4:
			SKYCDBG("%s START~ SPOT autofocus = %d\n",__func__, autofocus);
			rc =	icp_hd_i2c_write(icp_hd_client->addr,	0x4004, ICP_HD_FOCUS_RECT_START_REG, WORD_LEN);
			if (rc < 0) {
				SKYCERR("ERR:%s SF SET RECT FAIL!!!rc=%d return~~\n", __func__, rc);
				return rc;
			}
/*
			for (i = 0; i < AF_POLL_RETRY; i++) 
			{
				msleep(AF_POLL_PERIOD);
				
				rc = icp_hd_i2c_read(icp_hd_client->addr,
						0x4004, &read_buf,
						WORD_LEN);
				if (rc < 0)
				{
					SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
					return rc;
				}
				SKYCDBG("AF polling num = %d,  read_buf = %x\n", i, read_buf);

				if(read_buf == ICP_HD_FOCUS_RECT_COMPLETE_REG)
					break;
			}
*/			
			//SKYCDBG("%s END~ SPOT autofocus = %d\n",__func__, autofocus);
			break;
		default:
			//SKYCDBG("%s START~ DEFAULT autofocus = %d\n",__func__, autofocus);
#if 0//def F_PANTECH_CAMERA_FIX_CFG_LED_MODE			
			if(led_auto == ICP_HD_CFG_LED_MODE_AUTO)
			{	
				rc = icp_hd_i2c_read(icp_hd_client->addr,	0x400A , &nbrightness,WORD_LEN);
				SKYCDBG("%s FLASH_AUTO_MODE nbrightness =0x%x\n",__func__, nbrightness);
				if((nbrightness <= 0x0200)||(nbrightness >= 0x8000))
					icp_hd_sensor_set_led_mode(s_ctrl, ICP_HD_CFG_LED_MODE_MOVIE);
			}
#endif
	 		rc = icp_hd_i2c_write_table(&icp_hd_tune_regs.autofocus_trigger_cfg_settings[0],
							icp_hd_tune_regs.autofocus_trigger_cfg_settings_size);		

			if (rc < 0)
			{
				SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
				return rc;
			}		
/*		      
			for (i = 0; i < AF_POLL_RETRY; i++) 
			{
				msleep(AF_POLL_PERIOD);
				
				rc = icp_hd_i2c_read(icp_hd_client->addr,
						0x4004, &read_buf,
						WORD_LEN);
				if (rc < 0)
				{
					SKYCDBG("AUTO FOCUS READ I2C FAIL!!! return~~\n");
#if 0 //def F_PANTECH_CAMERA_FIX_CFG_LED_MODE
					if((led_auto == ICP_HD_CFG_LED_MODE_AUTO)&&((nbrightness <= 0x0200)||(nbrightness >= 0x8000)))
						icp_hd_sensor_set_led_mode(s_ctrl, ICP_HD_CFG_LED_MODE_OFF);
#endif			
					return rc;
				}

				SKYCDBG("AF polling num = %d,  read_buf = %x\n", i, read_buf);

				if(read_buf == ICP_HD_FOCUS_AUTO_COMPLETE_REG)
					break;
			}
*/
			//SKYCDBG("%s end\n",__func__);
#if 0 //def F_PANTECH_CAMERA_FIX_CFG_LED_MODE
			if((led_auto == ICP_HD_CFG_LED_MODE_AUTO)&&((nbrightness <= 0x0200)||(nbrightness >= 0x8000)))
				icp_hd_sensor_set_led_mode(s_ctrl, ICP_HD_CFG_LED_MODE_OFF);
#endif	
			SKYCDBG("%s END~ DEFAULT autofocus = %d\n",__func__, autofocus);
			break;			
	}

	return rc;
}
//p11634_add_af_check_130715
static int32_t icp_hd_sensor_check_af(struct msm_sensor_ctrl_t *s_ctrl ,int8_t autofocus, int32_t *af_result)
{
	int32_t rc = 0;
//	int32_t i = 0;
	unsigned int read_buf = 0; 

	//SKYCDBG("%s  auto_focus = %d, af_result = %d\n",__func__, autofocus, *af_result);

	*af_result = 0; //must be set to 0 for polling
	 
	switch(autofocus)
	{
		case 2:
		case 4:
			rc = icp_hd_i2c_read(icp_hd_client->addr,
					0x4004, &read_buf,
					WORD_LEN);
			if (rc < 0)
			{
				SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
				return rc;
			}
			//SKYCDBG("AF polling num = %d,  read_buf = %x\n", i++, read_buf);

			if(read_buf == ICP_HD_FOCUS_RECT_COMPLETE_REG)
			{
				*af_result = 1;
				break;
			}
			//SKYCDBG("%s END~ SPOT autofocus = %d\n",__func__, autofocus);
			break;
		default:
			//SKYCDBG("%s START~ DEFAULT autofocus = %d\n",__func__, autofocus);
			rc = icp_hd_i2c_read(icp_hd_client->addr,
					0x4004, &read_buf,
					WORD_LEN);
			if (rc < 0)
			{
				SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
				return rc;
			}
			//SKYCDBG("AF polling num = %d,  read_buf = %x\n", i++, read_buf);

			if(read_buf == ICP_HD_FOCUS_AUTO_COMPLETE_REG)
			{
				*af_result = 1;
				break;
			}
			//SKYCDBG("%s END~ SPOT autofocus = %d\n",__func__, autofocus);
			//SKYCDBG("%s end\n",__func__);
			break;

	}
	return rc ;
}
#endif
#ifdef F_PANTECH_CAMERA_FIX_CFG_FOCUS_RECT
static int32_t icp_hd_sensor_set_focus_rect(struct msm_sensor_ctrl_t *s_ctrl , int32_t focus_rect)
{
#define ICP_HD_FOCUS_RECT_WINDOW_MAX 255
	uint8_t xleft, xright, ytop, ybottom /*, xc, yc*/ ;
	uint32_t sysx = 0;
	uint32_t eyex = 0;
	int32_t rc = 0;
//	int32_t i = 0;
//	unsigned short read_buf = 0;

	//SKYCDBG("%s  focus_rect = %d\n",__func__, focus_rect);

	ytop = (uint8_t)((focus_rect & 0xff000000) >> 24);
	xleft = (uint8_t)((focus_rect & 0x00ff0000) >> 16);
	ybottom = (uint8_t)((focus_rect & 0x0000ff00) >> 8);
	xright = (uint8_t)((focus_rect & 0x000000ff));

	if (focus_rect == 0) {		
		xleft = 0x60;
		ytop = 0x60;
		xright = 0xA0;
		ybottom = 0xA0;
	}

	sysx = (ytop << 8)|xleft;
	eyex = (ybottom << 8)|xright;
	
	//SKYCDBG("%s  xleft = %d, ytop = %d, xright = %d, ybottom = %d, sysx = %d, eyex = %d,\n",__func__, xleft, ytop, xright, ybottom, sysx, eyex);
		
	rc =	icp_hd_i2c_write(icp_hd_client->addr,	0x404C, sysx, WORD_LEN);
	if (rc < 0) {
		SKYCERR("ERR:%s SF SET START RECT FAIL!!!rc=%d return~~\n", __func__, rc);
		return rc;
	}
	rc =	icp_hd_i2c_write(icp_hd_client->addr,	0x404E, eyex, WORD_LEN);
	if (rc < 0) {
		SKYCERR("ERR:%s SF SET END RECT FAIL!!!rc=%d return~~\n", __func__, rc);
		return rc;
	}

	//SKYCDBG("%s end\n",__func__);
	
	return rc;	
}
#endif
#ifdef F_PANTECH_CAMERA_FIX_CFG_SCENE_MODE
static int8_t old_scene_value = ICP_HD_CFG_SCENE_MODE_OFF;
static int scene_mode_to_table(int8_t scene_mode)
{
	return icp_scenemode[scene_mode];
}
static int32_t icp_hd_sensor_set_scene_mode(struct msm_sensor_ctrl_t *s_ctrl , int8_t scene_mode)
{
	/* ICP_HD supported the following scene */
	/* +off(Normal), +auto(Normal), +portrait, +landscape, indoor, +sports, +night, beach, snow, +sunset, text, party(firework) */

	int32_t rc = 0;
	int32_t scene_mode_off_index = 0;

	SKYCDBG("%s start~ ,  scene_mode = %d\n",__func__,scene_mode);
	scene_mode = scene_mode_to_table(scene_mode);
	SKYCDBG("%s ,  scene_mode_to_table = %d\n",__func__,scene_mode);
	if ((scene_mode < 0) || (scene_mode >= ICP_HD_CFG_SCENE_MODE_MAX))
	{
		SKYCERR("%s not support scene mode  = %d\n",__func__,scene_mode);
		return -EINVAL;
	}
	
#ifdef F_PANTECH_CAMERA_TUP_LOAD_FILE	
	if((icp_hd_tup_state == ICP_HD_TUNE_STATE_LOAD_VALUE) && (icp_hd_params_tbl.scene[scene_mode].num_params > 0))	
	{
		icp_hd_i2c_write_params(icp_hd_params_tbl.scene[scene_mode].params,
								icp_hd_params_tbl.scene[scene_mode].num_params);
	}
	// static value
	else
#endif		
	{
		if((scene_mode == ICP_HD_CFG_SCENE_MODE_OFF) || (scene_mode == ICP_HD_CFG_SCENE_MODE_AUTO))
		{
		SKYCDBG("DBG~1 old_scene_value=%d, new_scene_mode=%d, scene_mode_off_index=%d",old_scene_value, scene_mode, scene_mode_off_index);
			if((old_scene_value == ICP_HD_CFG_SCENE_MODE_PORTRAIT)||
			   (old_scene_value == ICP_HD_CFG_SCENE_MODE_SPORTS)||
			   (old_scene_value == ICP_HD_CFG_SCENE_MODE_NIGHT))
			{
				switch(old_scene_value)
				{
					case ICP_HD_CFG_SCENE_MODE_PORTRAIT:
						scene_mode_off_index = 0;
						break;
					case ICP_HD_CFG_SCENE_MODE_SPORTS:
						scene_mode_off_index = 1;
						break;
					case ICP_HD_CFG_SCENE_MODE_NIGHT:
						scene_mode_off_index = 2;
						break;
					default:
						break;
				}
				SKYCDBG("DBG~2 old_scene_value=%d, new_scene_mode=%d, scene_mode_off_index=%d",old_scene_value, scene_mode, scene_mode_off_index);
		  		rc = icp_hd_i2c_write_table(&icp_hd_tune_regs.scene_mode_off_cfg_settings[scene_mode_off_index][0],
								icp_hd_tune_regs.scene_mode_off_cfg_settings_size);		
				SKYCDBG("SCENE MODE OFF register setting!!! old_scene_value=%d, new_scene_mode=%d",old_scene_value, scene_mode);
			}
		}
  		rc = icp_hd_i2c_write_table(&icp_hd_tune_regs.scene_mode_cfg_settings[scene_mode][0],
						icp_hd_tune_regs.scene_mode_cfg_settings_size);		
	}

	//to turn off the portrait, sports, night scene
	old_scene_value = scene_mode;

	if (rc < 0)
	{
		SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
		return rc;
	}		

	params_info.scene_mode = scene_mode;

	SKYCDBG("%s end\n",__func__);

	return rc;
}
#endif
#ifdef F_PANTECH_CAMERA_FIX_CFG_ANTIBANDING
static int32_t icp_hd_sensor_set_antibanding(struct msm_sensor_ctrl_t *s_ctrl , int8_t antibanding)
{
	int32_t rc = 0;

	if ((antibanding < 0) || (antibanding >= ICP_HD_CFG_FLICKER_MAX))
	{
		SKYCERR("%s FAIL!!! return~~  antibanding = %d\n",__func__,antibanding);
		return 0;//-EINVAL;
	}
#ifdef F_PANTECH_CAMERA_TUP_LOAD_FILE
	if((icp_hd_tup_state == ICP_HD_TUNE_STATE_LOAD_VALUE) && (icp_hd_params_tbl.flicker[antibanding].num_params > 0))	
	{
		icp_hd_i2c_write_params(icp_hd_params_tbl.flicker[antibanding].params,
								icp_hd_params_tbl.flicker[antibanding].num_params);
	}
	// static value
	else
#endif		
	{
		rc = icp_hd_i2c_write_table(&icp_hd_tune_regs.flicker_cfg_settings[antibanding][0],	
					icp_hd_tune_regs.flicker_cfg_settings_size);
	}		

	if (rc < 0)
	{
		SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
		return rc;
	}		

	params_info.antibanding = antibanding;
	SKYCDBG("%s end\n",__func__);
	return rc;
}
#endif
#ifdef F_PANTECH_CAMERA_ADD_CFG_ANTISHAKE
static int8_t is_antishake = ICP_HD_CFG_ANTISHAKE_OFF;
static int32_t icp_hd_sensor_set_antishake(struct msm_sensor_ctrl_t *s_ctrl , int8_t antishake)
{
	int32_t rc = 0;

	if ((antishake < 0) || (antishake >= ICP_HD_CFG_ANTISHAKE_MAX))
	{
		SKYCERR("%s FAIL!!! return~~  antishake = %d\n",__func__,antishake);
		return 0;//-EINVAL;
	}
#ifdef F_PANTECH_CAMERA_TUP_LOAD_FILE
	if((icp_hd_tup_state == ICP_HD_TUNE_STATE_LOAD_VALUE) && (icp_hd_params_tbl.antishake[antishake].num_params > 0))	
	{
		icp_hd_i2c_write_params(icp_hd_params_tbl.antishake[antishake].params,
								icp_hd_params_tbl.antishake[antishake].num_params);
	}
	// static value
	else
#endif		
	{
		rc = icp_hd_i2c_write_table(&icp_hd_tune_regs.antishake_cfg_settings[antishake][0],	
					icp_hd_tune_regs.antishake_cfg_settings_size);
	}

	//check the antishake flag to do halfshutter
	is_antishake = antishake;

	if (rc < 0)
	{
		SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
		return rc;
	}		

	params_info.antishake = antishake;
	SKYCDBG("%s end\n",__func__);
	
	return rc;
}
#endif
#endif //CONFIG_PANTECH_CAMERA

static void icp_hd_sensor_params_info_restore(struct msm_sensor_ctrl_t *s_ctrl)
{
#ifdef F_PANTECH_CAMERA_FIX_CFG_BRIGHTNESS
	if(params_info.brightness >= 0)
		icp_hd_sensor_set_brightness(s_ctrl, params_info.brightness);
#endif

	if(params_info.effect >= 0)
		icp_hd_sensor_set_effect(s_ctrl, params_info.effect);

#ifdef F_PANTECH_CAMERA_FIX_CFG_EXPOSURE
	if(params_info.exposure >= 0)
		icp_hd_sensor_set_exposure_mode(s_ctrl, params_info.exposure);
#endif
#ifdef F_PANTECH_CAMERA_FIX_CFG_WB
	if(params_info.wb >= 0)
		icp_hd_sensor_set_wb(s_ctrl, params_info.wb);
#endif
#ifdef F_PANTECH_CAMERA_FIX_CFG_PREVIEW_FPS
	if(params_info.preview_fps >= 0)
		icp_hd_sensor_set_preview_fps(s_ctrl, params_info.preview_fps);
#endif
#ifdef F_PANTECH_CAMERA_FIX_CFG_REFLECT
	if(params_info.reflect >= 0)
		icp_hd_sensor_set_reflect(s_ctrl, params_info.reflect);
#endif
#ifdef F_PANTECH_CAMERA_FIX_CFG_SCENE_MODE
	if(params_info.scene_mode >= 0)
		icp_hd_sensor_set_scene_mode(s_ctrl, params_info.scene_mode);
#endif
#ifdef F_PANTECH_CAMERA_FIX_CFG_ANTIBANDING
	if(params_info.antibanding >= 0)
		icp_hd_sensor_set_antibanding(s_ctrl, params_info.antibanding);
#endif
#ifdef F_PANTECH_CAMERA_ADD_CFG_ANTISHAKE
	if(params_info.antishake >= 0)
		icp_hd_sensor_set_antishake(s_ctrl, params_info.antishake);
#endif
}

static int32_t icp_hd_1080p_config(int on)
{
	int32_t rc = 0;

	SKYCDBG("%s enter\n",__func__);

	if(on)
	{
		rc = icp_hd_i2c_write_table(&icp_hd_tune_regs.preview_1920_1080_cfg_settings[0], \
										icp_hd_tune_regs.preview_1920_1080_cfg_settings_size);
	}
	else
	{
		rc = icp_hd_i2c_write_table(&icp_hd_tune_regs.preview_1280_960_cfg_settings[0], \
										icp_hd_tune_regs.preview_1280_960_cfg_settings_size);
	}

	//rc = icp_hd_i2c_read(icp_hd_client->addr, 0x000C, &read_data, WORD_LEN);
	//SKYCDBG("PREVIEW_WITDH = %d\n",read_data);
	//rc = icp_hd_i2c_read(icp_hd_client->addr, 0x000E, &read_data, WORD_LEN);
	//SKYCDBG("PREVIEW_HEIGHT = %d\n",read_data);

	return rc;
}

static int32_t icp_hd_video_config(int snapShotFlag)
{
	int32_t rc = 0;
	int32_t i=0;
	//int16_t read_data =0;
	unsigned int read_data = 0;
	
	/* set preview resolution to 1280x960 */
	SKYCDBG("%s start = %d\n",__func__, snapShotFlag);

	if(!snapShotFlag)
	{
	rc = icp_hd_i2c_write(icp_hd_client->addr, 0x1000, 0x0001, WORD_LEN);
	rc = icp_hd_i2c_write(icp_hd_client->addr, 0x1000, 0x2000, WORD_LEN);
	mdelay(50);
	}
    
	for(i=0; i<30; i++) 
	{
       	rc = icp_hd_i2c_read(icp_hd_client->addr, 0x1000, &read_data, WORD_LEN);
		//SKYCDBG("%s icp_hd_i2c_read cont = %d, read_data = %d\n",__func__,i, read_data);
		mdelay(50);
		if (read_data == 0x0000) 
			break;
	}
	
	SKYCDBG("%s icp_hd_i2c_read cont = %d, read_data = %d\n",__func__,i, read_data);

	//rc = icp_hd_i2c_read(icp_hd_client->addr, 0x000C, &read_data, WORD_LEN);
	//SKYCDBG("PREVIEW_WITDH = %d\n",read_data);
	//rc = icp_hd_i2c_read(icp_hd_client->addr, 0x000E, &read_data, WORD_LEN);
	//SKYCDBG("PREVIEW_HEIGHT = %d\n",read_data);

	if (rc < 0 || read_data)
	{
		pr_err("ERR:%s FAIL!!!rc=%d, read_data=%d return~~\n", __func__, rc, read_data);
		if(rc == 0)
			return read_data;
		else
			return rc;
	}

	SKYCDBG("%s end rc = %d\n",__func__, rc);

	return rc;
}

void icp_hd_mipi_init(struct msm_sensor_ctrl_t *s_ctrl, int update_type, int res)
{
    SKYCDBG("%s: ==> MIPI setting  E %d\n", __func__, update_type);
	s_ctrl->curr_csic_params = s_ctrl->csic_params[res];
	CDBG("CSI config in progress\n");
	v4l2_subdev_notify(&s_ctrl->sensor_v4l2_subdev, NOTIFY_CSIC_CFG, s_ctrl->curr_csic_params);
	CDBG("CSI config is done\n");
	mb();
	msleep(30);
    SKYCDBG("%s: ==> MIPI setting  X %d\n", __func__, update_type);	
}

int32_t icp_hd_sensor_reg_init(struct msm_sensor_ctrl_t *s_ctrl)
{
#define ICP_HD_HALF_SHUTTER_POLLING_PERIOD 5
#define ICP_HD_LED_HALF_SHUTTER_POLLING_RETRY 100 //10->100
#define ICP_HD_LED_HALF_SHUTTER_COMPLETE_REG 0
#define ICP_HD_ANTISHAKE_HALF_SHUTTER_POLLING_RETRY 100
#define ICP_HD_OTP_UPLOADING_PEROID 5
#define ICP_HD_OTP_UPLOADING_POLLING_RETRY 100

	int i;
	int32_t rc = 0;
	unsigned int read_buf = 0;
	
	icp_hd_sensor_init(s_ctrl);//msm_sensor_write_init_settings(s_ctrl);
	if (icp_hd_i2c_write(icp_hd_client->addr, 0x2036, 0x0012, WORD_LEN) < 0)
		SKYCDBG("BURST_MODE_8192_BYTE_COMPRESS_INIT_NODATA 1");
	if (icp_hd_i2c_write(icp_hd_client->addr, 0x302E, 0x0012, WORD_LEN) < 0)
		SKYCDBG("BURST_MODE_8192_BYTE_COMPRESS_INIT_NODATA 2");
	if (icp_hd_i2c_write(icp_hd_client->addr, 0x1000, 0x2000, WORD_LEN) < 0)
		SKYCDBG("BURST_MODE_8192_BYTE_COMPRESS_INIT_NODATA 3");

#ifdef F_PANTECH_CAMERA_FIX_CFG_LED_MODE				
	rc = icp_hd_i2c_read(icp_hd_client->addr,	0x400A , &nbrightness,WORD_LEN);			
	if((nbrightness <= 0x0200)||(nbrightness >= 0x8000))
	{
		SKYCDBG("%s OTP Loading for Low Light delay, nbrightness =0x%x\n",__func__, nbrightness);
		msleep(150);
	}
#endif	
    //ICP_HD_OTP_UPLOADING_POLLING
    for (i = 0; i < ICP_HD_OTP_UPLOADING_POLLING_RETRY; i++) 
    {
        msleep(ICP_HD_OTP_UPLOADING_PEROID);
        
        rc = icp_hd_i2c_read(icp_hd_client->addr, 0x601A, &read_buf, WORD_LEN);
	if (rc < 0)
	{
            SKYCDBG("OTP UPLOADING POLLING READ I2C FAIL!!! return~~ rc=%d\n", rc);
		return rc;
	}

        //SKYCDBG("OTP UPLOADING polling num = %d,  read_buf = %x\n", i, read_buf);

        if(read_buf == 0)
        {
            SKYCDBG("OTP UPLOADING polling Complete!! \n");
            break;
        }
    }

	return rc;
}

static int32_t icp_hd_snapshot_config(void)
{
#define ICP_HD_SNAPSHOT_CONFIG_POLLING_PERIOD 5
#define ICP_HD_SNAPSHOT_CONFIG_POLLING_RETRY 100

	int32_t rc = 0;
	int i=0;
	//int16_t read_data =0;
	unsigned int read_data =0;

	/* set snapshot resolution to 3264x2448 */
	SKYCDBG("%s start\n",__func__);
#if 0
	rc = icp_hd_i2c_write_table(&icp_hd_regs.snapshot_cfg_settings[0],
					icp_hd_regs.snapshot_cfg_settings_size);
	msleep(300);
#else
	//rc = icp_hd_i2c_write(icp_hd_client->addr, 0x1004, 0x0028, WORD_LEN);
       rc = icp_hd_i2c_write(icp_hd_client->addr, 0x1000, 0x0008, WORD_LEN);
	for(i = 0; i<ICP_HD_SNAPSHOT_CONFIG_POLLING_RETRY; i++) {
       	rc = icp_hd_i2c_read(icp_hd_client->addr, 0x1000, &read_data, WORD_LEN);
		//SKYCDBG("%s icp_hd_i2c_read cont = %d, read_data = %d\n",__func__,i, read_data);
		msleep(ICP_HD_SNAPSHOT_CONFIG_POLLING_PERIOD);
		if (read_data == 0x0009) {
			SKYCDBG("%s icp_hd_i2c_read cont = %d, read_data = %d\n",__func__,i, read_data);
			//msleep(200);
			break;
		}
		//msleep(50);
	}
#endif
	if (rc < 0)
	{
		SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
		return rc;
	}
	SKYCDBG("%s end rc = %d\n",__func__, rc);
	
	return rc;
}

int32_t icp_hd_sensor_setting(struct msm_sensor_ctrl_t *s_ctrl,
			int update_type, int res)
{
	int32_t rc = 0;

	s_ctrl->func_tbl->sensor_stop_stream(s_ctrl);
	msleep(30);
	if (update_type == MSM_SENSOR_REG_INIT) {
		s_ctrl->curr_csi_params = NULL;
		msm_sensor_enable_debugfs(s_ctrl);
//needtocheck//        
		icp_hd_sensor_init(s_ctrl);//msm_sensor_write_init_settings(s_ctrl);
		
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
                si2c_write_param(SI2C_SA, SI2C_INIT, icp_hd_params);

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

int32_t icp_hd_sensor_setting1(struct msm_sensor_ctrl_t *s_ctrl,
			int update_type, int res)
{
#define ICP_HD_HALF_SHUTTER_POLLING_PERIOD 5
#define ICP_HD_LED_HALF_SHUTTER_POLLING_RETRY 100 //10->100
#define ICP_HD_LED_HALF_SHUTTER_COMPLETE_REG 0
#define ICP_HD_ANTISHAKE_HALF_SHUTTER_POLLING_RETRY 100
#define ICP_HD_OTP_UPLOADING_PEROID 5
#define ICP_HD_OTP_UPLOADING_POLLING_RETRY 100

	int32_t rc = 0;
	static int csi_config;
	unsigned int read_buf = 0;
	static bool b_snapshot_flag;
	static bool b_1080p_flag = false;
	
	int i;
       SKYCDBG("%s: update_type = %d, res=%d E\n", __func__, update_type, res);

	s_ctrl->func_tbl->sensor_stop_stream(s_ctrl);
	msleep(50);
	if (update_type == MSM_SENSOR_REG_INIT) 
	{
		CDBG("Register INIT\n");
		s_ctrl->curr_csi_params = NULL;
		msm_sensor_enable_debugfs(s_ctrl);
//needtocheck//        
		b_1080p_flag = false;
		b_snapshot_flag = false;

		#if 0
		icp_hd_sensor_init(s_ctrl);//msm_sensor_write_init_settings(s_ctrl);
		
		if (icp_hd_i2c_write(icp_hd_client->addr, 0x2036, 0x0012, WORD_LEN) < 0)
			SKYCDBG("BURST_MODE_8192_BYTE_COMPRESS_INIT_NODATA 1");
		if (icp_hd_i2c_write(icp_hd_client->addr, 0x302E, 0x0012, WORD_LEN) < 0)
			SKYCDBG("BURST_MODE_8192_BYTE_COMPRESS_INIT_NODATA 2");
		if (icp_hd_i2c_write(icp_hd_client->addr, 0x1000, 0x2000, WORD_LEN) < 0)
			SKYCDBG("BURST_MODE_8192_BYTE_COMPRESS_INIT_NODATA 3");

#ifdef F_PANTECH_CAMERA_FIX_CFG_LED_MODE				
		rc = icp_hd_i2c_read(icp_hd_client->addr,	0x400A , &nbrightness,WORD_LEN);			
		if((nbrightness <= 0x0200)||(nbrightness >= 0x8000))
		{
			SKYCDBG("%s OTP Loading for Low Light delay, nbrightness =0x%x\n",__func__, nbrightness);
			msleep(150);
		}
#endif	
        //ICP_HD_OTP_UPLOADING_POLLING
        for (i = 0; i < ICP_HD_OTP_UPLOADING_POLLING_RETRY; i++) 
        {
            msleep(ICP_HD_OTP_UPLOADING_PEROID);
            
            rc = icp_hd_i2c_read(icp_hd_client->addr, 0x601A, &read_buf, WORD_LEN);
            if (rc < 0)
            {
                SKYCERR("OTP UPLOADING POLLING READ I2C FAIL!!! return~~ rc=%d\n", rc);
                return rc;
            }

            //SKYCDBG("OTP UPLOADING polling num = %d,  read_buf = %x\n", i, read_buf);

            if(read_buf == 0)
            {
                SKYCDBG("OTP UPLOADING polling Complete!! \n");
                break;
            }
        }
	#else
		icp_hd_sensor_reg_init(s_ctrl);
	#endif
		csi_config = 0;
	}
	else if (update_type == MSM_SENSOR_UPDATE_PERIODIC) 
	{
		CDBG("PERIODIC : %d\n", res);
		//msm_sensor_write_conf_array(
		//	s_ctrl->sensor_i2c_client,
		//	s_ctrl->msm_sensor_reg->mode_settings, res);
		msleep(30);
		#if 0
		if (!csi_config) {
            SKYCDBG("%s: ==> MIPI setting  E %d\n", __func__, update_type);
			s_ctrl->curr_csic_params = s_ctrl->csic_params[res];
			CDBG("CSI config in progress\n");
			v4l2_subdev_notify(&s_ctrl->sensor_v4l2_subdev,
				NOTIFY_CSIC_CFG,
				s_ctrl->curr_csic_params);
			CDBG("CSI config is done\n");
			mb();
			msleep(30);
			csi_config = 1;
            SKYCDBG("%s: ==> MIPI setting  X %d\n", __func__, update_type);	
		}
		v4l2_subdev_notify(&s_ctrl->sensor_v4l2_subdev,
			NOTIFY_PCLK_CHANGE,
			&s_ctrl->sensordata->pdata->ioclk.vfe_clk_rate);
		#else
		if (!csi_config)
		{
			icp_hd_mipi_init(s_ctrl, update_type, res);
			csi_config = 1;
		}

		v4l2_subdev_notify(&s_ctrl->sensor_v4l2_subdev, NOTIFY_PCLK_CHANGE, &s_ctrl->sensordata->pdata->ioclk.vfe_clk_rate);
		#endif

		//s_ctrl->func_tbl->sensor_start_stream(s_ctrl);
		msleep(50);
#if 1//needtocheck		
        switch (res) {
            case 0:
                s_ctrl->func_tbl->sensor_start_stream(s_ctrl);
                msleep(50);
            
//              rc = ce1612_snapshot_config(s_ctrl);    
                //SKYCDBG("%s: si2c_write_param(SI2C_SA, SI2C_PREVIEW, icp_hd_params) / CALL\n",__func__);
                //rc = si2c_write_param(SI2C_SA, SI2C_SNAPSHOT, icp_hd_params);
		  if(!b_snapshot_flag) {
#ifdef F_PANTECH_CAMERA_FIX_CFG_LED_MODE
			if(led_auto == ICP_HD_CFG_LED_MODE_AUTO)
			{	
				rc = icp_hd_i2c_read(icp_hd_client->addr,	0x400A , &nbrightness,WORD_LEN);
				SKYCDBG("%s FLASH_AUTO_MODE SNAP nbrightness =0x%x\n",__func__, nbrightness);
				if((nbrightness <= 0x0200)||(nbrightness >= 0x8000))
				{
					icp_hd_sensor_set_led_mode(s_ctrl, ICP_HD_CFG_LED_MODE_SNAP);

					//AF, AWB stable time for FLASH auto mode
					rc = icp_hd_i2c_write_table(&icp_hd_tune_regs.half_sutter_cfg_settings[0],
							icp_hd_tune_regs.half_sutter_cfg_settings_size);
					for (i = 0; i < ICP_HD_LED_HALF_SHUTTER_POLLING_RETRY; i++) 
					{
						msleep(ICP_HD_HALF_SHUTTER_POLLING_PERIOD);
						
						rc = icp_hd_i2c_read(icp_hd_client->addr, 0x1000, &read_buf, WORD_LEN);
						if (rc < 0)
						{
							SKYCERR("LED HALF SHUTTER POLLING READ I2C FAIL!!! return~~ rc=%d\n", rc);
							return rc;
						}

						//SKYCDBG("LED HALF SHUTTER polling num = %d,  read_buf = %x\n", i, read_buf);

						if((read_buf & 0x0010) == ICP_HD_LED_HALF_SHUTTER_COMPLETE_REG)
							break;
					}
				}
			}
#endif
#ifdef F_PANTECH_CAMERA_ADD_CFG_ANTISHAKE
			if((is_antishake == ICP_HD_CFG_ANTISHAKE_ON) && (led_auto != ICP_HD_CFG_LED_MODE_AUTO))
			{
					//AF, AWB stable time for FLASH auto mode
					rc = icp_hd_i2c_write_table(&icp_hd_tune_regs.half_sutter_cfg_settings[0],
							icp_hd_tune_regs.half_sutter_cfg_settings_size);
				for (i = 0; i < ICP_HD_ANTISHAKE_HALF_SHUTTER_POLLING_RETRY; i++) 
					{
						msleep(ICP_HD_HALF_SHUTTER_POLLING_PERIOD);
						
						rc = icp_hd_i2c_read(icp_hd_client->addr, 0x1000, &read_buf, WORD_LEN);
						if (rc < 0)
						{
							SKYCERR("ANTISHAKE HALF SHUTTER POLLING READ I2C FAIL!!! return~~ rc=%d\n", rc);
							return rc;
						}

						//SKYCDBG("ANTISHAKE HALF SHUTTER polling num = %d,  read_buf = %x\n", i, read_buf);

						if((read_buf & 0x0010) == ICP_HD_LED_HALF_SHUTTER_COMPLETE_REG)
							break;
				}
			}
#endif
			rc = icp_hd_snapshot_config();
                     b_snapshot_flag = true;
		  }                
                if (rc < 0) {
                    SKYCERR("%s snapshot config err(%d)\n", __func__, rc);
                    return rc;
                }

                SKYCDBG("Sensor setting : Case 0 snapshot config"); 
                break;

            case 1:
//              rc = ce1612_video_config(s_ctrl);
                //SKYCDBG("%s: si2c_write_param(SI2C_SA, SI2C_PREVIEW, icp_hd_params) / CALL\n",__func__);
                //rc = si2c_write_param(SI2C_SA, SI2C_PREVIEW, icp_hd_params);
                if((led_auto == ICP_HD_CFG_LED_MODE_AUTO)&&((nbrightness <= 0x0200)||(nbrightness >= 0x8000)))
                {
                    icp_hd_sensor_set_led_mode(s_ctrl, ICP_HD_CFG_LED_MODE_AUTO);
                }

          		if(b_1080p_flag)
          		{
 					rc = icp_hd_1080p_config(OFF);  
					if(rc < 0)
					{
						SKYCERR("%s icp_hd_1080p_config err(%d)\n", __func__, rc);
						return rc;
					}
					b_1080p_flag = false;
				}
				
                rc = icp_hd_video_config(b_snapshot_flag);
                b_snapshot_flag = false;
                if (rc < 0) {
                    SKYCERR("%s preview config err(%d)\n", __func__, rc);
                    return rc;
                }

                if(rc == 8192)  // workaround
                {
                    SKYCDBG("%s retry video config(%d)\n", __func__, rc);
                    s_ctrl->func_tbl->sensor_stop_stream(s_ctrl);
                    msleep(50);
                    if (sgpio_ctrl(sgpios, CAMIO_R_RST_N, 0) < 0)	rc = -EIO;//gpio_set_value(SENSOR_RESET,0);
                    msleep(5);
                    if (sgpio_ctrl(sgpios, CAMIO_R_RST_N, 1) < 0)	rc = -EIO;//gpio_set_value(SENSOR_RESET,1);
                    msleep(5);	
                    icp_hd_sensor_reg_init(s_ctrl);
                    msleep(50);
                    icp_hd_mipi_init(s_ctrl, update_type, res);
                    csi_config = 1;

	                if((led_auto == ICP_HD_CFG_LED_MODE_AUTO)&&((nbrightness <= 0x0200)||(nbrightness >= 0x8000)))
	                {
	                    icp_hd_sensor_set_led_mode(s_ctrl, ICP_HD_CFG_LED_MODE_AUTO);
	                }
                    
                    v4l2_subdev_notify(&s_ctrl->sensor_v4l2_subdev, NOTIFY_PCLK_CHANGE, &s_ctrl->sensordata->pdata->ioclk.vfe_clk_rate);

					icp_hd_sensor_params_info_restore(s_ctrl);
                    
                    msleep(50);
                    rc = icp_hd_video_config(b_snapshot_flag);
                }

                s_ctrl->func_tbl->sensor_start_stream(s_ctrl);
                msleep(50);

#if 1//for_test
                if(current_fps != 31)
                    icp_hd_sensor_set_preview_fps(s_ctrl ,current_fps);
#endif

                SKYCDBG("Sensor setting : Case 1 preview config");
                break;

          	case 2: 
                if(!b_1080p_flag)
                {
                    rc = icp_hd_1080p_config(ON);  

                    if(rc < 0)
                    {
                        SKYCERR("%s icp_hd_1080p_config err(%d)\n", __func__, rc);
                        return rc;
                    }
					b_1080p_flag = true;
                }

                rc = icp_hd_video_config(b_snapshot_flag);
                b_snapshot_flag = false;
                if (rc < 0) 
                {
                    SKYCERR("%s preview config err(%d)\n", __func__, rc);
                    return rc;
                }

                if(rc == 8192)  // workaround
                {
                    SKYCDBG("%s retry video config(%d)\n", __func__, rc);
                    s_ctrl->func_tbl->sensor_stop_stream(s_ctrl);
                    msleep(50);
                    if (sgpio_ctrl(sgpios, CAMIO_R_RST_N, 0) < 0)	rc = -EIO;//gpio_set_value(SENSOR_RESET,0);
                    msleep(5);
                    if (sgpio_ctrl(sgpios, CAMIO_R_RST_N, 1) < 0)	rc = -EIO;//gpio_set_value(SENSOR_RESET,1);
                    msleep(5);	
                    icp_hd_sensor_reg_init(s_ctrl);
                    msleep(50);
                    icp_hd_mipi_init(s_ctrl, update_type, res);
                    csi_config = 1;

	                if((led_auto == ICP_HD_CFG_LED_MODE_AUTO)&&((nbrightness <= 0x0200)||(nbrightness >= 0x8000)))
	                {
	                    icp_hd_sensor_set_led_mode(s_ctrl, ICP_HD_CFG_LED_MODE_AUTO);
	                }
                    
                    v4l2_subdev_notify(&s_ctrl->sensor_v4l2_subdev, NOTIFY_PCLK_CHANGE, &s_ctrl->sensordata->pdata->ioclk.vfe_clk_rate);
                    msleep(50);

                    icp_hd_sensor_params_info_restore(s_ctrl);

                    rc = icp_hd_1080p_config(ON);  

                    if(rc < 0)
                    {
                        SKYCERR("%s icp_hd_1080p_config err(%d)\n", __func__, rc);
                        return rc;
                    }
                    msleep(5);
                    rc = icp_hd_video_config(b_snapshot_flag);
                }

                s_ctrl->func_tbl->sensor_start_stream(s_ctrl);
                msleep(50);
                SKYCDBG("Sensor setting : Case 2 preview config [1080p]");
                break;

            case 3: 
//              rc = ce1612_ZSL_config(s_ctrl); 
                SKYCDBG("Sensor setting : Case 3 ZSL config");              
                break;  
                 
            default:
                rc = icp_hd_video_config(b_snapshot_flag);
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

#ifdef F_ICP_HD_POWER
/* icp_hd_vreg_init */
static int icp_hd_vreg_init(void)
{
    int rc = 0;
    SKYCDBG("%s E\n", __func__);
    
    rc = sgpio_init(sgpios, CAMIO_MAX);
    //SKYCDBG("%s sgpio_init \n", __func__);
    if (rc < 0)
        goto sensor_init_fail;

    rc = svreg_init(svregs, CAMV_MAX);
    //SKYCDBG("%s svreg_init \n", __func__);
    if (rc < 0)
       goto sensor_init_fail;
    
    SKYCDBG("%s X\n", __func__);
    return 0;
  
sensor_init_fail:
    pr_err("%s sensor_init_fail \n", __func__);
    svreg_release(svregs, CAMV_MAX); 
    sgpio_release(sgpios, CAMIO_MAX);
    return -ENODEV;
}

int icp_hd_sensor_release(void)
{
	int rc = 0;
	
	CDBG("%s start\n",__func__);
#ifdef F_PANTECH_CAMERA_FIX_CFG_LED_MODE	
	nbrightness = 0;
	if(is_led_work != ICP_HD_CFG_LED_MODE_OFF)
           icp_hd_sensor_set_led_mode(NULL, ICP_HD_CFG_LED_MODE_OFF);
#endif	
#ifdef F_PANTECH_CAMERA_FIX_CFG_SCENE_MODE
       old_scene_value = ICP_HD_CFG_SCENE_MODE_OFF;
#endif
#ifdef F_PANTECH_CAMERA_ADD_CFG_ANTISHAKE
	is_antishake = ICP_HD_CFG_ANTISHAKE_OFF;
#endif	

#ifdef F_PANTECH_CAMERA_TUP_LOAD_FILE
	//tunning value loading check flag
	icp_hd_tup_state = ICP_HD_TUNE_STATE_NONE;
	//icp_hd_done_set_tune_load = FALSE;
	//icp_hd_done_set_tune_value = FALSE;
	//icp_hd_tup_mode = ICP_HD_TUNE_STATE_TUNNING_MODE_OFF;
#endif

	CDBG("%s end rc = %d\n",__func__, rc);
	return rc;
}
/* msm_sensor_power_up */
//icp_hd_sensor_power_up
int32_t icp_hd_sensor_power_up(struct msm_sensor_ctrl_t *s_ctrl)
{
    int32_t rc = 0;
    CDBG("%s E\n", __func__);

#if 0
	memset(icp_hd_params, 0, sizeof(icp_hd_params));

    rc = si2c_init(icp_hd_client->adapter, 
            icp_hd_const_params, icp_hd_params);
#endif    

    rc = msm_sensor_power_up(s_ctrl);
    CDBG(" %s : msm_sensor_power_up : rc = %d E\n",__func__, rc);  

   icp_hd_vreg_init(); 

    if (sgpio_ctrl(sgpios, CAMIO_R_STB_N, 0) < 0)	rc = -EIO;
    if (svreg_ctrl(svregs, CAM1V_S2_1P2V, 1) < 0)	rc = -EIO;
    mdelay(1);//msleep(1);
    if (svreg_ctrl(svregs, CAM1V_IO_1P8V, 1) < 0)	rc = -EIO;
    mdelay(1);//msleep(1);
    if (svreg_ctrl(svregs, CAM1V_LVS3_1P8V, 1) < 0)	rc = -EIO;
    mdelay(1);//msleep(1);
    if (svreg_ctrl(svregs, CAM1V_L2B_2P8V, 1) < 0) rc = -EIO;
    mdelay(1);//msleep(1);
    if (svreg_ctrl(svregs, CAM1V_L3_2P8V, 1) < 0) rc = -EIO;
  
    //msm_camio_clk_rate_set(24000000);     //build error : undefined reference
#if 1
SKYCDBG("%s: [wsyang_debug] msm_cam_clk_enable() / 1 \n", __func__);

msm_cam_clk_enable(&s_ctrl->sensor_i2c_client->client->dev,
    cam_mclk_info, &s_ctrl->cam_clk, ARRAY_SIZE(cam_mclk_info), 1);
#endif
    mdelay(5);//msleep(5);

    if (sgpio_ctrl(sgpios, CAMIO_R_RST_N, 0) < 0)   rc = -EIO;
    if (sgpio_ctrl(sgpios, CAMIO_R_RST_N, 1) < 0)	rc = -EIO;
    mdelay(5);//msleep(5);

    current_fps = 31;

    icp_hd_sensor_params_info_init();

    CDBG("%s X (%d)\n", __func__, rc);
    return rc;
}

int32_t icp_hd_sensor_power_down(struct msm_sensor_ctrl_t *s_ctrl)
{
    int32_t rc = 0;

    CDBG("%s E\n", __func__);
    icp_hd_sensor_release();

#if 1
    msm_sensor_power_down(s_ctrl);
    CDBG(" %s : msm_sensor_power_down : rc = %d E\n",__func__, rc);
    
    if (sgpio_ctrl(sgpios, CAMIO_R_RST_N, 0) < 0)   rc = -EIO;
    if (sgpio_ctrl(sgpios, CAMIO_R_STB_N, 0) < 0)   rc = -EIO;
     
     /* MCLK will be disabled once again after this. */
    //  (void)msm_camio_clk_disable(CAMIO_CAM_MCLK_CLK);
    if (svreg_ctrl(svregs, CAM1V_S2_1P2V, 0) < 0)    rc = -EIO;
    if (svreg_ctrl(svregs, CAM1V_LVS3_1P8V, 0) < 0)  rc = -EIO;
    if (svreg_ctrl(svregs, CAM1V_IO_1P8V, 0) < 0) rc = -EIO;
    if (svreg_ctrl(svregs, CAM1V_L2B_2P8V, 0) < 0) rc = -EIO;
    if (svreg_ctrl(svregs, CAM1V_L3_2P8V, 0) < 0) rc = -EIO;

    svreg_release(svregs, CAMV_MAX);
    sgpio_release(sgpios, CAMIO_MAX);
#else
CDBG("%s SKIP!! \n", __func__);

#endif    
    //si2c_release();    
    CDBG("%s X (%d)\n", __func__, rc);
    return rc;
}
#endif

static int __init msm_sensor_init_module(void)
{
//    CDBG("%s E\n", __func__);
	return i2c_add_driver(&icp_hd_i2c_driver);
}

static struct v4l2_subdev_core_ops icp_hd_subdev_core_ops = {
	.ioctl = msm_sensor_subdev_ioctl,
	.s_power = msm_sensor_power,
};

static struct v4l2_subdev_video_ops icp_hd_subdev_video_ops = {
	.enum_mbus_fmt = msm_sensor_v4l2_enum_fmt,
};

static struct v4l2_subdev_ops icp_hd_subdev_ops = {
	.core = &icp_hd_subdev_core_ops,
	.video  = &icp_hd_subdev_video_ops,
};

static struct msm_sensor_fn_t icp_hd_func_tbl = {
#if 1//def F_STREAM_ON_OFF
	.sensor_start_stream = icp_hd_sensor_start_stream,//msm_sensor_start_stream,
	.sensor_stop_stream = icp_hd_sensor_stop_stream,//msm_sensor_stop_stream,
#endif
#if 0
	.sensor_group_hold_on = msm_sensor_group_hold_on,
	.sensor_group_hold_off = msm_sensor_group_hold_off,
	.sensor_set_fps = msm_sensor_set_fps,
	.sensor_write_exp_gain = msm_sensor_write_exp_gain1,
	.sensor_write_snapshot_exp_gain = msm_sensor_write_exp_gain1,
#endif
	.sensor_setting = icp_hd_sensor_setting, //icp_hd_sensor_setting
	.sensor_csi_setting = icp_hd_sensor_setting1,//msm_sensor_setting1,
	.sensor_set_sensor_mode = msm_sensor_set_sensor_mode,
	.sensor_mode_init = msm_sensor_mode_init,
	.sensor_get_output_info = msm_sensor_get_output_info,
	.sensor_config = msm_sensor_config,
#ifdef F_ICP_HD_POWER
    .sensor_power_up = icp_hd_sensor_power_up,//msm_sensor_power_up,
    .sensor_power_down = icp_hd_sensor_power_down,//msm_sensor_power_down,
#else
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = msm_sensor_power_down,
#endif
//	.sensor_adjust_frame_lines = msm_sensor_adjust_frame_lines,
	.sensor_get_csi_params = msm_sensor_get_csi_params,
//icp_hd_set_function
#ifdef CONFIG_PANTECH_CAMERA
#ifdef F_PANTECH_CAMERA_FIX_CFG_BRIGHTNESS
    .sensor_set_brightness = icp_hd_sensor_set_brightness,
#endif
    .sensor_set_effect = icp_hd_sensor_set_effect,
#ifdef F_PANTECH_CAMERA_FIX_CFG_EXPOSURE
    .sensor_set_exposure_mode = icp_hd_sensor_set_exposure_mode,
#endif
#ifdef F_PANTECH_CAMERA_FIX_CFG_WB
    .sensor_set_wb = icp_hd_sensor_set_wb,
#endif
#ifdef F_PANTECH_CAMERA_FIX_CFG_PREVIEW_FPS
    .sensor_set_preview_fps = icp_hd_sensor_set_preview_fps,
#endif
#ifdef F_PANTECH_CAMERA_FIX_CFG_REFLECT
    .sensor_set_reflect = icp_hd_sensor_set_reflect,    
#endif    
#ifdef F_PANTECH_CAMERA_FIX_CFG_AF
    .sensor_set_auto_focus = icp_hd_sensor_set_auto_focus,
    .sensor_check_af = icp_hd_sensor_check_af,
#endif
#ifdef F_PANTECH_CAMERA_FIX_CFG_FOCUS_RECT
    .sensor_set_focus_rect = icp_hd_sensor_set_focus_rect,
#endif
#ifdef F_PANTECH_CAMERA_FIX_CFG_LED_MODE
    .sensor_set_led_mode = icp_hd_sensor_set_led_mode,
#endif
#ifdef F_PANTECH_CAMERA_FIX_CFG_SCENE_MODE
//    .sensor_get_scenes_supported = icp_hd_get_scenes_supported,
    .sensor_set_scene_mode = icp_hd_sensor_set_scene_mode,
#endif
#ifdef F_PANTECH_CAMERA_FIX_CFG_ANTIBANDING
    .sensor_set_antibanding = icp_hd_sensor_set_antibanding,
#endif
#ifdef F_PANTECH_CAMERA_ADD_CFG_ANTISHAKE
    .sensor_set_antishake = icp_hd_sensor_set_antishake,
#endif
#endif //CONFIG_PANTECH_CAMERA
};

static struct msm_sensor_reg_t icp_hd_regs = {
	.default_data_type = MSM_CAMERA_I2C_WORD_DATA, //MSM_CAMERA_I2C_BYTE_DATA,
#if 0//def F_STREAM_ON_OFF	        
	.start_stream_conf = icp_hd_start_settings,
	.start_stream_conf_size = ARRAY_SIZE(icp_hd_start_settings),
	.stop_stream_conf = icp_hd_stop_settings,
	.stop_stream_conf_size = ARRAY_SIZE(icp_hd_stop_settings),
#endif
#if 0
	.group_hold_on_conf = icp_hd_groupon_settings,
	.group_hold_on_conf_size = ARRAY_SIZE(icp_hd_groupon_settings),
	.group_hold_off_conf = icp_hd_groupoff_settings,
	.group_hold_off_conf_size =
		ARRAY_SIZE(icp_hd_groupoff_settings),
#endif
	.init_settings = NULL,//&icp_hd_init_conf[0],//NULL,//&icp_hd_init_conf[0],
	.init_size = 0,//ARRAY_SIZE(icp_hd_init_conf),//0,//ARRAY_SIZE(icp_hd_init_conf),
	.mode_settings = NULL,//&icp_hd_confs[0],//NULL,//&icp_hd_confs[0],
	.output_settings = &icp_hd_dimensions[0],
	.num_conf = ARRAY_SIZE(icp_hd_cid_cfg),//ARRAY_SIZE(icp_hd_confs),
	//include reg settings
};

static struct msm_sensor_ctrl_t icp_hd_s_ctrl = {
	.msm_sensor_reg = &icp_hd_regs,
	.sensor_i2c_client = &icp_hd_sensor_i2c_client,
	.sensor_i2c_addr = 0x7A,//0x5A,//0xB4,//0x5A,
//	.sensor_output_reg_addr = &icp_hd_reg_addr,
	.sensor_id_info = &icp_hd_id_info,  //not use matchid
//	.sensor_exp_gain_info = &icp_hd_exp_gain_info,
	.cam_mode = MSM_SENSOR_MODE_INVALID,
	.csic_params = &icp_hd_csic_params_array[0],
	.csi_params = &icp_hd_csi_params_array[0],
	.msm_sensor_mutex = &icp_hd_mut,
	.sensor_i2c_driver = &icp_hd_i2c_driver,
	.sensor_v4l2_subdev_info = icp_hd_subdev_info,  
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(icp_hd_subdev_info),
	.sensor_v4l2_subdev_ops = &icp_hd_subdev_ops,
	.func_tbl = &icp_hd_func_tbl,
	.clk_rate = MSM_SENSOR_MCLK_24HZ,
};

late_initcall(msm_sensor_init_module);//module_init//late_initcall
MODULE_DESCRIPTION("Sensor icp_hd (ISP 8M) SoC Sensor Driver");
MODULE_LICENSE("GPL v2");

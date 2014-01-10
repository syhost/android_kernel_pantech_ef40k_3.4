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

#include "ce1612_v4l2_cfg.h"

#define F_FW_UPDATE
#define F_MIPI2LANE
#define F_STREAM_ON_OFF
//#define I2C_LOG_PRINT

#define ON  1
#define OFF 0

#ifdef F_FW_UPDATE 
#define CE1612_UPLOADER_INFO_F   "/CE161F00.bin"
#define CE1612_UPLOADER_BIN_F    "/CE161F01.bin"
#define CE1612_FW_INFO_F         "/CE161F02.bin"
#define CE1612_FW_BIN_F          "/CE161F03.bin"
#endif

#define CE1612_FW_MAJOR_VER	17//49
#define CE1612_FW_MINOR_VER	21//7
#define CE1612_PRM_MAJOR_VER	17//49
#define CE1612_PRM_MINOR_VER	21//7

struct ce1612_ver_t {
	uint8_t fw_major_ver;
	uint8_t fw_minor_ver;
	uint8_t prm_major_ver;
	uint8_t prm_minor_ver;
};

#ifdef F_FW_UPDATE
static struct ce1612_ver_t ce1612_ver = {0, 0};
#endif

#define SENSOR_NAME "ce1612"
#define PLATFORM_DRIVER_NAME "msm_camera_ce1612"
#define ce1612_obj ce1612_##obj

/*=============================================================
	SENSOR REGISTER DEFINES
==============================================================*/

/* Sensor Model ID */
#define CE1612_PIDH_REG		0x00
#define CE1612_MODEL_ID		1612

//wsyang_temp
#define F_CE1612_POWER

#ifdef F_CE1612_POWER
//#define 8M_CAM_RESET

#ifdef CONFIG_PANTECH_CAMERA_MT9D113
#define C_PANTECH_CAMERA_EF40_MT9D113_STANDBY
#endif

//needtocheck
#if 1//def CONFIG_PANTECH_CAMERA//IRQ
static uint32_t ce1612_irq_stat = 0;
#ifdef AF_STATE_POLLING
static uint32_t current_af_state = 0;
#endif
static int32_t caf_camera_flag = 0;
//static bool b_snapshot_flag;
#endif
static uint16_t f_stop_capture = 0;	// test cts

#define CAM1_RST_N			0
#define CAM1_STB_N			1
#define CAM1_1P1V_LOWQ      2
#define CAM1_VDD_EN         3
#define FLASH_EN            4
#ifdef C_PANTECH_CAMERA_EF40_MT9D113_STANDBY
#define CAM2_RST_N				5
#define CAM2_STB_N				6
#define CAMIO_MAX               7
#else
#define CAMIO_MAX               5
#endif

#if 1
static struct msm_cam_clk_info cam_mclk_info[] = {
	{"cam_clk", MSM_SENSOR_MCLK_24HZ},
};
#endif

#define SENSOR_RESET 106                //8M_CAM_RESET
#ifdef CONFIG_MACH_MSM8X60_EF65L
#define SENSOR_STANDBY 62       //8M_CAM_STANBY for 65L
#else
#define SENSOR_STANDBY 46               //8M_CAM_STANBY for 40
#endif
#define SENSOR_1P1V_LOWQ 124            //8M_1.1V_LOWQ
#define VDD_CORE_EN 15                  //8M_CAM_1.1V_EN
#ifdef C_PANTECH_CAMERA_EF40_MT9D113_STANDBY
#define MT9D113_RESET 137               //VGA_CAM_RESET
#define MT9D113_STANDBY 139             //VGA_CAM_STANDBY
#endif
#if 1 //def F_PANTECH_CAMERA_FIX_CFG_LED_MODE
#define CE1612_LED_FLASH_ENABLE_GPIO 31 //FLASH_EN3
#endif
#define CAMERA_BESTSHOT_MAX	21
static int ce1612_scenemode[CAMERA_BESTSHOT_MAX] = {
	CE1612_CFG_SCENE_MODE_OFF, CE1612_CFG_SCENE_MODE_AUTO, 
	CE1612_CFG_SCENE_MODE_LANDSCAPE, CE1612_CFG_SCENE_MODE_WINTER,
	CE1612_CFG_SCENE_MODE_BEACH, CE1612_CFG_SCENE_MODE_SUNSET,
	CE1612_CFG_SCENE_MODE_NIGHT, CE1612_CFG_SCENE_MODE_PORTRAIT,
       -1  /*BACKLIGHT*/, CE1612_CFG_SCENE_MODE_SPORTS,
       -1  /*ANTISHAKE*/,  -1 /*FLOWERS*/,
       -1  /*CANDLELIGHT*/,  -1  /*FIREWORKS*/,
       CE1612_CFG_SCENE_MODE_PARTY, -1 /*NIGHT_PORTRAIT*/,
       -1  /*THEATRE*/, -1  /*ACTION*/,
       -1  /*AR*/, CE1612_CFG_SCENE_MODE_INDOOR,
       CE1612_CFG_SCENE_MODE_TEXT
};
static sgpio_ctrl_t sgpios[CAMIO_MAX] = {
	{CAM1_RST_N, "8M_CAM_RESET", SENSOR_RESET},
	{CAM1_STB_N, "8M_CAM_STANBY", SENSOR_STANDBY},
	{CAM1_1P1V_LOWQ, "8M_CAM_1P1V_LOWQ", SENSOR_1P1V_LOWQ},
	{CAM1_VDD_EN, "8M_CAM_1P1V_EN", VDD_CORE_EN},
	{FLASH_EN, "FLASH_EN3", CE1612_LED_FLASH_ENABLE_GPIO},	
#ifdef C_PANTECH_CAMERA_EF40_MT9D113_STANDBY	
	{CAM2_RST_N, "2M_CAM_RESET", MT9D113_RESET},
	{CAM2_STB_N, "2M_CAM_STANBY", MT9D113_STANDBY},	
#endif
};


#if defined(CONFIG_MACH_MSM8X60_EF39S) || \
	defined(CONFIG_MACH_MSM8X60_EF40S) || \
    defined(CONFIG_MACH_MSM8X60_EF40K)     

#define CAMV_AVDD_2P8V	0
#define CAMV_VDD_HOSTIO_1P8 1
#define CAMV_VDD_SYSIO_2P8 2
#define CAMV_IOVDD_1P8 3
#define CAMV_VDD_AF_2P8 4
#ifdef C_PANTECH_CAMERA_EF40_MT9D113_STANDBY
#define CAM2V_CORE_1P8V 5
#define CAM2V_A_2P8V    6
#define CAMV_MAX    7
#else
#define CAMV_MAX	5
#endif

static svreg_ctrl_t svregs[CAMV_MAX] = {
    {CAMV_AVDD_2P8V, "8058_l20", NULL, 2800},
    {CAMV_VDD_HOSTIO_1P8, "8901_mvs0", NULL, 0},//1800},
    {CAMV_VDD_SYSIO_2P8, "8058_l3", NULL, 2800},
    {CAMV_IOVDD_1P8, "8901_lvs1", NULL, 0},//1800},
    {CAMV_VDD_AF_2P8, "8058_l14", NULL, 2800},
#ifdef C_PANTECH_CAMERA_EF40_MT9D113_STANDBY    
    {CAM2V_CORE_1P8V, "8901_lvs3", NULL, 0},//1800},
    {CAM2V_A_2P8V, "8058_l19", NULL, 2800},    
#endif
};

#elif defined(CONFIG_MACH_MSM8X60_EF65L) 

#define CAMV_AVDD_2P8V	0
#define CAMV_VDD_HOSTIO_1P8 1
#define CAMV_VDD_SYSIO_2P8 2
#define CAMV_IOVDD_1P8 3
#define CAMV_VDD_AF_2P8 4
#ifdef C_PANTECH_CAMERA_EF40_MT9D113_STANDBY
#define CAM2V_A_2P8V    5
#define CAMV_MAX    6
#else
#define CAMV_MAX	5
#endif

static svreg_ctrl_t svregs[CAMV_MAX] = {
    {CAMV_AVDD_2P8V, "8058_l20", NULL, 2800},
    {CAMV_VDD_HOSTIO_1P8, "8901_mvs0", NULL, 0},//1800},
    {CAMV_VDD_SYSIO_2P8, "8901_l4", NULL, 2800},
    {CAMV_IOVDD_1P8, "8901_lvs1", NULL, 0},//1800},
    {CAMV_VDD_AF_2P8, "8058_l15", NULL, 2800},
#ifdef C_PANTECH_CAMERA_EF40_MT9D113_STANDBY    
    {CAM2V_A_2P8V, "8058_l9", NULL, 2800},
#endif
};
#endif

#if 0
static struct regulator *l20a_2p8v;		//VREG_L20A_2.8V        //CAM_AVDD2.8V
static struct regulator *mvs0b_1p8v;	//VREG_MVS0B_1.8V       //VDD_HOSTIO1.8V
static struct regulator *l3a_2p8v;		//VREG_L3A_2.8V         //VDD_SYSIO2.8
static struct regulator *lvs1b_1p8v;	//VREG_LVS1B_1.8V       //CAM_IOVDD1.8V
static struct regulator *l14a_2p8v;		//VREG_L14A_2.8V        //VDD_AF2.8

#ifdef C_PANTECH_CAMERA_EF40_MT9D113_STANDBY
static struct regulator *lvs3b_1p8v;	//VREG_LVS3B_1.8V       //CAMV_CORE_1P8V
static struct regulator *l19a_2p8v;		//VREG_L19A_2.8V        //CAMV_A_2P8V

#define MT9D113_RESET 137               //VGA_CAM_RESET
#define MT9D113_STANDBY 139             //VGA_CAM_STANDBY
#endif
#endif

#endif

DEFINE_MUTEX(ce1612_mut);
static struct msm_sensor_ctrl_t ce1612_s_ctrl;

//needtocheck
#if 1
//static int32_t ce1612_lens_stop(struct msm_sensor_ctrl_t *s_ctrl);
//static int32_t ce1612_set_coninuous_af(struct msm_sensor_ctrl_t *s_ctrl ,int8_t caf);

static int32_t ce1612_lens_stability(struct msm_sensor_ctrl_t *s_ctrl);
static int32_t ce1612_set_caf_camera(struct msm_sensor_ctrl_t *s_ctrl, int32_t caf);
static int32_t ce1612_set_led_gpio_set(int8_t led_mode);

static int8_t continuous_af_mode = 0;   	// 0: no caf, 1: af-c, 2: af-t
static int8_t sensor_mode = -1;   			// 0: full size,  1: qtr size, 2: fullhd size, 3: ZSL
static int32_t x1 = 0, y1=0, x2=0, y2=0;  // 0: full size,  1: qtr size, 2: fullhd size
//static int8_t g_get_hw_revision = 0;
//extern int get_hw_revision(void);
static int8_t zsl_status = 1;
#endif
static int current_fps = 31;
static int ce1612_set_preview_fps(struct msm_sensor_ctrl_t *s_ctrl ,int8_t preview_fps);

#if 0//def F_STREAM_ON_OFF	
static struct msm_camera_i2c_reg_conf ce1612_start_settings[] = {
	{0x0100, 0x01},
};

static struct msm_camera_i2c_reg_conf ce1612_stop_settings[] = {
	{0x0100, 0x00},
};
#endif
#if 0
static struct msm_camera_i2c_reg_conf ce1612_groupon_settings[] = {
	{0x104, 0x01},
};

static struct msm_camera_i2c_reg_conf ce1612_groupoff_settings[] = {
	{0x104, 0x00},
};
#endif
#if 0
static struct msm_camera_i2c_reg_conf ce1612_prev_settings[] = {
	{0x0307, 0x2D}, /*pll_multiplier*/
	{0x0340, 0x06}, /*frame_length_lines_hi*/
	{0x0341, 0x34}, /*frame_length_lines_lo*/
	{0x0342, 0x11}, /*line_length_pclk_hi*/
	{0x0343, 0x78}, /*line_length_pclk_lo*/
	{0x0347, 0x00}, /*y_addr_start*/
	{0x034b, 0x2F}, /*y_add_end*/
	{0x034c, 0x08}, /*x_output_size_msb*/
	{0x034d, 0x38}, /*x_output_size_lsb*/
	{0x034e, 0x06}, /*y_output_size_msb*/
	{0x034f, 0x18}, /*y_output_size_lsb*/
	{0x0381, 0x01}, /*x_even_inc*/
	{0x0383, 0x03}, /*x_odd_inc*/
	{0x0385, 0x01}, /*y_even_inc*/
	{0x0387, 0x03}, /*y_odd_inc*/
	{0x3001, 0x80}, /*hmodeadd*/
	{0x3016, 0x16}, /*vmodeadd*/
	{0x3069, 0x24}, /*vapplinepos_start*/
	{0x306b, 0x53}, /*vapplinepos_end*/
	{0x3086, 0x00}, /*shutter*/
	{0x30e8, 0x80}, /*haddave*/
	{0x3301, 0x83}, /*lanesel*/
};

static struct msm_camera_i2c_reg_conf ce1612_snap_settings[] = {
	{0x0307, 0x26}, /*pll_multiplier*/
	{0x0340, 0x0C}, /*frame_length_lines_hi*/
	{0x0341, 0x90}, /*frame_length_lines_lo*/
	{0x0342, 0x11}, /*line_length_pclk_hi*/
	{0x0343, 0x78}, /*line_length_pclk_lo*/
	{0x0347, 0x00}, /*y_addr_start*/
	{0x034b, 0x2F}, /*y_add_end*/
	{0x034c, 0x10}, /*x_output_size_msb*/
	{0x034d, 0x70}, /*x_output_size_lsb*/
	{0x034e, 0x0c}, /*y_output_size_msb*/
	{0x034f, 0x30}, /*y_output_size_lsb*/
	{0x0381, 0x01}, /*x_even_inc*/
	{0x0383, 0x01}, /*x_odd_inc*/
	{0x0385, 0x01}, /*y_even_inc*/
	{0x0387, 0x01}, /*y_odd_inc*/
	{0x3001, 0x00}, /*hmodeadd*/
	{0x3016, 0x06}, /*vmodeadd*/
	{0x3069, 0x24}, /*vapplinepos_start*/
	{0x306b, 0x53}, /*vapplinepos_end*/
	{0x3086, 0x00}, /*shutter*/
	{0x30e8, 0x00}, /*haddave*/
	{0x3301, 0x03}, /*lanesel*/
};

static struct msm_camera_i2c_reg_conf ce1612_recommend_settings[] = {
	{0x0305, 0x02},
	{0x302b, 0x4B},
	{0x3024, 0x03},
	{0x0101, 0x00},
	{0x300a, 0x80},
	{0x3014, 0x08},
	{0x3015, 0x37},
	{0x301c, 0x01},
	{0x302c, 0x05},
	{0x3031, 0x26},
	{0x3041, 0x60},
	{0x3051, 0x24},
	{0x3053, 0x34},
	{0x3057, 0xc0},
	{0x305c, 0x09},
	{0x305d, 0x07},
	{0x3060, 0x30},
	{0x3065, 0x00},
	{0x30aa, 0x08},
	{0x30ab, 0x1c},
	{0x30b0, 0x32},
	{0x30b2, 0x83},
	{0x30d3, 0x04},
	{0x3106, 0x78},
	{0x310c, 0x82},
	{0x3304, 0x05},
	{0x3305, 0x04},
	{0x3306, 0x11},
	{0x3307, 0x02},
	{0x3308, 0x0c},
	{0x3309, 0x06},
	{0x330a, 0x08},
	{0x330b, 0x04},
	{0x330c, 0x08},
	{0x330d, 0x06},
	{0x330f, 0x01},
	{0x3381, 0x00},
};
#endif

static struct v4l2_subdev_info ce1612_subdev_info[] = {
	{
	.code   = V4L2_MBUS_FMT_YUYV8_2X8,//V4L2_MBUS_FMT_SBGGR10_1X10,
	.colorspace = V4L2_COLORSPACE_JPEG,
	.fmt    = 1,
	.order    = 0,
	},
	/* more can be supported, to be added later */
};

#if 0
static struct msm_camera_i2c_conf_array ce1612_init_conf[] = {
	{&ce1612_recommend_settings[0],
	ARRAY_SIZE(ce1612_recommend_settings), 0, MSM_CAMERA_I2C_BYTE_DATA}
};

static struct msm_camera_i2c_conf_array ce1612_confs[] = {
	{&ce1612_snap_settings[0],
	ARRAY_SIZE(ce1612_snap_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},
	{&ce1612_prev_settings[0],
	ARRAY_SIZE(ce1612_prev_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},
};
#endif

#if 0//ori
static struct msm_sensor_output_info_t ce1612_dimensions[] = {
	{
		.x_output = 0x1070,//4208
		.y_output = 0xC30,//3120
		.line_length_pclk = 0x1178,//4472
		.frame_length_lines = 0xC90,//3216
		.vt_pixel_clk = 182400000,
		.op_pixel_clk = 182400000,
		.binning_factor = 1,
	},
	{
		.x_output = 0x838,//2104
		.y_output = 0x618,//1560
		.line_length_pclk = 0x1178,//4472
		.frame_length_lines = 0x634,//1588
		.vt_pixel_clk = 216000000,
		.op_pixel_clk = 108000000,
		.binning_factor = 2,
	},
};


#else//MARUKO
#define C_PANTECH_CAMERA_MIN_PREVIEW_FPS	5
#define C_PANTECH_CAMERA_MAX_PREVIEW_FPS	31

#define CE1612_FULL_SIZE_DUMMY_PIXELS     0
#define CE1612_FULL_SIZE_DUMMY_LINES    	0
#define CE1612_FULL_SIZE_WIDTH    			3264
#define CE1612_FULL_SIZE_HEIGHT   			2448

#define CE1612_QTR_SIZE_DUMMY_PIXELS  	0
#define CE1612_QTR_SIZE_DUMMY_LINES   	0
#define CE1612_QTR_SIZE_WIDTH     		1280 
#define CE1612_QTR_SIZE_HEIGHT    		960 

#define CE1612_HRZ_FULL_BLK_PIXELS   	16 //0
#define CE1612_VER_FULL_BLK_LINES     	12 //0

#define CE1612_HRZ_QTR_BLK_PIXELS    	16 //0
#define CE1612_VER_QTR_BLK_LINES      	12 //0

#define CE1612_1080P_SIZE_DUMMY_PIXELS	0
#define CE1612_1080P_SIZE_DUMMY_LINES	0
#define CE1612_1080P_SIZE_WIDTH        1920
#define CE1612_1080P_SIZE_HEIGHT        1080

#define CE1612_HRZ_1080P_BLK_PIXELS      0
#define CE1612_VER_1080P_BLK_LINES        0

#define CE1612_ZSL_SIZE_WIDTH    2560 
#define CE1612_ZSL_SIZE_HEIGHT   1920 


static struct msm_sensor_output_info_t ce1612_dimensions[] = {
	{
		.x_output = CE1612_FULL_SIZE_WIDTH,
		.y_output = CE1612_FULL_SIZE_HEIGHT,
		.line_length_pclk = CE1612_FULL_SIZE_WIDTH + CE1612_HRZ_FULL_BLK_PIXELS ,
		.frame_length_lines = CE1612_FULL_SIZE_HEIGHT+ CE1612_VER_FULL_BLK_LINES ,
		.vt_pixel_clk = 198000000, //207000000, //276824064, 
		.op_pixel_clk = 198000000, //207000000, //276824064, 
		.binning_factor = 1,
	},
	{
		.x_output = CE1612_QTR_SIZE_WIDTH,
		.y_output = CE1612_QTR_SIZE_HEIGHT,
		.line_length_pclk = CE1612_QTR_SIZE_WIDTH + CE1612_HRZ_QTR_BLK_PIXELS,
		.frame_length_lines = CE1612_QTR_SIZE_HEIGHT+ CE1612_VER_QTR_BLK_LINES,
		.vt_pixel_clk = 198000000, //207000000, //276824064, 
		.op_pixel_clk = 198000000, //207000000, //276824064, 
		.binning_factor = 2,
	},
	{

		.x_output = CE1612_1080P_SIZE_WIDTH,
		.y_output = CE1612_1080P_SIZE_HEIGHT,
		.line_length_pclk = CE1612_1080P_SIZE_WIDTH + CE1612_HRZ_1080P_BLK_PIXELS ,
		.frame_length_lines = CE1612_1080P_SIZE_HEIGHT+ CE1612_VER_1080P_BLK_LINES ,
		.vt_pixel_clk = 198000000, //207000000,//276824064,  
		.op_pixel_clk = 198000000, //207000000,//276824064,  
		.binning_factor = 2,
	},
	{
		.x_output = CE1612_ZSL_SIZE_WIDTH,
		.y_output = CE1612_ZSL_SIZE_HEIGHT,
		.line_length_pclk = CE1612_ZSL_SIZE_WIDTH,
		.frame_length_lines = CE1612_ZSL_SIZE_HEIGHT,
		.vt_pixel_clk = 198000000, //207000000,//276824064,  
		.op_pixel_clk = 198000000, //207000000,//276824064, 
		.binning_factor = 1,
	},	
};
#endif

#if 1//0
static struct msm_camera_csi_params ce1612_csic_params = {
	.data_format = CSI_8BIT,//CSI_10BIT,
	.lane_cnt    = 2,//4,
	.lane_assign = 0xe4,
	.dpcm_scheme = 0,
	.settle_cnt  = 0x14,
};

static struct msm_camera_csi_params *ce1612_csic_params_array[] = {
	&ce1612_csic_params,
	&ce1612_csic_params,
#if 1//test
    &ce1612_csic_params,
    &ce1612_csic_params,
#endif
};
#endif

static struct msm_camera_csid_vc_cfg ce1612_cid_cfg[] = {
#if 1//
	{0, CSI_YUV422_8, CSI_DECODE_8BIT}, 
	{1, CSI_EMBED_DATA, CSI_DECODE_8BIT},
	{2, CSI_RESERVED_DATA_0, CSI_DECODE_8BIT},
	{3, CSI_RESERVED_DATA_0, CSI_DECODE_8BIT},	
#else
	{0, CSI_RAW10, CSI_DECODE_10BIT},
	{1, CSI_EMBED_DATA, CSI_DECODE_8BIT},
	{2, CSI_RESERVED_DATA_0, CSI_DECODE_8BIT},
#endif
};

static struct msm_camera_csi2_params ce1612_csi_params = {
	.csid_params = {
		//lane_assign = 0xe4,
#ifdef F_MIPI2LANE
		.lane_cnt = 2, 
#else
		.lane_cnt = 4,
#endif
		.lut_params = {
			.num_cid = ARRAY_SIZE(ce1612_cid_cfg),//2?
			.vc_cfg = ce1612_cid_cfg,
		},
	},
	.csiphy_params = {
#ifdef F_MIPI2LANE
		.lane_cnt = 2,
#else		
		.lane_cnt = 4,
#endif
		.settle_cnt = 0x14,//0x1B,
#ifdef F_MIPI2LANE
		//ane_mask = 0x3,
#endif
	},
};

static struct msm_camera_csi2_params *ce1612_csi_params_array[] = {
	&ce1612_csi_params,
	&ce1612_csi_params,
#if 1
	&ce1612_csi_params,
	&ce1612_csi_params,
#endif	
};

#if 0
static struct msm_sensor_output_reg_addr_t ce1612_reg_addr = {
	.x_output = 0x34C,
	.y_output = 0x34E,
	.line_length_pclk = 0x342,
	.frame_length_lines = 0x340,
};
#endif

static struct msm_sensor_id_info_t ce1612_id_info = {
	.sensor_id_reg_addr = CE1612_PIDH_REG,//0x0,
	.sensor_id = 0x0,//0x0074,
};

#if 0
static struct msm_sensor_exp_gain_info_t ce1612_exp_gain_info = {
	.coarse_int_time_addr = 0x202,
	.global_gain_addr = 0x204,
	.vert_offset = 3,
};
#endif

int32_t ce1612_sensor_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id);

static const struct i2c_device_id ce1612_i2c_id[] = {
	{SENSOR_NAME, (kernel_ulong_t)&ce1612_s_ctrl},
	{ }
};

static struct i2c_driver ce1612_i2c_driver = {
	.id_table = ce1612_i2c_id,
	.probe  = ce1612_sensor_i2c_probe, //msm_sensor_i2c_probe
	.driver = {
		.name = SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client ce1612_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,//MSM_CAMERA_I2C_WORD_ADDR,
};


static int32_t ce1612_cmd(struct msm_sensor_ctrl_t *s_ctrl , uint8_t cmd, uint8_t * ydata, int32_t num_data)
{
	int32_t rc = 0;
	unsigned char buf[130];
	int32_t i = 0;

	memset(buf, 0, sizeof(buf));

	buf[0] = cmd;

#ifdef I2C_LOG_PRINT
	SKYCDBG("++++ I2C W : \n");
	SKYCDBG("0x%x \n", buf[0]);
#endif

	if(ydata != NULL)
	{
		for(i = 0; i < num_data; i++)
		{
			buf[i+1] = *(ydata+i);
#ifdef I2C_LOG_PRINT
			SKYCDBG("0x%x \n", buf[i+1]);	
#endif
		}	
	}

#ifdef I2C_LOG_PRINT
	SKYCDBG("++++\n");
#endif

	rc = msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client, buf, num_data+1);

	if (rc < 0)
		SKYCERR("ERR:%s FAIL!!!cmd=%d , rc=%d return~~\n", __func__, cmd, rc);	

	return rc;
}


#ifdef F_FW_UPDATE
static int32_t ce1612_read(struct msm_sensor_ctrl_t *s_ctrl , uint8_t * rdata, uint32_t len)
{
	int32_t rc = 0;

#ifdef I2C_LOG_PRINT
	int32_t i = 0;
	SKYCDBG("---- I2C R : \n");
#endif

	rc = msm_camera_i2c_rxdata_2(s_ctrl->sensor_i2c_client, rdata, len);

#ifdef I2C_LOG_PRINT
	if(len != 0)
	{
		for(i = 0; i < len; i++)
		{
			SKYCDBG("0x%x \n", *(rdata+i));	
		}	
	}
	SKYCDBG("----\n");
#endif

	return rc;
}
#endif

static int32_t ce1612_cmd_read(struct msm_sensor_ctrl_t *s_ctrl , unsigned char cmd, uint8_t * rdata, uint32_t len)
{
	int32_t rc = 0;

#ifdef I2C_LOG_PRINT
	int32_t i = 0;
	SKYCDBG("++++ I2C W :\n");
	SKYCDBG("0x%x \n", cmd);
	SKYCDBG("++++\n");
	SKYCDBG("---- I2C R : \n");
#endif

	*rdata = cmd;
	rc = msm_camera_i2c_rxdata(s_ctrl->sensor_i2c_client, rdata, len);

#ifdef I2C_LOG_PRINT
	if(len != 0)
	{
		for(i = 0; i < len; i++)
		{
			SKYCDBG("0x%x \n", *(rdata+i));	
		}	
	}
	SKYCDBG("----\n");
#endif

	if(rc >= 0)
		rc = 0;

	return rc;
}


static int32_t ce1612_poll(struct msm_sensor_ctrl_t *s_ctrl , unsigned char cmd, uint8_t pdata, 
			   uint8_t mperiod, uint32_t retry)
{
	unsigned char rdata = 0;
	uint32_t i = 0;
	int32_t rc = 0;
	unsigned char tmp_raddr;

	for (i = 0; i < retry; i++) {
		rc = ce1612_cmd_read(s_ctrl, cmd, &tmp_raddr, 1);
		
		rdata = tmp_raddr;
		
#if 0        
		SKYCDBG("%s: (mperiod=%d, retry=%d) <%d>poll data = 0x%x, read = 0x%x\n", __func__, mperiod, retry, i, pdata, rdata);        
#endif

		if (rdata == pdata)
			break;
		msleep(mperiod);
	}

	if (i == retry) {
		SKYCERR("%s: err(-ETIMEDOUT)\n", __func__);
		rc = -ETIMEDOUT;
		return rc; 
	}

	return 0;
}


static int32_t ce1612_poll_bit(struct msm_sensor_ctrl_t *s_ctrl, uint8_t cmd, uint8_t mperiod, uint32_t retry)
{
	uint8_t rdata = 0;
	uint32_t i = 0;
	int32_t rc = 0;

	for (i = 0; i < retry; i++) {

		rc = ce1612_cmd_read(s_ctrl, cmd, &rdata, 1);
		
#if 0        
		SKYCDBG("%s: (mperiod=%d, retry=%d) <%d> read = 0x%x\n", __func__, mperiod, retry, i, rdata);        
#endif

		if (rc < 0)
			break;
		
		if (!(rdata & 0x01))
			break;
		msleep(mperiod);
	}

	if (i == retry) {
		SKYCERR("%s: err(-ETIMEDOUT)\n", __func__);
		rc = -ETIMEDOUT;
		return rc;//charley2
	}

	return 0;
}

#if 0//EF39ICS api
static uint16_t ce1612_saddr = 0x3E;

static int32_t ce1612_i2c_txdata(unsigned short saddr,
	unsigned char *txdata, int length)
{
	uint32_t i = 0;
	int32_t rc = 0;
	
	struct i2c_msg msg[] = {
		{
			.addr = saddr,
			.flags = 0,
			.len = length,
			.buf = txdata,
		},
	};

#ifdef F_PANTECH_CAMERA_I2C_LOG
	SKYCDBG("%s: tx. \n", __func__);
	for(i = 0; i < length; i++)
		SKYCDBG(" %02x.", *(txdata+i));
	SKYCDBG("\n");
#endif


#if SENSOR_DEBUG
	if (length == 2)
		SKYCDBG("msm_io_i2c_w: 0x%04x 0x%04x\n",
			*(u16 *) txdata, *(u16 *) (txdata + 2));
	else if (length == 4)
		SKYCDBG("msm_io_i2c_w: 0x%04x\n", *(u16 *) txdata);
	else
		SKYCDBG("msm_io_i2c_w: length = %d\n", length);
#endif
	for (i = 0; i < CE1612_I2C_RETRY; i++) {
		rc = i2c_transfer(ce1612_client->adapter, msg, 1); 
		if (rc >= 0) {			
			return 0;
		}
		SKYCDBG("%s: tx retry. [%02x.%02x.%02x] len=%d rc=%d\n", __func__,saddr, *txdata, *(txdata + 1), length, rc);
		msleep(CE1612_I2C_MPERIOD);
	}
	return -EIO;
}

static int ce1612_i2c_rxdata(uint16_t saddr, uint8_t *addr, 
				unsigned char *rdata, uint16_t len)
{
	uint32_t i = 0;
	int32_t rc = 0;
	
	struct i2c_msg msgs[] = {
	{
		.addr   = saddr,
		.flags = 0,
		.len   = 1,
		.buf   = addr,
	},
	{
		.addr   = saddr,
		.flags = I2C_M_RD,
		.len   = len,
		.buf   = rdata,
	},
	};
	
	for (i = 0; i < CE1612_I2C_RETRY; i++) {
		rc = i2c_transfer(ce1612_client->adapter, msgs, 2); 
		if (rc >= 0) {		
			
#ifdef F_PANTECH_CAMERA_I2C_LOG
{
	int j;
	SKYCDBG("%s: tx. \n", __func__);
	SKYCDBG(" %02x.", *(addr));
	SKYCDBG("\n");
	
	SKYCDBG("%s: rx. \n", __func__);
	for(j = 0; j < len; j++)
		SKYCDBG(" %02x.", *(rdata+j));
	SKYCDBG("\n");
}
#endif
			return 0;
		}
		SKYCERR("%s: rx retry. [%02x.%02x.xx] len=%d, rc=%d\n", 
			__func__, saddr, *addr, len, rc);
		msleep(CE1612_I2C_MPERIOD);
	}
	SKYCERR("%s: error. [%02x.%02x.xx] len=%d rc=%d\n", 
		__func__, saddr, *addr, len, rc);
	
	return -EIO;
}

static int ce1612_i2c_rxdata_2(uint16_t saddr, 
				unsigned char *rdata, uint16_t len)
{
	uint32_t i = 0;
	int32_t rc = 0;
	
	struct i2c_msg msgs[] = {
	{
		.addr   = saddr,
		.flags = I2C_M_RD,
		.len   = len,
		.buf   = rdata,
	},
	};
	
	for (i = 0; i < CE1612_I2C_RETRY; i++) {
		rc = i2c_transfer(ce1612_client->adapter, msgs, 1); 
		if (rc >= 0) {		
#ifdef F_PANTECH_CAMERA_I2C_LOG
{
	int j;
	SKYCDBG("%s: rx. \n", __func__);
	for(j = 0; j < len; j++)
		SKYCDBG(" %02x.", *(rdata+j));
	SKYCDBG("\n");
}
#endif
			return 0;
		}
		SKYCERR("%s: rx retry. [%02x.] len=%d, rc=%d\n", 
			__func__, saddr, len, rc);
		msleep(CE1612_I2C_MPERIOD);
	}
	SKYCERR("%s: error. [%02x.] len=%d rc=%d\n", 
		__func__, saddr, len, rc);
	
	return -EIO;
}

static int32_t ce1612_i2c_read(uint16_t saddr, uint8_t cmd, unsigned char *rdata)
{
	int32_t rc = 0;
	unsigned char tmp_raddr = 0;

	if (!rdata)
		return -EIO;

	tmp_raddr = cmd;

	rc = ce1612_i2c_rxdata(saddr, &tmp_raddr, rdata, 1);

	return rc;	
}

static int32_t ce1612_cmd_read(uint8_t cmd, uint8_t *rdata, int32_t len)
{
	int32_t rc = 0;
	unsigned char tmp_raddr = 0;

	if (!rdata)
		return -EIO;

	tmp_raddr = cmd;

      rc = ce1612_i2c_rxdata(ce1612_saddr, &tmp_raddr, rdata, (uint16_t)len);

      if (rc < 0)
      {
		SKYCDBG("ce1612_cmd_read I2C FAIL_2!!! return~~\n");
      } 		
	  
	//SKYCDBG("%s end : 0x%x register read data = 0x%x\n", __func__, raddr, rdata);

	return rc;
}

static int32_t ce1612_read(uint8_t *rdata, int32_t len)
{
	int32_t rc = 0;	

	rc = ce1612_i2c_rxdata_2(ce1612_saddr, rdata, (uint16_t)len);
	if (rc < 0)
	{
		SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
	}
	//SKYCDBG("%s end : 0x%x register read data = 0x%x\n", __func__, raddr, rdata);
	return rc; 
}

static int32_t ce1612_poll_bit(uint16_t saddr, uint8_t addr,
			   uint8_t mperiod, uint32_t retry)
{
	uint8_t rdata = 0;
	uint32_t i = 0;
	int32_t rc = 0;

	for (i = 0; i < retry; i++) {
		rc = ce1612_i2c_read(saddr, addr, &rdata);
		if (rc < 0)
			break;
		if (!(rdata & 0x01))
			break;

		msleep(mperiod);
	}

	if (i == retry) {
		SKYCERR("%s: -ETIMEDOUT, mperiod=%d, retry=%d\n", 
			__func__, mperiod, retry);
		rc = -ETIMEDOUT;
	}

	return rc;
}

#ifdef F_FW_UPDATE
static int32_t ce1612_poll(uint16_t saddr, uint8_t addr, uint8_t pdata, 
			   uint8_t mperiod, uint32_t retry)
{
	uint8_t rdata = 0;
	uint32_t i = 0;
	int32_t rc = 0;

	for (i = 0; i < retry; i++) {
		rc = ce1612_i2c_read(saddr, addr, &rdata);
		if (rc < 0)
			break;
		if (rdata == pdata)
			break;

		msleep(mperiod);
	}

	if (i == retry) {
		SKYCERR("%s: -ETIMEDOUT, mperiod=%d, retry=%d\n", 
			__func__, mperiod, retry);
		rc = -ETIMEDOUT;
	}

	return rc;
}
#endif

static int32_t ce1612_cmd(uint16_t saddr, uint8_t cmd, uint8_t * ydata, int32_t num_data)
{
	int32_t rc = 0;
	unsigned char buf[10];
	int32_t i = 0;
	
	memset(buf, 0, sizeof(buf));
	buf[0] = cmd;

	if(ydata != NULL)
	{
		for(i = 0; i < num_data; i++)
		{
			buf[i+1] = *(ydata+i);
		}	
	}
		
	rc = ce1612_i2c_txdata(saddr, buf, num_data+1);
	
	if (rc < 0)
		SKYCERR("ERR:%s FAIL!!!cmd=%d , rc=%d return~~\n", __func__, cmd, rc);	
	
	return rc;
}

static int32_t ce1612_cmd_poll(uint16_t saddr, uint8_t cmd, uint8_t * ydata, int32_t num_data)
{
	int32_t rc = 0;
	unsigned char buf[10];
	int32_t i = 0;
	
	memset(buf, 0, sizeof(buf));
	buf[0] = cmd;

	if(ydata != NULL)
	{
		for(i = 0; i < num_data; i++)
		{
			buf[i+1] = *(ydata+i);
		}	
	}
		
	rc = ce1612_i2c_txdata(saddr, buf, num_data+1);
	if (rc < 0)
		return rc;

	rc = ce1612_poll_bit(saddr, 0x24, 10, 400);
	
	return rc;
}
#endif


#if 1//def F_CE1612_POWER
/* ce1612_vreg_init */

/* msm_sensor_power_up */
//ce1612_sensor_power_up

/* msm_sensor_power_down */
//ce1612_sensor_power_down
#endif

/* msm_sensor_write_init_settings */
//ce1612_sensor_write_init_settings

/* msm_sensor_write_res_settings */
//ce1612_sensor_write_res_settings

/* msm_sensor_setting */
//ce1612_sensor_setting

/* sensor_set_function */
//ce1612_set_function

//...

#ifdef F_CE1612_POWER
static int ce1612_vreg_init(void)
{
	int rc = 0;
	SKYCDBG("%s: %d E\n", __func__, __LINE__);

	rc = sgpio_init(sgpios, CAMIO_MAX);
	if (rc < 0) {
		SKYCERR("%s: sgpio_init failed \n", __func__);
		goto sensor_init_fail;
	}

	rc = svreg_init(svregs, CAMV_MAX);
	if (rc < 0) {
		SKYCERR("%s: svreg_init failed \n", __func__);
        SKYCDBG("%s: SKYCDBG/ svreg_init failed \n", __func__);
		goto sensor_init_fail;
	}

	SKYCDBG("%s: X\n",__func__);
	return rc;

sensor_init_fail:
    return -ENODEV;
}


int32_t ce1612_sensor_power_up(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;

//needtocheck	g_get_hw_revision = get_hw_revision();
//needtocheck	SKYCERR("%s: g_get_hw_revision=%d E\n",__func__, g_get_hw_revision);

    f_stop_capture = 0;	// test cts

#if 0
	msm_sensor_probe_on(&s_ctrl->sensor_i2c_client->client->dev);   //////////////////
#else
	rc = msm_sensor_power_up(s_ctrl);
	SKYCDBG(" %s : msm_sensor_power_up : rc = %d E\n",__func__, rc);  
#endif    
	ce1612_vreg_init();

#if 0//MARUKO
	if (sgpio_ctrl(sgpios, CAM1_1P1V_LOWQ, 1) < 0)	rc = -EIO;	// VDD (CORE) 1.1V
	if (sgpio_ctrl(sgpios, CAM1_DVDD_EN, 1) < 0)	   rc = -EIO;	// VDD (CORE) 1.1V
	mdelay(3);

	if (sgpio_ctrl(sgpios, CAM1_VDD_EN, 1) < 0)		rc = -EIO;	// VDD_HOSTIO 1.8V 
	if (sgpio_ctrl(sgpios, CAM1_IOVDD_EN, 1) < 0)	rc = -EIO;	// VDD_CAMIO & CAM _IOVDD 1.8V 
	
	if(g_get_hw_revision <= TP10)
		if (sgpio_ctrl(sgpios, CAM1_AVDD_EN, 1) < 0)	   rc = -EIO;	// VDD_SYSIO 2.8V (TP10 : +AF 2.8V)

	if (svreg_ctrl(svregs, CAMV_VDD_SYSIO_2P8V, 1) < 0) rc = -EIO;	// CAM_AVDD 2.8V (TP20 : +VDD_SYSIO 2.8V)
	mdelay(5);

	if (sgpio_ctrl(sgpios, CAMIO_R_STB_N, 1) < 0)	rc = -EIO;	// ISP Standby
	mdelay(1);
	if (sgpio_ctrl(sgpios, CAMIO_R_RST_N, 1) < 0)	rc = -EIO;	// ISP Reset
	
	if(g_get_hw_revision  > TP10)
		if (sgpio_ctrl(sgpios, CAM1_AVDD_EN, 1) < 0)	rc = -EIO;     // AF 2.8V
	mdelay(10);

	continuous_af_mode = 0;
	sensor_mode = -1;
#endif

    //8M_CAM_1.1V_EN : VDD_CORE1.1V
    if (sgpio_ctrl(sgpios, CAM1_1P1V_LOWQ, 1) < 0)	rc = -EIO;
    mdelay(1);
    if (sgpio_ctrl(sgpios, CAM1_VDD_EN, 1) < 0)       rc = -EIO;
    mdelay(1);
    //VDD_HOSTIO1.8V
    if (svreg_ctrl(svregs, CAMV_VDD_HOSTIO_1P8, 1) < 0)		rc = -EIO;
    
#ifdef C_PANTECH_CAMERA_EF40_MT9D113_STANDBY
    mdelay(5);
#ifdef CONFIG_MACH_MSM8X60_EF65L
    if (svreg_ctrl(svregs, CAMV_IOVDD_1P8, 1) < 0)     rc = -EIO;
#else
    if (svreg_ctrl(svregs, CAM2V_CORE_1P8V, 1) < 0)		rc = -EIO;
#endif
    if (sgpio_ctrl(sgpios, CAM2_RST_N, 0) < 0)       rc = -EIO; 
    mdelay(5);
    if (sgpio_ctrl(sgpios, CAM2_RST_N, 1) < 0)       rc = -EIO; 
    mdelay(1);

    if (svreg_ctrl(svregs, CAM2V_A_2P8V, 1) < 0)		rc = -EIO;
    mdelay(5);
    if (sgpio_ctrl(sgpios, CAM2_STB_N, 1) < 0)       rc = -EIO; 
    mdelay(1);
#endif

    //CAM_SYSIO2.8V
    if (svreg_ctrl(svregs, CAMV_VDD_SYSIO_2P8, 1) < 0)		rc = -EIO;
    mdelay(1);
    //CAM_IOVDD1.8
    if (svreg_ctrl(svregs, CAMV_IOVDD_1P8, 1) < 0)		rc = -EIO;
    //CAM_AVDD2.8V
    if (svreg_ctrl(svregs, CAMV_AVDD_2P8V, 1) < 0)      rc = -EIO;
    //VDD_AF2.8
    if (svreg_ctrl(svregs, CAMV_VDD_AF_2P8, 1) < 0)      rc = -EIO;
    mdelay(1);

    /* Input MCLK = 24MHz */
    /*
    SKYCDBG(" msm_camio_clk_rate_set E\n");
    msm_camio_clk_rate_set(24000000);
    SKYCDBG(" msm_camio_clk_rate_set X\n");
    mdelay(5);
    */
#if 1//needtocheck
    pr_err("%s: [wsyang_debug] msm_cam_clk_enable() / 1 \n", __func__);
    
    msm_cam_clk_enable(&s_ctrl->sensor_i2c_client->client->dev,
        cam_mclk_info, &s_ctrl->cam_clk, ARRAY_SIZE(cam_mclk_info), 1);
    mdelay(5);
#endif

    //standby control
    if (sgpio_ctrl(sgpios, CAM1_STB_N, 1) < 0)       rc = -EIO; 
    mdelay(1);
    
    if (sgpio_ctrl(sgpios, CAM1_RST_N, 0) < 0)       rc = -EIO; 
    if (sgpio_ctrl(sgpios, CAM1_RST_N, 1) < 0)       rc = -EIO; 
#if 0
    rc = ce1612_reset(ON);
    if (rc < 0) {
        SKYCERR("reset failed!\n");
        goto fail;
    }
#endif
    mdelay(10);

    current_fps = 31;

//needtocheck	continuous_af_mode = 0;
//needtocheck	sensor_mode = -1;

	SKYCERR("%s X (%d)\n", __func__, rc);
	return rc;
}


int32_t ce1612_sensor_power_down(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;

//needtocheck	g_get_hw_revision = get_hw_revision();
//needtocheck	SKYCERR("%s: g_get_hw_revision=%d E\n",__func__, g_get_hw_revision);

#if 1 //def F_PANTECH_CAMERA_FIX_CFG_LED_MODE	
    ce1612_set_led_gpio_set(CE1612_CFG_LED_MODE_OFF);
#endif	
//needtocheck    ce1612_lens_stability(s_ctrl);

#if 0
	msm_sensor_probe_off(&s_ctrl->sensor_i2c_client->client->dev);  //////////////
#else
	msm_sensor_power_down(s_ctrl);
	SKYCDBG(" %s : msm_sensor_power_down : rc = %d E\n",__func__, rc);  
#endif

#if 0//MARUKO
	if(g_get_hw_revision  > TP10)
		if (sgpio_ctrl(sgpios, CAM1_AVDD_EN, 0) < 0) 	rc = -EIO;	// AF 2.8V

	if (sgpio_ctrl(sgpios, CAMIO_R_RST_N, 0) < 0)	rc = -EIO;	// ISP Reset
	mdelay(1);	
	if (sgpio_ctrl(sgpios, CAMIO_R_STB_N, 0) < 0)	rc = -EIO;	// ISP Standby
	mdelay(2);

	if (svreg_ctrl(svregs, CAMV_VDD_SYSIO_2P8V, 0) < 0)	rc = -EIO;	// CAM_AVDD 2.8V (TP20 : +VDD_SYSIO 2.8V)
	if (sgpio_ctrl(sgpios, CAM1_IOVDD_EN, 0) < 0)	      rc = -EIO;	// VDD_CAMIO & CAM _IOVDD 1.8V  
	if (sgpio_ctrl(sgpios, CAM1_VDD_EN, 0) < 0)		      rc = -EIO;	// VDD_HOSTIO 1.8V 

	if(g_get_hw_revision <= TP10)
		if (sgpio_ctrl(sgpios, CAM1_AVDD_EN, 0) < 0)          rc = -EIO;	// VDD_SYSIO 2.8V (TP10 : +AF 2.8V)
	mdelay(2);

	if (sgpio_ctrl(sgpios, CAM1_1P1V_LOWQ, 0) < 0)	rc = -EIO;	// VDD (CORE) 1.1V
	if (sgpio_ctrl(sgpios, CAM1_DVDD_EN, 0) < 0)	rc = -EIO;	   // VDD (CORE) 1.1V
	mdelay(10);
#endif

    //ce1612_reset(OFF);
    if (sgpio_ctrl(sgpios, CAM1_RST_N, 0) < 0)       rc = -EIO; 
    mdelay(1);
    //standby control
    if (sgpio_ctrl(sgpios, CAM1_STB_N, 0) < 0)       rc = -EIO; 
    mdelay(1);
    //VDD_AF2.8
    if (svreg_ctrl(svregs, CAMV_VDD_AF_2P8, 0) < 0) rc = -EIO;
    //CAM_AVDD2.8V
    if (svreg_ctrl(svregs, CAMV_AVDD_2P8V, 0) < 0) rc = -EIO;
    //CAM_IOVDD1.8
    if (svreg_ctrl(svregs, CAMV_IOVDD_1P8, 0) < 0) rc = -EIO;
    //CAM_SYSIO2.8V
    if (svreg_ctrl(svregs, CAMV_VDD_SYSIO_2P8, 0) < 0) rc = -EIO;
    
#ifdef C_PANTECH_CAMERA_EF40_MT9D113_STANDBY
    mdelay(1);
    if (sgpio_ctrl(sgpios, CAM2_STB_N, 0) < 0)       rc = -EIO; 
    
    if (svreg_ctrl(svregs, CAM2V_A_2P8V, 0) < 0)        rc = -EIO;
    mdelay(5);
    
    if (sgpio_ctrl(sgpios, CAM2_RST_N, 0) < 0)       rc = -EIO; 
    mdelay(1);
#ifdef CONFIG_MACH_MSM8X60_EF65L
    if (svreg_ctrl(svregs, CAMV_IOVDD_1P8, 0) < 0)     rc = -EIO;
#else
    if (svreg_ctrl(svregs, CAM2V_CORE_1P8V, 0) < 0)		rc = -EIO;
#endif
#endif
            
    //VDD_HOSTIO1.8V
    if (svreg_ctrl(svregs, CAMV_VDD_HOSTIO_1P8, 0) < 0)		rc = -EIO;
    mdelay(1);
    //8M_CAM_1.1V_EN : VDD_CORE1.1V
    if (sgpio_ctrl(sgpios, CAM1_VDD_EN, 0) < 0)       rc = -EIO;

    if (sgpio_ctrl(sgpios, CAM1_1P1V_LOWQ, 0) < 0)	rc = -EIO;

	svreg_release(svregs, CAMV_MAX);
	sgpio_release(sgpios, CAMIO_MAX);

 mdelay(10);

	SKYCERR("%s X (%d)\n", __func__, rc);
	return rc;
}
#endif
#ifdef F_FW_UPDATE
#define CE1612_FW_INFO_W_CMD 0xF2
#define CE1612_FW_DATA_W_CMD 0xF4
#define CE1612_FW_DATA_WP_CMD 0xF3
#define CE1612_NUM_CFG_CMD 4
#define CE1612_NUM_UPDATE_DATA 129
#define CE1612_PACKETS_IN_TABLE 508

static int32_t ce1612_update_fw(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	uint32_t numPacket = 0;
	int fd = 0;

	uint8_t pFW[CE1612_NUM_UPDATE_DATA+1];
	uint32_t i = 0;
	uint8_t rdata = 0xC0;
	uint8_t *pcmd = &pFW[0];
	uint8_t *pdata = &pFW[1];

	mm_segment_t old_fs; 

	old_fs = get_fs();
	set_fs(KERNEL_DS); 

	fd = sys_open(CE1612_UPLOADER_INFO_F, O_RDONLY, 0);
	if (fd < 0) {
		SKYCERR("%s: Can not open %s\n", __func__, CE1612_UPLOADER_INFO_F);
		goto fw_update_fail;
	}

	if (sys_read(fd, (char *)pdata, CE1612_NUM_CFG_CMD) != CE1612_NUM_CFG_CMD) {
		SKYCERR("%s: Can not read %s\n", __func__, CE1612_UPLOADER_INFO_F);
		sys_close(fd);
		goto fw_update_fail;
	}

	numPacket = (*(pdata+1) & 0xFF) << 8;
	numPacket |= *pdata & 0xFF;

	SKYCDBG("%s start : number of uploader packets is 0x%x\n",__func__, numPacket);

	*pcmd= CE1612_FW_INFO_W_CMD;
	//	rc = ce1612_cmd(s_ctrl, cmd, pFW, CE1612_NUM_CFG_CMD);
	rc = msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client, pFW, CE1612_NUM_CFG_CMD+1);
	if (rc < 0)
	{
		SKYCERR("%s : uploader configs write ERROR 0x%x, 0x%x, 0x%x, 0x%x, 0x%x!!!\n",__func__, pFW[0], pFW[1], pFW[2], pFW[3], pFW[4]);
		sys_close(fd);
		goto fw_update_fail;
	}
	sys_close(fd);
	SKYCDBG("%s : fw uploader info write OK !!!!\n",__func__);
	
	/////////////////////////////////////////////////////////////////////////////////

	fd = sys_open(CE1612_UPLOADER_BIN_F, O_RDONLY, 0);

	if (fd < 0) {
		SKYCERR("%s: Can not open %s\n", __func__, CE1612_UPLOADER_BIN_F);
		goto fw_update_fail;
	}

	for(i = 0; i < numPacket; i++)
	{
		if (sys_read(fd, (char *)pdata, CE1612_NUM_UPDATE_DATA) != CE1612_NUM_UPDATE_DATA) {
			SKYCERR("%s: Can not read %s : %d packet \n", __func__, CE1612_UPLOADER_BIN_F, i);
			sys_close(fd);
			goto fw_update_fail;
		}

		*pcmd= CE1612_FW_DATA_W_CMD;
		//	rc = ce1612_cmd(s_ctrl, cmd, pFW, CE1612_NUM_CFG_CMD);
		rc = msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client, pFW, CE1612_NUM_UPDATE_DATA+1);
		if (rc < 0)
		{
			SKYCERR("%s : uploader packet %d write ERROR !!!\n",__func__, i);
			sys_close(fd);
			goto fw_update_fail;
		}
		
		if(*pcmd == CE1612_FW_DATA_WP_CMD)
		{
			rc = ce1612_read(s_ctrl, &rdata, 1);
			if(rdata != 0)
			{
				SKYCERR("%s : uploader packet %d write ERROR [0xF3 = 0x%x]!!!\n",__func__, i, rdata);
				goto fw_update_fail;
			}
		}
	}
	sys_close(fd);
	SKYCDBG("%s : fw uploader data %d packets write OK !!!\n",__func__, i);

	msleep(5);

	rc = ce1612_poll(s_ctrl, 0xF5, 0x05, 10, 500);
	if (rc < 0)
	{
		SKYCERR("%s : uploader polling ERROR !!!\n",__func__);
		goto fw_update_fail;
	}
	/////////////////////////////////////////////////////////////////////////////////

	fd = sys_open(CE1612_FW_INFO_F, O_RDONLY, 0);
	if (fd < 0) {
		SKYCERR("%s: Can not open %s\n", __func__, CE1612_FW_INFO_F);
		goto fw_update_fail;
	}

	if (sys_read(fd, (char *)pdata, CE1612_NUM_CFG_CMD) != CE1612_NUM_CFG_CMD) {
		SKYCERR("%s: Can not read %s\n", __func__, CE1612_FW_INFO_F);
		sys_close(fd);
		goto fw_update_fail;
	}

	numPacket = (*(pdata+1) & 0xFF) << 8;
	numPacket |= *pdata & 0xFF;

	SKYCDBG("%s start : number of fw packets is 0x%x\n",__func__, numPacket);


	*pcmd= CE1612_FW_INFO_W_CMD;
	//	rc = ce1612_cmd(s_ctrl, cmd, pFW, CE1612_NUM_CFG_CMD);
	rc = msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client, pFW, CE1612_NUM_CFG_CMD+1);
	if (rc < 0)
	{
		SKYCERR("%s : FW configs write ERROR 0x%x, 0x%x, 0x%x, 0x%x, 0x%x!!!\n",__func__, pFW[0], pFW[1], pFW[2], pFW[3], pFW[4]);
		sys_close(fd);
		goto fw_update_fail;
	}
	sys_close(fd);
	SKYCDBG("%s : fw info write OK !!!!\n",__func__);
	
	/////////////////////////////////////////////////////////////////////////////////

	fd = sys_open(CE1612_FW_BIN_F, O_RDONLY, 0);

	if (fd < 0) {
		SKYCERR("%s: Can not open %s\n", __func__, CE1612_FW_BIN_F);
		goto fw_update_fail;
	}

	for(i = 0; i < numPacket; i++)
	{
		if (sys_read(fd, (char *)pdata, CE1612_NUM_UPDATE_DATA) != CE1612_NUM_UPDATE_DATA) {
		SKYCERR("%s: Can not read %s : %d packet \n", __func__, CE1612_FW_BIN_F, i);
		sys_close(fd);
		goto fw_update_fail;
		}

		*pcmd= CE1612_FW_DATA_W_CMD;
		//	rc = ce1612_cmd(s_ctrl, cmd, pFW, CE1612_NUM_CFG_CMD);
		rc = msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client, pFW, CE1612_NUM_UPDATE_DATA+1);
		if (rc < 0)
		{
			SKYCERR("%s : fw packet %d write ERROR !!!\n",__func__, i);
			sys_close(fd);
			goto fw_update_fail;
		}
		if(*pcmd == CE1612_FW_DATA_WP_CMD)
		{
			rc = ce1612_read(s_ctrl, &rdata, 1);
			if(rdata != 0)
			{
				SKYCERR("%s : fw packet %d write ERROR [0xF3 = 0x%x]!!!\n",__func__, i, rdata);
				goto fw_update_fail;
			}
		}
	}
	sys_close(fd);
	SKYCDBG("%s : fw data %d packets write OK !!!\n",__func__, i);
	/////////////////////////////////////////////////////////////////////////////////
	set_fs(old_fs);

	rc = ce1612_poll(s_ctrl, 0xF5, 0x06, 10, 3000);
	if (rc < 0)
	{
		SKYCERR("%s : fw data upgrade ERROR !!!\n",__func__);
		goto fw_update_fail;
	}

	return rc;	

fw_update_fail:
	set_fs(old_fs);
	SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);	
	return rc;
}


static int32_t ce1612_update_fw_boot(struct msm_sensor_ctrl_t *s_ctrl, const struct msm_camera_sensor_info *info)
{
	int32_t rc = 0;
	unsigned char data;
	unsigned char rdata[4];

#ifdef F_CE1612_POWER	
	rc = ce1612_sensor_power_up(s_ctrl);
	if (rc)
	{		
		SKYCERR(" ce1612_power failed rc=%d\n",rc);
		goto update_fw_boot_done; 
	}
#endif

	SKYCDBG("%s : Boot Start F0 for fw update !!\n", __func__);

	rc = ce1612_cmd(s_ctrl, 0xF0, NULL, 0);
	if (rc < 0)
	{
#if 1
		goto update_fw_boot_done;             
#else
		if(ce1612_saddr == 0x3C)
		ce1612_saddr = 0x3E;
		else
		ce1612_saddr = 0x3C;

		rc = ce1612_cmd(ce1612_saddr, 0xF0, NULL, 0);
		if (rc < 0)
		{
		SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
		goto update_fw_boot_done; 
		}
#endif
	}
	msleep(70);
	
	data = 0x00;
	rc = ce1612_cmd(s_ctrl, 0x00, &data, 1);
	if (rc < 0)
	{
		SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
		//		goto update_fw_boot_done; 
		goto update_fw_boot; 
	}

	rc = ce1612_read(s_ctrl, rdata, 2);
	if (rc < 0)
	{
		SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
		goto update_fw_boot; 
	}

	ce1612_ver.fw_minor_ver = rdata[0] & 0xFF;	  
	ce1612_ver.fw_major_ver = rdata[1] & 0xFF;
	SKYCDBG("%s : FW minor version=0x%x, FW major viersion=0x%x\n",__func__, ce1612_ver.fw_minor_ver, ce1612_ver.fw_major_ver);

	data = 0x01;
	rc = ce1612_cmd(s_ctrl, 0x00, &data, 1);
	if (rc < 0)
	{
		SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
		//		goto update_fw_boot_done; 
		goto update_fw_boot; 
	}

	rc = ce1612_read(s_ctrl, rdata, 2);
	if (rc < 0)
	{
		SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
		goto update_fw_boot; 
	}

	ce1612_ver.prm_minor_ver = rdata[0] & 0xFF;	  
	ce1612_ver.prm_major_ver = rdata[1] & 0xFF;  
	SKYCDBG("%s : jun PRM minor version=0x%x, PRM major viersion=0x%x\n",__func__, ce1612_ver.prm_minor_ver, ce1612_ver.prm_major_ver);

#if 0   // check AF ver.
	data = 0x05;
	rc = ce1612_cmd(s_ctrl, 0x00, &data, 1);
	if (rc < 0)
	{
	SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
	goto update_fw_boot; 
	}

	rc = ce1612_read(s_ctrl, rdata, 4);
	if (rc < 0)
	{
	SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
	goto update_fw_boot_done; 
	}
	SKYCDBG("%s : AF version[0]=0x%x, version[1]=0x%x, version[2]=0x%x, version[3]= 0x%x\n",__func__, rdata[0]&0xFF, rdata[1]&0xFF, rdata[2]&0xFF, rdata[3]&0xFF);
#endif

	if ((ce1612_ver.fw_major_ver == CE1612_FW_MAJOR_VER) && (ce1612_ver.fw_minor_ver == CE1612_FW_MINOR_VER)) {

		if ((ce1612_ver.prm_major_ver == CE1612_PRM_MAJOR_VER) &&
			(ce1612_ver.prm_minor_ver == CE1612_PRM_MINOR_VER)) {			
			SKYCDBG("%s : PRM minor version=0x%x, PRM major viersion=0x%x\n",__func__, CE1612_PRM_MINOR_VER, CE1612_PRM_MAJOR_VER);
			goto update_fw_boot_done;
		}		
	}


update_fw_boot: 
#ifdef F_CE1612_POWER	
	rc = ce1612_sensor_power_down(s_ctrl);
	if (rc) {
		SKYCERR(" ce1612_power failed rc=%d\n",rc);		
	}
	rc = ce1612_sensor_power_up(s_ctrl);
	if (rc) {		
		SKYCERR(" ce1612_power failed rc=%d\n",rc);
		goto update_fw_boot_done; 
	}
#endif
	rc = ce1612_update_fw(s_ctrl);

update_fw_boot_done:
#ifdef F_CE1612_POWER	
	SKYCDBG(" ce1612_sensor_release E\n");	
	rc = ce1612_sensor_power_down(s_ctrl);
	if (rc) {
		SKYCERR(" ce1612_power failed rc=%d\n",rc);		
	}
#endif

	return rc;
}
#endif
static int32_t ce1612_42_command(struct msm_sensor_ctrl_t *s_ctrl, uint8_t data)
{
    int32_t rc = 0;

	 SKYCDBG("%s: E\n",__func__);

    rc = ce1612_cmd(s_ctrl, 0x42, &data, 1);
    rc = ce1612_poll(s_ctrl, 0x43, data, 10, 100);
    if (rc < 0)
    {
		 SKYCERR("%s err(%d)\n", __func__, rc);
       return rc;
    }

    return rc;
}

static int32_t ce1612_lens_stop(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;

	SKYCDBG("%s: E\n",__func__);

	rc = ce1612_cmd(s_ctrl, 0x35, 0, 0);	// Lens Stop	
	if (rc < 0)
	{
		SKYCERR("%s err(%d)\n", __func__, rc);
		return rc;
	}
	
	rc = ce1612_poll_bit(s_ctrl, 0x24, 10, 400);
	if (rc < 0)
	{
		SKYCERR("%s err(%d)\n", __func__, rc);
		return rc;
	}
	mdelay(10);

	SKYCDBG("%s: X\n",__func__);
	return rc;
}
void ce1612_sensor_start_stream(struct msm_sensor_ctrl_t *s_ctrl)
{

#if 1 
SKYCDBG("%s: %d\n", __func__, __LINE__);
#endif

}
#if 0//ori
static int32_t ce1612_video_config(void)
{
	int32_t rc = 0;
	uint8_t data_buf[4];
	uint8_t rdata = 0;

#if 1 // test cts
	if(f_stop_capture == 1)
	{
		f_stop_capture = 0;
		
		rc = ce1612_poll(ce1612_saddr, 0x6C, 0x08, 10, 1000);
		if (rc < 0)
		{
			SKYCERR("%s : Preview Start polling ERROR !!!\n",__func__);
			return rc;
		}	
		goto preview_ok;
	}
#endif	


	rc = ce1612_cmd_read(0x6C, &rdata, 1);

	if((rdata > 0) && (rdata < 8))
	{
#if 1
		// stop Capture	
		rc = ce1612_cmd(ce1612_saddr, 0x75, 0, 0);
		if (rc < 0)
		{
			SKYCERR("ERR:%s Capture Stop command FAIL!!!rc=%d return~~\n", __func__, rc);
			return rc;
		}	
#endif
		rc = ce1612_poll(ce1612_saddr, 0x6C, 0x00, 10, 1000);
		if (rc < 0)
		{
			SKYCERR("%s : Capture Stop polling ERROR !!!\n",__func__);
			return rc;
		}		
	}
	else if(rdata == 8)
	{
		goto preview_ok;
	}
	else if(rdata == 9)
	{
		rc = ce1612_poll(ce1612_saddr, 0x6C, 0x08, 10, 1000);
		if (rc < 0)
		{
			SKYCERR("%s : Preview Start polling ERROR !!!\n",__func__);
			return rc;
		}	
		goto preview_ok;
	}
	
#if 0
	// Preview setting (Sensor)
	data_buf[0] = 0x1C;	// SXGA
	data_buf[1] = 0x01;
	data_buf[2] = 0x00;	
	rc = ce1612_cmd(ce1612_saddr, 0x54, data_buf, 3);
	if (rc < 0)
	{
		SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
		return rc;
	}

	// Buffering setting
#if 0	//jpeg
	data_buf[0] = 0x00;
	data_buf[1] = 0x00;
	data_buf[2] = 0x01;	
	data_buf[3] = 0x01;	
	rc = ce1612_cmd(ce1612_saddr, 0x73, data_buf, 4);
	if (rc < 0)
	{
		SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
		return rc;
	}
#else	// yuv
	data_buf[0] = 0x21;
	data_buf[1] = 0x05;
	data_buf[2] = 0x00;	
	data_buf[3] = 0x00;	
	rc = ce1612_cmd(ce1612_saddr, 0x73, data_buf, 4);
	if (rc < 0)
	{
		SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
		return rc;
	}
#endif
#endif
	SKYCDBG("%s : Preview Start CMD !!!\n",__func__);

	// AE/AWB enable
	data_buf[0] = 0x00;
	rc = ce1612_cmd(ce1612_saddr, 0x11, data_buf, 1);

	// Preview start (PREVIEW)
	data_buf[0] = 0x01;
	rc = ce1612_cmd(ce1612_saddr, 0x6B, data_buf, 1);

	rc = ce1612_poll(ce1612_saddr, 0x6C, 0x08, 10, 1000);
	if (rc < 0)
	{
		SKYCERR("%s : Preview Start polling ERROR !!!\n",__func__);
		return rc;
	}	

preview_ok:

	if(caf_camera_flag)
		ce1612_set_caf_camera(caf_camera_flag);

	f_stop_capture = 0;	// test cts

	SKYCDBG("%s end rc = %d\n",__func__, rc);
	return rc;
}


static int32_t ce1612_snapshot_config(void)
{
	int32_t rc = 0;
//	int i=0;
//	uint16_t read_data =0;
	uint8_t data_buf[10];

	/* set snapshot resolution to 3264x2448 */
	SKYCDBG("%s start\n",__func__);

	// Buffering setting
#if 0	//jpeg
	data_buf[0] = 0x00;
	data_buf[1] = 0x01;
	data_buf[2] = 0x00;		
	rc = ce1612_cmd(ce1612_saddr, 0x95, data_buf, 3);
	if (rc < 0)
	{
		SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
		return rc;
	}
#endif
	if(caf_camera_flag)
		ce1612_set_caf_camera(0);

	// Data output setting
	data_buf[0] = 0x00;
	rc = ce1612_cmd(ce1612_saddr, 0x65, data_buf, 1);
	if (rc < 0)
	{
		SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
		return rc;
	}	

	// Buffering Capture start
	rc = ce1612_cmd(ce1612_saddr, 0x74, 0, 0);

	SKYCDBG("%s end rc = %d\n",__func__, rc);
	
	return rc;
}

#else//MARUKO
static int32_t ce1612_ZSL_config(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	uint8_t data_buf[4];
	uint8_t rdata = 0;

	rc = ce1612_cmd_read(s_ctrl, 0x6C, &rdata, 1);
	SKYCDBG("%s: rdata=%d, sensor_mode=%d E\n",__func__, rdata, sensor_mode);

	if((rdata > 0) && (rdata < 8))
	{
		// stop Capture 
		rc = ce1612_cmd(s_ctrl, 0x75, 0, 0);
		if (rc < 0)
		{
			SKYCERR("ERR:%s Capture Stop command FAIL!!!rc=%d return~~\n", __func__, rc);
			return rc;
		}	

		rc = ce1612_poll(s_ctrl, 0x6C, 0x00, 10, 100);
		if (rc < 0)
		{
			SKYCERR("%s : Capture Stop polling ERROR !!!\n",__func__);
			return rc;
		}		
	}
	else if(rdata == 8)
	{
		goto preview_ok;
	}
	else if(rdata == 9)
	{
		rc = ce1612_poll(s_ctrl, 0x6C, 0x08, 10, 100);
		if (rc < 0)
		{
			SKYCERR("%s : Preview Start polling ERROR !!!\n",__func__);
			return rc;
		}	
		goto preview_ok;
	}

	SKYCDBG("%s : ZSL Preview Start, zsl_status=%d !!!\n",__func__, zsl_status);


//----------------------------------------------------//

	// AE/AWB enable	
	data_buf[0] = 0x01;
	data_buf[1] = 0x10;
	rc = ce1612_cmd(s_ctrl, 0x04, data_buf, 2);
	if (rc < 0)
	{
		SKYCERR("%s err(%d)\n", __func__, rc);
		return rc;
	}	

	rc = ce1612_cmd(s_ctrl, 0x01, NULL, 0);
	if (rc < 0)
	{
		SKYCERR("%s err(%d)\n", __func__, rc);
		return rc;
	}	

//----------------------------------------------------//

	data_buf[0] = 0x00;
	rc = ce1612_cmd(s_ctrl, 0x5B, data_buf, 1);
	if (rc < 0)
	{
		SKYCERR("%s err(%d)\n", __func__, rc);
		return rc;
	}	

//----------------------------------------------------//

	data_buf[0] = 0x00;
	rc = ce1612_cmd(s_ctrl, 0x11, data_buf, 1);
	if (rc < 0)
	{
		SKYCERR("%s err(%d)\n", __func__, rc);
		return rc;
	}	

//----------------------------------------------------/

	data_buf[0] = 0x20; 
	data_buf[1] = 0x00;
	data_buf[2] = 0x00;
	rc = ce1612_cmd(s_ctrl, 0x54, data_buf, 3);
	if (rc < 0)
	{
		SKYCERR("%s err(%d)\n", __func__, rc);
		return rc;
	}

//----------------------------------------------------/

	data_buf[0] = 0x20; 
	data_buf[1] = 0x00;
	data_buf[2] = 0x00;
	data_buf[3] = 0x00;
	rc = ce1612_cmd(s_ctrl, 0x73, data_buf, 4);
	if (rc < 0)
	{
		SKYCERR("%s err(%d)\n", __func__, rc);
		return rc;
	}	

//----------------------------------------------------/

	// ZSL Preview start (PREVIEW)
	data_buf[0] = 0x01; 
	rc = ce1612_cmd(s_ctrl, 0x6B, data_buf, 1);

	rc = ce1612_poll(s_ctrl, 0x6C, 0x08, 10, 100);  
	if (rc < 0)
	{
		SKYCERR("%s : ZSL Preview Start polling ERROR !!!\n",__func__);
		return rc;
	} 

preview_ok:
	mdelay(30); // test 04.13.

	zsl_status = sensor_mode = 3;
//needtocheck	ce1612_set_coninuous_af(s_ctrl, continuous_af_mode);
//	if(caf_camera_flag)
//		ce1612_set_caf_camera(s_ctrl, caf_camera_flag);

	//	f_stop_capture = 0;	// test cts
	x1 = 0;
	x2 = 0;
	y1 = 0;
	y2 = 0;

	SKYCDBG("%s end rc = %d\n",__func__, rc);
	return rc;
}


static int32_t ce1612_video_config(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	uint8_t data_buf[4];
	uint8_t rdata = 0;

	rc = ce1612_cmd_read(s_ctrl, 0x6C, &rdata, 1);
	SKYCDBG("%s: rdata=%d, sensor_mode=%d E\n",__func__, rdata, sensor_mode);


	if((rdata > 0) && (rdata < 8))
	{
		// stop Capture	
		rc = ce1612_cmd(s_ctrl, 0x75, 0, 0);
		if (rc < 0)
		{
			SKYCERR("%s err(%d)\n", __func__, rc);
			return rc;
		}	

		rc = ce1612_poll(s_ctrl, 0x6C, 0x00, 50, 100);
		if (rc < 0)
		{
			SKYCERR("%s err(%d)\n", __func__, rc);
			return rc;
		}		
	}
	else if(rdata == 8)
	{
		goto preview_ok;
	}
	else if(rdata == 9)
	{
		rc = ce1612_poll(s_ctrl, 0x6C, 0x08, 50, 100);
		if (rc < 0)
		{
			SKYCERR("%s err(%d)\n", __func__, rc);
			return rc;
		}	
		goto preview_ok;
	}

	SKYCDBG("%s : Preview Start, zsl_status=%d !!!\n",__func__, zsl_status);

//----------------------------------------------------//

	if(zsl_status == 3){
		
		data_buf[0] = 0x01;
		data_buf[1] = 0x00;
		rc = ce1612_cmd(s_ctrl, 0x04, data_buf, 2);
		if (rc < 0)
		{
			SKYCERR("%s err(%d)\n", __func__, rc);
			return rc;
		}	

		rc = ce1612_cmd(s_ctrl, 0x01, NULL, 0);
		if (rc < 0)
		{
			SKYCERR("%s err(%d)\n", __func__, rc);
			return rc;
		}	
		
		// Capture Size (YUV)
		data_buf[0] = 0x21;
		data_buf[1] = 0x05;
		data_buf[2] = 0x00; 
		data_buf[3] = 0x00; 
		
		rc = ce1612_cmd(s_ctrl, 0x73, data_buf, 4);
		if (rc < 0)
		{
			SKYCERR("%s err(%d)\n", __func__, rc);
			return rc;
		}

		zsl_status = 1;		
	}   
//----------------------------------------------------/

	// Preview Size
	data_buf[0] = 0x1C;	//SXGA(1280x960)
	data_buf[1] = 0x01;
#ifdef CONFIG_MACH_MSM8X60_EF65L
	data_buf[2] = 0x02; 
#else  
	data_buf[2] = 0x00;	
#endif
	rc = ce1612_cmd(s_ctrl, 0x54, data_buf, 3);

	// AE/AWB enable
	data_buf[0] = 0x00;
	rc = ce1612_cmd(s_ctrl, 0x11, data_buf, 1);

	// Preview start (PREVIEW)
	data_buf[0] = 0x01;
	rc = ce1612_cmd(s_ctrl, 0x6B, data_buf, 1);

	rc = ce1612_poll(s_ctrl, 0x6C, 0x08, 50, 100);
	if (rc < 0)
	{
		SKYCERR("%s err(%d)\n", __func__, rc);
		return rc;
	}

preview_ok:
	sensor_mode = 1;
//needtocheck	ce1612_set_coninuous_af(s_ctrl, continuous_af_mode);
//	if(caf_camera_flag)
//		ce1612_set_caf_camera(s_ctrl, caf_camera_flag);

	//	f_stop_capture = 0;	// test cts
	x1 = 0;
	x2 = 0;
	y1 = 0;
	y2 = 0;

	SKYCDBG("%s: X\n",__func__);
	return rc;
}

static int32_t ce1612_1080p_config(struct msm_sensor_ctrl_t *s_ctrl)
{
        int32_t rc = 0;
        uint8_t read_data =0;
        uint8_t data_buf[4];

        SKYCDBG("%s enter\n",__func__);

        rc = ce1612_cmd_read(s_ctrl, 0x24, &read_data, 1);
	if (rc < 0)
	{
		SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
		return rc;
	}

#if 1 // AF-T state check
            rc = ce1612_cmd_read(s_ctrl, 0x2D, data_buf, 2);
            if (rc < 0)
            {
                SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
                return rc;
            }

            if(data_buf[0] == 0x01)
                return 0;
#endif

        if(read_data & 0x01)
        {
            rc = ce1612_lens_stop(s_ctrl);//ce1612_cmd_poll(s_ctrl, 0x35, 0, 0); // Lens Stop
            if (rc < 0)
            {
                SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
                return rc;
            }   
        }

	// Preview stop
        data_buf[0] = 0x00; 
        rc = ce1612_cmd(s_ctrl, 0x6B, data_buf, 1);
        rc = ce1612_poll(s_ctrl, 0x6C, 0x00, 10, 100);
        
	if (rc < 0)
	{
		SKYCERR("%s : Preview Stop polling ERROR !!!\n",__func__);
		return rc;
	}	
    
	data_buf[0] = 0x1E;	// Full HD
	data_buf[1] = 0x03;
#ifdef CONFIG_MACH_MSM8X60_EF65L 
	data_buf[2] = 0x02; 
#else  
	data_buf[2] = 0x00;		
#endif
	rc = ce1612_cmd(s_ctrl, 0x54, data_buf, 3);
	if (rc < 0)
	{
		SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
		return rc;
	}

    	// AE/AWB enable
    	#if 0 // reference video_config
	data_buf[0] = 0x00;
	rc = ce1612_cmd(s_ctrl, 0x11, data_buf, 1);
	#endif
    
	// Preview start (PREVIEW)
                data_buf[0] = 0x01; 
                rc = ce1612_cmd(s_ctrl, 0x6B, data_buf, 1);
                rc = ce1612_poll(s_ctrl, 0x6C, 0x08, 10, 100);  
	if (rc < 0)
	{
		SKYCERR("%s : Preview Start polling ERROR !!!\n",__func__);
		return rc;
	}	
    
        if(read_data & 0x01)
        {
            	rc = ce1612_cmd(s_ctrl, 0x23, 0, 0);	// AF Start
            	if (rc < 0)
            	{
            		SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
            		return rc;
            	}	
        }
        	SKYCDBG("%s end rc = %d\n",__func__, rc);

	return rc;
}

static int32_t ce1612_snapshot_config(struct msm_sensor_ctrl_t *s_ctrl)
{
    int32_t rc = 0;
    uint8_t data_buf[10];

//needtocheck    int8_t prev_caf = continuous_af_mode;
//needtocheck    rc = ce1612_set_coninuous_af(s_ctrl, 0);
//needtocheck    continuous_af_mode = prev_caf;


//needtocheck	if(caf_camera_flag)
//needtocheck		ce1612_set_caf_camera(0);

    rc = ce1612_lens_stop(s_ctrl);

	 SKYCDBG("%s: E\n",__func__);

    // additional setting
    data_buf[0] = 0x00;
    rc = ce1612_cmd(s_ctrl, 0x65, data_buf, 1);
    if (rc < 0)
    {
		 SKYCERR("%s err(%d)\n", __func__, rc);
		 return rc;
    }

    rc = ce1612_cmd(s_ctrl, 0x74, 0, 0);
    if (rc < 0)
    {
		 SKYCERR("%s err(%d)\n", __func__, rc);
		 return rc;
    }

    // AE/AWB enable
    data_buf[0] = 0x11;
    rc = ce1612_cmd(s_ctrl, 0x11, data_buf, 1);

    // Buffering Capture start
    rc = ce1612_cmd(s_ctrl, 0x74, 0, 0);
	 
	 SKYCDBG("%s: X\n",__func__);
    return rc;
}
#endif

void ce1612_sensor_stop_stream(struct msm_sensor_ctrl_t *s_ctrl)
{
    int rc = 0;
    uint8_t data_buf[2];

    SKYCDBG("%s: sensor_mode = %d\n", __func__, sensor_mode);

	// 	if(!(sensor_mode > 0 && sensor_mode < 4))
	if((sensor_mode < 0) || (sensor_mode > 3))
		return;

	if(sensor_mode != 0)	 
		rc = ce1612_lens_stop(s_ctrl);

	if(sensor_mode == 0)
	{
		// stop Capture 
		rc = ce1612_cmd(s_ctrl, 0x75, 0, 0);
		if (rc < 0)
		{
			SKYCERR("ERR:%s Capture Stop command FAIL!!!rc=%d return~~\n", __func__, rc);
			return;
		}	

		rc = ce1612_poll(s_ctrl, 0x6C, 0x00, 10, 100);
		if (rc < 0)
		{
			SKYCERR("%s : Capture Stop polling ERROR !!!\n",__func__);
			return;
		}			  
	}
	else
	{
		data_buf[0] = 0x00;	
		rc = ce1612_cmd(s_ctrl, 0x6B, data_buf, 1);
		rc = ce1612_poll(s_ctrl, 0x6C, 0x00, 10, 100);
		if (rc < 0)
		{
			SKYCERR("%s : Preview Stop polling ERROR !!!\n",__func__);
			return;
		}	
	}

    sensor_mode = -1;
    SKYCDBG("%s: END -(%d) \n", __func__, rc);	
}


int ce1612_sensor_init(struct msm_sensor_ctrl_t *s_ctrl)
{
	int rc = 0;
	uint8_t data_buf[4];

	SKYCDBG("%s: E\n",__func__);

	// ISP FW Boot UP !!
	rc = ce1612_cmd(&ce1612_s_ctrl, 0xF0, NULL, 0);
	mdelay(70);

	// Preview Stop
	//ce1612_sensor_stop_stream(s_ctrl); //charley2

	// Preview Size
	data_buf[0] = 0x1C;	//SXGA(1280x960)
	data_buf[1] = 0x01;
#ifdef CONFIG_MACH_MSM8X60_EF65L
	data_buf[2] = 0x02; 
#else  
	data_buf[2] = 0x00; 
#endif
	rc = ce1612_cmd(s_ctrl, 0x54, data_buf, 3);

	// Capture Size (YUV)
	data_buf[0] = 0x21;
	data_buf[1] = 0x05;
	data_buf[2] = 0x00; 
#ifdef CONFIG_MACH_MSM8X60_EF65L
	data_buf[3] = 0x20; 
#else  
	data_buf[3] = 0x00; 
#endif
	rc = ce1612_cmd(s_ctrl, 0x73, data_buf, 4);
	if (rc < 0)
	{
		SKYCERR("%s err(%d)\n", __func__, rc);
		return rc;
	}

	sensor_mode = 1;

	SKYCDBG("%s: X\n",__func__);	
	return rc;	
}

int32_t ce1612_sensor_setting(struct msm_sensor_ctrl_t *s_ctrl, int update_type, int res)
{
	int32_t rc = 0;

	SKYCDBG("%s: update_type = %d, res=%d E\n", __func__, update_type, res);

#if 1
	/*   if (s_ctrl->func_tbl->sensor_stop_stream)
	s_ctrl->func_tbl->sensor_stop_stream(s_ctrl);
	*/
	if(res != 0)
		ce1612_sensor_stop_stream(s_ctrl);
#endif
	msleep(30);
	if (update_type == MSM_SENSOR_REG_INIT) {
		s_ctrl->curr_csi_params = NULL;
		msm_sensor_enable_debugfs(s_ctrl);
		ce1612_sensor_init(s_ctrl);
		
	} else if (update_type == MSM_SENSOR_UPDATE_PERIODIC) {
	
		// msm_sensor_write_res_settings(s_ctrl, res);
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

//s_ctrl->func_tbl->sensor_start_stream(s_ctrl);	
#if 0//needtocheck		
		switch (res) {
			case 0:
				rc = ce1612_snapshot_config(s_ctrl);	
				break;

			case 1:
				rc = ce1612_video_config(s_ctrl);
				SKYCDBG("Sensor setting : Case 1 Video config"); 
				break;

	      //case 2: 
			//	rc = ce1612_1080p_config(s_ctrl);	
			//	break;

			case 3: 
				rc = ce1612_ZSL_config(s_ctrl);	
				SKYCDBG("Sensor setting : Case 3 ZSL config");				
				break;	
				 
			default:
				rc = ce1612_video_config(s_ctrl);
				SKYCDBG("Sensor setting : Default Video config"); 
				break;
		}
#endif
		sensor_mode = res;
		SKYCDBG("Sensor setting : Res = %d\n", res);

		//msleep(30);
	}
	SKYCDBG("%s: %d x\n", __func__, __LINE__);
	return rc;
}

int32_t ce1612_sensor_setting1(struct msm_sensor_ctrl_t *s_ctrl,
			int update_type, int res)
{
	int32_t rc = 0;
	static int csi_config;
	SKYCDBG("%s: update_type = %d, res=%d E\n", __func__, update_type, res);
	
#if 1
	/*   if (s_ctrl->func_tbl->sensor_stop_stream)
	s_ctrl->func_tbl->sensor_stop_stream(s_ctrl);
	*/
	if(res != 0)
		s_ctrl->func_tbl->sensor_stop_stream(s_ctrl);
#endif
	msleep(30);
	if (update_type == MSM_SENSOR_REG_INIT) {
		CDBG("Register INIT\n");
		s_ctrl->curr_csi_params = NULL;
		msm_sensor_enable_debugfs(s_ctrl);
		ce1612_sensor_init(s_ctrl);//msm_sensor_write_init_settings(s_ctrl);
		csi_config = 0;
	} else if (update_type == MSM_SENSOR_UPDATE_PERIODIC) {
		CDBG("PERIODIC : %d\n", res);
		//msm_sensor_write_conf_array(
		//	s_ctrl->sensor_i2c_client,
		//	s_ctrl->msm_sensor_reg->mode_settings, res);
		msleep(30);
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

		//s_ctrl->func_tbl->sensor_start_stream(s_ctrl);
//s_ctrl->func_tbl->sensor_start_stream(s_ctrl);	
#if 1//needtocheck		
		switch (res) {
			case 0:
				rc = ce1612_snapshot_config(s_ctrl);
                SKYCDBG("Sensor setting : Case 0 snapshot config");
				break;

			case 1:
				rc = ce1612_video_config(s_ctrl);
#if 1//for_test
                if(current_fps != 31)
                    ce1612_set_preview_fps(s_ctrl ,current_fps);
#else
                if ((previewFPS > C_PANTECH_CAMERA_MIN_PREVIEW_FPS) && (previewFPS < C_PANTECH_CAMERA_MAX_PREVIEW_FPS)) 
                {  
                    SKYCDBG("%s: preview_fps=%d\n", __func__, previewFPS);
                    ce1612_set_preview_fps(s_ctrl ,previewFPS);
                }
#endif
				SKYCDBG("Sensor setting : Case 1 Video config"); 
				break;

	      	case 2: 
                                    rc = ce1612_1080p_config(s_ctrl); 
                                    if(rc < 0)
				{
					SKYCDBG("%s ce1612_1080p_config err(%d)\n", __func__, rc);
					return rc;
				}
    			SKYCDBG("Sensor setting : Case 2 1080p config");	
				break;

			case 3: 
				rc = ce1612_ZSL_config(s_ctrl);	
                
				//rc = ce1612_video_config(s_ctrl);	
				SKYCDBG("Sensor setting : Case 3 ZSL config");				
				break;	
				 
			default:
				rc = ce1612_video_config(s_ctrl);
				SKYCDBG("Sensor setting : Default Video config"); 
				break;
		}
#endif
		sensor_mode = res;
		SKYCDBG("Sensor setting : Res = %d\n", res);
				
		msleep(50);
	}
	SKYCDBG("%s: %d x\n", __func__, __LINE__);
	return rc;
}

#ifdef CONFIG_PANTECH_CAMERA
static int ce1612_set_effect(struct msm_sensor_ctrl_t *s_ctrl ,int8_t effect)
{
    uint8_t data_buf[2];
    int rc = 0;

    SKYCDBG("%s start / effect:%d \n",__func__, effect);

    if(effect < 0 || effect >= CAMERA_EFFECT_MAX){
        SKYCERR("%s error. effect=%d\n", __func__, effect);
        return -EINVAL;
    }

    if(effect == 5) //posterize
    {
        data_buf[0] = 0x05;
        data_buf[1] = ce1612_effect_data[effect];//*(ce1612_tune_regs.effect_data);
        rc = ce1612_cmd(s_ctrl, 0x3D, data_buf, 2);   //effect off
    
        data_buf[0] = 0x07;
        data_buf[1] = 0x0E;
        rc = ce1612_cmd(s_ctrl, 0x3D, data_buf, 2);
        if (rc < 0)
        {
            SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
            return rc;
        }   
        rc = ce1612_cmd(s_ctrl, 0x01, NULL, 0);
        if (rc < 0)
        {
            SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
            return rc;
        }   

    }
    else
    {
        data_buf[0] = 0x05;
        data_buf[1] = ce1612_effect_data[effect];//*(ce1612_tune_regs.effect_data+effect);
        rc = ce1612_cmd(s_ctrl, 0x3D, data_buf, 2);
        
        data_buf[0] = 0x07;
        data_buf[1] = 0x06;
        rc = ce1612_cmd(s_ctrl, 0x3D, data_buf, 2);   // gamma default        
    if (rc < 0)
    {
        SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
        return rc;
    }   
        rc = ce1612_cmd(s_ctrl, 0x01, NULL, 0);
    if (rc < 0)
    {
        SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
        return rc;
    }   
    }

    SKYCDBG("%s end\n",__func__);
    
    return rc;
}

static int ce1612_set_whitebalance(struct msm_sensor_ctrl_t *s_ctrl ,int8_t whitebalance)
{
    uint8_t data_buf[2];
    int rc = 0;

    SKYCDBG("%s start / whitebalance:%d \n",__func__, whitebalance);

    if(whitebalance < 1 || whitebalance > CE1612_WHITEBALANCE_MAX){
        SKYCERR("%s error. whitebalance=%d\n", __func__, whitebalance);
        return -EINVAL;
    }


    if(whitebalance == 1 || whitebalance == 2)      // auto
    {
        data_buf[0] = 0x00;
        rc = ce1612_cmd(s_ctrl, 0x1A, data_buf, 1);
        if (rc < 0)
        {
            SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
            return rc;
        }   
        
        data_buf[0] = 0x11;
        data_buf[1] =  0x00;
    }   
    else
    {
        data_buf[0] = 0x01;
        rc = ce1612_cmd(s_ctrl, 0x1A, data_buf, 1);
        if (rc < 0)
        {
            SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
            return rc;
        }   

        data_buf[0] = 0x10;
        data_buf[1] =  ce1612_wb_data[whitebalance-1];//*(ce1612_tune_regs.wb_data+whitebalance-1);
    }
    
    rc = ce1612_cmd(s_ctrl, 0x04, data_buf, 2);   
    
    rc = ce1612_cmd(s_ctrl, 0x01, NULL, 0);
    if (rc < 0)
    {
        SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
        return rc;
    }   

    SKYCDBG("%s end\n",__func__);

    return rc;
}

static int ce1612_set_brightness(struct msm_sensor_ctrl_t *s_ctrl ,int8_t brightness)
{
    uint8_t data_buf[2];
    int rc = 0;

    SKYCDBG("%s start / brightness:%d \n",__func__, brightness);

    if(brightness < 0 || brightness >= CE1612_BRIGHTNESS_MAX){
        SKYCERR("%s error. brightness=%d\n", __func__, brightness);
        return -EINVAL;
    }

    data_buf[0] = 0x02;
    data_buf[1] = ce1612_bright_data[brightness];//*(ce1612_tune_regs.bright_data+brightness);
    rc = ce1612_cmd(s_ctrl, 0x04, data_buf, 2);
    if (rc < 0)
    {
        SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
        return rc;
    }   
    rc = ce1612_cmd(s_ctrl, 0x01, NULL, 0);
    if (rc < 0)
    {
        SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
        return rc;
    }   

    SKYCDBG("%s end\n",__func__);
    
    return rc;
}

static int32_t ce1612_set_led_gpio_set(int8_t led_mode)
{
	int rc;
	int enable_flash_main_gpio = 0;	
	SKYCDBG("%s led_mode:%d \n",__func__, led_mode);
    
	if(led_mode != CE1612_CFG_LED_MODE_OFF)
		enable_flash_main_gpio = ON;
	else
		enable_flash_main_gpio = OFF;

	//control ce1612 led flash main gpio
#if 1
    if (sgpio_ctrl(sgpios, FLASH_EN, enable_flash_main_gpio) < 0)       rc = -EIO;
#else
	rc = gpio_tlmm_config(GPIO_CFG(CE1612_LED_FLASH_ENABLE_GPIO, 0,
		GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),GPIO_CFG_ENABLE);

	if (!rc)
	{
		SKYCDBG("%s enable_flash_main_gpio = %d\n", __func__, enable_flash_main_gpio);
		gpio_set_value(CE1612_LED_FLASH_ENABLE_GPIO,enable_flash_main_gpio);					
	}
	else
	{
		SKYCERR("ERR:%s ENABLE FLASH GPIO FAIL!!!rc=%d return~~\n", __func__, rc);
		return rc;
	}
#endif

	return rc;
}

static int ce1612_set_led_mode(struct msm_sensor_ctrl_t *s_ctrl ,int8_t led_mode)
{
    /* off, auto, on, movie */  

    int rc;
    uint8_t data_buf[4];
    uint8_t read_data =0;
    
    SKYCDBG("%s: led_mode=%d\n", __func__, led_mode);
    if ((led_mode < CE1612_CFG_LED_MODE_OFF) || (led_mode >= CE1612_CFG_LED_MODE_MAX)) {
        SKYCERR("%s: -EINVAL, led_mode=%d\n", __func__, led_mode);
        return -EINVAL;
    }

    rc = ce1612_cmd_read(s_ctrl, 0x24, &read_data, 1);
    if (rc < 0)
    {
        SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
        return rc;
    }

#if 1 // AF-T state check
    rc = ce1612_cmd_read(s_ctrl, 0x2D, data_buf, 2);
    if (rc < 0)
    {
        SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
        return rc;
    }

    if(data_buf[0] == 0x01)
        read_data = 0;
#endif

    if(read_data & 0x01)
    {
        rc = ce1612_lens_stop(s_ctrl);//ce1612_cmd_poll(s_ctrl, 0x35, 0, 0); // Lens Stop
        if (rc < 0)
        {
            SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
            return rc;
        }   
    }

//  if((led_mode != CE1612_CFG_LED_MODE_OFF)&&(led_mode != CE1612_CFG_LED_MODE_AUTO))
    rc = ce1612_set_led_gpio_set(led_mode);


    //control ce1612 isp gpio
    switch(led_mode)
    {
        case CE1612_CFG_LED_MODE_OFF:
            SKYCDBG("CE1612_CFG_LED_MODE_OFF SET\n");
            data_buf[0] = 0x00;
            data_buf[1] = 0x00;
            rc = ce1612_cmd(s_ctrl, 0x06, data_buf, 2);
            
            data_buf[0] = 0x01;
            data_buf[1] = 0x00;
            rc = ce1612_cmd(s_ctrl, 0xB2, data_buf, 2);

            data_buf[0] = 0x03;
            data_buf[1] = 0x00;
            rc = ce1612_cmd(s_ctrl, 0xB2, data_buf, 2);
            break;
            
        case CE1612_CFG_LED_MODE_AUTO:
            SKYCDBG("CE1612_CFG_LED_MODE_AUTO SET\n");
            
            data_buf[0] = 0x00;
            data_buf[1] = 0x00;
            rc = ce1612_cmd(s_ctrl, 0x06, data_buf, 2);           
            
/*          data_buf[0] = 0x01;
            data_buf[1] = 0x00;
            data_buf[2] = 0x00;
            data_buf[3] = 0x00;
            rc = ce1612_cmd(s_ctrl, 0xB3, data_buf, 4);

            data_buf[0] = 0x01;
            data_buf[1] = 0x00; //0x02;  //AF flash off
            rc = ce1612_cmd(s_ctrl, 0xB2, data_buf, 2);
*/
#if 0	// ������������ ����A�� ��ECO. (auto)
            data_buf[0] = 0x03;
            data_buf[1] = 0x00;
            data_buf[2] = 0x15;
            data_buf[3] = 0x00;
            rc = ce1612_cmd(ce1612_saddr, 0xB3, data_buf, 4);
#endif
            data_buf[0] = 0x03;
            data_buf[1] = 0x02;
            rc = ce1612_cmd(s_ctrl, 0xB2, data_buf, 2);
            break;  
            
        case CE1612_CFG_LED_MODE_ON:
            SKYCDBG("CE1612_CFG_LED_MODE_ON SET\n");

            data_buf[0] = 0x00;
            data_buf[1] = 0x00;
            rc = ce1612_cmd(s_ctrl, 0x06, data_buf, 2);
            
/*          data_buf[0] = 0x01;
            data_buf[1] = 0x00;
            data_buf[2] = 0x00;
            data_buf[3] = 0x00;
            rc = ce1612_cmd(s_ctrl, 0xB3, data_buf, 4);

            data_buf[0] = 0x01;
            data_buf[1] = 0x00; //0x01;  //AF flash off
            rc = ce1612_cmd(s_ctrl, 0xB2, data_buf, 2);
*/
#if 0	// ������������ ����A�� ��ECO. (auto)
            data_buf[0] = 0x03;
            data_buf[1] = 0x00;
            data_buf[2] = 0x15;
            data_buf[3] = 0x00;
            rc = ce1612_cmd(ce1612_saddr, 0xB3, data_buf, 4);
#endif
            data_buf[0] = 0x03;
            data_buf[1] = 0x01;
            rc = ce1612_cmd(s_ctrl, 0xB2, data_buf, 2);
            break;
            
        case CE1612_CFG_LED_MODE_MOVIE:
            SKYCDBG("CE1612_CFG_LED_MODE_MOVIE SET\n");
            data_buf[0] = 0x01;
            data_buf[1] = 0x00;
            rc = ce1612_cmd(s_ctrl, 0xB2, data_buf, 2);

            data_buf[0] = 0x03;
            data_buf[1] = 0x00;
            rc = ce1612_cmd(s_ctrl, 0xB2, data_buf, 2);
            
            data_buf[0] = 0x00;
            data_buf[1] = 0x00;
            data_buf[2] = 0x00;
            data_buf[3] = 0x00;
            rc = ce1612_cmd(s_ctrl, 0xB3, data_buf, 4);


            data_buf[0] = 0x00;
            data_buf[1] = 0x01;
            rc = ce1612_cmd(s_ctrl, 0x06, data_buf, 2);
            break;

        case CE1612_CFG_LED_MODE_FLASH:
            SKYCDBG("CE1612_CFG_LED_MODE_MOVIE SET\n");
            data_buf[0] = 0x01;
            data_buf[1] = 0x00;
            rc = ce1612_cmd(s_ctrl, 0xB2, data_buf, 2);

            data_buf[0] = 0x03;
            data_buf[1] = 0x00;
            rc = ce1612_cmd(s_ctrl, 0xB2, data_buf, 2);
            
            data_buf[0] = 0x00;
            data_buf[1] = 0x00;
            data_buf[2] = 0x15;
            data_buf[3] = 0x00;
            rc = ce1612_cmd(s_ctrl, 0xB3, data_buf, 4);


            data_buf[0] = 0x00;
            data_buf[1] = 0x01;
            rc = ce1612_cmd(s_ctrl, 0x06, data_buf, 2);
            break;
        default:        
            break;          
    }

    if(read_data & 0x01)
    {
        rc = ce1612_cmd(s_ctrl, 0x23, 0, 0);  // AF Start
        if (rc < 0)
        {
            SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
            return rc;
        }   
    }

/*
    if (rc < 0)
    {
        SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
    }           
*/
    return rc;
}


static int ce1612_set_auto_focus(struct msm_sensor_ctrl_t *s_ctrl ,int8_t autofocus)
{
    int32_t rc = 0;
    uint8_t read_data =0;
    uint8_t data_buf[4];

    SKYCDBG("%s  auto_focus = %d\n",__func__, autofocus);
    if ((autofocus < 0) || (autofocus > 6))
    {
        SKYCERR("%s FAIL!!! return~~  autofocus = %d\n",__func__,autofocus);
        return 0;//-EINVAL;
    }

    if(autofocus == 6)  //cancel AF
    {
        rc = ce1612_lens_stop(s_ctrl);
        return rc;
    }

#if 1 // AF-T state check
    rc = ce1612_cmd_read(s_ctrl, 0x2D, data_buf, 2);
    if (rc < 0)
    {
        SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
        return rc;
    }

    if(data_buf[0] == 0x01)
        return 0;
#endif

    SKYCDBG("%s START~ autofocus mode = %d\n",__func__, autofocus);

    rc = ce1612_cmd_read(s_ctrl, 0x24, &read_data, 1);
    if (rc < 0)
    {
        SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
        return rc;
    }

    if(read_data & 0x01)
    {
        rc = ce1612_lens_stop(s_ctrl);//ce1612_cmd_poll(ce1612_saddr, 0x35, 0, 0); // Lens Stop
        if (rc < 0)
        {
            SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
            return rc;
        }   
    }
    rc = ce1612_cmd_read(s_ctrl, 0x43, &read_data, 1);
    SKYCDBG("%s read_data(%d)\n",__func__, read_data);

    if(read_data == 0x05){
        goto start_af;
    }
/*
  AF_MODE_UNCHANGED = -1,
  AF_MODE_NORMAL    = 0,
  AF_MODE_MACRO,
  AF_MODE_INFINITY,
  AF_MODE_CONTINUOUS,
  AF_MODE_AUTO,
  AF_MODE_CAF,
  AF_MODE_MAX
*/
    switch(autofocus)
    {
        case 0: //AF_MODE_NORMAL
        case 4: //AF_MODE_AUTO
            data_buf[0] = 0x00;
            rc = ce1612_cmd(s_ctrl, 0x42, data_buf, 1);
            
            data_buf[0] = 0x00;
//            rc = ce1612_cmd_poll(s_ctrl, 0x20, data_buf, 1);
            rc = ce1612_cmd(s_ctrl, 0x20, data_buf, 1);
            rc = ce1612_poll_bit(s_ctrl, 0x24, 10, 400);
            if (rc < 0)
            {
                SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
                return rc;
            }

            break;
        case 1: // MACRO
            data_buf[0] = 0x00;
            rc = ce1612_cmd(s_ctrl, 0x42, data_buf, 1);
        
            data_buf[0] = 0x01;
            //rc = ce1612_cmd_poll(s_ctrl, 0x20, data_buf, 1);
            rc = ce1612_cmd(s_ctrl, 0x20, data_buf, 1);
            rc = ce1612_poll_bit(s_ctrl, 0x24, 10, 400);
            if (rc < 0)
            {
                SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
                return rc;
            }       
              

        default:    // NORMAL
            break;
                        
    }
    //rc = ce1612_cmd_poll(s_ctrl, 0x23, 0, 0);
start_af:
    rc = ce1612_cmd(s_ctrl, 0x23, 0, 0);
//    rc = ce1612_poll_bit(s_ctrl, 0x24, 10, 400);
//    SKYCDBG("%s  //ce1612_poll_bit(s_ctrl, 0x24, 10, 400); \n",__func__);
    if (rc < 0)
    {
        SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
        return rc;
    }
    SKYCDBG("%s END~ autofocus mode = %d\n",__func__, autofocus);

    return rc;
}

static int32_t ce1612_sensor_check_af(struct msm_sensor_ctrl_t *s_ctrl ,int8_t autofocus, int32_t* af_result)
{
    uint8_t rdata = 0;
    uint8_t data_buf[4];
    int32_t rc = 0;
    *af_result = 0; //must be set to 0 for polling

#if 0        
    SKYCDBG("%s: sensor_mode=%d E\n",__func__, sensor_mode);
#endif

    if(!(sensor_mode > 0 && sensor_mode < 4))
        return 0;

	//if(continuous_af_mode == 1) 
	//	return 0;

    rc = ce1612_cmd_read(s_ctrl, 0x24, &rdata, 1);
#if 0        
    SKYCDBG("%s: rdata(%d) X\n",__func__, rdata);
#endif
    if (rc < 0){
        SKYCERR("%s err(%d)\n", __func__, rc);	 	
        return rc;
    }
    
    if (!(rdata & 0x01))
    {
        rc = 0;     
#if 1
        data_buf[0] = 0x00;
        rc = ce1612_cmd(s_ctrl, 0x42, data_buf, 1);
       *af_result = 1; //focus end
#endif
    }
    else
        rc = -1;
#if 0    
    SKYCDBG("%s: rdata(%d) X\n",__func__, rdata);
#endif

    return rc;
}

static int ce1612_set_iso(struct msm_sensor_ctrl_t *s_ctrl ,int32_t iso)
{
    uint8_t data_buf[2];
    int rc = 0;

    SKYCDBG("%s start / iso:%d \n",__func__, iso);

    if(iso < 0 || iso >= CE1612_ISO_MAX){
        SKYCERR("%s error. iso=%d\n", __func__, iso);
        return 0; //-EINVAL;
    }

    data_buf[0] = 0x01;
    data_buf[1] = ce1612_iso_data[iso];//*(ce1612_tune_regs.iso_data+iso);
    rc = ce1612_cmd(s_ctrl, 0x04, data_buf, 2);
    if (rc < 0)
    {
        SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
        return rc;
    }
    rc = ce1612_cmd(s_ctrl, 0x01, NULL, 0);
    if (rc < 0)
    {
        SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
        return rc;
    }   

    SKYCDBG("%s end\n",__func__);   
    
    return rc;
}

static int scene_mode_to_table(int8_t scene_mode)
{
	return ce1612_scenemode[scene_mode];
}

static int ce1612_set_scene_mode(struct msm_sensor_ctrl_t *s_ctrl ,int8_t scene_mode)
{
    uint8_t data_buf[2];
    int rc = 0;

    SKYCDBG("%s start / scene_mode:%d \n",__func__, scene_mode);
    scene_mode = scene_mode_to_table(scene_mode);
    SKYCDBG("%s / scene_mode_to_table:%d \n",__func__, scene_mode);
    if(scene_mode < 0 || scene_mode >= CE1612_SCENE_MAX+1){
        SKYCERR("%s error. scene_mode=%d\n", __func__, scene_mode);
        return -EINVAL;
    }

    if(scene_mode == 0) //OFF
    {
        data_buf[0] = 0x00;
        data_buf[1] = 0x00;
        rc = ce1612_cmd(s_ctrl, 0x82, data_buf, 2);

        data_buf[0] = 0x00;
        rc = ce1612_cmd(s_ctrl, 0x1A, data_buf, 1);
    }
    else if(scene_mode == CE1612_SCENE_MAX) //AUTO
    {
        data_buf[0] = 0x01;
        data_buf[1] = 0x00;
        rc = ce1612_cmd(s_ctrl, 0x82, data_buf, 2);

        data_buf[0] = 0x00;
        rc = ce1612_cmd(s_ctrl, 0x1A, data_buf, 1);

    }
    else    //MANUAL
    {
        data_buf[0] = 0x02;
        data_buf[1] = ce1612_scene_data[scene_mode];//*(ce1612_tune_regs.scene_data+scene_mode);
        rc = ce1612_cmd(s_ctrl, 0x82, data_buf, 2);
        if (rc < 0)
        {
            SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
            return rc;
        }   
        switch(scene_mode)
        {
        case 8: //sunset
            data_buf[0] = 0x01;
            rc = ce1612_cmd(s_ctrl, 0x1A, data_buf, 1);
            data_buf[0] = 0x10;
            data_buf[1] = 0x03;
            rc = ce1612_cmd(s_ctrl, 0x04, data_buf, 2);
            break;
#if 0
        case 1: //portrait
            data_buf[0] = 0x01; //face detection assist
            rc = ce1612_cmd(s_ctrl, 0x42, data_buf, 1);
#endif			
        default:
            data_buf[0] = 0x00;
            rc = ce1612_cmd(s_ctrl, 0x1A, data_buf, 1);
        }   
    }

    rc = ce1612_cmd(s_ctrl, 0x01, NULL, 0);

    SKYCDBG("%s end\n",__func__);
    
    return rc;
}

#if 0
static int32_t ce1612_set_focus_step(int mode, int32_t focus_step)
{
	int32_t rc = 0;

	if ((focus_step < 0) || (focus_step >= CE1612_CFG_FOCUS_STEP_MAX))
	{
		SKYCERR("%s FAIL!!! return~~  focus_step = %d\n",__func__,focus_step);
		return 0;//-EINVAL;
	}

#if 0 //jjhwang
	{
		rc = ce1612_i2c_write_table(ce1612_tune_regs.focus_step_cfg_settings[focus_step],	
						ce1612_tune_regs.focus_step_cfg_settings_size);
	}

	if (rc < 0)
	{
		SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
		return rc;
	}		
#endif
	SKYCDBG("%s end\n",__func__);

	return rc;
}
#endif

static int ce1612_set_antishake(struct msm_sensor_ctrl_t *s_ctrl ,int8_t antishake)
{
    uint8_t data_buf[2];
    int32_t rc = 0;
    SKYCDBG("%s start / antishake:%d \n",__func__, antishake);

    if ((antishake < 0) || (antishake >= CE1612_CFG_ANTISHAKE_MAX))
    {
        SKYCERR("%s FAIL!!! return~~  antishake = %d\n",__func__,antishake);
        return 0;//-EINVAL;
    }

    data_buf[0] = antishake;
    rc = ce1612_cmd(s_ctrl, 0x5B, data_buf, 1);
    if (rc < 0)
    {
        SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
        return rc;
    }   

    SKYCDBG("%s end\n",__func__);
    
    return rc;
}

static int ce1612_set_exposure_mode(struct msm_sensor_ctrl_t *s_ctrl ,int8_t exposure)
{
    uint8_t data_buf[2];
    int32_t rc = 0;

    SKYCDBG("%s  exposure = %d\n",__func__, exposure);

    if ((exposure < 0) || (exposure >= CE1612_EXPOSURE_MAX))
    {
        SKYCERR("%s FAIL!!! return~~  exposure = %d\n",__func__,exposure);
        return 0;//-EINVAL;
    }

    data_buf[0] = 0x00;
    data_buf[1] = ce1612_exposure_data[exposure];//*(ce1612_tune_regs.exposure_data+exposure);
    rc = ce1612_cmd(s_ctrl, 0x04, data_buf, 2);
    if (rc < 0)
    {
        SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
        return rc;
    }   
    rc = ce1612_cmd(s_ctrl, 0x01, NULL, 0);
    if (rc < 0)
    {
        SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
        return rc;
    }   

    SKYCDBG("%s end\n",__func__);
    
    return rc;
}
static int32_t ce1612_set_focus_rect_internel(struct msm_sensor_ctrl_t *s_ctrl)
{
    uint8_t data_buf[10];
    int32_t is_af_t = 0;

    int32_t rc = 0;
    //uint8_t read_data =0;

    SKYCDBG("%s E\n",__func__);

    if(!(sensor_mode > 0 && sensor_mode < 4))
        goto set_rect_end;

    if(!(x1|y1|x2|y2)) {
        goto set_rect_end;
    }
#if 0
    rc = ce1612_cmd_read(s_ctrl, 0x24, &read_data, 1);
	SKYCDBG("%s read_data(%d)\n",__func__, read_data);
	 
    if (rc < 0)
    {
		 SKYCERR("%s err(%d)\n", __func__, rc);
		 return rc;
    }

    if(read_data & 0x01)
#endif	 	
    {
#if 1 // AF-T state check
    	rc = ce1612_cmd_read(s_ctrl, 0x2D, data_buf, 2);
    	if (rc < 0)
    	{
			SKYCERR("%s err(%d)\n", __func__, rc);
			return rc;
    	}

    	if(data_buf[0] == 0x01)
    	{
	      is_af_t  = 1;
	      data_buf[0] = 0x02;
	      rc = ce1612_cmd(s_ctrl, 0x2C, data_buf, 1);
	      if (rc < 0)
	      {
				SKYCERR("%s err(%d)\n", __func__, rc);
				return rc;
	      }	
    	}		
#endif

		rc = ce1612_lens_stop(s_ctrl);

#if 1 // AF-T state check
    	if(is_af_t  == 1)
    	{
			data_buf[0] = 0x01;
			rc = ce1612_cmd(s_ctrl, 0x2C, data_buf, 1);
			if (rc < 0)
			{
				SKYCERR("%s err(%d)\n", __func__, rc);
				return rc;
			}	
    	}		
#endif	
    }

    SKYCDBG("%s  xleft = %d, ytop = %d, xright = %d, ybottom = %d\n",__func__, x1, y1, x2, y2);

    data_buf[0] = 0x05;
    data_buf[1] = 0x03;
    data_buf[2] = x1 & 0xff;
    data_buf[3] = (x1 >> 8) & 0xff;
    data_buf[4] = y1 & 0xff;
    data_buf[5] = (y1 >> 8) & 0xff;
    data_buf[6] = x2 & 0xff;
    data_buf[7] = (x2 >> 8) & 0xff;
    data_buf[8] = y2 & 0xff;
    data_buf[9] = (y2 >> 8) & 0xff;
	
    rc = ce1612_cmd(s_ctrl, 0x41, data_buf, 10);
    if (rc < 0)
    {
		 SKYCERR("%s err(%d)\n", __func__, rc);
		 return rc;
    }	

    ce1612_42_command(s_ctrl, 0x05);	 

set_rect_end:
    
#if 0 //def CONFIG_PANTECH_CAMERA//IRQ // AF-T state check
    rc = ce1612_cmd_read(0x2D, data_buf, 2);
    if (rc < 0)
    {
        SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
        return rc;
    }

    if(data_buf[0] == 0x01)
    {
        ce1612_irq_stat = 1;	// first trigger start
    }		
#endif

	 SKYCDBG("%s: X\n",__func__);
    return rc;	
}


static int32_t ce1612_set_focus_rect(struct msm_sensor_ctrl_t *s_ctrl, int32_t focus_rect)
{
    int32_t focus_rect_size = 256;
    int32_t focus_rect_size_1 = 255;
    int32_t focus_rect_size_half_1 = 127;

    //uint8_t data_buf[10];
    int32_t x_c, y_c, xleft, xright, ytop, ybottom;
    int32_t width, height;

    int32_t rc = 0;

	 SKYCDBG("%s: sensor_mode(%d) E\n",__func__, sensor_mode);


    if(!(sensor_mode > 0 && sensor_mode < 4))
        return rc;

    if (focus_rect == 0) {
        ce1612_42_command(s_ctrl, 0x00);  
        return rc;
    }

    width = ce1612_dimensions[sensor_mode].x_output;
    height = ce1612_dimensions[sensor_mode].y_output;

#if 0
    if(height == CE1612_ZSL_SIZE_HEIGHT)
    {
        focus_rect_size = 256;
        focus_rect_size_1 = 255;
        focus_rect_size_half_1 = 127;
    }
#endif

    x_c = (int32_t)((focus_rect & 0xffff0000) >> 16);
    x_c = (x_c*width)/2000;
    y_c = (int32_t)(focus_rect & 0xffff);
    y_c = (y_c*height)/2000;

    xleft = x_c - focus_rect_size_half_1;
    if(xleft < 0)
        xleft = 0;
    if(xleft > width - focus_rect_size)
        xleft = width - focus_rect_size;

    ytop = y_c - focus_rect_size_half_1;
    if(ytop < 0)
        ytop = 0;
    if(ytop > height - focus_rect_size)
        ytop = height - focus_rect_size;

    xright = xleft + focus_rect_size_1;
    ybottom = ytop + focus_rect_size_1;

    SKYCDBG("%s  xleft = %d, ytop = %d, xright = %d, ybottom = %d\n",__func__, xleft, ytop, xright, ybottom);

    x1 = xleft;
    x2 = xright;
    y1 = ytop;
    y2 = ybottom;

	 SKYCDBG("%s: continuous_af_mode(%d) E\n",__func__, continuous_af_mode);	 

#if 1 // AF-T state check
    if(continuous_af_mode == 0)
#else
    rc = ce1612_cmd_read(s_ctrl, 0x2D, data_buf, 2);
    if (rc < 0)
    {
        SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
        return rc;
    }

    if(data_buf[0] == 0x01)
    {
        return rc;
    }		
#endif

    rc = ce1612_set_focus_rect_internel(s_ctrl);

	 SKYCDBG("%s: X\n",__func__);
    return rc;	
}

#if 0 //for rect test
static int ce1612_set_focus_rect(struct msm_sensor_ctrl_t *s_ctrl ,int32_t focus_rect)
{
#if 1//needtocheck
    int rc = 0;
    
    SKYCDBG("%s SKIP!! /needtocheck \n",__func__);
    
    return rc;
    
#else

#define FOCUS_RECT_SIZE_1 63
    uint8_t data_buf[10];
    int32_t xleft, xright, ytop, ybottom;
    int32_t rc = 0;
    uint8_t read_data =0;

    SKYCDBG("%s  focus_rect = %x\n",__func__, focus_rect);

    if (focus_rect == 0) {
        data_buf[0] = 0x00;
        rc = ce1612_cmd(s_ctrl, 0x42, data_buf, 1);

        return rc;
    }

    rc = ce1612_cmd_read(s_ctrl, 0x24, &read_data, 1);
    if (rc < 0)
    {
        SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
        return rc;
    }

    if(read_data & 0x01)
    {
#if 1 // AF-T state check
        rc = ce1612_cmd_read(s_ctrl, 0x2D, data_buf, 2);
        if (rc < 0)
        {
            SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
            return rc;
        }

        if(data_buf[0] == 0x01)
        {
            data_buf[2] = 0x02;
            rc = ce1612_cmd(s_ctrl, 0x2C, &data_buf[2], 1);
            if (rc < 0)
            {
                SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
                return rc;
            }   
        }       
#endif
        rc = ce1612_lens_stop(s_ctrl);//ce1612_cmd_poll(s_ctrl, 0x35, 0, 0); // Lens Stop
        if (rc < 0)
        {
            SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
            return rc;
        }
#if 1 // AF-T state check
        if(data_buf[0] == 0x01)
        {
            data_buf[2] = 0x01;
            rc = ce1612_cmd(s_ctrl, 0x2C, &data_buf[2], 1);
            if (rc < 0)
            {
                SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
                return rc;
            }   
        }       
#endif	
    }

    xleft= (int32_t)((focus_rect & 0xffff0000) >> 16);
    ytop = (int32_t)(focus_rect & 0xffff);
    
    xright = xleft + FOCUS_RECT_SIZE_1;
    ybottom = ytop + FOCUS_RECT_SIZE_1;
    
    SKYCDBG("%s  xleft = %d, ytop = %d, xright = %d, ybottom = %d\n",__func__, xleft, ytop, xright, ybottom);
        
    data_buf[0] = 0x05;
    data_buf[1] = 0x03;
    data_buf[2] = xleft & 0xff;
    data_buf[3] = (xleft >> 8) & 0xff;
    data_buf[4] = ytop & 0xff;
    data_buf[5] = (ytop >> 8) & 0xff;
    data_buf[6] = xright & 0xff;
    data_buf[7] = (xright >> 8) & 0xff;
    data_buf[8] = ybottom & 0xff;
    data_buf[9] = (ybottom >> 8) & 0xff;
    
    rc = ce1612_cmd(s_ctrl, 0x41, data_buf, 10);
    if (rc < 0)
    {
        SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
        return rc;
    }   

    data_buf[0] = 0x05;
    rc = ce1612_cmd(s_ctrl, 0x42, data_buf, 1);

    rc = ce1612_poll(s_ctrl, 0x43, 0x05, 10, 1000);
    if (rc < 0)
    {
        SKYCERR("%s : uploader polling ERROR !!!\n",__func__);
        return rc;
    }
    
#ifdef CONFIG_PANTECH_CAMERA//IRQ // AF-T state check
    rc = ce1612_cmd_read(s_ctrl, 0x2D, data_buf, 2);
    if (rc < 0)
    {
        SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
        return rc;
    }

    if(data_buf[0] == 0x01)
    {
        ce1612_irq_stat = 1;    // first trigger start
    }       
#endif
    
    
    SKYCDBG("%s end\n",__func__);
    
    return rc;  
#endif    
}
#endif

static int ce1612_set_preview_fps(struct msm_sensor_ctrl_t *s_ctrl ,int8_t preview_fps)
{
    /* 0 : variable 30fps, 1 ~ 30 : fixed fps */
    /* default: variable 8 ~ 30fps */
    uint8_t data_buf[4];
    int32_t rc = 0;

    if ((preview_fps < C_PANTECH_CAMERA_MIN_PREVIEW_FPS) || 
        (preview_fps > C_PANTECH_CAMERA_MAX_PREVIEW_FPS)) {
        SKYCERR("%s: -EINVAL, preview_fps=%d\n", 
            __func__, preview_fps);
        return 0; //-EINVAL;
    }

    SKYCDBG("%s: preview_fps=%d\n", __func__, preview_fps);

    if(preview_fps == C_PANTECH_CAMERA_MAX_PREVIEW_FPS)
    {
        data_buf[0] = 0x01;
        data_buf[1] = 0xFF;
        data_buf[2] = 0xFF;
        data_buf[3] = 0x00;
    }
    else
    {
        data_buf[0] = 0x01;
        data_buf[1] = preview_fps & 0xFF;
        data_buf[2] = (preview_fps >> 8) & 0xFF;
        data_buf[3] = 0x00;
    }
    
    rc = ce1612_cmd(s_ctrl, 0x5A, data_buf, 4);
    if (rc < 0)
    {
        SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
        return rc;
    }   
    current_fps = preview_fps;

    SKYCDBG("%s end rc = %d\n",__func__, rc);

    return rc;
}


static int ce1612_set_antibanding(struct msm_sensor_ctrl_t *s_ctrl ,int8_t antibanding)
{
    uint8_t data_buf[2];
    int32_t rc = 0;

    SKYCDBG("%s start / antibanding:%d \n",__func__, antibanding);

    if ((antibanding < 0) || (antibanding >= CE1612_CFG_FLICKER_MAX))
    {
        SKYCERR("%s FAIL!!! return~~  antibanding = %d\n",__func__,antibanding);
        return 0;//-EINVAL;
    }

    data_buf[0] = ce1612_flicker_data[antibanding];//*(ce1612_tune_regs.exposure_data+antibanding);
    rc = ce1612_cmd(s_ctrl, 0x14, data_buf, 1);
    if (rc < 0)
    {
        SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
        return rc;
    }
    
    SKYCDBG("%s end\n",__func__);
    
    return rc;
}


static int ce1612_set_reflect(struct msm_sensor_ctrl_t *s_ctrl ,int8_t reflect)
{
    int32_t rc = 0;
#if 0 //jjhwang
    int32_t i = 0;
    //int8_t npolling = -1;
    uint16_t npolling = -1;

    SKYCDBG("%s  reflect = %d\n",__func__, reflect);

    if ((reflect < 0) || (reflect >= CE1612_CFG_REFLECT_MAX))
    {
        SKYCERR("%s FAIL!!! return~~  reflect = %d\n",__func__,reflect);
        return 0;//-EINVAL;
    }

    {
        rc = ce1612_i2c_write_table(ce1612_tune_regs.reflect_cfg_settings[reflect],
                    ce1612_tune_regs.reflect_cfg_settings_size);
    }

    for (i = 0; i < CE1612_POLLING_RETRY; i++) {
        rc = ce1612_i2c_read(ce1612_saddr,
        0x8404, &npolling,TRIPLE_LEN);
        if (rc < 0)
        {
            SKYCERR("ERR:%s POLLING FAIL!!!rc=%d return~~\n", __func__, rc);
            //return rc;
        }
        SKYCDBG("%s: retry npolling = %x, count = %d\n", __func__,npolling, i);
        if (npolling == 0) {                
            return 0;
        }       
        msleep(20);
    }   
 
    if (rc < 0)
    {
        SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
        return rc;
    }       
#endif
    SKYCDBG("%s end\n",__func__);

    return rc;
}

static int ce1612_set_continuous_af(struct msm_sensor_ctrl_t *s_ctrl ,int8_t caf)
{
#if 1//needtocheck
int rc = 0;

SKYCDBG("%s SKIP!! /needtocheck \n",__func__);

return rc;

#else
    uint8_t data_buf[2];
    uint8_t read_data =0;
    
    int rc = 0;

    SKYCDBG("%s start : caf = %d\n",__func__, caf);
    caf_camera_flag = caf;

    rc = ce1612_cmd_read(s_ctrl, 0x2D, &read_data, 1);  // check AF-T
    if(read_data == 1)
    {
        return 0;
    }

    if(caf)
    {
        rc = ce1612_cmd_read(s_ctrl, 0x24, &read_data, 1);
        if (rc < 0)
        {
            SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
            return rc;
        }

        if(read_data & 0x01)
        {
            rc = ce1612_lens_stop(s_ctrl);//ce1612_cmd_poll(ce1612_saddr, 0x35, 0, 0); // Lens Stop
            if (rc < 0)
            {
                SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
                return rc;
            }   
        }
    
        data_buf[0] = 0x02;
        //rc = ce1612_cmd_poll(s_ctrl, 0x20, data_buf, 1);
        rc = ce1612_cmd(s_ctrl, 0x20, data_buf, 1);
        rc = ce1612_poll_bit(s_ctrl, 0x24, 10, 400);
        
        if (rc < 0)
        {
            SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
            return rc;
        }       
          
        SKYCDBG("AF-C start\n");
        rc = ce1612_cmd(s_ctrl, 0x23, 0, 0);
        if (rc < 0)
        {
            SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
            return rc;
        }
    }
    else
    {
#if 0
        rc = ce1612_cmd(ce1612_saddr, 0x35, 0, 0);
#else
        rc = ce1612_lens_stop(s_ctrl);//ce1612_cmd_poll(s_ctrl, 0x35, 0, 0);
#endif
        if (rc < 0)
        {
            SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
            return rc;
        }
        
        data_buf[0] = 0x00;
        //rc = ce1612_cmd_poll(s_ctrl, 0x20, data_buf, 1);
        rc = ce1612_cmd(s_ctrl, 0x20, data_buf, 1);
        rc = ce1612_poll_bit(s_ctrl, 0x24, 10, 400);
        
        if (rc < 0)
        {
            SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
            return rc;
        }       
        
    }

    SKYCDBG("%s end\n",__func__);
    
    return rc;
#endif
}


static int ce1612_set_wdr(struct msm_sensor_ctrl_t *s_ctrl, int8_t wdr)
{
    uint8_t data_buf[2];
    int32_t rc = 0;

    SKYCDBG("%s start / wdr:%d \n",__func__, wdr);

    if ((wdr != 0) && (wdr != 1))
    {
        SKYCERR("%s FAIL!!! return~~  wdr = %d\n",__func__,wdr);
        return 0;//-EINVAL;
    }

    data_buf[0] = wdr;
    rc = ce1612_cmd(s_ctrl, 0x88, data_buf, 1);
    if (rc < 0)
    {
        SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
        return rc;
    }   

    SKYCDBG("%s end\n",__func__);
    
    return rc;
}

static int32_t ce1612_set_caf_camera(struct msm_sensor_ctrl_t *s_ctrl, int32_t caf)
{
	uint8_t data_buf[2];	
	int rc = 0;

	SKYCDBG("%s start : caf = %d\n",__func__, caf);

	if ((caf != 0) && (caf != 1))
	{
		SKYCERR("%s FAIL!!! return~~  caf = %d\n",__func__,caf);
		return 0;//-EINVAL;
	}
	caf_camera_flag = caf;
#ifdef CONFIG_PANTECH_CAMERA//IRQ
	if(caf == 1)
		ce1612_irq_stat = 1;	// first trigger start
#endif

	data_buf[0] = 0x00;
	//rc = ce1612_cmd_poll(s_ctrl, 0x20, data_buf, 1); //set focus mode normal
	rc = ce1612_cmd(s_ctrl, 0x20, data_buf, 1);
    rc = ce1612_poll_bit(s_ctrl, 0x24, 10, 400);

	data_buf[0] = caf;
	rc = ce1612_cmd(s_ctrl, 0x2C, data_buf, 1);
	if (rc < 0)
	{
		SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
		return rc;
	}	

	SKYCDBG("%s end\n",__func__);
	
	return rc;
}

static int32_t ce1612_lens_stability(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	uint8_t data_buf[4];
	uint8_t rdata = 0;

	SKYCDBG("%s  start\n",__func__);

	rc = ce1612_cmd_read(s_ctrl, 0x6C, &rdata, 1);

	if((rdata > 0) && (rdata < 8))
	{
		return 0;
	}
	
	rc = ce1612_lens_stop(s_ctrl);//ce1612_cmd_poll(ce1612_saddr, 0x35, 0, 0);	// Lens Stop
	if (rc < 0)
	{
		SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
		return rc;
	}	

	caf_camera_flag = 0;
	ce1612_set_caf_camera(s_ctrl, caf_camera_flag); // AF-T stop	

#if 0
	data_buf[0] = 0x01;
	data_buf[0] = 0x00;
	data_buf[0] = 0x00;
	rc = ce1612_cmd(ce1612_saddr, 0x33, data_buf, 3);
	if (rc < 0)
	{
		SKYCERR("ERR:%s FAIL!!!rc=%d return~~\n", __func__, rc);
		return rc;
	}
#else
	data_buf[0] = 0x00;
	//rc = ce1612_cmd_poll(s_ctrl, 0x20, data_buf, 1);
    rc = ce1612_cmd(s_ctrl, 0x20, data_buf, 1);
    rc = ce1612_poll_bit(s_ctrl, 0x24, 10, 400);
#endif
	SKYCDBG("%s END~\n",__func__);

	return rc;
}
#endif

int32_t ce1612_sensor_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int rc = 0;
	struct msm_sensor_ctrl_t *s_ctrl;
	CDBG("%s_i2c_probe called\n", client->name);
	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		CDBG("i2c_check_functionality failed\n");
		rc = -EFAULT;
		//return rc;
		goto probe_fail;
	}

	s_ctrl = (struct msm_sensor_ctrl_t *)(id->driver_data);
	if (s_ctrl->sensor_i2c_client != NULL) {
		s_ctrl->sensor_i2c_client->client = client;
		if (s_ctrl->sensor_i2c_addr != 0)
			s_ctrl->sensor_i2c_client->client->addr = s_ctrl->sensor_i2c_addr;
	} else {
		rc = -EFAULT;
		//return rc;
		goto probe_fail;
	}

	s_ctrl->sensordata = client->dev.platform_data;

#ifdef F_FW_UPDATE
	// ISP firmware update on probing
	ce1612_update_fw_boot(s_ctrl, s_ctrl->sensordata);
#endif

	snprintf(s_ctrl->sensor_v4l2_subdev.name,
		sizeof(s_ctrl->sensor_v4l2_subdev.name), "%s", id->name);
		v4l2_i2c_subdev_init(&s_ctrl->sensor_v4l2_subdev, client,
		s_ctrl->sensor_v4l2_subdev_ops);

	msm_sensor_register(&s_ctrl->sensor_v4l2_subdev);
	goto i2c_probe_end;
	
probe_fail:
	CDBG("%s_i2c_probe failed\n", client->name);
	
i2c_probe_end:
    CDBG("%s_i2c_probe end\n", client->name);
	return rc;
}

static int __init msm_sensor_init_module(void)
{
	return i2c_add_driver(&ce1612_i2c_driver);
}

static struct v4l2_subdev_core_ops ce1612_subdev_core_ops = {
	.ioctl = msm_sensor_subdev_ioctl,
	.s_power = msm_sensor_power,
};

static struct v4l2_subdev_video_ops ce1612_subdev_video_ops = {
	.enum_mbus_fmt = msm_sensor_v4l2_enum_fmt,
};

static struct v4l2_subdev_ops ce1612_subdev_ops = {
	.core = &ce1612_subdev_core_ops,
	.video  = &ce1612_subdev_video_ops,
};

static struct msm_sensor_fn_t ce1612_func_tbl = {
#if 1//def F_STREAM_ON_OFF
	.sensor_start_stream = ce1612_sensor_start_stream, //msm_sensor_start_stream,
	.sensor_stop_stream = ce1612_sensor_stop_stream, //msm_sensor_stop_stream,
#endif
#if 0
	.sensor_group_hold_on = msm_sensor_group_hold_on,
	.sensor_group_hold_off = msm_sensor_group_hold_off,
	.sensor_set_fps = msm_sensor_set_fps,
	.sensor_write_exp_gain = msm_sensor_write_exp_gain1,
//	.sensor_write_snapshot_exp_gain = msm_sensor_write_exp_gain1,
#endif
    .sensor_setting = ce1612_sensor_setting, //msm_sensor_setting,
	.sensor_csi_setting = ce1612_sensor_setting1,//msm_sensor_setting1,
	.sensor_set_sensor_mode = msm_sensor_set_sensor_mode,
	.sensor_mode_init = msm_sensor_mode_init,
	.sensor_get_output_info = msm_sensor_get_output_info,
	.sensor_config = msm_sensor_config,
#ifdef F_CE1612_POWER
    .sensor_power_up = ce1612_sensor_power_up,//msm_sensor_power_up,
    .sensor_power_down = ce1612_sensor_power_down,//msm_sensor_power_down,
#else
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = msm_sensor_power_down,
#endif
//	.sensor_adjust_frame_lines = msm_sensor_adjust_frame_lines,
	.sensor_get_csi_params = msm_sensor_get_csi_params,
#ifdef CONFIG_PANTECH_CAMERA
    .sensor_set_effect = ce1612_set_effect,
    .sensor_set_wb = ce1612_set_whitebalance,
    .sensor_set_brightness = ce1612_set_brightness,
    .sensor_set_led_mode = ce1612_set_led_mode,			
    .sensor_set_auto_focus = ce1612_set_auto_focus,
    .sensor_check_af = ce1612_sensor_check_af,
    .sensor_set_iso = ce1612_set_iso,
    .sensor_set_scene_mode = ce1612_set_scene_mode,
    //ce1612_set_focus_step,
    .sensor_set_antishake = ce1612_set_antishake,
    .sensor_set_exposure_mode = ce1612_set_exposure_mode,
    .sensor_set_focus_rect = ce1612_set_focus_rect,
    .sensor_set_preview_fps = ce1612_set_preview_fps,
    .sensor_set_antibanding = ce1612_set_antibanding,
    .sensor_set_reflect = ce1612_set_reflect,
    .sensor_set_coninuous_af = ce1612_set_continuous_af,
    .sensor_set_wdr = ce1612_set_wdr,
    .sensor_lens_stability = ce1612_lens_stability
#endif
};

static struct msm_sensor_reg_t ce1612_regs = {
	.default_data_type = MSM_CAMERA_I2C_BYTE_DATA,
#if 0//def F_STREAM_ON_OFF	        
	.start_stream_conf = ce1612_start_settings,
	.start_stream_conf_size = ARRAY_SIZE(ce1612_start_settings),
	.stop_stream_conf = ce1612_stop_settings,
	.stop_stream_conf_size = ARRAY_SIZE(ce1612_stop_settings),
#endif
#if 0
	.group_hold_on_conf = ce1612_groupon_settings,
	.group_hold_on_conf_size = ARRAY_SIZE(ce1612_groupon_settings),
	.group_hold_off_conf = ce1612_groupoff_settings,
	.group_hold_off_conf_size =
		ARRAY_SIZE(ce1612_groupoff_settings),
#endif
	.init_settings = NULL,//&ce1612_init_conf[0],
	.init_size = 0,//ARRAY_SIZE(ce1612_init_conf),
	.mode_settings = NULL,//&ce1612_confs[0],
	.output_settings = &ce1612_dimensions[0],
	.num_conf = ARRAY_SIZE(ce1612_cid_cfg),//ARRAY_SIZE(ce1612_confs),
};

static struct msm_sensor_ctrl_t ce1612_s_ctrl = {
	.msm_sensor_reg = &ce1612_regs,
	.sensor_i2c_client = &ce1612_sensor_i2c_client,
	//needtocheck
#if defined(CONFIG_MACH_MSM8X60_EF40S) ||defined(CONFIG_MACH_MSM8X60_EF40K) //EF40S/K
	.sensor_i2c_addr = 0x7C,//0x3E;
#elif defined(CONFIG_MACH_MSM8X60_EF65L)	//EF65L
	.sensor_i2c_addr = 0x7C,//0x3E;
#else	// EF39S
	.sensor_i2c_addr = 0x78,//0x3C;
#endif
//	.sensor_output_reg_addr = &ce1612_reg_addr,
	.sensor_id_info = &ce1612_id_info,
//	.sensor_exp_gain_info = &ce1612_exp_gain_info,
	.cam_mode = MSM_SENSOR_MODE_INVALID,
	.csic_params = &ce1612_csic_params_array[0],
	.csi_params = &ce1612_csi_params_array[0],
	.msm_sensor_mutex = &ce1612_mut,
	.sensor_i2c_driver = &ce1612_i2c_driver,
	.sensor_v4l2_subdev_info = ce1612_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(ce1612_subdev_info),
	.sensor_v4l2_subdev_ops = &ce1612_subdev_ops,
	.func_tbl = &ce1612_func_tbl,
	.clk_rate = MSM_SENSOR_MCLK_24HZ,
};

late_initcall(msm_sensor_init_module);
MODULE_DESCRIPTION("ISP CE1612 8MP YUV sensor driver");
MODULE_LICENSE("GPL v2");

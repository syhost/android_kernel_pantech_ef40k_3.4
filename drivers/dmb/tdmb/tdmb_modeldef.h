//=============================================================================
// File       : Tdmb_modeldef.h
//
// Description: 
//
// Revision History:
//
// Version         Date           Author        Description of Changes
//-----------------------------------------------------------------------------
//  1.0.0       2010/12/06       yschoi         Create
//                                              (tdmb_dev.h, tdmb_comdef.h 에서 분리)
//=============================================================================

#ifndef _TDMB_MODELDEF_INCLUDES_H_
#define _TDMB_MODELDEF_INCLUDES_H_

#ifdef CONFIG_ARCH_TEGRA
#include <../gpio-names.h>
#endif

/*================================================================== */
/*================     MODEL FEATURE               ================= */
/*================================================================== */

#if 0
/////////////////////////////////////////////////////////////////////////
// DMB GPIO (depend on model H/W)
// 중복되어 define 된 모델이 있으므로 사용하는 모델을 맨앞으로 가져온다.
/////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////
#elif defined(CONFIG_MACH_MSM8X60_EF65L)
  #define DMB_RESET     102
  #define DMB_INT       127
  #define DMB_PWR_EN    86
  //#define FEATURE_DMB_SET_ANT_PATH
  //#define DMB_ANT_SEL   42
  #define DMB_ANT_EAR_ACT   1 
  #define FEATURE_DMB_CLK_19200
  #define DMB_XO_SEL 41
  #define FEATURE_DMB_GPIO_INIT
  #define FEATURE_DMB_THREAD
#ifdef CONFIG_SKY_DMB_I2C_CMD
  #define FEATURE_DMB_THREAD_FIC_BUF
#endif
  #define FEATURE_DMB_AUTOSCAN_DISCRETE

/////////////////////////////////////////////////////////////////////////
#elif (defined(CONFIG_MACH_MSM8X60_EF40S) || defined(CONFIG_MACH_MSM8X60_EF40K))
  #define DMB_RESET     102
  #define DMB_INT       127
  #define DMB_PWR_EN    86
  #define DMB_ANT_SEL   42
  #define DMB_ANT_EAR_ACT   1 
#if 1//(BOARD_REV > WS20)
  #define FEATURE_DMB_CLK_19200
  #define DMB_XO_SEL 19
#else
  #define FEATURE_DMB_CLK_24576
#endif
  //#define FEATURE_DMB_PMIC_POWER
  #define FEATURE_DMB_SET_ANT_PATH
  #define FEATURE_DMB_SET_ANT_PATH_POWER
  #define FEATURE_DMB_PMIC8058
  #define FEATURE_DMB_GPIO_INIT
  #define FEATURE_DMB_THREAD
#ifdef CONFIG_SKY_DMB_I2C_CMD
  #define FEATURE_DMB_THREAD_FIC_BUF
#endif
  #define FEATURE_DMB_AUTOSCAN_DISCRETE

/////////////////////////////////////////////////////////////////////////
#elif defined(CONFIG_MACH_MSM8X60_EF39S)
  #define DMB_RESET     102
  #define DMB_INT       127
  #define DMB_PWR_EN    86
  #define DMB_ANT_SEL   42
#if 1//(BOARD_REV > WS10)
  #define DMB_ANT_EAR_ACT   0
#else
  #define DMB_ANT_EAR_ACT   1
#endif
#if 1//(BOARD_REV > WS20)
  #define FEATURE_DMB_CLK_19200
  #define DMB_XO_SEL 19
#else
  #define FEATURE_DMB_CLK_24576
#endif
  //#define FEATURE_DMB_PMIC_POWER // TP20 에서 잠시 검토
  #define FEATURE_DMB_SET_ANT_PATH
  #define FEATURE_DMB_SET_ANT_PATH_POWER
  #define FEATURE_DMB_PMIC8058
  #define FEATURE_DMB_GPIO_INIT
  #define FEATURE_DMB_THREAD
#ifdef CONFIG_SKY_DMB_I2C_CMD
  #define FEATURE_DMB_THREAD_FIC_BUF
#endif
  #define FEATURE_DMB_AUTOSCAN_DISCRETE

/////////////////////////////////////////////////////////////////////////
#elif defined(CONFIG_MACH_MSM8X60_EF35L)
  #define DMB_RESET     129
  #define DMB_INT       127
  #define DMB_PWR_EN    136
  #define DMB_I2C_SCL    73
  #define DMB_I2C_SDA    72
#if 1//EF35L_BDVER_GE(TP10)
  #define FEATURE_DMB_CLK_19200
  #define DMB_XO_SEL 42
#else
  #define FEATURE_TDMB_SET_ANT_PATH
  #define DMB_ANT_SEL   42
  #define DMB_ANT_EAR_ACT   1
#endif
    //#define FEATURE_TDMB_PMIC_POWER
  #define FEATURE_DMB_GPIO_INIT
  #define FEATURE_DMB_THREAD
#ifdef CONFIG_SKY_DMB_I2C_CMD
  #define FEATURE_DMB_THREAD_FIC_BUF
#endif
  #define FEATURE_DMB_AUTOSCAN_DISCRETE

/////////////////////////////////////////////////////////////////////////
#elif (defined(CONFIG_MACH_MSM8X60_EF33S) || defined(CONFIG_MACH_MSM8X60_EF34K))
  #define DMB_RESET     129
  #define DMB_INT       127
  #define DMB_PWR_EN    136
  #define DMB_I2C_SCL    73
  #define DMB_I2C_SDA    72
#if 0//(EF33S_BDVER_L(WS20) || EF34K_BDVER_L(WS20)) // Since WS20 use only retractable Antenna
  #define DMB_ANT_SEL   42
  #define DMB_ANT_EAR_ACT   1
  #define FEATURE_DMB_SET_ANT_PATH
#endif
  //#define FEATURE_DMB_PMIC_POWER
  #define FEATURE_DMB_GPIO_INIT
  #define FEATURE_DMB_CLK_24576
  #define FEATURE_DMB_THREAD
#ifdef CONFIG_SKY_DMB_I2C_CMD
  #define FEATURE_DMB_THREAD_FIC_BUF
#endif
  #define FEATURE_DMB_AUTOSCAN_DISCRETE

#else
  #error
#endif


/*================================================================== */
/*================     TEST FEATURE                ================= */
/*================================================================== */

//#define FEATURE_TS_PKT_MSG // Single CH : 모든 패킷을 보여줌, Mulch CH : 읽은 데이터의 첫 패킷만 보여줌.
//#define FEATURE_TEST_ON_BOOT
//#define FEATURE_TEST_READ_DATA_ON_BOOT
//#define FEATURE_NETBER_TEST_ON_BOOT
//#define FEATURE_NETBER_TEST_ON_AIR
//#define FEATURE_DMB_DUMP_FILE
//#define FEATURE_APP_CALL_TEST_FUNC
//#define FEAUTRE_USE_FIXED_FIC_DATA
//#define FEATURE_DMB_GPIO_DEBUG => dmb_hw.c 로 이동 (빌드 단축)
//#define FEATURE_COMMAND_TEST_ON_BOOT
//#define FEATURE_DMB_I2C_WRITE_CHECK => dmb_i2c.c 로 이동 (빌드 단축)
//#define FEATURE_DMB_I2C_DBG_MSG => dmb_i2c.c 로 이동 (빌드 단축)
//#define FEATURE_EBI_WRITE_CHECK
//#define FEATURE_HW_INPUT_MATCHING_TEST

#ifndef CONFIG_SKY_DMB_TSIF_IF
#if (defined(FEATURE_TEST_ON_BOOT) && !defined(FEATURE_DMB_THREAD)) // 부팅테스트시 쓰레드로 돌려 데이터 계속 읽음
#define FEATURE_DMB_THREAD
#endif
#endif

#ifdef CONFIG_SKY_TDMB_INC_BB
#if (defined(FEATURE_TEST_ON_BOOT) || defined(FEATURE_NETBER_TEST_ON_BOOT) || defined(FEATURE_APP_CALL_TEST_FUNC))
#define INC_FICDEC_USE // 테스트시 채널정보들이 정확하지 않을때 필요
#define FEATURE_INC_FIC_UPDATE_MSG
#endif
#endif /* CONFIG_SKY_TDMB_INC_BB */


#endif /* _TDMB_MODELDEF_INCLUDES_H_ */

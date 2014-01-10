#!/bin/bash
###############################################################################
#
#                           Kernel Build Script 
#
###############################################################################
# 2010-12-29 allydrop     : created
###############################################################################

if [ ${CMD_V_KERNEL_BUILD} = "no" ]
then
export OEM_PRODUCT_MANUFACTURER=PANTECH
export SYS_MODEL_NAME=EF39S
export MODEL_NAME=MODEL_EF39S
export SYS_MODEL_BOARD_VER=EV10
export SKY_MODEL_SW_VER_PREFIX=S0000
export SKY_MODEL_SW_VER_MJ=0
export SKY_MODEL_SW_VER_MN=00
export SKY_MODEL_SW_VER_POSTFIX=$SKY_MODEL_SW_VER_MJ$SKY_MODEL_SW_VER_MN
fi

#export TARGET_BUILD_SKY_MODEL_ID=MODEL_"$SYS_MODEL_NAME"
export CMD_D_ANDROID_OUT=../out/target/product/msm8660_surf
export TARGET_BUILD_SKY_FIRMWARE_VER=$SKY_MODEL_SW_VER_PREFIX$SKY_MODEL_SW_VER_POSTFIX
#export BOARD_REV_INCLUDE=../../../BOARD_REV.h
export TARGET_BUILD_SKY_BOARD_REV_INCLUDE_DIR="../../../../../../../"
export TARGET_BUILD_SKY_BOARD_REV_H="BOARD_REV.h"
export TARGET_BUILD_USER_DATA_VER=$CMD_V_USER_DATA_VER

export SKY_KERNEL_FLAGS

SKY_KERNEL_FLAGS+="-I$TARGET_BUILD_SKY_BOARD_REV_INCLUDE_DIR -include $TARGET_BUILD_SKY_BOARD_REV_H -DFIRM_VER=\\\"$TARGET_BUILD_SKY_FIRMWARE_VER\\\" -DSYS_MODEL_NAME=\\\"$SYS_MODEL_NAME\\\" -DSKY_MODEL_NAME=\\\"$SKY_MODEL_NAME\\\" -DFS_USER_DATA_VER=$TARGET_BUILD_USER_DATA_VER"

echo "$SKY_KERNEL_FLAGS"

export ARCH=arm
export CROSS_COMPILE=../prebuilt/linux-x86/toolchain/arm-eabi-4.4.0/bin/arm-eabi-

make 


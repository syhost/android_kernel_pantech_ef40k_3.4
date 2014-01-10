How to Build
    1. Get Toolchain (arm-eabi-4.6) and install
       (Visit android git server or codesourcery)

    2. modify below lines in build_kernel.sh and run
          1) modify below lines
                $ export ARCH=arm
                $ export PATH=$(pwd)/../../toolchain_arm-eabi-4.6/arm-eabi-4.6/bin:$PATH
                $ export CROSS_COMPILE=arm-eabi-
          2) run build_kernel.sh
                $ ./build_kernel.sh

    3.Output Files
      -	kernel : kernel/arch/arm/boot/zImaze
      -	module : kernel/drivers/*/*.ko

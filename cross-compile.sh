PLATFORM=arm-openwrt-linux
STAGING_DIR=/raid/Files/openwrt/pi2/staging_dir/toolchain-arm_cortex-a7+vfp_gcc-4.8-linaro_glibc-2.21_eabi
TARGET_DIR=/raid/Files/openwrt/pi2/staging_dir/target-arm_cortex-a7+vfp_glibc-2.21_eabi
export STAGING_DIR

PATH=$PATH:$STAGING_DIR/bin
export PATH

make CC=$PLATFORM-gcc CFLAGS="-O3 -I$TARGET_DIR/include -I$TARGET_DIR/usr/include" LDFLAGS="-L$TARGET_DIR/usr/lib -L$TARGET_DIR/lib -Wl,-rpath -Wl,$TARGET_DIR/usr/lib -Wl,-rpath -Wl,$TARGET_DIR/lib"


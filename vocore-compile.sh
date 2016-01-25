PLATFORM=mipsel-openwrt-linux
OPENWRT_DIR=/raid/Files/openwrt/vocore
STAGING_DIR=$OPENWRT_DIR/staging_dir/toolchain-mipsel_24kec+dsp_gcc-4.8-linaro_uClibc-0.9.33.2
TARGET_DIR=$OPENWRT_DIR/staging_dir/target-mipsel_24kec+dsp_uClibc-0.9.33.2
export STAGING_DIR

PATH=$PATH:$STAGING_DIR/bin
export PATH

make CC=$PLATFORM-gcc CFLAGS="-O3 -I$TARGET_DIR/include -I$TARGET_DIR/usr/include" LDFLAGS="-L$TARGET_DIR/usr/lib -L$TARGET_DIR/lib -Wl,-rpath -Wl,$TARGET_DIR/usr/lib -Wl,-rpath -Wl,$TARGET_DIR/lib"


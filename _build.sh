#!/bin/bash

KERNEL_DIR=$PWD
BUILD_RADISK=$1

cpoy_initramfs()
{
  if [ -d $INITRAMFS_TMP_DIR ]; then
    rm -rf $INITRAMFS_TMP_DIR  
  fi
  cp -a $INITRAMFS_SRC_DIR $(dirname $INITRAMFS_TMP_DIR)
  rm -rf $INITRAMFS_TMP_DIR/.git
  find $INITRAMFS_TMP_DIR -name .gitignore | xargs rm
}

# check target
BUILD_TARGET=$1
case "$BUILD_TARGET" in
  "boot" ) BUILD_DEFCONFIG=padfone2_boot_defconfig ;;
  "recovery" ) BUILD_DEFCONFIG=padfone2_recovery_defconfig ;;
  * ) echo "error: not found BUILD_TARGET" && exit -1 ;;
esac
BIN_DIR=out/$BUILD_TARGET/bin
OBJ_DIR=out/$BUILD_TARGET/obj
mkdir -p $BIN_DIR
mkdir -p $OBJ_DIR

# generate LOCALVERSION
. mod_version

# check and get compiler
. cross_compile

# set build env
export ARCH=arm
export CROSS_COMPILE=$BUILD_CROSS_COMPILE
export LOCALVERSION="-$BUILD_LOCALVERSION"

echo "=====> BUILD START $BUILD_KERNELVERSION-$BUILD_LOCALVERSION"

if [ ! -n "$2" ]; then
  echo ""
  read -p "select build? [(a)ll/(u)pdate/(i)mage default:update] " BUILD_SELECT
else
  BUILD_SELECT=$2
fi

# copy initramfs
if [ "$BUILD_TARGET" = 'recovery' -o "$BUILD_TARGET" = 'r' ]; then
  INITRAMFS_SRC_DIR=../padfone2_recovery_ramdisk
  INITRAMFS_TMP_DIR=/tmp/padfone2_recovery_ramdisk
  IMAGE_NAME=recovery
else
  INITRAMFS_SRC_DIR=../padfone2_boot_ramdisk
  INITRAMFS_TMP_DIR=/tmp/padfone2_boot_ramdisk
  IMAGE_NAME=boot
fi
echo ""
echo "=====> copy initramfs"
cpoy_initramfs

# make start
if [ "$BUILD_SELECT" = 'all' -o "$BUILD_SELECT" = 'a' ]; then
  echo ""
  echo "=====> cleaning"
  make clean
  cp -f ./arch/arm/configs/$BUILD_DEFCONFIG $OBJ_DIR/.config
  make -C $PWD O=$OBJ_DIR oldconfig || exit -1
fi

if [ "$BUILD_SELECT" != 'image' -a "$BUILD_SELECT" != 'i' ]; then
  echo ""
  echo "=====> build start"
  if [ -e make.log ]; then
    mv make.log make_old.log
  fi
  nice -n 10 make O=$OBJ_DIR -j12 2>&1 | tee make.log
fi

# check compile error
COMPILE_ERROR=`grep 'error:' ./make.log`
if [ "$COMPILE_ERROR" ]; then
  echo ""
  echo "=====> ERROR"
  grep 'error:' ./make.log
  exit -1
fi

echo ""
echo "=====> CREATE RELEASE IMAGE"
# clean release dir
if [ `find $BIN_DIR -type f | wc -l` -gt 0 ]; then
  rm -rf $BIN_DIR/*
fi
mkdir -p $BIN_DIR

# copy zImage
cp $OBJ_DIR/arch/arm/boot/zImage $BIN_DIR/kernel
echo "----- Making uncompressed $IMAGE_NAME ramdisk ------"
./release-tools/mkbootfs $INITRAMFS_TMP_DIR > $BIN_DIR/ramdisk-$IMAGE_NAME.cpio
echo "----- Making $IMAGE_NAME ramdisk ------"
./release-tools/minigzip < $BIN_DIR/ramdisk-$IMAGE_NAME.cpio > $BIN_DIR/ramdisk-$IMAGE_NAME.gz
echo "----- Making $IMAGE_NAME image ------"
./release-tools/mkbootimg --cmdline "console=ttyHSL0,115200,n8 androidboot.hardware=qcom user_debug=31 msm_rtb.filter=0x3F ehci-hcd.park=3" --kernel $BIN_DIR/kernel  --ramdisk $BIN_DIR/ramdisk-$IMAGE_NAME.gz --base 0x80200000 --pagesize 2048 --ramdiskaddr 0x81600000 --output $BIN_DIR/$IMAGE_NAME.img

# create cwm image
if [ -d tmp ]; then
  rm -rf tmp
fi
mkdir -p ./tmp/META-INF/com/google/android
cp $BIN_DIR/$IMAGE_NAME.img ./tmp/
cp $KERNEL_DIR/release-tools/update-binary ./tmp/META-INF/com/google/android/
sed -e "s/@VERSION/$BUILD_LOCALVERSION/g" $KERNEL_DIR/release-tools/updater-script-$IMAGE_NAME.sed > ./tmp/META-INF/com/google/android/updater-script
cd tmp && zip -rq ../cwm.zip ./* && cd ../
SIGNAPK_DIR=$KERNEL_DIR/release-tools/signapk
java -jar $SIGNAPK_DIR/signapk.jar $SIGNAPK_DIR/testkey.x509.pem $SIGNAPK_DIR/testkey.pk8 cwm.zip $BIN_DIR/$BUILD_LOCALVERSION-signed.zip
rm cwm.zip
rm -rf tmp
echo "  $BIN_DIR/$BUILD_LOCALVERSION.zip"

cd $KERNEL_DIR
echo ""
echo "=====> BUILD COMPLETE $BUILD_KERNELVERSION-$BUILD_LOCALVERSION"
exit 0

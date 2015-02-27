#!/bin/bash

set -e

PACKAGE_DIR=$(rospack find mv_driver)
echo "### downloading and unpacking drivers to" $PACKAGE_DIR "###"

BLUEFOX_URL="http://n.ethz.ch/~acmarkus/download/matrixvision_driver"

#BLUEFOX_FILE=""

API=mvIMPACT_acquire

BLUEFOX_VERSION=2.5.4
ABI=ABI2

LINKER_PATHS=$PACKAGE_DIR/linker_paths
COMPILER_FLAGS=$PACKAGE_DIR/compile_flags

TARGET=$(uname -m)
if [ "$TARGET" == "i686" ]; then
    TARGET=x86
else
    TARGET=x86_64
fi

BLUEFOX_NAME=mvBlueFOX
BLUEFOX_TARNAME=$BLUEFOX_NAME-$TARGET"_"$ABI-$BLUEFOX_VERSION

# cleanup first
rm -rf  $BLUEFOX_NAME* $API* tmp $LINKER_PATHS $COMPILER_FLAGS

#### bluefox runtime ####
# unpack
mkdir -p $PACKAGE_DIR/tmp/$BLUEFOX_NAME
cd $PACKAGE_DIR/tmp/$BLUEFOX_NAME
tar -xf $PACKAGE_DIR/download/$BLUEFOX_TARNAME.tar --overwrite

# copy blueFOX runtime libs
mkdir -p $PACKAGE_DIR/$BLUEFOX_NAME"_runtime/lib/"$TARGET
cp $PACKAGE_DIR/tmp/$BLUEFOX_NAME/$API-$TARGET-$BLUEFOX_VERSION/lib/$TARGET/libmvBlueFOX.* $PACKAGE_DIR/$BLUEFOX_NAME"_runtime/lib/"$TARGET

# copy udev rule
mkdir -p $PACKAGE_DIR/$BLUEFOX_NAME"_scripts"
cp $PACKAGE_DIR/tmp/$BLUEFOX_NAME/$API-$TARGET-$BLUEFOX_VERSION/Scripts/51-mvbf.rules $PACKAGE_DIR/$BLUEFOX_NAME"_scripts"

echo -L$PACKAGE_DIR/$BLUEFOX_NAME"_runtime/lib/"$TARGET >> $LINKER_PATHS

#### mvImpact (device independent stuff) ####
cd $PACKAGE_DIR
mv $PACKAGE_DIR/tmp/$BLUEFOX_NAME/$API-$TARGET-$BLUEFOX_VERSION $PACKAGE_DIR/$API
echo -L$PACKAGE_DIR/$API/lib/$TARGET >> $LINKER_PATHS
echo -I$PACKAGE_DIR/$API >> $COMPILER_FLAGS

#### clean up ####
rm -rf $PACKAGE_DIR/tmp

#### note down that this is done ####
touch $PACKAGE_DIR/downloadandinstall


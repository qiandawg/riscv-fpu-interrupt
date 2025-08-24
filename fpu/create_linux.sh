PROJECT_DIR=zybo_linux
HDL_XSA_DIR=./vivado/radio_periph_lab.xsa
ROOTFS_CONFIG_FILE=rootfs_config
CONFIG_FILE=config

# create a project
petalinux-create -t project -n $PROJECT_DIR --template zynq

# create a base project which is built on the XSA file exported from the hardware design
petalinux-config --get-hw-description $HDL_XSA_DIR --silentconfig -p $PROJECT_DIR

# point the config file just created (default) to our custom config file - which
# is a modified version that adds a couple of features.  We will just make a link
# so our config file can stay at the base level of the repository
# if you would rather select packages manually, you can be led through the menus
cp $CONFIG_FILE $PROJECT_DIR/project-spec/configs/config
petalinux-config -p $PROJECT_DIR --silentconfig
cp $ROOTFS_CONFIG_FILE $PROJECT_DIR/project-spec/configs/rootfs_config
petalinux-config -p $PROJECT_DIR -c rootfs --silentconfig

# build the entire project, this takes a while
petalinux-build -p $PROJECT_DIR

# create a boot.bin that includes uboot
petalinux-package --boot --u-boot --format BIN -p $PROJECT_DIR --force

# we want an SD card with a VFAT partition, and that SD card should
# contain the files below to boot linux, lets put them in one place for ease of use
mkdir -p sdcard
cp $PROJECT_DIR/images/linux/BOOT.BIN sdcard
cp $PROJECT_DIR/images/linux/image.ub sdcard
cp $PROJECT_DIR/images/linux/boot.scr sdcard
cp $PROJECT_DIR/images/linux/rootfs.cpio.gz.u-boot sdcard

# create a wic image so we can flash the card image.  
# the sd card needs to be a FAT32 boot partition with 
# BOOT.bin, ... ..., and the rootfs.cpio.gz.u-boot
# this can be done manually, or just use the wic image
# to make the entire card 
#cd $PROJECT_DIR
#petalinux-package --wic -e rootfs.cpio.gz.u-boot
#cd ..

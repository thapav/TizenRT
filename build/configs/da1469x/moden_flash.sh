#!/bin/bash
# ------------------------------------------------------------------
# moden_flash.sh
#
# Author:       Seongjin Oh
# Email:        tome@samsung.com
# Date:         2018-12-13
# Ver.:         1.0.0
#
# Usage:        ./moden_flash.sh
# Description:
#  1) Erase flash
#  2) make image
#  3) flash new image
#
# HISTORY:
# * Dec 13,2018 - 1.0.0  - initial version
# ------------------------------------------------------------------

PORT=
IMAGE_PATH=$PWD/DA1469x-00-Release_QSPI
CODE_IMAGE=Moden_DA1469x.img
TARGET_IMAGE=Moden_DA1469x_Final.img
PRODUCT_ID=DA1469x-00
OFFSET_ADDRESS=0x080000
VERSION_HEADER=$PWD/sw_version.h
SECURE_TYPE=d2522
SPEED=1000000

# --- get usb port info  -------------------------------------------
function get_port() {
    port_list=`ls /dev/ttyUSB*`
    port_count=`echo $port_list | wc -l`

    if [[ "$port_count" == 0 ]]; then
        exit
    fi

    PORT=$port_list
    echo "PORT=$PORT"
}

# --- erase rom  -------------------------------------------
function erase_rom() {
    echo "................................................................"
    echo "... Erasing QSPI..."
    echo "................................................................"
    #cli_programmer $PORT boot uarboot.bin
    #cli_programmer -i 57600 -s 115200 $PORT chip_erase_qspi
    cli_programmer $PORT chip_erase_qspi
    echo "           Done."
}

# --- make image  -------------------------------------------
function make_image() {
    echo "................................................................"
    echo "... Making image..."
    echo "................................................................"


    echo "mkimage $SECURE_TYPE $CODE_IMAGE $VERSION_HEADER $TARGET_IMAGE"
    mkimage $SECURE_TYPE $CODE_IMAGE $VERSION_HEADER $TARGET_IMAGE

    echo "           Done."

}


# --- flash image  -------------------------------------------
function flash_image() {
    echo "................................................................"
    echo "... Flashing image..."
    echo "................................................................"

    echo "cli_programmer --prod-id $PRODUCT_ID -s $SPEED $PORT write_qspi $OFFSET_ADDRESS $TARGET_IMAGE"
    cli_programmer --prod-id $PRODUCT_ID -s $SPEED $PORT write_qspi $OFFSET_ADDRESS $TARGET_IMAGE

    echo "cli_programmer $PORT write_qspi_bytes 0x0 0x50 0x70 0x00 0x00 0x08 0x00 0x00 0x00 0x08 0x00 0xEB 0x00 0x20 0xA8 0x66 0x00 0x00 0x00 0xAA"
    cli_programmer $PORT write_qspi_bytes 0x0 0x50 0x70 0x00 0x00 0x08 0x00 0x00 0x00 0x08 0x00 0xEB 0x00 0x20 0xA8 0x66 0x00 0x00 0x00 0xAA 0x11 0x04 0x00 0x01 0x00 0x02 0x07
    echo "           Done."
}

# --- clean up  -------------------------------------------
function clean_up() {
    rm $TARGET_IMAGE
    echo "................................................................"
    echo "... FINISHED!"
    echo "................................................................"
}

#
#
# get port info
get_port


# erase
erase_rom


# make image
make_image


# flash
flash_image

# clean up
clean_up



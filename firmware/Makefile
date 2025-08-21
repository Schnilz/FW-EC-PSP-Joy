all : flash

TARGET:=main
CH32V003FUN:=${RV003USB_PATH}/ch32v003fun/ch32v003fun
MINICHLINK?=${RV003USB_PATH}/ch32v003fun/minichlink

ADDITIONAL_C_FILES+=${RV003USB_PATH}/rv003usb/rv003usb.S ${RV003USB_PATH}/rv003usb/rv003usb.c
EXTRA_CFLAGS:=-I${RV003USB_PATH}/lib -I${RV003USB_PATH}/rv003usb -O2

LDFLAGS+=-T overrides.ld 

ADDITIONAL_C_FILES+= \
    drivers/icm42670p/icm42670p/imu/inv_imu_apex.c \
    drivers/icm42670p/icm42670p/imu/inv_imu_driver.c \
    drivers/icm42670p/icm42670p/imu/inv_imu_selftest.c \
    drivers/icm42670p/icm42670p/imu/inv_imu_transport.c
EXTRA_CFLAGS+=-Idrivers/icm42670p/icm42670p -Idrivers/icm42670p/examples

ADDITIONAL_C_FILES+= \
    drivers/i2c/lib_i2c.c
EXTRA_CFLAGS+=-Idrivers/i2c


ADDITIONAL_C_FILES+= \
    system_interface.c

include ${RV003USB_PATH}/ch32v003fun/ch32v003fun/ch32v003fun.mk

flash : cv_flash
clean : cv_clean



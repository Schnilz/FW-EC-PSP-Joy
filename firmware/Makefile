all : flash

TARGET:=main
CH32V003FUN:=${RV003USB_PATH}/ch32v003fun/ch32v003fun
MINICHLINK?=${RV003USB_PATH}/ch32v003fun/minichlink

ADDITIONAL_C_FILES+=${RV003USB_PATH}/rv003usb/rv003usb.S ${RV003USB_PATH}/rv003usb/rv003usb.c
EXTRA_CFLAGS:=-I${RV003USB_PATH}/lib -I${RV003USB_PATH}/rv003usb

include ${RV003USB_PATH}/ch32v003fun/ch32v003fun/ch32v003fun.mk

flash : cv_flash
clean : cv_clean



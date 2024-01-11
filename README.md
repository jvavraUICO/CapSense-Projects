Building the Kernel module for Raspberry Pi 4 Model B (August 2020 OS version)

    Setup your environment for cross-compiling the Linux kernel
        (We have tested with Ubuntu 20.04)
        On an Ubuntu system you must install the following packages:
            bc bison flex libssl-dev make libc6-dev libncurses5-dev crossbuild-essential-armhf

            Set the following environment variables:
            KERNEL=kernel7l
            ARCH=arm
            CROSS_COMPILE=arm-linux-gnueabihf-

        Clone a copy of the Raspberry Pi Linux Kernel sources from the rpi-5.4.y branch
            of this repository: https://github.com/raspberrypi/linux

    copy duraTOUCH.h and duraTOUCH.c into drivers/input/touchscreen/
    
    Merge Kconfig into drivers/input/touchscreen/Kconfig

    Add the following line to drivers/input/touchscreen/Makefile:
        obj-$(CONFIG_TOUCHSCREEN_DURATOUCH)		+= duraTOUCH.o

    From the root directory of the kernel sources run*:
        > make bcm2711_defconfig
        > make menuconfig
            Navigate to: Device Drivers > Input Device Support > Touchscreens
            Set "UICO DuraTouch touchscreen" to "M"

        > make scripts prepare modules_prepare
        > make -C . M=drivers/input/touchscreen

    The compiled module will be located at drivers/input/touchscreen/duraTOUCH.ko

Building the Device Tree overlay (duraTOUCHic.dto):

    Place a copy of the overlay file duraTOUCHic.dto on the Raspberry Pi
    From a terminal on the Raspberry Pi, compile the overlay file:
        > dtc -I dts -O dtb duraTOUCHic.dto > duraTOUCHic.dtbo

* The command "make bcm2711_defconfig" sets the default kernel configuration for a Raspberry Pi 4.
For other versions of raspberry Pi, see the RPi kernel build instructions here: 
https://www.raspberrypi.org/documentation/linux/kernel/building.md)

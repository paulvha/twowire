
# twowire (I2C) library for Raspberry Pi

Copyright (c) Paul van Haastrecht <paulvha@hotmail.com>

# version 1.0	September 2018 
initial version 

# version 2.0   October 2018
Added setpullup() call

## Background
The BCM2835 chip provides I2C communication protocol from the hardware. However the clock stretching implementation is bad, leading to many issues with slave devices that depend on or use  clock stretching to handle the requests. (e.g. CCS811, SCD30 ) This library can handle that. 

This library can be instructed to use the standard (hardware) I2C communication OR use a software version that supports clock stretching on any of the GPIO pins. This also supports for you to run multiple I2C channels at the same time on different GPIO.  To have low processor load and increase stability, the maximum speed that is supported for the soft_I2C is 200Khz. However for compatibility reasons a higher speed request (max. 400Khz) will be accepted, which can be executed by hardware I2C communication, but for software I2C the maximum executed will be around 200Khz. 

software version :
  advantage:
	can run multiple I2C channels for different programs in parallel
	flexible on GPIO’s to use
	run as any user (does not need root permission)
	supports proper clock-stretch

  disadvantage:
	generates some process load (although acceptable low)
	quality of communication depending on total processor load
	supports speeds up to max 200Khz

hardware version :
  advantage
	generates lower process load
	less / no dependency on processor load from other programs
	supports speeds up to max 400Khz

  disadvantage
	single channel
	fixed GPIO’s ( 2 and 3)
	must be root or super user.
	No clock-stretch support

For detailed information about findings, the hardware and software, please read the included twowire.odt
 
## Software installation

Make your self superuser : sudo bash

BCM2835 library
Install latest from BCM2835 from : http://www.airspayce.com/mikem/bcm2835/

1. cd /home/pi
2. wget http://www.airspayce.com/mikem/bcm2835/bcm2835-1.56.tar.gz
3. tar -zxf bcm2835-1.56.tar.gz		// 1.52 was version number at the time of writing
4. cd bcm2835-1.56
5. ./configure
6. sudo make check
7. sudo make install

Installation of twowire library software :

1. download the zip-file (clone or download / download zip-file) in the wanted directory
2. unzip twowire-master.zip (*1)
3. cd twowire-master
4. make install

*1) if you do not have unzip : sudo apt-get install zip unzip

“make install” will create and install a dynamic library,. If needed “make install-static” creates a static library,  In the software directory you will find the following files 

twowire.odt :
	detailed description of the library

twowire.cpp :
     This is the source of the library that will be discussed in detail in a next section. 

twowire.h :
     This is the header file for the twowire library. 

makefile :
	script to compile the source and create an executable with the  make command.

 
## Software usage

To access and compile a user program correctly, add on top in the user program:
	# include <twowire.h>

Add when compiling to include both libraries : -ltwowire -lbcm2835

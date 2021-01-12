#!/usr/bin/env bash

EXEDIR=`pwd`
BASEDIR=$(dirname $0)

SYSTYPE=`uname -s`

#
# Serial port defaults.
#
# XXX The uploader should be smarter than this.
#
if [ $SYSTYPE = "Darwin" ]; then
SERIAL_PORTS="/dev/tty.usbmodemPX*,/dev/tty.usbmodem*"
fi

if [ $SYSTYPE = "Linux" ]; then
SERIAL_PORTS="/dev/serial/by-id/*_PX4_*,/dev/serial/by-id/usb-3D_Robotics*,/dev/serial/by-id/usb-The_Autopilot*,/dev/serial/by-id/usb-Bitcraze*,/dev/serial/by-id/pci-Bitcraze*,/dev/serial/by-id/usb-Gumstix*,/dev/serial/by-id/usb-UVify*,/dev/serial/by-id/usb-ArduPilot*,"
fi

if [[ $SYSTYPE = *"CYGWIN"* ]]; then
SERIAL_PORTS="/dev/ttyS*"
fi

if [ $SYSTYPE = "" ]; then
SERIAL_PORTS="COM32,COM31,COM30,COM29,COM28,COM27,COM26,COM25,COM24,COM23,COM22,COM21,COM20,COM19,COM18,COM17,COM16,COM15,COM14,COM13,COM12,COM11,COM10,COM9,COM8,COM7,COM6,COM5,COM4,COM3,COM2,COM1,COM0"
fi

python $BASEDIR/px_uploader.py --port $SERIAL_PORTS $1

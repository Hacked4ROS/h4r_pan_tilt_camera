#!/bin/bash

port=$1
hexfile=`catkin_find h4r_pantilt_mcu_avr pan_tilt_avr_arduino_uno.hex --first`
avrdude -p atmega328p -c arduino  -P $1 -Uflash:w:$hexfile:a
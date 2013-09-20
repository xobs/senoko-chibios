#!/bin/bash

if [ -z ${JTAG} ]
then
	JTAG=jlink
fi

# Bring board out of "program" mode
echo 148 > /sys/class/gpio/export 2> /dev/null
echo out > /sys/class/gpio/gpio148/direction 2> /dev/null
echo 0 > /sys/class/gpio/gpio148/value 2> /dev/null

# Bring the board out of reset
echo 149 > /sys/class/gpio/export 2> /dev/null
echo out > /sys/class/gpio/gpio149/direction 2> /dev/null
echo 1 > /sys/class/gpio/gpio149/value 2> /dev/null

openocd -f interface/${JTAG}.cfg -f ../../boards/NOVENA_STM32F1_PMB/openocd.cfg \
            -c init -c targets -c halt \
            -c "flash write_image erase build/pmb.elf" \
            -c "reset run"

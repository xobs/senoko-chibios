#!/bin/bash

# Put board into "reprogram" mode
sudo /bin/sh -c 'echo 148 > /sys/class/gpio/export 2> /dev/null'
sudo /bin/sh -c 'echo out > /sys/class/gpio/gpio148/direction 2> /dev/null'
sudo /bin/sh -c 'echo 1 > /sys/class/gpio/gpio148/value 2> /dev/null'

# Reset board
sudo /bin/sh -c 'echo 149 > /sys/class/gpio/export 2> /dev/null'
sudo /bin/sh -c 'echo out > /sys/class/gpio/gpio149/direction 2> /dev/null'
sudo /bin/sh -c 'echo 0 > /sys/class/gpio/gpio149/value 2> /dev/null'
sleep 1
sudo /bin/sh -c 'echo 1 > /sys/class/gpio/gpio149/value 2> /dev/null'

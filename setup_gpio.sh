#!/bin/bash

#PH3 - control left
echo 1 > /sys/class/pwm/pwmchip0/export
echo 40000 > /sys/class/pwm/pwmchip0/pwm1/period
echo 40000 > /sys/class/pwm/pwmchip0/pwm1/duty_cycle
echo 1 > /sys/class/pwm/pwmchip0/pwm1/enable

#PH2 - control right
echo 2 > /sys/class/pwm/pwmchip0/export
echo 40000 > /sys/class/pwm/pwmchip0/pwm2/period
echo 40000 > /sys/class/pwm/pwmchip0/pwm2/duty_cycle
echo 1 > /sys/class/pwm/pwmchip0/pwm2/enable

#PC8 - encoder left
echo 72 > /sys/class/gpio/export
echo in > /sys/class/gpio/gpio72/direction
echo falling > /sys/class/gpio/gpio72/edge
gpio mode 8 up

#PC10 - encoder right
echo 74 > /sys/class/gpio/export
echo in > /sys/class/gpio/gpio74/direction
echo falling > /sys/class/gpio/gpio74/edge
gpio mode 16 up

#PC5 - direction left
echo 69 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio69/direction
echo 1 > /sys/class/gpio/gpio69/value

#PC6 - direction right
echo 70 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio70/direction
echo 1 > /sys/class/gpio/gpio70/value

#PH9 - brake all
echo 233 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio233/direction
echo 0 > /sys/class/gpio/gpio233/value

#UART for LIDAR
chmod 0666 /dev/ttyS2
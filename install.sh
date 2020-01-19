#!/bin/bash

sudo rmmod /home/pi/astro-workspace/RPi_precise_control/ptime_control.ko;
sudo insmod /home/pi/astro-workspace/RPi_precise_control/ptime_control.ko

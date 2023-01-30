#!/bin/bash

set -xe

dmesg -c

modprobe -a videodev videobuf2-common videobuf2-v4l2 videobuf2-vmalloc
rmmod af9035 || :
insmod af9035.ko
v4l2-ctl --all || :
cat /dev/video0
dmesg

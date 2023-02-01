#!/bin/bash

set -xe

rmmod af9035 || :

dmesg -c

modprobe -a videodev videobuf2-common videobuf2-v4l2 videobuf2-vmalloc
insmod af9035.ko dyndbg=+p
v4l2-ctl --all || :
#cat /dev/video0
dmesg

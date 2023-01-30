#!/bin/bash

set -xe

modprobe -a videodev videobuf2-common videobuf2-v4l2 videobuf2-vmalloc
rmmod af9035 || :
insmod af9035.ko
dmesg | tail -20

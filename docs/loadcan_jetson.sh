#!/bin/bash

busybox devmem 0x0c303018 w 0x458
busybox devmem 0x0c303010 w 0x400

modprobe can
modprobe can_raw
modprobe mttcan

ip link set can0 up type can bitrate 1000000
ifconfig can0 txqueuelen 1000

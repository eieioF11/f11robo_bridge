#!/bin/bash
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout",  SYMLINK+="lidar"' >/etc/udev/rules.d/lidar.rules
echo  'SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6015", SYMLINK+="F11Robo", GROUP="dialout"' >/etc/udev/rules.d/F11Robo.rules

service udev reload
sleep 2
service udev restart

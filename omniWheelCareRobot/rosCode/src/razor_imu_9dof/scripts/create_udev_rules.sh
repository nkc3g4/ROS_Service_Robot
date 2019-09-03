#!/bin/bash

echo "remap the device serial port(ttyUSBX) to  IMU"
echo "IMU usb connection as /dev/IMU , check it using the command : ls -l /dev|grep ttyUSB"
echo "start copy IMU.rules to  /etc/udev/rules.d/"
sudo cp ./IMU.rules  /etc/udev/rules.d
echo " "
echo "Restarting udev"
echo ""
sudo udevadm control --reload-rules
sudo service udev restart
sudo udevadm trigger
echo "rename IMU device mount point finish "


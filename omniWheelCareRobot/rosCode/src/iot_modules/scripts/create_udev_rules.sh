#!/bin/bash

echo "remap the device serial port(ttyUSBX) to IOTnet"
echo "IMU usb connection as /dev/IOTnet , check it using the command : ls -l /dev|grep ttyUSB"
echo "start copy IOT_net.rules to  /etc/udev/rules.d/"
sudo cp ./IOT_net.rules  /etc/udev/rules.d
echo " "
echo "Restarting udev"
echo ""
sudo udevadm control --reload-rules
sudo service udev restart
sudo udevadm trigger
echo "rename IOTnet device mount point finish "


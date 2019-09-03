#!/bin/bash

echo "remap the device serial port(ttyUSBX) to  laser"
echo "ls01d usb cp210x connection as /dev/laser , check it using the command : ls -l /dev|grep ttyUSB"
echo "start copy laser.rules to  /etc/udev/rules.d/"
sudo cp ./laser.rules  /etc/udev/rules.d
echo " "
echo "Restarting udev"
echo ""
sudo udevadm control --reload-rules
sudo service udev restart
sudo udevadm trigger
echo "finish "

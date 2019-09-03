#!/bin/bash

echo "delete remap the device serial port(ttyUSBX) to IOTnet"
echo "sudo rm /etc/udev/rules.d/IOT_net.rules"
sudo rm /etc/udev/rules.d/IMU.rules
echo " "
echo "Restarting udev"
echo ""
sudo udevadm control --reload-rules
sudo service udev restart
sudo udevadm trigger
echo "finish  delete"

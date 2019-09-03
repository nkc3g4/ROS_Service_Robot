#!/bin/sh

echo "checking the main thread 'roscode' ......"
ps -fe|grep "roscore"|grep -v grep

if [ $? -ne 0 ]
then
    echo "main thread does not exist, now starting main thread ......"
    roscore & 
    sleep 5
fi
gnome-terminal -x mv /home/sz/scripts/mymap.* /hom/sz/omniWheelCareRobot/rosCode/src/carebot_navigation/maps & 
sleep 2
gnome-terminal -x roslaunch carebot_navigation amcl_ls01d_lidar.launch &
# view navigation
sleep 3
gnome-terminal -x roslaunch carebot_navigation view_navigation.launch &
sleep 5
gnome-terminal -x roslaunch iot_modules speak_IOT.launch &
sleep 1
gnome-terminal -x rosrun teleop_twist_keyboard teleop_twist_keyboard.py &
sleep 3
gnome-terminal -x roslaunch voice_bringup voice_bringup.launch & 


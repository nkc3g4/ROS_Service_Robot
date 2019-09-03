#!/bin/sh
echo "checking the main thread 'roscode' ......"
ps -fe|grep "roscore"|grep -v grep

if [ $? -ne 0 ]
then
    echo "main thread does not exist, now starting main thread ......"
    roscore &
    sleep 5
fi

gnome-terminal -x roslaunch carebot_bringup carebot_bringup.launch &
sleep 5
gnome-terminal -x roslaunch carebot_navigation gmapping_ls01d_lidar.launch &
sleep 5
gnome-terminal -x roslaunch carebot_navigation view_navigation.launch &
sleep 5
gnome-terminal -x rosrun teleop_twist_keyboard teleop_twist_keyboard.py


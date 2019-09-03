#!/bin/bash

source /opt/ros/kinetic/setup.sh
source ~/omniWheelCareRobot/rosCode/devel/setup.bash
#config ros_voice_system env
source /home/sz/ros_voice_system/devel/setup.bash
source /omniWheelCareRobot/rosCode/src/dobot_ws/devel/setup.bash


echo "checking the main thread 'roscode' ......"
ps -fe|grep "roscore"|grep -v grep

if [ $? -ne 0 ]
then
    echo "main thread does not exist, now starting main thread ......"
    gnome-terminal -x roscore
    sleep 5
fi


gnome-terminal -x rosrun dobot DobotServer ttyACM1 &
sleep 2
gnome-terminal -x rosrun dobot DobotClient_JOG

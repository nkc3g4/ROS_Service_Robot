#!/bin/sh

echo "checking the main thread 'roscode' ......"
ps -fe|grep "roscore"|grep -v grep

if [ $? -ne 0 ]
then
    echo "main thread does not exist, now starting main thread ......"
    roscore & 
    sleep 5
fi

    gnome-terminal -x roslaunch voice_bringup voice_bringup.launch &
sleep 3
gnome-terminal -x roslaunch iot_modules speak_IOT.launch

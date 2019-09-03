#!/bin/sh

echo "checking the main thread 'roscode' ......"
ps -fe|grep "roscore"|grep -v grep

if [ $? -ne 0 ]
then
    echo "main thread does not exist, now starting main thread ......"
    roscore &
    sleep 3
fi

# bring up the camera
gnome-terminal -x roslaunch astra_camera astra.launch &

# launch the door detection module (wait for 1 second until the camera is ready)
sleep 5
gnome-terminal -x roslaunch door_detection door_detection.launch &



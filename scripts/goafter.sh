#!/bin/sh
 

gnome-terminal -x roslaunch carebot_bringup carebot_bringup.launch &
sleep 5
gnome-terminal -x roslaunch carebot_follower follower.launch


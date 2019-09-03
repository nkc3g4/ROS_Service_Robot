#!/bin/sh
ps -fe|grep "roscore"|grep -v grep

if [ $? -ne 0 ]
then
    echo "main thread does not exist, now starting main thread ......"
    roscore &
    sleep 2
fi

# check if the map is on the correct place
gnome-terminal -x mv /home/sz/scripts/mymap.* /hom/sz/omniWheelCareRobot/rosCode/src/carebot_navigation/maps &

## voice bringup system
#sleep 5
#gnome-terminal -x roslaunch voice_bringup voice_bringup.launch &

# carebot bringup
sleep 2
gnome-terminal -x roslaunch carebot_bringup carebot_bringup.launch &
# amcl navigation system
sleep 5
gnome-terminal -x roslaunch carebot_navigation amcl_ls01d_lidar.launch &
# view navigation
sleep 5
gnome-terminal -x roslaunch carebot_navigation view_navigation.launch &
sleep 2
gnome-terminal -x roslaunch carebot_velocity_smoother velocity_smoother.launch &

# dobot integration
sleep 5
gnome-terminal -x roslaunch dobot_integration dobot_integration.launch &

# face_rec
sleep 2
gnome-terminal -x ./face_rec.sh

# voic bringup system
sleep 5
gnome-terminal -x roslaunch voice_bringup voice_bringup.launch &

# multi-point navigation
sleep 2
gnome-terminal -x rosrun carebot_navigation position_nav3.py



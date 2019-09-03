#!/bin/sh
echo "checking the main thread 'roscode' ......"
ps -fe|grep "roscore"|grep -v grep

if [ $? -ne 0 ]
then
    echo "main thread does not exist, now starting main thread ......"
    roscore &
    sleep 5
fi

# carebot bringup

#gnome-terminal -x roslaunch carebot_bringup carebot_bringup.launch &
# check if the map is on the correct place
#sleep 5
gnome-terminal -x mv /home/sz/scripts/mymap.* /home/sz/omniWheelCareRobot/rosCode/src/carebot_navigation/maps &
# amcl navigation system
sleep 5
gnome-terminal -x roslaunch carebot_navigation amcl_ls01d_lidar.launch &
# velocity smoother
#sleep 5
#gnome-terminal -x roslaunch carebot_velocity_smoother velocity_smoother.launch &


# view navigation
sleep 5
gnome-terminal -x roslaunch carebot_navigation view_navigation.launch &
# dobot integration
sleep 5
gnome-terminal -x rosrun teleop_twist_keyboard teleop_twist_keyboard.py
#gnome-terminal -x roslaunch dobot_integration dobot_integration.launch

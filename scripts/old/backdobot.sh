#!/bin/bash
source /opt/ros/kinetic/setup.sh
source ~/omniWheelCareRobot/rosCode/devel/setup.bash
#config ros_voice_system env
source /home/sz/ros_voice_system/devel/setup.bash
source /omniWheelCareRobot/rosCode/src/dobot_ws/devel/setup.bash




gnome-terminal -x roslaunch dobot_integration dobot_integration.launch &
sleep 2
gnome-terminal -x rostopic pub /dobot_cmd std_msgs/Int32 99

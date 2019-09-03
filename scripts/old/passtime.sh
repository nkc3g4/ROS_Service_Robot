#!/bin/bash
#config ros system env
source /opt/ros/kinetic/setup.bash
source ~/omniWheelCareRobot/rosCode/devel/setup.bash
#config ros_voice_system env
source /home/sz/ros_voice_system/devel/setup.bash

PATH=/home/sz/bin:/home/sz/.local/bin:/opt/ros/kinetic/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin
#HOME=/
export PATH

time=`date +%R`
rostopic pub -1 /voice_system/tts_topic std_msgs/String "现在时间是$time" 
rostopic pub -1 /voice_system/tts_topic std_msgs/String "您该吃药了" 
#sleep 3
#play '/home/sz/您该吃药了.mp3'
echo $time

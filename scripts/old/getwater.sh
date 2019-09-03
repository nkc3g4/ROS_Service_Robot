roslaunch carebot_bringup carebot_bringup.launch & 
sleep 3
gnome-terminal -x roslaunch dobot_integration dobot_integration.launch &

sleep 2
gnome-terminal -x rostopic pub /dobot_cmd std_msgs/Int32 "data: 1024"

# gnome-terminal -x rostopic pub /dobot_cmd std_msgs/Int32 "data: 10241"

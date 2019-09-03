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

# launch the face recognization module (wait for 1 second until the camera is ready)
sleep 5
gnome-terminal -x roslaunch face_recognition face_recognition.launch &



sleep 5
gnome-terminal -x roslaunch voice_bringup voice_bringup.launch &

sleep 5
gnome-terminal -x rostopic pub /voice_system/tts_topic std_msgs/String "人脸识别模块启动成功"

# show the image window
gnome-terminal -x rqt_image_view /face_recognition/labelled_faces

#!/bin/bash

dirPath="/home/$USER/.carebot/"

#create carebot config folder to save log
if [ -d "$dirPath" ]
then
  echo "Already exist target folder,now delete old create new foler..."
  rm -rf "$dirPath"
  mkdir "$dirPath" 
else
  mkdir "$dirPath" 
fi

sudo cp carebot_start /usr/sbin/
sudo chmod +x /usr/sbin/carebot_start

sudo cp carebot_stop /usr/sbin/
sudo chmod +x /usr/sbin/carebot_stop

sudo cp carebot_restart /usr/sbin/
sudo chmod +x /usr/sbin/carebot_restart

sudo cp carebot.service /lib/systemd/system/

sudo systemctl enable carebot.service


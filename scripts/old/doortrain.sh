#!/bin/sh
imglab -c /home/sz/objDetection/door_train/door.xml /home/sz/objDetection/door_train/
imglab /home/sz/objDetection/door_train/door.xml	#标记

python /home/sz/objDetection/trainDoor.py
python /home/sz/objDetection/testDoor.py

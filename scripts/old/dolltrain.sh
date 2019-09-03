#!/bin/sh
imglab -c /home/sz/objDetection/doll_train/doll.xml /home/sz/objDetection/doll_train/
imglab /home/sz/objDetection/doll_train/doll.xml	#标记

python /home/sz/objDetection/trainDoll.py
python /home/sz/objDetection/testDoll.py

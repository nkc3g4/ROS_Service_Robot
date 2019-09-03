#!/usr/bin/env python3

import rospy
from door_detection import DoorDetection

rospy.init_node('door_detection_node')

def main():
    fr = DoorDetection()
    rospy.spin()


if __name__=='__main__':
    main()

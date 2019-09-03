#!/usr/bin/env python2
# _*_ coding:utf-8 _*_

"""
    Copyright(c):2016-2019 ROS小课堂
    Author: www.corvin.cn
    Description:
        校准小车底盘的源码文件.
    History:
        20180425: init this file.
"""

from geometry_msgs.msg import Twist, Quaternion
from math import radians, copysign
from nav_msgs.msg import Odometry
from math import pi
import PyKDL
import rospy
import tf


def quat_to_angle(quat):
    rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
    return rot.GetRPY()[2]

def normalize_angle(angle):
    res = angle
    while res > pi:
	res -= 2.0 * pi
    while res < -pi:
        res += 2.0 * pi
    return res

class CalibrateAngular():
    def __init__(self):
        rospy.init_node('calibrate_angular_node', anonymous=False)

        # Set rospy to execute a shutdown function when terminating the script
        rospy.on_shutdown(self.shutdown)

        try:
            self.rate = rospy.get_param("~check_odom_rate", 12)
            self.circle_cnt = rospy.get_param("~test_circle", '2')
            self.test_angle = eval('self.circle_cnt*2*pi')

            self.speed = rospy.get_param("~angular_speed", 0.3)   #radians speed
            self.tolerance = rospy.get_param("~tolerance_angle", 0.02)
            self.odom_angular_scale = rospy.get_param("~angular_scale", 1.000)

            # Publisher to control the robot's speed
            cmd_topic    = rospy.get_param("~cmd_topic", '/cmd_vel')
            self.cmd_vel = rospy.Publisher(cmd_topic, Twist, queue_size=5)

            # The base frame is usually base_link or base_footprint
            self.base_frame = rospy.get_param('~base_frame', '/base_footprint')

            # The odom frame is usually just /odom
            self.odom_frame = rospy.get_param('~odom_frame', '/odom')
        except:
            rospy.logerr("ERROR: Get config param error from yaml file...")

        # How fast will we check the odometry values?
        r = rospy.Rate(self.rate)

        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()

        # Give tf some time to fill its buffer
        rospy.sleep(2)

        # Make sure we see the odom and base frames
        self.tf_listener.waitForTransform(self.odom_frame, self.base_frame, rospy.Time(), rospy.Duration(30.0))

        start_time = rospy.get_time()  #get start test time
        while not rospy.is_shutdown():
            # Get the current rotation angle from tf
            self.odom_angle = 0 
            last_angle  = 0 
            turn_angle = 0.0
            rest_angle = self.test_angle - turn_angle
            # config move cmd
            move_cmd = Twist()
            move_cmd.angular.z = self.speed

            while rest_angle > self.tolerance:
                if rospy.is_shutdown():
                    return

                rospy.loginfo("***************************************")
                # Rotate the robot to reduce the rest_angle
                self.cmd_vel.publish(move_cmd)
                r.sleep()

                # Get the current rotation angle from tf                   
                self.odom_angle = self.get_odom_angle()
                rospy.loginfo("current rotation angle: " + str(self.odom_angle))

                # Compute how far we have gone since the last measurement
                delta_angle = self.odom_angular_scale * normalize_angle(self.odom_angle - last_angle)

                # Add to our total angle so far
                turn_angle += delta_angle
                rospy.loginfo("turn_angle: " + str(turn_angle))

                # Compute the new rest angle 
                rest_angle = self.test_angle - abs(turn_angle)
                rospy.loginfo("rest_angle: " + str(rest_angle))

                # Store the current angle for the next comparison
                last_angle = self.odom_angle

            # test completed 
            end_time = rospy.get_time() #get test end time
            rospy.logwarn("----------------------------")
            rospy.logwarn("---Angular Test Completed---")
            rospy.logwarn("----------------------------")
            rospy.logwarn("Test angle:" + str(self.test_angle))
            rospy.logwarn("Test Time:"  + str(end_time-start_time))
            rospy.logwarn("Test Angular speed:"  + str(self.test_angle/(end_time-start_time)))
            rospy.logwarn("Input Angular speed:" + str(self.speed))
            rospy.logwarn("-----------END--------------")
            rospy.signal_shutdown('End Test')

    # Get the current transform between the odom and base frames
    def get_odom_angle(self):
        try:
            (trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.logerr("TF Exception")
            return

        # Convert the rotation from a quaternion to an Euler angle
        return quat_to_angle(Quaternion(*rot))

    def shutdown(self):
        # Always stop the robot when shutting down the node
        rospy.logwarn("shutdown():Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        CalibrateAngular()
    except:
        rospy.logerr("Error: Calibration terminated.")



#!/usr/bin/env python2

import rospy
from geometry_msgs.msg import Twist, Point
from math import copysign, sqrt, pow
import tf

class CalibrateLinear():
    def __init__(self):
        #give the node a name
        rospy.init_node('calibrate_linear_node', anonymous=False)

        #set rospy to execute a shutdown function when terminating the script
        rospy.on_shutdown(self.shutdown)

        #How fast will we check the odometry values?
        self.rate = 12
        r = rospy.Rate(self.rate)

        #set the distance to travel
        self.test_distance = rospy.get_param("~test_distance", 2.0) 
        self.speed = rospy.get_param("~linear_speed", 0.2) 
        self.tolerance = rospy.get_param("~tolerance_linear", 0.01) 
        self.odom_linear_scale_correction = rospy.get_param("~linear_scale", 1.000)
        self.start_test = True

        #Publisher to control the robot's speed
        cmd_topic = rospy.get_param("~cmd_topic", '/cmd_vel')
        self.cmd_vel = rospy.Publisher(cmd_topic, Twist, queue_size=5)

        #The base frame is base_footprint for the robot
        self.base_frame = rospy.get_param('~base_frame', '/base_footprint')

        #The odom frame is usually just /odom
        self.odom_frame = rospy.get_param('~odom_frame', '/odom')

        #initialize the tf listener
        self.tf_listener = tf.TransformListener()

        #give tf some time to fill its buffer
        rospy.sleep(2)

        #make sure we see the odom and base frames
        self.tf_listener.waitForTransform(self.odom_frame, self.base_frame, rospy.Time(), rospy.Duration(60.0))
        self.position = Point()

        #get the starting position from the tf transform between the odom and base frames
        self.position = self.get_position()

        x_start = self.position.x
        y_start = self.position.y
        move_cmd = Twist()

        rospy.loginfo("~~~~~~Now Start Linear Test~~~~~~")
        rospy.loginfo("-> test_distance: " + str(self.test_distance))
        rospy.loginfo("-> linear_speed: " + str(self.speed))
        rospy.loginfo("-> cmd_topic: " + cmd_topic)
        rospy.loginfo("-> distance_tolerance: " + str(self.tolerance))
        rospy.loginfo("-> linear_scale_correction: " + str(self.odom_linear_scale_correction))

        while not rospy.is_shutdown():
            #Stop the robot by default
            move_cmd = Twist()

            if self.start_test:
                rospy.loginfo("**********************************")
                #get the current position from the tf transform between the odom and base frames
                self.position = self.get_position()

                #compute the euclidean distance from the target point
                distance = sqrt(pow((self.position.x - x_start), 2) +
                                pow((self.position.y - y_start), 2))

                #correct the estimate distance by the correction factor
                distance *= self.odom_linear_scale_correction
                #How close are we?
                error = self.test_distance - distance
                rospy.loginfo("rest_distance: " + str(error))

                #are we close enough?
                if not self.start_test or abs(error) < self.tolerance:
                    self.start_test = False
                    rospy.logwarn("Now set start_test flag: " + str(self.start_test))
                    self.cmd_vel.publish(Twist())   #stop the robot
                    rospy.logwarn("Now stop the robot")
                else:
                    #if not, move in the appropriate direction
                    move_cmd.linear.x = copysign(self.speed, error)
            else:
                self.position = self.get_position()
                x_start = self.position.x
                y_start = self.position.y

            self.cmd_vel.publish(move_cmd)
            r.sleep()

    def get_position(self):
        #get the current transform between the odom and base frames
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF exception")
            return

        return Point(*trans)

    def shutdown(self):
        #Always stop the robot when shutting down the node
        rospy.logwarn("shutdown test node,stopping the robot")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        CalibrateLinear()
        rospy.spin()
    except:
        rospy.logerr("Calibration terminated.")


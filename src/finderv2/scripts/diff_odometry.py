#!/usr/bin/env python
# -*- coding: utf8 -*-

"""
Some code borrowed from diff_tf.py by Jon Stephan
Copyright (C) 2013 Jakob Culebro Reyes
You know the GNU stuff, attribution, no warranty, etc.
"""

import rospy
from finder.msg import TwoWheelInt16
from std_msgs.msg import Bool
from std_msgs.msg import UInt16
from std_msgs.msg import Float32

from math import sin, cos, pi

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster

class Finder_diff_odom:
    
    def __init__(self, node_name_override = "finder_diff_odometry"):
		
        rospy.init_node(node_name_override)
        self.nodename = rospy.get_name()
        rospy.loginfo("finder_diff_odometry starting with name %s", self.nodename) 
        
        # parameters, last two are required by the ROS odometry transform, wich may
        # come in handy in the future, default of 20 Hz / 50 ms
        self.rate = rospy.get_param('finder_diff_odometry_rate', 20.0)
        self.t_delta = rospy.Duration(1. / self.rate)
        self.wheel_radii = rospy.get_param('wheel_radii', 0.254)
        self.base_width = rospy.get_param('base_width', 0.508)
        self.angleK = (2 * 3.14159265) / (1023.)
        
        self.base_frame_id = rospy.get_param('~base_frame_id','base_link') # the name of the base frame of the robot
        self.odom_frame_id = rospy.get_param('~odom_frame_id', 'odom')     # the name of the odometry reference frame 
        
        # internal variables, r is the LINEAR advance (like the radii of the x, y displacement)
        self.x = 0
        self.y = 0
        self.th = 0
        self.dr = 0
        self.dth = 0
        
        # suscribers and publishers, refer to the uC node spec
        self.WsSub = rospy.Subscriber("finder_lecture", TwoWheelInt16, self.wsCallback)
        
        # Sync parameters in micro with self, Ts = timeSample
        self.TsPub = rospy.Publisher("ts", UInt16)
        
        # self output
        self.odomPub = rospy.Publisher("odom", Odometry)
        self.odomBroadcaster = TransformBroadcaster()        
        
    def initNodes(self):
		
        # sync the parameters in the uC
        self.TsPub.publish(self.t_delta)  

    def update(self):
		
        now = rospy.Time.now()    
                            
        # publish the odom information
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = sin(self.th / 2.)
        quaternion.w = cos(self.th / 2.)
        self.odomBroadcaster.sendTransform(
            (self.x, self.y, 0),
            (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
            rospy.Time.now(),
            self.base_frame_id,
            self.odom_frame_id
            )
        
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0
        odom.pose.pose.orientation = quaternion
        odom.child_frame_id = self.base_frame_id
        odom.twist.twist.linear.x = self.dr
        odom.twist.twist.linear.y = 0
        odom.twist.twist.angular.z = self.dth
        self.odomPub.publish(odom)

    def spin(self):
		
        self.initNodes()
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()

    def wsCallback(self, data):    
    
        # should be cleaned, also, I'm not dealing with the special case for when
        # the encoder output is published witout any processing
        
        self.p_right = self.angleK * data.wheel1
        self.p_left = self.angleK * data.wheel2
        
        self.w_right = self.p_right / self.t_delta
        self.w_left = self.p_right / self.t_delta
        
        self.dr = self.wheel_radii * (self.w_right + self.w_left) / 2.
        self.dth = self.wheel_radii * (self.w_right - self.w_left) / self.base_width
        
        # I'm ussing the diff_ty code, I wonder if this is going to work. We have
        # tested other algorithms before anyway
        self.cth = self.t_delta * self.dth
        self.cx = self.t_delta * self.dr * cos(self.th + 0.5 * self.dth * self.t_delta)
        self.cy = self.t_delta * self.dr * sin(self.th + 0.5 * self.dth * self.t_delta)
        
        self.x = self.x + (cos(self.th) * self.cx - sin(self.th) * self.cy)
        self.y = self.y + (sin(self.th) * self.cx + cos(self.th) * self.cy)
        self.th = self.th + self.cth
        
        """
        self.dr = self.wheel_radii * 0.5 * (self.w_right + self.w_left)
        self.dth = self.wheel_radii * (self.w_right - self.w_left) / self.base_width
        self.cth = self.t_delta * self.dth
        self.cx = self.t_delta * self.dr * cos(self.th + 0.5 * self.dth * self.t_delta)
        self.cy = self.t_delta * self.dr * sin(self.th + 0.5 * self.dth * self.t_delta)
        self.x = self.x + self.cx
        self.y = self.y + self.cy
        self,th = self.th + self.cth
        """

if __name__ == '__main__':
    """ main """
    finder_diff_odom = Finder_diff_odom()
    finder_diff_odom.spin()
    
    
    

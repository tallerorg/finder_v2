#!/usr/bin/env python
# -*- coding: utf8 -*-

import rospy
from std_msgs.msg import Int16
from finder.msg import FourArmsInt16

class Interface_litarms:
    
    def __init__(self, node_name_override = 'interface_litarms'):
        rospy.init_node(node_name_override)
        self.nodename = rospy.get_name()
        rospy.loginfo("interface_litarms starting with name %s", self.nodename)
        self.rate = rospy.get_param("interface_litarms_rate", 10)
        
        self.armsData = FourArmsInt16()
        self.armsData.arm1 = 0
        self.armsData.arm2 = 0
        self.armsData.arm3 = 0
        self.armsData.arm4 = 0
        self.armsPub  = rospy.Publisher("traction_arms", FourArmsInt16)
        
        self.mta1 = rospy.Subscriber("motor_traction_arm_fr", Int16, self.cbmtafr)
        self.mta2 = rospy.Subscriber("motor_traction_arm_fl", Int16, self.cbmtafl)
        self.mta3 = rospy.Subscriber("motor_traction_arm_br", Int16, self.cbmtabr)
        self.mta4 = rospy.Subscriber("motor_traction_arm_bl", Int16, self.cbmtabl)
    
    def cbmtafr(self, data):
        self.armsData.arm1 = data.data
    def cbmtafl(self, data):
        self.armsData.arm2 = data.data
    def cbmtabr(self, data):
        self.armsData.arm3 = data.data
    def cbmtabl(self, data):
        self.armsData.arm4 = data.data
        
    def update(self):
        self.armsPub.publish(self.armsData)

    def spin(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()

if __name__ == '__main__':
    """ main """
    interface_litarms = Interface_litarms()
    interface_litarms.spin()
    


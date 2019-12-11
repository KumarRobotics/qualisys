#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
from /qualisys/DroneyDrone import *

def rviz_operator():
    pub = rospy.Publisher('Qualisys_rviz', pose)
    rospy.Subscriber("/qualisys/DroneyDrone", position)
    rospy.init_node('rviz_operator', anonymous=True)
    r = rospy.Rate(10)
    
    while not rospy.is_shutdown():

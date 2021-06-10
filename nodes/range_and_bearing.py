#!/usr/bin/env python

'''
A node to calculate range and bearing from map origin to BEN

Val Schmidt
Center for Coastal and Ocean Mapping
University of New Hampshire
Copyright 2021
License: BSD-Clause 2

FIX: Today we are stationary and the MAP frame is static,
and BEN starts adjacent to the Control Van, so this code
reliably reports range and bearing to BEN. But when the 
operator station is not static, this code will report range
and bearing to BEN's initialization location, which is less
helpful. 
'''

import rospy
import numpy as np
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry

rospy.init_node('range')


P = rospy.Publisher('/operator/range',Float32,queue_size=10)
Pb = rospy.Publisher('/operator/bearing',Float32,queue_size=10)

def odom_callback(data):

    x = data.pose.pose.position.x
    y = data.pose.pose.position.y

    R = np.sqrt(x**2 + y**2)
    theta = np.mod(np.arctan2(x,y) * 180.0/np.pi + 360, 360)
    P.publish(R)
    Pb.publish(theta)

S = rospy.Subscriber('/ben/odom',Odometry,odom_callback,queue_size=10)

while not rospy.is_shutdown():
    rospy.spin()


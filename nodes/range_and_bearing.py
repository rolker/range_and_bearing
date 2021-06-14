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
from sensor_msgs.msg import NavSatFix
from mru_transform.srv import LatLongToMap, MapToLatLong
from geographic_msgs.msg import GeoPose, GeoPointStamped, GeoPath, GeoPoint
from tf2_geometry_msgs import do_transform_pose, PoseStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose

import project11
import tf2_ros
rospy.init_node('range')

tfBuffer = tf2_ros.Buffer()
tfListener = tf2_ros.TransformListener(tfBuffer)

P = rospy.Publisher('/operator/range',Float32,queue_size=10)
Pb = rospy.Publisher('/operator/bearing',Float32,queue_size=10)

xo=None
yo=None

def odom_callback(data):
    global xo
    global yo

    if xo is None or yo is None:
        #rospy.loginfo('No Base Position for Range/Bearing Calculation')
        return

    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    dx = x-xo
    dy = y-yo

    R = np.sqrt(dx**2 + dy**2)
    theta = np.mod(np.arctan2(dx,dy) * 180.0/np.pi + 360, 360)
    P.publish(R)
    Pb.publish(theta)

def basepos_callback(data):
    global xo
    global yo
    global tfBuffer

    ex,ey,ez = project11.wgs84.toECEFfromDegrees(data.latitude,
            data.longitude)
    Pbase = PoseStamped()
    Pbase.pose.position.x = ex
    Pbase.pose.position.y = ey
    Pbase.pose.position.z = ez

    try:
        ecef_to_map = tfBuffer.lookup_transform("ben/map",'earth',data.header.stamp)
    except Exception as e:
        #print(e)
        return

    base_map = do_transform_pose(Pbase,ecef_to_map)

    xo = base_map.pose.position.x
    yo = base_map.pose.position.y

S = rospy.Subscriber('/ben/odom',Odometry,odom_callback,queue_size=10)
Sbase = rospy.Subscriber('base/position', NavSatFix, basepos_callback,queue_size=10)

while not rospy.is_shutdown():
    rospy.spin()

#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped

def PosiGet(msg):
    global vehicle_posi
    vehicle_posi = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]) + vehicle_initposi

rospy.init_node("fast_planner_control_node", anonymous=True)
rospy.Rate(30)

vehicle_id = '0'
vehicle_type = 'iris'

rospy.Subscriber('/'+vehicle_type+'_'+vehicle_id+"/mavros/local_position/pose", PoseStamped, PosiGet, queue_size=10)


vehicle_initposi = np.array([-10, 0, 0]) 
vehicle_posi = vehicle_initposi

target = np.array([0,0])
cruise_height = 5
twist_fp = Twist()


vel_cmd_pub = rospy.Publisher("/xtdrone/" + vehicle_type + '_' + vehicle_id + "/cmd_vel_enu", Twist, queue_size=10)

while not rospy.is_shutdown():
    offset = target - vehicle_posi[0:2]
    velcmd = 0.5*offset/np.linalg.norm(offset)
    twist_fp.linear.x = velcmd[0]
    twist_fp.linear.y = velcmd[1]
    twist_fp.linear.z = min(2, 2 * (cruise_height - vehicle_posi[2]))
    vel_cmd_pub.publish(twist_fp)
rospy.spin()






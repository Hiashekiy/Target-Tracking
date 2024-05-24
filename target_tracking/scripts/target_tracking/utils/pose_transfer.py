#! /usr/bin/env python
# -*- coding: utf-8 -*-

import math
import sys
import rospy
import numpy as np
import tf.transformations as tft
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class PoseTransfer:
    def __init__(self):
        rospy.init_node("pose_translation_and_rolation", anonymous=True)
        rospy.Rate(60)

        self.msgs_type = sys.argv[3]
        self.vehicle_id = sys.argv[2]
        self.vehicle_type = sys.argv[1]

        self.topic_sub = rospy.get_param( "~topic_sub_data", 'topic_sub_NONE')
        self.topic_pub = rospy.get_param("~topic_pub_data", 'topic_pub_NONE')
        self.pose_offset = eval(rospy.get_param( "~pose_offset", "[0,0,0,0,0,0]"))
        self.quat_offset = tft.quaternion_from_euler(self.pose_offset[3]*math.pi/180, self.pose_offset[4]*math.pi/180, self.pose_offset[5]*math.pi/180)

        if self.msgs_type == "odom":
            rospy.Subscriber(self.topic_sub, Odometry, self.odom_callback, queue_size=1)
            self.odom_pub = rospy.Publisher(self.topic_pub, Odometry, queue_size=1)
        elif self.msgs_type == "pose":
            rospy.Subscriber(self.topic_sub, PoseStamped, self.pose_callback, queue_size=1)
            self.pose_pub = rospy.Publisher(self.topic_pub, PoseStamped, queue_size=1)
        else:
            raise ValueError
        
    def odom_callback(self, msg):
        father_posi = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y,msg.pose.pose.position.z])
        father_quat = np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])

        child_posi  = father_posi + self.pose_offset[0:3]
        child_quat = tft.quaternion_multiply(father_quat, self.quat_offset)
    
        msg.pose.pose.position.x = child_posi[0]
        msg.pose.pose.position.y = child_posi[1]  
        msg.pose.pose.position.z = child_posi[2]  
        msg.pose.pose.orientation.x = child_quat[0]
        msg.pose.pose.orientation.y = child_quat[1]
        msg.pose.pose.orientation.z = child_quat[2] 
        msg.pose.pose.orientation.w = child_quat[3]

        self.odom_pub.publish(msg)
    
    def pose_callback(self, msg):
        father_posi = np.array([msg.pose.position.x, msg.pose.position.y,msg.pose.position.z])
        father_quat = np.array([msg.pose.orientation.x, msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w])

        child_posi  = father_posi + self.pose_offset[0:3]
        child_quat = tft.quaternion_multiply(father_quat, self.quat_offset)
        
        msg.pose.position.x = child_posi[0]
        msg.pose.position.y = child_posi[1]  
        msg.pose.position.z = child_posi[2]  
        msg.pose.orientation.x = child_quat[0]
        msg.pose.orientation.y = child_quat[1]
        msg.pose.orientation.z = child_quat[2] 
        msg.pose.orientation.w = child_quat[3]

        self.pose_pub.publish(msg)

if __name__ == "__main__":
    try:
        pose_trans_rot = PoseTransfer()
        rospy.spin()
    except ValueError as e:
        rospy.logerr("massege type is error")
    
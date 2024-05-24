#! /usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
import numpy as np
import tf.transformations as tft
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from message_filters import Subscriber, ApproximateTimeSynchronizer
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Accel


class FastPlanAuxiliary:
    def __init__(self):
        self.vehicle_id = sys.argv[2]
        self.vehicle_type = sys.argv[1]
    
        rospy.init_node(sys.argv[1]+'_'+self.vehicle_id+"fast_planner_auxiliary", anonymous=True)
        rospy.Rate(30)
        
        self.vehicle_id = sys.argv[2]
        self.vehicle_type = sys.argv[1]

        self.local_odom_pub = rospy.Publisher('/'+self.vehicle_type+'_'+self.vehicle_id+"/fast_planner/odom", Odometry, queue_size=10)
        self.local_accl_pub = rospy.Publisher('/'+self.vehicle_type+'_'+self.vehicle_id+"/local_accl", Accel, queue_size=10)

        body_odom_sub = Subscriber('/'+self.vehicle_type+'_'+self.vehicle_id+"/mavros/local_position/odom", Odometry)
        local_vel_sub = Subscriber('/'+self.vehicle_type+'_'+self.vehicle_id+"/mavros/local_position/velocity_local", TwistStamped)
        # body_imu_sub  = rospy.Subscriber('/'+self.vehicle_type+'_'+self.vehicle_id+"/imu_gazebo", Imu, self.imu_callback, queue_size=10)

        ts = ApproximateTimeSynchronizer([body_odom_sub, local_vel_sub], queue_size=100,  slop=0.1, allow_headerless=True)
        ts.registerCallback(self.OdomAndVelocity_callback)


    def OdomAndVelocity_callback(self, odom_msg, vel_msg):
        """ get the local pose and velocity of the vehicle
        :param odom_msg:
        :param vel_msg:
        :return: """
        odom_msg.twist.twist = vel_msg.twist
        self.local_odom_pub.publish(odom_msg)


    # def imu_callback(self, imu_msg):
    #     imu_data = imu_msg.linear_acceleration
    #     acc_body = np.array([-imu_data.x, -imu_data.y, imu_data.z])

    #     orient = imu_msg.orientation
    #     dotM = tft.quaternion_matrix([orient.x, orient.y, orient.z, orient.w])[0:3,0:3]
    #     acc_local = np.dot(dotM, acc_body) - [0, 0, 9.8]

    #     acc_msg = Accel()
    #     acc_msg.linear.x = acc_local[0]
    #     acc_msg.linear.y = acc_local[1]
    #     acc_msg.linear.z = acc_local[2]

    #     self.local_accl_pub.publish(acc_msg)


if __name__ == "__main__":
    fp_odom = FastPlanAuxiliary()
    rospy.spin()
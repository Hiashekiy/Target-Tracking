# #! /usr/bin/env python
# # -*- coding: utf-8 -*-

# import cv2
# import rospy
# import numpy as np
# from geometry_msgs.msg import PoseStamped
# from target_tracking.msg import TargetInfo
# from gazebo_msgs.srv import GetModelState
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge

# # def image_callback(msg):
# #     # 使用cv_bridge将ROS Image消息转换为OpenCV格式
# #     bridge = CvBridge()
# #     cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

# #     # 在这里可以使用cv_image进行进一步处理，例如显示图像
# #     cv2.imshow("Image", cv_image)
# #     print(cv_image.shape)
# #     cv2.waitKey(1)

# def traj_show_callback(msg):
#     global map

#     x = msg.pose.position.x
#     y = msg.pose.position.y
#     print(x, y)
#     ic = np.floor((x-map_init[0])/resolation).astype(int)
#     ir = np.floor((y-map_init[1])/resolation).astype(int)

#     map[ir, ic] = 255

#     cv2.imshow("map", map)
#     cv2.waitKey(1)


# if __name__ == "__main__":
#     rospy.init_node("test", anonymous=True)
#     rospy.Rate(30)
#     rospy.Time.now(); rospy.sleep(0.1)

#     map_size = np.array([160, 80])
#     map_init = [-map_size[0]/4, map_size[1]/2]
#     resolation = 0.1
#     map = np.zeros((map_size/resolation).astype(int)[::-1])

#     rospy.Subscriber("/iris_0/move_base_simple/goal", PoseStamped, traj_show_callback, queue_size=1)
#     rospy.spin()
#     # start_time = rospy.Time.now()
#     # target_pub = rospy.Publisher("/iris_0/target_info", TargetInfo, queue_size=1)
#     # while not rospy.is_shutdown():
#     #     duration = (rospy.Time.now() - start_time).to_sec()
#     #     print(duration)
#     #     if duration >= 10 and duration <= 10.5:
#     #         pass
#     #     else:
#     #         x = 20 + 2 * duration
#     #         y = duration**2/20
#     #         msg = TargetInfo()
#     #         msg.Class = "green"
#     #         msg.x = x; msg.y = y
#     #         target_pub.publish(msg)
#     #     rospy.sleep(0.1)



#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import matplotlib.pyplot as plt
from target_tracking.msg import PositionCommand
from nav_msgs.msg import Odometry
import numpy as np
from message_filters import Subscriber, ApproximateTimeSynchronizer
from geometry_msgs.msg import TwistStamped, Twist


def callback(msg1, msg2, msg3):
    global last_value, last_time, x_data, y_data1, y_data2, y_data3

    vel = np.array([msg3.twist.twist.linear.x, msg3.twist.twist.linear.y, msg3.twist.twist.linear.z, msg3.twist.twist.angular.z])
    if last_value is not None:
        data1 = np.linalg.norm(vel - last_value)
    else:
        data1 = 0
    
    data2 = msg1.velocity.y
    data3 = msg2.angular.z

    x_data.append(msg1.header.stamp.to_sec())
    y_data1.append(data1)
    y_data2.append(data2)
    y_data3.append(data3)

    last_value = vel
    last_time = msg1.header.stamp

    if len(x_data) > 5000:
        x_data = x_data[len(x_data)-5000:]
    if len(y_data1) > 5000:
        y_data1 = y_data1[len(y_data1)-5000:]
    if len(y_data2) > 5000:
        y_data2 = y_data2[len(y_data2)-5000:]
    if len(y_data3) > 5000:
        y_data3 = y_data3[len(y_data2)-5000:]


if __name__ == "__main__":
    rospy.init_node('plotting_node')
    x_data  = []
    y_data1 = []
    y_data2 = []
    y_data3 = []

    last_value = None
    last_time = rospy.Time.now()

    cmd_xt_sub = Subscriber("/xtdrone/iris_0/cmd_vel_enu", Twist)
    cmd_fp_sub  = Subscriber("iris_0/position_control", PositionCommand)
    # real_vel_sub = Subscriber('/iris_0/mavros/local_position/velocity_local', TwistStamped, queue_size=1)
    odom_sub = Subscriber("/iris_0/mavros/local_position/odom",  Odometry)

    ts = ApproximateTimeSynchronizer([cmd_fp_sub,  cmd_xt_sub, odom_sub], queue_size=10, slop=0.1, allow_headerless=True)
    ts.registerCallback(callback)

    plt.figure()
    while not rospy.is_shutdown():
        plt.clf()
        # plt.plot(x_data, y_data1, 'r', x_data, y_data2, 'g')
        plt.plot(x_data, y_data1)
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Real-time Curve')
        plt.grid(True)
        plt.pause(0.01)


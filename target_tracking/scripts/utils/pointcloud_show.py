#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import numpy as np
import cv2

map_size = np.array([40, 40])
map_origin = -map_size / 2
resolution = 0.1
map_voxel_size = np.ceil(map_size / resolution).astype (int)[: :-1]
map_voxel1 = np.zeros(map_voxel_size, dtype=np.uint8)
map_voxel2 = np.zeros(map_voxel_size, dtype=np.uint8)

def map_callback(msg):
    rospy.loginfo("receive map massege")
    global map_voxel1
    map_voxel1.fill(0)

    for point in point_cloud2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True):
        x,  y,  z = point[:3]
        index = np.floor(([x,y] - map_origin) / resolution).astype (int)
        index[0] = max(0, min(index[0], map_voxel_size[1]-1))
        index[1] = max(0, min(index[1], map_voxel_size[0]-1))
        # intensity = max(0, min(intensity, 255))
        map_voxel1[index[1], index[0]] = np.uint8(255)
        
    cv2.imshow("map1", map_voxel1)
    cv2.waitKey(1)

if __name__ == "__main__":
    print("start")
    rospy.init_node("build_map_node")
    rospy.Rate(20)
    rospy.Subscriber("/sdf_map/occupancy_inflate", PointCloud2,  map_callback, queue_size=10)
    rospy.spin()

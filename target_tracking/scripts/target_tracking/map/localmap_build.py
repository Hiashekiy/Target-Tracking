#! /usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import math
import rospy
import numpy as np
import tf.transformations as tft
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header


class occupancy_map:
    def __init__(self, vehicle_type, vehicle_id):
        self.vehicle_id = vehicle_id
        self.vehicle_type = vehicle_type
        self.vehicle_size = rospy.get_param("/vehicle_size", 0.5)
        self.vehicle_num = rospy.get_param("/vehicle_num", 1)
        self.vehicles_init = np.array(eval(rospy.get_param("/vehicle_init_posi", str([[0.0,0.0,0.0]] * self.vehicle_num)))).astype(np.float64)
        self.vehicles_posi = self.vehicles_init - self.vehicles_init[int(self.vehicle_id)]

        self.posi_body_free = np.zeros([3,0])
        self.posi_body_occupy = np.zeros([3,0])
        self.Laser_posi = None
        self.Laser_rotM = None

        self.angle_min = rospy.get_param("~angle_min", -np.pi)
        self.angle_max = rospy.get_param("~angle_max",  np.pi)
        self.sample_num = rospy.get_param("~sample_num", 512)
        self.laser_max_distance = rospy.get_param("~laser_detection_max_distance", 20)
        self.angle_range = np.linspace(self.angle_min, self.angle_max, self.sample_num)
      
        self.resolution_publish = 0.5
        self.map_pub_range = np.array([20, 20])
        self.map_size_x = rospy.get_param("~map_size_x", 180)
        self.map_size_y = rospy.get_param("~map_size_y", 100)
        self.map_init_x = rospy.get_param("~map_init_x", -self.map_size_x/4-self.vehicles_init[int(self.vehicle_id), 0])
        self.map_init_y = rospy.get_param("~map_init_y", -self.map_size_y/2-self.vehicles_init[int(self.vehicle_id), 1])
        self.map_size_z = rospy.get_param("~map_size_z", 2)
        self.map_init_z = rospy.get_param("~map_init_z", 4)
        self.resolution = rospy.get_param("~map_resolution", 1)

        self.grid_map_size = np.ceil(np.array([self.map_size_y, self.map_size_x])/self.resolution).astype(int)
        self.grid_map = np.zeros(self.grid_map_size, dtype=np.float)
        self.odds_map = np.zeros(self.grid_map_size, dtype=np.float)
        self.astar_map = np.zeros(self.grid_map_size, dtype=np.float)
        
        self.updata_esdfmap = False
        self.esdf_map = np.zeros(self.grid_map_size, dtype=np.float)

        self.inflation_ratio = int(math.ceil(2/self.resolution))
        self.inflation_kernel = np.array([[i, j] for i in np.arange(-self.inflation_ratio, self.inflation_ratio) for j in np.arange(-self.inflation_ratio, self.inflation_ratio)])
        self.obstacle_inflation_map = np.zeros(self.grid_map_size, dtype=np.float)

        self.last_velocity = None
        self.inverse_sensor_max = 0.7
        self.upper_confidence_limit = 0.9
        self.lower_confidence_limit = 0.1
        self.odds_upper = math.log(self.upper_confidence_limit/(1-self.upper_confidence_limit))
        self.odds_lower = math.log(self.lower_confidence_limit/(1-self.lower_confidence_limit))

        self.free_points = []
        self.occupy_points = []

        self.occupy_poxel_num = 0

        rospy.Subscriber('/'+self.vehicle_type+'_'+self.vehicle_id+"/scan", LaserScan, self.LaserScan_callback, queue_size=10)
        rospy.Subscriber('/'+self.vehicle_type+'_'+self.vehicle_id+"/mavros/local_position/odom", Odometry, self.Odom_callback, queue_size=10)
        
        for i in range(self.vehicle_num):
            rospy.Subscriber('/'+self.vehicle_type+'_'+str(i)+"/mavros/local_position/pose", PoseStamped, self.VehiclesPosi_callback, i, queue_size=10)

        self.points_pub = rospy.Publisher("~cloud_points", PointCloud2, queue_size=10)

        rospy.Timer(rospy.Duration(1, 0), self.MapPublish_timer)
        # rospy.Timer(rospy.Duration(0, 5e8), self.UpdataEsdf_timer)


    def LaserScan_callback(self, msg):
        detect_data = np.array(msg.ranges)

        theta_occupy = self.angle_range[np.isfinite(detect_data)]
        distance_occupy = detect_data[np.isfinite(detect_data)]
        self.posi_body_occupy = np.zeros([3, len(distance_occupy)])
        self.posi_body_occupy[0, :] = distance_occupy * np.cos(theta_occupy) 
        self.posi_body_occupy[1, :] = distance_occupy * np.sin(theta_occupy)
        
        detect_data[~np.isfinite(detect_data)] = self.laser_max_distance
        free_distance = detect_data * 0.5
        self.posi_body_free = np.zeros([3, len(detect_data)])
        self.posi_body_free[0, :] = free_distance * np.cos(self.angle_range)
        self.posi_body_free[1, :] = free_distance * np.sin(self.angle_range)


    def Odom_callback(self, msg):
        posi = msg.pose.pose.position
        orient = msg.pose.pose.orientation
        self.Laser_posi = np.array([posi.x, posi.y, posi.z])
        self.Laser_rotM = tft.quaternion_matrix([orient.x, orient.y, orient.z, orient.w])[0:3, 0:3]

        """ calculate the index of voxel occupied """
        posi_local_occupy = np.dot(self.Laser_rotM, self.posi_body_occupy) + self.Laser_posi.reshape(3,1)
        posi_local_occupy = self.RemoveVehiclePoint(posi_local_occupy)                   # remove the points around another vehicle
        posi_occupy_cut = posi_local_occupy[:, posi_local_occupy[2,:]>2]

        if len(posi_occupy_cut.T) > 0:
            self.occupy_points = np.array(self.PosiToIndex(posi_occupy_cut[0], posi_occupy_cut[1])).T
        else:
            self.occupy_points = []

        """ calculate the index of voxel free """
        start_point = np.array(self.PosiToIndex(self.Laser_posi[0], self.Laser_posi[1]))

        self.free_points = []
        free_endposi = np.dot(self.Laser_rotM, self.posi_body_free) + self.Laser_posi.reshape(3,1)
        posi_occupy_cut = free_endposi[:, free_endposi[2,:]>2]
        if len(posi_occupy_cut.T > 0):
            free_endpoints = np.array(self.PosiToIndex(posi_occupy_cut[0], posi_occupy_cut[1])).T
            free_endpoints = np.unique(free_endpoints, axis=0)
            for end_point in free_endpoints:
                for i, j in self.BresenhamLine(start_point, end_point):
                    if len(self.occupy_points) > 0 and (self.occupy_points == np.array([i,j])).all(axis=1).any():
                        break
                    else:
                        self.free_points.append([i,j])
            self.free_points = np.array(self.free_points)

        twt = msg.twist.twist 
        vel = np.array([twt.linear.x, twt.linear.y, twt.linear.z, twt.angular.z])
        if self.last_velocity is not None:
            d = np.linalg.norm(vel - self.last_velocity)
        else:
            d = 10
        self.last_velocity = vel
        
        inverse_value = math.exp(-100*d) + 0.5 if math.exp(-100*d) + 0.5 < self.inverse_sensor_max else self.inverse_sensor_max
        
        # inverse_value = self.inverse_sensor_max
        if len(self.free_points) > 0:
            self.free_points = np.unique(self.free_points, axis=0)
            self.odds_map[self.free_points[:,0], self.free_points[:,1]] += math.log((1-inverse_value)/inverse_value)

        if len(self.occupy_points) > 0:
            self.occupy_points = np.unique(self.occupy_points, axis=0)
            self.odds_map[self.occupy_points[:,0], self.occupy_points[:,1]] += math.log(inverse_value/(1-inverse_value))
                    
        self.grid_map[self.odds_map > self.odds_upper] = 1
        self.grid_map[self.odds_map < self.odds_lower] = -1

        obstacle = np.array(np.where(self.grid_map==1)).T
        obstacle_inflation = (obstacle [:, None, :] + self.inflation_kernel).reshape(-1, 2)
        self.obstacle_inflation_map[obstacle_inflation[:,0], obstacle_inflation[:,1]] = 1

        self.AddVehiclePoints()

        # if len(obstacle) != self.occupy_poxel_num:
        #     self.updata_esdfmap = True
        #     self.occupy_poxel_num = len(obstacle)


    def VehiclesPosi_callback(self, msg, i):
        posi = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        self.vehicles_posi[i] = posi + self.vehicles_init[i] - self.vehicles_init[int(self.vehicle_id)]
        

    def MapPublish_timer(self, event):
        """ cut the map around vehicle in self.map_pub_range"""
        if self.Laser_posi is not None:
            bound = self.Laser_posi[0:2]+ np.vstack((-self.map_pub_range, self.map_pub_range))
            bound_row, bound_col = self.PosiToIndex(bound[0:2, 0], bound[0:2, 1])
            map_pub = self.grid_map[bound_row[0]:bound_row[1], bound_col[0]:bound_col[1]]

            """ find these occupied grid and calculate their posi """
            obs_points = np.where(map_pub==1)
            obs_pointx = (obs_points[1]+bound_col[0]) * self.resolution + self.map_init_x
            obs_pointy = (obs_points[0]+bound_row[0]) * self.resolution + self.map_init_y
            points_xy = np.vstack([obs_pointx, obs_pointy]).T

            """ publish the points around another vehicles to fast planner"""
            other_vehicles_posi = np.delete(self.vehicles_posi, int(self.vehicle_id), axis=0)
            distance = np.linalg.norm(other_vehicles_posi[:, 0:2] -  self.Laser_posi[0:2], axis=1) 
            points_add = other_vehicles_posi[distance< self.map_pub_range[0], 0:2]
            inflation_kernel = np.array([[i,j] for i in np.arange(-1, 1) for j in np.arange(-1, 1)])
            points_add = (points_add[:, None, :] + inflation_kernel).reshape([-1,2])
            points_xy =  np.vstack((points_xy, points_add))
            
            """ add subdivision points """
            sub_div_times = int(self.resolution/self.resolution_publish)
            grid_offset =  np.array([[i, j] for i in range(sub_div_times) for j in range(sub_div_times)])
            points_subdivision = (points_xy[:, None, :] + grid_offset * self.resolution_publish).reshape(-1, 2)

            """ turn these 2d point to 3d """
            points_expand = np.concatenate((points_subdivision , np.zeros([len(points_subdivision), 1])), axis=1)
            heights = (np.arange(0, self.map_size_z, self.resolution_publish) + self.map_init_z).reshape((-1, 1))
            points_expand = np.tile(points_expand[None, :, :], (len(heights), 1, 1))
            points_expand[:, :, -1] = heights
            points_expand = points_expand.reshape((-1, 3))

            """ publish obstacle points clound """
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = "map"
            cloud = pc2.create_cloud_xyz32(header, points_expand)
            self.points_pub.publish(cloud)
    

    def UpdataEsdf_timer(self, event):
        if self.updata_esdfmap:
            self.esdf_map = np.full(self.grid_map_size, np.inf)
            # Forward pass
            for i in range(self.grid_map_size[0]):
                for j in range(self.grid_map_size[1]):
                    if self.grid_map[i, j] == 1:
                        self.esdf_map[i, j] = 0
                    else:
                        if i > 0:
                            self.esdf_map[i, j] = min(self.esdf_map[i, j], self.esdf_map[i - 1, j] + 1)
                        if j > 0:
                            self.esdf_map[i, j] = min(self.esdf_map[i, j], self.esdf_map[i, j - 1] + 1)
                        if i > 0 and j > 0:
                            self.esdf_map[i, j] = min(self.esdf_map[i, j], self.esdf_map[i - 1, j - 1] + np.sqrt(2))
            # Backward pass
            for i in range(self.grid_map_size[0]-1, -1, -1):
                for j in range(self.grid_map_size[1]-1, -1, -1):
                    if i < self.grid_map_size[0] - 1:
                        self.esdf_map[i, j] = min(self.esdf_map[i, j], self.esdf_map[i + 1, j] + 1)
                    if j < self.grid_map_size[1] - 1:
                        self.esdf_map[i, j] = min(self.esdf_map[i, j], self.esdf_map[i, j + 1] + 1)
                    if i < self.grid_map_size[0] - 1 and j < self.grid_map_size[1] - 1:
                        self.esdf_map[i, j] = min(self.esdf_map[i, j], self.esdf_map[i + 1, j + 1] + np.sqrt(2))
            self.updata_esdfmap = False


    def BresenhamLine(self, start_index, end_index):
        """Yield integer coordinates on the line from (row0, col0) to (row1, col1).
        Input coordinates should be integers.
        The result will contain both the start and the end point.
        """
        row1 = end_index[0]; col1 = end_index[1]
        row0 = start_index[0]; col0 = start_index[1]
        drow = row1 - row0; dcol = col1 - col0
        row_sign = 1 if drow > 0 else -1
        col_sign = 1 if dcol > 0 else -1
        drow = abs(drow); dcol = abs(dcol)

        if drow > dcol:
            ii, ij, ji, jj = row_sign, 0, 0, col_sign
        else:
            drow, dcol = dcol, drow
            ii, ij, ji, jj = 0, col_sign, row_sign, 0
        D = 2 * dcol - drow
        j = 0
        for i in range(drow + 1):
            yield row0 + i * ii + j * ji, col0 + i * ij + j * jj
            if D >= 0:
                j += 1
                D -= 2 * drow
            D += 2 * dcol


    def PosiToIndex(self, posix, posiy):
        col = np.floor((posix - self.map_init_x)/self.resolution).astype(int)
        row = np.floor((posiy - self.map_init_y)/self.resolution).astype(int)
        col = np.clip(col, 0, self.grid_map_size[1]-1)
        row = np.clip(row, 0, self.grid_map_size[0]-1)
        return row, col
    

    def RemoveVehiclePoint(self, posi):
        for i in range(self.vehicle_num):
            if i != int(self.vehicle_id):
                vehicle_posi_i = self.vehicles_posi[i, 0:2].reshape([2,1])
                distance = np.linalg.norm(posi[0:2, :]-vehicle_posi_i, axis=0)
                posi = posi[:, distance > self.vehicle_size]
        return posi


    def AddVehiclePoints(self):
        other_vehicles_posi = np.delete(self.vehicles_posi, int(self.vehicle_id), axis=0)
        other_vehicles_points = np.array(self.PosiToIndex(other_vehicles_posi[:,0], other_vehicles_posi[:,1])).T

        inflation_ratios = int(math.ceil(1/self.resolution))
        inflation_kernel = np.array([[i,j] for i in range(-inflation_ratios, inflation_ratios) for j in range(-inflation_ratios, inflation_ratios)])
        points_inflation = (other_vehicles_points[:, None, :] + inflation_kernel).reshape([-1,2])

        col = np.clip(points_inflation[:, 1], 0, self.grid_map_size[1]-1)
        row = np.clip(points_inflation[:, 0], 0, self.grid_map_size[0]-1)
        self.astar_map = self.obstacle_inflation_map.copy()
        # self.astar_map = self.grid_map.copy()
        self.astar_map[row, col] = 1

if __name__ == "__main__":
    rospy.init_node("localmap_build", anonymous=True)
    rospy.Rate(30)
    localmap = occupancy_map("iris", '0')
    rospy.spin()


        



#! /usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import math
import sys
import rospy
import numpy as np
from enum import Enum
import tf.transformations as tft
import matplotlib.pyplot as plt

from map.localmap_build import occupancy_map
from utils.astar import astar
from utils.distance_field import sedt_8points
from std_msgs.msg import String, Float32
from mavros_msgs.msg import State
from geometry_msgs.msg import Twist, PoseStamped, TwistStamped
from target_tracking.msg import PositionCommand, TargetInfo


class VehicleState(Enum):
    LOCK = 1               # vehicle is in lock
    RISING = 2             # vehicle is raising to the height of cruise
    CRUISE = 3             # vehicle is cruising
    TRACKING = 4           # vehicle is tracking target
    MISS_TARGET_JUST = 5   # vehicle lost tartget before long


class PlannerManage:
    def __init__(self):
        rospy.init_node("fast_planner_control_node", anonymous=True)
        rospy.Rate(30)

        """ parameter for control """
        self.receive_control = False                     # receive control command from fast planner
        self.time_last_receive = 0                       # the time of last receiving control command
    
        """ variables abouts vehicle """
        self.vehicle_id = sys.argv[2]
        self.vehicle_tp = sys.argv[1]
        self.armed = False                               # Check if the vehicle is not locked
        self.vehicle_state = VehicleState.LOCK           # the state of vehicle
        self.vehicle_local_yaw = 0
        self.vehicle_local_pos = np.array([0,0,0])
        self.vehicle_local_vel = np.array([0,0,0])
        self.vehicle_number = rospy.get_param("/vehicle_num", 1)
        self.vehicle_initposi = np.array(eval(rospy.get_param("/vehicle_init_posi", str([[0.0,0.0,0.0]] * self.vehicle_number)))) 

        """ msg to publish for controling """
        self.twist_fp = Twist()                          # msg to XTdrone received from fast planner
        self.tracking_goal = PoseStamped()               # msg to fast planner in tracking    
        self.cruising_goal = PoseStamped()               # msg to fast planner in cruising

        """ variables about target tracking """
        self.target_current = None                                      # store the position of the tartget to track: np.array([target.x, target.y])
        self.have_target = False                                        # have target to track
        self.miss_target_just = False                                   # miss the target in a while
        self.track_space = rospy.get_param("~track_distance", 3)        # the certain distance keep from the target
        self.traj_length = rospy.get_param("~track_distance", 2)        # the length of trajectory in tracking

        """ variables for cruise """
        self.cruise_order = 0                                           # the index of cruise point
        self.cruise_height = rospy.get_param("~cruise_height", 5)       # the flight altitude of cruising
        self.cruise_global_points = np.array(eval(rospy.get_param("~cruise_global_points", str([[0,0], [120, 0]]))))
        self.cruise_local_points = self.cruise_global_points - self.vehicle_initposi[int(self.vehicle_id), 0:2]
        
        """ build map """
        self.map = occupancy_map(self.vehicle_tp, self.vehicle_id)                # receive sensor data to build map  
        self.map_rslt = self.map.resolution                                       # the resolution of map
        self.grid_map_size = self.map.grid_map_size                               
        self.map_init = np.array([self.map.map_init_x, self.map.map_init_y])      # The position of the top left corner of the map

        """ variables for trajectory """
        self.path_current = []                # path got from astar search
        self.path_node_current = []           # path node publish to tast planner
        self.replan_current = False

        self.path_standby = []                # path got from astar search, used after completing the path_current 
        self.path_node_standby = []           # path node publish to tast planner, used after completing the path_current 
        self.replan_standby = False

        self.traj_left_time = 0                # the remaining time of each trajectory planned by fast planner
        self.path_node_space = int(10/self.map_rslt)
        self.path_search = astar(self.grid_map_size, self.map_init, self.map_rslt)

        """ creat publisher """
        self.move_goal_pub = rospy.Publisher("~fast_planner/move_goal_topic", PoseStamped, queue_size=10)                                  # goal massege publish to fast planner
        self.ser_cmd_pub = rospy.Publisher('/xtdrone/' + self.vehicle_tp + '_' + self.vehicle_id + '/cmd', String, queue_size=10)          # server control command pubulish to XTdrone
        self.vel_cmd_pub = rospy.Publisher("/xtdrone/" + self.vehicle_tp + '_' + self.vehicle_id + "/cmd_vel_enu", Twist, queue_size=10)   # velocity control command publish to XTdrone

        """ creat subscriber """
        rospy.Subscriber("~fast_planner/traj_control_topic", PositionCommand, self.FastPlanCmd_callback, queue_size=10)                    # control command(velocity) from fast planner
        rospy.Subscriber('/'+self.vehicle_tp+'_'+self.vehicle_id+"/mavros/state", State, self.ArmState_callback, queue_size=10)
        rospy.Subscriber('/'+self.vehicle_tp+'_'+self.vehicle_id+"/target_info", TargetInfo, self.TargetAndState_callback, queue_size=10)  # target massege from node target_pulish
        rospy.Subscriber('/'+self.vehicle_tp+'_'+self.vehicle_id+"/time_left_of_current_traj", Float32, self.TrajTimeLeft_callback, queue_size=10)
        rospy.Subscriber('/'+self.vehicle_tp+'_'+self.vehicle_id+"/mavros/local_position/pose", PoseStamped, self.VehiclePose_callback, queue_size=10)

        """ creat Timer """
        rospy.Timer(rospy.Duration(0, 1e8), self.SearchPath_timer)        # creat Timer to search path
        rospy.Timer(rospy.Duration(0, 1e8), self.CheckSafety_timer)       # creat Timer to search path
        rospy.Timer(rospy.Duration(0, 1e8), self.StateExecute_timer)      # creat Timer to run state machine


    def StateExecute_timer(self, event):
        """ execute correspondong tasks based on the state of vehicle
        :param event:
        :return: """
        self.UpdataState()

        if   self.vehicle_state == VehicleState.LOCK:
            self.ser_cmd_pub.publish('ARM')
                
        elif self.vehicle_state == VehicleState.RISING:
            self.ser_cmd_pub.publish('OFFBOARD')
            self.HeightKeep()
                
        elif self.vehicle_state == VehicleState.CRUISE:
            self.CruiseControl()

        elif self.vehicle_state == VehicleState.TRACKING:
            target_stamped = self.PoseStampedFill(self.target_current)
            self.move_goal_pub.publish(target_stamped)

        elif self.vehicle_state == VehicleState.MISS_TARGET_JUST:
            target_stamped = self.PoseStampedFill(self.target_current)
            self.move_goal_pub.publish(target_stamped)                    
        else:
            rospy.logerr("the state of vehicel in error")
        
        if rospy.Time.now().to_sec() - self.time_last_receive > 1:
            self.receive_control = False

    def UpdataState(self):
        """ update the state of vehicle
        :return: """
        if self.vehicle_state == VehicleState.LOCK:
            if self.armed:                                               # vehicle is unlocked 
                self.vehicle_state = VehicleState.RISING
        
        elif self.vehicle_state == VehicleState.RISING:
            if self.vehicle_local_pos[2] >  self.cruise_height - 0.1:    # vehicle has got the flight altitude
                self.vehicle_state = VehicleState.CRUISE
        
        elif self.vehicle_state == VehicleState.CRUISE:
            if self.have_target:                                         # vehilce find target 
                self.vehicle_state = VehicleState.TRACKING
        
        elif self.vehicle_state == VehicleState.TRACKING:
            self.path_node_current, self.path_node_standby = [], []      # clear the residual cruise path 
            if self.miss_target_just:                                    
                self.vehicle_state = VehicleState.MISS_TARGET_JUST       # lost target in a while 
            elif not self.have_target:
                self.vehicle_state = VehicleState.CRUISE                 # lost target too long and turn to cruise
        
        elif self.vehicle_state == VehicleState.MISS_TARGET_JUST:
            if self.have_target: 
                self.vehicle_state = VehicleState.TRACKING               # find target again
            elif not self.miss_target_just:
                self.vehicle_state = VehicleState.CRUISE                 # lost target too long and turn to cruise

    def HeightKeep(self):
        """ remain the fight altitude of vehicle using pid
        :return: """
        tw = Twist(); 
        tw.linear.z = min(2, 2 * (self.cruise_height - self.vehicle_local_pos[2]))    # calculate velocity control using pid
        self.vel_cmd_pub.publish(tw)

    def CruiseControl(self):
        """ publish the path node to fast planner and delete  the path node that have already passed through
        :return: """
        if len(self.path_node_current) > 0:
            if not self.receive_control:                                              # do not receive control command from fasr planner
                cruise_stamped = self.PoseStampedFill(self.path_node_current[0])
                self.move_goal_pub.publish(cruise_stamped)
                self.HeightKeep()
            else:
                d = np.linalg.norm(self.vehicle_local_pos[0:2] - self.path_node_current[0])    
                if  d < self.path_node_space * 0.4 or (self.receive_control and self.traj_left_time < 1):   # the vehicle is close to the path node
                    ic = np.where(np.all(self.path_current == self.path_node_current[0], axis=1))[0][0]
                    self.path_current = self.path_current[ic:]
                    self.path_node_current = np.delete(self.path_node_current, 0, axis=0)

                    if len(self.path_node_current) == 0:                                                    # complete current path
                        self.cruise_order = (self.cruise_order + 1) % len(self.cruise_local_points)
                    else:
                        cruise_stamped = self.PoseStampedFill(self.path_node_current[0])
                        self.move_goal_pub.publish(cruise_stamped)
        else:
            self.HeightKeep()

    def PoseStampedFill(self, goal):
        """ fill PoseStamped message with the data from input
        :param goal:
        :return: pose_stamped """
        pose_stamped = PoseStamped()
        if self.vehicle_state == VehicleState.TRACKING or self.vehicle_state == VehicleState.MISS_TARGET_JUST:
            p = self.SearchGoal(goal, self.vehicle_local_pos[0:2])                               # seach a suitable point for tracking
            pose_stamped.pose.position.x = p[0]
            pose_stamped.pose.position.y = p[1]
        else:
            pose_stamped.pose.position.x = goal[0]
            pose_stamped.pose.position.y = goal[1]
        pose_stamped.pose.position.z = self.cruise_height

        yaw = math.atan2(goal[1]-self.vehicle_local_pos[1], goal[0]-self.vehicle_local_pos[0])   
        quat = tft.quaternion_from_euler(0, 0, yaw)                                              # turn Euler angle into quaternion
        pose_stamped.pose.orientation.x = quat[0]
        pose_stamped.pose.orientation.y = quat[1]
        pose_stamped.pose.orientation.z = quat[2]
        pose_stamped.pose.orientation.w = quat[3]
        return pose_stamped

    def SearchGoal(self, target_posi, vehicle_posi):
        """ seach a suitable goal point for target tracking 
        :param target_posi:
        :param vehicle_posi:
        :return: a suitable goal point """
        vector_diff = target_posi - vehicle_posi
        vector_norm = np.linalg.norm(vector_diff)
        vector_unit = vector_diff / vector_norm

        if vector_norm > (self.track_space + self.traj_length) or vector_norm < abs(self.track_space - self.traj_length):
            return vector_unit * (vector_norm - self.track_space) + vehicle_posi
        else:
            r1 = self.traj_length
            r2 = self.track_space
    
            Xaxis = vector_unit
            Yaxis = np.array([-Xaxis[1], Xaxis[0]])

            RotM = np.zeros([2,2])
            RotM[0,:] = Xaxis; RotM[1,:] = Yaxis
            xy_new_coord = np.dot(RotM, vector_diff)

            x_cross = (xy_new_coord[0] - (r2**2 - r1**2) / xy_new_coord[0]) / 2
            y_cross = np.sqrt(r1**2 - x_cross**2)
            
            posi1 = np.dot(RotM.T, [x_cross,  y_cross]) + vehicle_posi
            posi2 = np.dot(RotM.T, [x_cross, -y_cross]) + vehicle_posi

            index1 = self.map.PosiToIndex(posi1[0], posi1[1])
            index2 = self.map.PosiToIndex(posi2[0], posi2[1])

            d1 = self.path_search.disf.RMDTO_Search(index1)
            d2 = self.path_search.disf.RMDTO_Search(index2)

            if d1 != 0 or d2 != 0:
                return posi1 if d1 > d2 else posi2                                  # return the point further from obstacle
            else:
                return vehicle_posi

    def SearchPath_timer(self, event):
        """ seach global current or standby path for cruise, current path will be used immediately, 
        and standby path will be used when current path is completed
        :param event:
        :return: """
        if len(self.path_node_current) == 0 or self.replan_current:
            if len(self.path_standby) > 0 and self.CheckPathSafety(self.path_standby):       # check if there is standby path for using
                self.path_current, self.path_standby = self.path_standby, []                 
                self.path_node_current, self.path_node_standby = self.path_node_standby, []
            else:
                self.CheckCuisePoint(self.cruise_local_points[self.cruise_order])
                self.SeachPath(True)

        if len(self.path_node_standby) == 0 or self.replan_standby:
            if len(self.path_node_current) == 1:
                index = (self.cruise_order + 1) % len(self.cruise_local_points)
                self.CheckCuisePoint(self.cruise_local_points[index])
                self.SeachPath(False)

    def CheckCuisePoint(self, cruise_point):
        """ check if the cruise points is too close to obstacle, if it is too close, search a suitable points 
        nearby to server as the new cruise points
        :param cruise_point: 
        :return: """
        rowc, colc = self.map.PosiToIndex(cruise_point[0], cruise_point[1])
        lu = np.array([max(0, rowc-4), max(0, colc-4)])
        rd = np.array([min(self.grid_map_size[0], rowc+4), min(self.grid_map_size[1], colc+4)])

        if (self.map.astar_map[lu[0]:rd[0], lu[1]:rd[1]] == 1).any():
            lu_sedt = np.array([max(0, rowc-20), max(0, colc-20)])
            rd_sedt = np.array([min(self.grid_map_size[0], rowc+20), min(self.grid_map_size[1], colc+20)])                
            sedt_region = self.map.astar_map[lu_sedt[0]:rd_sedt[0], lu_sedt[1]:rd_sedt[1]]                                 # slice a region from map

            sedt = sedt_8points(sedt_region)

            lu_search = np.array([max(0, rowc-10) , max(0, colc-10)]) - lu_sedt
            rd_search = np.array([min(self.grid_map_size[0], rowc+10), min(self.grid_map_size[1], colc+10)]) - lu_sedt
            search_region = sedt[lu_search[0]:rd_search[0], lu_search[1]:rd_search[1]]                                     # slice the effective region

            best_points = np.array(np.unravel_index(np.argmax(search_region), search_region.shape)) + lu_search + lu_sedt  # search the point furthest from obstacle
            cruise_point = best_points[::-1] * self.map_rslt + self.map_init
            self.cruise_local_points[self.cruise_order] = cruise_point

            print("new cruise point: ", cruise_point)

    def SeachPath(self, search_curr):
        """ seach global current or standby according to the value of search_curr using astar
        :param  search_curr: if search_curr is True, search current path, otherwise search standby path
        :return: """
        rospy.loginfo(self.vehicle_tp + '_' + self.vehicle_id + " seach path")
        i0 = self.cruise_order; i1 = (i0 + 1) % len(self.cruise_local_points)
        if search_curr:
            origin_point = self.vehicle_local_pos[0:2]
            target_point = self.cruise_local_points[i0]
        else:
            origin_point = self.cruise_local_points[i0]
            target_point = self.cruise_local_points[i1]
        
        self.path_search.run(self.map.astar_map, origin_point, target_point)
        if self.path_search.search_success:
            path_node = self.path_search.path_posi[range(self.path_node_space, len(self.path_search.path_posi), self.path_node_space)]   # select path node at certain intervals
            if len(path_node) > 0:                                               # add target_point to the end of path_node
                if np.any(path_node[-1] != target_point):
                    path_node = np.vstack((path_node, target_point))
            else:
                path_node = np.vstack((path_node, target_point))
            if search_curr:
                self.path_node_current = path_node
                self.path_current = self.path_search.path_posi
            else:
                self.path_node_standby = path_node
                self.path_standby = self.path_search.path_posi
        else:
            rospy.loginfo("search failed")

    def CheckSafety_timer(self, event):
        """ check if the path is too close to obstacle, if it is too close, mark the flag for a new path search
        :param event: 
        :return: """
        if len(self.path_current) > 0:
            self.replan_current = not self.CheckPathSafety(self.path_current)
        if len(self.path_standby) > 0:
            self.replan_standby = not self.CheckPathSafety(self.path_standby)

    def CheckPathSafety(self, path):
        """ check if the path is too close to obstacle
        :param path: the path to check
        :return: whether the path is safe """
        index_row, index_col = self.map.PosiToIndex(path[:,0], path[:,1]) 
        path_index = self.map.astar_map[index_row, index_col]
        return False if (path_index == 1).any() else True         

    def FastPlanCmd_callback(self, msg):
        """ receive position control massage from fast planner and publish it to XTdrone
        :param msg:
        :return: """
        self.twist_fp.linear.x = msg.velocity.x
        self.twist_fp.linear.y = msg.velocity.y
        self.twist_fp.linear.z = min(2, 2 * (self.cruise_height - self.vehicle_local_pos[2]))
        
        if self.vehicle_state == VehicleState.TRACKING or self.vehicle_state == VehicleState.MISS_TARGET_JUST:      
            linkyaw = math.atan2(self.target_current[1]-self.vehicle_local_pos[1], self.target_current[0]-self.vehicle_local_pos[0])
            if abs(linkyaw - self.vehicle_local_yaw) <= math.pi:                        
                yaw_offset = linkyaw - self.vehicle_local_yaw
            else:
                if (linkyaw - self.vehicle_local_yaw) > math.pi:
                    yaw_offset = (linkyaw - self.vehicle_local_yaw) - 2 * math.pi
                else:
                    yaw_offset = (linkyaw - self.vehicle_local_yaw) + 2 * math.pi
            self.twist_fp.angular.z = 4 * yaw_offset                                   # control the camero orientation towards target
        else:
            self.twist_fp.angular.z = msg.yaw_dot

        if self.vehicle_state in [VehicleState.CRUISE, VehicleState.TRACKING, VehicleState.MISS_TARGET_JUST]:
            self.vel_cmd_pub.publish(self.twist_fp)

        self.receive_control = True
        self.time_last_receive = rospy.Time.now().to_sec()

    def GetYawOffset(self, yaw1, yaw2):
        """ calculate the difference between yaw1 and yaw2, and limit it into -pi to pi 
        :param yaw1:
        :param yaw2:
        :return: the difference between yaw1 and yaw2 """
        if abs(yaw1 -  yaw2) <= math.pi:
            yaw_offset = yaw1 -  yaw2
        else:
            if (yaw1 -  yaw2) > math.pi:
                yaw_offset = (yaw1 -  yaw2) - 2 * math.pi
            else:
                yaw_offset = (yaw1 -  yaw2) + 2 * math.pi
        return yaw_offset
    
    def TargetAndState_callback(self, msg):
        """ get target infomation and chage the state of vehicle
        :param msg:
        :return: """
        if msg.Class == "NoTarget":
            self.have_target = False
            self.miss_target_just = False

        elif msg.Class == "MissTargetJust":
            self.have_target = False
            self.miss_target_just = True
        else:
            self.have_target = True
            self.miss_target_just = False
            self.target_current = np.array([msg.x, msg.y])

    def VehiclePose_callback(self, pose_msg):
        """ get the yaw and position of vehicle from ros massege
        :param msg:
        :return: """
        pose = pose_msg.pose
        self.vehicle_local_yaw = tft.euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])[2]
        self.vehicle_local_pos = np.array([pose.position.x, pose.position.y, pose.position.z])

    def TrajTimeLeft_callback(self, msg):
        """ get the remaining time of each trajectory planned by fast planner
        :param msg:
        :return: """
        self.traj_left_time = msg.data
    
    def ArmState_callback(self, msg):
        """ check the arm state of vehicle
        :param msg:
        :return: """
        self.armed = msg.armed


if __name__ == "__main__":
    fpcontrol = PlannerManage()
    # plt.figure(fpcontrol.vehicle_tp+fpcontrol.vehicle_id)
    # while not rospy.is_shutdown():   
    #     map_to_show = ((fpcontrol.map.astar_map.copy()+1) * 127 + 1).astype(np.uint8)
    #     map_to_show = cv2.cvtColor(map_to_show, cv2.COLOR_GRAY2BGR)
    #     if len(fpcontrol.path_current) > 0:
    #         index = np.floor((fpcontrol.path_current -fpcontrol.map_init) / fpcontrol.map_rslt).astype(int)
    #         map_to_show[index[:,1], index[:,0], :] = [0,255,0]
    #     y0, x0 = fpcontrol.map.PosiToIndex(fpcontrol.vehicle_local_pos[0],  fpcontrol.vehicle_local_pos[1])
    #     pt1 = (x0-1, y0-1)
    #     pt2 = (x0+1, y0+1)
    #     cv2.rectangle(map_to_show, pt1, pt2, (255, 255,255), 1)
    #     plt.imshow(map_to_show)
    #     plt.pause(1)
    rospy.spin()






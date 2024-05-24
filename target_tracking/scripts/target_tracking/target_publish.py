#! /usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import math
import rospy
import numpy as np
import tf.transformations as tft

from enum import Enum
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import PoseStamped
from yolov8_ros_msgs.msg import BoundingBoxes
from target_tracking.msg import TargetInfo
from ros_actor_cmd_pose_plugin_msgs.msg import ActorInfo
from message_filters import Subscriber, ApproximateTimeSynchronizer

class TrackState(Enum):
    NOTARGET = 1
    TRACKING = 2
    MISS_TARGET_JUST = 3

class Target_Broadcast:
    def __init__(self):
        rospy.init_node('target_broadcast', anonymous=True)
        self.rate = rospy.Rate(30)
            
        self.vehicle_tp = sys.argv[1]                               # the type of vehicle 
        self.vehicle_id = sys.argv[2]                               # the id of vehicle 
        self.vehicle_state = TrackState.NOTARGET
        self.vehicle_number = rospy.get_param("/vehicle_num", 1)    # total number of vehicles
        self.vehicle_initposi = np.array(eval(rospy.get_param("/vehicle_init_posi", str([[0.0, 0.0, 0.0]] * self.vehicle_number))))             
                          
        self.cx = rospy.get_param("~cx", 376.0)                     # the parameters of camero
        self.cy = rospy.get_param("~cy", 240.0)
        self.fx = rospy.get_param("~fx", 375.9986188774177)
        self.fy = rospy.get_param("~fy", 375.9986188774177)    
        
        self.receive_detection = False
        self.lose_detection_count = 0                               # the times that has lost massege from yolov8

        self.have_target = False
        self.miss_start_time = 0                                    # the first time of lose target 
        self.target_followed_byself = None
        self.targets_followed_byall = [None] * self.vehicle_number

        self.target_classes = ["green", "blue", "brown", "white", "red"]                               # the yolov8 classes of target
        self.score_topic_color = ["green", "blue", "brown", "white", "red1", "red2"]

        self.noise_posi = np.array([-27, 13.65]) - self.vehicle_initposi[int(self.vehicle_id), 0:2]    # the position of interference

        self.target_publish  = rospy.Publisher('/'+self.vehicle_tp+'_'+self.vehicle_id+"/target_info", TargetInfo, queue_size=1)           # pulish the massege of target to plan_manage node
        self.score_topic_pub = [rospy.Publisher("/actor_" + color + "_info", ActorInfo, queue_size=1) for color in self.score_topic_color] # pulish the massege of target to judge program

        yolov8_left_sub = Subscriber("~yolov8_left", BoundingBoxes, queue_size=1)
        camera_sub = Subscriber("~camero_pose", PoseStamped, queue_size=1)

        ts = ApproximateTimeSynchronizer([yolov8_left_sub, camera_sub], queue_size=1, slop=0.1, allow_headerless=True)
        ts.registerCallback(self.YoloAndPose_callback) 

        for i in range(self.vehicle_number):                                                            # subscrib massege of all the targets which are being followed 
            rospy.Subscriber('/'+self.vehicle_tp+'_'+str(i)+"/target_info", TargetInfo, self.AllTargetsTracked_callback, i, queue_size=10)

        rospy.Timer(rospy.Duration(0.1), self.TargetPosiPub_timer)

        # self.get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)  # get the real pisition of target to verify the accuracy of the localization algorithm

    def YoloAndPose_callback(self, yolov8_left_msg, camera_msg):
        """ traverse the targets deteced by yolov8, if the target is what we sreach for then record and publish it
        :param yolov8_left_msg:  
        :param camera_msg:  
        :return: """
        self.receive_detection = True
        self.have_target = False

        for target in yolov8_left_msg.bounding_boxes:
        
            if self.noise_posi is None and target.Class == "woman":                         # updata the pisition of interference
                noise_posi = rospy.get_param("/noise_posi", None)
                if noise_posi is None :
                    noise_xy = self.GetTargetLocalPosi(target, camera_msg)
                    if np.linalg.norm(noise_xy - [camera_msg.pose.position.x, camera_msg.pose.position.y]) < 4:
                        self.noise_posi = noise_xy
                        rospy.set_param("/noise_posi", (noise_xy+self.vehicle_initposi[int(self.vehicle_id)][0:2]).tolist())
                        print("set woman position")
                else:
                    self.noise_posi = noise_posi - self.vehicle_initposi[int(self.vehicle_id)][0:2]

            if target.Class in self.target_classes:
                targetxy = self.GetTargetLocalPosi(target, camera_msg)                      # calculate the pisition in local coordinate systerm
                if self.TargetValidityCheck(targetxy):
                    if self.vehicle_state == TrackState.NOTARGET:                       
                        self.RecordPublishTarget(targetxy, target.Class, camera_msg)
                        break
                    elif self.vehicle_state == TrackState.TRACKING and target.Class == self.target_followed_byself[1]:     # check if the target is the one which is followed last moment                     
                        self.RecordPublishTarget(targetxy, target.Class, camera_msg)
                        break                 
        self.StateChange()

    def RecordPublishTarget(self, targetxy, Class, camera_msg):
        """ record and publish target massege
        :param targetxy: the position of target 
        :param Class: the calss of target
        :param camera_msg:  
        :return: """
        self.have_target = True                      
        self.target_followed_byself = [targetxy, Class]
        print(self.vehicle_id, "following ", Class)
        if np.linalg.norm(targetxy - [camera_msg.pose.position.x, camera_msg.pose.position.y]) < 8:
            self.PublishScoreMsg(Class, targetxy+self.vehicle_initposi[int(self.vehicle_id)][0:2])
        # rospy.loginfo("in accuracy:" + str(self.posi_check(targetxy+self.vehicle_initposi[int(self.vehicle_id)], Class)))                                             

    def StateChange(self):
        """ update the state of vehicle
        :return: """
        if self.vehicle_state == TrackState.NOTARGET:
            if self.have_target:                                                # find target 
                self.vehicle_state = TrackState.TRACKING 

        elif self.vehicle_state == TrackState.TRACKING:
            if not self.have_target:
                self.vehicle_state = TrackState.MISS_TARGET_JUST                # lost target in a while 
                self.miss_start_time = rospy.Time.now()
                self.target_followed_byself = None

        elif self.vehicle_state == TrackState.MISS_TARGET_JUST:
            if self.have_target:                                                # find target again
                self.vehicle_state = TrackState.TRACKING
            else:
                if (rospy.Time.now()-self.miss_start_time).to_sec() > 4:        # Lost target for more than 4 seconds
                    self.vehicle_state = TrackState.NOTARGET

    def AllTargetsTracked_callback(self, msg, i):
        """  subscrib massege of all the targets which are being followed 
        :param msg:
        :param msg: the id of vehicle where the masseage from
        :return: """
        if  msg.Class == "NoTarget" or msg.Class == "MissTargetJust":
            self.targets_followed_byall[i] = None
        else:
            self.targets_followed_byall[i] = [np.array([msg.x, msg.y]), msg.Class]

    def GetTargetLocalPosi(self, target, camera_msg):
        """  calculate the pisition in local coordinate systerm 
        :param target: the detection about the target from yolov8
        :param camera_msg: the pose and position of carome 
        :return: he pisition in local coordinate systerm """
        v = target.ymax; u = (target.xmax + target.xmin) / 2
        theta = math.atan((self.cy-v)/self.fx)

        orient = camera_msg.pose.orientation
        camera_rotM = tft.quaternion_matrix([orient.x, orient.y,orient.z,orient.w])[0:3,0:3]
        
        x_camera = camera_rotM [:, 0]
        pitch_camera = math.acos(math.sqrt(x_camera[0]**2+x_camera[1]**2)/np.linalg.norm(x_camera))
        if x_camera[2] > 0:       
            pitch_camera = -pitch_camera

        Zc0 = camera_msg.pose.position.z/math.sin(pitch_camera)
        Zc = Zc0 + Zc0 / math.sin(pitch_camera-theta) * math.sin(theta) * math.cos(pitch_camera)

        Cxyz = Zc * np.array([(u - self.cx) / self.fx, (v - self.cy) / self.fy, 1])  
        rot  = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])      

        posi = camera_msg.pose.position
        Wxyz = np.dot(camera_rotM, np.dot(rot, Cxyz)) + [posi.x, posi.y, posi.z]

        return Wxyz[0:2]

    def TargetValidityCheck(self, targetxy):
        """ check the validity of target  
        :param targetxy: the pisition of target
        :return: """
        if_valid = True

        for i in range(self.vehicle_number):                                                # check if the target is followed by another vehicel
            if i != int(self.vehicle_id) and self.targets_followed_byall[i] is not None:
                posi_global1 = self.targets_followed_byall[i][0] + self.vehicle_initposi[i][0:2]
                posi_global2 = targetxy + self.vehicle_initposi[int(self.vehicle_id)][0:2]
                if np.linalg.norm(posi_global1 - posi_global2) < 4:
                    if_valid = False
                    break

        if self.noise_posi is None:                                                         # check if the target is disturbance
            noise_posi = rospy.get_param("/noise_posi", None)
            if noise_posi is not None:
                self.noise_posi = np.array(noise_posi) - self.vehicle_initposi[int(self.vehicle_id)][0:2]
                if np.linalg.norm(targetxy - self.noise_posi) < 4:
                    if_valid = False
        else:
            if np.linalg.norm(targetxy - self.noise_posi) < 4:
                if_valid = False

        return if_valid

    def PublishScoreMsg(self, Class, posi):
        """ pulish the massege of target to judge program 
        :param Class: the class of target
        :param posi: the position of targrt
        :return: """
        score_msg = ActorInfo(cls=Class, x=posi[0], y=posi[1])
        order_num = self.target_classes.index(Class)
        self.score_topic_pub[order_num].publish(score_msg)
        if order_num == 4:                                            # there are two red targets
            self.score_topic_pub[order_num+1].publish(score_msg)

    def TargetPosiPub_timer(self, event):
        """  pulish the massege of target to plan_manage node
        :param event: 
        :return: """
        if self.receive_detection:                                    # determine if YOLOv8 communication has been lost
            self.receive_detection = False
            self.lose_detection_count = 0
        else:
            self.lose_detection_count += 1
            if self.lose_detection_count > 100:
                self.have_target = False

        target_msg = TargetInfo()                                     # pulish the massege of target
        if self.vehicle_state == TrackState.TRACKING:    
            target_msg.Class = self.target_followed_byself[1]
            target_msg.x = self.target_followed_byself[0][0]
            target_msg.y = self.target_followed_byself[0][1]

        elif self.vehicle_state == TrackState.MISS_TARGET_JUST:  
            target_msg.Class = "MissTargetJust"
            target_msg.x = 0
            target_msg.y = 0
        else:
            target_msg.Class = "NoTarget"
            target_msg.x = 0
            target_msg.y = 0

        self.target_publish.publish(target_msg)


    # def posi_check(self, posi, Class):
    #     in_accuracy = False
    #     if Class is not "red":
    #         index = self.target_classes.index(Class)
    #         posi_real = self.get_model_state("actor_"+str(index), "")
    #         if np.linalg.norm(posi - [posi_real.pose.position.x, posi_real.pose.position.y]) < 1:
    #             in_accuracy = True
    #     else:
    #         posi_real1 = self.get_model_state("actor_4", "")
    #         posi_real2 = self.get_model_state("actor_5", "")
    #         if np.linalg.norm(posi - [posi_real1.pose.position.x, posi_real1.pose.position.y]) < 1 or np.linalg.norm(posi - [posi_real2.pose.position.x, posi_real2.pose.position.y]) < 1:
    #             in_accuracy = True
    #     return in_accuracy


if __name__ == "__main__":
    target_broadcast = Target_Broadcast()
    rospy.spin() 
            


                    
                

                    

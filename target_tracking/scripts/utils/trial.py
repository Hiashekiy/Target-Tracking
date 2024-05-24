#! /usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
import numpy as np

def check_publisher():
    topic_name = '/time_left_of_current_traj'

    # 获取话题信息
    topic_info = rospy.get_published_topics()

    # 遍历所有话题，查找指定话题
    topic_found = False
    for info in topic_info:
        print(info)
        if info[0] == topic_name:
            topic_found = True
            if info[1]:
                rospy.loginfo("Topic '%s' has publishers.", topic_name)
            else:
                rospy.loginfo("Topic '%s' has no publishers.", topic_name)
            break

    if not topic_found:
        rospy.loginfo("Topic '%s' not found.", topic_name)
    
def change(map1):
    map = map1.copy()
    map[0,0] = 1-map[0,0]
    return map

if __name__ == '__main__':
    print(sys.path)


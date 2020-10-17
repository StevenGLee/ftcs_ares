#!/usr/bin/env python
# -*- coding: UTF-8 -*-
# license removed for brevity

import sys
import rospy
from math import sqrt
from ftcs_control_msg.msg import Int8Vector
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3

position_Robot1 = Vector3()
position_Robot2 = Vector3()
position_Robot3 = Vector3()

#默认可通信距离为5
dist_range = 5.0

def dist(vec1, vec2):
    return sqrt((vec1.x - vec2.x)**2 + (vec1.y - vec2.y)**2 + (vec1.z - vec2.z)**2)

def can_comm(vec1, vec2):
    global dist_range
    if dist(vec1, vec2) <= dist_range:
        return 1
    else:
        return0

def OnGroundTruth_Robot1(ground_truth):
    global position_Robot1
    position_Robot1=ground_truth.pose.pose.position

def OnGroundTruth_Robot2(ground_truth):
    global position_Robot2
    position_Robot2=ground_truth.pose.pose.position

def OnGroundTruth_Robot3(ground_truth):
    global position_Robot3
    position_Robot3=ground_truth.pose.pose.position
    
def topology_publisher():
    
    global position_Robot1
    global position_Robot2
    global position_Robot3
    global dist_range

    rospy.init_node('topology_publisher', anonymous=False)
    topology_pub1 = rospy.Publisher('/robot1/topology', Int8Vector, queue_size=10)
    topology_pub2 = rospy.Publisher('/robot2/topology', Int8Vector, queue_size=10)
    topology_pub3 = rospy.Publisher('/robot3/topology', Int8Vector, queue_size=10)
    rate = rospy.Rate(50)
    if sys.argv[0] == 'dynamic_topology':
        rospy.Subscriber("/robot1/base_pose_ground_truth", Odometry, OnGroundTruth_Robot1)
        rospy.Subscriber("/robot2/base_pose_ground_truth", Odometry, OnGroundTruth_Robot2)
        rospy.Subscriber("/robot3/base_pose_ground_truth", Odometry, OnGroundTruth_Robot3)   
        if len(sys.argv) > 2:
            dist_range = float(sys.argv[1])
        

    while not rospy.is_shutdown():  
        topology1 = Int8Vector()
        topology2 = Int8Vector()
        topology3 = Int8Vector()
        if sys.argv[2] == 'dynamic_topology':
            #topology1.data = [0, can_comm(position_Robot1, position_Robot2), can_comm(position_Robot1, position_Robot3)] 
            topology1.data.append(0)
            topology1.data.append(can_comm(position_Robot1, position_Robot2))
            topology1.data.append(can_comm(position_Robot1, position_Robot3))

            #topology2.data = [can_comm(position_Robot2, position_Robot1), 0, can_comm(position_Robot2, position_Robot3)] 
            topology2.data.append(can_comm(position_Robot2, position_Robot1))
            topology2.data.append(0)
            topology2.data.append(an_comm(position_Robot2, position_Robot3))

            #topology3.data = [can_comm(position_Robot3, position_Robot1), can_comm(position_Robot3, position_Robot2), 0]
            topology3.data.append(can_comm(position_Robot3, position_Robot1))
            topology3.data.append(can_comm(position_Robot3, position_Robot2))
            topology3.data.append(0)

        else:
            #topology1.data = [0, 1, 1] 
            topology1.data.append(0)
            topology1.data.append(1)
            topology1.data.append(1)

            #topology2.data = [1, 0, 1]
            topology2.data.append(1)
            topology2.data.append(0)
            topology2.data.append(1) 

            #topology3.data = [1, 1, 0]
            topology3.data.append(1)
            topology3.data.append(1)
            topology3.data.append(0)
        
        topology_pub1.publish(topology1)
        topology_pub2.publish(topology2)
        topology_pub3.publish(topology3)
        #按照循环频率延时
        rate.sleep()


if __name__ == '__main__':
    try:
        topology_publisher()
    except rospy.ROSInterruptException as r:
        rospy.loginfo("topology_publisher node terminated.") 
        rospy.loginfo(r) 

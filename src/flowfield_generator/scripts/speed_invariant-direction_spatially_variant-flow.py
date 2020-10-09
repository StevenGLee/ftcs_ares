#!/usr/bin/env python
# -*- coding: UTF-8 -*-
# license removed for brevity
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import sin cos pi

ground_truth_latest = Odometry() #记录当前的实时状态

def OnGroundTruth(ground_truth):
    global ground_truth_latest
    ground_truth_latest = ground_truth

def flowfield_generator():
    
    global ground_truth_latest

    # ROS节点初始化
    rospy.init_node('flowfield_generator', anonymous=False)
    
    #创建一个Publisher，发布名为/person_info的topic，消息类型为learning_topic::Person，队列长度10
    flowfield_publisher = rospy.Publisher('flowfield_velocity', Twist, queue_size=10)
    rospy.Subscriber("base_pose_ground_truth", Odometry, OnGroundTruth)
    
    #设置循环的频率
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():  
        flowfield_velocity = Twist()
        
        #与王赞赞论文相同的流场设置
        fd_par=10*pi()/(360*(ground_truth_latest.pose.pose.position.x+ground_truth_latest.pose.pose.position.y))
        speed = 2
        
        flowfield_velocity.linear.x = -speed * sin(fd_par)
        flowfield_velocity.linear.y =  speed * cos(fd_par)
        #rospy.loginfo("Publish flowfield_velocity[linear.x=%f, linear.y=%f]",
        #              flowfield_velocity.linear.x, flowfield_velocity.linear.y)
        flowfield_publisher.publish(flowfield_velocity)
        #按照循环频率延时
        rate.sleep()
     

if __name__ == '__main__':
     try:
         flowfield_generator()
     except rospy.ROSInterruptException as r:
         rospy.loginfo("flowfield_generator node terminated.") 
         rospy.loginfo(r) 

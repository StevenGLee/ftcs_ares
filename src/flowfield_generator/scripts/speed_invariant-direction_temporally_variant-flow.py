#!/usr/bin/env python
# -*- coding: UTF-8 -*-
# license removed for brevity
import rospy
from geometry_msgs.msg import Twist
from math import sin, cos

def flowfield_generator():
    # ROS节点初始化
    rospy.init_node('flowfield_generator', anonymous=False)
    
    #创建一个Publisher，发布名为/person_info的topic，消息类型为learning_topic::Person，队列长度10
    flowfield_publisher = rospy.Publisher('flowfield_velocity', Twist, queue_size=10)
    
    #设置循环的频率
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():  
        flowfield_velocity = Twist()
        
        #与学长的仿真相同的流场设置
        speed = 0.2
        fd_par = rospy.get_time() * 5
        flowfield_velocity.linear.x = speed * sin(fd_par)
        flowfield_velocity.linear.y = speed * cos(fd_par)
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

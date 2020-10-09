#!/usr/bin/env python
# -*- coding: UTF-8 -*-
# license removed for brevity
import rospy
import sys

from geometry_msgs.msg import Twist, Accel, Vector3
from nav_msgs.msg import Odometry
from ftcs_control_msg.msg import Int8Vector
from math import sqrt, log, atan2, acos, pi

from numpy import *


#不考虑流场干扰的情况下控制器需要的物理量：1.当前位置坐标x、y 2.轨迹圆半径 3.扰动（在控制器中设置） 4.

ground_truth_latest = Odometry()
xi1 = 0
xi2 = 0
xi3 = 0
a1 = 0
a2 = 0
a3 = 0
def OnGroundTruth(ground_truth):
    global ground_truth_latest
    ground_truth_latest = ground_truth
    
def OnXi1(xi):
    global xi1
    xi1 = xi.x

def OnXi2(xi):
    global xi2
    xi2 = xi.x

def OnXi3(xi):
    global xi3
    xi3 = xi.x
    
def OnTopology(topology):
    global a1
    global a2
    global a3
    a1 = topology[0]
    a2 = topology[1]
    a3 = topology[2]

def sign(x):
    if x>0.0:
        return 1.0
    elif x==0.0:
        return 0.0
    else:
        return -1.0
    
def sat(x):
    if abs(x) <= 0.05:
        return 1/0.05*x
    else:
        return 1*sign(x)
    
def Controller_flowfield():
    global flowfield_velocity_latest
    global xi1
    global xi2
    global xi3
    global a1
    global a2
    global a3
    k1 = 1  
    k2 = 5  
    a = 0.1
    
    rho = float(sys.argv[1])
    
    rospy.init_node('controller', anonymous=False)
    rospy.loginfo("controller node initialized.") 
    
    control_msg_publisher = rospy.Publisher('control_msg', Twist, queue_size=10)
    xi_publisher = rospy.Publisher('xi', Vector3, queue_size=10)
    
    rospy.Subscriber("base_pose_ground_truth", Odometry, OnGroundTruth)
    rospy.Subscriber("/robot1/xi", Vector3, OnXi1)
    rospy.Subscriber("/robot2/xi", Vector3, OnXi2)
    rospy.Subscriber("/robot3/xi", Vector3, OnXi3)
    rospy.Subscriber('topology', Int8Vector, OnTopology)
    
    rate = rospy.Rate(50)
    while ground_truth_latest.pose.pose.position.x == 0:
        rate.sleep()
    
    x = ground_truth_latest.pose.pose.position.x
    y = ground_truth_latest.pose.pose.position.y
    xi = atan2(x, y)
    #if(x<0):
    #    xi -= pi
    pos = mat([[x],[y]])
    start_time = rospy.get_time()
    while not rospy.is_shutdown():  
        prev_x = x
        prev_y = y
        x = ground_truth_latest.pose.pose.position.x
        y = ground_truth_latest.pose.pose.position.y
        pos = mat([[x],[y]])
        
        #计算本机xi
        xi += acos((x*prev_x + y*prev_y) / sqrt((x*x + y*y) * (prev_x**2 + prev_y**2)))
        xi_msg = Vector3()
        xi_msg.x = xi
        xi_publisher.publish(xi_msg)
        Lambda = 1 - 1/rho * sqrt(x**2 + y**2)
        f00 = log(abs(1+Lambda)) - log(abs(Lambda-1)) + 5*Lambda
        N = pos / (rho * linalg.norm(pos)) #N方向，norm二范数
        T = mat([[0, 1], [-1, 0]]) * N                 #T方向
        pksi=1/rho                                     #偏xi除以偏s
        
        #判断通讯时间的关系,产生通讯间隔
        xicha=xi-a*(rospy.get_time() - start_time)
        #控制率
        vn=k1*f00+0.5*sat(f00)
        #vn=5*(Lambda-1)
        vt=linalg.norm(T)*(1/pksi)*(a-k2*((a1+a2+a3)*xi-a1*xi1-a2*xi2-a3*xi3)-20*xicha)#sat(xicha))
        vx=mat([1, 0]) * linalg.pinv(vstack((N.T, T.T))) * vstack((vn, vt))
        vy=mat([0, 1]) * linalg.pinv(vstack((N.T, T.T))) * vstack((vn, vt))
        print("-----")
        print(Lambda)
        print(vx)
        print("xi1:%f xi2:%f xi3:%f xhicha:%f" %(xi1, xi2, xi3, xicha))
        #print(vy)
        control_msg = Twist()
        control_msg.linear.x = vx.tolist()[0][0]
        control_msg.linear.y = vy.tolist()[0][0]
        control_msg_publisher.publish(control_msg)
        
        rate.sleep()

if __name__ == '__main__':
     try:
         Controller_flowfield()
     except rospy.ROSInterruptException as r:
         rospy.loginfo("controller node terminated.") 
         rospy.loginfo(r) 
 

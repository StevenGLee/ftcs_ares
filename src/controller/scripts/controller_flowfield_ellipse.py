#!/usr/bin/env python
# -*- coding: UTF-8 -*-
# license removed for brevity
import rospy
import sys

from geometry_msgs.msg import Twist, Accel, Vector3
from nav_msgs.msg import Odometry
from ftcs_control_msg.msg import Int8Vector, Formation
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
xi_bias = 0
x_bias = 0
y_bias = 0
x_scope = 0
y_scope = 0
clockwise_rotate = 0
controlling = 0
a = 0.2

def OnFormation(formation):
    global xi_bias, x_bias, y_bias, x_scope, y_scope, clockwise_rotate, a, controlling
    #如果长轴短轴都为0，则不控制
    if formation.scale.x==0 or formation.scale.y==0:
        controlling = 0
        return
    elif controlling == 0:
        start_time = rospy.get_time()
        
    xi_bias = formation.theta
    x_bias = formation.position.x
    y_bias = formation.position.y
    x_scope = formation.scale.x
    y_scope = formation.scale.y
    clockwise_rotate = formation.clockwise_rotation
    a=formation.speed
    if controlling == 0:
        controlling = 1
        xi = atan2(ground_truth_latest.pose.pose.position.x - x_bias, ground_truth_latest.pose.pose.position.y - y_bias) - xi_bias



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
    a1 = topology.data[0]
    a2 = topology.data[1]
    a3 = topology.data[2]

def sign(x):
    if x>0.0:
        return 1.0
    elif x==0.0:
        return 0.0
    else:
        return -1.0
    
def sat(x):
    if abs(x) <= 5:
        return x
    else:
        return 5*sign(x)
    
def Controller_flowfield():
    global flowfield_velocity_latest
    global xi1, xi2, xi3
    global a1, a2, a3
    global xi_bias, x_bias, y_bias, x_scope, y_scope, clockwise_rotate, a, controlling

    k1 = 10  
    k2 = 5  
    k3 = 1
    k4 = 5
    
    #rho = float(sys.argv[1])
    
    rospy.init_node('controller', anonymous=False)
    rospy.loginfo("controller node initialized.") 
    
    control_msg_publisher = rospy.Publisher('control_msg', Twist, queue_size=10)
    xi_publisher = rospy.Publisher('xi', Vector3, queue_size=10)
    
    rospy.Subscriber("base_pose_ground_truth", Odometry, OnGroundTruth)
    rospy.Subscriber("/robot1/xi", Vector3, OnXi1)
    rospy.Subscriber("/robot2/xi", Vector3, OnXi2)
    rospy.Subscriber("/robot3/xi", Vector3, OnXi3)
    rospy.Subscriber('topology', Int8Vector, OnTopology)
    rospy.Subscriber('formation', Formation, OnFormation)
    
    rate = rospy.Rate(50)
    while ground_truth_latest.pose.pose.position.x == 0:
        rate.sleep()
    
    x = ground_truth_latest.pose.pose.position.x - x_bias
    y = ground_truth_latest.pose.pose.position.y - y_bias

    xi = atan2(x, y) - xi_bias
    #if(x<0):
    #    xi -= pi
    pos = mat([[x],[y]])
    start_time = rospy.get_time()
    Lambda = 0
    Lambda_int = 0
    while not rospy.is_shutdown():  
        if controlling != 0:
            prev_x = x
            prev_y = y
            x = ground_truth_latest.pose.pose.position.x - x_bias
            y = ground_truth_latest.pose.pose.position.y - y_bias
            pos = mat([[x],[y]])

            #计算本机xi
            if (y>=0 and prev_y>=0 and x<=prev_x) or (y<=0 and prev_y<=0 and x>=prev_x) or (x>=0 and prev_x>=0 and y>=prev_y) or (x<=0 and prev_x<=0 and y<=prev_y):
                xi -= acos((x*prev_x + y*prev_y) / sqrt((x*x + y*y) * (prev_x**2 + prev_y**2)))
            else:
                xi += acos((x*prev_x + y*prev_y) / sqrt((x*x + y*y) * (prev_x**2 + prev_y**2)))
            expect_xi = a*(rospy.get_time() - start_time)
            xicha=xi-expect_xi
            if xicha > pi:
                xicha -= pi
            if xicha < -pi:
                xicha += pi
            if xicha < 0.1:
                xi = atan2(x, y) - xi_bias
                while abs(xi - expect_xi) > pi:
                    xi += 2*pi
                xicha=xi-expect_xi
            xi_msg = Vector3()
            xi_msg.x = xi
            xi_publisher.publish(xi_msg)
            rho = sqrt((x_scope*cos(xi - clockwise_rotate + xi_bias))**2 + (y_scope*sin(xi - clockwise_rotate + xi_bias))**2)
            Lambda_prev = Lambda
            Lambda = 1 - 1/rho * sqrt(x**2 + y**2)
            Lambda_int += Lambda
            if Lambda_int>7:
                Lambda_int = 7
            if Lambda_int<-7:
                Lambda_int=-7
            dLambda = Lambda - Lambda_prev
            f00 = log(abs(1+Lambda)) - log(abs(Lambda-1)) + 5*Lambda
            NNN = vstack((pos.tolist()[0][0]/(x_scope**2), pos.tolist()[1][0]/(y_scope**2)))
            N = NNN / linalg.norm(NNN) #N方向，norm二范数
            T = mat([[0, 1], [-1, 0]]) * N                 #T方向
            pksi=1/rho                                     #偏xi除以偏s

            #控制率
            vn=k1*f00 + k3 * abs(Lambda) * Lambda_int - k4 * dLambda * (abs((1 - Lambda)/2 + abs(1 - Lambda)/2) + abs((1 + Lambda)/2 + abs(1+ Lambda)/2))
            #vn=5*(Lambda-1)
            vt=linalg.norm(T)*(1/pksi)*(a-k2*((a1+a2+a3)*xi-a1*xi1-a2*xi2-a3*xi3)-20*xicha)#sat(xicha))
            v_square = vn**2 + vt**2
            if v_square >= 50:
                vn *= sqrt(50/v_square)
                vt *= sqrt(50/v_square)
            vx=mat([1, 0]) * linalg.pinv(vstack((N.T, T.T))) * vstack((vn, vt))
            vy=mat([0, 1]) * linalg.pinv(vstack((N.T, T.T))) * vstack((vn, vt))
            print("-----")
            print(Lambda)
            print("N:",N,", T:", T)
            print("vn:%f, vt:%f"%(vn, vt))
            print("vx:%f, vy:%f"%(vx, vy))
            print("xi1:%f xi2:%f xi3:%f xhicha:%f" %(xi1, xi2, xi3, xicha))
            #print(vy)
            control_msg = Twist()
            control_msg.linear.x = vx.tolist()[0][0]
            control_msg.linear.y = vy.tolist()[0][0]
            control_msg_publisher.publish(control_msg)
        else:
            control_msg = Twist()
            control_msg.linear.x = 0
            control_msg.linear.y = 0
            control_msg_publisher.publish(control_msg)
        rate.sleep()

if __name__ == '__main__':
     try:
         Controller_flowfield()
     except rospy.ROSInterruptException as r:
         rospy.loginfo("controller node terminated.") 
         rospy.loginfo(r) 
 

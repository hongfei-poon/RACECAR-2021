#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float32, Float32MultiArray, Int16
import nav_msgs.msg
import tf
import math


class vision_control:
    
    def __init__(self):
        self.x1 = 0
        self.y1 = 0
        self.z1 = 0

        # self.reply=0

        # self.light_x_max=rospy.get_param("light_x_max",3.61)  #红绿灯模块开启x坐标    hotel
        # self.light_x_min=rospy.get_param("light_x_min",1.91)  #红绿灯模块结束x坐标
        # self.light_y_max=rospy.get_param("light_y_max",0.5)  #红绿灯模块开启y坐标
        # self.light_y_min=rospy.get_param("light_y_min",-1)  #红绿灯模块结束y坐标

        
        self.light_x_max=rospy.get_param("light_x_max",2.51)  #红绿灯模块开启x坐标
        self.light_x_min=rospy.get_param("light_x_min",1.69)  #红绿灯模块结束x坐标
        self.light_y_max=rospy.get_param("light_y_max",-5.19)  #红绿灯模块开启y坐标
        self.light_y_min=rospy.get_param("light_y_min",-6.49)  #红绿灯模块结束y坐标

        # self.linex1=rospy.get_param("linex1",1.7)
        # self.line_x_min1=rospy.get_param("line_x_min1",3.70)    #车道线启动区间   hotel
        # self.line_x_max1=rospy.get_param("line_x_max1",3.10)
        # # self.liney1=rospy.get_param("liney1",-1.1)
        # self.line_y_min1=rospy.get_param("line_y_min1",-1)
        # self.line_y_max1=rospy.get_param("line_y_max1",0.5)

        self.line_x_min1=rospy.get_param("line_x_min1",0)    #车道线启动区间
        self.line_x_max1=rospy.get_param("line_x_max1",1.69)
        # self.liney1=rospy.get_param("liney1",-1.1)
        self.line_y_min1=rospy.get_param("line_y_min1",-4.70)
        self.line_y_max1=rospy.get_param("line_y_max1",-4.10)

        # self.line_x_min2=rospy.get_param("line_x_min2",4.60)   #车道线停止区间   hotel
        # self.line_x_max2=rospy.get_param("line_x_max2",4.50)
        # self.line_y_min2=rospy.get_param("line_y_min2",-1)
        # self.line_y_max2=rospy.get_param("line_y_max2",0.5)
        
        self.line_x_min2=rospy.get_param("line_x_min2",-2.75)   #车道线停止区间
        self.line_x_max2=rospy.get_param("line_x_max2",-0.36)
        self.line_y_min2=rospy.get_param("line_y_min2",-1.48)
        self.line_y_max2=rospy.get_param("line_y_max2",0)


        self.c1x=rospy.get_param("c1x",-1.408)
        self.c1y=rospy.get_param("c1y",-3.954)
        self.r1=rospy.get_param('r1',1.425)
        self.w1=rospy.get_param('w1',0.4)

        self.c2x=rospy.get_param("c2x",-0.235)
        self.c2y=rospy.get_param("c2y",-0.587)
        self.r2=rospy.get_param('r2',1.425)
        self.w2=rospy.get_param('w2',0.4)

        #订阅/odom
        self.odom_msg = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.odom_callback)
        # self.cv_return=rospy.Subscriber("",Float32MultiArray,self.cv_return_callback)
        #发布cv_control

        self.control = rospy.Publisher("cv_control",Int16, queue_size=1)
        # self.control_bringup = rospy.Publisher("bringup_control",Int16,queue_size=1)

        self.timer=rospy.Timer(rospy.Duration(0.025),self.timer_callback)
        self.cv_mode=0
        # self.bringup_mode=0
        

    def odom_callback(self,msg):
        self.x1=msg.pose.pose.position.x
        self.y1=msg.pose.pose.position.y
        self.z1=msg.pose.pose.position.z

        # print "x1=",self.x1,"\ny1=",self.y1,"\nz1=",self.z1
        print "light_x_max=",self.light_x_max
        print "light_x_min=",self.light_x_min
        print "light_y_max=",self.light_y_max
        print "light_y_min=",self.light_y_min



        if(self.x1<=self.light_x_max and self.x1>=self.light_x_min and self.y1<=self.light_y_max and self.y1>=self.light_y_min):        #红绿灯启动
            self.cv_mode=1
            rospy.loginfo("LIGHT")
        # elif(x1>=linex2 & x1<=linex1 & y1>=liney2 & y1<=liney1):
            # self.cv_mode=2
        elif(self.x1>=self.line_x_min1 and self.x1<=self.line_x_max1 and self.y1>=self.line_y_min1 and self.y1<=self.line_y_max1):     #车道线启动
            rospy.loginfo("LINE_START")
            self.cv_mode=2
            # self.bringup_mode=0  #跟随车道线


        elif(self.x1>=self.line_x_min2 and self.x1<=self.line_x_max2 and self.y1>=self.line_y_min2 and self.y1<=self.line_y_max2):      #车道线停止
            self.cv_mode=3
            rospy.loginfo("LINE_END")

            # self.bringup_mode=1  #跟随导航
        else:                                   #啥都没有
            self.cv_mode=0
            # print "6666"
       
        
        
        # self.control_bringup(self.bringup_mode)

    # def cv_return_callback(self,backmsg):
    #     self.reply= backmsg[0]


    def timer_callback(self,event):
        #rospy.loginfo('callback')
        print("cv_mode=",self.cv_mode)
        self.control.publish(self.cv_mode)
        print('x=',self.x1,'y=',self.y1)


if __name__=="__main__":
    rospy.init_node("vision_controller", anonymous=True)
    rospy.loginfo("start vision_controller mode")
    

    con=vision_control()
    
    
    rospy.spin()

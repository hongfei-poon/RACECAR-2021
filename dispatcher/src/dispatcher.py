#!/usr/bin/python
#-*- coding:utf-8 -*-


import os
import sys
import tty
import termios
import socket
import threading
import multiprocessing

import time
from time import sleep
import math

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan
from move_base_msgs.msg import MoveBaseActionFeedback
from move_base_msgs.msg import MoveBaseGoal
from std_srvs.srv import Empty

import numpy as np

class SockClient(threading.Thread):

    def __init__(self, host_ip, host_port,pipe):
        threading.Thread.__init__(self)
        self.running = False
        self.sock = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        
        self.sock.settimeout(2000)  # 20 seconds
        
        self.pipe=pipe
        try:  
            #print(host_ip,host_port)
            self.sock.connect((host_ip, host_port))
        except socket.error, e:
            print("Socket Connect Error:%s" % e)
            exit(1)
        rospy.loginfo("Socket connect success!")

        self.running = True
        self.error_cnt = 0
         
    def run(self):

        while self.running:
            #print('socket running\r\n')
            try:

                data = self.sock.recv(1024) 
                
                recv_data = data.encode('hex')
                cmd=self.decode_rx(data)
                print('socet_cmd:',cmd)
                self.pipe.send(cmd)
                print('send to pip')
                if not self.pipe.poll():
                    self.sock.send('no msg yet')
                    rospy.loginfo('NO MSG')
                    sleep(0.05)
                    continue
   
                pipe_rx=self.pipe.recv()
                
                if pipe_rx[0]=='odom':
                    car_odom=pipe_rx[1]
                    self.send_odommsg(car_odom)
                elif pipe_rx[0]=='message':
                    self.sock.send(pipe_rx[1])

                
                #print('socket end')
            except socket.error, e:
                print 'socket running error:', str(e)
                os._exit(0)
                break

        print 'SockClient Thread Exit\n'


    def send_odommsg(self,car_odom):
        header=np.array([0x01,0x10,0x01,0x10],dtype=np.uint8)
        msg_id=np.array([0xa0],dtype=np.uint8)
        x=car_odom.pose.pose.position.x
        y=car_odom.pose.pose.position.y
        yaw=math.atan2(car_odom.pose.pose.orientation.w,car_odom.pose.pose.orientation.z)
        vx=car_odom.twist.twist.linear.x
        vy=car_odom.twist.twist.linear.y
        vyaw=car_odom.twist.twist.angular.z
 
        odom_msg=np.array([x,y,yaw,vx,vy,vyaw],dtype=np.float32)
        odom_msg.dtype=np.uint8

        rear=np.array([0x03,0x30,0x03,0x30],dtype=np.uint8)
        msg=np.concatenate([header,msg_id,odom_msg,rear])
        self.sock.send(msg.tostring())

    def send_statusmsg(self,status):
        msg='\x01\x10\x01\x10'
        msg_id='\xb0'
        rear=rear='\x03\x30\x03\x30'
        msg.extend(msg_id)
        msg.extend(status)
        msg.extend(rear)
        self.sock.send()

    def decode_rx(self,rx_data):

        ret=('cmd','pause')
        param=[]

        index=rx_data.find('\x02\x20\x02\x20',0,len(rx_data))
        
        if index==-1:
            return []

        frame_size=np.array(rx_data[(index+4):(index+8)])
        frame_size.dtype=np.int32

        rx_data=rx_data[(index+8):(index+8+frame_size)]
        
        #if index!=-1 and index<len(rx_data)-frame_size:
        if True:
            command=rx_data[0]

            #check
            #check_sum=0
            #for ch in rx_data[0:(frame_size-1)]:
            #    check_sum=check_sum+ch
            #if check_sum==rx_data[8+frame_size]:
            if True:
                #running
                if(command=='\x10'):
                    ret=('cmd','running')
                    param=[]
                elif(command=='\x11'):
                    ret=('cmd','clearing')
                    
                    param=[]
                elif(command=='\x20'):
                    ret=('cmd','pause')
                    param=[]
                #target: initial pose of waiting area
                elif(command=='\x30'):
                    ret=('target','waiting')
                    
                    px=np.array(rx_data[1:5])  
                    py=np.array(rx_data[5:9])
                    pyaw=np.array(rx_data[9:13])
                    px.dtype=np.float32
                    py.dtype=np.float32
                    pyaw.dtype=np.float32
                    param=Pose()
                    param.position.x,param.position.y=px,py
                    param.orientation.w=math.cos(0.5*pyaw*math.pi/180)
                    param.orientation.z=math.sin(0.5*pyaw*math.pi/180)
                #target: go back to waiting area
                elif(command=='\x31'):
                    ret=('target','back')
                    px=np.array(rx_data[1:5])  
                    py=np.array(rx_data[5:9])
                    pyaw=np.array(rx_data[9:13])
                    px.dtype=np.float32
                    py.dtype=np.float32
                    pyaw.dtype=np.float32
                    param=Pose()
                    param.position.x,param.position.y=px,py
                    param.orientation.w=math.cos(0.5*pyaw*math.pi/180)
                    param.orientation.z=math.sin(0.5*pyaw*math.pi/180)
                #target: loading area
                elif(command=='\x40'):
                    ret=('target','loading')
                    px=np.array(rx_data[1:5])  
                    py=np.array(rx_data[5:9])
                    pyaw=np.array(rx_data[9:13])
                    px.dtype=np.float32
                    py.dtype=np.float32
                    pyaw.dtype=np.float32
                    param=Pose()
                    param.position.x,param.position.y=px,py
                    param.orientation.w=math.cos(0.5*pyaw*math.pi/180)
                    param.orientation.z=math.sin(0.5*pyaw*math.pi/180)
                #target: unloading area
                elif(command=='\x50'):
                    ret=('target','unloading')
                    px=np.array(rx_data[1:5])  
                    py=np.array(rx_data[5:9])
                    pyaw=np.array(rx_data[9:13])
                    px.dtype=np.float32
                    py.dtype=np.float32
                    pyaw.dtype=np.float32
                    param=Pose()
                    param.position.x,param.position.y=px,py
                    param.orientation.w=math.cos(0.5*pyaw*math.pi/180)
                    param.orientation.z=math.sin(0.5*pyaw*math.pi/180)
                #target: viapoint_1
                elif(command=='\x60'):
                    ret=('target','viapoint_1')
                    px=np.array(rx_data[1:5])  
                    py=np.array(rx_data[5:9])
                    pyaw=np.array(rx_data[9:13])
                    px.dtype=np.float32
                    py.dtype=np.float32
                    pyaw.dtype=np.float32
                    param=Pose()
                    param.position.x,param.position.y=px,py
                    param.orientation.w=math.cos(0.5*pyaw*math.pi/180)
                    param.orientation.z=math.sin(0.5*pyaw*math.pi/180)
                #target: viapoint_22
                elif(command=='\x61'):
                    ret=('target','viapoint_2')
                    px=np.array(rx_data[1:5])  
                    py=np.array(rx_data[5:9])
                    pyaw=np.array(rx_data[9:13])
                    px.dtype=np.float32
                    py.dtype=np.float32
                    pyaw.dtype=np.float32
                    param=Pose()
                    param.position.x,param.position.y=px,py
                    param.orientation.w=math.cos(0.5*pyaw*math.pi/180)
                    param.orientation.z=math.sin(0.5*pyaw*math.pi/180)
                
        #print ret,param
        return ret,param
 
class CarDispatcherROS(threading.Thread):
    def __init__(self,pipe):
        threading.Thread.__init__(self)

        rospy.init_node('Dispatcher',anonymous=True)

        #rospy.logerr("NO SERVICE")
  
       
        rospy.wait_for_service("move_base/clear_costmaps")

        self.clearing_request = rospy.ServiceProxy("move_base/clear_costmaps",Empty)
        self.timer_model = rospy.Timer(rospy.Duration(0.1),self.model_callback)
        self.s_path_model_time_start=0


        

        rospy.loginfo("services established!")

        print('CarDispatcherROS Thread Running\n')
        self.running=True
        self.odom_msg=Odometry()   
        self.viapoints_vis=PoseArray()
        self.viapoints_vis.header.frame_id='map'
        self.viapoints_vis.poses=[Pose(),Pose(),Pose(),Pose(),Pose()]


        self.pipe=pipe        
        self.command=String()
        self.command.data='running'

        

        '''
        nav_status:
        -1 ---no command yet  
        0  ---from waiting area to loading area
        1  ---from loading area to viapoint 1
        2  ---from viapoint 1 to unloading area
        3  ---from unloading area to viapoint 2
        4  ---from viapoint 2 to waiting area
        '''
        self.nav_status=-1

        self.s_path_approaching=False
        
        self.reach_status=False



        #waiting point (0,0,0)
        self.waiting_point=PoseStamped()
        self.waiting_point.header.frame_id='map'
        self.waiting_point.pose.position.x=rospy.get_param('waiting_x',default=0.0)
        self.waiting_point.pose.position.y=rospy.get_param('waiting_y',default=0.0)
        waiting_yaw=rospy.get_param('waiting_yaw',default=0.0)
        self.waiting_point.pose.orientation.z=math.sin(0.5*waiting_yaw*math.pi/180)
        self.waiting_point.pose.orientation.w=math.cos(0.5*waiting_yaw*math.pi/180)
        
        #initial localization
        self.initial=PoseWithCovarianceStamped()
        self.initial.header.frame_id='map'
        self.initial.pose.pose=self.waiting_point


        #loading point(2.13,-3.31,-90)
        self.loading_point=PoseStamped()
        self.loading_point.header.frame_id='map'
        self.loading_point.pose.position.x=rospy.get_param('loading_x',default=2.13)
        self.loading_point.pose.position.y=rospy.get_param('loading_y',default=-3.31)
        loading_yaw=rospy.get_param('loading_yaw',default=-90)
        self.loading_point.pose.orientation.z=math.sin(0.5*loading_yaw*math.pi/180)
        self.loading_point.pose.orientation.w=math.cos(0.5*loading_yaw*math.pi/180)

        #viapoint-1(-0.22,-8.13,-180)
        
        self.viapoint_1=PoseStamped()
        self.viapoint_1.header.frame_id='map'
        self.viapoint_1.pose.position.x=rospy.get_param('vp1_x',default=-0.22)
        self.viapoint_1.pose.position.y=rospy.get_param('vp1_y',default=-8.13)
        vp1_yaw=rospy.get_param('vp1_yaw',default=180)
        self.viapoint_1.pose.orientation.z=math.sin(0.5*vp1_yaw*math.pi/180)
        self.viapoint_1.pose.orientation.w=math.cos(0.5*vp1_yaw*math.pi/180)
        

        #unloading point   (-2.57,-6.23,90)
        self.unloading_point=PoseStamped()
        self.unloading_point.header.frame_id='map'
        self.unloading_point.pose.position.x=rospy.get_param('unloading_x',default=-2.57)
        self.unloading_point.pose.position.y=rospy.get_param('unloading_y',default=-6.24)
        unloading_yaw=rospy.get_param('unloading_yaw',default=90)
        self.unloading_point.pose.orientation.z=math.sin(0.5*unloading_yaw*math.pi/180)
        self.unloading_point.pose.orientation.w=math.cos(0.5*unloading_yaw*math.pi/180)

        #viapoint-2 (0.82,-4.43,90)
        self.viapoint_2=PoseStamped()   
        self.viapoint_2.header.frame_id='map'   
        self.viapoint_2.pose.position.x=rospy.get_param('vp2_x',default=0.94)
        self.viapoint_2.pose.position.y=rospy.get_param('vp2_y',default=-3.93)
        vp2_yaw=rospy.get_param('vp2_yaw',default=90)
        self.viapoint_2.pose.orientation.z=math.sin(0.5*vp2_yaw*math.pi/180)
        self.viapoint_2.pose.orientation.w=math.cos(0.5*vp2_yaw*math.pi/180)


        self.viapoints_vis.poses[0]=self.waiting_point.pose
        self.viapoints_vis.poses[1]=self.loading_point.pose
        self.viapoints_vis.poses[2]=self.viapoint_1.pose
        self.viapoints_vis.poses[3]=self.unloading_point.pose
        self.viapoints_vis.poses[4]=self.viapoint_2.pose
        
        #target rect
        num_pts=30
        self.rect_xmax=1.35
        self.rect_xmin=0.18
        self.rect_ymax=-3.83
        self.rect_ymin=-5.89
        self.rect_model_vel=1.0
        self.pth=Path()
        self.pth.header.frame_id='map'
        self.pth_length=self.rect_ymax-self.rect_ymin
        self.rect_time=self.pth_length/self.rect_model_vel
        
        pstart=PoseStamped()
        pstart.pose.position.x=(self.rect_xmin+self.rect_xmax)/2
        pstart.pose.position.y=self.rect_ymin
        pstart.pose.orientation.z=math.sin(math.pi/4)
        pstart.pose.orientation.w=math.cos(math.pi/4)
        pend=PoseStamped()
        pend.pose.position.x=pstart.pose.position.x
        pend.pose.position.y=self.rect_ymax
        pend.pose.orientation.x=0
        pend.pose.orientation.y=0
        pend.pose.orientation.z=math.sin(math.pi/4)
        pend.pose.orientation.w=math.cos(math.pi/4)
        self.pth.poses.append(pstart)
        step_y=(pend.pose.position.y-pstart.pose.position.y)/num_pts
        
        p=PoseStamped()
        for i in range(num_pts):
            p.pose.position.y=(i+1)*step_y+pstart.pose.position.y
            p.pose.position.x=pstart.pose.position.x
            p.pose.orientation=pstart.pose.orientation
            self.pth.poses.append(p)
        self.pth.poses.append(pend) 

        self.ref_model=PoseStamped()
        self.ref_model.pose.position=pstart.pose.position
        self.ref_model.pose.orientation=pstart.pose.orientation     

        self.target=PoseStamped()
        self.target.header.frame_id='map'
        
        self.sub_odom=rospy.Subscriber('/odom',Odometry,self.odom_callback)
        self.sub_path=rospy.Subscriber('move_base/TebLocalPlannerROS/local_plan',Path,self.path_callback) 
        self.sub_amcl=rospy.Subscriber('/amcl_pose',PoseWithCovarianceStamped,self.amcl_callback)
        self.pub_cmd=rospy.Publisher('/dispatcher/cmd',String,queue_size=1)
        self.pub_path=rospy.Publisher('dispatcher/plan',Path,queue_size=1)
        self.pub_target=rospy.Publisher('move_base_simple/goal',PoseStamped,queue_size=1)
        self.pub_initial=rospy.Publisher('/initialpose',PoseWithCovarianceStamped,queue_size=1)
        self.pub_viapoints_vis=rospy.Publisher('dispatcher/viapoints',PoseArray,queue_size=1)
        self.pub_ref_model=rospy.Publisher('dispatcher/ref_model',PoseStamped,queue_size=1)

    def odom_callback(self,odom_data):
        self.odom_msg=odom_data
        #self.pipe.send(('odom',self.odom_msg))
   
    def path_callback(self,path_data):
        #culculate path length
        print('nav_status=',self.nav_status)
        #self.pipe.send('nav_status',)

        path=path_data
        if self.nav_status==0:
            pass

        if self.nav_status==1:
            print('len',len(path.poses))
            if len(path.poses)<7:
                # viapoint_1 reached, go to unloading point
                self.nav_status=2
                self.target.pose=self.unloading_point.pose
                self.pub_target.publish(self.target)   
                rospy.loginfo("reach p1")
                #self.pipe.send(('message','vp1-reached') )             
        if self.nav_status==3:
            print('len=',len(path.poses))
            if len(path.poses)<7:
                
                self.nav_status=4    
                self.target.pose=self.waiting_point.pose
                print self.target.pose
                self.pub_target.publish(self.target)  
                rospy.loginfo("reach p2")    
                #self.pipe.send(('message','vp2-reached'))

    def amcl_callback(self,amcl_data):
        rospy.loginfo('amcl')
        self.pub_path.publish(self.pth)
        if amcl_data.pose.pose.position.x>self.rect_xmin and amcl_data.pose.pose.position.x<self.rect_xmax and amcl_data.pose.pose.position.y<self.rect_ymax and amcl_data.pose.pose.position.y>self.rect_ymin:
            #rospy.loginfo('S-Path Approaching')
            if(self.s_path_approaching==False):
                rospy.loginfo('S-Path Approaching, Reference Model Working')
                self.s_path_model_time_start=rospy.Time.now().to_sec()
            self.s_path_approaching=True
        else:
            self.s_path_approaching=False

    def model_callback(self,event):
        if(self.s_path_approaching==True):
            t=rospy.Time.now().to_sec()-self.s_path_model_time_start
            if(t<self.rect_time):
                
                self.ref_model.header.frame_id='map'
                self.ref_model.pose.position.x=(self.rect_xmax+self.rect_xmin)/2
                self.ref_model.pose.position.y=self.rect_ymin+t*self.rect_model_vel
                self.ref_model.pose.orientation.x=0
                self.ref_model.pose.orientation.y=0
                self.ref_model.pose.orientation.z=math.sin(math.pi/4)
                self.ref_model.pose.orientation.w=math.cos(math.pi/4)
                rospy.loginfo('ref_y=%f',self.ref_model.pose.position.y)
                self.pub_ref_model.publish(self.ref_model)

    def run(self):
        while self.running:
 
                #print('rospy running\r\n')
            
                    #if self.pipe.poll():
                    #print('begin to receive')
                    received=self.pipe.recv()
                    #print('poll:',received)
                
                    
                    #TODO:
                    self.pub_viapoints_vis.publish(self.viapoints_vis)
                    if received[0]==('cmd','running'):
                        self.command.data=received[0][1]
                        self.pub_cmd.publish(self.command)
                        rospy.loginfo('pipe sending')
                        #self.pipe.send(('cmd','running'))
                        rospy.loginfo('finish sending')
                        rospy.loginfo('running')
                    elif received[0]==('cmd','pause'):
                        self.command.data=received[0][1]
                        self.pub_cmd.publish(self.command)
                        #self.pipe.send(('message','pausing'))
                        rospy.loginfo('pausing')
                    elif received[0]==('cmd','clearing'):
                        self.clearing_request()
                        #self.pipe.send(('message','clearing costmap'))
                        rospy.loginfo('clearing costmap')
                    elif received[0]==('target','waiting'):
                        self.initial.pose.pose=received[1]
                        self.pub_initial.publish(self.initial)
                        #self.pipe.send(('message','setting initial pose'))
                        rospy.loginfo('hear initial pose')
                        
                    elif received[0]==('target','back'):
                        self.nav_status=3
                        self.waiting_point.pose=received[1]
                        self.target.pose=self.viapoint_2.pose
                        self.pub_target.publish(self.target)
                        #self.pipe.send(('message','going back'))
                        rospy.loginfo('going back home')
                    elif received[0]==('target','loading'):
                        self.loading_point.pose=received[1]
                        self.target.pose=self.loading_point.pose
                        self.pub_target.publish(self.target)
                        self.nav_status=0
                        #self.pipe.send(('message','going to loading point'))
                        rospy.loginfo('go to loading point')
                    elif received[0]==('target','unloading'):
                        self.unloading_point.pose=received[1]
                        #setting unloading point and then
                        #go to viapoint_1 first, when reached, continue navigating to unloading point
                        self.target.pose=self.viapoint_1.pose
                        #self.target.pose=self.unloading_point
                        self.pub_target.publish(self.target)
                        self.nav_status=1
                        #self.nav_status=2
                        #self.pipe.send(('message','going to unloading point'))
                        rospy.loginfo('going to unloading point')
                    elif received[0]==('target','viapoint_1'):
                        #setting viapoint_1, but no navigation until setting unloading point
                        self.viapoint_1.pose=received[1]
                        #self.pipe.send(('message','setting vp1'))


                    elif received[0]==('target','viapoint_2'):
                        #setting viapoint_2, but no navigation until receiving the returning command to waiting point 
                        self.viapoint_2.pose=received[1]
                        #self.pipe.send(('message','setting vp2'))



                    
        print('ROS:',received[0])
        print('CarDispatcherROS Thread Exit\n')
        os._exit(0)        

    


def readkey(getchar_fn=None):
    fd=sys.stdin.fileno()

    old_settings=termios.tcgetattr(fd)
    try:

        tty.setraw(sys.stdin.fileno())
        ch=sys.stdin.read(1)
        
    finally:
        termios.tcsetattr(fd,termios.TCSADRAIN,old_settings)
    return ch

if __name__ == "__main__":
         

        (pipe_socket,pipe_ros)=multiprocessing.Pipe()
        car_dispatcher=CarDispatcherROS(pipe_ros)
        ip=rospy.get_param('ip',default='192.168.31.43')
        port=rospy.get_param('port',default=8888)
        #ip='192.168.68.1'
        #port=6000
        sock_client = SockClient(ip, port, pipe_socket)
        

        t1 = multiprocessing.Process(target=sock_client.run) 
        t2 = multiprocessing.Process(target=car_dispatcher.run)
        
        car_dispatcher.start()
        sock_client.start()
        while True:
            key=readkey()
            
            if key=='q' or key=='Q' or key=='\x03':
                os._exit(0)
            elif key=='A' or key=='a':
                car_dispatcher.clearing_request()
                rospy.loginfo('clearing costmap')
            # elif key=='P' or key=='p':
            #     global_plan=car_dispatcher.plan_request(car_dispatcher.unloading_point,car_dispatcher.waiting_point,0.1)
                
            #     global_plan.plan.header.stamp=rospy.Time.now()
            #     global_plan.plan.header.frame_id='map'
            #     print(global_plan.plan.header)
            #     car_dispatcher.pub_target.publish(car_dispatcher.loading_point)  
            #     car_dispatcher.pub_path.publish(global_plan.plan)
                
            #     rospy.loginfo('make plan')
                
        sock_client.join()
        car_dispatcher.join()
        print('Terminated')

        

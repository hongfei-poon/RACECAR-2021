#ifndef NODE_EXAMPLE_TALKER_H
#define NODE_EXAMPLE_TALKER_H


#define NAVIGATION 0
#define VISUAL_CONTROL 1




// ROS includes.
#include "ros/ros.h"
#include "ros/time.h"

#include <iostream>
#include <math.h>
#include <string.h>
#include <vector>
#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Range.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>

//lib
#include <serial/serial.h>
#include <time.h>

#define CARL 0.31
#define CARW 0.325
#define PI 3.14159265
using namespace std;
using namespace boost::asio; 
using namespace boost;

typedef struct sMartcarControl{
  int TargetAngleDir;     //转向角度符号 0:直行；0x10:左转；0x20:右转  //not used
  int TargetAngle;        //角度
  int TargetSpeed;        //速度
  int TargetModeSelect;   //模式
  int TargetShiftPosition;
  bool control;
}sMartcarControl;        

 
class actuator
{
public:
  actuator(ros::NodeHandle nh);
  ~actuator();
  
  void run();

public:
     int m_baudrate;            
     std::string m_serialport;               //对应USB端口
     int servo_mid;                           //舵机中位
     int encoderLeft;                        //左编码器
     int encoderRight;                       //右编码器
     int calibrate_lineSpeed;                //标定线速度
     int calibrate_angularSpeed;             //标定角速度
     float ticksPerMeter;                    //一米脉冲数
     float ticksPer2PI;                      //每圈脉冲数
     float linearSpeed;                      //线速度
     float angularSpeed;                     //角速度
     float batteryVoltage;                   //电池电压

     int nav_status;                          //导航状态
     bool goal_reached;

     float lineTern;                          //车道线转向系数(角度）
     int  vel1;                               //S弯速度

    //  float v1;                                //S弯线速度（备用）
    //  float w1;                                //S弯角速度（备用）

    //  float v2;                                //测试S弯线速度
    //  float w2;                                //测试S弯角速度

     float angle2;                            //测试转角
     float direction2;                        //测试方向
     int vel2;                                //测试匀速速度
       
     short tempaccelX,tempaccelY,tempaccelZ; //加速度缓存区
     short tempgyroX,tempgyroY,tempgyroZ;    //角速度缓存区
     short tempmagX,tempmagY,tempmagZ;       //磁力计缓存区
     double accelX,accelY,accelZ;            //加速度 
     double gyroX,gyroY,gyroZ;               //角速度 
     double magX,magY,magZ;                  //磁力计 

     double velDeltaTime;                    //时间，存放转换成秒的时间
     double detdistance,detth;               //计算距离和计算角度角度
     double detEncode;

     float light_info;                  //红绿灯信息（是否停车）//////////////////////////////////////////////////
     //short int traffic_light(short int);    //红绿灯识别
     
     //line control
     float line_info;                   //车道线信息
     char socket_cmd[10];                   //socket 上位机指令
     float line_kp,line_kd,line_ki,line_vel;   //车道线跟随控制
     float line_err,line_err_last;
     float line_angle_out;

     sMartcarControl carParasControl;           
  
     serial::Serial ser;                        
	 
     //msg
     sMartcarControl  moveBaseControl;          
     std_msgs::Float32  currentBattery;          
     std_msgs::Float32MultiArray test_array;
     //teb trajectory controller
     
     float v_teb_command;
     float w_teb_command;
     float max_vel;

     int teb_min_pts;
     bool follow_local_planner;
     nav_msgs::Path teb_path;
     std::vector<float> vel_trans;
     std::vector<float> vel_rot;
     float err_rot,err_rot_last,err_trans;
     nav_msgs::Odometry car_odom;
     geometry_msgs::PoseStamped pose_target;
     ros::Timer timer_teb_control;
     float controller_ang_kp,controller_ang_ki,controller_ang_kd,controller_ang_factor_vel;
     float controller_c_translation,controller_c_rotation;
     float controller_ang_out;
     float controller_vel_kp,controller_vel_ki;
     float controller_vel_out;
     //订阅话题
     ros::Subscriber sub_move_base;
     ros::Subscriber sub_light_info;  //订阅红绿灯信息
     ros::Subscriber sub_line_info;   //订阅车道线信息
     ros::Subscriber sub_socket;      //订阅MFC上位机命令
     ros::Subscriber sub_localpath;        //订阅TEB路径
     ros::Subscriber sub_nav_status;

     //发布话题
     ros::Publisher pub_odom;              //发布odom topic
     ros::Publisher pub_imu;               //发布imu topic
     ros::Publisher pub_mag;               //发布磁力计 topic
     ros::Publisher pub_battery;           //发布电池 topic
     ros::Publisher pub_targetpt;
     ros::Publisher pub_test;
     
     //坐标变换
     tf::TransformListener tf_listener;

     void pub_9250();                      //发布9250函数
     void recvCarInfoKernel();             //接收下位机发来数据函数
     void sendCarInfoKernel();             //发送小车数据到下位机函数

     void callback_move_base(const geometry_msgs::Twist::ConstPtr &msg);    //move_base回调函数
     void light_callback(const std_msgs::Float32::ConstPtr& msg);     //红绿灯回调函数
     void line_callback(const std_msgs::Float32MultiArray::ConstPtr& msg);      //车道线回调函数 
     void socket_callback(const std_msgs::String::ConstPtr& msg);     //socket回调函数
     void localpath_callback(const nav_msgs::Path::ConstPtr& msg);  
     void teb_control_callback(const ros::TimerEvent&);
     void movebase_fb_callbcack(const std_msgs::Int16::ConstPtr& msg);
};

#endif 


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"

// 接收到订阅的消息后，会进入消息回调函数
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    float t = msg->twist.twist.angular.z;
	// float m = t * 57.3;
	// 将接收到的消息打印出来
	// ROS_INFO("I heard:position x=[%f] \n y=[%f] \n z=[%f]", msg->pose.pose.position.x,msg->pose.pose.position.y ,msg->pose.pose.position.z);
    ROS_INFO("I heard: odom linear x=[%f]\n  angular z=[%f]\n  w=[%f]", msg->twist.twist.linear.x, t,msg->pose.pose.orientation.w);
}
//订阅得到小车的线速度和角速度和角度
void cmdvelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	// 将接收到的消息打印出来
	//ROS_INFO("I heard: [%s]", msg->);
	// ROS_INFO("I heard: cmd_vel linear x=[%f]\n  angular z=[%f]", msg->linear.x,msg->angular.z);
}

int main(int argc, char** argv)
{
	// 初始化ROS节点
	ros::init(argc, argv, "listener1");

	// 创建节点句柄
	ros::NodeHandle n;

	// 创建一个Subscriber，订阅名为chatter的topic，注册回调函数chatterCallback
	ros::Subscriber sub = n.subscribe("odom", 1000, odomCallback);
    ros::Subscriber sub_cmd = n.subscribe("cmd_vel", 1000, cmdvelCallback);

	// 循环等待回调函数
	ros::spin();

	return 0;
}
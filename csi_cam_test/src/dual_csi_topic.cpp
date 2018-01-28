#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <iostream>


using namespace cv;

std::string gstreamer_pipeline(int sensor_id, int sensor_mode, int capture_width, int capture_height, int display_width, int display_height, int framerate, int flip_method) {
	return "nvarguscamerasrc sensor_id=" + std::to_string(sensor_id) + " sensor_mode=" + std::to_string(sensor_mode) + " ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(capture_width) + ", height=(int)" +
		std::to_string(capture_height) + ", format=(string)NV12, framerate=(fraction)" + std::to_string(framerate) +
		"/1 ! nvvidconv flip-method=" + std::to_string(flip_method) + " ! video/x-raw, width=(int)" + std::to_string(display_width) + ", height=(int)" +
		std::to_string(display_height) + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
}

int main(int argc, char** argv) {
	int capture_width = 640*4;
	int capture_height = 480*4;
	int display_width = 640;
	int display_height = 480;
	int framerate = 10;
	int flip_method = 0;

	ros::init(argc, argv, "image_publisher");
	ros::NodeHandle nh;

  nh.param("capture_width",capture_width,capture_width);
  nh.param("capture_height",capture_height,capture_height);
  nh.param("display_width",display_width,display_width);
  nh.param("display_height",display_height,display_height);
  nh.param("framerate",framerate,framerate);
  nh.param("flip_method",flip_method,flip_method);
  printf("capture_width=[%i]",capture_width);
  printf("framerate=[%i]",framerate);

	image_transport::ImageTransport it(nh);
	
	image_transport::Publisher pub_left = it.advertise("csi_stereo/left/image_raw", 1);
	//image_transport::Publisher pub_right = it.advertise("csi_stereo/right/image_raw", 1);

	std::string pipeline_left = gstreamer_pipeline(0, 3, capture_width,
		capture_height,
		display_width,
		display_height,
		framerate,
		flip_method);
	std::cout << "Using pipeline_left: \n\t" << pipeline_left << "\n";
	//std::string pipeline_right = gstreamer_pipeline(1, 3, capture_width,
	//	capture_height,
	//	display_width,
	//	display_height,
	//	framerate,
	//	flip_method);
	//std::cout << "Using pipeline_right: \n\t" << pipeline_right << "\n";

	cv::VideoCapture cap_left(pipeline_left, cv::CAP_GSTREAMER);
	if (!cap_left.isOpened()) {
		ROS_INFO("cannot open video device0\n");
		return 1;
	}

	//cv::VideoCapture cap_right(pipeline_right, cv::CAP_GSTREAMER);

	//if (!cap_right.isOpened()) {
	//	ROS_INFO("cannot open video device1\n");
	//	return 1;
	//}

	sensor_msgs::ImagePtr msg_left;
	//sensor_msgs::ImagePtr msg_right;
	ros::Rate loop_rate(10);//hz
	cv::Mat frameL;
	//cv::Mat frameR;
	int count = 0;
	char image_left[200];
	//char image_right[200];
	while (nh.ok()) {
		if (!cap_left.read(frameL)) {
			std::cout << "Capture left read error" << std::endl;
			break;
		}
		/*if (!cap_right.read(frameR)) {
			std::cout << "Capture right read error" << std::endl;
			break;
		}*/

		ros::Time time_L = ros::Time::now();
		//ros::Time time_R = ros::Time::now();

		//std::cout << "frameR " << "frameR.cols: " << frameR.cols << "; frameR.rows: " << frameR.rows << std::endl;
		std::cout << "frameL " << "frameL.cols: " << frameL.cols << "; frameL.rows: " << frameL.rows << std::endl;
		//if (!frameR.empty()) {
		//	msg_right = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frameR).toImageMsg();
		//	msg_right->header.stamp = time_R;
		//	pub_right.publish(msg_right);
		//}
		if (!frameL.empty()) {
			msg_left = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frameL).toImageMsg();
			msg_left->header.stamp = time_L;
			pub_left.publish(msg_left);
		}

		//ROS_INFO("Publishing csi_stereo/left/image_raw csi_stereo/right/image_raw ROS topic MSG!! ");
		ROS_INFO("Publishing csi_stereo/left/image_raw ROS topic MSG!! ");
		ros::spinOnce();
		loop_rate.sleep();//��ros::Rate loop_rate���Ӧ

		int keycode = cv::waitKey(30) & 0xff;
		if (keycode == 27) break;
		if (keycode == 32) {
			count++;
			snprintf(image_left, sizeof(image_left), "./img/left%02d.jpg", count);
			cv::imwrite(image_left, frameL);
			//snprintf(image_right, sizeof(image_right), "./img/right%02d.jpg", count);
			//cv::imwrite(image_right, frameR);
		}
	}
	cap_left.release();
	//cap_right.release();
}

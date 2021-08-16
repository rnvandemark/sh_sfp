#include <iostream>
#include <string>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>

static const std::string NODE_NAME = "smart_home_cv_bridge";

class SmartHomeImageConverter
{
protected:
	std::string window_name;
	ros::NodeHandle nh;
	image_transport::ImageTransport it;
	image_transport::Subscriber image_sub;
	image_transport::Publisher image_pub;

public:
	SmartHomeImageConverter(std::string image_sub_topic, std::string image_pub_topic):
		window_name("TestImage"),
		it(nh)
	{
		image_sub = it.subscribe(
			image_sub_topic,
			1,
			&SmartHomeImageConverter::image_callback,
			this
		);
		image_pub = it.advertise(
			image_pub_topic,
			1
		);
		cv::namedWindow(window_name);
	}

	~SmartHomeImageConverter()
	{
		cv::destroyWindow(window_name);
	}

	void image_callback(const sensor_msgs::ImageConstPtr& msg)
	{
		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception& e)
		{
			std::cerr << "CvBridge exception: " << e.what() << std::endl;
			return;
		}
		cv::imshow(window_name, cv_ptr->image);
		cv::waitKey(3);
		image_pub.publish(cv_ptr->toImageMsg());
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, NODE_NAME);
	if (argc != 3)
	{
		std::cerr << "Expected exactly two args (image sub topic, bridged image pub topic), got " << (argc-1) << std::endl;
		return -1;
	}
	SmartHomeImageConverter ic(argv[1], argv[2]);
	ros::spin();
	return 0;
}

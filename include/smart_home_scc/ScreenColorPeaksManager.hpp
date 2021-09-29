#ifndef __SMART_HOME_SCC_SCREENCOLORPEAKSMANAGER_HPP
#define __SMART_HOME_SCC_SCREENCOLORPEAKSMANAGER_HPP

#include <string>
#include <queue>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <smart_home_common_msgs/Color.h>
#include <smart_home_scc_msgs/ColorPeaksTelem.h>
#include <smart_home_scc_msgs/RequestScreenCalibration.h>
#include <smart_home_scc_msgs/SetScreenCalibrationPointsOfHomography.h>

namespace smart_home {

struct ColorPeakWindow
{
protected:
	const static size_t MAX_ELEMS = 10;
	unsigned int size;
	unsigned int total_r;
	unsigned int total_g;
	unsigned int total_b;
	std::queue<unsigned char> queue_r;
	std::queue<unsigned char> queue_g;
	std::queue<unsigned char> queue_b;

public:
	void push_and_eval(unsigned char r, unsigned char g, unsigned char b, smart_home_common_msgs::ColorPtr& color);
};

class ScreenManager
{
protected:
	const static int WORLD_HEIGHT = 300;
	const static int WORLD_WIDTH = 400;
	const static std::vector<cv::Point2f> WORLD_PLANE;
	const static cv::Size WORLD_SIZE;
	const static size_t NUM_COLOR_PEAKS_TELEM_IMGS = 2;

	ros::NodeHandle nh;
	ros::ServiceServer screen_calibration_request_srv;
	ros::ServiceServer screen_calibration_set_homography_points_srv;
	ros::Subscriber cap_image_raw_sub;
	ros::Publisher color_peak_left_pub;
	ros::Publisher color_peak_right_pub;
	ros::Publisher color_peaks_telem_pub;

	smart_home_common_msgs::ColorPtr color_peak_left_msg;
	smart_home_common_msgs::ColorPtr color_peak_right_msg;
	smart_home_scc_msgs::ColorPeaksTelemPtr color_peaks_telem_msg;

	cv_bridge::CvImagePtr last_received_frame;

	bool homog_set;
	cv::Mat homog;

	ColorPeakWindow window_left;
	ColorPeakWindow window_right;

public:
	ScreenManager(
		std::string screen_calibration_request_srv_topic,
		std::string screen_calibration_set_homography_points_srv_topic,
		std::string cap_image_raw_sub_topic,
		std::string color_peak_left_pub_topic,
		std::string color_peak_right_pub_topic,
		std::string color_peaks_telem_pub_topic);

	bool screen_calibration_request_callback(
		smart_home_scc_msgs::RequestScreenCalibration::Request& req,
		smart_home_scc_msgs::RequestScreenCalibration::Response& res);

	bool screen_calibration_set_homography_points_callback(
		smart_home_scc_msgs::SetScreenCalibrationPointsOfHomography::Request& req,
		smart_home_scc_msgs::SetScreenCalibrationPointsOfHomography::Response& res);

	void cap_image_raw_callback(const sensor_msgs::ImageConstPtr& msg);
};

}

#endif // __SMART_HOME_SCC_SCREENCOLORPEAKSMANAGER_HPP

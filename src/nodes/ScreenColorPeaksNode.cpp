#include <iostream>
#include <ros/ros.h>
#include "smart_home_scc/ScreenColorPeaksManager.hpp"

static const std::string NODE_NAME = "smart_home_screen_color_peaks";

int main(int argc, char** argv) 
{ 
	ros::init(argc, argv, NODE_NAME);
	if (argc != 7)
	{
		std::cerr << "Expected exactly five args (coordinate request service topic, image sub topic, left and right peak pub topics, telemetry pub topic), got " << (argc-1) << std::endl;
		return -1;
	}
	smart_home::ScreenManager manager(argv[1], argv[2], argv[3], argv[4], argv[5], argv[6]);
	ros::spin();
	return 0;
}

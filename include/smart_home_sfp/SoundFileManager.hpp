#ifndef __SMART_HOME_SFP_SOUNDFILEMANAGER_HPP
#define __SMART_HOME_SFP_SOUNDFILEMANAGER_HPP

#include "smart_home_sfp/SoundHelpers.hpp"

#include <string>
#include <queue>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <smart_home_common_msgs/StringArr.h>
#include <smart_home_common_msgs/PlaybackCommand.h>
#include <smart_home_common_msgs/PlaybackUpdate.h>
#include <smart_home_common_msgs/Float32Arr.h>

namespace smart_home {

class SfManager
{
protected:
	ros::NodeHandle nh;
	ros::Subscriber sound_file_path_sub;
	ros::Subscriber sound_file_path_list_sub;
	ros::Subscriber fft_window_sub;
	ros::Subscriber playback_command_sub;
	ros::Publisher playback_frequencies_pub;
	ros::Publisher playback_updates_pub;
	ros::Timer calc_frequencies_tmr;
	ros::Timer send_playback_update_tmr;

	smart_home_common_msgs::Float32ArrPtr playback_frequencies_msg;
	smart_home_common_msgs::PlaybackUpdatePtr playback_updates_msg;

	const int max_num_freqs;

	std::queue<std::string> queued_sound_file_paths;
	float current_fft_window;
	float current_fft_reeval_period;
	std::string current_sf_name;

	struct SfPlaybackData; // Forward declare
	bool ready; // Whether or not the value of sfpd is valid
	SfPlaybackData* sfpd;

	void queue_sound_file(const std::string& sf_path);
	void pop_next_playback();
	int get_current_max_freqs(std::vector<float>& output);

public:
	SfManager(
		const int max_num_freqs,
		const float init_fft_window,
		const float init_fft_reeval_period,
		const std::string sound_file_path_sub_topic,
		const std::string sound_file_path_list_sub_topic,
		const std::string fft_window_sub_topic,
		const std::string playback_command_sub_topic,
		const std::string playback_frequencies_pub_topic,
		const std::string playback_updates_pub_topic
	);
	~SfManager();

	void sound_file_path_callback(const std_msgs::String& msg);
	void sound_file_path_list_callback(const smart_home_common_msgs::StringArr& msg);
	void fft_window_callback(const std_msgs::Float32& msg);
	void playback_command_callback(const smart_home_common_msgs::PlaybackCommand& msg);
	void calc_frequencies_callback(const ros::TimerEvent& evt);
	void send_playback_update_callback(const ros::TimerEvent& evt);
};

}

#endif // __SMART_HOME_SFP_SOUNDFILEMANAGER_HPP

#include "smart_home_sfp/SoundFileManager.hpp"

#include <stdlib.h>
#include <assert.h>
#include <iterator>
#include <sndfile.h>
#include <SFML/Audio.hpp>
#include <ros/package.h>

namespace smart_home {

#define INST_CALC_FREQ_TMR() \
	nh.createTimer( \
		ros::Duration(current_fft_reeval_period), \
		&smart_home::SfManager::calc_frequencies_callback, \
		this, \
		false, \
		false \
	)

#define PLAYBACK_PREPARE() \
	ready = true; \
	calc_frequencies_tmr.start()

#define PLAYBACK_PLAY() sfpd->sound.play()

#define PLAYBACK_RESUME() \
	PLAYBACK_PLAY(); \
	calc_frequencies_tmr.start()

#define PLAYBACK_PAUSE() \
	sfpd->sound.pause(); \
	calc_frequencies_tmr.stop()

#define PLAYBACK_STOP() \
	sfpd->sound.stop(); \
	calc_frequencies_tmr.stop(); \
	sfpd->clear(); \
	ready = false

#define PLAYBACK_CONTINUE() \
	PLAYBACK_STOP(); \
	pop_next_playback()

struct SfManager::SfPlaybackData
{
	std::string path;
	uint64_t num_frames;
	int num_channels;
	int sample_rate;

	std::vector<double> pending_playback_frames;

	sf::SoundBuffer sound_buffer;
	sf::Sound sound;

	SfPlaybackData()
	{
		clear();
	}

	void clear()
	{
		path = "";
		num_frames = -1;
		num_channels = -1;
		sample_rate = -1;
		pending_playback_frames.clear();
	}

	uint8_t getPlaybackStatus()
	{
		switch (sound.getStatus())
		{
			case sf::SoundSource::Status::Stopped:
				return smart_home_common_msgs::PlaybackUpdate::STOPPED;
			case sf::SoundSource::Status::Playing:
				return smart_home_common_msgs::PlaybackUpdate::PLAYING;
			case sf::SoundSource::Status::Paused:
				return smart_home_common_msgs::PlaybackUpdate::PAUSED;
			default:
				return smart_home_common_msgs::PlaybackUpdate::UNKNOWN;
		}
	}

	static bool isActive(uint8_t playbackStatus)
	{
		switch (playbackStatus)
		{
			case smart_home_common_msgs::PlaybackUpdate::PLAYING:
			case smart_home_common_msgs::PlaybackUpdate::PAUSED:
				return true;
			default:
				return false;
		}
	}

	bool isActive()
	{
		return isActive(getPlaybackStatus());
	}

	ros::Duration duration_total()
	{
		return ros::Duration(static_cast<double>(num_frames) / sample_rate);
	}

	ros::Duration duration_current()
	{
		return ros::Duration(sound.getPlayingOffset().asSeconds());
	}
};

void SfManager::queue_sound_file(const std::string& sf_path)
{
	queued_sound_file_paths.push(sf_path);
	if (!ready)
	{
		pop_next_playback();
	}
}

void SfManager::pop_next_playback()
{
	// Ignore if no next sound file is queued
	if (queued_sound_file_paths.empty())
	{
		return;
	}

	// Get full file path
	const std::string new_path = queued_sound_file_paths.front();
	queued_sound_file_paths.pop();
	sfpd->path = new_path;

#define STR_ENDS_WITH(s,sx) (s.size() >= strlen(sx) && 0 == s.compare(s.size()-strlen(sx), strlen(sx), sx))

	// Create temporary file from MP3 to WAV, overwrite sfpd->path with temporary file
	if (STR_ENDS_WITH(sfpd->path, ".mp3"))
	{
		const size_t N = 256;
		const std::string command = std::string("bash \"") + ros::package::getPath("smart_home_sfp") + "/bin/mp3_2_wav.sh\" \"" + sfpd->path + "\"";
		std::array<char,N> buffer;
		std::string result;

		FILE* pipe = popen(command.c_str(), "r");
		if (0 == pipe)
		{
			ROS_ERROR("Failed to start MP3 to WAV conversion script.");
			return;
		}

		// Capture output to get the tmp mp3 file
		while (0 != fgets(buffer.data(), N, pipe))
		{
			result += buffer.data();
		}
		result = result.substr(0, result.size()-1);

		// Command succeeded
		if (0 == pclose(pipe))
		{
			ROS_INFO("Created copy of '%s' to '%s'.", sfpd->path.c_str(), result.c_str());
			sfpd->path = result;
		}
		// Command failed
		else
		{
			ROS_ERROR("MP3 to WAV conversion script for %s failed (%s).", sfpd->path.c_str(), result.c_str());
			return;
		}
	}
	// Except for WAV files, nothing else is valid
	else if (!STR_ENDS_WITH(sfpd->path, ".wav"))
	{
		ROS_ERROR("Invalid file extension for '%s'.", sfpd->path.c_str());
		return;
	}

#undef STR_ENDS_WITH

	// Load file into file path
	if (!sfpd->sound_buffer.loadFromFile(sfpd->path))
	{
		ROS_ERROR("Failed to load sound buffer from file '%s'.", sfpd->path.c_str());
		return;
	}
	sfpd->sound.setBuffer(sfpd->sound_buffer);

	// Get the human-friendly name of the sound file
	current_sf_name = new_path;
	const std::size_t directory = current_sf_name.rfind("/");
	if (std::string::npos != directory)
	{
		current_sf_name = current_sf_name.substr(directory+1);
	}
	const std::size_t extension = current_sf_name.rfind(".");
	if (std::string::npos != extension)
	{
		current_sf_name = current_sf_name.substr(0, extension);
	}

	// Read the file into buffer
	SF_INFO sfinfo;
	SNDFILE* file_in = sf_open(sfpd->path.c_str(), SFM_READ, &sfinfo);

	// Capture file parameters
	sfpd->num_frames = static_cast<uint64_t>(sfinfo.frames);
	sfpd->num_channels = static_cast<int>(sfinfo.channels);
	sfpd->sample_rate = static_cast<int>(sfinfo.samplerate);
	const uint64_t num_all_items = sfpd->num_frames * sfpd->num_channels;

	// Read sound file into buffer
	sfpd->pending_playback_frames.resize(num_all_items);
	const uint64_t items_read = sf_read_double(file_in, &sfpd->pending_playback_frames[0], num_all_items);
	sf_close(file_in);

	ROS_DEBUG(
		"frames=%lu, channels=%d, sample_rate=%d, num_all_items=%lu, items_read=%lu",
		sfpd->num_frames,
		sfpd->num_channels,
		sfpd->sample_rate,
		num_all_items,
		items_read
	);

	ROS_INFO("Finished loading playback data for '%s'.", sfpd->path.c_str());
	PLAYBACK_PREPARE();
}

int SfManager::get_current_max_freqs(std::vector<float>& output)
{
	assert(ready);
	const uint64_t total_items = sfpd->sample_rate * sfpd->num_channels;
	const uint64_t current_frame = total_items * sfpd->duration_current().toSec();
	const uint64_t window_frames = total_items * current_fft_window;
	const std::vector<double>::iterator start = sfpd->pending_playback_frames.begin() + current_frame;
	const std::vector<double>::iterator end = (current_frame + window_frames >= sfpd->pending_playback_frames.size()) ? sfpd->pending_playback_frames.end() : start + window_frames;
	const std::vector<double> current_chunk(start, end);
	output = get_max_fft_freqs(
		do_1d_dft(current_chunk),
		sfpd->sample_rate,
		max_num_freqs
	);
	const uint64_t reeval_size = total_items * current_fft_reeval_period;
	return (current_frame <= reeval_size) ? 1 : (
		(std::distance(start, sfpd->pending_playback_frames.end()) <= reeval_size) ? 2 : 0
	);
}

SfManager::SfManager(
		const int max_num_freqs,
		const float init_fft_window,
		const float init_fft_reeval_period,
		const std::string sound_file_path_sub_topic,
		const std::string sound_file_path_list_sub_topic,
		const std::string fft_window_sub_topic,
		const std::string playback_command_sub_topic,
		const std::string playback_frequencies_pub_topic,
		const std::string playback_updates_pub_topic
) :
		max_num_freqs(max_num_freqs),
		current_fft_window(init_fft_window),
		current_fft_reeval_period(init_fft_reeval_period),
		ready(false),
		sfpd(new SfPlaybackData),
		playback_frequencies_msg(new smart_home_common_msgs::Float32Arr),
		playback_updates_msg(new smart_home_common_msgs::PlaybackUpdate)
{
	assert(max_num_freqs > 0);
	sound_file_path_sub = nh.subscribe(
		sound_file_path_sub_topic,
		1,
		&smart_home::SfManager::sound_file_path_callback,
		this
	);
	sound_file_path_list_sub = nh.subscribe(
		sound_file_path_list_sub_topic,
		1,
		&smart_home::SfManager::sound_file_path_list_callback,
		this
	);
	fft_window_sub = nh.subscribe(
		fft_window_sub_topic,
		1,
		&smart_home::SfManager::fft_window_callback,
		this
	);
	playback_command_sub = nh.subscribe(
		playback_command_sub_topic,
		1,
		&smart_home::SfManager::playback_command_callback,
		this
	);
	playback_frequencies_pub = nh.advertise<smart_home_common_msgs::Float32Arr>(
		playback_frequencies_pub_topic,
		10
	);
	playback_updates_pub = nh.advertise<smart_home_common_msgs::PlaybackUpdate>(
		playback_updates_pub_topic,
		10
	);
	calc_frequencies_tmr = INST_CALC_FREQ_TMR();

	send_playback_update_tmr = nh.createTimer(
		ros::Duration(0.2),
		&smart_home::SfManager::send_playback_update_callback,
		this,
		false,
		true
	);
	send_playback_update_tmr.start();

	ROS_INFO("Initialized SoundFileAnalyzer.");
}
SfManager::~SfManager()
{
	delete sfpd;
}

void SfManager::sound_file_path_callback(const std_msgs::String& msg)
{
	queue_sound_file(msg.data);
}
void SfManager::sound_file_path_list_callback(const smart_home_common_msgs::StringArr& msg)
{
	for (std::vector<std::string>::const_iterator iter = msg.data.cbegin(); iter != msg.data.cend(); ++iter)
	{
		queue_sound_file(*iter);
	}
}
void SfManager::fft_window_callback(const std_msgs::Float32& msg)
{
	// Ignore if currently processing an audio file
	if (!ready)
	{
		current_fft_window = msg.data;
		calc_frequencies_tmr = INST_CALC_FREQ_TMR();
	}
}
void SfManager::playback_command_callback(const smart_home_common_msgs::PlaybackCommand& msg)
{
	switch (msg.cmd)
	{
		case smart_home_common_msgs::PlaybackCommand::STOP:
			PLAYBACK_STOP();
			break;
		case smart_home_common_msgs::PlaybackCommand::PAUSE:
			PLAYBACK_PAUSE();
			break;
		case smart_home_common_msgs::PlaybackCommand::RESUME:
			PLAYBACK_RESUME();
			break;
		case smart_home_common_msgs::PlaybackCommand::SKIP:
			PLAYBACK_CONTINUE();
			break;
	}
}
void SfManager::calc_frequencies_callback(const ros::TimerEvent& evt)
{
	if (!ready)
	{
		// Catch the case where a STOP command, etc. was requested the same spin cycle as a timer tick
		return;
	}

	const double start = ros::Time::now().toSec();
	const int status = get_current_max_freqs(playback_frequencies_msg->data);
	playback_frequencies_pub.publish(playback_frequencies_msg);

	switch (status)
	{
		case 1:
			PLAYBACK_PLAY();
			break;
		case 2:
			PLAYBACK_CONTINUE();
			break;
	}

	ROS_DEBUG(
		"Callback is %f seconds behind, took %f seconds.",
		evt.current_real.toSec() - evt.current_expected.toSec(),
		ros::Time::now().toSec() - start
	);
}

void SfManager::send_playback_update_callback(const ros::TimerEvent& evt)
{
	playback_updates_msg->status = sfpd->getPlaybackStatus();
	if (sfpd->isActive(playback_updates_msg->status))
	{
		playback_updates_msg->name = current_sf_name;
		playback_updates_msg->duration_total = sfpd->duration_total();
		playback_updates_msg->duration_current = sfpd->duration_current();
	}
	else
	{
		playback_updates_msg->name = "";
		playback_updates_msg->duration_total = ros::Duration(0);
		playback_updates_msg->duration_current = ros::Duration(0);
	}
	playback_updates_pub.publish(playback_updates_msg);
}

#undef INST_CALC_FREQ_TMR

}

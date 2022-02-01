#include "sh_sfp/sound_file_playback.hpp"

#include <sh_common/ros_names.hpp>

#include <SFML/Audio.hpp>

#include <filesystem>
#include <thread>

namespace sh {

SoundFilePlayback::SoundFilePlayback() :
        HeartbeatNode("sound_file_playback"),
        update_rate_hz(60),
        playback_active(false),
        pending_command(sh_sfp_interfaces::srv::RequestPlaybackCommand::Request::NONE)
{
    play_sound_file_asr = rclcpp_action::create_server<sh_sfp_interfaces::action::PlaySoundFile>(
        get_node_base_interface(),
        get_node_clock_interface(),
        get_node_logging_interface(),
        get_node_waitables_interface(),
        sh::names::actions::REQUEST_PLAY_SOUND_FILE,
        std::bind(
            &sh::SoundFilePlayback::handle_goal,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        ),
        std::bind(
            &sh::SoundFilePlayback::handle_cancel,
            this,
            std::placeholders::_1
        ),
        std::bind(
            &sh::SoundFilePlayback::handle_accepted,
            this,
            std::placeholders::_1
        )
    );
    request_playback_command_scl = create_service<sh_sfp_interfaces::srv::RequestPlaybackCommand>(
        sh::names::services::PLAYBACK_COMMANDS,
        std::bind(
            &sh::SoundFilePlayback::handle_playback_command_requests,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );
    labeled_audio_characteristics_pub = create_publisher<sh_sfp_interfaces::msg::LabeledAudioCharacteristics>(
        sh::names::topics::PLAYBACK_AUDIO_CHARACTERISTICS,
        10
    );
    playback_update_verbose_pub = create_publisher<sh_sfp_interfaces::msg::PlaybackUpdate>(
        sh::names::topics::PLAYBACK_UPDATES_VERBOSE,
        1
    );

    RCLCPP_INFO(get_logger(), "Initialized sound file playback.");
}
SoundFilePlayback::~SoundFilePlayback()
{
}

rclcpp_action::GoalResponse SoundFilePlayback::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const sh_sfp_interfaces::action::PlaySoundFile::Goal> goal)
{
    // Process request
    rclcpp_action::GoalResponse rv = rclcpp_action::GoalResponse::REJECT;
    const std::string url = goal->labeled_audio_characteristics.local_url;

    // Deny request if the file can't be found
    if (!std::filesystem::exists(url))
    {
        RCLCPP_ERROR(get_logger(), "Requested sound file '%s' does not exist.", url.c_str());
        goto END;
    }

    // Otherwise, accept
    rv = rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    RCLCPP_INFO(get_logger(), "Accepted request to play sound file at '%s'.", url.c_str());

END:
    return rv;
}

rclcpp_action::CancelResponse SoundFilePlayback::handle_cancel(const PlaySoundFileGoalHandleSharedPtr goal_handle)
{
    (void)goal_handle;
    RCLCPP_INFO(get_logger(), "Received request to cancel playing sound file.");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void SoundFilePlayback::handle_accepted(const PlaySoundFileGoalHandleSharedPtr goal_handle)
{
    std::thread {
        std::bind(&sh::SoundFilePlayback::execute, this, std::placeholders::_1),
        goal_handle
    }.detach();
}

void SoundFilePlayback::execute(const PlaySoundFileGoalHandleSharedPtr goal_handle)
{
    // Immediately publish the labeled audio characteristics
    const auto labeled_audio_characteristics = goal_handle->get_goal()->labeled_audio_characteristics;
    labeled_audio_characteristics_pub->publish(labeled_audio_characteristics);

    // Declare action feedback and result
    auto playback_feedback = std::make_shared<sh_sfp_interfaces::action::PlaySoundFile::Feedback>();
    auto playback_result = std::make_shared<sh_sfp_interfaces::action::PlaySoundFile::Result>();
    playback_result->was_stopped = false;

    // We expect for the specified local file URL to be a WAV file
    const std::string wav_path = labeled_audio_characteristics.local_url;

    // Load music file from file path
    sf::Music sound;
    if (!sound.openFromFile(wav_path))
    {
        RCLCPP_ERROR(get_logger(), "Failed to load sound buffer from file '%s'.", wav_path.c_str());
        if (rclcpp::ok())
        {
            goal_handle->abort(playback_result);
        }
        return;
    }

    // Lock the mutex and initialize
    {
        const std::lock_guard<std::mutex> guard(playback_mutex);
        playback_active = true;
        pending_command = sh_sfp_interfaces::srv::RequestPlaybackCommand::Request::NONE;
    }

    // Play the sound file
    sound.play();
    RCLCPP_INFO(get_logger(), "Playing sound file at '%s'.", wav_path.c_str());

    // Loop until the sound playback is finished
    sh_sfp_interfaces::msg::PlaybackUpdate playback_update;
    rclcpp::Time last_sparse_update(0, 0, get_clock()->get_clock_type());
    rclcpp::Rate loop_rate(update_rate_hz);
    bool sf_finished = false;
    while (rclcpp::ok() && !sf_finished)
    {
        // Lock the mutex, then do everything else
        {
            const std::lock_guard<std::mutex> guard(playback_mutex);

            // Handle any pending commands
            switch (pending_command)
            {
                case sh_sfp_interfaces::srv::RequestPlaybackCommand::Request::STOP:
                    playback_result->was_stopped = true;
                case sh_sfp_interfaces::srv::RequestPlaybackCommand::Request::SKIP:
                    sound.stop();
                    pending_command = sh_sfp_interfaces::srv::RequestPlaybackCommand::Request::NONE;
                    break;
                case sh_sfp_interfaces::srv::RequestPlaybackCommand::Request::PAUSE:
                    sound.pause();
                    pending_command = sh_sfp_interfaces::srv::RequestPlaybackCommand::Request::NONE;
                    break;
                case sh_sfp_interfaces::srv::RequestPlaybackCommand::Request::RESUME:
                    sound.play();
                    pending_command = sh_sfp_interfaces::srv::RequestPlaybackCommand::Request::NONE;
                    break;
                default:
                    break;
            }

            // Calculate whether or not this sound file's playback is 'finished'
            sf_finished = (sound.getStatus() == sf::SoundSource::Status::Stopped);

            // Populate the playback update message
            playback_update.duration_current = rclcpp::Duration::from_seconds(sound.getPlayingOffset().asSeconds());
            playback_update.duration_total = rclcpp::Duration::from_seconds(sound.getDuration().asSeconds());
            playback_update.is_paused = (sound.getStatus() == sf::SoundSource::Status::Paused);

            // Publish the verbose output
            playback_update_verbose_pub->publish(playback_update);

            // Publish the sparse output if it's time to do so
            const rclcpp::Time time_now = now();
            if ((time_now - last_sparse_update).seconds() >= (1.0-(1/update_rate_hz)))
            {
                // Send the action's feedback
                playback_feedback->update = playback_update;
                goal_handle->publish_feedback(playback_feedback);
                last_sparse_update = time_now;
            }
        }

        // If we're not done, sleep for a little bit
        if (!sf_finished)
        {
            loop_rate.sleep();
        }
    }

    // Lock the mutex and clear all flags, etc.
    {
        const std::lock_guard<std::mutex> guard(playback_mutex);
        playback_active = false;
        pending_command = sh_sfp_interfaces::srv::RequestPlaybackCommand::Request::NONE;
    }

    // Send the action's result (if ROS is still running)
    if (rclcpp::ok())
    {
        goal_handle->succeed(playback_result);
        RCLCPP_INFO(get_logger(), "Finished playing sound file at '%s'.", wav_path.c_str());
    }
}

void SoundFilePlayback::handle_playback_command_requests(
    const sh_sfp_interfaces::srv::RequestPlaybackCommand::Request::SharedPtr req,
    sh_sfp_interfaces::srv::RequestPlaybackCommand::Response::SharedPtr res)
{
    bool rc = true;
    const unsigned char cmd = req->cmd;
    switch (cmd)
    {
        case sh_sfp_interfaces::srv::RequestPlaybackCommand::Request::NONE:
            RCLCPP_WARN(get_logger(), "Received an empty command.");
            break;
        case sh_sfp_interfaces::srv::RequestPlaybackCommand::Request::STOP:
        case sh_sfp_interfaces::srv::RequestPlaybackCommand::Request::PAUSE:
        case sh_sfp_interfaces::srv::RequestPlaybackCommand::Request::RESUME:
        case sh_sfp_interfaces::srv::RequestPlaybackCommand::Request::SKIP:
        {
            const std::lock_guard<std::mutex> guard(playback_mutex);
            pending_command = cmd;
            RCLCPP_INFO(get_logger(), "Queued a command of type %u.", cmd);
            break;
        }
        default:
            RCLCPP_ERROR(get_logger(), "Received an unknown command type: %u", cmd);
            rc = false;
    }
    res->success = rc;
}

}

int main(int argc, char** argv) 
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<sh::SoundFilePlayback>();
    executor.add_node(node);
    executor.spin();
    return 0;
}

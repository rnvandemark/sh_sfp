#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sh_common/heartbeat_node.hpp>
#include <sh_common_interfaces/msg/float32_arr.hpp>
#include <sh_sfp_interfaces/action/play_sound_file.hpp>
#include <sh_sfp_interfaces/srv/request_playback_command.hpp>
#include <sh_sfp_interfaces/msg/begin_playback.hpp>
#include <sh_sfp_interfaces/msg/playback_update.hpp>

#include <mutex>

namespace sh {

class SoundFilePlayback : public HeartbeatNode
{
protected:
    using PlaySoundFileGoalHandle = rclcpp_action::ServerGoalHandle<sh_sfp_interfaces::action::PlaySoundFile>;
    using PlaySoundFileGoalHandleSharedPtr = std::shared_ptr<PlaySoundFileGoalHandle>;

    rclcpp_action::Server<sh_sfp_interfaces::action::PlaySoundFile>::SharedPtr play_sound_file_asr;
    rclcpp::Service<sh_sfp_interfaces::srv::RequestPlaybackCommand>::SharedPtr request_playback_command_scl;
    rclcpp::Publisher<sh_sfp_interfaces::msg::BeginPlayback>::SharedPtr begin_playback_pub;
    rclcpp::Publisher<sh_sfp_interfaces::msg::PlaybackUpdate>::SharedPtr playback_update_verbose_pub;

    double update_rate_hz;

    bool playback_active;
    unsigned char pending_command;

    std::mutex playback_mutex;

public:
    SoundFilePlayback();
    ~SoundFilePlayback();

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const sh_sfp_interfaces::action::PlaySoundFile::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(
        const PlaySoundFileGoalHandleSharedPtr goal_handle);

    void handle_accepted(const PlaySoundFileGoalHandleSharedPtr goal_handle);

    void execute(const PlaySoundFileGoalHandleSharedPtr goal_handle);

    void handle_playback_command_requests(
        const sh_sfp_interfaces::srv::RequestPlaybackCommand::Request::SharedPtr req,
        sh_sfp_interfaces::srv::RequestPlaybackCommand::Response::SharedPtr res);
};

}

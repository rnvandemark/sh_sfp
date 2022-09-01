#include "sh_sfp/audio_visualizer_ascii.hpp"

#include <sh_common/ros_names.hpp>

#include <ncurses.h>

#define BOX_ROWS_PER_ACTOR 3
#define BOX_WIDTH 100
#define BOX_BEGIN_Y 1
#define BOX_BEGIN_X 1

#define DRAW_CHAR_HIGH 'X'
#define DRAW_CHAR_LOW ' '

namespace sh {

class AsciiVizImpl
{
protected:
    WINDOW* const window;
    bool heartbeat_high;

public:
    const int num_actors;

    AsciiVizImpl(const int num_actors) :
        num_actors(num_actors),
        window(newwin(3 + (BOX_ROWS_PER_ACTOR * num_actors),
                      2 + BOX_WIDTH,
                      BOX_BEGIN_Y,
                      BOX_BEGIN_X)),
        heartbeat_high(false)
    {
        box(window, '|', '-');
        draw_heartbeat(false);
    }

    void refresh_window()
    {
        wrefresh(window);
    }

    bool draw_heartbeat(const bool new_heartbeat_high)
    {
        heartbeat_high = new_heartbeat_high;
        return (ERR != mvwaddstr(window, 1, 1, "<3: "))
            && (ERR != mvwaddstr(window, 1, 5, heartbeat_high ? "+++" : "---"))
        ;
    }
    bool toggle_heartbeat()
    {
        return draw_heartbeat(!heartbeat_high);
    }

    bool draw_actor(const int index, const float sample)
    {
        // Calculate the appearance
        const int bar_length = ((sample > 0.0f) ? static_cast<int>(BOX_WIDTH * sample) : 0);
        std::string bar(BOX_WIDTH, DRAW_CHAR_LOW);
        bar.replace(bar.cbegin(), bar.cbegin() + bar_length, bar_length, DRAW_CHAR_HIGH);
        const char* text = bar.c_str();

        // Draw it
        bool rc = true;
        for (int i = 0; rc && (i < BOX_ROWS_PER_ACTOR); i++)
        {
            rc = (ERR != mvwaddstr(window, 2 + (index * BOX_ROWS_PER_ACTOR) + i, 1, text));
        }

        return rc;
    }
};

AudioVisualizerAscii::AudioVisualizerAscii(AsciiVizImplSharedPtr& impl) :
    AudioVisualizerBarGraphI("audio_visualizer_ascii"),
    impl(impl)
{
    heartbeat_sub = create_subscription<std_msgs::msg::Header>(
        sh::names::topics::HEARTBEAT_PREFIX + "/" + get_name(),
        1,
        std::bind(
            &sh::AudioVisualizerAscii::handle_heartbeat,
            this,
            std::placeholders::_1
        )
    );

    clear_visuals();
}

bool AudioVisualizerAscii::update_actors(const std::vector<float>& sample_data)
{
    bool rc = true;
    for (int i = 0; rc && (i < sample_data.size()); i++)
    {
        rc = impl->draw_actor(i, sample_data[i]);
    }
    return rc;
}

void AudioVisualizerAscii::clear_visuals()
{
    if (!update_actors(std::vector<float>(impl->num_actors, 0.0f)))
    {
        RCLCPP_ERROR(get_logger(), "Failed to clear visuals!");
    }
}

void AudioVisualizerAscii::handle_heartbeat(const std_msgs::msg::Header::SharedPtr msg)
{
    if (!impl->toggle_heartbeat())
    {
        RCLCPP_ERROR(get_logger(), "Failed to toggle heartbeat!");
    }
}
}

namespace {
    // This is only nonblocking due to the call to nodelay() in main()
    bool getch_nonblocking(int& ch)
    {
        ch = getch();
        return (ch != ERR);
    }
}

int main(int argc, char** argv) 
{
    // Enable curses mode before anything ROS
    initscr();
    raw();
    noecho();
    nodelay(stdscr, TRUE);
//    use_default_colors();
    start_color();
//    init_pair(1, COLOR_CYAN, COLOR_BLACK);

    // Now initialize ROS
    rclcpp::init(argc, argv);

    // TODO: parameterize
    const int num_actors = 7;

    // Scope this routine to easily have everything else destroyed before
    // trying to end curses mode
    {
        // We'll run at ~60Hz. TODO: parameterize?
        using namespace std::chrono_literals;
        const std::chrono::nanoseconds sleep_period(1s/60);

        auto impl = std::make_shared<sh::AsciiVizImpl>(num_actors);
        auto node = std::make_shared<sh::AudioVisualizerAscii>(impl);

        int ch;
        while (rclcpp::ok())
        {
            // Give the node a chance to spin
            rclcpp::spin_some(node);

            // React to user input, if any
            if (getch_nonblocking(ch))
            {
                // Was the ESCAPE key pressed?
                if (27 == ch)
                {
                    // Shut down if the ESCAPE key was pressed, which will
                    // break this loop
                    rclcpp::shutdown();
                }
            }

            // Update the window and then briefly sleep
            impl->refresh_window();
            rclcpp::sleep_for(sleep_period);
        }
    }

    // Disable curses mode before we exit
    endwin();
    return 0;
}

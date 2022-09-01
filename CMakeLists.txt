cmake_minimum_required(VERSION 3.5)
project(sh_sfp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sh_common REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(sh_common_interfaces REQUIRED)
find_package(sh_sfp_interfaces REQUIRED)

include_directories(include)

add_library(audio_translator_i SHARED src/audio_translator_i.cpp)
ament_target_dependencies(audio_translator_i
    rclcpp
    rclcpp_action
    sh_common
    sh_common_interfaces
    sh_sfp_interfaces
)

add_executable(audio_translator_bar_graph src/audio_translator_bar_graph.cpp)
target_link_libraries(audio_translator_bar_graph audio_translator_i)

add_library(audio_visualizer_i SHARED src/audio_visualizer_i.cpp)
ament_target_dependencies(audio_visualizer_i
    rclcpp
    sh_common
    sh_sfp_interfaces
)

add_library(audio_visualizer_bar_graph_i SHARED src/audio_visualizer_bar_graph_i.cpp)
target_link_libraries(audio_visualizer_bar_graph_i audio_visualizer_i)

add_executable(audio_visualizer_ascii src/audio_visualizer_ascii.cpp)
target_link_libraries(audio_visualizer_ascii audio_visualizer_bar_graph_i ncurses)

add_executable(sound_file_playback src/sound_file_playback.cpp)
ament_target_dependencies(sound_file_playback
    rclcpp
    rclcpp_action
    sh_common
    sensor_msgs
    sh_common_interfaces
    sh_sfp_interfaces
)
target_link_libraries(sound_file_playback sfml-audio sfml-system)

install(
    DIRECTORY include/
    DESTINATION include
)

install(
    TARGETS audio_translator_i audio_translator_bar_graph audio_visualizer_i audio_visualizer_bar_graph_i audio_visualizer_ascii sound_file_playback
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    INCLUDES DESTINATION include
)

install(
    PROGRAMS scripts/audio_downloader.py scripts/audio_analyzer.py
    DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies(
    ament_cmake
    rclcpp
    rclcpp_action
    sh_common
    sensor_msgs
    sh_common_interfaces
    sh_sfp_interfaces
)
ament_export_libraries(audio_translator_i audio_visualizer_i)

ament_package()

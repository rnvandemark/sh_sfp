cmake_minimum_required(VERSION 3.5)
project(sh_sfp)

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sh_common REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(sh_common_interfaces REQUIRED)
find_package(sh_sfp_interfaces REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)

add_executable(sound_file_playback src/sound_file_playback.cpp)
ament_target_dependencies(sound_file_playback
    rclcpp
    rclcpp_action
    cv_bridge
    sh_common
    sensor_msgs
    sh_common_interfaces
    sh_sfp_interfaces
    OpenCV
)
target_link_libraries(sound_file_playback sfml-audio sfml-system)

install(
    DIRECTORY include/
    DESTINATION include
)

install(
    TARGETS sound_file_playback
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    INCLUDES DESTINATION include
)

ament_python_install_package(${PROJECT_NAME})
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

ament_package()

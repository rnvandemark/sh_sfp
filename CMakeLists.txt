cmake_minimum_required(VERSION 3.5)
project(sh_sfp)

find_package(rclcpp REQUIRED)
#find_package(std_msgs REQUIRED)
#find_package(sh_common_interfaces REQUIRED)
#find_package(sh_sfp_interfaces REQUIRED)

include_directories(include)

#add_executable(SfAnalyzer src/nodes/SoundFileHandlerNode.cpp src/lib/SoundHelpers.cpp src/lib/SoundFileManager.cpp)
#ament_target_dependencies(SfAnalyzer
#    rclcpp
#    cv_bridge
#    sh_common
#    image_transport
#    sensor_msgs
#    sh_common_interfaces
#    sh_scc_interfaces
#    OpenCV
#)

install(
    DIRECTORY include/
    DESTINATION include
)

#install(
#    TARGETS SfAnalyzer
#    RUNTIME DESTINATION lib/${PROJECT_NAME}
#    INCLUDES DESTINATION include
#)

ament_python_install_package(${PROJECT_NAME})
install(
    PROGRAMS scripts/audio_downloader.py
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

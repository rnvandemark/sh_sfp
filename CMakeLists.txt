cmake_minimum_required(VERSION 3.0.2)
project(smart_home_sfp)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	roslib
	std_msgs
	smart_home_common_msgs
	smart_home_sfp_msgs
)

catkin_package(
	CATKIN_DEPENDS roscpp roslib std_msgs smart_home_common_msgs smart_home_sfp_msgs
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

add_executable(SfAnalyzer src/nodes/SoundFileHandlerNode.cpp src/lib/SoundHelpers.cpp src/lib/SoundFileManager.cpp)
target_link_libraries(SfAnalyzer m fftw3 sndfile sfml-audio sfml-system ${catkin_LIBRARIES})

install(TARGETS
	SfAnalyzer
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(
  DIRECTORY bin/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/bin
)

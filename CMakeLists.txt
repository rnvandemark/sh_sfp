cmake_minimum_required(VERSION 3.0.2)
project(smart_home_scc)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	sensor_msgs
	cv_bridge
	image_transport
	smart_home_msgs
)
find_package(OpenCV)

catkin_package(
	CATKIN_DEPENDS roscpp sensor_msgs cv_bridge image_transport smart_home_msgs std_msgs
	DEPENDS OpenCV
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)

add_executable(CvBridge src/nodes/CvBridge.cpp)
target_link_libraries(CvBridge ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(ScreenColorPeaksNode src/nodes/ScreenColorPeaksNode.cpp src/lib/ScreenColorPeaksManager.cpp)
target_link_libraries(ScreenColorPeaksNode ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

install(TARGETS
	CvBridge
	ScreenColorPeaksNode
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

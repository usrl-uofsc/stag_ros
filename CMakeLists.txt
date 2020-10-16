cmake_minimum_required(VERSION 2.8.3)
project(stag_ros)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)
link_libraries(${JSONCPP_LIBRARIES})

################
# Dependencies #
################
find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  cv_bridge
  geometry_msgs
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  swri_nodelet
  tf
)

catkin_package(
  INCLUDE_DIRS include src/stag
  LIBRARIES stag_ros
  CATKIN_DEPENDS camera_info_manager cv_bridge geometry_msgs image_transport roscpp sensor_msgs std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

###########
## Build ##
###########

## Runnables
add_library(stag_core
  src/stag/Decoder.cpp
  src/stag/Drawer.cpp
  src/stag/EDInterface.cpp
  src/stag/Ellipse.cpp
  src/stag/Marker.cpp
  src/stag/PoseRefiner.cpp
  src/stag/Quad.cpp
  src/stag/QuadDetector.cpp
  src/stag/Stag.cpp
  src/stag/utility.cpp
  src/stag/ED/ED.cpp
  src/stag/ED/EDInternals.cpp
  src/stag/ED/EDLines.cpp
  src/stag/ED/GradientOperators.cpp
  src/stag/ED/ImageSmooth.cpp
  src/stag/ED/LineSegment.cpp
  src/stag/ED/MyMath.cpp
  src/stag/ED/NFA.cpp
#  src/stag/ED/Utilities.cpp
  src/stag/ED/ValidateEdgeSegments.cpp
)

add_library(stag_nodelet
        src/stag_ros/stag_nodelet.cpp
)

add_executable(stag_node
        src/stag_ros/stag_node.cpp
)

## Dependencies
add_dependencies(stag_nodelet
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
)
add_dependencies(stag_node
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
)

## Links
target_link_libraries(stag_core
  ${OpenCV_LIBS}
)

message(STATUS "JSON lib : ${JSONCPP_LIBRARIES}")

target_link_libraries(stag_nodelet
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${JSONCPP_LIBRARIES}
  stag_core
)
swri_nodelet_add_node(stag_nodelet_runnable stag_ros StagNodelet)
target_link_libraries(stag_nodelet_runnable stag_nodelet)

target_link_libraries(stag_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  stag_core
)

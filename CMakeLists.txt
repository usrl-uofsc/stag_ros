cmake_minimum_required(VERSION 2.8.3)
project(stag_ros)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(OpenCV REQUIRED)

################
# Dependencies #
################
find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  cv_bridge
  geometry_msgs
  image_transport
  message_generation
  roscpp
  sensor_msgs
  std_msgs
  swri_nodelet
  tf
)

## Generate messages in the 'msg' folder
add_message_files(DIRECTORY msg
  FILES
  StagMarker.msg
  StagMarkers.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES stag_ros
  CATKIN_DEPENDS camera_info_manager cv_bridge geometry_msgs image_transport message_generation roscpp sensor_msgs std_msgs
  DEPENDS system_lib
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
add_library(stag
  src/Decoder.cpp
  src/Drawer.cpp
  src/EDInterface.cpp
  src/Ellipse.cpp
  src/Marker.cpp
  src/PoseRefiner.cpp
  src/Quad.cpp
  src/QuadDetector.cpp
  src/Stag.cpp
  src/utility.cpp
  src/ED/ED.cpp
  src/ED/EDInternals.cpp
  src/ED/EDLines.cpp
  src/ED/GradientOperators.cpp
  src/ED/ImageSmooth.cpp
  src/ED/LineSegment.cpp
  src/ED/MyMath.cpp
  src/ED/NFA.cpp
  src/ED/Utilities.cpp
  src/ED/ValidateEdgeSegments.cpp
)

add_library(stag_nodelet
  src/stag_nodelet.cpp
)


add_executable(stag_node
  src/stag_node.cpp
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
target_link_libraries(stag
  ${OpenCV_LIBS}
)

target_link_libraries(stag_nodelet
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  stag
)
swri_nodelet_add_node(stag_nodelet_runnable stag_ros StagNodelet)
target_link_libraries(stag_nodelet_runnable stag_nodelet)

target_link_libraries(stag_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  stag
)

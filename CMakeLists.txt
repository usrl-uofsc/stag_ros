cmake_minimum_required(VERSION 2.8.3)
project(stag_ros)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
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
  INCLUDE_DIRS include lib/stag_core
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
add_library(stag_core
  lib/stag_core/Decoder.cpp
  lib/stag_core/Drawer.cpp
  lib/stag_core/EDInterface.cpp
  lib/stag_core/Ellipse.cpp
  lib/stag_core/Marker.cpp
  lib/stag_core/PoseRefiner.cpp
  lib/stag_core/Quad.cpp
  lib/stag_core/QuadDetector.cpp
  lib/stag_core/Stag.cpp
  lib/stag_core/utility.cpp
  lib/stag_core/ED/ED.cpp
  lib/stag_core/ED/EDInternals.cpp
  lib/stag_core/ED/EDLines.cpp
  lib/stag_core/ED/GradientOperators.cpp
  lib/stag_core/ED/ImageSmooth.cpp
  lib/stag_core/ED/LineSegment.cpp
  lib/stag_core/ED/MyMath.cpp
  lib/stag_core/ED/NFA.cpp
  lib/stag_core/ED/Utilities.cpp
  lib/stag_core/ED/ValidateEdgeSegments.cpp
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

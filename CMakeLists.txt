cmake_minimum_required(VERSION 3.15)
project(stag_ros)

# default to C99
if (NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif ()

# default to C++17
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp 0.7.0 REQUIRED)

ament_auto_find_build_dependencies()

file(GLOB MSGS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} msg/*.msg)
rosidl_generate_interfaces(${PROJECT_NAME} ${MSGS} DEPENDENCIES geometry_msgs)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

###########
## Build ##
###########

## Runnables
ament_auto_add_library(stag_core SHARED
  ${PROJECT_SOURCE_DIR}/src/stag/Decoder.cpp
  ${PROJECT_SOURCE_DIR}/src/stag/Drawer.cpp
  ${PROJECT_SOURCE_DIR}/src/stag/EDInterface.cpp
  ${PROJECT_SOURCE_DIR}/src/stag/Ellipse.cpp
  ${PROJECT_SOURCE_DIR}/src/stag/Marker.cpp
  ${PROJECT_SOURCE_DIR}/src/stag/PoseRefiner.cpp
  ${PROJECT_SOURCE_DIR}/src/stag/Quad.cpp
  ${PROJECT_SOURCE_DIR}/src/stag/QuadDetector.cpp
  ${PROJECT_SOURCE_DIR}/src/stag/Stag.cpp
  ${PROJECT_SOURCE_DIR}/src/stag/utility.cpp
  ${PROJECT_SOURCE_DIR}/src/stag/ED/ED.cpp
  ${PROJECT_SOURCE_DIR}/src/stag/ED/EDInternals.cpp
  ${PROJECT_SOURCE_DIR}/src/stag/ED/EDLines.cpp
  ${PROJECT_SOURCE_DIR}/src/stag/ED/GradientOperators.cpp
  ${PROJECT_SOURCE_DIR}/src/stag/ED/ImageSmooth.cpp
  ${PROJECT_SOURCE_DIR}/src/stag/ED/LineSegment.cpp
  ${PROJECT_SOURCE_DIR}/src/stag/ED/MyMath.cpp
  ${PROJECT_SOURCE_DIR}/src/stag/ED/NFA.cpp
  ${PROJECT_SOURCE_DIR}/src/stag/ED/ValidateEdgeSegments.cpp
  )

## Links
target_link_libraries(stag_core
  ${OpenCV_LIBS}
  ${CPP_REQUIRED_LIBRARIES}
  yaml-cpp
  )

ament_auto_add_executable(stag_node ${PROJECT_SOURCE_DIR}/src/stag_ros/stag_node.cpp)
# link messages
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(stag_node stag_core "${cpp_typesupport_target}")

# install the launch directory
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
  )

# install the sh scripts
install(
  PROGRAMS scripts/download_bags.sh
  DESTINATION lib/${PROJECT_NAME})

# install config files
install(DIRECTORY
  cfg
  DESTINATION share/${PROJECT_NAME}
  )

ament_auto_package()

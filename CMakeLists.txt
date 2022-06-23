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

ament_auto_find_build_dependencies()

# find library dependencies
set(CPP_REQUIRED_LIBRARIES yaml-cpp)

foreach (REQUIRED_LIBRARY IN LISTS CPP_REQUIRED_LIBRARIES)
  find_library(${REQUIRED_LIBRARY} REQUIRED)
endforeach ()

file(GLOB MSGS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} msg/*.msg)
rosidl_generate_interfaces(${PROJECT_NAME} ${MSGS} DEPENDENCIES geometry_msgs)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
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
  )

ament_auto_add_executable(stag_node ${PROJECT_SOURCE_DIR}/src/stag_ros/stag_node.cpp)

get_available_rmw_implementations(rmw_implementations2)
foreach (rmw_implementation ${rmw_implementations2})
  find_package("${rmw_implementation}" REQUIRED)
  get_rmw_typesupport(typesupport_impls "${rmw_implementation}" LANGUAGE "cpp")
  foreach (typesupport_impl ${typesupport_impls})
    rosidl_target_interfaces(stag_node
      ${PROJECT_NAME} ${typesupport_impl}
      )
  endforeach ()
endforeach ()

target_link_libraries(stag_node stag_core)

# install the launch directory
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
  )

# install the sh scripts
install(
  PROGRAMS scripts/download_bags.sh
  DESTINATION lib/${PROJECT_NAME})

ament_auto_package()

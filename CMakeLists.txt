cmake_minimum_required(VERSION 2.8.3)
project(stag)

add_compile_options(-std=c++11)

find_package(OpenCV 4 REQUIRED)

include_directories(
	src
	src/ED
	${OpenCV_INCLUDE_DIRS}
)

add_executable(stag_test
	src/main.cpp
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

target_link_libraries(stag_test
	${OpenCV_LIBS}
)

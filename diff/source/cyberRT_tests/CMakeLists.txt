cmake_minimum_required(VERSION 2.8)
project( VideoCap )

set(CMAKE_BINARY_DIR ..)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})

find_package(OpenCV 4.1.1 REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable( video_cap video_cap.cpp )
target_compile_features(video_cap PRIVATE cxx_range_for)
target_link_libraries( video_cap ${OpenCV_LIBS} )

add_executable( video_cap_2cam video_cap_2cam.cpp )
target_compile_features(video_cap_2cam PRIVATE cxx_range_for)
target_link_libraries( video_cap_2cam ${OpenCV_LIBS} )

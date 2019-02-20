cmake_minimum_required(VERSION 2.8.3)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
message(STATUS "Configuring tway_core")

file(GLOB TARGET_SRC
	"include/*.h"
	"include/*.hpp"
	"include/imu/*.h"
	"include/imu/*.hpp"
	"src/*.cpp"
	"src/imu/*.cpp"
)
find_package(catkin REQUIRED COMPONENTS rtt_ros)
catkin_package(
	INCLUDE_DIRS
		include
)
find_package(orocos_kdl REQUIRED)
find_package(Boost REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)

# Use 'source orocos_toolchain/env.sh' if the command below fails:
find_package(OROCOS-RTT QUIET)
if (NOT OROCOS-RTT_FOUND)
  message (FATAL_ERROR "\nCould not find Orocos. Please use the shell command\n 'source orocos_toolchain/env.sh' and then run cmake again.")
endif()

# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include( ${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )
########################################################################33
# rtt_ros
add_definitions(-DRTT_COMPONENT)
orocos_component(RealTimeTask src/real_time_task.cpp include/real_time_task.h)
orocos_install_headers( include/real_time_task.h ) # ...you may add multiple header files
orocos_generate_package(
	INCLUDE_DIRS include
	DEPENDS rtt_ros)


include_directories(
	include
)
# Create library
set(TARGET_NAME tway_core)
add_library(${TARGET_NAME} SHARED ${TARGET_SRC})
set_property(TARGET ${TARGET_NAME} PROPERTY CXX_STANDARD 14)

target_link_libraries(${TARGET_NAME} kacanopen_master)


target_link_libraries(${TARGET_NAME} ${catkin_LIBRARIES})

# Install
kacanopen_install(${TARGET_NAME} TRUE)






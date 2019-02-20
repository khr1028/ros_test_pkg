cmake_minimum_required(VERSION 2.8)

message(STATUS "Configuring tway_com")

file(GLOB TARGET_SRC
	"src/*.cpp"
)
#######################################################################
find_package(catkin REQUIRED COMPONENTS rtt_ros)


add_definitions(-DRTT_COMPONENT)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
########################################################################################


# Create library
set(TARGET_NAME kacanopen_tway_com)
add_library(${TARGET_NAME} SHARED ${TARGET_SRC})
set_property(TARGET ${TARGET_NAME} PROPERTY CXX_STANDARD 14)

target_link_libraries(${TARGET_NAME} kacanopen_master)
target_link_libraries(${TARGET_NAME} ${catkin_LIBRARIES})

# Install
kacanopen_install(${TARGET_NAME} TRUE)

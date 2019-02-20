#
# The find_package macro for Orocos-RTT works best with
# cmake >= 2.6.3
#
cmake_minimum_required(VERSION 2.6.3)

#
# This creates a standard cmake project. You may extend this file with
# any cmake macro you see fit.
#
project(test_component)

find_package(catkin REQUIRED)
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

# include_directories(include
#     ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS}
# )
  
orocos_component(${PROJECT_NAME} src/test_component.hpp src/test_component.cpp)
#orocos_executable(test test/test.cpp)
## Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-test test/test.cpp)
if(TARGET ${PROJECT_NAME}-test)
    target_link_libraries(
        ${PROJECT_NAME}-test ${PROJECT_NAME} 
        ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES}
        ${Boost_LIBRARIES}
    )


endif()

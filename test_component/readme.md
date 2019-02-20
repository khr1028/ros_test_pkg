# Orocos Test component #

## The idea
Unit testing for Orocos components using Google test

## To Do
- Read and write on ports
- To be completed

## How to use

### Dependencies
- Google test
- ROS / RTT

### Build
- build with catkin:
`catkin_make run_tests_test_component`

- or cmake:
    - `mkdir buid && cd build`
    - `cmake ..`
    - `make all`
    - `make run_tests` OR `make run_tests_test_component`

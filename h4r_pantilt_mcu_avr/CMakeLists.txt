cmake_minimum_required(VERSION 2.8.3)
project(h4r_pantilt_mcu_avr)

find_package(catkin REQUIRED COMPONENTS
    h4r_pantilt_proto
    h4r_libsersyncproto
)
catkin_package()

add_subdirectory(mcu)

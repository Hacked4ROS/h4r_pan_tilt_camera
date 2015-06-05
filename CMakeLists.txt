cmake_minimum_required(VERSION 2.8.3)
project(h4r_pantilt_adapter)

find_package(catkin REQUIRED COMPONENTS
  h4r_libsersyncproto
  geometry_msgs
  roscpp
  sensor_msgs
  serial
  std_msgs
  tf
  tf2
)

find_package(Boost REQUIRED COMPONENTS system thread)

catkin_package(
  INCLUDE_DIRS include shared
)

include_directories(  
  include
  shared
  ${catkin_INCLUDE_DIRS}
)

add_executable(pantilt_adapter_node src/pantilt_adapter_node.cpp src/PanTiltAdapter.cpp)
add_executable(pantilt_joy_node src/pantilt_joy_node.cpp src/PanTiltJoy.cpp)


target_link_libraries(pantilt_adapter_node
   ${h4r_libsersyncproto_LIBRARIES}
   ${catkin_LIBRARIES}
)

target_link_libraries(pantilt_joy_node
   ${catkin_LIBRARIES}
)


## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(h4r_pan_tilt_camera_node h4r_pan_tilt_camera_generate_messages_cpp)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html


##TODO
##Upload script for Arduino
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS h4r_pan_tilt_camera h4r_pan_tilt_camera_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_h4r_pan_tilt_camera.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

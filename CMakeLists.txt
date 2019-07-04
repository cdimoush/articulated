cmake_minimum_required(VERSION 2.8.3)
project(articulated)

SET(CMAKE_CXX_FLAGS "-std=c++11 -O3")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  actionlib_msgs
  message_generation
  tf
)

find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)

## System dependencies are found with CMake's conventions
#find_package(Boost REQUIRED COMPONENTS system)



## Generate messages in the 'msg' folder
add_message_files(
  FILES
  serial_msg.msg
)

## Generate services in the 'srv' folder
add_service_files(
 FILES
 StepperDriver.srv
)

## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action
  FILES ik.action
)

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs 
   geometry_msgs 
   sensor_msgs 
   actionlib_msgs
 )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES articulated
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs sensor_msgs actionlib_msgs
  #message_runtime
  #DEPENDS system_lib
)

include_directories(
include 
${catkin_INCLUDE_DIRS}
${eigen3_INCLUDE_DIRS})

add_library(mechanism_calculations src/mechanism_calculations.cpp)

add_executable(stepper_driver_client src/stepper_driver_client.cpp)
target_link_libraries(stepper_driver_client ${catkin_LIBRARIES} ${eigen3_LIBRARIES})
target_link_libraries(stepper_driver_client mechanism_calculations)

add_executable(articulated_action src/articulated_action.cpp)
target_link_libraries(articulated_action ${catkin_LIBRARIES} ${eigen3_LIBRARIES})
target_link_libraries(articulated_action mechanism_calculations)

add_executable(articulated_client src/articulated_simple_client.cpp)
target_link_libraries(articulated_client ${catkin_LIBRARIES})

add_executable(articulated_tf_broadcaster src/articulated_tf_broadcaster.cpp)
target_link_libraries(articulated_tf_broadcaster ${catkin_LIBRARIES} ${eigen3_LIBRARIES})


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_articulated.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

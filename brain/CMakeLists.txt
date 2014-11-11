cmake_minimum_required(VERSION 2.8.3)
project(brain)

#find_package(catkin REQUIRED COMPONENTS roscpp)
# std_msgs)
find_package(common REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  rospy
)

################################################
## Declare ROS messages, services and actions ##
################################################

catkin_package()

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

include_directories(${common_INCLUDE_DIRS})

#add_executable(brain brain.cpp)
#target_link_libraries(brain ${catkin_LIBRARIES})
#target_link_libraries(brain ${common_LIBRARIES})

#############
## Install ##
#############

catkin_install_python(PROGRAMS src/brain.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


#############
## Testing ##
#############


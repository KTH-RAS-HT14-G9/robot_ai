cmake_minimum_required(VERSION 2.8.3)
project(ir_converter)

find_package(catkin REQUIRED COMPONENTS 
roscpp
std_msgs
message_generation
common
)
#find_package(common REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  Distance.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES 
  CATKIN_DEPENDS message_runtime
  DEPENDS
)

###########
## Build ##
###########

include_directories(${common_INCLUDE_DIRS})

add_executable(ir_converter ir_converter.cpp)
add_dependencies(ir_converter ir_converter_generate_messages_cpp)
target_link_libraries(ir_converter ${catkin_LIBRARIES})
target_link_libraries(ir_converter ${common_LIBRARIES})

#############
## Install ##
#############


#############
## Testing ##
#############


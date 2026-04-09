cmake_minimum_required(VERSION 3.0.2)
project(my_class_pkg)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  actionlib
  actionlib_msgs
  dynamic_reconfigure
  geometry_msgs
  tf2_ros
  upros_message
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
	DIRECTORY msg
	FILES MyMessage.msg
)

add_service_files(
	DIRECTORY srv
	FILES MyServiceMsg.srv
)

add_action_files(
	DIRECTORY action
	FILES MyAction.action
)

generate_messages(
	DEPENDENCIES
	std_msgs
	actionlib_msgs
)

###################################################
## Declare ROS dynamic reconfigure parameters    ##
###################################################

generate_dynamic_reconfigure_options(
  cfg/Tutorials.cfg
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_class_pkg
  CATKIN_DEPENDS
  roscpp
  rospy
  std_msgs
  message_runtime
  actionlib
  actionlib_msgs
  dynamic_reconfigure
  geometry_msgs
  upros_message
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/my_class_pkg.cpp
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_my_class_pkg.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

# ===== 基础话题节点（无自定义消息，不需要 dependencies）=====
add_executable(ros_publisher_node src/ros_publisher.cpp)
target_link_libraries(ros_publisher_node ${catkin_LIBRARIES})

add_executable(ros_subscriber_node src/ros_subscriber.cpp)
target_link_libraries(ros_subscriber_node ${catkin_LIBRARIES})

# ===== 自定义消息节点（必须加 dependencies）=====
add_executable(msg_publisher_node src/msg_publisher.cpp)
add_dependencies(msg_publisher_node ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(msg_publisher_node ${catkin_LIBRARIES})

add_executable(msg_subscriber_node src/msg_subscriber.cpp)
add_dependencies(msg_subscriber_node ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(msg_subscriber_node ${catkin_LIBRARIES})

# ===== 自定义服务节点（必须加 dependencies）=====
add_executable(ros_server_node src/ros_server.cpp)
add_dependencies(ros_server_node ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(ros_server_node ${catkin_LIBRARIES})

add_executable(ros_client_node src/ros_client.cpp)
add_dependencies(ros_client_node ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(ros_client_node ${catkin_LIBRARIES})

# ===== 自定义动作节点（必须加 dependencies）=====
add_executable(ros_action_server src/ros_action_server.cpp)
add_dependencies(ros_action_server ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(ros_action_server ${catkin_LIBRARIES})

add_executable(ros_action_client src/ros_action_client.cpp)
add_dependencies(ros_action_client ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(ros_action_client ${catkin_LIBRARIES})

# ===== 里程计移动节点 =====
add_executable(odom_move src/odom_move.cpp)
target_link_libraries(odom_move ${catkin_LIBRARIES})

# ===== Homework2 节点 =====
add_executable(bumper_controller src/bumper_controller.cpp)
target_link_libraries(bumper_controller ${catkin_LIBRARIES})

add_executable(tof_avoidance src/tof_avoidance.cpp)
target_link_libraries(tof_avoidance ${catkin_LIBRARIES})

add_executable(rotate_180 src/rotate_180.cpp)
target_link_libraries(rotate_180 ${catkin_LIBRARIES})

# ===== dynamic_reconfigure 节点（必须加 gencfg 依赖）=====
add_executable(dynamic_reconfigure_node src/dynamic_reconfigure.cpp)
add_dependencies(dynamic_reconfigure_node ${PROJECT_NAME}_gencfg)
target_link_libraries(dynamic_reconfigure_node ${catkin_LIBRARIES})

add_executable(ros_dynamic_speed_node src/ros_dynamic_speed.cpp)
add_dependencies(ros_dynamic_speed_node my_class_pkg_gencfg)
target_link_libraries(ros_dynamic_speed_node ${catkin_LIBRARIES} ${my_class_pkg_LIBRARIES})

#fifth homework
add_executable(tag_grab_node src/tag_grab.cpp)
target_link_libraries(tag_grab_node ${catkin_LIBRARIES})

cmake_minimum_required(VERSION 2.8.3)
project(yolact_ros)

find_package(catkin REQUIRED COMPONENTS
    rospy
    actionlib_msgs
    geometry_msgs
    sensor_msgs
    std_msgs
    message_generation
)

add_message_files(
  FILES
    Segment.msg
    Segments.msg
)

add_action_files(
  FILES
    Segmentation.action
)

generate_messages(
  DEPENDENCIES
    actionlib_msgs
    geometry_msgs
    sensor_msgs
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    actionlib_msgs
    geometry_msgs
    sensor_msgs
    message_runtime
    std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

cmake_minimum_required(VERSION 2.8.3)
project(yolact_ros)

find_package(catkin REQUIRED COMPONENTS
    rospy
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
    std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

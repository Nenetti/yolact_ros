cmake_minimum_required(VERSION 2.8.3)
project(yolact_ros_msgs)

add_definitions(-std=c++11)

set(PACKAGE_DEPENDENCIES
        std_msgs
        geometry_msgs
        sensor_msgs
        actionlib_msgs
        )

find_package(catkin REQUIRED
        COMPONENTS rospy ${PACKAGE_DEPENDENCIES} message_generation
        )

add_message_files(
        FILES
        Segment.msg
        Segments.msg
)

add_action_files(
        FILES
        CheckForObjects.action
)

generate_messages(
        DEPENDENCIES ${PACKAGE_DEPENDENCIES}
)

catkin_package(
        CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES} message_runtime
)
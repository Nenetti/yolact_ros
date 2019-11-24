cmake_minimum_required(VERSION 2.8.12)
project(yolact_ros_octomap_mapping)

set(PACKAGE_DEPENDENCIES
        roscpp
        visualization_msgs
        sensor_msgs
        pcl_ros
        pcl_conversions
        nav_msgs
        std_msgs
        std_srvs
        octomap_ros
        octomap_msgs
        dynamic_reconfigure
        cv_bridge
        actionlib_msgs
        )

file(GLOB MODULES
        src/custom_octomap/*.cpp
        src/octomap_server/modules/*.cpp
        src/semantic_segmentation/modules/*.cpp
        )

SET(SRCS
        ${MODULES}
        src/octomap_server/octomap_server.cpp
        src/semantic_segmentation/segmentation_client.cpp
        src/semantic_segmentation/segmentation_server.cpp
        )

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES} message_generation)
find_package(OpenCV REQUIRED)
find_package(octomap REQUIRED)


add_definitions(-DOCTOMAP_NODEBUGOUT)
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${OCTOMAP_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

generate_dynamic_reconfigure_options(cfg/OctomapServer.cfg)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
        DEPENDS OCTOMAP
)

set(LINK_LIBS
        ${OCTOMAP_LIBRARIES}
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        )

find_package(OpenMP REQUIRED)
SET(CMAKE_C_FLAGS ${OpenMP_C_FLAGS})
SET(CMAKE_CXX_FLAGS ${OpenMP_CXX_FLAGS})

#add_library(${PROJECT_NAME} src/octomap_server/octomap_server.cpp)
add_library(${PROJECT_NAME} ${SRCS})
add_definitions(-std=c++11)

target_link_libraries(${PROJECT_NAME} ${LINK_LIBS})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

add_executable(octomap_server_node src/octomap_server/octomap_server_node.cpp)
target_link_libraries(octomap_server_node ${PROJECT_NAME} ${LINK_LIBS})

add_executable(segmentation_server_node src/semantic_segmentation/segmentation_server_node.cpp)
target_link_libraries(segmentation_server_node ${PROJECT_NAME} ${LINK_LIBS})

# install targets:
install(TARGETS ${PROJECT_NAME}
        octomap_server_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE
        )

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE
        )

install(FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

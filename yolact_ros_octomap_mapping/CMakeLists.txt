cmake_minimum_required(VERSION 2.8)
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
        nodelet
        cv_bridge
        )

SET(SRCS
        src/custom_octomap/AbstractOccupancyOcTree.cpp
        src/custom_octomap/AbstractOcTree.cpp
        src/custom_octomap/Color.cpp
        src/custom_octomap/ColorOcTree.cpp
        src/custom_octomap/OccupancyOcTreeBase.cpp
        src/custom_octomap/OcTreeBaseImpl.cpp
        src/custom_octomap/OcTreeIterator.cpp
        src/custom_octomap/OcTreeKey.cpp
        src/custom_octomap/OcTreeNode.cpp
        src/octomap_server/OctomapServer.cpp
        src/octomap_server/Filter.cpp
        src/semantic_mapping/GeometricEdgeMap.cpp
        src/semantic_mapping/Filter.cpp
        src/semantic_mapping/SegmentationClient.cpp
        src/semantic_mapping/Cluster.cpp
        src/semantic_mapping/Segment.cpp
        src/semantic_mapping/MarkerClient.cpp
        )

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})
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

#add_library(${PROJECT_NAME} src/octomap_server/OctomapServer.cpp)
add_library(${PROJECT_NAME} ${SRCS})
add_definitions(-std=c++11)

target_link_libraries(${PROJECT_NAME} ${LINK_LIBS})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

add_executable(octomap_server_node src/octomap_server/octomap_server_node.cpp)
target_link_libraries(octomap_server_node ${PROJECT_NAME} ${LINK_LIBS})

#add_executable(ColorOcTree src/ColorOcTree.cpp)
#target_link_libraries(ColorOcTree ${PROJECT_NAME} ${LINK_LIBS})

#add_executable(octomap_color_server_node src/octomap_color_server_node.cpp)
#target_link_libraries(octomap_color_server_node ${PROJECT_NAME} ${LINK_LIBS})

#add_executable(octomap_server_static src/octomap_server_static.cpp)
#target_link_libraries(octomap_server_static ${PROJECT_NAME} ${LINK_LIBS})

#add_executable(octomap_server_multilayer src/octomap_server_multilayer.cpp)
#target_link_libraries(octomap_server_multilayer ${PROJECT_NAME} ${LINK_LIBS})

#add_executable(octomap_saver src/octomap_server/octomap_saver.cpp)
#target_link_libraries(octomap_saver ${PROJECT_NAME} ${LINK_LIBS})

#add_executable(octomap_tracking_server_node src/octomap_tracking_server_node.cpp)
#target_link_libraries(octomap_tracking_server_node ${PROJECT_NAME} ${LINK_LIBS})

# Nodelet
#add_library(octomap_server_nodelet src/octomap_server_nodelet.cpp)
#target_link_libraries(octomap_server_nodelet ${PROJECT_NAME} ${LINK_LIBS})

# install targets:
install(TARGETS ${PROJECT_NAME}
        octomap_server_node
        #  octomap_server_static
        #  octomap_server_multilayer
#        octomap_saver
        #  octomap_tracking_server_node
        #  octomap_server_nodelet
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

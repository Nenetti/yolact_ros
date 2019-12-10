//
// Created by ubuntu on 2019/11/14.
//

#ifndef SEGMENTATION_SERVER_SEGMENTATIONSERVER_H
#define SEGMENTATION_SERVER_SEGMENTATIONSERVER_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <sensor_msgs/PointCloud2.h>
#include <yolact_ros_octomap_mapping/SegmentMarker.h>
#include <yolact_ros_octomap_mapping/SegmentMarkers.h>

#include <yolact_ros/SegmentationAction.h>

#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>

#include <segmentation_server/semantic_mapping.h>
#include <tf/transform_listener.h>

namespace segmentation_server {

    class SegmentationServer {

        public:

            explicit SegmentationServer(ros::NodeHandle nh = ros::NodeHandle("~"));

            void call_back(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg);

            void apply_segmentation_to_cloud(pcl::PointCloud<pcl::PointXYZRGB> &cloud, const yolact_ros::Segments &segment_msg);

        protected:
            ros::NodeHandle m_nh;
            ros::Publisher m_cloud_pub, m_segmentation_filter_pub, m_clustering_pub, m_marker_info_pub;
            ros::Subscriber m_cloud_sub;
            actionlib::SimpleActionClient<yolact_ros::SegmentationAction> *m_segmentation_client;

            std::string m_baseFrameId, m_worldFrameId;
            double m_ground_distance, m_ceiling_distance;
            bool m_is_cloud_pub, m_is_segmentation_filter_pub, m_is_clustering_pub, m_is_marker_pub, m_is_marker_info_pub;
            tf::TransformListener m_tfListener;

            segmentation_server::SemanticMapping m_semantic_map;
            semantic_mapping::MarkerClient m_marker_client;

            void publish_segmentation_filter_image(const pcl::PointCloud<pcl::PointXYZRGB> &cloud, const std::vector<semantic_mapping::Segment> &segments);

            void publish_clustering_image(const pcl::PointCloud<pcl::PointXYZRGB> &cloud, const std::vector<semantic_mapping::Cluster> &clusters);

            void publish_segmentation_cloud(const pcl::PointCloud<pcl::PointXYZRGB> &cloud, const std::vector<semantic_mapping::Segment> &segments);

            static void convert_segment_data(const yolact_ros::Segments &segment_msg, std::vector<semantic_mapping::Segment> &segments, int width);

            void publish_all(const pcl::PointCloud<pcl::PointXYZRGB> &cloud, const std::vector<semantic_mapping::Segment> &segments,
                             const std::vector<semantic_mapping::Cluster> &clusters);

            void publish_marker_info(const std::vector<semantic_mapping::Segment> &segments);
    };

}

#endif //SEGMENTATION_SERVER_SEGMENTATIONSERVER_H

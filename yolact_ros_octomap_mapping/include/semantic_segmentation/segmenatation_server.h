//
// Created by ubuntu on 2019/11/14.
//

#ifndef SEGMENTATION_SERVER_SEGMENTATIONSERVER_H
#define SEGMENTATION_SERVER_SEGMENTATIONSERVER_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <sensor_msgs/PointCloud2.h>
#include <yolact_ros_octomap_mapping_msgs/RequestSemanticCloudAction.h>

#include <yolact_ros_msgs/CheckForObjectsAction.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>

#include <semantic_segmentation/modules/cluster.h>
#include <semantic_segmentation/modules/segment.h>
#include <semantic_segmentation/modules/semantic_mapping.h>
#include <semantic_segmentation/modules/marker_client.h>

namespace semantic_segmentation {

    class SegmentationServer {

        public:

            explicit SegmentationServer(ros::NodeHandle nh = ros::NodeHandle("~"));

            void call_back(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg);

        protected:
            ros::NodeHandle m_nh;
            ros::Publisher m_cloud_pub;
            ros::Publisher m_segmentation_filter_pub;
            ros::Publisher m_clustering_pub;
            ros::Subscriber m_cloud_sub;
            actionlib::SimpleActionClient<yolact_ros_msgs::CheckForObjectsAction> *m_segmentation_client;
            tf::TransformListener m_tfListener;

            bool m_is_cloud_pub;
            bool m_is_segmentation_filter_pub;
            bool m_is_clustering_pub;
            bool m_is_marker_pub;

            int m_width;
            int m_height;

            double m_ground_distance;
            double m_ceiling_distance;

            std::string m_baseFrameId;
            std::string m_worldFrameId;

            semantic_segmentation::SemanticMapping m_semantic_map;
            semantic_segmentation::MarkerClient m_marker_client;

            void publish_segmentation_filter_image(const std::vector<semantic_segmentation::Segment> &segments);

            void publish_clustering_image(const std::vector<semantic_segmentation::Cluster> &clusters);

            void publish_segmentation_cloud(const pcl::PointCloud<pcl::PointXYZRGB> &cloud, const std::vector<semantic_segmentation::Segment> &segments);

            void convert_segment_data(const yolact_ros_msgs::Segments &segment_msg, std::vector<semantic_segmentation::Segment> &segments);

            void publish_all(const pcl::PointCloud<pcl::PointXYZRGB> &cloud, const std::vector<semantic_segmentation::Segment> &segments,
                             const std::vector<semantic_segmentation::Cluster> &clusters);

            actionlib::SimpleActionServer<yolact_ros_octomap_mapping_msgs::RequestSemanticCloudAction> *m_action_server;

            bool apply_segmentation_to_cloud(const sensor_msgs::PointCloud2 &cloud_msg, pcl::PointCloud<pcl::PointXYZRGB> &cloud,
                                             std::vector<semantic_segmentation::Segment> &segments, std::vector<semantic_segmentation::Cluster> &clusters);

            void to_segment_msg(const std::vector<semantic_segmentation::Segment> &segments, yolact_ros_octomap_mapping_msgs::SemanticObjects &segments_msg);

            void action_call_back();
    };

}

#endif //SEGMENTATION_SERVER_SEGMENTATIONSERVER_H

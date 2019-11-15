//
// Created by ubuntu on 2019/10/16.
//

#ifndef SEMANTIC_MAPPING_MARKERCLIENT_H
#define SEMANTIC_MAPPING_MARKERCLIENT_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <ros/node_handle.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <yolact_ros/SegmentationResult.h>
#include <semantic_mapping/Cluster.h>
#include <semantic_mapping/Segment.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/ColorRGBA.h>

namespace semantic_mapping {

    using pcl::PointCloud;
    using pcl::PointXYZRGB;
    using pcl::PointXYZRGBL;
    using pcl::Normal;

    class MarkerClient {

        public:

            MarkerClient();


            void publish_segment_info(const PointCloud<PointXYZRGB> &cloud, const std::vector<Segment> &segments);

        protected:
            ros::NodeHandle m_nh;
            ros::Publisher segment_marker_publisher;
            int m_line_list_id_base = 1000;
            int m_segment_name_id_base = 1100;

            void publish_line_list(const PointCloud<PointXYZRGB> &cloud, const std::vector<Segment> &segments);

            void publish_segment_name(const PointCloud<PointXYZRGB> &cloud, const std::vector<Segment> &segments);

            static void to_line_list(const PointCloud<PointXYZRGB> &cloud, const std::vector<Segment> &segments,
                                     std::vector<std::vector<geometry_msgs::Point>> &line_list);

            static void set_scale(double x, double y, double z, geometry_msgs::Vector3_<std::allocator<void>> &vector);

            static void set_coordinate(double x, double y, double z, geometry_msgs::Point &point);

            void set_color(double r, double g, double b, double a, std_msgs::ColorRGBA_<std::allocator<void>> &color);
    };

}

#endif //SEMANTIC_MAPPING_MARKERCLIENT_H

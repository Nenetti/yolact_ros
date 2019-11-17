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
#include <semantic_mapping/cluster.h>
#include <semantic_mapping/segment.h>
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

            explicit MarkerClient(ros::NodeHandle nh = ros::NodeHandle("~"));

            void publish_segment_info(const PointCloud<PointXYZRGB> &cloud, const std::vector<Segment> &segments);

        protected:
            ros::NodeHandle m_nh;
            ros::Publisher m_marker_pub;

            std::string m_bounding_box_ns;
            std::string m_name_ns;
            std::string m_frame;
            int m_bounding_box_id_base;
            int m_name_id_base;

            int m_before_marker_size;
            int m_marker_size;

            void publish_line_list(const PointCloud<PointXYZRGB> &cloud, const std::vector<Segment> &segments);

            void publish_segment_name(const PointCloud<PointXYZRGB> &cloud, const std::vector<Segment> &segments);

            void generate_bounding_box(const PointCloud<PointXYZRGB> &cloud, const std::vector<Segment> &segments,
                                              std::vector<std::vector<geometry_msgs::Point>> &line_list);

            static void set_coordinate(geometry_msgs::Point &point, double x, double y, double z);

            static void set_color(std_msgs::ColorRGBA &color, double r, double g, double b, double a);

            static void set_scale(geometry_msgs::Vector3 &vector, double x, double y, double z);

    };

}

#endif //SEMANTIC_MAPPING_MARKERCLIENT_H

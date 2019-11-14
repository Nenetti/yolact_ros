//
// Created by ubuntu on 2019/10/16.
//

#ifndef SEGMENTATION_SERVER_SEMANTICMAPPING_H
#define SEGMENTATION_SERVER_SEMANTICMAPPING_H

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
#include <semantic_mapping/MarkerClient.h>

namespace segmentation_server {

    class SemanticMapping {

        public:

            SemanticMapping();

            void set_filter(double ground_filter, double ceiling_filter);

            void to_segmentation(pcl::PointCloud<pcl::PointXYZRGB> &cloud, std::vector<semantic_mapping::Segment> &segments,
                                 std::vector<semantic_mapping::Cluster> &clusters);


        protected:

            semantic_mapping::MarkerClient m_marker_client;
            ros::Publisher geometric_publisher;
            std::vector<custom_octomap::Color> m_colors;

            double m_ground_filter;
            double m_ceiling_filter;


            static void calc_segment_range(const pcl::PointCloud<pcl::PointXYZRGB> &cloud, const std::vector<bool> &is_exclude,
                                           std::vector<semantic_mapping::Segment>
                                           &segments);

            void init_color(int size);

            static void clustering_from_edge(const pcl::PointCloud<pcl::PointXYZRGB> &cloud, const std::vector<bool> &edge_map, std::vector<int> &cluster_ids,
                                             std::vector<semantic_mapping::Cluster> &clusters,
                                             double threshold);

            void publish_geometric_image(const pcl::PointCloud<pcl::PointXYZRGB> &cloud, const std::vector<semantic_mapping::Segment> &segments,
                                         const std::vector<bool> &is_exclude);

            static void detect_segmentation_cluster(const std::vector<int> &cluster_ids, std::vector<semantic_mapping::Cluster> &clusters,
                                             std::vector<semantic_mapping::Segment> &segments);

            void set_color(std::vector<semantic_mapping::Segment> &segments);

            void set_result(const pcl::PointCloud<pcl::PointXYZRGB> &cloud, const std::vector<bool> &is_ground, const std::vector<bool> &is_ceiling,
                            const std::vector<bool> &is_infinite,
                            const std::vector<int8_t> &segment_map, const std::vector<semantic_mapping::Segment> &segments, int ground_point_size,
                            int ceiling_point_size,
                            pcl::PointCloud<pcl::PointXYZRGB> &ground, pcl::PointCloud<pcl::PointXYZRGB> &ceiling,
                            pcl::PointCloud<pcl::PointXYZRGB> &nonground_nonseg,
                            pcl::PointCloud<pcl::PointXYZRGBL> &nonground_seg);

            void to_mask_map(std::vector<semantic_mapping::Segment> &segments, std::vector<int8_t> &mask_map);
    };

}

#endif //SEMANTIC_MAPPING_GEOMETRICEDGEMAP_H

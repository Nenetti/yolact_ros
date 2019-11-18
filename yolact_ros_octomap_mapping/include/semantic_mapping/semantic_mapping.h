//
// Created by ubuntu on 2019/10/16.
//

#ifndef SEMANTIC_MAPPING_SEMANTICMAPPING_H
#define SEMANTIC_MAPPING_SEMANTICMAPPING_H

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
#include <semantic_mapping/marker_client.h>
#include <sensor_msgs/Image.h>
#include <semantic_mapping/filter.h>

namespace semantic_mapping {

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
                                           std::vector<semantic_mapping::Segment> &segments);

            void init_color(int size);

            static void clustering_from_edge(const pcl::PointCloud<pcl::PointXYZRGB> &cloud, const std::vector<bool> &edge_map, std::vector<int> &cluster_ids,
                                             std::vector<semantic_mapping::Cluster> &clusters, double threshold);

            static void detect_segmentation_cluster(const std::vector<int> &cluster_ids, std::vector<semantic_mapping::Cluster> &clusters,
                                                    std::vector<semantic_mapping::Segment> &segments);

            void set_color(std::vector<semantic_mapping::Segment> &segments);

            static void to_mask_map(std::vector<semantic_mapping::Segment> &segments, std::vector<int8_t> &mask_map);
    };

}

#endif //SEMANTIC_MAPPING_SEMANTICMAPPING_H

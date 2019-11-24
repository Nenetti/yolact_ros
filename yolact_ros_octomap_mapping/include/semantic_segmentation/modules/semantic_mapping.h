//
// Created by ubuntu on 2019/10/16.
//

#ifndef SEMANTIC_MAPPING_SEMANTICMAPPING_H
#define SEMANTIC_MAPPING_SEMANTICMAPPING_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <ros/node_handle.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Image.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <yolact_ros_msgs/CheckForObjectsResult.h>

#include <semantic_segmentation/modules/cluster.h>
#include <semantic_segmentation/modules/segment.h>
#include <semantic_segmentation/modules/marker_client.h>
#include <semantic_segmentation/modules/filter.h>
#include <semantic_segmentation/modules/color.h>

namespace semantic_segmentation {

    class SemanticMapping {

        public:

            SemanticMapping();

            void set_filter(double ground_filter, double ceiling_filter);

            void to_segmentation(pcl::PointCloud<pcl::PointXYZRGB> &cloud, std::vector<semantic_segmentation::Segment> &segments,
                                 std::vector<semantic_segmentation::Cluster> &clusters);

        protected:

            semantic_segmentation::MarkerClient m_marker_client;
            ros::Publisher geometric_publisher;
            std::vector<Color> m_colors;

            double m_ground_filter;
            double m_ceiling_filter;

            static void calc_segment_range(const pcl::PointCloud<pcl::PointXYZRGB> &cloud, const std::vector<bool> &is_exclude,
                                           std::vector<semantic_segmentation::Segment> &segments);

            void init_color(int size);

            static void clustering_from_edge(const pcl::PointCloud<pcl::PointXYZRGB> &cloud, const std::vector<bool> &edge_map, std::vector<int> &cluster_ids,
                                             std::vector<semantic_segmentation::Cluster> &clusters, double threshold);

            static void detect_segmentation_cluster(const std::vector<int> &cluster_ids, std::vector<semantic_segmentation::Cluster> &clusters,
                                                    std::vector<semantic_segmentation::Segment> &segments);

            void set_color(std::vector<semantic_segmentation::Segment> &segments);

            static void to_mask_map(std::vector<semantic_segmentation::Segment> &segments, std::vector<int8_t> &mask_map);

            void remove_unavailable_segments(std::vector<Segment> &segments);
    };

}

#endif //SEMANTIC_MAPPING_SEMANTICMAPPING_H

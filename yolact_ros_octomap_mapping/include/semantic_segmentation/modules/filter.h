//
// Created by ubuntu on 2019/10/16.
//

#ifndef SEMANTIC_MAPPING_FILTER_H
#define SEMANTIC_MAPPING_FILTER_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <ros/node_handle.h>
#include <opencv-3.3.1-dev/opencv2/core/core.hpp>
#include <opencv-3.3.1-dev/opencv2/highgui/highgui.hpp>
#include <opencv-3.3.1-dev/opencv2/imgproc/imgproc.hpp>
#include <yolact_ros_msgs/CheckForObjectsResult.h>
#include <semantic_segmentation/modules/cluster.h>
#include <semantic_segmentation/modules/segment.h>
#include <geometry_msgs/Point.h>
//#include <omp.h>

namespace semantic_segmentation {

    using pcl::PointCloud;
    using pcl::PointXYZRGB;
    using pcl::PointXYZRGBL;
    using pcl::Normal;

    class Filter {

        public:

            static void infinite_filter(const PointCloud<PointXYZRGB> &cloud, std::vector<bool> &is_infinite);

            static void exclude_noise_from_depth_edge_filter(const PointCloud<PointXYZRGB> &cloud, std::vector<bool> &is_depth_edge, int threshold_size);

            static void depth_edge_filter(const PointCloud<PointXYZRGB> &cloud, const std::vector<bool> &is_exclude,
                                          std::vector<bool> &is_depth_edge, double threshold);

            static void mask_edge_filter(const PointCloud<PointXYZRGB> &cloud, const std::vector<int8_t> &mask_map, std::vector<bool> &is_mask_edge);

            static int ground_filter(const PointCloud<PointXYZRGB> &cloud, std::vector<bool> &is_ground, double ground_filter);

            static void combine_bool_filter(const std::vector<bool> &inputA, const std::vector<bool> &inputB, std::vector<bool> &output);

            static void cluster_filter(const std::vector<Segment> &segments, const std::vector<Cluster> &clusters, std::vector<int8_t> &segment_map);

            static int ceiling_filter(const PointCloud<PointXYZRGB> &cloud, std::vector<bool> &is_ceiling, double ceiling_filter);
    };

}

#endif //SEMANTIC_MAPPING_FILTER_H

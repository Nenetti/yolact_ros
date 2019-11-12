//
// Created by ubuntu on 2019/10/16.
//

#ifndef SEMANTIC_MAPPING_GEOMETRICEDGEMAP_H
#define SEMANTIC_MAPPING_GEOMETRICEDGEMAP_H

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

namespace semantic_mapping {

    using pcl::PointCloud;
    using pcl::PointXYZRGB;
    using pcl::PointXYZRGBL;
    using pcl::Normal;

    class GeometricEdgeMap {

        public:

            GeometricEdgeMap();

            void set_filter(double ground_filter, double ceiling_filter);

            void toSegmentation(PointCloud<PointXYZRGB> &cloud, std::vector<Segment> &segments,
                                PointCloud<PointXYZRGB> &ground, PointCloud<PointXYZRGB> &ceiling,
                                PointCloud<PointXYZRGB> &nonground_nonseg, PointCloud<PointXYZRGBL> &nonground_seg);


        protected:

            semantic_mapping::MarkerClient m_marker_client;
            ros::NodeHandle m_nh;
            ros::Publisher geometric_publisher;
            std::vector<custom_octomap::Color> m_colors;

            double m_ground_filter;
            double m_ceiling_filter;


            static void calc_segment_range(const PointCloud<PointXYZRGB> &cloud, const std::vector<bool> &is_exclude, std::vector<Segment> &segments);

            void init_color(int size);

            static void clustering_from_edge(const PointCloud<PointXYZRGB> &cloud, const std::vector<bool> &edge_map, std::vector<int> &cluster_ids,
                                             std::vector<Cluster> &clusters,
                                             double threshold);

            void publish_geometric_image(const PointCloud<PointXYZRGB> &cloud, const std::vector<Segment> &segments, const std::vector<bool> &is_exclude);

            void detect_segmentation_cluster(const std::vector<int> &cluster_ids, std::vector<Cluster> &clusters, std::vector<Segment> &segments);

            void set_color(std::vector<Segment> &segments);

            void set_result(const PointCloud<PointXYZRGB> &cloud, const std::vector<bool> &is_ground, const std::vector<bool> &is_ceiling,
                            const std::vector<bool> &is_infinite,
                            const std::vector<int8_t> &segment_map, const std::vector<Segment> &segments, int ground_point_size, int ceiling_point_size,
                            PointCloud<PointXYZRGB> &ground, PointCloud<PointXYZRGB> &ceiling, PointCloud<PointXYZRGB> &nonground_nonseg,
                            PointCloud<PointXYZRGBL> &nonground_seg);
    };

}

#endif //SEMANTIC_MAPPING_GEOMETRICEDGEMAP_H

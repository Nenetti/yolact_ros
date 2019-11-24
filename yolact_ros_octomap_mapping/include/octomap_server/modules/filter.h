//
// Created by ubuntu on 2019/10/16.
//

#ifndef OCTOMAP_SERVER_FILTER_H
#define OCTOMAP_SERVER_FILTER_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace octomap_server {

    using pcl::PointCloud;
    using pcl::PointXYZRGB;
    using pcl::PointXYZRGBL;
    using pcl::Normal;

    class Filter {

        public:

            static void down_sampling(const PointCloud<PointXYZRGB> &cloud_in, PointCloud<PointXYZRGB> &cloud_out, float resolution);

            static void filterGroundPlane(const PointCloud<PointXYZRGB> &pc, PointCloud<PointXYZRGB> &ground, PointCloud<PointXYZRGB> &non_ground,
                                          float limit);

            static void down_sampling(const PointCloud <PointXYZRGBL> &cloud, PointCloud <PointXYZRGBL> &down_cloud, float size);

            static void infinite_filter(const PointCloud <PointXYZRGB> &cloud, std::vector<bool> &filter);

            static int exclude_filter(const PointCloud <PointXYZRGB> &cloud, std::vector<bool> &filter, double lower_limit, double upper_limit);
    };

}

#endif //OCTOMAP_SERVER_FILTER_H

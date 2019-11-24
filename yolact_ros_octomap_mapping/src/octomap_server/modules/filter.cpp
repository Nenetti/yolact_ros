//
// Created by ubuntu on 2019/10/16.
//

#include <octomap_server/modules/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <tr1/unordered_set>

namespace octomap_server {

    void Filter::down_sampling(const PointCloud<PointXYZRGB> &cloud_in, PointCloud<PointXYZRGB> &cloud_out, float resolution) {
        if (&cloud_in != &cloud_out) {
            cloud_out.resize(cloud_in.size());
        }
        std::tr1::unordered_set<int64_t> hash_set;
        double resolution_factor = 1.0 / resolution;
        int i = 0;
        for (const auto &point:cloud_in) {
            int64_t hash = static_cast<int64_t>(int(resolution_factor * point.z) + 32768) * 345637
                           + (int(resolution_factor * point.y) + 32768) * 1447
                           + (int(resolution_factor * point.x) + 32768);
            if (hash_set.find(hash) != hash_set.end()) {
                continue;
            }
            hash_set.insert(hash);
            cloud_out[i] = point;
            ++i;
        }
        cloud_out.resize(i);
    }

    void Filter::down_sampling(const PointCloud<PointXYZRGBL> &cloud, PointCloud<PointXYZRGBL> &down_cloud, float size) {
        pcl::VoxelGrid<PointXYZRGBL> voxel_grid;
        voxel_grid.setInputCloud(cloud.makeShared());
        voxel_grid.setLeafSize(size, size, size);
        voxel_grid.filter(down_cloud);
    }

    void Filter::filterGroundPlane(const PointCloud<PointXYZRGB> &pc, PointCloud<PointXYZRGB> &ground, PointCloud<PointXYZRGB> &non_ground, float limit) {
        ground.header = pc.header;
        non_ground.header = pc.header;

        pcl::PassThrough<PointXYZRGB> second_pass;
        second_pass.setFilterFieldName("z");
        second_pass.setFilterLimits(-limit, limit);
        second_pass.setInputCloud(pc.makeShared());

        second_pass.filter(ground);

        second_pass.setNegative(true);
        second_pass.filter(non_ground);
    }

    /*******************************************************************************************************************
     * 入力点群の内、XYZの値がInfiniteかどうかを返す
     * @param cloud
     * @param filter
     */
    void Filter::infinite_filter(const PointCloud<PointXYZRGB> &cloud, std::vector<bool> &filter) {
        filter.resize(cloud.size(), false);
        int i = 0;
        for (auto &point:cloud) {
            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
                filter[i] = true;
            }
            ++i;
        }
    }

    /*******************************************************************************************************************
     *
     * @param cloud
     * @param filter
     * @param value
     * @return
     */
    int Filter::exclude_filter(const PointCloud<PointXYZRGB> &cloud, std::vector<bool> &filter, double lower_limit, double upper_limit) {
        filter.resize(cloud.size(), false);
        int count = 0;
        int i = 0;
        for (auto &point:cloud) {
            if (point.z < lower_limit || upper_limit < point.z) {
                filter[i] = true;
                ++count;
            }
            ++i;
        }
        return count;
    }

}
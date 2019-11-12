//
// Created by ubuntu on 2019/10/16.
//

#include <octomap_server/Filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

namespace octomap_server {

    void Filter::down_sampling(const PointCloud<PointXYZRGB> &cloud, PointCloud<PointXYZRGB> &down_cloud, float size) {
        pcl::VoxelGrid<PointXYZRGB> voxel_grid;
        voxel_grid.setInputCloud(cloud.makeShared());
        voxel_grid.setLeafSize(size, size, size);
        voxel_grid.filter(down_cloud);
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
}
/*
 * Copyright (c) 2010-2013, A. Hornung, University of Freiburg
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <octomap_server/octomap_server.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/integral_image_normal.h>

//#include <pcl/point_types.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/search/search.h>
//#include <pcl/search/kdtree.h>
//#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/features/normal_3d.h>

#include <string>
#include <octomap_server/modules/filter.h>

//#include <pcl/point_types.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/search/organized.h>
//#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
//#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.h>
//#include <pcl/features/don.h>
#include <sensor_msgs/Image.h>
#include <pcl/registration/icp.h>
#include <visualization_msgs/MarkerArray.h>
#include <semantic_segmentation/modules/segment.h>

using namespace pcl;
using namespace std;

using octomap_msgs::Octomap;
using custom_octomap::OcTreeKey;
using custom_octomap::KeySet;
using custom_octomap::KeyIntMap;
using custom_octomap::point3d;
using custom_octomap::ColorOcTree;
using custom_octomap::OcTreeNode;
using custom_octomap::OcTreeNode;


bool is_equal(double a, double b, double epsilon = 1.0e-7) {
    return std::abs(a - b) < epsilon;
}

namespace octomap_server {

    /***********************************************************************************************************
     * Constructor
     **********************************************************************************************************/
    OctomapServer::OctomapServer(ros::NodeHandle private_nh_) :
            m_nh(),
            m_pointCloudSub(nullptr),
            m_tfPointCloudSub(nullptr),
            m_reconfigureServer(m_config_mutex),
            m_octree(nullptr),
            m_maxRange(-1.0),
            m_worldFrameId("/map"), m_baseFrameId("base_footprint"),
            m_useHeightMap(true),
            m_useColoredMap(false),
            m_colorFactor(0.8),
            m_latchedTopics(true),
            m_publishFreeSpace(false),
            m_clusterDistance(0.05),
            m_pointColorThreshold(6),
            m_regionColorThreshold(5),
            m_change_factor(0.01),
            m_smoothing_size(0.01),
            m_res(0.05),
            m_low_resolution(0.04),
            m_high_resolution(0.01),
            m_search_radius(0.01),
            m_treeDepth(0),
            m_maxTreeDepth(0),
            m_cluster_normal_threshold(0.1),
            m_pointcloudMinX(-std::numeric_limits<double>::max()),
            m_pointcloudMaxX(std::numeric_limits<double>::max()),
            m_pointcloudMinY(-std::numeric_limits<double>::max()),
            m_pointcloudMaxY(std::numeric_limits<double>::max()),
            m_pointcloudMinZ(-std::numeric_limits<double>::max()),
            m_pointcloudMaxZ(std::numeric_limits<double>::max()),
            m_occupancyMinZ(-std::numeric_limits<double>::max()),
            m_occupancyMaxZ(std::numeric_limits<double>::max()),
            m_minSizeX(0.0), m_minSizeY(0.0),
            m_filterSpeckles(false), m_filterGroundPlane(false),
            m_ground_filter_distance(0.04), m_groundFilterAngle(0.15), m_groundFilterPlaneDistance(0.07),
            m_ceiling_filter_distance(2.0),
            m_downSampling(true),
            m_downSamplingSize(0.01),
            m_normal_color(false),
            m_compressMap(true),
            m_incrementalUpdate(false),
            m_initConfig(true),
            m_frameRate(1.0) {

        double probHit, probMiss, thresMin, thresMax;

        ros::NodeHandle private_nh(private_nh_);
        private_nh.param("frame_id", m_worldFrameId, m_worldFrameId);
        private_nh.param("base_frame_id", m_baseFrameId, m_baseFrameId);
        private_nh.param("height_map", m_useHeightMap, m_useHeightMap);
        private_nh.param("colored_map", m_useColoredMap, m_useColoredMap);
        private_nh.param("color_factor", m_colorFactor, m_colorFactor);

        private_nh.param("pointcloud_min_x", m_pointcloudMinX, m_pointcloudMinX);
        private_nh.param("pointcloud_max_x", m_pointcloudMaxX, m_pointcloudMaxX);
        private_nh.param("pointcloud_min_y", m_pointcloudMinY, m_pointcloudMinY);
        private_nh.param("pointcloud_max_y", m_pointcloudMaxY, m_pointcloudMaxY);
        private_nh.param("pointcloud_min_z", m_pointcloudMinZ, m_pointcloudMinZ);
        private_nh.param("pointcloud_max_z", m_pointcloudMaxZ, m_pointcloudMaxZ);
        private_nh.param("occupancy_min_z", m_occupancyMinZ, m_occupancyMinZ);
        private_nh.param("occupancy_max_z", m_occupancyMaxZ, m_occupancyMaxZ);
        private_nh.param("min_x_size", m_minSizeX, m_minSizeX);
        private_nh.param("min_y_size", m_minSizeY, m_minSizeY);

        private_nh.param("filter_speckles", m_filterSpeckles, m_filterSpeckles);
        private_nh.param("filter_ground", m_filterGroundPlane, m_filterGroundPlane);
        // distance of points from plane for RANSAC
        private_nh.param("ground_filter/distance", m_ground_filter_distance, m_ground_filter_distance);
        // angular derivation of found plane:
        private_nh.param("ground_filter/angle", m_groundFilterAngle, m_groundFilterAngle);
        // distance of found plane from z=0 to be detected as ground (e.g. to exclude tables)
        private_nh.param("ground_filter/plane_distance", m_groundFilterPlaneDistance, m_groundFilterPlaneDistance);

        private_nh.param("sensor_model/max_range", m_maxRange, m_maxRange);

        private_nh.param("down_sampling", m_downSampling, m_downSampling);
        private_nh.param("down_sampling_size", m_downSamplingSize, m_downSamplingSize);

        private_nh.param("cluster_distance", m_clusterDistance, m_clusterDistance);
        private_nh.param("point_color_threshold", m_pointColorThreshold, m_pointColorThreshold);
        private_nh.param("region_color_threshold", m_regionColorThreshold, m_regionColorThreshold);

        private_nh.param("change_factor", m_change_factor, m_change_factor);
        private_nh.param("smoothing_size", m_smoothing_size, m_smoothing_size);

        private_nh.param("search_radius", m_search_radius, m_search_radius);

        private_nh.param("cluster_normal_threshold", m_cluster_normal_threshold, m_cluster_normal_threshold);

        private_nh.param("normal_color", m_normal_color, m_normal_color);

        private_nh.param("resolution", m_res, m_res);
        private_nh.param("sensor_model/hit", probHit, 0.7);
        private_nh.param("sensor_model/miss", probMiss, 0.4);
        private_nh.param("sensor_model/min", thresMin, 0.12);
        private_nh.param("sensor_model/max", thresMax, 0.97);
        private_nh.param("compress_map", m_compressMap, m_compressMap);
        private_nh.param("incremental_2D_projection", m_incrementalUpdate, m_incrementalUpdate);

        private_nh.param("frame_rate", m_frameRate, m_frameRate);

        if (m_filterGroundPlane && (m_pointcloudMinZ > 0.0 || m_pointcloudMaxZ < 0.0)) {
            ROS_WARN_STREAM("You enabled ground filtering but incoming pointclouds will be pre-filtered in ["
                                    << m_pointcloudMinZ << ", " << m_pointcloudMaxZ << "], excluding the ground level z=0. "
                                    << "This will not work.");
        }

        if (m_useHeightMap && m_useColoredMap) {
            ROS_WARN_STREAM("You enabled both height map and RGB color registration. This is contradictory. Defaulting to height map.");
            m_useColoredMap = false;
        }

        if (m_useColoredMap) {
            ROS_INFO_STREAM("Using RGB color registration (if information available)");
        }


        // initialize custom_octomap object & params
        m_octree = new ColorOcTree(m_res);
        m_octree->setProbHit(probHit);
        m_octree->setProbMiss(probMiss);
        m_octree->setClampingThresMin(thresMin);
        m_octree->setClampingThresMax(thresMax);
        m_octree->createRoot();
        m_treeDepth = m_octree->getTreeDepth();
        m_maxTreeDepth = m_treeDepth;
        m_gridmap.info.resolution = m_res;

        m_low_resolution_factor = 1. / m_low_resolution;
        m_high_resolution_factor = 1. / m_high_resolution;
        m_tree_max_val = 32768;

        double r, g, b, a;
        private_nh.param("color/r", r, 0.0);
        private_nh.param("color/g", g, 0.0);
        private_nh.param("color/b", b, 1.0);
        private_nh.param("color/a", a, 1.0);
        m_color.r = r;
        m_color.g = g;
        m_color.b = b;
        m_color.a = a;

        private_nh.param("color_free/r", r, 0.0);
        private_nh.param("color_free/g", g, 1.0);
        private_nh.param("color_free/b", b, 0.0);
        private_nh.param("color_free/a", a, 1.0);
        m_colorFree.r = r;
        m_colorFree.g = g;
        m_colorFree.b = b;
        m_colorFree.a = a;

        std::random_device rnd;
        std::mt19937 mt(rnd());
        std::uniform_int_distribution<> rand100(0, 255);
        for (int i = 0; i < 320000; ++i) {
            custom_octomap::Color color(rand100(mt), rand100(mt), rand100(mt));
            cluster_colors.emplace_back(color);
        }


        private_nh.param("publish_free_space", m_publishFreeSpace, m_publishFreeSpace);

        private_nh.param("latch", m_latchedTopics, m_latchedTopics);
        if (m_latchedTopics) {
            ROS_INFO("Publishing latched (single publish will take longer, all topics are prepared)");
        } else {
            ROS_INFO("Publishing non-latched (topics are only prepared as needed, will only be re-published on map change");
        }

        m_markerPub = m_nh.advertise<visualization_msgs::MarkerArray>("occupied_cells_vis_array", 1, m_latchedTopics);
        m_binaryMapPub = m_nh.advertise<Octomap>("octomap_binary", 1, m_latchedTopics);
        m_fullMapPub = m_nh.advertise<Octomap>("octomap_full", 1, m_latchedTopics);
        m_pointCloudPub = m_nh.advertise<sensor_msgs::PointCloud2>("octomap_point_cloud_centers", 1, m_latchedTopics);
        m_mapPub = m_nh.advertise<nav_msgs::OccupancyGrid>("projected_map", 5, m_latchedTopics);
        m_fmarkerPub = m_nh.advertise<visualization_msgs::MarkerArray>("free_cells_vis_array", 1, m_latchedTopics);

        m_pointCloudSub = new message_filters::Subscriber<sensor_msgs::PointCloud2>(m_nh, "cloud_in", 5);
        m_tfPointCloudSub = new tf::MessageFilter<sensor_msgs::PointCloud2>(*m_pointCloudSub, m_tfListener, m_worldFrameId, 5);
        m_tfPointCloudSub->registerCallback(boost::bind(&OctomapServer::insertCloudCallback, this, _1));

        m_testPub = m_nh.advertise<sensor_msgs::Image>("test_pub", 1, m_latchedTopics);

        m_octomapBinaryService = m_nh.advertiseService("octomap_binary", &OctomapServer::octomapBinarySrv, this);
        m_octomapFullService = m_nh.advertiseService("octomap_full", &OctomapServer::octomapFullSrv, this);
        m_clearBBXService = private_nh.advertiseService("clear_bbx", &OctomapServer::clearBBXSrv, this);
        m_resetService = private_nh.advertiseService("reset", &OctomapServer::resetSrv, this);


        dynamic_reconfigure::Server<OctomapServerConfig>::CallbackType f;
        f = boost::bind(&OctomapServer::reconfigureCallback, this, _1, _2);
        m_reconfigureServer.setCallback(f);
    }

    OctomapServer::~OctomapServer() {
        if (m_tfPointCloudSub) {
            delete m_tfPointCloudSub;
            m_tfPointCloudSub = nullptr;
        }

        if (m_pointCloudSub) {
            delete m_pointCloudSub;
            m_pointCloudSub = nullptr;
        }


        if (m_octree) {
            delete m_octree;
            m_octree = nullptr;
        }

    }

    /***********************************************************************************************************
     * Main Process
     **********************************************************************************************************/
    void OctomapServer::insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg) {
        //************************************************************************************************************//
        // Skip Process
        //************************************************************************************************************//
        ros::WallTime startTime = ros::WallTime::now();
        ROS_INFO("Subscribe");
        if ((startTime - beforeTime).toSec() < m_frameRate) {
            ROS_INFO("Skip");
            return;
        } else {
            beforeTime = startTime;
        }
        //************************************************************************************************************//
        // TF
        //************************************************************************************************************//
        tf::StampedTransform sensorToWorldTf;
        for (int i = 1; i <= 5; ++i) {
            try {
                m_tfListener.lookupTransform(m_worldFrameId, cloud_msg->header.frame_id, cloud_msg->header.stamp, sensorToWorldTf);
                break;
            } catch (exception e) {
//                ROS_WARN("%s", e.what());
                if (i == 5) {
                    ROS_ERROR("Failed LookupTransform");
                    return;
                }
            }
        }

        //************************************************************************************************************//
        // Segmentation
        //************************************************************************************************************//
        std::vector<semantic_segmentation::SemanticObject> segments;
        bool is_succeed = m_semantic_segmentation_client.send_segmentation_request(cloud_msg, segments);
        if (is_succeed) {
            ROS_INFO("Segmentation Succeed done %f sec)", (ros::WallTime::now() - startTime).toSec());
        } else {
            ROS_INFO("Segmentation Failed");
        }

        //************************************************************************************************************//
        // PointCloud
        //************************************************************************************************************//
        PointCloud<PointXYZRGB> cloud;
        PointCloud<PointXYZRGB> excluded_cloud;
        PointCloud<PointXYZRGB> uncategorized_cloud;
        sensor_msgs::PointCloud2 base_cloud;
        sensor_msgs::PointCloud2 world_cloud;
        pcl_ros::transformPointCloud(m_baseFrameId, *cloud_msg, base_cloud, m_tfListener);
        pcl_ros::transformPointCloud("/odom", base_cloud, world_cloud, m_tfListener);
        pcl::fromROSMsg(world_cloud, cloud);

        cloud_classification(cloud, excluded_cloud, uncategorized_cloud, segments);

        insertScan(sensorToWorldTf.getOrigin(), excluded_cloud, uncategorized_cloud, segments);

        ROS_INFO("Pointcloud insertion in octomap_server done %f sec)", (ros::WallTime::now() - startTime).toSec());
        publishAll(cloud_msg->header.stamp);

    }

    void OctomapServer::cloud_classification(const PointCloud<PointXYZRGB> &cloud,
                                             PointCloud<PointXYZRGB> &excluded_cloud,
                                             PointCloud<PointXYZRGB> &uncategorized_cloud,
                                             std::vector<semantic_segmentation::SemanticObject> &segments) {
        std::vector<bool> is_excluded;
        std::vector<bool> is_infinite;
        int excluded_size = Filter::exclude_filter(cloud, is_excluded, m_ground_filter_distance, m_ceiling_filter_distance);
        Filter::infinite_filter(cloud, is_infinite);
        int excluded_index = 0;
        int uncategorized_index = 0;
        excluded_cloud.resize(excluded_size);
        uncategorized_cloud.resize(cloud.size() - excluded_cloud.size());
        std::vector<int> is_segments(cloud.size(), false);
        for (auto &segment:segments) {
            segment.cloud.resize(segment.masks.size());
            for (int i = 0; i < int(segment.masks.size()); ++i) {
                auto index = segment.masks[i];
                auto &point = cloud[index];
                segment.cloud[i] = point;
                is_segments[index] = true;
            }
        }
        for (int i = 0; i < int(cloud.size()); ++i) {
            if (is_infinite[i]) {
                continue;
            }
            auto &point = cloud[i];
            if (is_excluded[i]) {
                excluded_cloud[excluded_index] = point;
                ++excluded_index;
                continue;
            }
            if (!is_segments[i]) {
                uncategorized_cloud[uncategorized_index] = point;
                ++uncategorized_index;
            }
        }
        excluded_cloud.resize(excluded_index);
        uncategorized_cloud.resize(uncategorized_index);
    }

    void OctomapServer::insertScan(const tf::Point &sensorOriginTf,
                                   const PointCloud<PointXYZRGB> &excluded_cloud,
                                   const PointCloud<PointXYZRGB> &uncategorized_cloud,
                                   const std::vector<semantic_segmentation::SemanticObject> &segments) {
        //************************************************************************************************************//
        // Check data
        //************************************************************************************************************//
        Point3D sensorOrigin = octomap::pointTfToOctomap(sensorOriginTf);
        Point3D check_lower_limit, check_upper_limit;
        if (m_maxRange < 0.0) {
            ROS_ERROR("Could not generate Key for max_range = %lf < 0", m_maxRange);
            return;
        }
        if (!m_octree->isCoordinateAvailable(check_lower_limit) || !m_octree->isCoordinateAvailable(check_upper_limit)) {
            ROS_ERROR("Could not generate Key for near the boundary");
            return;
        }
        if (!m_octree->coordToKeyChecked(sensorOrigin, m_updateBBXMin) || !m_octree->coordToKeyChecked(sensorOrigin, m_updateBBXMax)) {
            ROS_ERROR_STREAM("Could not generate Key for origin " << sensorOrigin);
            return;
        }
        //************************************************************************************************************//
        // Init
        //************************************************************************************************************//
        ros::WallTime startTime = ros::WallTime::now();
        OcTreeKey origin_key, oc_tree_key;
        KeySet free_cells, occupied_cells, hash_set;
        KeyIntMap cluster_map;
        m_octree->coordToKey(sensorOrigin, origin_key);

        //************************************************************************************************************//
        // Excluded Points process
        //************************************************************************************************************//
        //#pragma omp parallel for
        for (const auto &point:excluded_cloud) {
            Point3D oc_point(point.x, point.y, point.z);
            m_octree->coordToKey(oc_point, oc_tree_key);
            if (hash_set.find(oc_tree_key) == hash_set.end()) {
                continue;
            }
            hash_set.insert(oc_tree_key);
            if (m_octree->computeRayKeys(origin_key, oc_tree_key, m_keyRay)) {
                free_cells.insert(m_keyRay.begin(), m_keyRay.end());
            }
            updateMinKey(oc_tree_key, m_updateBBXMin);
            updateMaxKey(oc_tree_key, m_updateBBXMax);
        }
        ROS_INFO("Excluded Points done %f sec)", (ros::WallTime::now() - startTime).toSec());

        //************************************************************************************************************//
        // uncategorized data process
        //************************************************************************************************************//
        for (const auto &point : uncategorized_cloud) {
            Point3D oc_point(point.x, point.y, point.z);
            m_octree->coordToKey(oc_point, oc_tree_key);

            if (hash_set.find(oc_tree_key) != hash_set.end()) {
                continue;
            }
            hash_set.insert(oc_tree_key);
            if (hash_set.find(oc_tree_key) == hash_set.end()) {
                hash_set.insert(oc_tree_key);
                if (m_octree->computeRayKeys(origin_key, oc_tree_key, m_keyRay)) {
                    free_cells.insert(m_keyRay.begin(), m_keyRay.end());
                }
                occupied_cells.insert(oc_tree_key);
                updateMinKey(oc_tree_key, m_updateBBXMin);
                updateMaxKey(oc_tree_key, m_updateBBXMax);
            }
            m_octree->updateNodeValue(oc_tree_key, true);
        }
        ROS_INFO("Uncategorized Data done %f sec)", (ros::WallTime::now() - startTime).toSec());

        //************************************************************************************************************//
        // Segmentation data process
        //************************************************************************************************************//
        //#pragma omp parallel for
        for (const auto &segment:segments) {
            KeySet cells;
            std::tr1::unordered_set<int> id_set;
            for (const auto &point:segment.cloud) {
                Point3D oc_point(point.x, point.y, point.z);
                m_octree->coordToKey(oc_point, oc_tree_key);
                if (hash_set.find(oc_tree_key) != hash_set.end()) {
                    continue;
                }
                if (hash_set.find(oc_tree_key) == hash_set.end()) {
                    hash_set.insert(oc_tree_key);
                    if (m_octree->computeRayKeys(origin_key, oc_tree_key, m_keyRay)) {
                        free_cells.insert(m_keyRay.begin(), m_keyRay.end());
                    }
                    occupied_cells.insert(oc_tree_key);
                    cells.insert(oc_tree_key);
                    updateMinKey(oc_tree_key, m_updateBBXMin);
                    updateMaxKey(oc_tree_key, m_updateBBXMax);
                    OcTreeNode *node = m_octree->updateNodeValue(oc_tree_key, true);
                    int id = node->get_id(segment.Class);
                    if (id >= 0) {
                        id_set.insert(id);
                    }
                }
            }
            // 取得してきた範囲にあるIDのうち最小のものを取得してくる
            // IDが存在しない場合は、新たにIDを生成
            int object_id = INT_MAX;
            if (id_set.empty()) {
                object_id = m_number_of_objects;
                ++m_number_of_objects;
            } else {
                for (auto key:id_set) {
                    if (object_id > key) {
                        object_id = key;
                    }
                }
            }
            // 取得してきたIDが複数ある場合すべてのIDを上書きする
            if (id_set.size() > 1) {
                auto nodes = m_octree->get_semantic_nodes(object_id);
                if (nodes != nullptr) {
                    for (auto &key:*nodes) {
                        OcTreeNode *node = m_octree->get_node(key);
                        node->update_label_probability(segment.Class, object_id, segment.probability);
                    }
                }
            }
            printf("%s, %d, %zu\n", segment.Class.data(), object_id, id_set.size());
            for (auto &key:cells) {
                OcTreeNode *node = m_octree->get_node(key);
                node->update_label_probability(segment.Class, object_id, segment.probability);
                auto map = m_octree->semantic_node_map.find(object_id);
                if (map == m_octree->semantic_node_map.end()) {
                    m_octree->semantic_node_map[object_id] = std::tr1::unordered_set<OcTreeKey>();
                    map = m_octree->semantic_node_map.find(object_id);
                }
                (*map).second.insert(key);
            }
        }
        ROS_INFO("Segmentation Data done %f sec)", (ros::WallTime::now() - startTime).toSec());

        //************************************************************************************************************//
        // Cell update
        //************************************************************************************************************//
        for (auto &key:free_cells) {
            if (occupied_cells.find(key) == occupied_cells.end()) {
                m_octree->updateNodeValue(key, false);
            }
        }
//        for (auto &key:occupied_cells) {
//            m_octree->pruneNode(key);
//        }
//
//        for (auto &key:free_cells) {
//            if (occupied_cells.find(key) == occupied_cells.end()) {
//                m_octree->pruneNode(key);
//            }
//        }
        ROS_INFO("Occupied = %zu,  Free = %zu", occupied_cells.size(), free_cells.size());
        if (m_compressMap) {
            m_octree->prune();
        }

//        update_occupied_cells = occupied_cells;
    }


/***********************************************************************************************************
 * Publish
 **********************************************************************************************************/
    void OctomapServer::publishAll(const ros::Time &rostime) {
        ros::WallTime startTime = ros::WallTime::now();
        size_t octomapSize = m_octree->size();
        // TODO: estimate num occ. voxels for size of arrays (reserve)
        if (octomapSize <= 1) {
            ROS_WARN("Nothing to publish, octree is empty");
            return;
        }

//        bool publishFreeMarkerArray = m_publishFreeSpace && (m_latchedTopics || m_fmarkerPub.getNumSubscribers() > 0);
//        bool publishMarkerArray = (m_latchedTopics || m_markerPub.getNumSubscribers() > 0);
//        bool publishPointCloud = (m_latchedTopics || m_pointCloudPub.getNumSubscribers() > 0);
//        bool publishBinaryMap = (m_latchedTopics || m_binaryMapPub.getNumSubscribers() > 0);
        bool publishFullMap = (m_latchedTopics || m_fullMapPub.getNumSubscribers() > 0);
        for (iterator it = m_octree->begin(m_maxTreeDepth), end = m_octree->end(); it != end; ++it) {
            int r = it->getColor().r;
            int g = it->getColor().g;
            int b = it->getColor().b;
//            ROS_INFO("(%d, %d, %d) -> %lf", r, g, b, it->getLogOdds());
        }
//        m_publish2DMap = (m_latchedTopics || m_mapPub.getNumSubscribers() > 0);
//
//        // init markers for free space:
//        visualization_msgs::MarkerArray freeNodesVis;
//        // each array stores all cubes of a different size, one for each depth level:
//        freeNodesVis.markers.resize(m_treeDepth + 1);
//
//        geometry_msgs::Pose pose;
//        pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
//
//        // init markers:
//        visualization_msgs::MarkerArray occupiedNodesVis;
//        // each array stores all cubes of a different size, one for each depth level:
//        occupiedNodesVis.markers.resize(m_treeDepth + 1);
//
//        // init pointcloud:
//        pcl::PointCloud<PointXYZRGB> pclCloud;
//
//        // call pre-traversal hook:
//        handlePreNodeTraversal(rostime);
//
//        // now, traverse all leafs in the tree:
//        for (iterator it = m_octree->begin(m_maxTreeDepth), end = m_octree->end(); it != end; ++it) {
//            bool inUpdateBBX = isInUpdateBBX(it);
//
//            // call general hook:
//            handleNode(it);
//            if (inUpdateBBX) {
//                handleNodeInBBX(it);
//            }
//            if (m_octree->isNodeOccupied(*it)) {
//                double z = it.getZ();
//                double half_size = it.getSize() / 2.0;
//                if (z + half_size > m_occupancyMinZ && z - half_size < m_occupancyMaxZ) {
//                    double size = it.getSize();
//                    double x = it.getX();
//                    double y = it.getY();
//                    int r = it->getColor().r;
//                    int g = it->getColor().g;
//                    int b = it->getColor().b;
////                    ROS_INFO("(%lf, %lf, %lf) -> %lf", x, y, z, it->getLogOdds());
//
//                    // Ignore speckles in the map:
//                    if (m_filterSpeckles && (it.getDepth() == m_treeDepth + 1) && isSpeckleNode(it.getKey())) {
//                        ROS_DEBUG("Ignoring single speckle at (%f,%f,%f)", x, y, z);
//                        continue;
//                    } // else: current octree node is no speckle, send it out
//
//                    handleOccupiedNode(it);
//                    if (inUpdateBBX) {
//                        handleOccupiedNodeInBBX(it);
//                    }
//
//
//                    //create marker:
//                    if (publishMarkerArray) {
//                        unsigned idx = it.getDepth();
//                        assert(idx < occupiedNodesVis.markers.size());
//
//                        geometry_msgs::Point cubeCenter;
//                        cubeCenter.x = x;
//                        cubeCenter.y = y;
//                        cubeCenter.z = z;
//
//                        occupiedNodesVis.markers[idx].points.push_back(cubeCenter);
//                        if (m_useHeightMap) {
//                            double minX, minY, minZ, maxX, maxY, maxZ;
//                            m_octree->getMetricMin(minX, minY, minZ);
//                            m_octree->getMetricMax(maxX, maxY, maxZ);
//
//                            double h = (1.0 - std::min(std::max((cubeCenter.z - minZ) / (maxZ - minZ), 0.0), 1.0)) * m_colorFactor;
//                            occupiedNodesVis.markers[idx].colors.push_back(heightMapColor(h));
//                        }
//
//                        if (m_useColoredMap) {
//                            std_msgs::ColorRGBA _color;
//                            _color.r = (r / 255.);
//                            _color.g = (g / 255.);
//                            _color.b = (b / 255.);
//                            _color.a = 1.0; // TODO/EVALUATE: potentially use occupancy as measure for alpha channel?
//                            occupiedNodesVis.markers[idx].colors.push_back(_color);
//                        }
//                    }
//
//                    // insert into pointcloud:
//                    if (publishPointCloud) {
//                        PointXYZRGB _point = PointXYZRGB();
//                        _point.x = x;
//                        _point.y = y;
//                        _point.z = z;
//                        _point.r = r;
//                        _point.g = g;
//                        _point.b = b;
//                        pclCloud.push_back(_point);
//                    }
//
//                }
//            } else { // node not occupied => mark as free in 2D map if unknown so far
//                double z = it.getZ();
//                double half_size = it.getSize() / 2.0;
//                if (z + half_size > m_occupancyMinZ && z - half_size < m_occupancyMaxZ) {
//                    handleFreeNode(it);
//                    if (inUpdateBBX) {
//                        handleFreeNodeInBBX(it);
//                    }
//
//                    if (m_publishFreeSpace) {
//                        double x = it.getX();
//                        double y = it.getY();
//
//                        //create marker for free space:
//                        if (publishFreeMarkerArray) {
//                            unsigned idx = it.getDepth();
//                            assert(idx < freeNodesVis.markers.size());
//
//                            geometry_msgs::Point cubeCenter;
//                            cubeCenter.x = x;
//                            cubeCenter.y = y;
//                            cubeCenter.z = z;
//
//                            freeNodesVis.markers[idx].points.push_back(cubeCenter);
//                        }
//                    }
////                    if (publishPointCloud) {
////                        double x = it.getX();
////                        double y = it.getY();
////                        PCLPoint _point = PCLPoint();
////                        _point.x = x;
////                        _point.y = y;
////                        _point.z = z;
////                        _point.r = 255;
////                        _point.g = 0;
////                        _point.b = 0;
////                        pclCloud.push_back(_point);
////                    }
//
//                }
//            }
//        }
//
//        // call post-traversal hook:
//        handlePostNodeTraversal(rostime);
//
//        // finish MarkerArray:
//        if (publishMarkerArray) {
//            for (unsigned i = 0; i < occupiedNodesVis.markers.size(); ++i) {
//                double size = m_octree->getNodeSize(i);
//
//                occupiedNodesVis.markers[i].header.frame_id = m_worldFrameId;
//                occupiedNodesVis.markers[i].header.stamp = rostime;
//                occupiedNodesVis.markers[i].ns = "map";
//                occupiedNodesVis.markers[i].id = i;
//                occupiedNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
//                occupiedNodesVis.markers[i].scale.x = size;
//                occupiedNodesVis.markers[i].scale.y = size;
//                occupiedNodesVis.markers[i].scale.z = size;
//                if (!m_useColoredMap) {
//                    occupiedNodesVis.markers[i].color = m_color;
//                }
//
//
//                if (occupiedNodesVis.markers[i].points.size() > 0) {
//                    occupiedNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
//                } else {
//                    occupiedNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
//                }
//            }
//
//            m_markerPub.publish(occupiedNodesVis);
//        }
//
//
//        // finish FreeMarkerArray:
//        if (publishFreeMarkerArray) {
//            for (unsigned i = 0; i < freeNodesVis.markers.size(); ++i) {
//                double size = m_octree->getNodeSize(i);
//
//                freeNodesVis.markers[i].header.frame_id = m_worldFrameId;
//                freeNodesVis.markers[i].header.stamp = rostime;
//                freeNodesVis.markers[i].ns = "map";
//                freeNodesVis.markers[i].id = i;
//                freeNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
//                freeNodesVis.markers[i].scale.x = size;
//                freeNodesVis.markers[i].scale.y = size;
//                freeNodesVis.markers[i].scale.z = size;
//                freeNodesVis.markers[i].color = m_colorFree;
//
//
//                if (freeNodesVis.markers[i].points.size() > 0) {
//                    freeNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
//                } else {
//                    freeNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
//                }
//            }
//
//            m_fmarkerPub.publish(freeNodesVis);
//        }


        // finish pointcloud:
//        if (publishPointCloud) {
//            sensor_msgs::PointCloud2 cloud;
//            pcl::toROSMsg(pclCloud, cloud);
//            cloud.header.frame_id = m_worldFrameId;
//            cloud.header.stamp = rostime;
//            m_pointCloudPub.publish(cloud);
//        }

//        if (publishBinaryMap) {
//            publishBinaryOctoMap(rostime);
//        }

        if (publishFullMap) {
            publishFullOctoMap(rostime);
        }


        double total_elapsed = (ros::WallTime::now() - startTime).toSec();
        ROS_INFO("Map publishing in octomap_server took %f sec", total_elapsed);

    }


    /***********************************************************************************************************
     * File
     **********************************************************************************************************/
    bool OctomapServer::openFile(const std::string &filename) {
//        if (filename.length() <= 3) {
//            return false;
//        }
//
//        std::string suffix = filename.substr(filename.length() - 3, 3);
//        if (suffix == ".bt") {
//            if (!m_octree->readBinary(filename)) {
//                return false;
//            }
//        } else if (suffix == ".ot") {
//            AbstractColorOcTree *tree = AbstractColorOcTree::read(filename);
//            if (!tree) {
//                return false;
//            }
//            if (m_octree) {
//                delete m_octree;
//                m_octree = NULL;
//            }
//            m_octree = dynamic_cast<ColorColorOcTree *>(tree);
//            if (!m_octree) {
//                ROS_ERROR("Could not read ColorOcTree in file, currently there are no other types supported in .ot");
//                return false;
//            }
//
//        } else {
//            return false;
//        }
//
//        ROS_INFO("Octomap file %s loaded (%zu nodes).", filename.c_str(), m_octree->size());
//
//        m_treeDepth = m_octree->getTreeDepth();
//        m_maxTreeDepth = m_treeDepth;
//        m_res = m_octree->getResolution();
//        m_gridmap.info.resolution = m_res;
//        double minX, minY, minZ;
//        double maxX, maxY, maxZ;
//        m_octree->getMetricMin(minX, minY, minZ);
//        m_octree->getMetricMax(maxX, maxY, maxZ);
//
//        m_updateBBXMin[0] = m_octree->coordToKey(minX);
//        m_updateBBXMin[1] = m_octree->coordToKey(minY);
//        m_updateBBXMin[2] = m_octree->coordToKey(minZ);
//
//        m_updateBBXMax[0] = m_octree->coordToKey(maxX);
//        m_updateBBXMax[1] = m_octree->coordToKey(maxY);
//        m_updateBBXMax[2] = m_octree->coordToKey(maxZ);
//
//        publishAll();

        return true;

    }

    void OctomapServer::publishBinaryOctoMap(const ros::Time &rostime) const {

        Octomap map;
        map.header.frame_id = m_worldFrameId;
        map.header.stamp = rostime;

        if (octomap_msgs::binaryMapToMsg(*m_octree, map)) {
            m_binaryMapPub.publish(map);
        } else {
            ROS_ERROR("Error serializing OctoMap");
        }
    }

    void OctomapServer::publishFullOctoMap(const ros::Time &rostime) const {

        Octomap map;
        map.header.frame_id = m_worldFrameId;
        map.header.stamp = rostime;

        if (octomap_msgs::fullMapToMsg(*m_octree, map)) {
            m_fullMapPub.publish(map);
        } else {
            ROS_ERROR("Error serializing OctoMap");
        }

    }

    /***********************************************************************************************************
     * Service Callback
     **********************************************************************************************************/
    bool OctomapServer::octomapBinarySrv(OctomapSrv::Request &req,
                                         OctomapSrv::Response &res) {
        ros::WallTime startTime = ros::WallTime::now();
        ROS_INFO("Sending binary map data on service request");
        res.map.header.frame_id = m_worldFrameId;
        res.map.header.stamp = ros::Time::now();
        if (!octomap_msgs::binaryMapToMsg(*m_octree, res.map)) {
            return false;
        }

        double total_elapsed = (ros::WallTime::now() - startTime).toSec();
        ROS_INFO("Binary custom_octomap sent in %f sec", total_elapsed);
        return true;
    }

    bool OctomapServer::octomapFullSrv(OctomapSrv::Request &req,
                                       OctomapSrv::Response &res) {
        ROS_INFO("Sending full map data on service request");
        res.map.header.frame_id = m_worldFrameId;
        res.map.header.stamp = ros::Time::now();

        if (!octomap_msgs::fullMapToMsg(*m_octree, res.map)) {
            return false;
        }

        return true;
    }

    bool OctomapServer::clearBBXSrv(BBXSrv::Request &req, BBXSrv::Response &resp) {
        Point3D min = octomap::pointMsgToOctomap(req.min);
        Point3D max = octomap::pointMsgToOctomap(req.max);

        double thresMin = m_octree->getClampingThresMin();
        for (ColorOcTree::leaf_bbx_iterator it = m_octree->begin_leafs_bbx(min, max),
                     end = m_octree->end_leafs_bbx(); it != end; ++it) {

            it->setLogOdds(octomap::logodds(thresMin));
            //			m_octree->updateNode(it.getKey(), -6.0f);
        }
        // TODO: eval which is faster (setLogOdds+updateInner or updateNode)
        m_octree->updateInnerOccupancy();

        publishAll(ros::Time::now());

        return true;
    }

    bool OctomapServer::resetSrv(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp) {
        visualization_msgs::MarkerArray occupiedNodesVis;
        occupiedNodesVis.markers.resize(m_treeDepth + 1);
        ros::Time rostime = ros::Time::now();
        m_octree->clear();
        // clear 2D map:
        m_gridmap.data.clear();
        m_gridmap.info.height = 0.0;
        m_gridmap.info.width = 0.0;
        m_gridmap.info.resolution = 0.0;
        m_gridmap.info.origin.position.x = 0.0;
        m_gridmap.info.origin.position.y = 0.0;

        ROS_INFO("Cleared custom_octomap");
        publishAll(rostime);

        publishBinaryOctoMap(rostime);
        for (unsigned i = 0; i < occupiedNodesVis.markers.size(); ++i) {

            occupiedNodesVis.markers[i].header.frame_id = m_worldFrameId;
            occupiedNodesVis.markers[i].header.stamp = rostime;
            occupiedNodesVis.markers[i].ns = "map";
            occupiedNodesVis.markers[i].id = i;
            occupiedNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
            occupiedNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
        }

        m_markerPub.publish(occupiedNodesVis);

        visualization_msgs::MarkerArray freeNodesVis;
        freeNodesVis.markers.resize(m_treeDepth + 1);

        for (unsigned i = 0; i < freeNodesVis.markers.size(); ++i) {

            freeNodesVis.markers[i].header.frame_id = m_worldFrameId;
            freeNodesVis.markers[i].header.stamp = rostime;
            freeNodesVis.markers[i].ns = "map";
            freeNodesVis.markers[i].id = i;
            freeNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
            freeNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
        }
        m_fmarkerPub.publish(freeNodesVis);

        return true;
    }

/***********************************************************************************************************
 * Handle
 **********************************************************************************************************/
    void OctomapServer::handlePreNodeTraversal(const ros::Time &rostime) {
        if (m_publish2DMap) {
            // init projected 2D map:
            m_gridmap.header.frame_id = m_worldFrameId;
            m_gridmap.header.stamp = rostime;
            nav_msgs::MapMetaData oldMapInfo = m_gridmap.info;

            // TODO: move most of this stuff into c'tor and init map only once (adjust if size changes)
            double minX, minY, minZ, maxX, maxY, maxZ;
            m_octree->getMetricMin(minX, minY, minZ);
            m_octree->getMetricMax(maxX, maxY, maxZ);

            Point3D minPt(minX, minY, minZ);
            Point3D maxPt(maxX, maxY, maxZ);
            OcTreeKey minKey = m_octree->coordToKey(minPt, m_maxTreeDepth);
            OcTreeKey maxKey = m_octree->coordToKey(maxPt, m_maxTreeDepth);

            ROS_DEBUG("MinKey: %d %d %d / MaxKey: %d %d %d", minKey[0], minKey[1], minKey[2], maxKey[0], maxKey[1], maxKey[2]);

            // add padding if requested (= new min/maxPts in x&y):
            double halfPaddedX = 0.5 * m_minSizeX;
            double halfPaddedY = 0.5 * m_minSizeY;
            minX = std::min(minX, -halfPaddedX);
            maxX = std::max(maxX, halfPaddedX);
            minY = std::min(minY, -halfPaddedY);
            maxY = std::max(maxY, halfPaddedY);
            minPt = Point3D(minX, minY, minZ);
            maxPt = Point3D(maxX, maxY, maxZ);

            OcTreeKey paddedMaxKey;
            if (!m_octree->coordToKeyChecked(minPt, m_maxTreeDepth, m_paddedMinKey)) {
                ROS_ERROR("Could not create padded min ColorOcTree key at %f %f %f", minPt.x(), minPt.y(), minPt.z());
                return;
            }
            if (!m_octree->coordToKeyChecked(maxPt, m_maxTreeDepth, paddedMaxKey)) {
                ROS_ERROR("Could not create padded max ColorOcTree key at %f %f %f", maxPt.x(), maxPt.y(), maxPt.z());
                return;
            }

            ROS_DEBUG("Padded MinKey: %d %d %d / padded MaxKey: %d %d %d", m_paddedMinKey[0], m_paddedMinKey[1], m_paddedMinKey[2], paddedMaxKey[0],
                      paddedMaxKey[1], paddedMaxKey[2]);
            assert(paddedMaxKey[0] >= maxKey[0] && paddedMaxKey[1] >= maxKey[1]);

            m_multires2DScale = 1 << (m_treeDepth - m_maxTreeDepth);
            m_gridmap.info.width = (paddedMaxKey[0] - m_paddedMinKey[0]) / m_multires2DScale + 1;
            m_gridmap.info.height = (paddedMaxKey[1] - m_paddedMinKey[1]) / m_multires2DScale + 1;

            int mapOriginX = minKey[0] - m_paddedMinKey[0];
            int mapOriginY = minKey[1] - m_paddedMinKey[1];
            assert(mapOriginX >= 0 && mapOriginY >= 0);

            // might not exactly be min / max of octree:
            Point3D origin = m_octree->keyToCoord(m_paddedMinKey, m_treeDepth);
            double gridRes = m_octree->getNodeSize(m_maxTreeDepth);
            m_projectCompleteMap = (!m_incrementalUpdate || (std::abs(gridRes - m_gridmap.info.resolution) > 1e-6));
            m_gridmap.info.resolution = gridRes;
            m_gridmap.info.origin.position.x = origin.x() - gridRes * 0.5;
            m_gridmap.info.origin.position.y = origin.y() - gridRes * 0.5;
            if (m_maxTreeDepth != m_treeDepth) {
                m_gridmap.info.origin.position.x -= m_res / 2.0;
                m_gridmap.info.origin.position.y -= m_res / 2.0;
            }

            // workaround for  multires. projection not working properly for inner nodes:
            // force re-building complete map
            if (m_maxTreeDepth < m_treeDepth) {
                m_projectCompleteMap = true;
            }


            if (m_projectCompleteMap) {
                ROS_DEBUG("Rebuilding complete 2D map");
                m_gridmap.data.clear();
                // init to unknown:
                m_gridmap.data.resize(m_gridmap.info.width * m_gridmap.info.height, -1);

            } else {

                if (mapChanged(oldMapInfo, m_gridmap.info)) {
                    ROS_DEBUG("2D grid map size changed to %dx%d", m_gridmap.info.width, m_gridmap.info.height);
                    adjustMapData(m_gridmap, oldMapInfo);
                }
                nav_msgs::OccupancyGrid::_data_type::iterator startIt;
                size_t mapUpdateBBXMinX = std::max(0, (int(m_updateBBXMin[0]) - int(m_paddedMinKey[0])) / int(m_multires2DScale));
                size_t mapUpdateBBXMinY = std::max(0, (int(m_updateBBXMin[1]) - int(m_paddedMinKey[1])) / int(m_multires2DScale));
                size_t mapUpdateBBXMaxX = std::min(int(m_gridmap.info.width - 1),
                                                   (int(m_updateBBXMax[0]) - int(m_paddedMinKey[0])) / int(m_multires2DScale));
                size_t mapUpdateBBXMaxY = std::min(int(m_gridmap.info.height - 1),
                                                   (int(m_updateBBXMax[1]) - int(m_paddedMinKey[1])) / int(m_multires2DScale));

                assert(mapUpdateBBXMaxX > mapUpdateBBXMinX);
                assert(mapUpdateBBXMaxY > mapUpdateBBXMinY);

                size_t numCols = mapUpdateBBXMaxX - mapUpdateBBXMinX + 1;

                // test for max idx:
                uint max_idx = m_gridmap.info.width * mapUpdateBBXMaxY + mapUpdateBBXMaxX;
                if (max_idx >= m_gridmap.data.size()) {
                    ROS_ERROR("BBX index not valid: %d (max index %zu for size %d x %d) update-BBX is: [%zu %zu]-[%zu %zu]", max_idx, m_gridmap.data.size(),
                              m_gridmap.info.width, m_gridmap.info.height, mapUpdateBBXMinX, mapUpdateBBXMinY, mapUpdateBBXMaxX, mapUpdateBBXMaxY);
                }

                // reset proj. 2D map in bounding box:
                for (unsigned int j = mapUpdateBBXMinY; j <= mapUpdateBBXMaxY; ++j) {
                    std::fill_n(m_gridmap.data.begin() + m_gridmap.info.width * j + mapUpdateBBXMinX,
                                numCols, -1);
                }

            }


        }

    }

    void OctomapServer::handlePostNodeTraversal(const ros::Time &rostime) {

        if (m_publish2DMap) {
            m_mapPub.publish(m_gridmap);
        }
    }

    void OctomapServer::handleOccupiedNode(const iterator &it) {

        if (m_publish2DMap && m_projectCompleteMap) {
            update2DMap(it, true);
        }
    }

    void OctomapServer::handleFreeNode(const iterator &it) {

        if (m_publish2DMap && m_projectCompleteMap) {
            update2DMap(it, false);
        }
    }

    void OctomapServer::handleOccupiedNodeInBBX(const iterator &it) {

        if (m_publish2DMap && !m_projectCompleteMap) {
            update2DMap(it, true);
        }
    }

    void OctomapServer::handleFreeNodeInBBX(const iterator &it) {

        if (m_publish2DMap && !m_projectCompleteMap) {
            update2DMap(it, false);
        }
    }

/***********************************************************************************************************
 * etc
 **********************************************************************************************************/
    void OctomapServer::update2DMap(const iterator &it, bool occupied) {

        // update 2D map (occupied always overrides):

        if (it.getDepth() == m_maxTreeDepth) {
            unsigned idx = mapIdx(it.getKey());
            if (occupied) {
                m_gridmap.data[mapIdx(it.getKey())] = 100;
            } else if (m_gridmap.data[idx] == -1) {
                m_gridmap.data[idx] = 0;
            }

        } else {
            int intSize = 1 << (m_maxTreeDepth - it.getDepth());
            OcTreeKey minKey = it.getIndexKey();
            for (int dx = 0; dx < intSize; dx++) {
                int i = (minKey[0] + dx - m_paddedMinKey[0]) / m_multires2DScale;
                for (int dy = 0; dy < intSize; dy++) {
                    unsigned idx = mapIdx(i, (minKey[1] + dy - m_paddedMinKey[1]) / m_multires2DScale);
                    if (occupied) {
                        m_gridmap.data[idx] = 100;
                    } else if (m_gridmap.data[idx] == -1) {
                        m_gridmap.data[idx] = 0;
                    }
                }
            }
        }


    }

    bool OctomapServer::isSpeckleNode(const OcTreeKey &nKey) const {
        OcTreeKey key;
        bool neighborFound = false;
        for (key[2] = nKey[2] - 1; !neighborFound && key[2] <= nKey[2] + 1; ++key[2]) {
            for (key[1] = nKey[1] - 1; !neighborFound && key[1] <= nKey[1] + 1; ++key[1]) {
                for (key[0] = nKey[0] - 1; !neighborFound && key[0] <= nKey[0] + 1; ++key[0]) {
                    if (key != nKey) {
                        OcTreeNode *node = m_octree->search(key);
                        if (node && m_octree->isNodeOccupied(node)) {
                            // we have a neighbor => break!
                            neighborFound = true;
                        }
                    }
                }
            }
        }

        return neighborFound;
    }

    void OctomapServer::reconfigureCallback(OctomapServerConfig &config, uint32_t level) {
        if (m_maxTreeDepth != unsigned(config.max_depth)) {
            m_maxTreeDepth = unsigned(config.max_depth);
        } else {
            m_pointcloudMinZ = config.pointcloud_min_z;
            m_pointcloudMaxZ = config.pointcloud_max_z;
            m_occupancyMinZ = config.occupancy_min_z;
            m_occupancyMaxZ = config.occupancy_max_z;
            m_filterSpeckles = config.filter_speckles;
            m_filterGroundPlane = config.filter_ground;
            m_compressMap = config.compress_map;
            m_incrementalUpdate = config.incremental_2D_projection;

            // Parameters with a namespace require an special treatment at the beginning, as dynamic reconfigure
            // will overwrite them because the server is not able to match parameters' names.
            if (m_initConfig) {
                // If parameters do not have the default value, dynamic reconfigure server should be updated.
                if (!is_equal(m_ground_filter_distance, 0.04)) {
                    config.ground_filter_distance = m_ground_filter_distance;
                }
                if (!is_equal(m_groundFilterAngle, 0.15)) {
                    config.ground_filter_angle = m_groundFilterAngle;
                }
                if (!is_equal(m_groundFilterPlaneDistance, 0.07)) {
                    config.ground_filter_plane_distance = m_groundFilterPlaneDistance;
                }
                if (!is_equal(m_maxRange, -1.0)) {
                    config.sensor_model_max_range = m_maxRange;
                }
                if (!is_equal(m_octree->getProbHit(), 0.7)) {
                    config.sensor_model_hit = m_octree->getProbHit();
                }
                if (!is_equal(m_octree->getProbMiss(), 0.4)) {
                    config.sensor_model_miss = m_octree->getProbMiss();
                }
                if (!is_equal(m_octree->getClampingThresMin(), 0.12)) {
                    config.sensor_model_min = m_octree->getClampingThresMin();
                }
                if (!is_equal(m_octree->getClampingThresMax(), 0.97)) {
                    config.sensor_model_max = m_octree->getClampingThresMax();
                }
                m_initConfig = false;

                boost::recursive_mutex::scoped_lock reconf_lock(m_config_mutex);
                m_reconfigureServer.updateConfig(config);
            } else {
                m_ground_filter_distance = config.ground_filter_distance;
                m_groundFilterAngle = config.ground_filter_angle;
                m_groundFilterPlaneDistance = config.ground_filter_plane_distance;
                m_maxRange = config.sensor_model_max_range;
                m_octree->setClampingThresMin(config.sensor_model_min);
                m_octree->setClampingThresMax(config.sensor_model_max);

                // Checking values that might create unexpected behaviors.
                if (is_equal(config.sensor_model_hit, 1.0)) {
                    config.sensor_model_hit -= 1.0e-6;
                }
                m_octree->setProbHit(config.sensor_model_hit);
                if (is_equal(config.sensor_model_miss, 0.0)) {
                    config.sensor_model_miss += 1.0e-6;
                }
                m_octree->setProbMiss(config.sensor_model_miss);
            }
        }
        publishAll();
    }

    void OctomapServer::adjustMapData(nav_msgs::OccupancyGrid &map, const nav_msgs::MapMetaData &oldMapInfo) const {
        if (map.info.resolution != oldMapInfo.resolution) {
            ROS_ERROR("Resolution of map changed, cannot be adjusted");
            return;
        }

        int i_off = int((oldMapInfo.origin.position.x - map.info.origin.position.x) / map.info.resolution + 0.5);
        int j_off = int((oldMapInfo.origin.position.y - map.info.origin.position.y) / map.info.resolution + 0.5);

        if (i_off < 0 || j_off < 0
            || oldMapInfo.width + i_off > map.info.width
            || oldMapInfo.height + j_off > map.info.height) {
            ROS_ERROR("New 2D map does not contain old map area, this case is not implemented");
            return;
        }

        nav_msgs::OccupancyGrid::_data_type oldMapData = map.data;

        map.data.clear();
        // init to unknown:
        map.data.resize(map.info.width * map.info.height, -1);

        nav_msgs::OccupancyGrid::_data_type::iterator fromStart, fromEnd, toStart;

        for (int j = 0; j < int(oldMapInfo.height); ++j) {
            // copy chunks, row by row:
            fromStart = oldMapData.begin() + j * oldMapInfo.width;
            fromEnd = fromStart + oldMapInfo.width;
            toStart = map.data.begin() + ((j + j_off) * m_gridmap.info.width + i_off);
            copy(fromStart, fromEnd, toStart);

//    for (int i =0; i < int(oldMapInfo.width); ++i){
//      map.data[m_gridmap.info.width*(j+j_off) +i+i_off] = oldMapData[oldMapInfo.width*j +i];
//    }

        }

    }

    void OctomapServer::low_coordToKey(const point3d &coord, OcTreeKey &key) const {
        for (unsigned int i = 0; i < 3; i++) {
            key[i] = ((int) floor(m_low_resolution_factor * coord(i))) + m_tree_max_val;
        }
    }

    void OctomapServer::high_coordToKey(const point3d &coord, OcTreeKey &key) const {
        for (unsigned int i = 0; i < 3; i++) {
            key[i] = ((int) floor(m_high_resolution_factor * coord(i))) + m_tree_max_val;
        }
    }


    std_msgs::ColorRGBA OctomapServer::heightMapColor(double h) {

        std_msgs::ColorRGBA color;
        color.a = 1.0;
        // blend over HSV-values (more colors)

        double s = 1.0;
        double v = 1.0;

        h -= floor(h);
        h *= 6;
        int i;
        double m, n, f;

        i = floor(h);
        f = h - i;
        if (!(i & 1)) {
            f = 1 - f;
        } // if i is even
        m = v * (1 - s);
        n = v * (1 - s * f);

        switch (i) {
            case 6:
            case 0:
                color.r = v;
                color.g = n;
                color.b = m;
                break;
            case 1:
                color.r = n;
                color.g = v;
                color.b = m;
                break;
            case 2:
                color.r = m;
                color.g = v;
                color.b = n;
                break;
            case 3:
                color.r = m;
                color.g = n;
                color.b = v;
                break;
            case 4:
                color.r = n;
                color.g = m;
                color.b = v;
                break;
            case 5:
                color.r = v;
                color.g = m;
                color.b = n;
                break;
            default:
                color.r = 1;
                color.g = 0.5;
                color.b = 0.5;
                break;
        }

        return color;
    }

}




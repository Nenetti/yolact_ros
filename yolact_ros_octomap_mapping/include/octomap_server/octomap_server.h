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

#ifndef OCTOMAP_SERVER_OCTOMAPSERVER_H
#define OCTOMAP_SERVER_OCTOMAPSERVER_H


#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

// #include <moveit_msgs/CollisionObject.h>
// #include <moveit_msgs/CollisionMap.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <dynamic_reconfigure/server.h>
#include <octomap_server/OctomapServerConfig.h>

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>


#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/BoundingBoxQuery.h>
#include <octomap_msgs/conversions.h>

#include <octomap_ros/conversions.h>
#include <octomap/Pointcloud.h>
#include <octomap/ScanGraph.h>

#include <custom_octomap/ColorOcTree.h>
#include <actionlib/client/simple_action_client.h>
#include <yolact_ros/SegmentationAction.h>
#include <semantic_mapping/geometric_edge_map.h>
#include <semantic_mapping/segmentation_client.h>
#include <std_msgs/ColorRGBA.h>

//#define COLOR_OCTOMAP_SERVER // turned off here, turned on identical ColorOctomapServer.h - easier maintenance, only maintain octomap_server and then copy and paste to ColorOctomapServer and change define. There are prettier ways to do this, but this works for now


namespace octomap_server {

    using pcl::PointXYZ;
    using pcl::PointXYZRGB;
    using pcl::PointXYZRGBL;
    using pcl::PointCloud;

    class OctomapServer {

        public:
            typedef custom_octomap::ColorOcTree::leaf_iterator iterator;
            typedef custom_octomap::point3d Point3D;

            typedef octomap_msgs::GetOctomap OctomapSrv;
            typedef octomap_msgs::BoundingBoxQuery BBXSrv;

            typedef pcl::PointIndices PCLIndices;

            OctomapServer(ros::NodeHandle private_nh_ = ros::NodeHandle("~"));

            virtual ~OctomapServer();

            virtual bool octomapBinarySrv(OctomapSrv::Request &req, OctomapSrv::GetOctomap::Response &res);

            virtual bool octomapFullSrv(OctomapSrv::Request &req, OctomapSrv::GetOctomap::Response &res);

            bool clearBBXSrv(BBXSrv::Request &req, BBXSrv::Response &resp);

            bool resetSrv(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);

            virtual void insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg);

            virtual bool openFile(const std::string &filename);

            void low_coordToKey(const Point3D &coord, custom_octomap::OcTreeKey &key) const;

            void high_coordToKey(const Point3D &coord, custom_octomap::OcTreeKey &key) const;

        protected:
            inline static void updateMinKey(const custom_octomap::OcTreeKey &in, custom_octomap::OcTreeKey &min) {
                for (unsigned i = 0; i < 3; ++i) {
                    min[i] = std::min(in[i], min[i]);
                }
            };

            inline static void updateMaxKey(const custom_octomap::OcTreeKey &in, custom_octomap::OcTreeKey &max) {
                for (unsigned i = 0; i < 3; ++i) {
                    max[i] = std::max(in[i], max[i]);
                }
            };

            /// Test if key is within update area of map (2D, ignores height)
            inline bool isInUpdateBBX(const iterator &it) const {
                // 2^(tree_depth-depth) voxels wide:
                unsigned voxelWidth = (1 << (m_maxTreeDepth - it.getDepth()));
                custom_octomap::OcTreeKey key = it.getIndexKey(); // lower corner of voxel
                return (key[0] + voxelWidth >= m_updateBBXMin[0]
                        && key[1] + voxelWidth >= m_updateBBXMin[1]
                        && key[0] <= m_updateBBXMax[0]
                        && key[1] <= m_updateBBXMax[1]);
            }

            void reconfigureCallback(OctomapServer::OctomapServerConfig &config, uint32_t level);

            void publishBinaryOctoMap(const ros::Time &rostime = ros::Time::now()) const;

            void publishFullOctoMap(const ros::Time &rostime = ros::Time::now()) const;

            virtual void publishAll(const ros::Time &rostime = ros::Time::now());


            /**
            * @brief update occupancy map with a scan labeled as ground and nonground.
            * The scans should be in the global map frame.
            *
            * @param sensorOrigin origin of the measurements for raycasting
            * @param ground scan endpoints on the ground plane (only clear space)
            * @param nonground all other endpoints (clear up to occupied endpoint)
            */
            void insertScan(const tf::Point &sensorOriginTf, const PointCloud<PointXYZRGB> &ground, const PointCloud<PointXYZRGB> &nonground_nonseg,
                            const PointCloud<PointXYZRGBL> &nonground_seg, const yolact_ros::Segments &segments);

            /// label the input cloud "pc" into ground and nonground. Should be in the robot's fixed frame (not world!)
            void filterPlane(const PointCloud<PointXYZRGB> &pc, PointCloud<PointXYZRGB> &plane, PointCloud<PointXYZRGB> &nonplane) const;

            void downSampling(const PointCloud<PointXYZRGBL> &cloud, PointCloud<PointXYZRGBL> &down_cloud, double size) const;

            void EuclideanClustering(const PointCloud<PointXYZRGB> &cloud, std::vector<pcl::PointIndices> &cluster_indices);

            void EuclideanClustering(const pcl::PointCloud<pcl::PointXYZ> &cloud, std::vector<pcl::PointIndices> &cluster_indices, double to_lerance);

            static void IndicesToCloud(const PointCloud<PointXYZRGB> &cloud, const std::vector<pcl::PointIndices> &point_indices,
                                       std::vector<PointCloud<PointXYZRGB>> &clouds);

            static void IndicesToCloud(const pcl::PointCloud<pcl::PointXYZ> &cloud, const std::vector<pcl::PointIndices> &point_indices,
                                       std::vector<pcl::PointCloud<pcl::PointXYZ>> &clouds);

//            void clustering(const PointCloud<PointXYZRGB> &cloud, std::vector<pcl::PointIndices> &cluster_indices);

            void clustering(const PointCloud<PointXYZRGB> &cloud, PointCloud<PointXYZRGB> &pcl_cloud);


            static void normalEstimation(const PointCloud<PointXYZRGB> &cloud, pcl::PointCloud<pcl::Normal> &cloud_normals, double search_radius);

            static void normalEstimationOMP(const PointCloud<PointXYZRGB> &cloud, pcl::PointCloud<pcl::Normal> &cloud_normals, double search_radius);


            void integralImageNormalEstimation(const PointCloud<PointXYZRGB> &cloud, pcl::PointCloud<pcl::Normal> &cloud_normals, float change_factor,
                                               float smoothing_size);

            void StatisticalOutlierRemoval(const PointCloud<PointXYZRGB> &cloud, std::vector<PCLIndices> &cloud_filtered);

            void absolutelyNormal(pcl::PointCloud<pcl::Normal> &normals);

            void
            searchPointNeighbours(pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_normals, std::vector<std::vector<int>> &neighbours_indices, double radius);
//
//            void searchClusterNeighbours(std::vector<custom_octomap::cluster> &clusters, std::vector<std::tr1::unordered_set<int>> &clusters_neighbours_indices,
//                                         double threshold);
//
//
//            void searchClusterNeighboursRecurs(custom_octomap::cluster &root_cluster, custom_octomap::cluster &parent_cluster,
//                                               std::vector<custom_octomap::cluster> &clusters,
//                                               std::vector<std::tr1::unordered_set<int>> &clusters_neighbours_indices, double threshold,
//                                               std::tr1::unordered_set<int> &set);
//
//            void clusteringNormal(std::vector<custom_octomap::cluster> &clusters, std::vector<std::tr1::unordered_set<int>> &clusters_neighbours_indices,
//                                  std::vector<std::vector<int>> &neighbours_indices, pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_normals,
//                                  double threshold);

            /**
            * @brief Find speckle nodes (single occupied voxels with no neighbors). Only works on lowest resolution!
            * @param key
            * @return
            */
            bool isSpeckleNode(const custom_octomap::OcTreeKey &key) const;

            /// hook that is called before traversing all nodes
            virtual void handlePreNodeTraversal(const ros::Time &rostime);

            /// hook that is called when traversing all nodes of the updated Octree (does nothing here)
            virtual void handleNode(const iterator &it) {};

            /// hook that is called when traversing all nodes of the updated Octree in the updated area (does nothing here)
            virtual void handleNodeInBBX(const iterator &it) {};

            /// hook that is called when traversing occupied nodes of the updated Octree
            virtual void handleOccupiedNode(const iterator &it);

            /// hook that is called when traversing occupied nodes in the updated area (updates 2D map projection here)
            virtual void handleOccupiedNodeInBBX(const iterator &it);

            /// hook that is called when traversing free nodes of the updated Octree
            virtual void handleFreeNode(const iterator &it);

            /// hook that is called when traversing free nodes in the updated area (updates 2D map projection here)
            virtual void handleFreeNodeInBBX(const iterator &it);

            /// hook that is called after traversing all nodes
            virtual void handlePostNodeTraversal(const ros::Time &rostime);

            /// updates the downprojected 2D map as either occupied or free
            virtual void update2DMap(const iterator &it, bool occupied);

            inline unsigned mapIdx(int i, int j) const {
                return m_gridmap.info.width * j + i;
            }

            inline unsigned mapIdx(const custom_octomap::OcTreeKey &key) const {
                return mapIdx((key[0] - m_paddedMinKey[0]) / m_multires2DScale,
                              (key[1] - m_paddedMinKey[1]) / m_multires2DScale);

            }

            /**
             * Adjust data of map due to a change in its info properties (origin or size,
             * resolution needs to stay fixed). map already contains the new map info,
             * but the data is stored according to oldMapInfo.
             */

            void adjustMapData(nav_msgs::OccupancyGrid &map, const nav_msgs::MapMetaData &oldMapInfo) const;

            inline bool mapChanged(const nav_msgs::MapMetaData &oldMapInfo, const nav_msgs::MapMetaData &newMapInfo) {
                return (oldMapInfo.height != newMapInfo.height
                        || oldMapInfo.width != newMapInfo.width
                        || oldMapInfo.origin.position.x != newMapInfo.origin.position.x
                        || oldMapInfo.origin.position.y != newMapInfo.origin.position.y);
            }

            static std_msgs::ColorRGBA heightMapColor(double h);

            ros::NodeHandle m_nh;
            ros::Publisher m_markerPub, m_binaryMapPub, m_fullMapPub, m_pointCloudPub, m_collisionObjectPub, m_mapPub, m_cmapPub, m_fmapPub, m_fmarkerPub,
                    m_testPub;
            message_filters::Subscriber<sensor_msgs::PointCloud2> *m_pointCloudSub;
            tf::MessageFilter<sensor_msgs::PointCloud2> *m_tfPointCloudSub;
            ros::ServiceServer m_octomapBinaryService, m_octomapFullService, m_clearBBXService, m_resetService;
            tf::TransformListener m_tfListener;
            boost::recursive_mutex m_config_mutex;
            dynamic_reconfigure::Server<OctomapServerConfig> m_reconfigureServer;

            custom_octomap::ColorOcTree *m_octree;
            custom_octomap::KeyRay m_keyRay;  // temp storage for ray casting
            custom_octomap::OcTreeKey m_updateBBXMin;
            custom_octomap::OcTreeKey m_updateBBXMax;

            double m_maxRange;
            std::string m_worldFrameId; // the map frame
            std::string m_baseFrameId; // base of the robot for ground plane filtering
            bool m_useHeightMap;
            std_msgs::ColorRGBA m_color;
            std_msgs::ColorRGBA m_colorFree;
            std::vector<custom_octomap::Color> cluster_colors;
            double m_colorFactor;

            bool m_latchedTopics;
            bool m_publishFreeSpace;

            double m_clusterDistance;
            double m_pointColorThreshold;
            double m_regionColorThreshold;

            double m_res;
            double m_high_resolution;
            double m_low_resolution;
            double m_high_resolution_factor;
            double m_low_resolution_factor;
            unsigned int m_tree_max_val;

            unsigned m_treeDepth;
            unsigned m_maxTreeDepth;

            float m_pointcloudMinX;
            float m_pointcloudMaxX;
            float m_pointcloudMinY;
            float m_pointcloudMaxY;
            float m_pointcloudMinZ;
            float m_pointcloudMaxZ;
            double m_occupancyMinZ;
            double m_occupancyMaxZ;
            double m_minSizeX;
            double m_minSizeY;
            bool m_filterSpeckles;

            bool m_filterGroundPlane;
            double m_groundFilterDistance;
            double m_groundFilterAngle;
            double m_groundFilterPlaneDistance;

            bool m_compressMap;

            bool m_initConfig;
            custom_octomap::KeySet update_occupied_cells;

            // downprojected 2D map:
            bool m_incrementalUpdate;
            nav_msgs::OccupancyGrid m_gridmap;
            bool m_publish2DMap;
            bool m_mapOriginChanged;
            custom_octomap::OcTreeKey m_paddedMinKey;
            unsigned m_multires2DScale;
            bool m_projectCompleteMap;
            bool m_useColoredMap;
            bool m_downSampling;
            double m_downSamplingSize;

            double m_change_factor;
            double m_smoothing_size;

            double m_cluster_normal_threshold;

            double m_search_radius;

            bool m_normal_color;

            ros::WallTime beforeTime;
            double m_frameRate;

            semantic_mapping::SegmentationClient m_semantic_client;
            semantic_mapping::GeometricEdgeMap m_geometric_edge_map;

            PointCloud<PointXYZRGBL> m_all_cloud;

            void insertScan(const tf::Point &sensorOriginTf, const PointCloud<PointXYZRGB> &ground, const PointCloud<PointXYZRGB> &ceiling,
                            const PointCloud<PointXYZRGB> &nonground_nonseg, const PointCloud<PointXYZRGBL> &nonground_seg,
                            const std::vector<semantic_mapping::Segment> &segments);
    };
}

#endif

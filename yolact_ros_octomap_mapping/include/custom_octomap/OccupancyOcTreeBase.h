/*
 * OctoMap - An Efficient Probabilistic 3D Mapping Framework Based on Octrees
 * http://octomap.github.com/
 *
 * Copyright (c) 2009-2013, K.M. Wurm and A. Hornung, University of Freiburg
 * All rights reserved.
 * License: New BSD
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

#ifndef CUSTOM_OCTOMAP_OCCUPANCY_OCTREE_BASE_H
#define CUSTOM_OCTOMAP_OCCUPANCY_OCTREE_BASE_H


#include <list>
#include <cstdlib>
#include <vector>
#include <bitset>
#include <algorithm>

#include <octomap/Pointcloud.h>
#include <octomap/ScanGraph.h>

#include <custom_octomap/MCTables.h>
#include <custom_octomap/Types.h>
#include <custom_octomap/Utils.h>
#include <custom_octomap/OcTreeBaseImpl.h>

namespace custom_octomap {

    class OccupancyOcTreeBase : public OcTreeBaseImpl {

        public:

            /***********************************************************************************************************
             * Constructor
             **********************************************************************************************************/
            explicit OccupancyOcTreeBase(double resolution);

            virtual ~OccupancyOcTreeBase();

            OccupancyOcTreeBase(const OccupancyOcTreeBase &rhs);

            /***********************************************************************************************************
             * Function
             **********************************************************************************************************/

            void insertPointCloud(const octomap::Pointcloud &scan, const point3d &sensor_origin,
                                  double maxrange = -1., bool lazy_eval = false, bool discretize = false);

            void insertPointCloud(const octomap::Pointcloud &scan, const point3d &sensor_origin, const pose6d &frame_origin,
                                  double maxrange = -1., bool lazy_eval = false, bool discretize = false);

            void insertPointCloud(const octomap::ScanNode &scan, double maxrange = -1., bool lazy_eval = false, bool discretize = false);

//            void insertPointCloudRays(const octomap::Pointcloud &scan, const point3d &sensor_origin, double maxrange = -1., bool lazy_eval = false);


            /***********************************************************************************************************
             * Node
             **********************************************************************************************************/
            OcTreeNode *setNodeValue(const OcTreeKey &key, float log_odds_value, bool lazy_eval = false);

            OcTreeNode *setNodeValue(const point3d &value, float log_odds_value, bool lazy_eval = false);

            OcTreeNode *setNodeValue(double x, double y, double z, float log_odds_value, bool lazy_eval = false);

            OcTreeNode *updateNode(const OcTreeKey &key, float log_odds_update, bool lazy_eval = false);

            OcTreeNode *updateNode(const point3d &value, float log_odds_update, bool lazy_eval = false);

            OcTreeNode *updateNode(double x, double y, double z, float log_odds_update, bool lazy_eval = false);

            OcTreeNode *updateNode(const OcTreeKey &key, bool occupied, bool lazy_eval = false);

            OcTreeNode *updateNode(const point3d &value, bool occupied, bool lazy_eval = false);

            OcTreeNode *updateNode(double x, double y, double z, bool occupied, bool lazy_eval = false);

            void toMaxLikelihood() override;

            /***********************************************************************************************************
             * Ray
             **********************************************************************************************************/
            bool insertRay(const point3d &origin, const point3d &end, double maxrange = -1.0, bool lazy_eval = false);

            bool castRay(const point3d &origin, const point3d &direction, point3d &end, bool ignoreUnknownCells = false, double maxRange = -1.0) const;

            bool getRayIntersection(const point3d &origin, const point3d &direction, const point3d &center, point3d &intersection, double delta = 0.0) const;

            /***********************************************************************************************************
             * BBX
             **********************************************************************************************************/
            bool getNormals(const point3d &point, std::vector<point3d> &normals, bool unknownStatus = true) const;

            void useBBXLimit(bool enable);

            bool bbxSet() const;

            void setBBXMin(point3d &min);

            void setBBXMax(point3d &max);

            point3d getBBXMin() const;

            point3d getBBXMax() const;

            point3d getBBXBounds() const;

            point3d getBBXCenter() const;

            bool inBBX(const point3d &p) const;

            bool inBBX(const OcTreeKey &key) const;

            void enableChangeDetection(bool enable);

            bool isChangeDetectionEnabled() const;

            void resetChangeDetection();

            KeyBoolMap::const_iterator changedKeysBegin() const;

            KeyBoolMap::const_iterator changedKeysEnd() const;

            size_t numChangesDetected() const;

            void computeUpdate(const octomap::Pointcloud &scan, const point3d &origin, KeySet &free_cells, KeySet &occupied_cells, double maxrange);


            void computeDiscreteUpdate(const octomap::Pointcloud &scan, const point3d &origin, KeySet &free_cells, KeySet &occupied_cells, double maxrange);

            std::istream &readBinaryData(std::istream &s) override;

            std::istream &readBinaryNode(std::istream &s, OcTreeNode *node);

            std::ostream &writeBinaryNode(std::ostream &s, const OcTreeNode *node) const;

            std::ostream &writeBinaryData(std::ostream &s) const override;

            virtual void updateInnerOccupancy();

            virtual void integrateHit(OcTreeNode *occupancyNode) const;

            virtual void integrateMiss(OcTreeNode *occupancyNode) const;

            virtual void updateNodeValue(OcTreeNode *occupancyNode, const float &update) const;

            virtual void updateNodeLogOdds(OcTreeNode *occupancyNode, const float &update) const;

            virtual void nodeToMaxLikelihood(OcTreeNode *occupancyNode) const;

            virtual void nodeToMaxLikelihood(OcTreeNode &occupancyNode) const;

            OcTreeNode *createRoot();

            bool pruneNode(const OcTreeKey &key);

            OcTreeNode *updateNodeValue(const OcTreeKey &key, const bool occupied);

            OcTreeNode *clustering(KeySet &set);

            OcTreeNode *get_node(const OcTreeKey &key);

        protected:

            OccupancyOcTreeBase(double resolution, unsigned int tree_depth, unsigned int tree_max_val);

            bool integrateMissOnRay(const point3d &origin, const point3d &end, bool lazy_eval = false);

            OcTreeNode *updateNodeRecurs(OcTreeNode *node, bool node_just_created, const OcTreeKey &key,
                                         unsigned int depth, const float &log_odds_update, bool lazy_eval = false);

            OcTreeNode *setNodeValueRecurs(OcTreeNode *node, bool node_just_created, const OcTreeKey &key,
                                           unsigned int depth, const float &log_odds_value, bool lazy_eval = false);

            virtual void updateInnerOccupancyRecurs(OcTreeNode *node, unsigned int depth);

            void toMaxLikelihoodRecurs(OcTreeNode *node, unsigned int depth, unsigned int max_depth);

            bool use_bbx_limit;
            point3d bbx_min;
            point3d bbx_max;
            OcTreeKey bbx_min_key;
            OcTreeKey bbx_max_key;

            bool use_change_detection;

            KeyBoolMap changed_keys;


    };

} // namespace

#endif

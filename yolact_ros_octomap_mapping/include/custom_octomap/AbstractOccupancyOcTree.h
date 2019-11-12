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

#ifndef CUSTOM_OCTOMAP_ABSTRACT_OCCUPANCY_OCTREE_H
#define CUSTOM_OCTOMAP_ABSTRACT_OCCUPANCY_OCTREE_H

#include <cassert>
#include <fstream>

#include <custom_octomap/Utils.h>
#include <custom_octomap/OcTreeKey.h>
#include <custom_octomap/AbstractOcTree.h>
#include <custom_octomap/OcTreeNode.h>

namespace custom_octomap {

    /**
     * Interface class for all octree types that store occupancy. This serves
     * as a common base class e.g. for polymorphism and contains common code
     * for reading and writing binary trees.
     */
    class AbstractOccupancyOcTree : public AbstractOcTree {
        public:

            /***********************************************************************************************************
             * Constructor
             **********************************************************************************************************/
            AbstractOccupancyOcTree();

            ~AbstractOccupancyOcTree();

            /***********************************************************************************************************
             * File IO
             **********************************************************************************************************/
            virtual bool writeBinary(const std::string &filename);

            bool writeBinary(std::ostream &s);

            bool writeBinaryConst(const std::string &filename) const;

            bool writeBinaryConst(std::ostream &s) const;

            virtual std::ostream &writeBinaryData(std::ostream &s) const = 0;

            bool readBinary(std::istream &s);

            bool readBinary(const std::string &filename);

            virtual std::istream &readBinaryData(std::istream &s) = 0;

            /***********************************************************************************************************
             * Occupied
             **********************************************************************************************************/
            void setOccupancyThres(double prob);

            bool isNodeOccupied(const OcTreeNode *occupancyNode) const;

            bool isNodeOccupied(const OcTreeNode &occupancyNode) const;

            /***********************************************************************************************************
             * Probability
             **********************************************************************************************************/
            void setProbHit(double prob);

            /// sets the probability for a "miss" (will be converted to logodds) - sensor model
            void setProbMiss(double prob);

            double getProbHit() const;

            float getProbHitLog() const;

            double getProbMiss() const;

            float getProbMissLog() const;

            /***********************************************************************************************************
             * Threshold
             **********************************************************************************************************/
            bool isNodeAtThreshold(const OcTreeNode *occupancyNode) const;

            bool isNodeAtThreshold(const OcTreeNode &occupancyNode) const;

            double getClampingThresMin() const;

            float getClampingThresMinLog() const;

            double getClampingThresMax() const;

            float getClampingThresMaxLog() const;

            void setClampingThresMin(double thresProb);

            void setClampingThresMax(double thresProb);

            double getOccupancyThres() const;

            float getOccupancyThresLog() const;

            /***********************************************************************************************************
             * UpdateNode
             **********************************************************************************************************/
//            virtual OcTreeNode *updateNode(const custom_octomap::OcTreeKey &key, float log_odds_update, bool lazy_eval = false) = 0;
//
//            virtual OcTreeNode *updateNode(const octomap::point3d &value, float log_odds_update, bool lazy_eval = false) = 0;
//
//            virtual OcTreeNode *updateNode(const custom_octomap::OcTreeKey &key, bool occupied, bool lazy_eval = false) = 0;
//
//            virtual OcTreeNode *updateNode(const octomap::point3d &value, bool occupied, bool lazy_eval = false) = 0;

            virtual void toMaxLikelihood() = 0;

        protected:
            bool readBinaryLegacyHeader(std::istream &s, unsigned int &size, double &res);

            float clamping_thres_min = 0;
            float clamping_thres_max = 0;
            float prob_hit_log = 0;
            float prob_miss_log = 0;
            float occ_prob_thres_log = 0;

            static const std::string binaryFileHeader;
    };

}; // end namespace


#endif

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

#ifndef CUSTOM_OCTOMAP_OCTREE_NODE_H
#define CUSTOM_OCTOMAP_OCTREE_NODE_H

#include <bitset>
#include <cassert>
#include <cmath>
#include <fstream>
#include <cstdlib>
#include <cinttypes>
#include <limits>

#include <custom_octomap/Types.h>
#include <custom_octomap/Utils.h>
#include <custom_octomap/Color.h>
#include <tr1/unordered_map>

namespace custom_octomap {

    /**
     * Nodes to be used in OcTree. They represent 3d occupancy grid cells.
     * "value" stores their log-odds occupancy.
     *
     * Note: If you derive a class (directly or indirectly) from OcTreeNode or
     * OcTreeDataNode, you have to implement (at least) the following functions:
     * createChild(), getChild(), getChild() const, expandNode() to avoid slicing
     * errors and memory-related bugs.
     * See ColorOcTreeNode in ColorOcTree.h for an example.
     *
     */
    class OcTreeNode {

        public:

            OcTreeNode();

            ~OcTreeNode();

            OcTreeNode(const OcTreeNode &rhs);

            explicit OcTreeNode(float initVal);

            void copyData(const OcTreeNode &from);

            bool operator==(const OcTreeNode &rhs) const;

            /***********************************************************************************************************
             * children
             **********************************************************************************************************/

            bool isChildDefined(unsigned int i) const;

            void setChild(OcTreeNode &child, unsigned int i);

            OcTreeNode *getChild(unsigned int i) const;

            OcTreeNode *createChild(unsigned int i);

            void deleteChild(unsigned int i);

            bool isChildrenDefined() const;

            void deleteChildren();

            bool childExists(unsigned int i) const;

            bool hasChildren() const;

            int countChildrenDefined() const;

            void allocChildren();

            /***********************************************************************************************************
             * Detail
             **********************************************************************************************************/

            bool isDetailDefined(unsigned int i) const;

            void setDetail(OcTreeNode &detail, unsigned int i);

            OcTreeNode *getDetail(unsigned int i) const;

            OcTreeNode *createDetail(unsigned int i);

            void deleteDetail(unsigned int i);

            bool isDetailsDefined() const;

            void deleteDetails();

            bool detailExists(unsigned int i) const;

            bool hasDetails() const;

            void allocDetails();

            /***********************************************************************************************************
             * Value
             **********************************************************************************************************/
            float getValue() const;

            void setValue(float v);

            void addValue(const float &p);

            /***********************************************************************************************************
             * File IO
             **********************************************************************************************************/
            virtual std::istream &readData(std::istream &s);

            virtual std::ostream &writeData(std::ostream &s) const;

            /***********************************************************************************************************
             * Node occupancy
             **********************************************************************************************************/

            double getOccupancy() const;

            float getLogOdds() const;

            void setLogOdds(float l);

            double getMeanChildLogOdds() const;

            float getMaxChildLogOdds() const;

            void updateOccupancyChildren();

            bool prune();

            bool isNodeCollapsible() const;

            /***********************************************************************************************************
             * Color
             **********************************************************************************************************/
            Color getColor() const;

            void setColor(Color c);

            void setColor(uint8_t r, uint8_t g, uint8_t b);

            bool isColorSet() const;

            void updateColorChildren();

            Color getAverageChildColor() const;

            void update_label_probability(const std::string &name, int id, double probability);

            int get_id(const std::string &name);

            bool is_pruned = false;

            struct Label {

                public:

                    Label() = default;

                    explicit Label(int _id, double _probability, int _number_of_observation) :
                            id(_id), probability(_probability), number_of_observation(_number_of_observation) {}

                    int id{};
                    double probability{};
                    int number_of_observation{};

            };

        protected:

            OcTreeNode **children = nullptr;

            OcTreeNode **details = nullptr;

            float value;

            std::tr1::unordered_map<std::string, Label> label_map;
            Color color;

    };

    std::ostream &operator<<(std::ostream &out, Color const &c);
} // end namespace

#endif

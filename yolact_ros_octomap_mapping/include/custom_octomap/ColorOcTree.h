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

#ifndef CUSTOM_CUSTOM_OCTOMAP_COLOR_OCTREE_H
#define CUSTOM_CUSTOM_OCTOMAP_COLOR_OCTREE_H


#include <iostream>

#include <custom_octomap/Types.h>
#include <custom_octomap/OcTreeNode.h>
#include <custom_octomap/OccupancyOcTreeBase.h>
#include <custom_octomap/OcTreeNode.h>

namespace custom_octomap {

    class ColorOcTree : public OccupancyOcTreeBase {

        public:

            /// Default constructor, sets resolution of leafs
            explicit ColorOcTree(double resolution);

            /// virtual constructor: creates a new object of same type
            /// (Covariant return type requires an up-to-date compiler)
            ColorOcTree *create() const override;

            std::string getTreeType() const override;

            /**
            * Prunes a node when it is collapsible. This overloaded
            * version only considers the node occupancy for pruning,
            * different colors of child nodes are ignored.
            * @return true if pruning was successful
            */
//            bool pruneNode(OcTreeNode *node) override;

            bool isNodeCollapsible(const OcTreeNode *node) const override;

            // set node color at given key or coordinate. Replaces previous color.
            OcTreeNode *setNodeColor(const OcTreeKey &key, uint8_t r, uint8_t g, uint8_t b);

            OcTreeNode *setNodeColor(float x, float y, float z, uint8_t r, uint8_t g, uint8_t b);

            // integrate color measurement at given key or coordinate. Average with previous color
            OcTreeNode *averageNodeColor(const OcTreeKey &key, uint8_t r, uint8_t g, uint8_t b);

            void averageNodeColor(OcTreeNode &node, uint8_t r, uint8_t g, uint8_t b);

            void *averageNodeColor(OcTreeNode &node, const Color &color);

//            int getLabel(const OcTreeKey &key);


            OcTreeNode *averageNodeColor(float x, float y, float z, uint8_t r, uint8_t g, uint8_t b);

            // integrate color measurement at given key or coordinate. Average with previous color
            OcTreeNode *integrateNodeColor(const OcTreeKey &key, uint8_t r,
                                           uint8_t g, uint8_t b);

            OcTreeNode *integrateNodeColor(float x, float y, float z, uint8_t r, uint8_t g, uint8_t b);

            // update inner nodes, sets color to average child color
            void updateInnerOccupancy() override;

            // uses gnuplot to plot a RGB histogram in EPS format
//            void writeColorHistogram(std::string filename);

        protected:


            void updateInnerOccupancyRecurs(OcTreeNode *node, unsigned int depth) override;

            /**
             * Static member object which ensures that this OcTree's prototype
             * ends up in the classIDMapping only once. You need this as a
             * static member in any derived octree class in order to read .ot
             * files through the AbstractOcTree factory. You should also call
             * ensureLinking() once from the constructor.
             */
            class StaticMemberInitializer {
                public:
                    StaticMemberInitializer();

                    /**
                    * Dummy function to ensure that MSVC does not drop the
                    * StaticMemberInitializer, causing this tree failing to register.
                    * Needs to be called from the constructor of this octree.
                    */
                    void ensureLinking();
            };

            /// static member to ensure static initialization (only once)
            static StaticMemberInitializer colorOcTreeMemberInit;

    };

    //! user friendly output in format (r g b)
//    std::ostream &operator<<(std::ostream &out, Color const &c);

} // end namespace

#endif

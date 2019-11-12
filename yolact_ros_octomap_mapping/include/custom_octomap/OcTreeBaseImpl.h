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

#ifndef CUSTOM_OCTOMAP_OCTREE_BASE_IMPL_H
#define CUSTOM_OCTOMAP_OCTREE_BASE_IMPL_H


#include <list>
#include <limits>
#include <iterator>
#include <stack>
#include <bitset>

#include <custom_octomap/Types.h>
#include <custom_octomap/OcTreeKey.h>
#include <custom_octomap/AbstractOccupancyOcTree.h>
#include <custom_octomap/OcTreeNode.h>

namespace custom_octomap {

    class OcTreeBaseImpl : public AbstractOccupancyOcTree {

        public:

            /***********************************************************************************************************
             * Inner Class
             **********************************************************************************************************/
            class iterator_base : public std::iterator<std::forward_iterator_tag, OcTreeNode> {

                public:

                    struct StackElement;

                    iterator_base();

                    explicit iterator_base(OcTreeBaseImpl const *ptree, uint8_t depth = 0);

                    iterator_base(const iterator_base &other);

                    bool operator==(const iterator_base &other) const;

                    bool operator!=(const iterator_base &other) const;

                    iterator_base &operator=(const iterator_base &other);

//                    OcTreeNode const *operator->() const;

                    OcTreeNode *operator->();

                    OcTreeNode &operator*();

                    point3d getCoordinate() const;

                    double getX() const;

                    double getY() const;

                    double getZ() const;

                    double getSize() const;

                    unsigned getDepth() const;

                    const OcTreeKey &getKey() const;

                    OcTreeKey getIndexKey() const;

                    struct StackElement {
                        OcTreeNode *node;
                        OcTreeKey key;
                        uint8_t depth;
                    };


                protected:
                    OcTreeBaseImpl const *tree;
                    uint8_t maxDepth;

                    std::stack<StackElement, std::vector<StackElement>> stack;

                    virtual void singleIncrement();
            };

            class tree_iterator : public iterator_base {

                public:

                    tree_iterator();

                    explicit tree_iterator(OcTreeBaseImpl const *ptree, uint8_t depth = 0);

                    tree_iterator operator++(int);

                    tree_iterator &operator++();

                    bool isLeaf() const;
            };

            class leaf_iterator : public iterator_base {

                public:

                    leaf_iterator();

                    explicit leaf_iterator(OcTreeBaseImpl const *ptree, uint8_t depth = 0);

                    leaf_iterator(const leaf_iterator &other);

                    leaf_iterator operator++(int);

                    leaf_iterator &operator++();

            };

            class leaf_bbx_iterator : public iterator_base {

                public:

                    leaf_bbx_iterator();

                    leaf_bbx_iterator(OcTreeBaseImpl const *ptree, const point3d &min, const point3d &max, uint8_t depth = 0);

                    leaf_bbx_iterator(OcTreeBaseImpl const *ptree, const OcTreeKey &min, const OcTreeKey &max, uint8_t depth = 0);

                    leaf_bbx_iterator(const leaf_bbx_iterator &other);

                    leaf_bbx_iterator operator++(int);

                    leaf_bbx_iterator &operator++();

                protected:

                    void singleIncrement() override;

                    OcTreeKey minKey;
                    OcTreeKey maxKey;
            };

        public:
            /***********************************************************************************************************
             * Constructor
             **********************************************************************************************************/
            explicit OcTreeBaseImpl(double resolution);

            ~OcTreeBaseImpl();

            OcTreeBaseImpl(const OcTreeBaseImpl &rhs);

            void swapContent(OcTreeBaseImpl &rhs);

            bool operator==(const OcTreeBaseImpl &rhs) const;

            std::string getTreeType() const override;

            /***********************************************************************************************************
             * Resolution
             **********************************************************************************************************/
            void setResolution(double r) override;

            double getResolution() const override;

            /***********************************************************************************************************
             * Depth
             **********************************************************************************************************/
            unsigned int getTreeDepth() const;

            size_t size() const override;

            OcTreeKey adjustKeyAtDepth(const OcTreeKey &key, unsigned int depth) const;

            key_type adjustKeyAtDepth(key_type key, unsigned int depth) const;

            /***********************************************************************************************************
             * Node
             **********************************************************************************************************/
            OcTreeNode *createNodeChild(OcTreeNode *node, unsigned int childIdx);

            void deleteNodeChild(OcTreeNode *node, unsigned int childIdx);

            OcTreeNode *getNodeChild(OcTreeNode *node, unsigned int childIdx) const;

            const OcTreeNode *getNodeChild(const OcTreeNode *node, unsigned int childIdx) const;

            virtual bool isNodeCollapsible(const OcTreeNode *node) const;

            bool nodeChildExists(const OcTreeNode *node, unsigned int childIdx) const;

            bool nodeHasChildren(const OcTreeNode *node) const;

            virtual void expandNode(OcTreeNode *node);

            virtual bool pruneNode(OcTreeNode *node);

            OcTreeNode *getRoot() const { return root; }

            OcTreeNode *search(double x, double y, double z, unsigned int depth = 0) const;

            OcTreeNode *search(const point3d &value, unsigned int depth = 0) const;

            OcTreeNode *search(const OcTreeKey &key, unsigned int depth = 0) const;

            bool deleteNode(double x, double y, double z, unsigned int depth = 0);

            bool deleteNode(const point3d &value, unsigned int depth = 0);

            bool deleteNode(const OcTreeKey &key, unsigned int depth = 0);

            size_t calcNumNodes() const;

            size_t getNumLeafNodes() const;

            double getNodeSize(unsigned depth) const;

            /***********************************************************************************************************
             * Function
             **********************************************************************************************************/
            void clear() override;

            void prune() override;

            void expand() override;

            void getUnknownLeafCenters(point3d_list &node_centers, point3d pmin, point3d pmax, unsigned int depth = 0) const;

            /***********************************************************************************************************
             * Memory
             **********************************************************************************************************/
            size_t memoryUsage() const override;

            size_t memoryUsageNode() const override;

            unsigned long long memoryFullGrid() const;

            double volume();

            /***********************************************************************************************************
             * Metric
             **********************************************************************************************************/
            void getMetricSize(double &x, double &y, double &z) override;

            virtual void getMetricSize(double &x, double &y, double &z) const;

            void getMetricMin(double &x, double &y, double &z) override;

            void getMetricMin(double &x, double &y, double &z) const override;

            void getMetricMax(double &x, double &y, double &z) override;

            void getMetricMax(double &x, double &y, double &z) const override;

            /***********************************************************************************************************
             * Ray
             **********************************************************************************************************/
            void clearKeyRays();

            bool computeRayKeys(const OcTreeKey &origin, const OcTreeKey &end, KeyRay &ray) const;

            bool computeRayKeys(const point3d &origin, const point3d &end, KeyRay &ray) const;

            bool computeRay(const point3d &origin, const point3d &end, std::vector<point3d> &ray);

            /***********************************************************************************************************
             * File IO
             **********************************************************************************************************/
            std::istream &readData(std::istream &s) override;

            std::ostream &writeData(std::ostream &s) const override;

            /***********************************************************************************************************
             * Iterator
             **********************************************************************************************************/

            leaf_iterator begin(unsigned char maxDepth = 0) const;

            const leaf_iterator end() const;

            leaf_iterator begin_leafs(unsigned char maxDepth = 0) const;

            const leaf_iterator end_leafs() const;

            leaf_bbx_iterator begin_leafs_bbx(const OcTreeKey &min, const OcTreeKey &max, unsigned char maxDepth = 0) const;

            leaf_bbx_iterator begin_leafs_bbx(const point3d &min, const point3d &max, unsigned char maxDepth = 0) const;

            const leaf_bbx_iterator end_leafs_bbx() const;

            tree_iterator begin_tree(unsigned char maxDepth = 0) const;

            const tree_iterator end_tree() const;

            /***********************************************************************************************************
             * Convert Point3D OcTreeKey
             **********************************************************************************************************/
            key_type coordToKey(double coordinate) const;

            key_type coordToKey(double coordinate, unsigned depth) const;

            void coordToKey(const point3d &coord, OcTreeKey &key) const;

            OcTreeKey coordToKey(const point3d &coord) const;

            OcTreeKey coordToKey(double x, double y, double z) const;

            OcTreeKey coordToKey(const point3d &coord, unsigned depth) const;

            OcTreeKey coordToKey(double x, double y, double z, unsigned depth) const;

            bool isCoordinateAvailable(const point3d &point) const;

            bool coordToKeyChecked(const point3d &coord, OcTreeKey &key) const;

            bool coordToKeyChecked(const point3d &coord, unsigned depth, OcTreeKey &key) const;

            bool coordToKeyChecked(double x, double y, double z, OcTreeKey &key) const;

            bool coordToKeyChecked(double x, double y, double z, unsigned depth, OcTreeKey &key) const;

            bool coordToKeyChecked(double coordinate, key_type &key) const;

            bool coordToKeyChecked(double coordinate, unsigned depth, key_type &key) const;

            double keyToCoord(key_type key, unsigned depth) const;

            double keyToCoord(key_type key) const;

            point3d keyToCoord(const OcTreeKey &key) const;

            point3d keyToCoord(const OcTreeKey &key, unsigned depth) const;

        protected:

            OcTreeBaseImpl(double resolution, unsigned int tree_depth, unsigned int tree_max_val);

            void init();

            void calcMinMax();

            void calcNumNodesRecurs(OcTreeNode *node, size_t &num_nodes) const;

            std::istream &readNodesRecurs(OcTreeNode *, std::istream &s);

            std::ostream &writeNodesRecurs(const OcTreeNode *, std::ostream &s) const;

            void deleteNodeRecurs(OcTreeNode *node);

            bool deleteNodeRecurs(OcTreeNode *node, unsigned int depth, unsigned int max_depth, const OcTreeKey &key);

            void pruneRecurs(OcTreeNode *node, unsigned int depth, unsigned int max_depth, unsigned int &num_pruned);

            void expandRecurs(OcTreeNode *node, unsigned int depth, unsigned int max_depth);

            size_t getNumLeafNodesRecurs(const OcTreeNode *parent) const;


        private:

            OcTreeBaseImpl &operator=(const OcTreeBaseImpl &);

        protected:

            void allocNodeChildren(OcTreeNode *node);

            OcTreeNode *root;

            const int tree_depth;
            const int tree_max_val;
            double resolution;
            double resolution_factor;

            size_t tree_size; ///< number of nodes in tree
            /// flag to denote whether the octree extent changed (for lazy min/max eval)
            bool size_changed;

            point3d tree_center;  // coordinate offset of tree

            double max_value[3]; ///< max in x, y, z
            double min_value[3]; ///< min in x, y, z
            /// contains the size of a voxel at level i (0: root node). tree_depth+1 levels (incl. 0)
            std::vector<double> sizeLookupTable;

            /// data structure for ray casting, array for multithreading
            std::vector<KeyRay> keyrays;

            const leaf_iterator leaf_iterator_end;
            const leaf_bbx_iterator leaf_iterator_bbx_end;
            const tree_iterator tree_iterator_end;


    };

}

#endif

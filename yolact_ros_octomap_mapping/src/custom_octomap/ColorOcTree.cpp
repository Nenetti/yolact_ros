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

#include <custom_octomap/ColorOcTree.h>

namespace custom_octomap {

    // tree implementation  --------------------------------------
    ColorOcTree::ColorOcTree(double in_resolution) : OccupancyOcTreeBase(in_resolution) {
        colorOcTreeMemberInit.ensureLinking();
    }

    /// Default constructor, sets resolution of leafs

    /// virtual constructor: creates a new object of same type
    /// (Covariant return type requires an up-to-date compiler)
    ColorOcTree *ColorOcTree::create() const {
        return new ColorOcTree(resolution);
    }

    std::string ColorOcTree::getTreeType() const {
        return "ColorOcTree";
    }

    /**
    * Prunes a node when it is collapsible. This overloaded
    * version only considers the node occupancy for pruning,
    * different colors of child nodes are ignored.
    * @return true if pruning was successful
    */


    OcTreeNode *ColorOcTree::setNodeColor(float x, float y, float z, uint8_t r, uint8_t g, uint8_t b) {
        OcTreeKey key;
        if (!this->coordToKeyChecked(octomap::point3d(x, y, z), key)) {
            return NULL;
        }
        return setNodeColor(key, r, g, b);
    }

    OcTreeNode *ColorOcTree::averageNodeColor(float x, float y, float z, uint8_t r, uint8_t g, uint8_t b) {
        OcTreeKey key;
        if (!this->coordToKeyChecked(octomap::point3d(x, y, z), key)) {
            return NULL;
        }
        return averageNodeColor(key, r, g, b);
    }

    OcTreeNode *ColorOcTree::integrateNodeColor(float x, float y, float z, uint8_t r, uint8_t g, uint8_t b) {
        OcTreeKey key;
        if (!this->coordToKeyChecked(octomap::point3d(x, y, z), key)) {
            return NULL;
        }
        return integrateNodeColor(key, r, g, b);
    }

    OcTreeNode *ColorOcTree::setNodeColor(const OcTreeKey &key, uint8_t r, uint8_t g, uint8_t b) {
        OcTreeNode *n = search(key);
        if (n != 0) {
            n->setColor(r, g, b);
        }
        return n;
    }

//    bool ColorOcTree::pruneNode(OcTreeNode *node) {
//        if (!isNodeCollapsible(node)) {
//            return false;
//        }
//
//        // set value to children's values (all assumed equal)
//        node->copyData(*(getNodeChild(node, 0)));
//
//        if (node->isColorSet()) { // TODO check
//            node->setColor(node->getAverageChildColor());
//        }
//
//        // delete children
//        for (unsigned int i = 0; i < 8; i++) {
//            deleteNodeChild(node, i);
//        }
////        delete[] node->children;
////        node->children = NULL;
//        node->deleteChildren();
//
//        return true;
//    }

    bool ColorOcTree::isNodeCollapsible(const OcTreeNode *node) const {
        // all children must exist, must not have children of
        // their own and have the same occupancy probability
        if (!nodeChildExists(node, 0)) {
            return false;
        }

        const OcTreeNode *firstChild = getNodeChild(node, 0);
        if (nodeHasChildren(firstChild)) {
            return false;
        }

        for (unsigned int i = 1; i < 8; i++) {
            // compare nodes only using their occupancy, ignoring color for pruning
            if (!nodeChildExists(node, i) || nodeHasChildren(getNodeChild(node, i)) || !(getNodeChild(node, i)->getValue() == firstChild->getValue())) {
                return false;
            }
        }

        return true;
    }

    OcTreeNode *ColorOcTree::averageNodeColor(const OcTreeKey &key, uint8_t r, uint8_t g, uint8_t b) {
        OcTreeNode *n = search(key);
        if (n != 0) {
            if (n->isColorSet()) {
                Color prev_color = n->getColor();
                n->setColor((prev_color.r + r) / 2, (prev_color.g + g) / 2, (prev_color.b + b) / 2);
            } else {
                n->setColor(r, g, b);
            }
        }
        return n;
    }

    void ColorOcTree::averageNodeColor(OcTreeNode &node, uint8_t r, uint8_t g, uint8_t b) {
        if (node.isColorSet()) {
            Color prev_color = node.getColor();
            node.setColor((prev_color.r + r) / 2, (prev_color.g + g) / 2, (prev_color.b + b) / 2);
        } else {
            node.setColor(r, g, b);
        }
    }

    void *ColorOcTree::averageNodeColor(OcTreeNode &node, const Color &color) {
        if (node.isColorSet()) {
            Color prev_color = node.getColor();
            node.setColor((prev_color.r + color.r) / 2, (prev_color.g + color.g) / 2, (prev_color.b + color.b) / 2);
        } else {
            node.setColor(color);
        }
    }

//    int ColorOcTree::getLabel(const OcTreeKey &key) {
//        OcTreeNode *n = search(key);
//        if (n != nullptr) {
//            return n->getLabel();
//        }
//        return -1;
//    }

    OcTreeNode *ColorOcTree::integrateNodeColor(const OcTreeKey &key, uint8_t r, uint8_t g, uint8_t b) {
        OcTreeNode *n = search(key);
        if (n != 0) {
            if (n->isColorSet()) {
                Color prev_color = n->getColor();
                double node_prob = n->getOccupancy();
                uint8_t new_r = (uint8_t) ((double) prev_color.r * node_prob + (double) r * (0.99 - node_prob));
                uint8_t new_g = (uint8_t) ((double) prev_color.g * node_prob + (double) g * (0.99 - node_prob));
                uint8_t new_b = (uint8_t) ((double) prev_color.b * node_prob + (double) b * (0.99 - node_prob));
                n->setColor(new_r, new_g, new_b);
            } else {
                n->setColor(r, g, b);
            }
        }
        return n;
    }


    void ColorOcTree::updateInnerOccupancy() {
        this->updateInnerOccupancyRecurs(this->root, 0);
    }

    void ColorOcTree::updateInnerOccupancyRecurs(OcTreeNode *node, unsigned int depth) {
        // only recurse and update for inner nodes:
        if (nodeHasChildren(node)) {
            // return early for last level:
            if (depth < this->tree_depth) {
                for (unsigned int i = 0; i < 8; i++) {
                    if (nodeChildExists(node, i)) {
                        updateInnerOccupancyRecurs(getNodeChild(node, i), depth + 1);
                    }
                }
            }
            node->updateOccupancyChildren();
            node->updateColorChildren();
        }
    }

//    std::ostream &operator<<(std::ostream &out, Color const &c) {
//        return out << '(' << (unsigned int) c.r << ' ' << (unsigned int) c.g << ' ' << (unsigned int) c.b << ')';
//    }

    ColorOcTree::StaticMemberInitializer ColorOcTree::colorOcTreeMemberInit;

    ColorOcTree::StaticMemberInitializer::StaticMemberInitializer() {
        ColorOcTree *tree = new ColorOcTree(0.1);
        tree->clearKeyRays();
        AbstractOcTree::registerTreeType(tree);
    }

    void ColorOcTree::StaticMemberInitializer::ensureLinking() {}


} // end namespace

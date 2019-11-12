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

#ifndef OCTOMAP_OCTREEITERATOR_HXX_
#define OCTOMAP_OCTREEITERATOR_HXX_

#include <custom_octomap/OcTreeBaseImpl.h>
/**
 * Base class for OcTree iterators. So far, all iterator's are
 * const with respect to the tree. This file is included within
 * OcTreeBaseImpl.h, you should probably not include this directly.
 */
namespace custom_octomap {

    /***********************************************************************************************************
     * Itarator_Base
     **********************************************************************************************************/
    OcTreeBaseImpl::iterator_base::iterator_base() : tree(NULL), maxDepth(0) {}

    OcTreeBaseImpl::iterator_base::iterator_base(OcTreeBaseImpl const *ptree, uint8_t depth) :
            tree((ptree && ptree->root) ? ptree : NULL), maxDepth(depth) {
        if (ptree && maxDepth == 0) {
            maxDepth = ptree->getTreeDepth();
        }

        if (tree && tree->root) { // tree is not empty
            StackElement s;
            s.node = tree->root;
            s.depth = 0;
            s.key[0] = s.key[1] = s.key[2] = tree->tree_max_val;
            stack.push(s);
        } else { // construct the same as "end"
            tree = NULL;
            this->maxDepth = 0;
        }
    }

    OcTreeBaseImpl::iterator_base::iterator_base(const iterator_base &other) : tree(other.tree), maxDepth(other.maxDepth), stack(other.stack) {}


    bool OcTreeBaseImpl::iterator_base::operator==(const iterator_base &other) const {
        return (tree == other.tree && stack.size() == other.stack.size()
                && (stack.size() == 0 || (stack.size() > 0 && (stack.top().node == other.stack.top().node
                                                               && stack.top().depth == other.stack.top().depth
                                                               && stack.top().key == other.stack.top().key))));
    }

    bool OcTreeBaseImpl::iterator_base::operator!=(const iterator_base &other) const {
        return (tree != other.tree || stack.size() != other.stack.size()
                || (stack.size() > 0 && ((stack.top().node != other.stack.top().node
                                          || stack.top().depth != other.stack.top().depth
                                          || stack.top().key != other.stack.top().key))));
    }

    OcTreeBaseImpl::iterator_base &OcTreeBaseImpl::iterator_base::operator=(const iterator_base &other) {
        tree = other.tree;
        maxDepth = other.maxDepth;
        stack = other.stack;
        return *this;
    }

    OcTreeNode *OcTreeBaseImpl::iterator_base::operator->() {
        return stack.top().node;
    }


    OcTreeNode &OcTreeBaseImpl::iterator_base::operator*() {
        return *(stack.top().node);
    }


    point3d OcTreeBaseImpl::iterator_base::getCoordinate() const {
        return tree->keyToCoord(stack.top().key, stack.top().depth);
    }


    double OcTreeBaseImpl::iterator_base::getX() const {
        return tree->keyToCoord(stack.top().key[0], stack.top().depth);
    }


    double OcTreeBaseImpl::iterator_base::getY() const {
        return tree->keyToCoord(stack.top().key[1], stack.top().depth);
    }


    double OcTreeBaseImpl::iterator_base::getZ() const {
        return tree->keyToCoord(stack.top().key[2], stack.top().depth);
    }


    double OcTreeBaseImpl::iterator_base::getSize() const {
        return tree->getNodeSize(stack.top().depth);
    }


    unsigned OcTreeBaseImpl::iterator_base::getDepth() const {
        return unsigned(stack.top().depth);
    }


    const OcTreeKey &OcTreeBaseImpl::iterator_base::getKey() const {
        return stack.top().key;
    }

    OcTreeKey OcTreeBaseImpl::iterator_base::getIndexKey() const {
        return OcTreeKey::computeIndexKey(tree->getTreeDepth() - stack.top().depth, stack.top().key);
    }

    void OcTreeBaseImpl::iterator_base::singleIncrement() {
        StackElement top = stack.top();
        stack.pop();
        if (top.depth == maxDepth) {
            return;
        }

        StackElement s;
        s.depth = top.depth + 1;

        key_type center_offset_key = tree->tree_max_val >> s.depth;
        // push on stack in reverse order
        for (int i = 7; i >= 0; --i) {
            if (tree->nodeChildExists(top.node, i)) {
                OcTreeKey::computeChildKey(i, center_offset_key, top.key, s.key);
                s.node = tree->getNodeChild(top.node, i);
                //OCTOMAP_DEBUG_STR("Current depth: " << int(top.depth) << " new: "<< int(s.depth) << " child#" << i <<" ptr: "<<s.node);
                stack.push(s);
                assert(s.depth <= maxDepth);
            }
        }
    }

    /***********************************************************************************************************
     * Tree_Iterator
     **********************************************************************************************************/
    OcTreeBaseImpl::tree_iterator::tree_iterator() : iterator_base::iterator_base() {}

    OcTreeBaseImpl::tree_iterator::tree_iterator(OcTreeBaseImpl const *ptree, uint8_t depth) : iterator_base(ptree, depth) {}

    OcTreeBaseImpl::tree_iterator OcTreeBaseImpl::tree_iterator::operator++(int) {
        tree_iterator result = *this;
        ++(*this);
        return result;
    }

    OcTreeBaseImpl::tree_iterator &OcTreeBaseImpl::tree_iterator::operator++() {

        if (!this->stack.empty()) {
            this->singleIncrement();
        }

        if (this->stack.empty()) {
            this->tree = NULL;
        }

        return *this;
    }

    bool OcTreeBaseImpl::tree_iterator::isLeaf() const {
        return (!this->tree->nodeHasChildren(this->stack.top().node) || this->stack.top().depth == this->maxDepth);
    }

    /***********************************************************************************************************
     * Leaf_Iterator
     **********************************************************************************************************/
    OcTreeBaseImpl::leaf_iterator::leaf_iterator() : iterator_base() {}

    OcTreeBaseImpl::leaf_iterator::leaf_iterator(const OcTreeBaseImpl *ptree, uint8_t depth) : iterator_base(ptree, depth) {
        // tree could be empty (= no stack)
        if (this->stack.size() > 0) {
            // skip forward to next valid leaf node:
            // add root another time (one will be removed) and ++
            this->stack.push(this->stack.top());
            operator++();
        }
    }

    OcTreeBaseImpl::leaf_iterator::leaf_iterator(const leaf_iterator &other) : iterator_base(other) {}

    OcTreeBaseImpl::leaf_iterator OcTreeBaseImpl::leaf_iterator::operator++(int) {
        leaf_iterator result = *this;
        ++(*this);
        return result;
    }

    OcTreeBaseImpl::leaf_iterator &OcTreeBaseImpl::leaf_iterator::operator++() {
        if (this->stack.empty()) {
            this->tree = NULL; // TODO check?

        } else {
            this->stack.pop();

            // skip forward to next leaf
            while (!this->stack.empty()
                   && this->stack.top().depth < this->maxDepth
                   && this->tree->nodeHasChildren(this->stack.top().node)) {
                this->singleIncrement();
            }
            // done: either stack is empty (== end iterator) or a next leaf node is reached!
            if (this->stack.empty()) {
                this->tree = NULL;
            }
        }


        return *this;
    }

    /***********************************************************************************************************
     * Leaf_BBX_Iterator
     **********************************************************************************************************/
    OcTreeBaseImpl::leaf_bbx_iterator::leaf_bbx_iterator() : iterator_base() {}

    OcTreeBaseImpl::leaf_bbx_iterator::leaf_bbx_iterator(OcTreeBaseImpl const *ptree, const point3d &min, const point3d &max, uint8_t depth)
            : iterator_base(ptree, depth) {
        if (this->stack.size() > 0) {
            assert(ptree);
            if (!this->tree->coordToKeyChecked(min, minKey) || !this->tree->coordToKeyChecked(max, maxKey)) {
                // coordinates invalid, set to end iterator
                this->tree = NULL;
                this->maxDepth = 0;
            } else {  // else: keys are generated and stored

                // advance from root to next valid leaf in bbx:
                this->stack.push(this->stack.top());
                this->operator++();
            }
        }

    }

    OcTreeBaseImpl::leaf_bbx_iterator::leaf_bbx_iterator(OcTreeBaseImpl const *ptree, const OcTreeKey &min, const OcTreeKey &max, uint8_t depth)
            : iterator_base(ptree, depth), minKey(min), maxKey(max) {
        // tree could be empty (= no stack)
        if (this->stack.size() > 0) {
            // advance from root to next valid leaf in bbx:
            this->stack.push(this->stack.top());
            this->operator++();
        }
    }

    OcTreeBaseImpl::leaf_bbx_iterator::leaf_bbx_iterator(const leaf_bbx_iterator &other) : iterator_base(other) {
        minKey = other.minKey;
        maxKey = other.maxKey;
    }


    OcTreeBaseImpl::leaf_bbx_iterator OcTreeBaseImpl::leaf_bbx_iterator::operator++(int) {
        leaf_bbx_iterator result = *this;
        ++(*this);
        return result;
    }

    OcTreeBaseImpl::leaf_bbx_iterator &OcTreeBaseImpl::leaf_bbx_iterator::operator++() {
        if (this->stack.empty()) {
            this->tree = NULL; // TODO check?

        } else {
            this->stack.pop();

            // skip forward to next leaf
            while (!this->stack.empty()
                   && this->stack.top().depth < this->maxDepth
                   && this->tree->nodeHasChildren(this->stack.top().node)) {
                this->singleIncrement();
            }
            // done: either stack is empty (== end iterator) or a next leaf node is reached!
            if (this->stack.empty()) {
                this->tree = NULL;
            }
        }


        return *this;
    }

    void OcTreeBaseImpl::leaf_bbx_iterator::singleIncrement() {
        typename iterator_base::StackElement top = this->stack.top();
        this->stack.pop();

        typename iterator_base::StackElement s;
        s.depth = top.depth + 1;
        key_type center_offset_key = this->tree->tree_max_val >> s.depth;
        // push on stack in reverse order
        for (int i = 7; i >= 0; --i) {
            if (this->tree->nodeChildExists(top.node, i)) {
                OcTreeKey::computeChildKey(i, center_offset_key, top.key, s.key);

                // overlap of query bbx and child bbx?
                if ((minKey[0] <= (s.key[0] + center_offset_key)) && (maxKey[0] >= (s.key[0] - center_offset_key))
                    && (minKey[1] <= (s.key[1] + center_offset_key)) && (maxKey[1] >= (s.key[1] - center_offset_key))
                    && (minKey[2] <= (s.key[2] + center_offset_key)) && (maxKey[2] >= (s.key[2] - center_offset_key))) {
                    s.node = this->tree->getNodeChild(top.node, i);
                    this->stack.push(s);
                    assert(s.depth <= this->maxDepth);
                }
            }
        }
    }

}

#endif

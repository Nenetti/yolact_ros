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



#include <custom_octomap/OcTreeNode.h>

namespace custom_octomap {

    /***********************************************************************************************************
     * Constructor
     **********************************************************************************************************/
    OcTreeNode::OcTreeNode() : children(nullptr), value(0) {
        allocDetails();
    }

    OcTreeNode::~OcTreeNode() {
        assert(children == nullptr);
        allocDetails();
    }

    OcTreeNode::OcTreeNode(float initVal) : children(nullptr), value(initVal) {
        allocDetails();
    }

    OcTreeNode::OcTreeNode(const OcTreeNode &rhs) : children(nullptr), value(rhs.value), color(rhs.color) {
        if (rhs.children != nullptr) {
            allocChildren();
            allocDetails();
            for (unsigned i = 0; i < 8; ++i) {
                if (rhs.children[i] != nullptr) {
                    children[i] = new OcTreeNode(*rhs.children[i]);
                }
            }
        }
    }

    /***********************************************************************************************************
     * Copy
     **********************************************************************************************************/
    void OcTreeNode::copyData(const OcTreeNode &from) {
        value = from.value;
        this->color = from.getColor();
    }

    /***********************************************************************************************************
     * Operator
     **********************************************************************************************************/
    bool OcTreeNode::operator==(const OcTreeNode &rhs) const {
        return (rhs.value == value && rhs.color == color);
    }

    /***********************************************************************************************************
     * Children
     **********************************************************************************************************/
    bool OcTreeNode::isChildDefined(unsigned int i) const {
        return children[i] != nullptr;
    }

    void OcTreeNode::setChild(OcTreeNode &child, unsigned int i) {
        children[i] = &child;
    }

    OcTreeNode *OcTreeNode::getChild(unsigned int i) const {
        return children[i];
    }

    OcTreeNode *OcTreeNode::createChild(unsigned int i) {
        if (children == nullptr) {
            allocChildren();
        }
        auto *newNode = new OcTreeNode();
        children[i] = newNode;
        return newNode;
    }

    void OcTreeNode::deleteChild(unsigned int i) {
        delete children[i];
        children[i] = nullptr;
    }


    bool OcTreeNode::isChildrenDefined() const {
        return children != nullptr;
    }

    void OcTreeNode::deleteChildren() {
        delete[] children;
        children = nullptr;
    }


    void OcTreeNode::allocChildren() {
        children = new OcTreeNode *[8];
        for (unsigned int i = 0; i < 8; i++) {
            children[i] = nullptr;
        }
    }

    bool OcTreeNode::childExists(unsigned int i) const {
        assert(i < 8);
        if ((children != nullptr) && (children[i] != nullptr)) {
            return true;
        } else {
            return false;
        }
    }

    int OcTreeNode::countChildrenDefined() const {
        if (children == nullptr) {
            return 0;
        }
        int count = 0;
        for (int i = 0; i < 8; i++) {
            if (children[i] != nullptr) {
                count += 1;
            }
        }
        return count;
    }


    bool OcTreeNode::hasChildren() const {
        if (children == nullptr) {
            return false;
        }
        for (unsigned int i = 0; i < 8; i++) {
            // fast check, we know children != nullptr
            if (children[i] != nullptr) {
                return true;
            }
        }
        return false;
    }

    /***********************************************************************************************************
     * Details
     **********************************************************************************************************/
    bool OcTreeNode::isDetailDefined(unsigned int i) const {
        return details[i] != nullptr;
    }

    void OcTreeNode::setDetail(OcTreeNode &detail, unsigned int i) {
        details[i] = &detail;
    }

    OcTreeNode *OcTreeNode::getDetail(unsigned int i) const {
        return details[i];
    }

    OcTreeNode *OcTreeNode::createDetail(unsigned int i) {
        if (details == nullptr) {
            allocDetails();
        }
        auto *newNode = new OcTreeNode();
        details[i] = newNode;
        return newNode;
    }

    void OcTreeNode::deleteDetail(unsigned int i) {
        delete details[i];
        details[i] = nullptr;
    }


    bool OcTreeNode::isDetailsDefined() const {
        return details != nullptr;
    }

    void OcTreeNode::deleteDetails() {
        delete[] details;
        details = nullptr;
    }


    void OcTreeNode::allocDetails() {
        details = new OcTreeNode *[8];
        for (unsigned int i = 0; i < 8; i++) {
            details[i] = nullptr;
        }
    }

    bool OcTreeNode::detailExists(unsigned int i) const {
        assert(i < 8);
        if ((details != nullptr) && (details[i] != nullptr)) {
            return true;
        } else {
            return false;
        }
    }

    bool OcTreeNode::hasDetails() const {
        if (details == nullptr) {
            return false;
        }
        for (unsigned int i = 0; i < 8; i++) {
            // fast check, we know details != nullptr
            if (details[i] != nullptr) {
                return true;
            }
        }
        return false;
    }

    /****************************************************************
     * Value
     ***************************************************************/

    float OcTreeNode::getValue() const {
        return value;
    }

    void OcTreeNode::setValue(float v) {
        value = v;
    }

    void OcTreeNode::addValue(const float &logOdds) {
        value += logOdds;
    }

    /****************************************************************
     * File IO
     ***************************************************************/
    std::istream &OcTreeNode::readData(std::istream &s) {
        s.read((char *) &value, sizeof(value)); // occupancy
        s.read((char *) &color, sizeof(Color)); // color

        return s;
    }

    std::ostream &OcTreeNode::writeData(std::ostream &s) const {
        s.write((const char *) &value, sizeof(value)); // occupancy
        s.write((const char *) &color, sizeof(Color)); // color

        return s;
    }

    /****************************************************************
     * Node occupancy
     ***************************************************************/

    double OcTreeNode::getOccupancy() const {
        return probability(value);
    }

    float OcTreeNode::getLogOdds() const {
        return value;
    }

    void OcTreeNode::setLogOdds(float l) {
        value = l;
    }

    bool OcTreeNode::prune() {
        if (is_pruned) {
            return false;
        }
        if (!isNodeCollapsible()) {
            return false;
        }
        copyData(*children[0]);

        for (int i = 0; i < 8; i++) {
            deleteChild(i);
        }

        deleteChildren();
        is_pruned = true;

        return true;
    }

    bool OcTreeNode::isNodeCollapsible() const {
        // all children must exist, must not have children of
        // their own and have the same occupancy probability
        if (children == nullptr || children[0] == nullptr) {
            return false;
        }
        const OcTreeNode *firstChild = children[0];
        if (firstChild->hasChildren()) {
            return false;
        }
        for (int i = 1; i < 8; i++) {
            if (children[i] == nullptr || children[i]->hasChildren() || !(*children[i] == *firstChild)) {
                return false;
            }
        }

        return true;
    }

    double OcTreeNode::getMeanChildLogOdds() const {
        double mean = 0;
        uint8_t c = 0;
        if (children != nullptr) {
            for (unsigned int i = 0; i < 8; i++) {
                if (children[i] != nullptr) {
                    mean += children[i]->getOccupancy(); // TODO check if works generally
                    ++c;
                }
            }
        }

        if (c > 0) {
            mean /= (double) c;
        }

        return log(mean / (1 - mean));
    }

    float OcTreeNode::getMaxChildLogOdds() const {
        float max = -std::numeric_limits<float>::max();

        if (children != nullptr) {
            for (unsigned int i = 0; i < 8; i++) {
                if (children[i] != nullptr) {
                    float l = children[i]->getLogOdds(); // TODO check if works generally
                    if (l > max) {
                        max = l;
                    }
                }
            }
        }
        return max;
    }

    void OcTreeNode::updateOccupancyChildren() {
        this->setLogOdds(this->getMaxChildLogOdds());  // conservative
    }


    /***********************************************************************************************************
     * Color
     **********************************************************************************************************/
    void OcTreeNode::setColor(Color c) {
        this->color = c;
    }

    void OcTreeNode::setColor(uint8_t r, uint8_t g, uint8_t b) {
        this->color = Color(r, g, b);
    }

    Color OcTreeNode::getColor() const {
        return color;
    }

    bool OcTreeNode::isColorSet() const {
        return ((color.r != 0) || (color.g != 0) || (color.b != 0));
    }

    Color OcTreeNode::getAverageChildColor() const {
        int mr = 0;
        int mg = 0;
        int mb = 0;
        int c = 0;

        if (children != nullptr) {
            for (int i = 0; i < 8; i++) {
                OcTreeNode *child = children[i];

                if (child != nullptr && child->isColorSet()) {
                    mr += child->getColor().r;
                    mg += child->getColor().g;
                    mb += child->getColor().b;
                    ++c;
                }
            }
        }

        if (c > 0) {
            mr /= c;
            mg /= c;
            mb /= c;
            return Color((uint8_t) mr, (uint8_t) mg, (uint8_t) mb);
        } else { // no child had a color other than white
            return Color(255, 255, 255);
        }
    }


    void OcTreeNode::updateColorChildren() {
        color = getAverageChildColor();
    }

    void OcTreeNode::update_label_probability(const std::string &name, int id, double probability) {
        auto key = label_map.find(name);
        if (key == label_map.end()) {
            label_map[name] = Label(id, probability, 1);
        } else {
            auto &label = label_map[name];
            double prob = label.probability;
            if ((*key).second.id != id) {
                label.id = id;
            }
            label.probability = (prob + probability) / label.number_of_observation;
            ++(label.number_of_observation);
        }
    }

    int OcTreeNode::get_id(const std::string &name) {
        auto key = label_map.find(name);
        if (key != label_map.end()) {
            return (*key).second.id;
        } else {
            return -1;
        }
    }


} // end namespace



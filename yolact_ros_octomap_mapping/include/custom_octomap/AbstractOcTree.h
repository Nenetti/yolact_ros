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

#ifndef CUSTOM_OCTOMAP_ABSTRACT_OCTREE_H
#define CUSTOM_OCTOMAP_ABSTRACT_OCTREE_H

#include <cstddef>
#include <fstream>
#include <string>
#include <iostream>
#include <map>

#include <custom_octomap/Types.h>

namespace custom_octomap {

    /**
     * This abstract class is an interface to all octrees and provides a
     * factory design pattern for readin and writing all kinds of OcTrees
     * to files (see read()).
     */
    class AbstractOcTree {

        public:

            /***********************************************************************************************************
             * Constructor
             **********************************************************************************************************/
            AbstractOcTree();

            ~AbstractOcTree();

            virtual AbstractOcTree *create() const = 0;

            static AbstractOcTree *createTree(const std::string& id, double res);

            virtual std::string getTreeType() const = 0;

            /***********************************************************************************************************
             * Tree
             **********************************************************************************************************/
            virtual void prune() = 0;

            virtual void expand() = 0;

            virtual void clear() = 0;

            /***********************************************************************************************************
             * Resolution
             **********************************************************************************************************/
            virtual double getResolution() const = 0;

            virtual void setResolution(double res) = 0;

            virtual size_t size() const = 0;

            /***********************************************************************************************************
             * Memory
             **********************************************************************************************************/
            virtual size_t memoryUsage() const = 0;

            virtual size_t memoryUsageNode() const = 0;

            /***********************************************************************************************************
             * Metric
             **********************************************************************************************************/
            virtual void getMetricMin(double &x, double &y, double &z) = 0;

            virtual void getMetricMin(double &x, double &y, double &z) const = 0;

            virtual void getMetricMax(double &x, double &y, double &z) = 0;

            virtual void getMetricMax(double &x, double &y, double &z) const = 0;

            virtual void getMetricSize(double &x, double &y, double &z) = 0;

            /***********************************************************************************************************
             * File IO
             **********************************************************************************************************/
            static AbstractOcTree *read(const std::string &filename);

            static AbstractOcTree *read(std::istream &s);

            virtual std::istream &readData(std::istream &s) = 0;

            bool write(const std::string &filename) const;

            bool write(std::ostream &s) const;

            virtual std::ostream &writeData(std::ostream &s) const = 0;

        private:
            static std::map<std::string, AbstractOcTree *> &classIDMapping();

        protected:
            static bool readHeader(std::istream &s, std::string &id, unsigned &size, double &res);

            static void registerTreeType(AbstractOcTree *tree);

            static const std::string fileHeader;
    };


} // end namespace


#endif

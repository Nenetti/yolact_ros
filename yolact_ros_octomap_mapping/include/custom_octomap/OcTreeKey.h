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

#ifndef CUSTOM_OCTOMAP_OCTREE_KEY_H
#define CUSTOM_OCTOMAP_OCTREE_KEY_H

/* According to c++ standard including this header has no practical effect
 * but it can be used to determine the c++ standard library implementation.
 */
#include <ciso646>
#include <cstdio>
#include <vector>
#include <cstdint>
#include <cassert>

/* Libc++ does not implement the TR1 namespace, all c++11 related functionality
 * is instead implemented in the std namespace.
 */
#if defined(__GNUC__) && !defined(_LIBCPP_VERSION)

#include <tr1/unordered_set>
#include <tr1/unordered_map>


namespace custom_octomap {
    namespace unordered_ns = std::tr1;
};
#else
#include <unordered_set>
#include <unordered_map>
namespace octomap {
  namespace unordered_ns = std;
}
#endif

namespace custom_octomap {


    typedef uint16_t key_type;

    class OcTreeKey {

        public:

            /***********************************************************************************************************
             * Constructor
             **********************************************************************************************************/
            OcTreeKey();

            OcTreeKey(key_type a, key_type b, key_type c);

            OcTreeKey(const OcTreeKey &other);

            /***********************************************************************************************************
             * Operator
             **********************************************************************************************************/
            bool operator==(const OcTreeKey &other) const;

            bool operator!=(const OcTreeKey &other) const;

            OcTreeKey &operator=(const OcTreeKey &other);

            const key_type &operator[](unsigned int i) const;

            key_type &operator[](unsigned int i);

            key_type k[3];

            struct KeyHash {
                size_t operator()(const OcTreeKey &key) const;
            };

            /***********************************************************************************************************
             * Child
             **********************************************************************************************************/
            static void computeChildKey(unsigned int pos, key_type center_offset_key, const OcTreeKey &parent_key, OcTreeKey &child_key);

            static uint8_t computeChildIndex(const OcTreeKey &key, int depth);

            static OcTreeKey computeIndexKey(key_type level, const OcTreeKey &key);

    };

    typedef unordered_ns::unordered_set<OcTreeKey, OcTreeKey::KeyHash> KeySet;
    typedef unordered_ns::unordered_map<OcTreeKey, int, OcTreeKey::KeyHash> KeyIntMap;
    typedef unordered_ns::unordered_map<OcTreeKey, bool, OcTreeKey::KeyHash> KeyBoolMap;

    class KeyRay {
        public:

            KeyRay();

            KeyRay(const KeyRay &other);

            void reset();

            void addKey(const OcTreeKey &k);

            size_t size() const;

            size_t sizeMax() const;

            typedef std::vector<OcTreeKey>::iterator iterator;
            typedef std::vector<OcTreeKey>::const_iterator const_iterator;
            typedef std::vector<OcTreeKey>::reverse_iterator reverse_iterator;

            iterator begin();

            iterator end();

            const_iterator begin() const;

            const_iterator end() const;

            reverse_iterator rbegin();

            reverse_iterator rend();

        private:
            std::vector<OcTreeKey> ray;
            std::vector<OcTreeKey>::iterator end_of_ray;
            const static size_t maxSize = 100000;
    };


} // namespace

#endif

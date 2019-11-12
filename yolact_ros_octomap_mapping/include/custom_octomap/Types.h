//
// Created by ubuntu on 2019/09/19.
//

#ifndef OCTOMAP_SERVER_TYPES_H
#define OCTOMAP_SERVER_TYPES_H

#include <cstdio>
#include <vector>
#include <list>
#include <cinttypes>

#include <octomap/math/Vector3.h>
#include <octomap/math/Pose6D.h>

namespace custom_octomap {

    ///Use Vector3 (float precision) as a point3d in octomap
    typedef octomath::Vector3 point3d;
    /// Use our Pose6D (float precision) as pose6d in octomap
    typedef octomath::Pose6D pose6d;

    typedef std::vector<octomath::Vector3> point3d_collection;
    typedef std::list<octomath::Vector3> point3d_list;

    /// A voxel defined by its center point3d and its side length
    typedef std::pair<point3d, double> OcTreeVolume;

}

#ifdef NDEBUG
#ifndef OCTOMAP_NODEBUGOUT
#define OCTOMAP_NODEBUGOUT
#endif
#endif

#ifdef OCTOMAP_NODEBUGOUT
#define OCTOMAP_DEBUG(...)       (void)0
#define OCTOMAP_DEBUG_STR(...)   (void)0
#else
#define OCTOMAP_DEBUG(...)        fprintf(stderr, __VA_ARGS__), fflush(stderr)
#define OCTOMAP_DEBUG_STR(args)   std::cerr << args << std::endl
#endif

#define OCTOMAP_WARNING(...)      fprintf(stderr, "WARNING: "), fprintf(stderr, __VA_ARGS__), fflush(stderr)
#define OCTOMAP_WARNING_STR(args) std::cerr << "WARNING: " << args << std::endl
#define OCTOMAP_ERROR(...)        fprintf(stderr, "ERROR: "), fprintf(stderr, __VA_ARGS__), fflush(stderr)
#define OCTOMAP_ERROR_STR(args)   std::cerr << "ERROR: " << args << std::endl

#endif //OCTOMAP_SERVER_TYPES_H


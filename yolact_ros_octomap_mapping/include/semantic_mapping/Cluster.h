
#ifndef SEMANTIC_MAPPING_CLUSTER_H
#define SEMANTIC_MAPPING_CLUSTER_H

#include <vector>
#include <pcl/point_types.h>
#include <custom_octomap/Color.h>
#include <yolact_ros/Segment.h>

namespace semantic_mapping {


    class Cluster {
        public:
            explicit Cluster(std::vector<int> &indices, double depth);

            std::vector<int> indices;

            double depth;

    };


} // end namespace

#endif

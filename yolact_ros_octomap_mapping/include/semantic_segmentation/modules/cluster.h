
#ifndef SEMANTIC_MAPPING_CLUSTER_H
#define SEMANTIC_MAPPING_CLUSTER_H

#include <vector>
#include <yolact_ros_msgs/Segment.h>

namespace semantic_segmentation {


    class Cluster {
        public:
            explicit Cluster(std::vector<int> &indices, double depth);

            std::vector<int> indices;

            double depth;

    };


} // end namespace

#endif

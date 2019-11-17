//
// Created by ubuntu on 2019/10/31.
//

#ifndef SEMANTIC_MAPPING_SEGMENT_H
#define SEMANTIC_MAPPING_SEGMENT_H

#include <vector>
#include <yolact_ros/Segment.h>
#include <semantic_mapping/cluster.h>

namespace semantic_mapping {


    class Segment {

        public:

            void set_rgb(int r, int g, int b);

            void set_average_coordinate(double ave_x, double ave_y, double ave_z);

            void set_range_coordinate(double min_x, double max_x, double min_y, double max_y, double min_z, double max_z);

            explicit Segment();

            yolact_ros::Segment segment;

            std::vector<int> mask;

            std::vector<Cluster> clusters;
            std::vector<int> cluster_occupied;

            int r, g, b;
            double min_x, max_x, min_y, max_y, min_z, max_z;
            double ave_x, ave_y, ave_z;
            bool is_available;

    };


} // end namespace

#endif //SEMANTIC_MAPPING_SEGMENT_H

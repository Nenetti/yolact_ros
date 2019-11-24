//
// Created by ubuntu on 2019/11/23.
//

#ifndef YOLACT_ROS_SEMANTIC_CLOUD_COLOR_H
#define YOLACT_ROS_SEMANTIC_CLOUD_COLOR_H


#include <cstdint>

namespace semantic_segmentation {

    class Color {

        public:
            Color();

            Color(uint8_t _r, uint8_t _g, uint8_t _b);

            uint8_t r, g, b;

    };
}

#endif //YOLACT_ROS_SEMANTIC_CLOUD_COLOR_H

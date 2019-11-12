//
// Created by ubuntu on 2019/10/31.
//

#include <semantic_mapping/Segment.h>

namespace semantic_mapping {

    Segment::Segment() {

    }

    /*******************************************************************************************************************
     * Set rgb
     *
     * @param r
     * @param g
     * @param b
     */
    void Segment::set_rgb(int r, int g, int b) {
        this->r = r;
        this->g = g;
        this->b = b;
    }

    /*******************************************************************************************************************
     * Set average's coordinate
     * @param ave_x
     * @param ave_y
     * @param ave_z
     */
    void Segment::set_average_coordinate(double ave_x, double ave_y, double ave_z) {
        this->ave_x = ave_x;
        this->ave_y = ave_y;
        this->ave_z = ave_z;
    }

    /*******************************************************************************************************************
     *
     * Set segment's range
     *
     * @param min_x
     * @param max_x
     * @param min_y
     * @param max_y
     * @param min_z
     * @param max_z
     */
    void Segment::set_range_coordinate(double min_x, double max_x, double min_y, double max_y, double min_z, double max_z) {
        this->min_x = min_x;
        this->max_x = max_x;
        this->min_y = min_y;
        this->max_y = max_y;
        this->min_z = min_z;
        this->max_z = max_z;
    }
}

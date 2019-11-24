
#include <semantic_segmentation/modules/cluster.h>

namespace semantic_segmentation {

    /*******************************************************************************************************************
     * コントラクター
     *
     * @param indices
     * @param depth
     */
    Cluster::Cluster(std::vector<int> &indices, double depth) : indices(indices), depth(depth) {}

}



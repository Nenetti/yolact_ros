
#include <semantic_segmentation/segmenatation_server.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "semantic_segmentation");
    const ros::NodeHandle &private_nh = ros::NodeHandle("~");

    semantic_segmentation::SegmentationServer server;

    ros::spin();

    return 0;
}

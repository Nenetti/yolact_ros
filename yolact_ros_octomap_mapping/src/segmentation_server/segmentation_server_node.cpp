
#include <ros/ros.h>
#include <segmentation_server/segmenatation_server.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "segmentation_server");
    const ros::NodeHandle &private_nh = ros::NodeHandle("~");

    segmentation_server::SegmentationServer server;

    ros::spin();

    return 0;
}

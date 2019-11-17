//
// Created by ubuntu on 2019/10/19.
//

#ifndef SEMANTIC_MAPPING_SEGMENTATIONCLIENT_H
#define SEMANTIC_MAPPING_SEGMENTATIONCLIENT_H

#include <sensor_msgs/PointCloud2.h>
#include <yolact_ros/SegmentationResult.h>
#include <actionlib/client/simple_action_client.h>
#include <yolact_ros/SegmentationAction.h>
#include <semantic_mapping/segment.h>

namespace semantic_mapping {

    class SegmentationClient {

        public:

            SegmentationClient();

            bool send_segmentation_request(const sensor_msgs::PointCloud2_<std::allocator<void>>::ConstPtr &cloud_msg, std::vector<Segment> &segments);

            static void encoding_pixel_to_mask(std::vector<Segment> &segments, std::vector<int8_t> &mask_map);

            static void sort_segments(std::vector<Segment> &segments);

        protected:
            ros::NodeHandle m_nh;
            ros::Publisher m_image_pub;
            actionlib::SimpleActionClient<yolact_ros::SegmentationAction> *m_segmentationClient;

            void publish_image(const sensor_msgs::Image &image, const yolact_ros::Segments &segments);

            void to_segments(const yolact_ros::Segments &input, std::vector<Segment> &output);

            void to_segments(const yolact_ros::Segment &input, Segment &output);
    };

}

#endif //CUSTOM_OCTOMAP_SERVER_SEGMENTATIONCLIENT_H

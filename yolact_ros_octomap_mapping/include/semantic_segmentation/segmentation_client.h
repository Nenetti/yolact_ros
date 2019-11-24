//
// Created by ubuntu on 2019/10/19.
//

#ifndef SEMANTIC_CLOUD_SEMANTICCLOUDCLIENT_H
#define SEMANTIC_CLOUD_SEMANTICCLOUDCLIENT_H

#include <sensor_msgs/PointCloud2.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>

#include <semantic_segmentation/modules/semantic_object.h>
#include <yolact_ros_octomap_mapping_msgs/RequestSemanticCloudAction.h>
#include <yolact_ros_octomap_mapping_msgs/RequestSemanticCloudGoal.h>

namespace semantic_segmentation {

    class SemanticCloudClient {

        public:

            SemanticCloudClient();

            bool send_segmentation_request(const sensor_msgs::PointCloud2_<std::allocator<void>>::ConstPtr &cloud_msg, std::vector<SemanticObject> &segments);

        protected:
            ros::NodeHandle m_nh;
            actionlib::SimpleActionClient<yolact_ros_octomap_mapping_msgs::RequestSemanticCloudAction> *m_segmentationClient;
    };

}

#endif //CUSTOM_OCTOMAP_SERVER_SEGMENTATIONCLIENT_H

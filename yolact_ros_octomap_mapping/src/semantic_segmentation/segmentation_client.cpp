//
// Created by ubuntu on 2019/10/19.
//

#include <semantic_segmentation/segmentation_client.h>

namespace semantic_segmentation {

    /*******************************************************************************************************************
     * コンストラクター
     */
    SemanticCloudClient::SemanticCloudClient() {
        m_segmentationClient = new actionlib::SimpleActionClient<yolact_ros_octomap_mapping_msgs::RequestSemanticCloudAction>
                ("yolact_ros/request_semantic_cloud", true);
    }

    /*******************************************************************************************************************
     * Segmentationのモジュールに対してリクエストを送る
     *
     * @param cloud_msg
     * @param segments
     * @return
     */
    bool SemanticCloudClient::send_segmentation_request(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg, std::vector<SemanticObject> &segments) {

        yolact_ros_octomap_mapping_msgs::RequestSemanticCloudGoal goal;
        goal.cloud = (*cloud_msg);

        m_segmentationClient->sendGoal(goal);
        m_segmentationClient->waitForResult(ros::Duration(10.0));
        if (m_segmentationClient->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            auto &result = m_segmentationClient->getResult().get()->semantic_objects;
            segments.resize(result.objetcs.size());
            for (int i = 0; i < int(segments.size()); ++i) {
                auto &segment_msg = result.objetcs[i];
                auto &segment = segments[i];
                segment.Class = segment_msg.Class;
                segment.probability = segment_msg.probability;
                segment.masks = segment_msg.masks;
            }
            return true;
        }
        return false;
    }
}
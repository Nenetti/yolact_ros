//
// Created by ubuntu on 2019/11/14.
//

#include <semantic_segmentation/segmenatation_server.h>

namespace semantic_segmentation {

    using pcl::PointCloud;
    using pcl::PointXYZRGB;

    /*******************************************************************************************************************
     * コンストラクター
     */
    SegmentationServer::SegmentationServer(ros::NodeHandle nh) :
            m_baseFrameId(""),
            m_worldFrameId(""),
            m_is_cloud_pub(true),
            m_is_segmentation_filter_pub(false),
            m_is_clustering_pub(false),
            m_is_marker_pub(true),
            m_ground_distance(DBL_MIN),
            m_ceiling_distance(DBL_MAX) {

        nh.param("base_frame_id", m_baseFrameId, m_baseFrameId);
        nh.param("frame_id", m_worldFrameId, m_worldFrameId);
        nh.param("publish/segmentation_cloud", m_is_cloud_pub, m_is_cloud_pub);
        nh.param("publish/segmentation_filter_image", m_is_segmentation_filter_pub, m_is_segmentation_filter_pub);
        nh.param("publish/clustering_edge_image", m_is_clustering_pub, m_is_clustering_pub);
        nh.param("publish/segmentation_marker", m_is_marker_pub, m_is_marker_pub);
        nh.param("filter/ground_distance", m_ground_distance, m_ground_distance);
        nh.param("filter/ceiling_distance", m_ceiling_distance, m_ceiling_distance);

        m_semantic_map.set_filter(m_ground_distance, m_ceiling_distance);

        m_cloud_sub = m_nh.subscribe<sensor_msgs::PointCloud2>("cloud_in", 10, &SegmentationServer::call_back, this);

        m_marker_client = semantic_segmentation::MarkerClient();

        m_segmentation_client = new actionlib::SimpleActionClient<yolact_ros_msgs::CheckForObjectsAction>("yolact_ros/check_for_objects", true);
        m_segmentation_filter_pub = m_nh.advertise<sensor_msgs::Image>("geometric_map", 1, true);
        m_clustering_pub = m_nh.advertise<sensor_msgs::Image>("clustering_map", 1, true);
        m_cloud_pub = m_nh.advertise<sensor_msgs::PointCloud2>("cloud_out", 1, true);
        m_action_server = new actionlib::SimpleActionServer<yolact_ros_octomap_mapping_msgs::RequestSemanticCloudAction>
                ("yolact_ros/request_semantic_cloud", false);
        m_action_server->registerGoalCallback(boost::bind(&SegmentationServer::action_call_back, this));
        m_action_server->start();
    }

    void SegmentationServer::action_call_back() {
        ROS_INFO("Subscribe");
        auto goal = m_action_server->acceptNewGoal();
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        std::vector<semantic_segmentation::Segment> segments;
        std::vector<semantic_segmentation::Cluster> clusters;
        if (apply_segmentation_to_cloud((*goal).cloud, cloud, segments, clusters)) {
            yolact_ros_octomap_mapping_msgs::RequestSemanticCloudResult result;
            result.id = (*goal).id;
            to_segment_msg(segments, result.semantic_objects);
            m_action_server->setSucceeded(result);
            publish_all(cloud, segments, clusters);
        } else {
            m_action_server->setAborted();
        }
    }

    /*******************************************************************************************************************
     * PointCloudのコールバック
     * @param cloud_msg
     */
    void SegmentationServer::call_back(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg) {
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        std::vector<semantic_segmentation::Segment> segments;
        std::vector<semantic_segmentation::Cluster> clusters;
        if (apply_segmentation_to_cloud(*cloud_msg, cloud, segments, clusters)) {
            publish_all(cloud, segments, clusters);
        }
    }

    bool SegmentationServer::apply_segmentation_to_cloud(const sensor_msgs::PointCloud2 &cloud_msg,
                                                         PointCloud<PointXYZRGB> &cloud,
                                                         std::vector<semantic_segmentation::Segment> &segments,
                                                         std::vector<semantic_segmentation::Cluster> &clusters) {
        //************************************************************************************************************//
        // Init
        //************************************************************************************************************//
        ROS_INFO("Subscribe PointCloud2 msg.");
        ros::WallTime startTime = ros::WallTime::now();
        m_width = cloud_msg.width;
        m_height = cloud_msg.height;

        //************************************************************************************************************//
        // Transform PointCloud2
        //************************************************************************************************************//
        sensor_msgs::PointCloud2 base_cloud;
        sensor_msgs::PointCloud2 world_cloud;
        try {
            pcl_ros::transformPointCloud(m_baseFrameId, cloud_msg, base_cloud, m_tfListener);
            pcl_ros::transformPointCloud(m_worldFrameId, base_cloud, world_cloud, m_tfListener);
        } catch (std::exception e) {
            ROS_INFO("%s\n", e.what());
            return false;
        }

        pcl::fromROSMsg(world_cloud, cloud);

        //************************************************************************************************************//
        // Action
        //************************************************************************************************************//
        yolact_ros_msgs::CheckForObjectsGoal goal;
        sensor_msgs::Image image;
        pcl::toROSMsg(cloud_msg, image);
        goal.image = image;
        m_segmentation_client->sendGoal(goal);
        m_segmentation_client->waitForResult(ros::Duration(5.0));
        if (m_segmentation_client->getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_WARN("Segmentation required failed.");
            return false;
        }
        auto &result = m_segmentation_client->getResult().get()->segments;

        convert_segment_data(result, segments);
        m_semantic_map.to_segmentation(cloud, segments, clusters);
        ROS_INFO("Done (%f sec)\n", (ros::WallTime::now() - startTime).toSec());
        return true;
    }

    void SegmentationServer::convert_segment_data(const yolact_ros_msgs::Segments &segment_msg, std::vector<semantic_segmentation::Segment> &segments) {
        segments.resize(segment_msg.segments.size());
        for (int i = 0; i < int(segment_msg.segments.size()); ++i) {
            auto &segment = segments[i];
            auto &info = segment_msg.segments[i];
            segment.segment = info;
            segment.mask.resize(info.x_masks.size());
            for (int k = 0; k < int(segment.mask.size()); ++k) {
                segment.mask[k] = info.x_masks[k] + info.y_masks[k] * m_width;
            }
        }
    }

    void SegmentationServer::to_segment_msg(const std::vector<semantic_segmentation::Segment> &segments,
                                            yolact_ros_octomap_mapping_msgs::SemanticObjects &segments_msg) {
        segments_msg.objetcs.resize(segments.size());
        for (int i = 0; i < int(segments.size()); ++i) {
            auto &segment = segments[i];
            auto &segment_msg = segments_msg.objetcs[i];
            segment_msg.Class = segment.segment.Class;
            segment_msg.probability = segment.segment.probability;
            int size = 0;
            for (const auto &cluster : segment.clusters) {
                size += int(cluster.indices.size());
            }
            for (const auto &cluster : segment.clusters) {
                segment_msg.masks.insert(segment_msg.masks.end(), cluster.indices.begin(), cluster.indices.end());
            }
        }
    }

    void SegmentationServer::publish_all(const PointCloud<PointXYZRGB> &cloud, const std::vector<semantic_segmentation::Segment> &segments,
                                         const std::vector<semantic_segmentation::Cluster> &clusters) {
        if (m_is_cloud_pub) {
            publish_segmentation_cloud(cloud, segments);
        }
        if (m_is_segmentation_filter_pub) {
            publish_segmentation_filter_image(segments);
        }
        if (m_is_clustering_pub) {
            publish_clustering_image(clusters);
        }
        if (m_is_marker_pub) {
            m_marker_client.publish_segment_info(cloud, segments);
        }
    }

    /*******************************************************************************************************************
     * フィルターをかけたセグメンテーションの結果をPointCloudでPublish
     * @param cloud
     * @param segments
     */
    void SegmentationServer::publish_segmentation_cloud(const PointCloud<PointXYZRGB> &cloud, const std::vector<semantic_segmentation::Segment> &segments) {
        PointCloud<PointXYZRGB> pcl_cloud;
        pcl::copyPointCloud(cloud, pcl_cloud);
        for (auto &segment:segments) {
            for (auto &cluster:segment.clusters) {
                for (auto &index:cluster.indices) {
                    auto &point = pcl_cloud[index];
                    point.r = segment.r;
                    point.g = segment.g;
                    point.b = segment.b;
                }
            }
        }
        sensor_msgs::PointCloud2 publish_cloud_msg;
        pcl::toROSMsg(pcl_cloud, publish_cloud_msg);
        publish_cloud_msg.header.frame_id = m_worldFrameId;
        publish_cloud_msg.header.stamp = ros::Time::now();
        m_cloud_pub.publish(publish_cloud_msg);
    }

    /*******************************************************************************************************************
     * クラスタリングした結果を画像でPublish
     * @param clusters
     */
    void SegmentationServer::publish_clustering_image(const std::vector<semantic_segmentation::Cluster> &clusters) {
        cv::Mat image(m_height, m_width, CV_8UC3, cv::Scalar(255, 255, 255));
        std::random_device rnd;
        std::mt19937 mt(rnd());
        std::uniform_int_distribution<> rand100(0, 255);
        for (auto &cluster:clusters) {
            int r = rand100(mt);
            int g = rand100(mt);
            int b = rand100(mt);
            for (auto &index:cluster.indices) {
                int x = int(index % m_width);
                int y = int(index / m_width);
                auto &src = image.at<cv::Vec3b>(y, x);
                src[0] = r;
                src[1] = g;
                src[2] = b;
            }
        }
        sensor_msgs::ImagePtr ros_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
        m_clustering_pub.publish(ros_image);
    }

    /*******************************************************************************************************************
     * フィルターをかけたセグメンテーションの結果を画像でPublish
     * @param segments
     */
    void SegmentationServer::publish_segmentation_filter_image(const std::vector<semantic_segmentation::Segment> &segments) {
        cv::Mat image(m_height, m_width, CV_8UC3, cv::Scalar(255, 255, 255));
        for (auto &segment:segments) {
            for (auto &cluster:segment.clusters) {
                for (auto &index:cluster.indices) {
                    int x = int(index % m_width);
                    int y = int(index / m_width);
                    auto &src = image.at<cv::Vec3b>(y, x);
                    src[0] = segment.b;
                    src[1] = segment.g;
                    src[2] = segment.r;
                }
            }
        }
        sensor_msgs::ImagePtr ros_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
        m_segmentation_filter_pub.publish(ros_image);
    }

}
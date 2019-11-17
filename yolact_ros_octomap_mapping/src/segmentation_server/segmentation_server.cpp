//
// Created by ubuntu on 2019/11/14.
//

#include <segmentation_server/segmenatation_server.h>
#include <pcl_ros/transforms.h>


namespace segmentation_server {

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
            m_is_marker_info_pub(true),
            m_ground_distance(DBL_MIN),
            m_ceiling_distance(DBL_MAX) {

        nh.param("base_frame_id", m_baseFrameId, m_baseFrameId);
        nh.param("frame_id", m_worldFrameId, m_worldFrameId);
        nh.param("publish/segmentation_cloud", m_is_cloud_pub, m_is_cloud_pub);
        nh.param("publish/segmentation_filter_image", m_is_segmentation_filter_pub, m_is_segmentation_filter_pub);
        nh.param("publish/clustering_edge_image", m_is_clustering_pub, m_is_clustering_pub);
        nh.param("publish/segmentation_marker", m_is_marker_pub, m_is_marker_pub);
        nh.param("publish/segmentation_marker_info", m_is_marker_info_pub, m_is_marker_info_pub);
        nh.param("filter/ground_distance", m_ground_distance, m_ground_distance);
        nh.param("filter/ceiling_distance", m_ceiling_distance, m_ceiling_distance);

        m_semantic_map.set_filter(m_ground_distance, m_ceiling_distance);

        m_cloud_sub = m_nh.subscribe<sensor_msgs::PointCloud2>("cloud_in", 10, &SegmentationServer::call_back, this);

        m_marker_client = semantic_mapping::MarkerClient();

        m_segmentation_client = new actionlib::SimpleActionClient<yolact_ros::SegmentationAction>("/yolact_ros/check_for_objects", true);
        m_segmentation_filter_pub = m_nh.advertise<sensor_msgs::Image>("geometric_map", 1, true);
        m_clustering_pub = m_nh.advertise<sensor_msgs::Image>("clustering_map", 1, true);
        m_cloud_pub = m_nh.advertise<sensor_msgs::PointCloud2>("cloud_out", 1, true);
        m_marker_info_pub = m_nh.advertise<yolact_ros_octomap_mapping::SegmentMarkers>("marker_info", 1, true);
    }

    /*******************************************************************************************************************
     * PointCloudのコールバック
     * @param cloud_msg
     */
    void SegmentationServer::call_back(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg) {
        //************************************************************************************************************//
        // Transform PointCloud2
        //************************************************************************************************************//
        ROS_INFO("Subscribe PointCloud2 msg.");
        ros::WallTime startTime = ros::WallTime::now();
        m_width = cloud_msg->width;
        m_height = cloud_msg->height;

        //************************************************************************************************************//
        // Transform PointCloud2
        //************************************************************************************************************//
        sensor_msgs::PointCloud2 base_cloud;
        sensor_msgs::PointCloud2 world_cloud;

        pcl_ros::transformPointCloud(m_baseFrameId, *cloud_msg, base_cloud, m_tfListener);
        pcl_ros::transformPointCloud(m_worldFrameId, base_cloud, world_cloud, m_tfListener);

        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        pcl::fromROSMsg(world_cloud, cloud);

        //************************************************************************************************************//
        // Action
        //************************************************************************************************************//
        yolact_ros::SegmentationGoal goal;
        sensor_msgs::Image image;
        pcl::toROSMsg(*cloud_msg, image);
        goal.image = image;
        m_segmentation_client->sendGoal(goal);
        m_segmentation_client->waitForResult(ros::Duration(5.0));

        if (m_segmentation_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            auto &result = m_segmentation_client->getResult().get()->segments;
            apply_segmentation_to_cloud(cloud, result);
        } else {
            ROS_WARN("Segmentation required failed.");
        }
        ROS_INFO("Done (%f sec)\n", (ros::WallTime::now() - startTime).toSec());
    }

    void SegmentationServer::apply_segmentation_to_cloud(PointCloud<PointXYZRGB> &cloud, const yolact_ros::Segments &segment_msg) {
        if (!segment_msg.segments.empty()) {
            std::vector<semantic_mapping::Segment> segments;
            std::vector<semantic_mapping::Cluster> clusters;

            convert_segment_data(segment_msg, segments);

            m_semantic_map.to_segmentation(cloud, segments, clusters);

            publish_all(cloud, segments, clusters);
        }
    }

    void SegmentationServer::convert_segment_data(const yolact_ros::Segments &segment_msg, std::vector<semantic_mapping::Segment> &segments) {
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

    void SegmentationServer::publish_all(const PointCloud<PointXYZRGB> &cloud, const std::vector<semantic_mapping::Segment> &segments,
                                         const std::vector<semantic_mapping::Cluster> &clusters) {
        if (m_is_cloud_pub) {
            publish_segmentation_cloud(cloud, segments);
        }
        if (m_is_segmentation_filter_pub) {
            publish_segmentation_filter_image(segments);
        }
        if (m_is_clustering_pub) {
            publish_clustering_image(clusters);
        }
        if (m_is_marker_info_pub) {
            publish_marker_info(segments);
        }
        if (m_is_marker_pub) {
            m_marker_client.publish_segment_info(cloud, segments);
        }
    }

    /*******************************************************************************************************************
     * マーカー情報をPublishする
     * @param segments
     */
    void SegmentationServer::publish_marker_info(const std::vector<semantic_mapping::Segment> &segments) {
        yolact_ros_octomap_mapping::SegmentMarkers markers;
        markers.header.stamp = ros::Time::now();
        markers.markers.resize(segments.size());
        for (int i = 0; i < int(segments.size()); ++i) {
            auto &marker = markers.markers[i];
            auto &segment = segments[i];
            marker.Class = segment.segment.Class;
            marker.probability = segment.segment.probability;
            marker.xmin = segment.min_x;
            marker.ymin = segment.min_y;
            marker.zmin = segment.min_z;
            marker.xmax = segment.max_x;
            marker.ymax = segment.max_y;
            marker.zmax = segment.max_z;
            marker.color.r = segment.r / 255.0;
            marker.color.g = segment.g / 255.0;
            marker.color.b = segment.b / 255.0;
            marker.color.a = 1.0;
        }
        m_marker_info_pub.publish(markers);
    }

    /*******************************************************************************************************************
     * フィルターをかけたセグメンテーションの結果をPointCloudでPublish
     * @param cloud
     * @param segments
     */
    void SegmentationServer::publish_segmentation_cloud(const PointCloud<PointXYZRGB> &cloud, const std::vector<semantic_mapping::Segment> &segments) {
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
    void SegmentationServer::publish_clustering_image(const std::vector<semantic_mapping::Cluster> &clusters) {
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
    void SegmentationServer::publish_segmentation_filter_image(const std::vector<semantic_mapping::Segment> &segments) {
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
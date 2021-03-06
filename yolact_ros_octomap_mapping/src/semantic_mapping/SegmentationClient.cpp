//
// Created by ubuntu on 2019/10/19.
//

#include <semantic_mapping/SegmentationClient.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <pcl_conversions/pcl_conversions.h>
#include <yolact_ros/SegmentationGoal.h>
#include <yolact_ros/Segments.h>
#include <yolact_ros/Segment.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <random>


namespace semantic_mapping {

    /*******************************************************************************************************************
     * コンストラクター
     */
    SegmentationClient::SegmentationClient() {
        m_segmentationClient = new actionlib::SimpleActionClient<yolact_ros::SegmentationAction>("/yolact_ros/check_for_objects", true);
        m_image_pub = m_nh.advertise<sensor_msgs::CompressedImage>("/yolact_ros/segmentation/image/compressed", 1, true);
    }

    /*******************************************************************************************************************
     * Segmentationのモジュールに対してリクエストを送る
     *
     * @param cloud_msg
     * @param segments
     * @return
     */
    bool SegmentationClient::send_segmentation_request(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg, std::vector<Segment> &segments) {
        sensor_msgs::Image image;
        pcl::toROSMsg(*cloud_msg, image);

        yolact_ros::SegmentationGoal goal;
        goal.image = image;

        m_segmentationClient->sendGoal(goal);
        m_segmentationClient->waitForResult(ros::Duration(5.0));
        if (m_segmentationClient->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            auto *result = &m_segmentationClient->getResult().get()->segments;
            segments.resize(result->segments.size());
            for (int i = 0; i < int(segments.size()); ++i) {
                segments[i].segment = result->segments[i];
            }
            sort_segments(segments);
            publish_image(image, *result);
            return true;
        }
        return false;
    }

    /*******************************************************************************************************************
     * Segmentationの結果をイメージとしてPublishする
     *
     * @param image
     * @param segments
     */
    void SegmentationClient::publish_image(const sensor_msgs::Image &image, const yolact_ros::Segments &segments) {
        cv::Mat cv_image = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8)->image;
        cv::Mat cv_mask;
        cv::Mat mask_image;
        cv_image.copyTo(cv_mask);
        cv::Scalar bounding_box_colors[int(segments.segments.size())];
        cv::Scalar text_color(255, 255, 255);
//        std::random_device rnd;
        std::mt19937 mt(0);
        std::uniform_int_distribution<> rand100(0, 255);
        for (auto &color:bounding_box_colors) {
            color = cv::Scalar(rand100(mt), rand100(mt), rand100(mt));
        }
        int x, y;
        for (int i = 0; i < int(segments.segments.size()); ++i) {
            cv::Scalar *color = &bounding_box_colors[i];
            const yolact_ros::Segment *segment = &segments.segments[i];
            for (int k = 0; k < int(segment->x_masks.size()); ++k) {
                x = segment->x_masks[k];
                y = segment->y_masks[k];
                cv::Vec3b *src = &cv_mask.at<cv::Vec3b>(y, x);
                (*src)[0] = (*color)[0];
                (*src)[1] = (*color)[1];
                (*src)[2] = (*color)[2];
            }
        }
        cv::addWeighted(cv_image, 0.5, cv_mask, 0.5, 20, mask_image);
        int base_line = 0;
        for (int i = 0; i < int(segments.segments.size()); ++i) {
            const yolact_ros::Segment *segment = &segments.segments[i];
            std::string text = segment->Class;
            double probability = segment->probability;
            text += ": " + std::to_string(probability);
            cv::Scalar *color = &bounding_box_colors[i];
            int x1 = segment->xmin;
            int x2 = segment->xmax;
            int y1 = segment->ymin;
            int y2 = segment->ymax;
            cv::rectangle(mask_image, cv::Point(x1, y1), cv::Point(x2, y2), *color, 2);
            cv::Size text_size = cv::getTextSize(text, cv::FONT_HERSHEY_DUPLEX, 0.6, 1, &base_line);
            cv::rectangle(mask_image, cv::Point(x1, y1), cv::Point(x1 + text_size.width, y1 - text_size.height - 4), *color, -1);
            cv::putText(mask_image, text, cv::Point(x1, y1 - 3), cv::FONT_HERSHEY_DUPLEX, 0.6, text_color, 1, 16);
        }
        sensor_msgs::CompressedImagePtr msg = cv_bridge::CvImage(image.header, "bgr8", mask_image).toCompressedImageMsg();
        m_image_pub.publish(msg);
        cv::imwrite("/root/HSR/catkin_ws/src/yolact_ros/mask_image.jpg", mask_image);
    }

    /*******************************************************************************************************************
     * encoding_pixelを通常のMaskに変換する
     *
     * @param segments
     * @param mask_map
     */
    void SegmentationClient::encoding_pixel_to_mask(std::vector<Segment> &segments, std::vector<int8_t> &mask_map) {
        int i = 0;
        int index, x, y;
        for (auto &segment:segments) {
            auto *mask = &segment.mask;
            mask->resize(segment.segment.x_masks.size());
            for (int k = 0; k < int(segment.segment.x_masks.size()); ++k) {
                x = segment.segment.x_masks[k];
                y = segment.segment.y_masks[k];
                index = x + y * 640;
                mask_map[index] = i;
                (*mask)[k] = index;
            }
            ++i;
        }
    }

    /*******************************************************************************************************************
     * セグメンテーション結果をマスクサイズでsortする
     *
     * @param segments
     */
    void SegmentationClient::sort_segments(std::vector<Segment> &segments) {
        std::vector<Segment *> sort(segments.size());
        std::vector<std::pair<int, int>> pairs(segments.size());
        for (int i = 0; i < int(segments.size()); ++i) {
            sort[i] = &segments[pairs[i].second];
        }
        for (int i = 0; i < int(segments.size()); ++i) {
            segments[i] = *sort[i];
        }
    }
}

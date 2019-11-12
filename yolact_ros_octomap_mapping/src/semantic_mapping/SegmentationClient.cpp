//
// Created by ubuntu on 2019/10/19.
//

#include <semantic_mapping/SegmentationClient.h>
#include <sensor_msgs/Image.h>
#include <pcl_conversions/pcl_conversions.h>
#include <yolact_ros/SegmentationGoal.h>
#include <yolact_ros/Segments.h>
#include <yolact_ros/Segment.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>


namespace semantic_mapping {

    /*******************************************************************************************************************
     * コンストラクター
     */
    SegmentationClient::SegmentationClient() {
        m_segmentationClient = new actionlib::SimpleActionClient<yolact_ros::SegmentationAction>("/segmentation", true);
        m_image_pub = m_nh.advertise<sensor_msgs::Image>("/segmentation/image_raw", 1, true);
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
        std::random_device rnd;
        std::mt19937 mt(rnd());
        std::uniform_int_distribution<> rand100(0, 255);
        for (auto &color:bounding_box_colors) {
            color = cv::Scalar(rand100(mt), rand100(mt), rand100(mt));
        }
        for (int i = 0; i < int(segments.segments.size()); ++i) {
            cv::Scalar *color = &bounding_box_colors[i];
            const yolact_ros::Segment *segment = &segments.segments[i];
            for (int k = 0; k < int(segment->encoded_pixels.size()); k += 2) {
                int index = segment->encoded_pixels[k];
                int length = segment->encoded_pixels[k + 1];
                for (int n = index; n < index + length; ++n) {
                    int x = n % 640;
                    int y = n / 640;
                    cv::Vec3b *src = &cv_mask.at<cv::Vec3b>(y, x);
                    (*src)[0] = (*color)[0];
                    (*src)[1] = (*color)[1];
                    (*src)[2] = (*color)[2];
                }
            }
        }
        cv::addWeighted(cv_image, 0.5, cv_mask, 0.5, 20, mask_image);
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
            cv::Size text_size = cv::getTextSize(text, cv::FONT_HERSHEY_DUPLEX, 0.6, 1, 0);
            cv::rectangle(mask_image, cv::Point(x1, y1), cv::Point(x1 + text_size.width, y1 - text_size.height - 4), *color, -1);
            cv::putText(mask_image, text, cv::Point(x1, y1 - 3), cv::FONT_HERSHEY_DUPLEX, 0.6, text_color, 1, 16);
        }
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(image.header, "bgr8", mask_image).toImageMsg();
        m_image_pub.publish(msg);
        cv::imwrite("/root/HSR/catkin_ws/mask_image.jpg", mask_image);
    }

    /*******************************************************************************************************************
     * encoding_pixelを通常のMaskに変換する
     *
     * @param segments
     * @param mask_map
     */
    void SegmentationClient::encoding_pixel_to_mask(std::vector<Segment> &segments, std::vector<int8_t> &mask_map) {
        int i = 0;
        for (auto &segment:segments) {
            auto *mask = &segment.mask;
            mask->resize(segment.segment.pixel_size);
            int k = 0;
            for (int m = 0; m < int(segment.segment.encoded_pixels.size()); m += 2) {
                int index = segment.segment.encoded_pixels[m];
                int length = segment.segment.encoded_pixels[m + 1];
                for (int n = index; n < index + length; ++n) {
                    mask_map[n] = i;
                    (*mask)[k] = n;
                    ++k;
                }
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
        for (int i = 0; i < segments.size(); ++i) {
            pairs[i] = std::make_pair(segments[i].segment.pixel_size, i);
        }
        std::sort(pairs.begin(), pairs.end(), std::greater<std::pair<int, int>>());
        for (int i = 0; i < segments.size(); ++i) {
            sort[i] = &segments[pairs[i].second];
        }
        for (int i = 0; i < segments.size(); ++i) {
            segments[i] = *sort[i];
        }
    }
}
//
// Created by ubuntu on 2019/10/16.
//

#include <semantic_mapping/MarkerClient.h>
#include <visualization_msgs/MarkerArray.h>

namespace semantic_mapping {

    /*******************************************************************************************************************
     * コンストラクター
     */
    MarkerClient::MarkerClient() {
        segment_marker_publisher = m_nh.advertise<visualization_msgs::MarkerArray>("segment_markers", 10);
    }

    /*******************************************************************************************************************
     * セグメント情報をまとめてPublishするための呼び出し関数
     * @param cloud
     * @param segments
     */
    void MarkerClient::publish_segment_info(const PointCloud<PointXYZRGB> &cloud, const std::vector<Segment> &segments) {
        publish_line_list(cloud, segments);
        publish_segment_name(cloud, segments);
    }

    /*******************************************************************************************************************
     * セグメントを3次元的に囲むMarkerをPublishする
     * @param cloud
     * @param segments
     */
    void MarkerClient::publish_line_list(const PointCloud<PointXYZRGB> &cloud, const std::vector<Segment> &segments) {
        std::vector<std::vector<geometry_msgs::Point>> line_list;
        to_line_list(cloud, segments, line_list);

        visualization_msgs::MarkerArray marker_array;
        marker_array.markers.resize(100);
        for (int i = 0; i < int(marker_array.markers.size()); ++i) {
            auto *marker = &marker_array.markers[i];
            marker->header.frame_id = "/map";
            marker->header.stamp = ros::Time::now();
            marker->ns = "line_list";
            marker->id = m_line_list_id_base + i;
            marker->action = visualization_msgs::Marker::DELETE;
        }
        for (int i = 0; i < int(line_list.size()); ++i) {
            auto *segment = &segments[i];
            if (segment->is_available) {
                auto *marker = &marker_array.markers[i];
                marker->action = visualization_msgs::Marker::ADD;
                marker->pose.orientation.w = 1.0;
                marker->type = visualization_msgs::Marker::LINE_LIST;
                set_scale(0.01, 0, 0, marker->scale);
                set_color(segment->r / 255.0, segment->g / 255.0, segment->b / 255.0, 1.0, marker->color);
                marker->points = line_list[i];
            }
        }
        segment_marker_publisher.publish(marker_array);
    }

    /*******************************************************************************************************************
     * セグメントの名前表すMarkerをPublishする
     * @param cloud
     * @param segments
     */
    void MarkerClient::publish_segment_name(const PointCloud<PointXYZRGB> &cloud, const std::vector<Segment> &segments) {
        visualization_msgs::MarkerArray marker_array;
        marker_array.markers.resize(100);
        for (int i = 0; i < int(marker_array.markers.size()); ++i) {
            auto *marker = &marker_array.markers[i];
            marker->header.frame_id = "/map";
            marker->header.stamp = ros::Time::now();
            marker->ns = "segment_name";
            marker->id = m_segment_name_id_base + i;
            marker->action = visualization_msgs::Marker::DELETE;
        }
        for (int i = 0; i < int(segments.size()); ++i) {
            auto *segment = &segments[i];
            if (segment->is_available) {
                auto *marker = &marker_array.markers[i];
                marker->action = visualization_msgs::Marker::ADD;
                marker->pose.orientation.w = 1.0;
                marker->type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                marker->text = segments[i].segment.Class;
                set_color(1.0, 1.0, 1.0, 1.0, marker->color);
                set_scale(0.2, 0.2, 0.2, marker->scale);
                set_coordinate((segment->min_x + segment->max_x) / 2.0, (segment->min_y + segment->max_y) / 2.0, segment->max_z + 0.2, marker->pose.position);
            }
        }
        segment_marker_publisher.publish(marker_array);
    }

    /*******************************************************************************************************************
     * 直方体を表す
     *
     * @param cloud
     * @param segments
     * @param line_list
     */
    void MarkerClient::to_line_list(const PointCloud<PointXYZRGB> &cloud, const std::vector<Segment> &segments,
                                    std::vector<std::vector<geometry_msgs::Point>> &line_list) {
        line_list.resize(segments.size());
        std::vector<std::vector<PointXYZRGB>> vectors(segments.size());
        for (int i = 0; i < int(segments.size()); i++) {
            auto *segment = &segments[i];
            auto *line_strip = &line_list[i];
            line_strip->resize(24);
            set_coordinate(segment->min_x, segment->min_y, segment->min_z, (*line_strip)[0]);
            set_coordinate(segment->max_x, segment->min_y, segment->min_z, (*line_strip)[1]);
            set_coordinate(segment->max_x, segment->min_y, segment->min_z, (*line_strip)[2]);
            set_coordinate(segment->max_x, segment->max_y, segment->min_z, (*line_strip)[3]);
            set_coordinate(segment->max_x, segment->max_y, segment->min_z, (*line_strip)[4]);
            set_coordinate(segment->min_x, segment->max_y, segment->min_z, (*line_strip)[5]);
            set_coordinate(segment->min_x, segment->max_y, segment->min_z, (*line_strip)[6]);
            set_coordinate(segment->min_x, segment->min_y, segment->min_z, (*line_strip)[7]);

            set_coordinate(segment->min_x, segment->min_y, segment->min_z, (*line_strip)[8]);
            set_coordinate(segment->min_x, segment->min_y, segment->max_z, (*line_strip)[9]);
            set_coordinate(segment->max_x, segment->min_y, segment->min_z, (*line_strip)[10]);
            set_coordinate(segment->max_x, segment->min_y, segment->max_z, (*line_strip)[11]);
            set_coordinate(segment->max_x, segment->max_y, segment->min_z, (*line_strip)[12]);
            set_coordinate(segment->max_x, segment->max_y, segment->max_z, (*line_strip)[13]);
            set_coordinate(segment->min_x, segment->max_y, segment->min_z, (*line_strip)[14]);
            set_coordinate(segment->min_x, segment->max_y, segment->max_z, (*line_strip)[15]);

            set_coordinate(segment->min_x, segment->min_y, segment->max_z, (*line_strip)[16]);
            set_coordinate(segment->max_x, segment->min_y, segment->max_z, (*line_strip)[17]);
            set_coordinate(segment->max_x, segment->min_y, segment->max_z, (*line_strip)[18]);
            set_coordinate(segment->max_x, segment->max_y, segment->max_z, (*line_strip)[19]);
            set_coordinate(segment->max_x, segment->max_y, segment->max_z, (*line_strip)[20]);
            set_coordinate(segment->min_x, segment->max_y, segment->max_z, (*line_strip)[21]);
            set_coordinate(segment->min_x, segment->max_y, segment->max_z, (*line_strip)[22]);
            set_coordinate(segment->min_x, segment->min_y, segment->max_z, (*line_strip)[23]);
        }
    }

    /*******************************************************************************************************************
     * Set coordinate's xyz
     *
     * @param x
     * @param y
     * @param z
     * @param point
     */
    void MarkerClient::set_coordinate(double x, double y, double z, geometry_msgs::Point &point) {
        point.x = x;
        point.y = y;
        point.z = z;
    }

    /*******************************************************************************************************************
     * Set rgb
     *
     * @param r
     * @param g
     * @param b
     * @param a
     * @param color
     */
    void MarkerClient::set_color(double r, double g, double b, double a, std_msgs::ColorRGBA &color) {
        color.r = r;
        color.g = g;
        color.b = b;
        color.a = a;
    }

    /*******************************************************************************************************************
     * Set scale's xyz
     * @param x
     * @param y
     * @param z
     * @param vector
     */
    void MarkerClient::set_scale(double x, double y, double z, geometry_msgs::Vector3 &vector) {
        vector.x = x;
        vector.y = y;
        vector.z = z;
    }

}
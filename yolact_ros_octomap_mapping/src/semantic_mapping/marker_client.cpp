//
// Created by ubuntu on 2019/10/16.
//

#include <semantic_mapping/marker_client.h>
#include <visualization_msgs/MarkerArray.h>

namespace semantic_mapping {

    /*******************************************************************************************************************
     * コンストラクター
     */
    MarkerClient::MarkerClient(ros::NodeHandle nh) :
            m_frame("/map"),
            m_bounding_box_ns("bounding_box"),
            m_name_ns("name"),
            m_bounding_box_id_base(1000),
            m_name_id_base(2000),
            m_before_marker_size(0) {
        nh.param("marker/frame", m_frame, m_frame);
        nh.param("marker/bounding_box_ns", m_bounding_box_ns, m_bounding_box_ns);
        nh.param("marker/name_ns", m_name_ns, m_name_ns);
        nh.param("marker/bounding_box_id", m_bounding_box_id_base, m_bounding_box_id_base);
        nh.param("marker/name_id", m_name_id_base, m_name_id_base);

        m_marker_pub = m_nh.advertise<visualization_msgs::MarkerArray>("segment_markers", 10);
    }

    /*******************************************************************************************************************
     * セグメント情報をまとめてPublishするための呼び出し関数
     * @param cloud
     * @param segments
     */
    void MarkerClient::publish_segment_info(const PointCloud<PointXYZRGB> &cloud, const std::vector<Segment> &segments) {
        m_marker_size = int(segments.size());
        std_msgs::Header header;
        publish_line_list(cloud, segments);
        publish_segment_name(cloud, segments);
        m_before_marker_size = m_marker_size;
    }

    /*******************************************************************************************************************
     * それぞれのセグメントを3次元空間上で囲むMarkerをPublishする
     * @param cloud
     * @param segments
     */
    void MarkerClient::publish_line_list(const PointCloud<PointXYZRGB> &cloud, const std::vector<Segment> &segments) {
        //************************************************************************************************************//
        // それぞれのマーカーをDELETEで初期化
        //************************************************************************************************************//
        visualization_msgs::MarkerArray marker_array;
        int max_size = std::max(m_marker_size, m_before_marker_size);
        marker_array.markers.resize(max_size);
        ros::Time time_stamp = ros::Time::now();
        for (int i = 0; i < max_size; ++i) {
            auto &marker = marker_array.markers[i];
            marker.header.frame_id = m_frame;
            marker.header.stamp = time_stamp;
            marker.ns = m_bounding_box_ns;
            marker.id = m_bounding_box_id_base + i;
            marker.action = visualization_msgs::Marker::DELETE;
        }

        //************************************************************************************************************//
        // SegmentごとにBounding Boxを生成
        //************************************************************************************************************//
        std::vector<std::vector<geometry_msgs::Point>> line_list(m_marker_size);
        generate_bounding_box(cloud, segments, line_list);

        //************************************************************************************************************//
        // 追加するマーカーのパラメータを設定
        //************************************************************************************************************//
        for (int i = 0; i < m_marker_size; ++i) {
            auto &segment = segments[i];
            if (segment.is_available) {
                auto &marker = marker_array.markers[i];
                marker.action = visualization_msgs::Marker::ADD;
                marker.type = visualization_msgs::Marker::LINE_LIST;
                marker.pose.orientation.w = 1.0;
                set_scale(marker.scale, 0.01, 0, 0);
                set_color(marker.color, segment.r / 255.0, segment.g / 255.0, segment.b / 255.0, 1.0);
                marker.points = line_list[i];
            }
        }
        //************************************************************************************************************//
        // Publish
        //************************************************************************************************************//
        m_marker_pub.publish(marker_array);
    }

    /*******************************************************************************************************************
     * セグメントの名前表すMarkerをPublishする
     * @param cloud
     * @param segments
     */
    void MarkerClient::publish_segment_name(const PointCloud<PointXYZRGB> &cloud, const std::vector<Segment> &segments) {
        //************************************************************************************************************//
        // それぞれのマーカーをDELETEで初期化
        //************************************************************************************************************//
        visualization_msgs::MarkerArray marker_array;
        int max_size = std::max(m_marker_size, m_before_marker_size);
        marker_array.markers.resize(max_size);
        ros::Time time_stamp = ros::Time::now();
        for (int i = 0; i < max_size; ++i) {
            auto &marker = marker_array.markers[i];
            marker.header.frame_id = m_frame;
            marker.header.stamp = time_stamp;
            marker.ns = m_name_ns;
            marker.id = m_name_id_base + i;
            marker.action = visualization_msgs::Marker::DELETE;
        }
        //************************************************************************************************************//
        // 追加するマーカーのパラメータを設定
        //************************************************************************************************************//
        for (int i = 0; i < m_marker_size; ++i) {
            auto &segment = segments[i];
            if (segment.is_available) {
                auto &marker = marker_array.markers[i];
                marker.action = visualization_msgs::Marker::ADD;
                marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                marker.text = segments[i].segment.Class;
                marker.pose.orientation.w = 1.0;
                set_color(marker.color, 1.0, 1.0, 1.0, 1.0);
                set_scale(marker.scale, 0.2, 0.2, 0.2);
                set_coordinate(marker.pose.position, (segment.min_x + segment.max_x) / 2.0, (segment.min_y + segment.max_y) / 2.0, segment.max_z + 0.2);
            }
        }
        //************************************************************************************************************//
        // Publish
        //************************************************************************************************************//
        m_marker_pub.publish(marker_array);
    }

    /*******************************************************************************************************************
     * 立体を表現する線群を生成
     *
     * @param cloud
     * @param segments
     * @param line_list
     */
    void MarkerClient::generate_bounding_box(const PointCloud<PointXYZRGB> &cloud, const std::vector<Segment> &segments,
                                             std::vector<std::vector<geometry_msgs::Point>> &line_list) {
        std::vector<std::vector<PointXYZRGB>> vectors(m_marker_size);
        for (int i = 0; i < m_marker_size; i++) {
            auto &segment = segments[i];
            auto &line_strip = line_list[i];

            line_strip.resize(24);

            // 底面 (水平向きの辺)
            set_coordinate(line_strip[0], segment.min_x, segment.min_y, segment.min_z);
            set_coordinate(line_strip[1], segment.max_x, segment.min_y, segment.min_z);
            set_coordinate(line_strip[2], segment.max_x, segment.min_y, segment.min_z);
            set_coordinate(line_strip[3], segment.max_x, segment.max_y, segment.min_z);
            set_coordinate(line_strip[4], segment.max_x, segment.max_y, segment.min_z);
            set_coordinate(line_strip[5], segment.min_x, segment.max_y, segment.min_z);
            set_coordinate(line_strip[6], segment.min_x, segment.max_y, segment.min_z);
            set_coordinate(line_strip[7], segment.min_x, segment.min_y, segment.min_z);

            // 側面 (垂直向きの辺)
            set_coordinate(line_strip[8], segment.min_x, segment.min_y, segment.min_z);
            set_coordinate(line_strip[9], segment.min_x, segment.min_y, segment.max_z);
            set_coordinate(line_strip[10], segment.max_x, segment.min_y, segment.min_z);
            set_coordinate(line_strip[11], segment.max_x, segment.min_y, segment.max_z);
            set_coordinate(line_strip[12], segment.max_x, segment.max_y, segment.min_z);
            set_coordinate(line_strip[13], segment.max_x, segment.max_y, segment.max_z);
            set_coordinate(line_strip[14], segment.min_x, segment.max_y, segment.min_z);
            set_coordinate(line_strip[15], segment.min_x, segment.max_y, segment.max_z);

            // 上面 (水平向きの辺)
            set_coordinate(line_strip[16], segment.min_x, segment.min_y, segment.max_z);
            set_coordinate(line_strip[17], segment.max_x, segment.min_y, segment.max_z);
            set_coordinate(line_strip[18], segment.max_x, segment.min_y, segment.max_z);
            set_coordinate(line_strip[19], segment.max_x, segment.max_y, segment.max_z);
            set_coordinate(line_strip[20], segment.max_x, segment.max_y, segment.max_z);
            set_coordinate(line_strip[21], segment.min_x, segment.max_y, segment.max_z);
            set_coordinate(line_strip[22], segment.min_x, segment.max_y, segment.max_z);
            set_coordinate(line_strip[23], segment.min_x, segment.min_y, segment.max_z);
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
    void MarkerClient::set_coordinate(geometry_msgs::Point &point, double x, double y, double z) {
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
    void MarkerClient::set_color(std_msgs::ColorRGBA &color, double r, double g, double b, double a) {
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
    void MarkerClient::set_scale(geometry_msgs::Vector3 &vector, double x, double y, double z) {
        vector.x = x;
        vector.y = y;
        vector.z = z;
    }

}
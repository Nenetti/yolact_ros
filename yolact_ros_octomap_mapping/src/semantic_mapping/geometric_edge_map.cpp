//
// Created by ubuntu on 2019/10/16.
//

#include <semantic_mapping/geometric_edge_map.h>
#include <sensor_msgs/Image.h>
#include <semantic_mapping/filter.h>
#include <semantic_mapping/segmentation_client.h>
#include <omp.h>
#include <cv_bridge/cv_bridge.h>

namespace semantic_mapping {

    /*******************************************************************************************************************
     * コンストラクター
     */
    GeometricEdgeMap::GeometricEdgeMap() : m_ground_filter(0) {
        geometric_publisher = m_nh.advertise<sensor_msgs::Image>("geometric_map", 1, true);
        init_color(100);
    }


    /*******************************************************************************************************************
     * メインの処理
     *
     * @param cloud
     * @param segments
     * @param ground
     * @param ceiling
     * @param nonground_nonseg
     * @param nonground_seg
     */
    void GeometricEdgeMap::toSegmentation(PointCloud<PointXYZRGB> &cloud, std::vector<Segment> &segments,
                                          PointCloud<PointXYZRGB> &ground, PointCloud<PointXYZRGB> &ceiling,
                                          PointCloud<PointXYZRGB> &nonground_nonseg, PointCloud<PointXYZRGBL> &nonground_seg) {
        //************************************************************************************************************//
        // Init
        //************************************************************************************************************//
        std::vector<std::vector<int>> seg_ids(cloud.size());
        std::vector<bool> is_ground(cloud.size(), false);
        std::vector<bool> is_ceiling(cloud.size(), false);
        std::vector<bool> is_infinite(cloud.size(), false);
        std::vector<bool> is_depth_edge(cloud.size(), false);
        std::vector<bool> is_mask_edge(cloud.size(), false);
        std::vector<bool> is_exclude(cloud.size(), false);
        std::vector<bool> edge_map(cloud.size(), false);
        std::vector<int8_t> mask_map(cloud.size(), -1);
        std::vector<int8_t> segment_map(cloud.size(), -1);
        std::vector<int> cluster_ids(cloud.size(), -1);
        std::vector<Cluster> clusters;

        SegmentationClient::encoding_pixel_to_mask(segments, mask_map);

        //************************************************************************************************************//
        // Filtering
        //************************************************************************************************************//

        int ground_point_size = Filter::ground_filter(cloud, is_ground, m_ground_filter);
        int ceilign_point_size = Filter::ceiling_filter(cloud, is_ceiling, m_ceiling_filter);
        Filter::infinite_filter(cloud, is_infinite);
        Filter::combine_bool_filter(is_infinite, is_ground, is_exclude);
        Filter::combine_bool_filter(is_exclude, is_ceiling, is_exclude);
        Filter::depth_edge_filter(cloud, is_exclude, is_depth_edge, 0.05f);
        Filter::exclude_noise_from_depth_edge_filter(cloud, is_depth_edge, 50);
        Filter::mask_edge_filter(cloud, mask_map, is_mask_edge);
        Filter::combine_bool_filter(is_depth_edge, is_mask_edge, edge_map);
        Filter::combine_bool_filter(is_exclude, edge_map, edge_map);

        //************************************************************************************************************//
        // Clustering
        //************************************************************************************************************//
        clustering_from_edge(cloud, edge_map, cluster_ids, clusters, 0.05f);
        detect_segmentation_cluster(cluster_ids, clusters, segments);
        Filter::cluster_filter(segments, clusters, segment_map);

        //************************************************************************************************************//
        // After Process
        //************************************************************************************************************//
        calc_segment_range(cloud, is_exclude, segments);
        set_color(segments);
        set_result(cloud, is_ground, is_ceiling, is_infinite, segment_map, segments, ground_point_size, ceilign_point_size, ground, ceiling, nonground_nonseg,
                   nonground_seg);

        //************************************************************************************************************//
        // Publish
        //************************************************************************************************************//
        m_marker_client.publish_segment_info(cloud, segments);
        publish_geometric_image(cloud, segments, is_exclude);
    }


    /*******************************************************************************************************************
     * Edgeからクラスタリングを行う
     *
     * @param cloud
     * @param edge_map
     * @param cluster_ids
     * @param clusters
     * @param threshold
     */
    void GeometricEdgeMap::clustering_from_edge(const PointCloud<PointXYZRGB> &cloud, const std::vector<bool> &edge_map,
                                                std::vector<int> &cluster_ids, std::vector<Cluster> &clusters, double threshold) {
        int width = cloud.width;
        int height = cloud.height;
        int now_cluster_id = 0;

        std::vector<std::pair<double, int>> pairs(cloud.size());
        int i = 0;
        for (auto &p:cloud) {
            pairs[i] = std::make_pair(std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z), i);
            i++;
        }
        std::sort(pairs.begin(), pairs.end());

        cluster_ids = std::vector<int>(width * height, -1);
        clusters.reserve(width * height);
        std::vector<int> queue(width * height);
        std::vector<int> cluster(width * height);
        int queue_begin = 0, queue_end = 0;
        int cluster_begin = 0, cluster_end = 0;
        double dx, dy, dz;
        for (auto &pair:pairs) {
            int indexA = pair.second;
            if (cluster_ids[indexA] != -1 || edge_map[indexA]) {
                continue;
            }
            cluster_ids[indexA] = now_cluster_id;
            queue[queue_end] = indexA;
            queue_end++;
            double ave_d = 0;
            while (queue_begin < queue_end) {
                int indexB = queue[queue_begin];
                int cx = indexB % width;
                int cy = indexB / width;
                auto *pointB = &cloud[cx + cy * width];
                double various_threshold = threshold * (1 + std::sqrt(pointB->x * pointB->x + pointB->y * pointB->y + pointB->z * pointB->z) * 0.25);
                ave_d += std::sqrt(pointB->x * pointB->x + pointB->y * pointB->y + pointB->z * pointB->z);
                for (int tx = std::max(0, cx - 1); tx <= std::min(cx + 1, width - 1); ++tx) {
                    for (int ty = std::max(0, cy - 1); ty <= std::min(cy + 1, height - 1); ++ty) {
                        int indexC = tx + ty * width;
                        if (cluster_ids[indexC] == -1 && !edge_map[indexC]) {
                            auto *pointC = &cloud[tx + ty * width];
                            dx = std::abs(pointB->x - pointC->x);
                            dy = std::abs(pointB->y - pointC->y);
                            dz = std::abs(pointB->z - pointC->z);
                            if (dx < various_threshold && dy < various_threshold && dz < various_threshold) {
                                cluster_ids[indexC] = now_cluster_id;
                                queue[queue_end] = indexC;
                                ++queue_end;
                            }
                        }
                    }
                }
                cluster[cluster_end] = indexB;
                ++cluster_end;
                ++queue_begin;
            }
            std::vector<int> copy(cluster_end - cluster_begin);
            std::copy(cluster.begin(), cluster.begin() + cluster_end, copy.begin());
            Cluster c(copy, cloud[copy[0]].y);
            c.depth = ave_d / copy.size();
            clusters.emplace_back(c);
            ++now_cluster_id;
            queue_begin = 0;
            queue_end = 0;
            cluster_begin = 0;
            cluster_end = 0;
        }
        clusters.shrink_to_fit();
        ROS_INFO("Clusters: %d, %zu", now_cluster_id, clusters.size());
    }

    /*******************************************************************************************************************
     * セグメンテーションに一致するクラスターを決定する
     *
     * @param cluster_ids
     * @param clusters
     * @param segments
     */
    void GeometricEdgeMap::detect_segmentation_cluster(const std::vector<int> &cluster_ids, std::vector<Cluster> &clusters, std::vector<Segment> &segments) {
        std::vector<double> occupied;
        int id, index, size;
        for (auto &segment:segments) {
            occupied = std::vector<double>(clusters.size(), 0);
            size = segment.mask.size();
            for (auto &pixel:segment.mask) {
                id = cluster_ids[pixel];
                if (id >= 0) {
                    ++occupied[id];
                }
            }
            index = std::max_element(occupied.begin(), occupied.end()) - occupied.begin();
            segment.clusters.emplace_back(clusters[index]);
            segment.cluster_occupied.emplace_back(occupied[index]);
            for (int i = 0; i < index; ++i) {
                if (occupied[i] / size > 0.1) {
                    segment.clusters.emplace_back(clusters[i]);
                    segment.cluster_occupied.emplace_back(occupied[i]);
                }
            }
        }
    }

    /*******************************************************************************************************************
     * 地面点群、通常点群、セグメンテーション点群のそれぞれに入力点群を分類する
     *
     * @param cloud
     * @param is_ground
     * @param is_infinite
     * @param segment_map
     * @param segments
     * @param ground_point_size
     * @param ceiling_point_size
     * @param ground
     * @param ceiling
     * @param nonground_nonseg
     * @param nonground_seg
     */
    void GeometricEdgeMap::set_result(const PointCloud<PointXYZRGB> &cloud,
                                      const std::vector<bool> &is_ground, const std::vector<bool> &is_ceiling, const std::vector<bool> &is_infinite,
                                      const std::vector<int8_t> &segment_map, const std::vector<Segment> &segments,
                                      int ground_point_size, int ceiling_point_size,
                                      PointCloud<PointXYZRGB> &ground, PointCloud<PointXYZRGB> &ceiling,
                                      PointCloud<PointXYZRGB> &nonground_nonseg, PointCloud<PointXYZRGBL> &nonground_seg) {
        int width = cloud.width;
        int height = cloud.height;
        int size = width * height;
        int ground_index = 0;
        int ceiling_index = 0;
        int nonseg_index = 0;
        int seg_index = 0;
        ground.resize(ground_point_size);
        ceiling.resize(ceiling_point_size);
        nonground_nonseg.resize(size - (ground_point_size + ceiling_point_size));
        nonground_seg.resize(size - (ground_point_size + ceiling_point_size));
        for (int i = 0; i < int(cloud.size()); ++i) {
            if (is_infinite[i]) {
                continue;
            }
            auto *point = &cloud[i];
            if (is_ground[i]) {
                ground[ground_index] = *point;
                ++ground_index;
                continue;
            }
            if (is_ceiling[i]) {
                ceiling[ceiling_index] = *point;
                ++ceiling_index;
                continue;
            }
            int id = segment_map[i];
            if (id != -1) {
                auto &segment = segments[id];
                PointXYZRGBL p = PointXYZRGBL(segment.r, segment.g, segment.b, id);
                p.x = point->x;
                p.y = point->y;
                p.z = point->z;
                nonground_seg[seg_index] = p;
                ++seg_index;
            } else {
                nonground_nonseg[nonseg_index] = *point;
                ++nonseg_index;
            }
        }
        nonground_nonseg.resize(nonseg_index);
        nonground_seg.resize(seg_index);
    }


    /*******************************************************************************************************************
     * 各セグメントの座標範囲を求める
     *
     * @param cloud
     * @param is_exclude
     * @param segments
     */
    void GeometricEdgeMap::calc_segment_range(const PointCloud<PointXYZRGB> &cloud, const std::vector<bool> &is_exclude, std::vector<Segment> &segments) {
        for (auto &segment : segments) {
            double min_x = DBL_MAX, max_x = -DBL_MAX, min_y = DBL_MAX, max_y = -DBL_MAX, min_z = DBL_MAX, max_z = -DBL_MAX;
            double ave_x = 0, ave_y = 0, ave_z = 0;
            int count = 0;
            for (auto &cluster:segment.clusters) {
                for (auto &index : cluster.indices) {
                    if (!is_exclude[index]) {
                        auto *point = &cloud[index];
                        if (point->x < min_x) {
                            min_x = point->x;
                        }
                        if (point->x > max_x) {
                            max_x = point->x;
                        }
                        if (point->y < min_y) {
                            min_y = point->y;
                        }
                        if (point->y > max_y) {
                            max_y = point->y;
                        }
                        if (point->z < min_z) {
                            min_z = point->z;
                        }
                        if (point->z > max_z) {
                            max_z = point->z;
                        }
                        ave_x += point->x;
                        ave_y += point->y;
                        ave_z += point->z;
                        ++count;
                    }
                }
            }
            if (count != 0) {
                segment.set_range_coordinate(min_x, max_x, min_y, max_y, min_z, max_z);
                segment.set_average_coordinate(ave_x / count, ave_y / count, ave_z / count);
                segment.is_available = true;
            } else {
                segment.set_range_coordinate(0, 0, 0, 0, 0, 0);
                segment.set_average_coordinate(0, 0, 0);
                segment.is_available = false;
            }
        }
    }

    /*******************************************************************************************************************
     *
     * 結果を画像データに変換してPublishする
     * 主に、デバッグ用
     *
     * @param cloud
     * @param segments
     * @param is_exclude
     */
    void GeometricEdgeMap::publish_geometric_image(const PointCloud<PointXYZRGB> &cloud, const std::vector<Segment> &segments,
                                                   const std::vector<bool> &is_exclude) {
        cv::Mat image(cloud.height, cloud.width, CV_8UC3, cv::Scalar(255, 255, 255));
        for (auto &segment:segments) {
            for (auto &cluster:segment.clusters) {
                for (auto &index:cluster.indices) {
                    int x = int(index % cloud.width);
                    int y = int(index / cloud.width);
                    auto *src = &image.at<cv::Vec3b>(y, x);
                    (*src)[0] = segment.b;
                    (*src)[1] = segment.g;
                    (*src)[2] = segment.r;
                }
            }
        }
        for (int i = 0; i < int(cloud.size()); ++i) {
            int x = int(i % cloud.width);
            int y = int(i / cloud.width);
            auto *src = &image.at<cv::Vec3b>(y, x);
            if (is_exclude[i]) {
                (*src)[0] = 0;
                (*src)[1] = 0;
                (*src)[2] = 0;
            }
        }
        sensor_msgs::ImagePtr ros_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
        geometric_publisher.publish(ros_image);
    }


    /*******************************************************************************************************************
     * 使用カラーを事前に定義する
     *
     * @param size
     */
    void GeometricEdgeMap::init_color(int size) {
        m_colors.resize(size);
        std::random_device rnd;
        std::mt19937 mt(rnd());
        std::uniform_int_distribution<> rand100(0, 255);
        for (auto &color:m_colors) {
            int r, g, b;
            while (true) {
                r = rand100(mt);
                g = rand100(mt);
                b = rand100(mt);
                if ((r > 150 || g > 150 || b > 150) && r + g + b > 300) {
                    break;
                }
            }
            color.r = r;
            color.g = g;
            color.b = b;
        }
    }

    /*******************************************************************************************************************
     * 各segmentに色を割り当てる
     *
     * @param segments
     */
    void GeometricEdgeMap::set_color(std::vector<Segment> &segments) {
        for (int i = 0; i < int(segments.size()); ++i) {
            auto *segment = &segments[i];
            auto *color = &m_colors[i];
            segment->set_rgb(color->r, color->g, color->b);
        }
    }

    /*******************************************************************************************************************
     * 地面と天井それぞれを判定する高さの値を設定
     *
     * @param ground_filter
     */
    void GeometricEdgeMap::set_filter(double ground_filter, double ceiling_filter) {
        m_ground_filter = ground_filter;
        m_ceiling_filter = ceiling_filter;
    }
}
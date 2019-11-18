//
// Created by ubuntu on 2019/10/16.
//

#include <semantic_mapping/semantic_mapping.h>

namespace semantic_mapping {


    using pcl::PointCloud;
    using pcl::PointXYZRGB;
    using pcl::PointXYZRGBL;
    using semantic_mapping::Filter;
    using semantic_mapping::Segment;
    using semantic_mapping::Cluster;

    /*******************************************************************************************************************
     * コンストラクター
     */
    SemanticMapping::SemanticMapping() : m_ground_filter(DBL_MIN), m_ceiling_filter(DBL_MAX) {
        init_color(100);
    }


    /*******************************************************************************************************************
     * メインの処理
     *
     * @param cloud
     * @param segments
     */
    void SemanticMapping::to_segmentation(PointCloud<PointXYZRGB> &cloud, std::vector<Segment> &segments, std::vector<Cluster> &clusters) {
        //************************************************************************************************************//
        // Init
        //************************************************************************************************************//
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

        to_mask_map(segments, mask_map);

        //************************************************************************************************************//
        // Filtering (Edge detection)
        //************************************************************************************************************//
        Filter::ground_filter(cloud, is_ground, m_ground_filter);
        Filter::ceiling_filter(cloud, is_ceiling, m_ceiling_filter);
        Filter::infinite_filter(cloud, is_infinite);
        Filter::combine_bool_filter(is_infinite, is_ground, is_exclude);
        Filter::combine_bool_filter(is_exclude, is_ceiling, is_exclude);
        Filter::depth_edge_filter(cloud, is_exclude, is_depth_edge, 0.05f);
        Filter::exclude_noise_from_depth_edge_filter(cloud, is_depth_edge, 50);
        Filter::mask_edge_filter(cloud, mask_map, is_mask_edge);
        Filter::combine_bool_filter(is_depth_edge, is_mask_edge, edge_map);
        Filter::combine_bool_filter(is_exclude, edge_map, edge_map);

        //************************************************************************************************************//
        // Clustering from edge
        //************************************************************************************************************//
        clustering_from_edge(cloud, edge_map, cluster_ids, clusters, 0.05f);
        detect_segmentation_cluster(cluster_ids, clusters, segments);

        //************************************************************************************************************//
        // After Process
        //************************************************************************************************************//
        calc_segment_range(cloud, is_exclude, segments);
        set_color(segments);
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
    void SemanticMapping::clustering_from_edge(const PointCloud<PointXYZRGB> &cloud, const std::vector<bool> &edge_map,
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
                auto &pointB = cloud[cx + cy * width];
                double various_threshold = threshold * (1 + std::sqrt(pointB.x * pointB.x + pointB.y * pointB.y + pointB.z * pointB.z) * 0.25);
                ave_d += std::sqrt(pointB.x * pointB.x + pointB.y * pointB.y + pointB.z * pointB.z);
                for (int tx = std::max(0, cx - 1); tx <= std::min(cx + 1, width - 1); ++tx) {
                    for (int ty = std::max(0, cy - 1); ty <= std::min(cy + 1, height - 1); ++ty) {
                        int indexC = tx + ty * width;
                        if (cluster_ids[indexC] == -1 && !edge_map[indexC]) {
                            auto &pointC = cloud[tx + ty * width];
                            dx = std::abs(pointB.x - pointC.x);
                            dy = std::abs(pointB.y - pointC.y);
                            dz = std::abs(pointB.z - pointC.z);
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
            auto &nearest_point = cloud[copy[0]];
            double depth = std::sqrt(nearest_point.x * nearest_point.x + nearest_point.y * nearest_point.y + nearest_point.z * nearest_point.z);
            Cluster c(copy, depth);
            c.depth = ave_d / copy.size();
            clusters.emplace_back(c);
            ++now_cluster_id;
            queue_begin = 0;
            queue_end = 0;
            cluster_begin = 0;
            cluster_end = 0;
        }
        clusters.shrink_to_fit();
    }

    /*******************************************************************************************************************
     * セグメンテーションに一致するクラスターを決定する
     *
     * @param cluster_ids
     * @param clusters
     * @param segments
     */
    void SemanticMapping::detect_segmentation_cluster(const std::vector<int> &cluster_ids, std::vector<Cluster> &clusters, std::vector<Segment> &segments) {
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
            if (occupied[index] == 0) {
                segment.is_available = false;
                continue;
            }
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
     * 各セグメントの座標範囲を求める
     *
     * @param cloud
     * @param is_exclude
     * @param segments
     */
    void SemanticMapping::calc_segment_range(const PointCloud<PointXYZRGB> &cloud, const std::vector<bool> &is_exclude, std::vector<Segment> &segments) {
        for (auto &segment : segments) {
            if (!segment.is_available) {
                continue;
            }
            double min_x = DBL_MAX, max_x = -DBL_MAX, min_y = DBL_MAX, max_y = -DBL_MAX, min_z = DBL_MAX, max_z = -DBL_MAX;
            double ave_x = 0, ave_y = 0, ave_z = 0;
            int count = 0;
            for (auto &cluster:segment.clusters) {
                for (auto &index : cluster.indices) {
                    if (!is_exclude[index]) {
                        auto &point = cloud[index];
                        if (point.x < min_x) {
                            min_x = point.x;
                        }
                        if (point.x > max_x) {
                            max_x = point.x;
                        }
                        if (point.y < min_y) {
                            min_y = point.y;
                        }
                        if (point.y > max_y) {
                            max_y = point.y;
                        }
                        if (point.z < min_z) {
                            min_z = point.z;
                        }
                        if (point.z > max_z) {
                            max_z = point.z;
                        }
                        ave_x += point.x;
                        ave_y += point.y;
                        ave_z += point.z;
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
     * 使用カラーを事前に定義する
     *
     * @param size
     */
    void SemanticMapping::init_color(int size) {
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
    void SemanticMapping::set_color(std::vector<Segment> &segments) {
        for (int i = 0; i < int(segments.size()); ++i) {
            auto &segment = segments[i];
            auto &color = m_colors[i];
            segment.set_rgb(color.r, color.g, color.b);
        }
    }

    void SemanticMapping::to_mask_map(std::vector<Segment> &segments, std::vector<int8_t> &mask_map) {
        int i = 0;
        for (auto &segment:segments) {
            for (int k = 0; k < int(segment.segment.x_masks.size()); ++k) {
                int x = segment.segment.x_masks[k];
                int y = segment.segment.y_masks[k];
                mask_map[x + y * 640] = i;
            }
            ++i;
        }
    }

    /*******************************************************************************************************************
     * 地面と天井それぞれを判定する高さの値を設定
     *
     * @param ground_filter
     */
    void SemanticMapping::set_filter(double ground_filter, double ceiling_filter) {
        m_ground_filter = ground_filter;
        m_ceiling_filter = ceiling_filter;
    }
}
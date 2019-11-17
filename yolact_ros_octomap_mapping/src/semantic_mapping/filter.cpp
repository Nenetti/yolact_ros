//
// Created by ubuntu on 2019/10/16.
//

#include <semantic_mapping/filter.h>

namespace semantic_mapping {

    /*******************************************************************************************************************
     * セグメント情報から各ピクセルにセグメント情報を付与する
     */
    void Filter::cluster_filter(const std::vector<Segment> &segments, const std::vector<Cluster> &clusters, std::vector<int8_t> &segment_map) {
        int i = 0;
        for (auto &segment:segments) {
            for (auto &cluster:segment.clusters) {
                for (auto index:cluster.indices) {
                    segment_map[index] = i;
                }
            }
            ++i;
        }
    }

    /*******************************************************************************************************************
     * マスク情報を元にエッジを生成する
     *
     * @param cloud
     * @param mask_map
     * @param is_mask_edge
     */
    void Filter::mask_edge_filter(const PointCloud<PointXYZRGB> &cloud, const std::vector<int8_t> &mask_map, std::vector<bool> &is_mask_edge) {
        int width = cloud.width;
        int height = cloud.height;
        int indexA, indexB;
        for (int x = 0; x < width; ++x) {
            for (int y = 0; y < height; ++y) {
                indexA = x + width * y;
                if (!is_mask_edge[indexA]) {
                    for (int fx = std::max(0, x - 1); fx <= std::min(x + 1, width - 1); ++fx) {
                        for (int fy = std::max(0, y - 1); fy <= std::min(y + 1, height - 1); ++fy) {
                            indexB = fx + width * fy;
                            if (mask_map[indexA] != mask_map[indexB]) {
                                is_mask_edge[indexA] = true;
                                is_mask_edge[indexB] = true;
                                goto BREAK_FILTER;
                            }
                        }
                    }
                    BREAK_FILTER:;
                }
            }
        }
    }

    /*******************************************************************************************************************
     * 深度情報を元にエッジを生成する
     *
     * @param cloud
     * @param is_exclude
     * @param is_depth_edge
     * @param threshold
     */
    void Filter::depth_edge_filter(const PointCloud<PointXYZRGB> &cloud, const std::vector<bool> &is_exclude,
                                   std::vector<bool> &is_depth_edge, double threshold) {
        int width = cloud.width;
        int height = cloud.height;
        int indexA, indexB;
        double dx, dy, dz, various_threshold;
        for (int x = 0; x < width; ++x) {
            for (int y = 0; y < height; ++y) {
                indexA = x + width * y;
                if (is_exclude[indexA]) {
                    is_depth_edge[indexA] = true;
                    continue;
                }
                if (!is_depth_edge[indexA]) {
                    auto pointA = &cloud[indexA];
                    dx = pointA->x;
                    dy = pointA->y;
                    dz = pointA->z;
                    various_threshold = threshold * (1 + std::sqrt(pointA->x * pointA->x + pointA->y * pointA->y + pointA->z * pointA->z) * 0.25);
                    for (int fx = std::max(0, x - 1); fx <= std::min(x + 1, width - 1); ++fx) {
                        for (int fy = std::max(0, y - 1); fy <= std::min(y + 1, height - 1); ++fy) {
                            indexB = fx + width * fy;
                            auto pointB = &cloud[indexB];
                            if (std::abs(dx - pointB->x) > various_threshold || std::abs(dy - pointB->y) > various_threshold ||
                                std::abs(dz - pointB->z) > various_threshold) {
                                is_depth_edge[indexA] = true;
                                is_depth_edge[indexB] = true;
                                goto BREAK_FILTER;
                            }
                        }
                    }
                    BREAK_FILTER:;
                }
            }
        }
    }

    /*******************************************************************************************************************
     * 深度情報のエッジからノイズとなるようなエッジを消去する
     *
     * @param cloud
     * @param is_depth_edge
     * @param threshold_size
     */
    void Filter::exclude_noise_from_depth_edge_filter(const PointCloud<PointXYZRGB> &cloud, std::vector<bool> &is_depth_edge, int threshold_size) {
        int width = cloud.width;
        int height = cloud.height;
        int cx, cy;
        int indexA, indexB;
        std::vector<bool> is_opened(cloud.size());
        std::vector<int> queue(cloud.size());
        int queue_begin = 0, queue_end = 0;
        for (int i = 0; i < int(cloud.size()); ++i) {
            if (is_opened[i] || !is_depth_edge[i]) {
                continue;
            }
            is_opened[i] = true;
            queue[queue_end] = i;
            ++queue_end;
            while (queue_begin < queue_end) {
                indexA = queue[queue_begin];
                cx = indexA % width;
                cy = indexA / width;
                for (int tx = std::max(0, cx - 1); tx <= std::min(cx + 1, width - 1); ++tx) {
                    for (int ty = std::max(0, cy - 1); ty <= std::min(cy + 1, height - 1); ++ty) {
                        indexB = tx + ty * width;
                        if (!is_opened[indexB] && is_depth_edge[indexB]) {
                            is_opened[indexB] = true;
                            queue[queue_end] = indexB;
                            ++queue_end;
                        }
                    }
                }
                ++queue_begin;
            }
            if (queue_end < threshold_size) {
                for (int k = 0; k < queue_end; ++k) {
                    is_depth_edge[queue[k]] = false;
                }
            }
            queue_begin = 0;
            queue_end = 0;
        }
    }

    /*******************************************************************************************************************
     * 入力点群の内、地面かどうかを返す
     *
     * @param cloud
     * @param is_ground
     * @param ground_filter
     * @return
     */
    int Filter::ground_filter(const PointCloud<PointXYZRGB> &cloud, std::vector<bool> &is_ground, double ground_filter) {
        int ground_point_size = 0;
        int i = 0;
        for (auto &point:cloud) {
            if (point.z < ground_filter) {
                is_ground[i] = true;
                ++ground_point_size;
            }
            ++i;
        }
        return ground_point_size;
    }

    /*******************************************************************************************************************
     * 入力点群の内、地面かどうかを返す
     *
     * @param cloud
     * @param is_ground
     * @param ground_filter
     * @return
     */
    int Filter::ceiling_filter(const PointCloud<PointXYZRGB> &cloud, std::vector<bool> &is_ceiling, double ceiling_filter) {
        int ceiling_point_size = 0;
        int i = 0;
        for (auto &point:cloud) {
            if (point.z > ceiling_filter) {
                is_ceiling[i] = true;
                ++ceiling_point_size;
            }
            ++i;
        }
        return ceiling_point_size;
    }

    /*******************************************************************************************************************
     * 入力点群の内、XYZの値がInfiniteかどうかを返す
     * @param cloud
     * @param is_infinite
     */
    void Filter::infinite_filter(const PointCloud<PointXYZRGB> &cloud, std::vector<bool> &is_infinite) {
        int i = 0;
        for (auto &point:cloud) {
            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
                is_infinite[i] = true;
            }
            ++i;
        }
    }

    /*******************************************************************************************************************
     * 入力Aと入力Bのor演算を返す
     * @param inputA
     * @param inputB
     * @param output
     */
    void Filter::combine_bool_filter(const std::vector<bool> &inputA, const std::vector<bool> &inputB, std::vector<bool> &output) {
//#pragma omp parallel for default(none), shared(inputA, inputB, output), num_threads(4)
        for (int i = 0; i < int(output.size()); ++i) {
            if (inputA[i] || inputB[i]) {
                output[i] = true;
            }
        }
    }

}
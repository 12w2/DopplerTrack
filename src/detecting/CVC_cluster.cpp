// This file includes code from:
// https://github.com/wangx1996/Lidar-Segementation
// Licensed under the MIT License.
// © Original author: wangx1996

#include "detecting/CVC_cluster.h"

std::unordered_map<int, float> max_heights;            // Store max height of each cluster
std::unordered_map<int, float> ground_contact_ratios;  // Store ground contact ratio of each cluster

// Sort clusters by value in descending order
bool compare_cluster(std::pair<int,int> a,std::pair<int,int> b){
    return a.second > b.second;
}

// Compute polar angle (theta in radians)
float Polar_angle_cal(float x, float y){
    if (x == 0 && y == 0) return 0;
    return static_cast<float>(atan2(y, x) + M_PI / 2);
}

// Compute APR (Azimuth-Polar-Range) representation for each point
void CVC::calculateAPR(const pcl::PointCloud<pcl::PointXYZI>& cloud_IN, std::vector<PointAPR>& vapr, float& min_range_new) {
    for (int i = 0; i < cloud_IN.points.size(); ++i) {
        PointAPR par;
        par.polar_angle = Polar_angle_cal(cloud_IN.points[i].x, cloud_IN.points[i].y);
        par.range = std::sqrt(cloud_IN.points[i].x * cloud_IN.points[i].x + cloud_IN.points[i].y * cloud_IN.points[i].y);
        par.azimuth = static_cast<float>(atan2(cloud_IN.points[i].z, par.range));
        par.z = cloud_IN.points[i].z;
        par.velocity = cloud_IN.points[i].intensity;

        if (par.range < min_range_) min_range_ = par.range;
        if (par.range > max_range_) max_range_ = par.range;

        vapr.push_back(par);
    }

    min_range_new = min_range_;
    length_ = static_cast<int>((max_range_ - min_range_) / deltaR_) + 1;
    std::cout << "min_range: " << min_range_ << std::endl;
    width_ = static_cast<int>(std::round(360 / deltaP_));
}

// Build voxel hash table from APR representation
void CVC::build_hash_table(const std::vector<PointAPR>& vapr, std::unordered_map<int, Voxel>& map_out) {
    for (int i = 0; i < vapr.size(); ++i) {
        int polar_index = static_cast<int>(vapr[i].polar_angle * 180 / M_PI / deltaP_);
        int range_index = static_cast<int>((vapr[i].range - min_range_) / deltaR_);
        int voxel_index = polar_index * length_ + range_index;

        auto it = map_out.find(voxel_index);
        if (it != map_out.end()) {
            it->second.index.push_back(i);
            it->second.avg_velocity_sum += vapr[i].velocity;
            it->second.avg_velocity = it->second.avg_velocity_sum / it->second.index.size();
        } else {
            Voxel vox;
            vox.haspoint = true;
            vox.index.push_back(i);
            vox.avg_velocity_sum = vapr[i].velocity;
            vox.avg_velocity = vapr[i].velocity;
            map_out.insert({voxel_index, vox});
        }
    }
}

// Find neighboring voxel indices (cyclic boundary on polar axis)
void CVC::find_neighbors(int polar, int range, int azimuth, std::vector<int>& neighborindex) {
    for (int y = range - 1; y <= range + 1; y++) {
        if (y < 0 || y >= length_) continue;

        for (int x = polar - 1; x <= polar + 1; x++) {
            int px = x;
            if (x < 0) px = width_ - 1;
            if (x >= width_) px = 0;

            neighborindex.push_back(px * length_ + y);
        }
    }
}

// Find neighbors with dynamic range extension for secondary clustering
void CVC::find_neighbors_second(int polar, int range, int azimuth, std::vector<int>& neighborindex, const int& current_cluster_index) {
    int dynamic_range = range + 3;

    for (int y = range - 1; y <= dynamic_range; y++) {
        if (y < 0 || y >= length_) continue;
        if (y * deltaR_ + min_range_ > cluster_info_map[current_cluster_index].min_mutex_range) continue;

        for (int x = polar - 1; x <= polar + 1; x++) {
            int px = x;
            if (x < 0) px = width_ - 1;
            if (x >= width_) px = 0;
            neighborindex.push_back(px * length_ + y);
        }
    }
}

// Group points by cluster labels and return valid clusters
std::vector<pcl::PointIndices> CVC::most_frequent_value(std::vector<int> values) {
    std::unordered_map<int, std::vector<int>> cluster_map;
    for (int i = 0; i < values.size(); i++) {
        if (values[i] == -1) continue;
        cluster_map[values[i]].push_back(i);
    }

    std::vector<pcl::PointIndices> result;
    for (const auto& pair : cluster_map) {
        if (pair.second.size() > 4) {
            pcl::PointIndices indices;
            indices.indices = pair.second;
            result.push_back(indices);
        }
    }
    return result;
}

// Perform voxel-based clustering
std::vector<int> CVC::cluster(std::unordered_map<int, Voxel>& map_in, const std::vector<PointAPR>& vapr, std::unordered_map<int, int>& voxel_cluster_map, const float& GROUND_HEIGHT) {
    int current_cluster = 0;
    std::queue<int> bfs_queue;
    int min_points_in_voxel = 3;

    for (auto& voxel_entry : map_in) {
        int voxel_idx = voxel_entry.first;
        if (voxel_cluster_map.count(voxel_idx)) continue;
        if (voxel_entry.second.index.size() < min_points_in_voxel) {
            voxel_cluster_map[voxel_idx] = -1;
            continue;
        }

        current_cluster++;
        bfs_queue.push(voxel_idx);

        while (!bfs_queue.empty()) {
            int current_voxel_idx = bfs_queue.front();
            bfs_queue.pop();

            if (voxel_cluster_map.count(current_voxel_idx)) continue;
            voxel_cluster_map[current_voxel_idx] = current_cluster;

            int polar_index = current_voxel_idx / length_;
            int range_index = current_voxel_idx % length_;
            std::vector<int> neighborid;
            find_neighbors(polar_index, range_index, 0, neighborid);

            for (int neighbor_idx : neighborid) {
                auto it_voxel = map_in.find(neighbor_idx);
                if (voxel_cluster_map.count(neighbor_idx) || it_voxel == map_in.end()) continue;
                if (it_voxel->second.index.size() < min_points_in_voxel) continue;
                if (std::abs(it_voxel->second.avg_velocity - map_in[current_voxel_idx].avg_velocity) > 0.5) continue;

                bfs_queue.push(neighbor_idx);
            }
        }
    }

    std::vector<int> point_clusters(vapr.size(), -1);
    std::unordered_map<int, std::vector<float>> cluster_heights;

    for (int i = 0; i < vapr.size(); ++i) {
        int polar_index = static_cast<int>(vapr[i].polar_angle * 180 / M_PI / deltaP_);
        int range_index = static_cast<int>((vapr[i].range - min_range_) / deltaR_);
        int voxel_idx = polar_index * length_ + range_index;

        if (voxel_cluster_map.count(voxel_idx)) {
            int cluster_idx = voxel_cluster_map[voxel_idx];
            point_clusters[i] = cluster_idx;

            float height = vapr[i].z - GROUND_HEIGHT;
            float polar_angle = vapr[i].polar_angle;
            float range = vapr[i].range;
            cluster_heights[cluster_idx].push_back(height);

            cluster_info_map[cluster_idx].min_polar = std::min(cluster_info_map[cluster_idx].min_polar, polar_angle);
            cluster_info_map[cluster_idx].max_polar = std::max(cluster_info_map[cluster_idx].max_polar, polar_angle);
            cluster_info_map[cluster_idx].max_height = std::max(cluster_info_map[cluster_idx].max_height, height);
            cluster_info_map[cluster_idx].min_range = std::min(cluster_info_map[cluster_idx].min_range, range);
        }
    }

    for (auto& entry : cluster_heights) {
        int cluster_idx = entry.first;
        if (entry.second.size() > 5) {
            std::sort(entry.second.begin(), entry.second.end());
            int count_below_threshold = std::lower_bound(entry.second.begin(), entry.second.end(), 
                (entry.second.back() - GROUND_HEIGHT) * 0.6 + GROUND_HEIGHT) - entry.second.begin();
            cluster_info_map[cluster_idx].ground_contact_ratio = static_cast<float>(count_below_threshold) / entry.second.size();
        } else {
            cluster_info_map[cluster_idx].ground_contact_ratio = 0;
        }
    }

    cluster_info_map[-1] = ClusterInfo();

    std::vector<int> filtered_clusters;
    for (const auto& entry : cluster_info_map) {
        if (entry.second.ground_contact_ratio > 0.3) {
            filtered_clusters.push_back(entry.first);
        }
    }

    std::sort(filtered_clusters.begin(), filtered_clusters.end(), [&](int a, int b) {
        return cluster_info_map[a].min_polar < cluster_info_map[b].min_polar;
    });

    for (size_t i = 0; i < filtered_clusters.size(); ++i) {
        int current_cluster = filtered_clusters[i];
        for (size_t j = i + 1; j < filtered_clusters.size(); ++j) {
            int next_cluster = filtered_clusters[j];
            if (cluster_info_map[current_cluster].max_polar >= cluster_info_map[next_cluster].min_polar) {
                cluster_info_map[current_cluster].mutex_clusters.push_back(next_cluster);
                cluster_info_map[next_cluster].mutex_clusters.push_back(current_cluster);

                if (cluster_info_map[next_cluster].min_range > cluster_info_map[current_cluster].min_range) {
                    cluster_info_map[current_cluster].min_mutex_range = std::min(cluster_info_map[next_cluster].min_range, cluster_info_map[current_cluster].min_mutex_range);
                }

                if (cluster_info_map[current_cluster].min_range > cluster_info_map[next_cluster].min_range) {
                    cluster_info_map[next_cluster].min_mutex_range = std::min(cluster_info_map[current_cluster].min_range, cluster_info_map[next_cluster].min_mutex_range);
                }
            } else {
                break;
            }
        }
    }

    return point_clusters;
}

// Merge two clusters by replacing idx1 with idx2
void CVC::mergeClusters(std::vector<int>& cluster_indices, int idx1, int idx2) {
    for (int& idx : cluster_indices) {
        if (idx == idx1) idx = idx2;
    }
}

// Merge mutual exclusion lists
void mergeMutexClusters(std::vector<int>& mutex_list1, const std::vector<int>& mutex_list2, int current_cluster_idx) {
    for (int idx : mutex_list2) {
        if (idx != current_cluster_idx && std::find(mutex_list1.begin(), mutex_list1.end(), idx) == mutex_list1.end()) {
            mutex_list1.push_back(idx);
        }
    }
}

// Secondary clustering to merge neighboring or similar clusters
std::vector<int> CVC::secondaryClusterMerge(std::unordered_map<int, Voxel>& map_in, const std::vector<int>& point_clusters, const std::vector<PointAPR>& vapr, std::unordered_map<int, int>& voxel_cluster_map) {
    std::vector<int> point_clusters_new = point_clusters;
    int current_secondary_cluster = 0;
    std::unordered_map<int, int> secondary_voxel_cluster_map;
    std::queue<int> bfs_queue;

    std::vector<int> voxel_indices;
    voxel_indices.reserve(map_in.size());
    for (const auto& entry : map_in) {
        voxel_indices.push_back(entry.first);
    }

    std::sort(voxel_indices.begin(), voxel_indices.end(), [this](int a, int b) {
        return (a % length_) < (b % length_);
    });

    for (int voxel_idx : voxel_indices) {
        if (voxel_cluster_map[voxel_idx] == -1) continue;
        if (secondary_voxel_cluster_map.count(voxel_idx)) continue;

        current_secondary_cluster++;
        bfs_queue.push(voxel_idx);

        while (!bfs_queue.empty()) {
            int current_voxel_idx = bfs_queue.front();
            bfs_queue.pop();

            if (secondary_voxel_cluster_map.count(current_voxel_idx)) continue;

            int current_cluster_index = voxel_cluster_map.at(current_voxel_idx);
            secondary_voxel_cluster_map[current_voxel_idx] = current_cluster_index;

            int polar_index = current_voxel_idx / length_;
            int range_index = current_voxel_idx % length_;
            std::vector<int> neighborid;
            find_neighbors_second(polar_index, range_index, 0, neighborid, current_cluster_index);

            for (int neighbor_idx : neighborid) {
                if (secondary_voxel_cluster_map.count(neighbor_idx)) continue;
                if (map_in.count(neighbor_idx) == 0) continue;

                if (std::abs(map_in[neighbor_idx].avg_velocity - map_in[current_voxel_idx].avg_velocity) > 0.5) continue;

                if (voxel_cluster_map[neighbor_idx] == -1) {
                    voxel_cluster_map[neighbor_idx] = voxel_cluster_map[current_voxel_idx];
                }

                mergeMutexClusters(cluster_info_map[current_cluster_index].mutex_clusters, cluster_info_map[voxel_cluster_map[neighbor_idx]].mutex_clusters, current_cluster_index);
                mergeMutexClusters(cluster_info_map[voxel_cluster_map[neighbor_idx]].mutex_clusters, cluster_info_map[current_cluster_index].mutex_clusters, voxel_cluster_map[neighbor_idx]);

                voxel_cluster_map[neighbor_idx] = current_cluster_index;
                bfs_queue.push(neighbor_idx);
            }
        }
    }

    for (int i = 0; i < point_clusters_new.size(); ++i) {
        int polar_index = static_cast<int>(vapr[i].polar_angle * 180 / M_PI / deltaP_);
        int range_index = static_cast<int>((vapr[i].range - min_range_) / deltaR_);
        int voxel_idx = polar_index * length_ + range_index;

        if (voxel_cluster_map.count(voxel_idx)) {
            point_clusters_new[i] = voxel_cluster_map[voxel_idx];
        }
    }

    return point_clusters_new;
}
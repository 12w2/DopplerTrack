#pragma once

#define PCL_NO_PRECOMPILE

#include "fmcw_lidar.hpp"
#include <pcl/filters/extract_indices.h>
#include <unordered_map>
#include <numeric>
#include <cmath>
#include <deque>

// === Grid structure ===
struct GridInfo {
    int point_count = 0;
    float avg_height = 0.0;
    std::vector<int> indices;
};

struct HistoricalGrid {
    std::deque<int> point_counts;
    int average_point_count = 0;
};

// === Update historical grid data ===
void update_historical_grid(
    const std::unordered_map<std::string, GridInfo>& current_grid_map,
    size_t max_frame_history,
    std::unordered_map<std::string, HistoricalGrid>& historical_grid_map
) {
    for (const auto& item : current_grid_map) {
        const std::string& key = item.first;
        const GridInfo& current_grid = item.second;

        HistoricalGrid& historical_grid = historical_grid_map[key];
        historical_grid.point_counts.push_back(current_grid.point_count);

        if (historical_grid.point_counts.size() > max_frame_history) {
            historical_grid.point_counts.pop_front();
        }

        int total_points = std::accumulate(
            historical_grid.point_counts.begin(),
            historical_grid.point_counts.end(),
            0
        );

        historical_grid.average_point_count = static_cast<int>(
            std::floor(static_cast<float>(total_points) / historical_grid.point_counts.size())
        );
    }
}

// === Detect motion regions ===
AQ_PCloudTPtr detect_motion_objects(
    AQ_PCloudTPtr& current_cloud,
    float grid_size,
    int frame_counter,
    std::unordered_map<std::string, HistoricalGrid>& historical_grid_map,
    float ground_height
) {
    AQ_PCloudTPtr motion_cloud(new AQ_PCloudT);
    std::unordered_map<std::string, GridInfo> current_grid_map;

    for (size_t i = 0; i < current_cloud->points.size(); ++i) {
        const auto& point = current_cloud->points[i];
        int x = static_cast<int>(std::floor(point.x / grid_size));
        int y = static_cast<int>(std::floor(point.y / grid_size));
        std::string key = std::to_string(x) + "_" + std::to_string(y);

        current_grid_map[key].point_count += 1;
        current_grid_map[key].avg_height += point.z;
        current_grid_map[key].indices.push_back(i);
    }

    for (auto& grid : current_grid_map) {
        grid.second.avg_height /= grid.second.point_count;
    }

    std::vector<int> diff_indices;
    for (size_t i = 0; i < current_cloud->points.size(); ++i) {
        const auto& point = current_cloud->points[i];
        int x = static_cast<int>(std::floor(point.x / grid_size));
        int y = static_cast<int>(std::floor(point.y / grid_size));
        std::string key = std::to_string(x) + "_" + std::to_string(y);

        int diff = current_grid_map[key].point_count - historical_grid_map[key].average_point_count;

        if (
            diff > 5 &&
            static_cast<double>(diff) / current_grid_map[key].point_count > 0.3 &&
            current_grid_map[key].avg_height > ground_height + 0.5
        ) {
            diff_indices.push_back(i);
        }
    }

    pcl::ExtractIndices<AQ_PointT> extract;
    extract.setInputCloud(current_cloud);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    inliers->indices = std::move(diff_indices);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*motion_cloud);

    update_historical_grid(current_grid_map, 30, historical_grid_map);

    AQ_PCloudTPtr zero_cloud(new AQ_PCloudT);
    if (frame_counter < 3) return zero_cloud;

    return motion_cloud;
}

// === Main function: background subtraction ===
AQ_PCloudT backgroundSubtraction(
    const AQ_PCloudT& static_cloud,
    const AQ_PCloudT& dynamic_cloud,
    int frame_counter,
    std::unordered_map<std::string, HistoricalGrid>& historical_grid_map,
    float ground_height = 0.0f,                 
    float grid_resolution = 0.5f                 
) {

    pcl::PointIndices::Ptr ground_indices(new pcl::PointIndices);
    for (size_t i = 0; i < static_cloud.points.size(); ++i) {
        float z = static_cloud.points[i].z;
        if (z < ground_height + 0.3 || z > ground_height + 3.0) {
            ground_indices->indices.push_back(i);
        }
    }

    pcl::ExtractIndices<AQ_PointT> extract;
    extract.setInputCloud(static_cloud.makeShared());
    extract.setIndices(ground_indices);

    AQ_PCloudTPtr non_ground(new AQ_PCloudT);
    AQ_PCloudTPtr ground(new AQ_PCloudT);
    extract.setNegative(true); extract.filter(*non_ground);
    extract.setNegative(false); extract.filter(*ground);

    AQ_PCloudTPtr motion = detect_motion_objects(
        non_ground, grid_resolution, frame_counter, historical_grid_map, ground_height
    );

    AQ_PCloudT combined = dynamic_cloud;
    combined += *motion;

    return combined;
}
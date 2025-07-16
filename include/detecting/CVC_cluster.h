// This file includes code from:
// https://github.com/wangx1996/Lidar-Segementation
// Licensed under the MIT License.
// © Original author: wangx1996

#ifndef CVC_CLUSTER_H
#define CVC_CLUSTER_H

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/impl/passthrough.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>

// C++
#include <iostream>
#include <stdlib.h>
#include <unordered_map>
#include <algorithm>
#include <limits>

template<typename T>
std::string toString(const T& t) {
    std::ostringstream oss;
    oss << t;
    return oss.str();
}

// Point attributes in angular-polar-range space
struct PointAPR {
    float azimuth;
    float polar_angle;
    float range;
    float z;
    float velocity;
};

// Voxel structure
struct Voxel {
    bool haspoint = false;
    int cluster = -1;
    float avg_velocity_sum = 0;
    float avg_velocity = 0;
    std::vector<int> index;
};

// Cluster information
struct ClusterInfo {
    float ground_contact_ratio;         // Contact ratio with ground
    float min_polar;                    // Minimum polar angle
    float max_polar;                    // Maximum polar angle
    float max_height;                   // Maximum height
    float min_range;                    // Minimum range
    float min_mutex_range;             // Minimum mutual exclusion range
    std::vector<int> mutex_clusters;    // Indices of mutually exclusive clusters

    ClusterInfo() 
        : ground_contact_ratio(0.0),
          min_polar(std::numeric_limits<float>::max()),
          max_polar(std::numeric_limits<float>::min()),
          max_height(std::numeric_limits<float>::min()),
          min_range(std::numeric_limits<float>::max()),
          min_mutex_range(std::numeric_limits<float>::max()) {}
};

// Curved Voxel Clustering main class
class CVC {
public:
    CVC();
    CVC(std::vector<float>& param) {
        if (param.size() != 3) {
            printf("Param number is not correct!");
            std::abort();
        }
        for (int i = 0; i < param.size(); ++i) {
            deltaA_ = param[0];
            deltaR_ = param[1];
            deltaP_ = param[2];
        }
    }

    ~CVC() {}

    // Compute angular-polar-range representation
    void calculateAPR(const pcl::PointCloud<pcl::PointXYZI>& cloud_IN, std::vector<PointAPR>& vapr, float& min_range);

    // Build hash table from APR points
    void build_hash_table(const std::vector<PointAPR>& vapr, std::unordered_map<int, Voxel> &map_out);

    // Neighbor search in voxel space
    void find_neighbors(int polar, int range, int azimuth, std::vector<int>& neighborindex);

    // Find most frequent cluster indices
    std::vector<pcl::PointIndices> most_frequent_value(std::vector<int> values);

    // Merge cluster labels
    void mergeClusters(std::vector<int>& cluster_indices, int idx1, int idx2);

    // Clustering algorithm
    std::vector<int> cluster(std::unordered_map<int, Voxel> &map_in,
                             const std::vector<PointAPR>& vapr,
                             std::unordered_map<int, int>& voxel_cluster_map,
                             const float& GROUND_HEIGHT);

    // Post-processing to correct clusters
    void correctClusters(std::vector<int>& cluster_indices, const std::unordered_map<int, Voxel> &map_in);

    // Secondary merge for cluster refinement
    std::vector<int> secondaryClusterMerge(std::unordered_map<int, Voxel> &map_in,
                                           const std::vector<int>& point_clusters,
                                           const std::vector<PointAPR>& vapr,
                                           std::unordered_map<int, int>& voxel_cluster_map);

    // Neighbor search for merging
    void find_neighbors_second(int polar, int range, int azimuth,
                                std::vector<int>& neighborindex,
                                const int &current_cluster_index);

    // Entry point
    void process();

    std::unordered_map<int, ClusterInfo> cluster_info_map;

private:
    float deltaA_ = 2;
    float deltaR_ = 0.35;
    float deltaP_ = 0.6;
    float min_range_ = std::numeric_limits<float>::max();
    float max_range_ = std::numeric_limits<float>::min();
    float min_azimuth_ = -24.8 * M_PI / 180;
    float max_azimuth_ = 2 * M_PI / 180;
    int length_ = 0;
    int width_ = 0;
    int height_ = 0;
};

#endif
#define PCL_NO_PRECOMPILE
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <pcl_conversions/pcl_conversions.h>
#include <dopplertrack/DetectedObjectArray.h>
#include <dopplertrack/DetectedObject.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <unordered_map>
#include <numeric>
#include <deque>
#include <map>
#include <vector>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <iostream>

#include "fmcw_lidar.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include "detecting/CVC_cluster.h"
#include <pcl/filters/extract_indices.h>
#include "detecting/backgroundSubtraction.h"


constexpr float PI = 3.1415926f;


struct BoundingBox {
    cv::Point2f center;
    cv::Size2f size;
    float angle;
    float min_z, max_z;
    float avg_velocity;
    int class_id;
    int points_number;
    float best_angle;
    float speed;
    float direction_det;
    float conf;
};


class PointCloudDetector {
public:
    PointCloudDetector(ros::NodeHandle& nh);
    void run();

private:
    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;
    ros::Publisher merge_objects_pub_;
    int frame_counter_ = 0;

    // Hyperparameters
    float GROUND_HEIGHT_ = 0.0f;
    float DEFAULT_BUCKET_SIZE_ = 0.8f;
    float DELTA_RHO_ = 0.2f;
    float DELTA_THETA_ = 0.4f;
    float IOU_THRESHOLD_ = 0.4f;
    float COVERAGE_THRESHOLD_ = 0.5f;
    bool use_car_flag_ = false;
    bool debug_flag_ = true;

    std::unordered_map<std::string, HistoricalGrid> historical_grid_map_;

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input);
    AQ_PCloudT loadPointCloud(const sensor_msgs::PointCloud2ConstPtr& input);
    void preprocessPointCloud(AQ_PCloudT& cloud, AQ_PCloudT& static_cloud, AQ_PCloudT& dynamic_cloud);
    std::vector<BoundingBox> detecting(const AQ_PCloudT& cloud);
    std::vector<BoundingBox> gettingBoxes(AQ_PCloudTPtr& filtered_dynamic_cloud_ptr, std::vector<pcl::PointIndices>& cluster_indices, std::vector<BoundingBox>& merge_boxes);
    BoundingBox Fitting(std::vector<cv::Point2f>& points_2d);

    std::vector<BoundingBox> NMS(const std::vector<BoundingBox>& boxes, float iou_threshold, float coverage_threshold, bool wait_flag, std::map<float, std::vector<cv::Point2f>>& velocity_to_points_map);
    void publishDetections(const std::vector<BoundingBox>& boxes);
    dopplertrack::DetectedObjectArray msgTransform(std::vector<BoundingBox>& boxes, std_msgs::Header& header);
};


PointCloudDetector::PointCloudDetector(ros::NodeHandle& nh) : nh_(nh) {
    ros::NodeHandle pnh("~");  // 添加private NodeHandle

    pnh.param("ground_height", GROUND_HEIGHT_, GROUND_HEIGHT_);
    pnh.param("default_bucket_size", DEFAULT_BUCKET_SIZE_, DEFAULT_BUCKET_SIZE_);
    pnh.param("delta_rho", DELTA_RHO_, DELTA_RHO_);
    pnh.param("delta_theta", DELTA_THETA_, DELTA_THETA_);
    pnh.param("iou_threshold", IOU_THRESHOLD_, IOU_THRESHOLD_);
    pnh.param("coverage_threshold", COVERAGE_THRESHOLD_, COVERAGE_THRESHOLD_);
    pnh.param("use_car_flag", use_car_flag_, use_car_flag_);
    pnh.param("debug_flag", debug_flag_, debug_flag_);

    cloud_sub_ = nh_.subscribe("/fmcw_pc", 100, &PointCloudDetector::cloudCallback, this);
    merge_objects_pub_ = nh_.advertise<dopplertrack::DetectedObjectArray>("/merge_detected_objects1", 1000);
}


void PointCloudDetector::run() {
    ros::Rate loop_rate(20);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

	// ====================== Main Process ======================
void PointCloudDetector::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input) {
    auto t_start = std::chrono::steady_clock::now();

    AQ_PCloudT raw_cloud = loadPointCloud(input);
    AQ_PCloudT static_cloud, dynamic_cloud;

    preprocessPointCloud(raw_cloud, static_cloud, dynamic_cloud);

    AQ_PCloudT dynamic_all;
    if (use_car_flag_)
        dynamic_all = dynamic_cloud;
    else
        dynamic_all = backgroundSubtraction(static_cloud, dynamic_cloud, frame_counter_, historical_grid_map_, GROUND_HEIGHT_);

    std::vector<BoundingBox> boxes = detecting(dynamic_all);
    publishDetections(boxes);

    auto t_end = std::chrono::steady_clock::now();
    ROS_INFO_STREAM("Frame processed in " << std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count() << " ms");

    frame_counter_++;
}

// ====================== Point Cloud Loading ======================
AQ_PCloudT PointCloudDetector::loadPointCloud(const sensor_msgs::PointCloud2ConstPtr& input) {
    AQ_PCloudT cloud;
    pcl::fromROSMsg(*input, cloud);

    for (auto& point : cloud.points) {
        float Vr = point.velocity;
        float theta = point.phi + PI / 2;
        float phi = point.theta;

        point.reconstruct_velocity = Vr / cos(phi) / sin(PI - theta);
    }

    return cloud;
}

	// ====================== Static/Dynamic Separation ======================
void PointCloudDetector::preprocessPointCloud(AQ_PCloudT& cloud, AQ_PCloudT& static_cloud, AQ_PCloudT& dynamic_cloud) {
    std::unordered_map<int, AQ_PCloudT> buckets;
    std::unordered_map<int, int> radius_count;
    std::unordered_map<int, int> bucket_indices;

    for (const auto& point : cloud.points) {
        int bucket = std::floor(point.reconstruct_velocity / DEFAULT_BUCKET_SIZE_);
        buckets[bucket].points.push_back(point);
        if (point.radius > 20) {
            radius_count[bucket]++;
        }
    }

    std::vector<AQ_PCloudT> clusters;
    for (auto& bucket : buckets) {
        AQ_PCloudT& cluster = bucket.second;
        cluster.width = cluster.points.size();
        cluster.height = 1;
        clusters.push_back(cluster);
        bucket_indices[bucket.first] = clusters.size() - 1;
    }

    int max_index = 0, max_radius_count = 0;
    for (const auto& pair : radius_count) {
        if (pair.second > max_radius_count) {
            max_radius_count = pair.second;
            max_index = bucket_indices[pair.first];
        }
    }

    float avg_velocity = 0.0f;
    if (!clusters[max_index].points.empty()) {
        float sum_v = 0.0f;
        for (const auto& pt : clusters[max_index].points) sum_v += pt.reconstruct_velocity;
        avg_velocity = sum_v / clusters[max_index].points.size();
    }
    
    for (const auto& cluster : clusters) {
        for (const auto& pt : cluster.points) {
            if (std::abs(pt.reconstruct_velocity - avg_velocity) <= 0.8f)
                static_cloud.points.push_back(pt);
            else if (debug_flag_ && pt.z > GROUND_HEIGHT_ - 1 && pt.z < GROUND_HEIGHT_ + 3) {
                dynamic_cloud.points.push_back(pt);
            }
        }
    }
}

	// ====================== Detection Pipeline ======================
std::vector<BoundingBox> PointCloudDetector::detecting(const AQ_PCloudT& cloud) {
    std::vector<BoundingBox> merge_boxes;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    for (const auto& point : cloud.points) {
        pcl::PointXYZI p;
        p.x = point.x; p.y = point.y; p.z = point.z;
        p.intensity = point.velocity;
        cluster_cloud->points.push_back(p);
    }

    std::vector<float> param = {5, DELTA_RHO_, DELTA_THETA_};
    CVC clusterer(param);
    std::vector<PointAPR> capr;
    float min_range;
    clusterer.calculateAPR(*cluster_cloud, capr, min_range);

    std::unordered_map<int, Voxel> hash_table;
    clusterer.build_hash_table(capr, hash_table);

    std::unordered_map<int, int> voxel_cluster_map;
    auto first = clusterer.cluster(hash_table, capr, voxel_cluster_map, GROUND_HEIGHT_);
    auto second = clusterer.secondaryClusterMerge(hash_table, first, capr, voxel_cluster_map);
    auto cluster_indices = clusterer.most_frequent_value(second);

    AQ_PCloudTPtr cloud_ptr = cloud.makeShared();
    return gettingBoxes(cloud_ptr, cluster_indices, merge_boxes);
}

	// ====================== Extract Bounding Boxes ======================
std::vector<BoundingBox> PointCloudDetector::gettingBoxes(AQ_PCloudTPtr& cloud_ptr, std::vector<pcl::PointIndices>& cluster_indices, std::vector<BoundingBox>& merge_boxes) {
    std::vector<BoundingBox> boxes;
    std::map<float, std::vector<cv::Point2f>> velocity_to_points_map;

    for (const auto& indices : cluster_indices) {
        float max_z = -std::numeric_limits<float>::infinity();
        float min_z =  std::numeric_limits<float>::infinity();
        std::vector<cv::Point2f> points_2d;
        float sum_v = 0.0f;

        for (const auto& idx : indices.indices) {
            const AQ_PointT& pt = cloud_ptr->points[idx];
            max_z = std::max(max_z, pt.z);
            min_z = std::min(min_z, pt.z);
            points_2d.emplace_back(pt.x, pt.y);
            sum_v += pt.velocity;
        }

        float avg_v = sum_v / indices.indices.size();
        velocity_to_points_map[avg_v] = points_2d;

        BoundingBox box = Fitting(points_2d);
        float radius = sqrt(box.center.x * box.center.x + box.center.y * box.center.y);

        if ((debug_flag_ && radius > 80 && points_2d.size() < 5) || (radius > 25 && points_2d.size() < 7) || (radius <= 25 && points_2d.size() < 20)) continue;

        boxes.push_back({box.center, box.size, box.angle, min_z, max_z, avg_v, 0, (int)indices.indices.size(), 0, 0});
    }

    return NMS(boxes, IOU_THRESHOLD_, COVERAGE_THRESHOLD_, false, velocity_to_points_map);
}

	// ====================== Fitting ======================
BoundingBox PointCloudDetector::Fitting(std::vector<cv::Point2f>& points_2d) {
    cv::RotatedRect rect = cv::minAreaRect(points_2d);
    return {rect.center, rect.size, rect.angle};
}

// ====================== NMS ======================
std::vector<BoundingBox> PointCloudDetector::NMS(const std::vector<BoundingBox>& boxes, float iou_threshold, float coverage_threshold, bool wait_flag, std::map<float, std::vector<cv::Point2f>>& velocity_to_points_map) {
    std::vector<BoundingBox> output;
    if (boxes.empty()) return output;

    std::vector<int> indices(boxes.size());
    std::iota(indices.begin(), indices.end(), 0);
    std::sort(indices.begin(), indices.end(), [&boxes](int i, int j) {
        return boxes[i].size.area() > boxes[j].size.area();
    });

    while (!indices.empty()) {
        int current_index = indices[0];
        BoundingBox current_box = boxes[current_index];
        output.push_back(current_box);
        indices.erase(indices.begin());

        cv::RotatedRect rect1(current_box.center, current_box.size, current_box.angle);
        for (auto it = indices.begin(); it != indices.end();) {
            cv::RotatedRect rect2(boxes[*it].center, boxes[*it].size, boxes[*it].angle);
            std::vector<cv::Point2f> intersection;
            int flag = cv::rotatedRectangleIntersection(rect1, rect2, intersection);
            float iou = 0.0f, coverage = 0.0f;

            if (flag > 0) {
                float inter_area = cv::contourArea(intersection);
                iou = inter_area / (rect1.size.area() + rect2.size.area() - inter_area);
                coverage = inter_area / std::min(rect1.size.area(), rect2.size.area());
            }

            if (iou > iou_threshold || coverage > coverage_threshold) {
                it = indices.erase(it);
            } else {
                ++it;
            }
        }
    }

    return output;
}

// ====================== Publish Detection Results =====================
void PointCloudDetector::publishDetections(const std::vector<BoundingBox>& boxes) {
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = "fmcw_lidar";
    dopplertrack::DetectedObjectArray msg = msgTransform(const_cast<std::vector<BoundingBox>&>(boxes), header);
    merge_objects_pub_.publish(msg);
}

dopplertrack::DetectedObjectArray PointCloudDetector::msgTransform(std::vector<BoundingBox>& boxes, std_msgs::Header& header) {
    dopplertrack::DetectedObjectArray detect_objects;
    detect_objects.header = header;

    for (const auto& box : boxes) {
        dopplertrack::DetectedObject obj;
        obj.header = header;
        obj.x = box.center.x;
        obj.y = box.center.y;
        obj.z = (box.min_z + box.max_z) / 2;
        obj.width = box.size.width;
        obj.height = box.size.height;
        obj.z_height = box.max_z - box.min_z;
        obj.box_angle = box.angle;
        obj.label = std::to_string(box.class_id);
        obj.velocity_r = box.avg_velocity;
        obj.velocity = box.speed;
        obj.direction = box.direction_det;
        obj.class_id = box.class_id;
        obj.conf = box.conf;
        detect_objects.objects.push_back(obj);
    }

    return detect_objects;
}

// ====================== Main Function ======================
int main(int argc, char** argv) {
    ros::init(argc, argv, "fmcw_detecting1");
    ros::NodeHandle nh;

    PointCloudDetector detector(nh);
    detector.run();

    return 0;
}
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <fstream>
#include <iostream>
#include <vector>
#include <dirent.h>
#include <string>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>

#include "fmcw_lidar.hpp"  // Contains definitions like AQ_PointT, AQ_PCloudT

struct AevaPoint {
    float x, y, z, reflectivity, velocity, stamp;
};

class PointCloudPublisher {
public:
    PointCloudPublisher(ros::NodeHandle& nh,
                        const std::string& topic,
                        const std::string& folder_path,
                        const std::string& type = "aq",
                        int start_frame = 0,
                        int rate_hz = 10)
        : nh_(nh),
          folder_path_(folder_path),
          type_(type),
          start_frame_(start_frame),
          frame_count_(0),
          loop_rate_(rate_hz) {
        pub_ = nh_.advertise<sensor_msgs::PointCloud2>(topic, 10);
        file_list_ = getBinFiles(folder_path_);
    }

    void run() {
        if (file_list_.empty()) {
            ROS_ERROR_STREAM("No .bin files found in: " << folder_path_);
            return;
        }

        ROS_INFO_STREAM("Publishing from: " << folder_path_);
        ROS_INFO_STREAM("Data type: " << type_);
        ros::Duration(1.0).sleep();  // Wait for subscribers to be ready

        while (ros::ok() && frame_count_ < file_list_.size()) {
            if (frame_count_ < start_frame_) {
                ++frame_count_;
                continue;
            }

            auto start = std::chrono::high_resolution_clock::now();

            std::vector<AQ_PointT> points;
            if (type_ == "aeva") {
                points = readAevaBinFile(file_list_[frame_count_]);
            } else if (type_ == "aq") {
                points = readAQBinFile(file_list_[frame_count_]);
            } else {
                ROS_ERROR_STREAM("Unknown type: " << type_ << ". Use 'aeva' or 'aq'.");
                return;
            }

            publish(points, "fmcw_lidar");

            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> duration = end - start;

            ROS_INFO("Frame %d published in %.2f ms", frame_count_, duration.count());

            ++frame_count_;
            ros::spinOnce();
            loop_rate_.sleep();
        }

        ROS_INFO("All frames published.");
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    std::string folder_path_;
    std::string type_;
    int start_frame_;
    int frame_count_;
    ros::Rate loop_rate_;
    std::vector<std::string> file_list_;

    std::vector<AQ_PointT> readAQBinFile(const std::string& filename) {
        std::vector<AQ_PointT> points;
        std::ifstream file(filename, std::ios::binary);
        if (!file) {
            ROS_ERROR_STREAM("Failed to open AQ .bin file: " << filename);
            return points;
        }

        while (file.peek() != EOF) {
            float data[6];
            file.read(reinterpret_cast<char*>(data), sizeof(data));
            if (file.gcount() < sizeof(data)) break;

            AQ_PointT pt;
            pt.x = data[0];
            pt.y = data[1];
            pt.z = data[2];
            pt.intensity = data[3];
            pt.velocity = data[4];
            pt.time = data[5];

            computeSpherical(pt);
            points.push_back(pt);
        }

        return points;
    }

    std::vector<AQ_PointT> readAevaBinFile(const std::string& filename) {
        std::vector<AQ_PointT> points;
        std::ifstream file(filename, std::ios::binary);
        if (!file) {
            ROS_ERROR_STREAM("Failed to open Aeva file: " << filename);
            return points;
        }

        bool isOldFormat = std::stoull(filename.substr(filename.find_last_of('/') + 1).substr(0, filename.find_last_of('.'))) <= 1691936557946849179;

        while (file.peek() != EOF) {
            AevaPoint pt_raw;
            char buffer[29];
            file.read(buffer, isOldFormat ? 25 : 29);
            if (file.gcount() < 25) break;
            std::memcpy(&pt_raw, buffer, 24);

            AQ_PointT pt;
            pt.x = pt_raw.x;
            pt.y = pt_raw.y;
            pt.z = pt_raw.z;
            pt.intensity = pt_raw.reflectivity;
            pt.velocity = pt_raw.velocity;
            pt.time = pt_raw.stamp;

            computeSpherical(pt);
            correctPoint(pt);         // 2. Apply correction to all points

            if (filterPoint(pt)) {    // 3. Keep the point if it passes the filter
                points.push_back(pt);
            }
           
        }

        return points;
    }

    void computeSpherical(AQ_PointT& pt) {
        float r = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
        pt.radius = r;
        pt.theta = std::atan2(pt.y, pt.x);
        pt.phi = std::asin(pt.z / r);
    }

    // Apply correction (all points must be corrected)
    void correctPoint(AQ_PointT& point) {
        const float adjustmentAngle = 1.405 * (3.1415926 / 180.0); // Approx. 1.4 degree correction
        point.phi += adjustmentAngle;

        float r = point.radius;
        point.x = r * std::cos(point.theta) * std::cos(point.phi);
        point.y = r * std::sin(point.theta) * std::cos(point.phi);
        point.z = r * std::sin(point.phi);
    }

    // Determine whether the point should be kept (filter)
    bool filterPoint(const AQ_PointT& point) {
        const float X_MAX = 100.0f;
        const float Y_MIN = -20.0f;
        const float Y_MAX = 20.0f;

        return (point.x > 2 &&
                point.radius > 3 &&
                point.velocity > -70 && point.velocity < 70 &&
                point.theta > -0.7 && point.theta < 0.7 &&
                point.x > 0 && point.x < X_MAX &&
                point.y > Y_MIN && point.y < Y_MAX);
    }


    void publish(const std::vector<AQ_PointT>& points, const std::string& frame_id) {
        AQ_PCloudT cloud;
        cloud.points.assign(points.begin(), points.end());
        cloud.width = cloud.points.size();
        cloud.height = 1;
        cloud.is_dense = true;

        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(cloud, msg);
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = frame_id;

        pub_.publish(msg);
        std::cout << "Published frame with " << cloud.points.size() << " points." << std::endl;
    }

    std::vector<std::string> getBinFiles(const std::string& folder_path) {
        std::vector<std::string> file_names;
        DIR* dir = opendir(folder_path.c_str());
        if (!dir) {
            ROS_ERROR_STREAM("Could not open directory: " << folder_path);
            return file_names;
        }

        struct dirent* ent;
        while ((ent = readdir(dir)) != NULL) {
            std::string fileName = ent->d_name;
            if (fileName.size() > 4 && fileName.substr(fileName.size() - 4) == ".bin") {
                file_names.push_back(folder_path + "/" + fileName);
            }
        }
        closedir(dir);
        std::sort(file_names.begin(), file_names.end());
        return file_names;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "fmcw_pointcloud_publisher");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");  // Used to read private parameters (from launch file)

    std::string type;
    std::string folder_path;
    int start_frame;

    // Read parameters from launch file, use default values to avoid missing parameters
    private_nh.param<std::string>("type", type, "aq");
    private_nh.param<std::string>("path", folder_path, "./data");
    private_nh.param<int>("start_frame", start_frame, 0);

    PointCloudPublisher publisher(nh, "/fmcw_pc", folder_path, type, start_frame);
    publisher.run();

    return 0;
}
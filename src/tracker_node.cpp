#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <dopplertrack/DetectedObjectArray.h>
#include <dopplertrack/DetectedObject.h>
#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>
#include <tf/transform_datatypes.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <map>
#include <vector>
#include <deque>
#include <sstream>
#include <string>
#include "hungarian/hungarian_optimizer.h"
#include "fmcw_lidar.hpp" 
#include <sys/stat.h>         // mkdir
using namespace std;

struct Params {
    double l_mean, l_std, w_mean, w_std;
    double z_mean, z_std;
    double v_max, alpha_v;
    double l_min, l_max, alpha_l_min, alpha_l_max;
};

class ObjectTrackerNode {
public:
    ObjectTrackerNode(ros::NodeHandle& nh)
        : frame_counter_(0), id_counter_(0), searcher_num_(12), searcher_range_(10) {

        sub_ = nh.subscribe("/merge_detected_objects1", 10, &ObjectTrackerNode::callback, this);
        initParameters();

        ros::NodeHandle private_nh("~");
        private_nh.param<std::string>("output_dir", output_dir_, "label");
        std::cout << "[tracker_node] Output directory: " << output_dir_ << std::endl;
    }

    void parseArguments(int argc, char** argv) {
        for (int i = 1; i < argc; i++) {
            std::string arg = argv[i];
            if (arg == "-c" && i + 1 < argc) {
                searcher_num_ = std::stoi(argv[++i]);
                std::cout << "searcher_num set to: " << searcher_num_ << std::endl;
            } else if (arg == "-s" && i + 1 < argc) {
                searcher_range_ = std::stoi(argv[++i]);
                std::cout << "searcher_range set to: " << searcher_range_ << std::endl;
            } else {
                std::cerr << "Warning: Unknown argument '" << arg << "'" << std::endl;
            }
        }
    }

private:
    ros::Subscriber sub_;


    int frame_counter_;
    int id_counter_;
    int searcher_num_;
    int searcher_range_;
    std::string output_dir_;
    std::vector<dopplertrack::DetectedObjectArray> predictions_;
    std::map<std::string, Params> parameters_;

   
    void callback(const dopplertrack::DetectedObjectArray::ConstPtr& msg) {
        frame_counter_++;
        std::cout << "--------------------- 第 " << frame_counter_ << " 帧 -------------------" << std::endl;

        dopplertrack::DetectedObjectArray unmatched_boxes;
        dopplertrack::DetectedObjectArray matched_boxes;
        dopplertrack::DetectedObjectArray output_boxes;

        std::vector<std::vector<float>> association_mat;
        std::vector<std::vector<int>> maxIoUIndices;

        auto assignments = computeAssignments(msg, association_mat, maxIoUIndices);
        matched_boxes = performMatching(msg, assignments, maxIoUIndices, unmatched_boxes);
        handleLifecycleLoss(assignments, maxIoUIndices, matched_boxes);

        dopplertrack::DetectedObjectArray classify_output_boxes = classifyBoxes(matched_boxes);
        classify_output_boxes.header = msg->header;

        dopplertrack::DetectedObjectArray this_boxes;
        this_boxes.objects.insert(this_boxes.objects.end(), classify_output_boxes.objects.begin(), classify_output_boxes.objects.end());
        this_boxes.objects.insert(this_boxes.objects.end(), unmatched_boxes.objects.begin(), unmatched_boxes.objects.end());

        predictions_ = predict(this_boxes, searcher_num_, searcher_range_);

        msg2json(classify_output_boxes, frame_counter_);

        std::cout << "output boxes num: " << classify_output_boxes.objects.size() << std::endl;
    }


    void initParameters() {
        parameters_ = {
            {"pedestrian", {0.8, 0.3, 0.4, 0.2, 1.5, 0.3, 2, 1.5, 0, 0, 0, 0}},
            {"cyclist",    {1.8, 0.5, 0.5, 0.2, 1.5, 0.3, 15, 0.5, 0, 0, 0, 0}},
            {"car",        {0, 0, 1.9, 0.4, 1.5, 0.3, 28, 1.5, 3.0, 5.8, 2.1, 3.6}},
            {"truck",      {0, 0, 2.5, 0.3, 2.8, 0.6, 40, 0.3, 6.5, 15.0, 1.6, 1.0}}
        };
    }

    // ------------------------ Main Functionalities ------------------------

    std::vector<std::pair<size_t, size_t>> computeAssignments(
        const dopplertrack::DetectedObjectArray::ConstPtr& msg,
        std::vector<std::vector<float>>& association_mat,
        std::vector<std::vector<int>>& maxIoUIndices
    ) {
        size_t rows = msg->objects.size();
        size_t cols = predictions_.size();

        association_mat.assign(rows, std::vector<float>(cols, 0.0f));
        maxIoUIndices.assign(rows, std::vector<int>(cols, -1));

        for (size_t i = 0; i < rows; ++i) {
            for (size_t j = 0; j < cols; ++j) {
                double minCost = 5.0;
                int minIndex = -1;
                for (size_t k = 1; k < predictions_[j].objects.size(); ++k) {
                    double cost = CostAll(msg->objects[i], predictions_[j].objects[k]);
                    if (cost < minCost) {
                        minCost = cost;
                        minIndex = k;
                    }
                }
                association_mat[i][j] = minCost;
                maxIoUIndices[i][j] = minIndex;
            }
        }

        return hungarian(association_mat);
    }

    dopplertrack::DetectedObjectArray performMatching(
        const dopplertrack::DetectedObjectArray::ConstPtr& msg,
        const std::vector<std::pair<size_t, size_t>>& assignments,
        const std::vector<std::vector<int>>& maxIoUIndices,
        dopplertrack::DetectedObjectArray& unmatched_boxes
    ) {
        dopplertrack::DetectedObjectArray matched_boxes;

        for (size_t i = 0; i < msg->objects.size(); ++i) {
            dopplertrack::DetectedObject new_box = msg->objects[i];
            bool isMatched = false;

            for (const auto& pair : assignments) {
                if (pair.first == i) {
                    int clusterIndex = pair.second;
                    int maxIoUBoxIndex = maxIoUIndices[i][clusterIndex];
                    if (maxIoUBoxIndex != -1) {
                        dopplertrack::DetectedObject best_match = predictions_[clusterIndex].objects[maxIoUBoxIndex];
                        new_box.life += best_match.life;
                        new_box.directions_history.push_back(best_match.direction);
                        if (new_box.life > 2) {
                            new_box.id = best_match.id ? best_match.id : ++id_counter_;
                            new_box.velocity = best_match.velocity;
                            new_box.direction = best_match.direction;
                            new_box.last_velocity_r = best_match.velocity_r;
                            new_box.loss = best_match.loss;
                            new_box.last_x = best_match.last_x;
                            new_box.last_y = best_match.last_y;
                            new_box.miss_time = 0;
                            matched_boxes.objects.push_back(new_box);
                            isMatched = true;
                        }
                    }
                    break;
                }
            }

            if (!isMatched) {
                unmatched_boxes.objects.push_back(new_box);
            }
        }

        return matched_boxes;
    }

    void handleLifecycleLoss(
        const std::vector<std::pair<size_t, size_t>>& assignments,
        const std::vector<std::vector<int>>& maxIoUIndices,
        dopplertrack::DetectedObjectArray& output_boxes
    ) {
        for (size_t j = 0; j < predictions_.size(); ++j) {
            bool found_match = false;

            for (const auto& pair : assignments) {
                if (pair.second == j && maxIoUIndices[pair.first][j] != -1) {
                    found_match = true;
                    break;
                }
            }

            if (!found_match) {
                dopplertrack::DetectedObject lost_box = predictions_[j].objects[0];
                lost_box.miss_time += 1;

                if (lost_box.life > 3 && lost_box.miss_time < 3) {
                    output_boxes.objects.push_back(lost_box);
                }
            }
        }
    }


        std::vector<dopplertrack::DetectedObjectArray> predict(
            const dopplertrack::DetectedObjectArray& input_boxes,
            const int& searcher_num,
            const int& searcher_range) {

            std::vector<dopplertrack::DetectedObjectArray> search_groups;

            for (const auto& box : input_boxes.objects) {
                double x = box.x;
                double y = box.y;
                float search_angle = 0;
                double speed;

                dopplertrack::DetectedObjectArray search_group;

                dopplertrack::DetectedObject init_box = box;
                speed = init_box.velocity;
                search_angle = init_box.direction;
                init_box.last_x = init_box.x;
                init_box.last_y = init_box.y;
                init_box.x += speed * std::sin(search_angle) * 0.1;
                init_box.y -= speed * std::cos(search_angle) * 0.1;
                init_box.life += 1;
                search_group.objects.push_back(init_box);

                if (box.id == 0) {
                    for (float angle = 5; angle <= 175; angle += 170.0 / (searcher_num - 1)) {
                        if (box.velocity_r > 0) {
                            search_angle = atan2(y, x) + angle * M_PI / 180.0;
                            speed = std::abs(box.velocity_r) / std::cos((angle - 90) * M_PI / 180.0);
                        } else {
                            search_angle = atan2(y, x) - angle * M_PI / 180.0;
                            speed = std::abs(box.velocity_r) / std::cos((-angle + 90) * M_PI / 180.0);
                        }

                        if (std::abs(speed) > 35) continue;

                        dopplertrack::DetectedObject new_box = box;
                        new_box.last_x = new_box.x;
                        new_box.last_y = new_box.y;
                        new_box.x += speed * std::sin(search_angle) * 0.1;
                        new_box.y -= speed * std::cos(search_angle) * 0.1;
                        new_box.velocity = speed;
                        new_box.direction = search_angle;
                        new_box.life += 1;

                        search_group.objects.push_back(new_box);
                    }
                } else {
                    for (float angle_add = -searcher_range; angle_add <= searcher_range;
                         angle_add += 2 * float(searcher_range) / (searcher_num - 1)) {
                        search_angle = box.direction + angle_add * M_PI / 180;
                        double angle = box.velocity_r > 0 ?
                                       search_angle - atan2(y, x) :
                                       atan2(y, x) - search_angle;
                        speed = std::abs(box.velocity_r) / std::cos(angle - M_PI / 2);

                        if (std::abs(speed) > 35) continue;

                        dopplertrack::DetectedObject new_box = box;
                        new_box.last_x = new_box.x;
                        new_box.last_y = new_box.y;
                        new_box.x += speed * std::sin(search_angle) * 0.1;
                        new_box.y -= speed * std::cos(search_angle) * 0.1;
                        new_box.velocity = (speed < 0) ? -speed : speed;
                        new_box.direction = (speed < 0) ? M_PI + search_angle : search_angle;
                        new_box.life += 1;

                        search_group.objects.push_back(new_box);
                    }
                }

                search_groups.push_back(search_group);
            }

            return search_groups;
        }

        // ---------------- Loss ----------------

        double computeIoU(const dopplertrack::DetectedObject& box1, dopplertrack::DetectedObject& box2) {
            cv::Point2f center1(box1.x, box1.y);
            cv::Point2f center2(box2.x, box2.y);
            cv::Size2f size1(box1.width, box1.height);
            cv::Size2f size2(box2.width, box2.height);
            float distance = cv::norm(center1 - center2);

            cv::RotatedRect rect1(center1, size1, box1.box_angle);
            cv::RotatedRect rect2(center2, size2, box2.box_angle);

            std::vector<cv::Point2f> intersection;
            int flag = cv::rotatedRectangleIntersection(rect1, rect2, intersection);
            float intersection_area = (flag <= 0) ? 0 : cv::contourArea(intersection);

            if (flag <= 0 && distance > 3) return 100.0f;

            float union_area = rect1.size.area() + rect2.size.area() - intersection_area;
            std::vector<cv::Point2f> points;
            cv::Point2f rect1_points[4], rect2_points[4];
            rect1.points(rect1_points);
            rect2.points(rect2_points);
            for (int i = 0; i < 4; i++) {
                points.push_back(rect1_points[i]);
                points.push_back(rect2_points[i]);
            }
            cv::RotatedRect minBoundingRect = cv::minAreaRect(points);
            float bounding_box_area = minBoundingRect.size.area();

            double iou = intersection_area / union_area;
            double giou = (bounding_box_area - union_area) / bounding_box_area;
            double giou2 = 1 - iou + giou;

            box2.loss = giou2;
            return giou2;
        }

        double computeDopplerLoss(const dopplertrack::DetectedObject& box1, dopplertrack::DetectedObject& box2) {
            if (std::max(std::abs(box1.velocity_r), std::abs(box2.velocity_r)) == 0) return 0;
            return std::min(std::abs(box1.velocity_r), std::abs(box2.velocity_r)) /
                   std::max(std::abs(box1.velocity_r), std::abs(box2.velocity_r));
        }


        double CostAll(const dopplertrack::DetectedObject& box1, dopplertrack::DetectedObject& box2) {
            double alpha = 1.0, bias = 0.5;
            double iou_loss = computeIoU(box1, box2);
            double doppler_loss = 1 - computeDopplerLoss(box1, box2);

            double all_loss = 100.0;
            if (iou_loss != 1) {
                all_loss = alpha * iou_loss + doppler_loss;
                if (box2.id == 0) all_loss += bias;
            }

            return all_loss;
        }

        // ---------------- classify ----------------

        double normalizedGaussian(double x, double mean, double std) {
            double raw = (1 / (std * std::sqrt(2 * M_PI))) *
                         std::exp(-(x - mean) * (x - mean) / (2 * std * std));
            return raw / (1 / (std * std::sqrt(2 * M_PI)));
        }

        double sigmoid(double x, double cutoff, double alpha) {
            return 1 / (1 + std::exp(-alpha * (x - cutoff)));
        }

        double probabilityGivenParams(double l, double w, double z, double v, const Params& params) {
            double p_l = (params.l_std != 0.0) ?
                         normalizedGaussian(l, params.l_mean, params.l_std) :
                         sigmoid(l, params.l_min, params.alpha_l_min) *
                         (1 - sigmoid(l, params.l_max, params.alpha_l_max));
            double p_w = normalizedGaussian(w, params.w_mean, params.w_std);
            double p_z = normalizedGaussian(z, params.z_mean, params.z_std);
            double p_v = 1.0; // disable velocity filtering

            return p_l * p_w * p_z * p_v;
        }

        void classifyBox(dopplertrack::DetectedObject& box) {
            double max_prob = 0.0;
            std::string best_category = "";
            int class_id = 0;

            for (const auto& pair : parameters_) {
                double prob = probabilityGivenParams(
                    std::max(box.width, box.height),
                    std::min(box.width, box.height),
                    box.z + box.z_height / 2 - (-7.3),
                    box.velocity,
                    pair.second
                );
                if (prob > max_prob) {
                    max_prob = prob;
                    best_category = pair.first;
                }
            }

            box.conf = max_prob;
            if (best_category == "pedestrian") class_id = 1;
            else if (best_category == "cyclist") class_id = 2;
            else if (best_category == "car") class_id = 3;
            else if (best_category == "truck") class_id = 4;
            else class_id = 0;

            box.class_id = class_id;
        }

        dopplertrack::DetectedObjectArray classifyBoxes(const dopplertrack::DetectedObjectArray& boxes) {
            dopplertrack::DetectedObjectArray output = boxes;
            for (auto& box : output.objects) {
                classifyBox(box);
            }
            return output;
        }

        // ---------------- hungarian algorithm ----------------

        std::vector<std::pair<size_t, size_t>> hungarian(std::vector<std::vector<float>> cost_matrix) {
            HungarianOptimizer<float> optimizer;
            std::vector<std::pair<size_t, size_t>> assignments;
            optimizer.costs()->Resize(cost_matrix.size(), cost_matrix[0].size());

            for (size_t i = 0; i < cost_matrix.size(); ++i) {
                for (size_t j = 0; j < cost_matrix[i].size(); ++j) {
                    (*optimizer.costs())(i, j) = cost_matrix[i][j];
                }
            }

            optimizer.Minimize(&assignments);
            return assignments;
        }

        // ---------------- JSON output ----------------

        void msg2json(const dopplertrack::DetectedObjectArray& boxes, const int& frameNum) {
            nlohmann::json j = nlohmann::json::array();
            std::string label;

            for (const auto& box : boxes.objects) {
                if (box.miss_time > 0) continue;

                float velocity_direction = box.direction - M_PI / 2 - 2 * M_PI * std::floor((box.direction - M_PI / 2 + M_PI) / (2 * M_PI));
                float new_z_height = box.z + box.z_height / 2 - (-7.3);
                float new_z = new_z_height / 2 + (-7.3);

                float x = box.width;
                float y = box.height;
                float rotation = box.box_angle / 180 * M_PI;

                if (box.width < box.height) {
                    x = box.height;
                    y = box.width;
                    rotation += 1.57;
                }

                if (box.label_id == 1) label = "Pedestrian";
                else if (box.label_id == 2) label = "Bicycle";
                else if (box.label_id == 3) label = "Car";
                else if (box.label_id == 4) label = "Truck";
                else {
                    if (box.class_id == 1) label = "Pedestrian";
                    else if (box.class_id == 2) label = "Bicycle";
                    else if (box.class_id == 3) label = "Car";
                    else if (box.class_id == 4) label = "Truck";
                    else label = "Unknown";
                }

                nlohmann::json box_json = {
                    {"annotator", "a"},
                    {"obj_id", std::to_string(box.id)},
                    {"obj_type", label},
                    {"psr", {
                        {"position", {{"x", box.x}, {"y", box.y}, {"z", new_z}}},
                        {"rotation", {{"x", 0.0f}, {"y", 0.0f}, {"z", rotation}}},
                        {"scale", {{"x", x}, {"y", y}, {"z", new_z_height}}},
                        {"velocity", {
                            {"direction", velocity_direction},
                            {"speed", box.velocity},
                            {"conf", box.conf}
                        }}
                    }}
                };

                j.push_back(box_json);
            }

            
            std::ostringstream formattedFrame;
            formattedFrame << std::setw(6) << std::setfill('0') << frameNum;
            std::string filename = output_dir_ + "/" + formattedFrame.str() + ".json";

            std::ofstream file(filename);
            if (!file.is_open()) {
                std::cerr << "FAIL: " << filename << std::endl;
                return;
            }

            file << j.dump(4);
            file.close();
        }
};
// ------------------ main ------------------

int main(int argc, char** argv) {
    ros::init(argc, argv, "object_merger_node");
    ros::NodeHandle nh;

    ObjectTrackerNode tracker(nh);
    tracker.parseArguments(argc, argv);

    ros::Rate rate(10.0);
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

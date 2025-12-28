#pragma once
#include <Eigen/Core>
#include <vector>
#include <array>
#include <open3d/geometry/TriangleMesh.h>
#include <chrono>
#include <geometry_msgs/msg/pose_stamped.hpp>

struct RobotConfig
{
    std::string name;
    double tool_radius; // in meters
};

struct SceneConfig
{
    std::string name;
    std::string frame_id;
};

struct AlgorithmConfig
{
    std::string name;

};

struct SurfacePoint {
    Eigen::Vector3d position;
    Eigen::Vector3d normal;
    int triangle_id;
    bool covered;
};

struct RobotWaypoint {
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    double coverage_radius;
    double timestamp;

    RobotWaypoint(){}
    RobotWaypoint(const geometry_msgs::msg::PoseStamped& pose)
    {
        position = Eigen::Vector3d(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
        orientation = Eigen::Quaterniond(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);
    }
};

struct ToolParameters
{
    float param1;
    float param2;
    float param3;

    int size_triple_array_param1;
    float* float_triple_array_param1;

    ToolParameters(): param1(0.0f), param2(0.0f), param3(0.0f), size_triple_array_param1(0), float_triple_array_param1(nullptr) {}

    ~ToolParameters()
    {
        if (float_triple_array_param1 != nullptr)
        {
            delete[] float_triple_array_param1;
        }
    }
};

struct CoverageResult 
{
    std::vector<RobotWaypoint> robot_path;
    double coverage_ratio;
    std::vector<int> point_covered_num; // The number of times that each surface point is continuously visited
    double computation_time;

    // These are sampled dense surface points, for evaluation only. Not from the raw point cloud
    size_t total_points;
    size_t covered_points;

    void reset()
    {
        robot_path.clear();
        coverage_ratio = 0.0;
        point_covered_num.clear();
        computation_time = std::numeric_limits<double>::infinity();
        total_points = 0;
        covered_points = 0;
    }
};

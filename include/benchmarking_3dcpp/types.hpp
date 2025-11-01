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

struct CoverageResult 
{
    std::vector<RobotWaypoint> robot_path;
    double coverage_ratio;
    std::vector<bool> coverage_mask;
    double computation_time;

    // These are sampled dense surface points, for evaluation only. Not from the raw point cloud
    size_t total_points;
    size_t covered_points;
    std::vector<SurfacePoint> surface_points;

    void reset()
    {
        robot_path.clear();
        coverage_ratio = 0.0;
        coverage_mask.clear();
        computation_time = std::numeric_limits<double>::infinity();
        total_points = 0;
        covered_points = 0;
        surface_points.clear();
    }
};


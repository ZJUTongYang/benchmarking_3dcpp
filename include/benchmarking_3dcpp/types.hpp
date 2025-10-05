#pragma once
#include <Eigen/Core>
#include <vector>
#include <array>
#include <open3d/geometry/TriangleMesh.h>

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
};

struct CoverageResult {
    double coverage_ratio;
    std::vector<bool> coverage_mask;
    size_t total_points;
    size_t covered_points;
};
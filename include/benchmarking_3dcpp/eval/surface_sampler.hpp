#pragma once
#include <benchmarking_3dcpp/types.hpp>
#include <open3d/geometry/TriangleMesh.h>
#include <memory>
#include <random>

class SurfaceSampler {
public:
    SurfaceSampler(double point_density = 1000.0); // points per unit area
    
    std::vector<SurfacePoint> sampleUniformly(
        const open3d::geometry::TriangleMesh& mesh);

    std::vector<SurfacePoint> sampleUniformly(
        const open3d::geometry::PointCloud& mesh);
    
    std::vector<SurfacePoint> sampleUniformlyCUDA(
        const open3d::geometry::TriangleMesh& mesh);

private:
    double point_density_;
    std::mt19937 rng_;
    
    std::vector<double> computeTriangleAreas(
        const open3d::geometry::TriangleMesh& mesh) const;
    
    std::vector<SurfacePoint> rejectionSamplingCPU(
        const open3d::geometry::TriangleMesh& mesh,
        const std::vector<double>& triangle_areas,
        double total_area);
        
    Eigen::Vector3d randomPointInTriangle(
        const Eigen::Vector3d& v0,
        const Eigen::Vector3d& v1, 
        const Eigen::Vector3d& v2);
};
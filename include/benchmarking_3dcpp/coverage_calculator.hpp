#pragma once
#include <benchmarking_3dcpp/types.hpp>
#include <benchmarking_3dcpp/surface_sampler.hpp>
#include <open3d/geometry/TriangleMesh.h>
#include <memory>
#include <vector>

class CoverageCalculator {
public:
    CoverageCalculator(bool use_cuda = true);
    ~CoverageCalculator();
    
    CoverageResult calculateCoverage(
        const open3d::geometry::TriangleMesh& mesh,
        const std::vector<RobotWaypoint>& path,
        double max_distance = 0.1,
        double max_angle_rad = M_PI / 4.0);
        
    void setSurfaceSamplingDensity(double density);

private:
    bool use_cuda_;
    std::unique_ptr<SurfaceSampler> sampler_;
    
    std::vector<bool> calculateCoverageCPU(
        const std::vector<SurfacePoint>& surface_points,
        const std::vector<RobotWaypoint>& path,
        double max_distance, double max_angle);
        
    std::vector<bool> calculateCoverageCUDA(
        const std::vector<SurfacePoint>& surface_points,
        const std::vector<RobotWaypoint>& path,
        double max_distance, double max_angle);
};
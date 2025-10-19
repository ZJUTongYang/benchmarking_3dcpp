#pragma once
#include <cuda_runtime.h>
#include <Eigen/Core>

struct CudaSurfacePoint {
    float x, y, z;
    float nx, ny, nz;
    bool covered;
};

struct CudaWaypoint {
    float x, y, z;
    float qx, qy, qz, qw;
    float coverage_radius;
};

extern "C" {
    void coverageKernelLauncher(
        CudaSurfacePoint* points, size_t num_points,
        const CudaWaypoint* waypoints, size_t num_waypoints,
        float max_distance, float max_angle);
        
    void setupCUDA();
    void cleanupCUDA();
}
#pragma once

#ifdef USE_CUDA
#include <cuda_runtime.h>
#endif

#include <Eigen/Core>

struct CudaSurfacePoint {
    float x, y, z;
    float nx, ny, nz;
};

struct CudaWaypoint {
    float x, y, z;
    float qx, qy, qz, qw;
    float coverage_radius;
};

extern "C" {
        
    void detailedCoverageKernelLauncher(
        const CudaSurfacePoint* points, size_t num_points,
        const CudaWaypoint* waypoints, size_t num_waypoints,
        float max_distance,
        char* coverage_matrix, int* coverage_counts);

    void setupCUDA();
    void cleanupCUDA();
}
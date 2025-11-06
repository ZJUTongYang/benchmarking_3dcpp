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

enum ToolType
{
    CIRCULAR_TOOL = 0, 
    LINE_LIDAR_TOOL = 1
};

struct ToolParameters{
    float param1;
    float param2;
    float param3;
};

struct CoverageConfig {
    ToolType tool_type;
    ToolParameters params;
};

extern "C" {
        
    __device__ bool isCircularToolCovered(
        const CudaSurfacePoint& point, 
        const CudaWaypoint& waypoint, 
        float radius, float depth
    );

    void CoverageKernelLauncher(
        const CudaSurfacePoint* points, size_t num_points,
        const CudaWaypoint* waypoints, size_t num_waypoints,
        ToolType tool_type,
        ToolParameters params,
        char* coverage_matrix, int* coverage_counts);

    void setupCUDA();
    void cleanupCUDA();
}
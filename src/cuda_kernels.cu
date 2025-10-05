#include <benchmarking_3dcpp/cuda_kernels.cuh>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>

__device__ bool isPointCovered(
    const CudaSurfacePoint& point,
    const CudaWaypoint& waypoint,
    float max_distance, float max_angle) {
    
    // Calculate distance
    float dx = point.x - waypoint.x;
    float dy = point.y - waypoint.y;
    float dz = point.z - waypoint.z;
    float distance_sq = dx*dx + dy*dy + dz*dz;
    
    if (distance_sq > waypoint.coverage_radius * waypoint.coverage_radius) {
        return false;
    }
    
    // Calculate angle between surface normal and robot approach direction
    // Assuming robot Z-axis is approach direction
    Eigen::Vector3f normal(point.nx, point.ny, point.nz);
    Eigen::Quaternionf robot_rot(waypoint.qw, waypoint.qx, waypoint.qy, waypoint.qz);
    Eigen::Vector3f robot_approach = robot_rot * Eigen::Vector3f::UnitZ();
    
    float cos_angle = normal.dot(-robot_approach);
    float angle = acosf(fminf(fmaxf(cos_angle, -1.0f), 1.0f));
    
    return angle <= max_angle;
}

__global__ void coverageKernel(
    CudaSurfacePoint* points, size_t num_points,
    const CudaWaypoint* waypoints, size_t num_waypoints,
    float max_distance, float max_angle) {
    
    int point_idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (point_idx >= num_points) return;
    
    for (int wp_idx = 0; wp_idx < num_waypoints; ++wp_idx) {
        if (isPointCovered(points[point_idx], waypoints[wp_idx], 
                          max_distance, max_angle)) {
            points[point_idx].covered = true;
            break; // Point covered by at least one waypoint
        }
    }
}

void coverageKernelLauncher(
    CudaSurfacePoint* points, size_t num_points,
    const CudaWaypoint* waypoints, size_t num_waypoints,
    float max_distance, float max_angle) {
    
    const int block_size = 256;
    const int grid_size = (num_points + block_size - 1) / block_size;
    
    coverageKernel<<<grid_size, block_size>>>(
        points, num_points, waypoints, num_waypoints, 
        max_distance, max_angle);
    
    cudaDeviceSynchronize();
}

void setupCUDA() {
    cudaFree(0); // Initialize CUDA context
}

void cleanupCUDA() {
    // CUDA cleanup if needed
}
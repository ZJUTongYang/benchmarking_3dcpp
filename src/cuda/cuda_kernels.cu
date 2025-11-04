#include <benchmarking_3dcpp/cuda/cuda_kernels.cuh>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>

__device__ float3 quaternionRotateVector(float qx, float qy, float qz, float qw, float3 v) {
    // Optimized quaternion-vector rotation
    float uvx = qy * v.z - qz * v.y;
    float uvy = qz * v.x - qx * v.z;
    float uvz = qx * v.y - qy * v.x;
    
    float uuvx = qy * uvz - qz * uvy;
    float uuvy = qz * uvx - qx * uvz;
    float uuvz = qx * uvy - qy * uvx;
    
    float3 result;
    result.x = v.x + 2.0f * (qw * uvx + qx * uuvx);
    result.y = v.y + 2.0f * (qw * uvy + qy * uuvy);
    result.z = v.z + 2.0f * (qw * uvz + qz * uuvz);
    
    return result;
}

__device__ bool isCircularToolCovered(
    const CudaSurfacePoint& point,
    const CudaWaypoint& waypoint,
    float radius, float depth
)
{
    // 1. 计算工具在世界坐标系下的主轴方向向量
    float3 tool_z_axis = make_float3(0.0f, 0.0f, -1.0f); // 工具的-Z方向
    float3 tool_direction = quaternionRotateVector(
        waypoint.qx, waypoint.qy, waypoint.qz, waypoint.qw, tool_z_axis);

    // 2. 计算从工具中心到曲面点的向量
    float3 vec_to_point = make_float3(
        point.x - waypoint.x,
        point.y - waypoint.y,
        point.z - waypoint.z
    );

    // 3. 计算曲面点在工具轴线上的投影长度
    float projection_length = 
        vec_to_point.x * tool_direction.x +
        vec_to_point.y * tool_direction.y +
        vec_to_point.z * tool_direction.z;

    // 4. 检查投影点是否在工具的有效长度范围内 [0, depth]
    if (projection_length < 0.0f || projection_length > depth) {
        return false;
    }

    // 5. 计算工具轴线上离曲面点最近的点
    float3 closest_point_on_axis = make_float3(
        waypoint.x + projection_length * tool_direction.x,
        waypoint.y + projection_length * tool_direction.y,
        waypoint.z + projection_length * tool_direction.z
    );

    // 6. 计算曲面点到工具轴线的最短距离（即径向距离）
    float dx = point.x - closest_point_on_axis.x;
    float dy = point.y - closest_point_on_axis.y;
    float dz = point.z - closest_point_on_axis.z;
    float radial_distance_sq = dx*dx + dy*dy + dz*dz;

    // 7. 检查径向距离是否在覆盖半径内
    if (radial_distance_sq > radius) {
        return false;
    }

    return true;
}

__device__ bool isPointCovered(
    const CudaSurfacePoint& point,
    const CudaWaypoint& waypoint,
    float max_distance) {
    
    // Calculate distance
    float dx = point.x - waypoint.x;
    float dy = point.y - waypoint.y;
    float dz = point.z - waypoint.z;
    float distance_sq = dx*dx + dy*dy + dz*dz;
    float max_distance_sq = max_distance * max_distance;

    if(distance_sq > max_distance_sq)
    {
        return false;
    }
    
    return true;

    // Calculate angle between surface normal and robot approach direction
    // float3 normal = make_float3(point.nx, point.ny, point.nz);
    // float3 robot_z_axis = make_float3(0.0f, 0.0f, 1.0f); // Unit Z in robot frame
    // float3 robot_approach = quaternionRotateVector(
    //     waypoint.qx, waypoint.qy, waypoint.qz, waypoint.qw, robot_z_axis);
    
    // Dot product and angle calculation
    // float cos_angle = normal.x * (-robot_approach.x) + 
    //                  normal.y * (-robot_approach.y) + 
    //                  normal.z * (-robot_approach.z);
    
    // Clamp to valid range for acos
    // cos_angle = fmaxf(fminf(cos_angle, 1.0f), -1.0f);
    // float angle = acosf(cos_angle);
    
    // return angle <= max_angle;
}

__global__ void coverageKernelDetailed(
    const CudaSurfacePoint* points, size_t num_points,
    const CudaWaypoint* waypoints, size_t num_waypoints,
    float radius, float depth,
    char* coverage_matrix) {  // two-dim matrix: points × waypoints
    
    // 计算全局索引：每个线程处理一个点-路径点对
    int global_idx = blockIdx.x * blockDim.x + threadIdx.x;
    int total_pairs = num_points * num_waypoints;
    
    if (global_idx >= total_pairs) return;
    
    // 从全局索引计算点索引和路径点索引
    int point_idx = global_idx / num_waypoints;
    int wp_idx = global_idx % num_waypoints;
    
    const CudaSurfacePoint& point = points[point_idx];
    const CudaWaypoint& waypoint = waypoints[wp_idx];
    
    // 检查是否覆盖
    bool covered = isPointCovered(point, waypoint, max_distance);
    
    // 写入结果矩阵
    coverage_matrix[point_idx * num_waypoints + wp_idx] = covered ? 1 : 0;
}

// 辅助内核：统计每个点的覆盖次数
__global__ void countCoverageKernel(
    const char* coverage_matrix, size_t num_points, size_t num_waypoints,
    int* coverage_counts) {
    
    int point_idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (point_idx >= num_points) return;
    
    int count = 0;
    for (int wp_idx = 0; wp_idx < num_waypoints; ++wp_idx) {
        if (coverage_matrix[point_idx * num_waypoints + wp_idx] == 1) {
            count++;
        }
    }
    coverage_counts[point_idx] = count;
}

// __global__ void coverageKernel(
//     CudaSurfacePoint* points, size_t num_points,
//     const CudaWaypoint* waypoints, size_t num_waypoints,
//     float max_distance, float max_angle) {
    
//     int point_idx = blockIdx.x * blockDim.x + threadIdx.x;
//     if (point_idx >= num_points) return;

//     // points[point_idx].covered = false;
//     points[point_idx].coverage_count = 0;
//     points[point_idx].first_covering_waypoint = -1;
    
//     for (int wp_idx = 0; wp_idx < num_waypoints; ++wp_idx) {
//         if (isPointCovered(points[point_idx], waypoints[wp_idx], 
//                           max_distance, max_angle)) 
//         {
//             points[point_idx].coverage_count++;

//             if (points[point_idx].first_covering_waypoint == -1) 
//             {
//                 points[point_idx].first_covering_waypoint = wp_idx;
//             }
//             // points[point_idx].covered = true;
//             // break; // Point covered by at least one waypoint
//         }
//     }
// }

void detailedCoverageKernelLauncher(
    const CudaSurfacePoint* points, size_t num_points,
    const CudaWaypoint* waypoints, size_t num_waypoints,
    float radius, float depth,
    char* coverage_matrix, int* coverage_counts) {
    
    // 启动第一个内核：计算覆盖矩阵
    int total_pairs = num_points * num_waypoints;
    const int block_size = 256;
    const int grid_size = (total_pairs + block_size - 1) / block_size;
    
    coverageKernelDetailed<<<grid_size, block_size>>>(
        points, num_points, waypoints, num_waypoints,
        radius, depth, coverage_matrix);
    
    cudaError_t err = cudaGetLastError();
    if (err != cudaSuccess) {
        fprintf(stderr, "Coverage kernel launch error: %s\n", cudaGetErrorString(err));
        return;
    }
    
    // 启动第二个内核：统计覆盖次数
    const int count_grid_size = (num_points + block_size - 1) / block_size;
    countCoverageKernel<<<count_grid_size, block_size>>>(
        coverage_matrix, num_points, num_waypoints, coverage_counts);
    
    err = cudaGetLastError();
    if (err != cudaSuccess) {
        fprintf(stderr, "Count kernel launch error: %s\n", cudaGetErrorString(err));
    }
    
    cudaDeviceSynchronize();
}

// void coverageKernelLauncher(
//     CudaSurfacePoint* points, size_t num_points,
//     const CudaWaypoint* waypoints, size_t num_waypoints,
//     float max_distance, float max_angle) {
    
//     const int block_size = 256;
//     const int grid_size = (num_points + block_size - 1) / block_size;
    
//     coverageKernel<<<grid_size, block_size>>>(
//         points, num_points, waypoints, num_waypoints, 
//         max_distance, max_angle);
    
//     // Check for kernel launch errors
//     cudaError_t err = cudaGetLastError();
//     if (err != cudaSuccess) {
//         fprintf(stderr, "Kernel launch error: %s\n", cudaGetErrorString(err));
//         return;
//     }
    

//     cudaDeviceSynchronize();
//     if (err != cudaSuccess) {
//         fprintf(stderr, "Kernel execution error: %s\n", cudaGetErrorString(err));
//     }
// }

void setupCUDA() {
    cudaFree(0); // Initialize CUDA context
}

void cleanupCUDA() {
    // CUDA cleanup if needed
}
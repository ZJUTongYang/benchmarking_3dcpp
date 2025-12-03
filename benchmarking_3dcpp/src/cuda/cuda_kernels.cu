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

__device__ CudaVec3 operator*(const CudaQuat& q, const CudaVec3& v)
{
    CudaVec3 q_vec(q.x, q.y, q.z);
    float q_w = q.w;

    CudaVec3 term1 = v;
    CudaVec3 term2 = q_vec * (2.0f * q_vec.dot(v));
    CudaVec3 term3 = q_vec.cross(v) * (2.0f * q_w);

    return term1 + term2 + term3;
}

__device__ bool isCircularToolCovered(
    const CudaSurfacePoint& point,
    const CudaWaypoint& waypoint,
    float radius, float depth
)
{
    // 计算工具在世界坐标系下的主轴方向向量
    // Compute the main axis direction of the tool in the world coordinate system
    float3 tool_z_axis = make_float3(0.0f, 0.0f, -1.0f); // 工具的-Z方向
    float3 tool_direction = quaternionRotateVector(
        waypoint.orientation.x, waypoint.orientation.y, waypoint.orientation.z, waypoint.orientation.w, tool_z_axis);

    // 计算从工具中心到曲面点的向量
    // Compute the vector from the tool center to the point of interest
    float3 vec_to_point = make_float3(
        point.position.x - waypoint.position.x,
        point.position.y - waypoint.position.y,
        point.position.z - waypoint.position.z
    );

    // 计算曲面点在工具轴线上的投影长度
    // the position of the closest point on the tool axis to the point
    float projection_length = 
        vec_to_point.x * tool_direction.x +
        vec_to_point.y * tool_direction.y +
        vec_to_point.z * tool_direction.z;

    float3 closest_point_on_axis = make_float3(
        waypoint.position.x + projection_length * tool_direction.x,
        waypoint.position.y + projection_length * tool_direction.y,
        waypoint.position.z + projection_length * tool_direction.z
    );

    // 计算曲面点到工具轴线的最短距离（即径向距离）
    // the shortest distance between the point and the tool axis
    float dx = point.position.x - closest_point_on_axis.x;
    float dy = point.position.y - closest_point_on_axis.y;
    float dz = point.position.z - closest_point_on_axis.z;
    float radial_distance_sq = dx*dx + dy*dy + dz*dz;

    // We check all if-else in the end. Maybe faster for CUDA?

    // 检查投影点是否在工具的有效长度范围内 [0, depth]
    // check the distance
    if (projection_length < 0.0f || projection_length > depth) {
        return false;
    }

    // 检查径向距离是否在覆盖半径内
    if (radial_distance_sq > radius) {
        return false;
    }

    return true;
}

__device__ bool isLineLidarToolCovered(
    const CudaSurfacePoint& point, 
    const CudaWaypoint& waypoint, 
    float max_sensing_distance, float epsilon, 
    int beam_num, float* beam_direction
)
{
    // For each line lidar sensor waypoint, we compute all the beams-point intersection in a single thread

    CudaVec3 vec_to_point = point.position - waypoint.position;

    // First we construct the beams' origin and heading direction
    for (size_t i = 0; i < beam_num; i++) 
    {
        CudaVec3 local_dir_vec(beam_direction[3*i], beam_direction[3*i+1], beam_direction[3*i+2]);
        const CudaQuat& waypoint_orientation = waypoint.orientation;

        CudaVec3 beam_direction_world = waypoint_orientation * local_dir_vec;

        float projection_length = vec_to_point.dot(beam_direction_world);

        if (projection_length < 0.0f || projection_length > max_sensing_distance)
        {
            continue;
        }

        CudaVec3 closest_point_on_beam = waypoint.position + beam_direction_world * projection_length;
        
        float distance_to_beam_axis = (point.position - closest_point_on_beam).norm();

        if (distance_to_beam_axis <= epsilon)
        {
            return true;
        }
    }

    return false;
}

__device__ bool checkCoverageByToolType(
    const CudaSurfacePoint& point, 
    const CudaWaypoint& waypoint, 
    ToolType tool_type, 
    const CudaToolParameters* params)
{
    switch (tool_type) {
        case CIRCULAR_TOOL:
            // point, waypoint, radius, depth
            return isCircularToolCovered(point, waypoint, params->param1, params->param2);
        case LINE_LIDAR_TOOL:
            // point, waypoint, max_sensing_distance, epsilon, beam_num, beam_direction
            return isLineLidarToolCovered(point, waypoint, params->param1, params->param2, 
                params->size_triple_array_param1, params->float_triple_array_param1);
        default:
            return false;
    }
}

__global__ void coverageKernelDetailed(
    const CudaSurfacePoint* points, size_t num_points,
    const CudaWaypoint* waypoints, size_t num_waypoints,
    ToolType tool_type,
    const CudaToolParameters* params,
    // float radius, float depth,
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
    bool covered = checkCoverageByToolType(point, waypoint, tool_type, params);
    
    // 写入结果矩阵
    coverage_matrix[point_idx * num_waypoints + wp_idx] = covered ? 1 : 0;
}


void CoverageKernelLauncher(
    const CudaSurfacePoint* points, size_t num_points,
    const CudaWaypoint* waypoints, size_t num_waypoints,
    ToolType tool_type,
    const CudaToolParameters* params,
    char* coverage_matrix)
{
    
    // 启动第一个内核：计算覆盖矩阵
    int total_pairs = num_points * num_waypoints;
    const int block_size = 256;
    const int grid_size = (total_pairs + block_size - 1) / block_size;
    
    coverageKernelDetailed<<<grid_size, block_size>>>(
        points, num_points, waypoints, num_waypoints,
        tool_type, params, coverage_matrix);
    
    cudaError_t err = cudaGetLastError();
    if (err != cudaSuccess) {
        fprintf(stderr, "Coverage kernel launch error: %s\n", cudaGetErrorString(err));
        return;
    }
    
    err = cudaGetLastError();
    if (err != cudaSuccess) {
        fprintf(stderr, "Count kernel launch error: %s\n", cudaGetErrorString(err));
    }
    
    cudaDeviceSynchronize();
}

void setupCUDA() {
    cudaFree(0); // Initialize CUDA context
}

void cleanupCUDA() {
    // CUDA cleanup if needed
}

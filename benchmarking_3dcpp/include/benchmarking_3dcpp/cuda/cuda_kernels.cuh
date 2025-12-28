#pragma once

#ifdef USE_CUDA
#include <cuda_runtime.h>
#endif

#include <benchmarking_3dcpp/cuda_types.hpp>
#include <Eigen/Core>

struct CudaVec3
{
    float x, y, z;

    __device__ CudaVec3() : x(0), y(0), z(0) {}

    __device__ CudaVec3(float x, float y, float z) : x(x), y(y), z(z) {}

    __device__
    CudaVec3 operator+(const CudaVec3& v) const 
    {
        return CudaVec3(x + v.x, y + v.y, z + v.z);
    }

    __device__ CudaVec3 operator-(const CudaVec3& v) const
    {
        return CudaVec3(x - v.x, y - v.y, z - v.z);
    }

    __device__ float dot(const CudaVec3& v) const 
    {
        return x * v.x + y * v.y + z * v.z;
    }

    __device__ CudaVec3 operator*(float s) const 
    {
        return CudaVec3(x * s, y * s, z * s);
    }

    __device__
    CudaVec3 cross(const CudaVec3& v) const 
    {
        return CudaVec3(y * v.z - z * v.y,
                        z * v.x - x * v.z,
                        x * v.y - y * v.x);
    }

    __device__ float norm() const
    {
        float len_sq = x * x + y * y + z * z;
        return sqrtf(len_sq);
        // return rsqrtf(len_sq);
    }
};


struct CudaQuat
{
    float x, y, z, w;
    __device__
    CudaQuat() : x(0), y(0), z(0), w(1) {}

    __device__
    CudaQuat(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) {}
};

struct CudaSurfacePoint 
{
    CudaVec3 position;
    CudaVec3 normal;
};

struct CudaWaypoint {
    CudaVec3 position;
    CudaQuat orientation;
    float coverage_radius;
};

struct CudaBeam
{
    CudaVec3 origin;
    CudaVec3 direction;
};

__device__ CudaVec3 operator*(const CudaQuat& q, const CudaVec3& v);

__device__ bool isCircularToolCovered(
    const CudaSurfacePoint& point, 
    const CudaWaypoint& waypoint, 
    float radius, float depth
);

__device__ bool isLineLidarToolCovered(
    const CudaSurfacePoint& point, 
    const CudaWaypoint& waypoint, 
    float max_sensing_distance, float epsilon,
    int beam_num, float* beam_direction);


extern "C" 
{
        
    void CoverageKernelLauncher(
        const CudaSurfacePoint* points, size_t num_points,
        const CudaWaypoint* waypoints, size_t num_waypoints,
        ToolType tool_type,
        const CudaToolParameters* params,
        char* coverage_matrix);

    void setupCUDA();
    void cleanupCUDA();
}


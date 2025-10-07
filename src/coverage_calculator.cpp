#include <benchmarking_3dcpp/coverage_calculator.hpp>
#include <benchmarking_3dcpp/surface_sampler.hpp>
#include <benchmarking_3dcpp/cuda_kernels.cuh>
#include <algorithm>
#include <execution>
#include <rclcpp/rclcpp.hpp>

CoverageCalculator::CoverageCalculator(bool use_cuda, double point_density) 
    : use_cuda_(use_cuda), sampler_(std::make_unique<SurfaceSampler>(point_density)) {
    if (use_cuda_) {
        setupCUDA();
    }
}

CoverageCalculator::~CoverageCalculator() {
    if (use_cuda_) {
        cleanupCUDA();
    }
}

CoverageResult CoverageCalculator::calculateCoverage(
    const open3d::geometry::TriangleMesh& mesh,
    const std::vector<RobotWaypoint>& path,
    double max_distance, double max_angle_rad) {
    
    // Sample surface points
    auto surface_points = sampler_->sampleUniformly(mesh);
    
    rclcpp::Time t1 = rclcpp::Clock().now();
    // Calculate coverage
    std::vector<bool> coverage_mask;
    if (use_cuda_) {
        coverage_mask = calculateCoverageCUDA(surface_points, path, 
                                            max_distance, max_angle_rad);
    } else {
        coverage_mask = calculateCoverageCPU(surface_points, path,
                                           max_distance, max_angle_rad);
    }
    rclcpp::Time t2 = rclcpp::Clock().now();
    std::cout << "Coverage evaluation time (CUDA): " << (t2 - t1).seconds() << "s" << std::endl;
    
    // Count covered points
    size_t covered_count = std::count(coverage_mask.begin(), 
                                     coverage_mask.end(), true);
    double coverage_ratio = static_cast<double>(covered_count) / 
                           surface_points.size();
    
    return {path, coverage_ratio, coverage_mask, 
            surface_points.size(), covered_count, surface_points};
}

std::vector<bool> CoverageCalculator::calculateCoverageCPU(
    const std::vector<SurfacePoint>& surface_points,
    const std::vector<RobotWaypoint>& path,
    double max_distance, double max_angle_rad) {
    
    std::vector<bool> coverage_mask(surface_points.size(), false);
    const double max_distance_sq = max_distance * max_distance;
    
    // Parallelize over surface points
    std::for_each(std::execution::par_unseq,
                 surface_points.begin(), surface_points.end(),
                 [&](const SurfacePoint& point) {
        size_t idx = &point - &surface_points[0];
        
        for (const auto& waypoint : path) {
            double distance_sq = (point.position - waypoint.position).squaredNorm();
            if (distance_sq > max_distance_sq) continue;
            
            // Check angle constraint
            Eigen::Vector3d robot_approach = 
                waypoint.orientation * Eigen::Vector3d::UnitZ();
            double cos_angle = point.normal.dot(-robot_approach);
            double angle = std::acos(std::clamp(cos_angle, -1.0, 1.0));
            
            if (angle <= max_angle_rad) {
                coverage_mask[idx] = true;
                break;
            }
        }
    });
    
    return coverage_mask;
}

std::vector<bool> CoverageCalculator::calculateCoverageCUDA(
    const std::vector<SurfacePoint>& surface_points,
    const std::vector<RobotWaypoint>& path,
    double max_distance, double max_angle_rad) {
    
    // Convert to CUDA types
    // YT: we don't need surface point normal direction
    std::vector<CudaSurfacePoint> cuda_points;
    cuda_points.reserve(surface_points.size());
    for (const auto& point : surface_points) {
        cuda_points.push_back({
            static_cast<float>(point.position.x()),
            static_cast<float>(point.position.y()),
            static_cast<float>(point.position.z()),
            static_cast<float>(point.normal.x()),
            static_cast<float>(point.normal.y()),
            static_cast<float>(point.normal.z()),
            false
        });
    }
    
    std::vector<CudaWaypoint> cuda_waypoints;
    cuda_waypoints.reserve(path.size());
    for (const auto& wp : path) {
        cuda_waypoints.push_back({
            static_cast<float>(wp.position.x()),
            static_cast<float>(wp.position.y()),
            static_cast<float>(wp.position.z()),
            static_cast<float>(wp.orientation.x()),
            static_cast<float>(wp.orientation.y()),
            static_cast<float>(wp.orientation.z()),
            static_cast<float>(wp.orientation.w()),
            static_cast<float>(wp.coverage_radius)
        });
    }
    
    // Allocate GPU memory
    CudaSurfacePoint* d_points = nullptr;
    CudaWaypoint* d_waypoints = nullptr;
    
    cudaError_t err = cudaMalloc(&d_points, cuda_points.size() * sizeof(CudaSurfacePoint));
    if (err != cudaSuccess) {
        fprintf(stderr, "cudaMalloc failed for points: %s\n", cudaGetErrorString(err));
        return std::vector<bool>(surface_points.size(), false);
    }

    err = cudaMalloc(&d_waypoints, cuda_waypoints.size() * sizeof(CudaWaypoint));
    if (err != cudaSuccess) {
        fprintf(stderr, "cudaMalloc failed for waypoints: %s\n", cudaGetErrorString(err));
        cudaFree(d_points);
        return std::vector<bool>(surface_points.size(), false);
    }
    
    // Copy data to GPU
    err = cudaMemcpy(d_points, cuda_points.data(),
              cuda_points.size() * sizeof(CudaSurfacePoint),
              cudaMemcpyHostToDevice);
    if (err != cudaSuccess) {
        fprintf(stderr, "cudaMemcpy failed for points: %s\n", cudaGetErrorString(err));
        cudaFree(d_points);
        cudaFree(d_waypoints);
        return std::vector<bool>(surface_points.size(), false);
    }

    err = cudaMemcpy(d_waypoints, cuda_waypoints.data(),
              cuda_waypoints.size() * sizeof(CudaWaypoint),
              cudaMemcpyHostToDevice);
    if (err != cudaSuccess) {
        fprintf(stderr, "cudaMemcpy failed for waypoints: %s\n", cudaGetErrorString(err));
        cudaFree(d_points);
        cudaFree(d_waypoints);
        return std::vector<bool>(surface_points.size(), false);
    }
    
    // Launch kernel
    coverageKernelLauncher(d_points, cuda_points.size(),
                          d_waypoints, cuda_waypoints.size(),
                          static_cast<float>(max_distance),
                          static_cast<float>(max_angle_rad));
    
    // Copy results back
    err = cudaMemcpy(cuda_points.data(), d_points,
              cuda_points.size() * sizeof(CudaSurfacePoint),
              cudaMemcpyDeviceToHost);
    if (err != cudaSuccess) {
        fprintf(stderr, "cudaMemcpy failed for results: %s\n", cudaGetErrorString(err));
    }
    

    
    // Convert back to bool mask
    std::vector<bool> coverage_mask(cuda_points.size());
    for (size_t i = 0; i < cuda_points.size(); ++i) {
        coverage_mask[i] = cuda_points[i].covered;
    }
    
    // Cleanup
    cudaFree(d_points);
    cudaFree(d_waypoints);
    
    return coverage_mask;
}
#include <benchmarking_3dcpp/eval/coverage_evaluator.hpp>
#include <benchmarking_3dcpp/eval/surface_sampler.hpp>
#include <benchmarking_3dcpp/cuda/cuda_kernels.cuh>
#include <algorithm>
#include <execution>
#include <rclcpp/rclcpp.hpp>

CoverageEvaluator::CoverageEvaluator(bool use_cuda, double point_density) 
    : use_cuda_(use_cuda), sampler_(std::make_unique<SurfaceSampler>(point_density)) {
    if (use_cuda_) {
        setupCUDA();
    }
}

CoverageEvaluator::~CoverageEvaluator() {
    if (use_cuda_) {
        cleanupCUDA();
    }
}

void CoverageEvaluator::calculateCoverage(int current_test_id)
{
    std::shared_ptr<GeometryData> surface = tasks_[current_test_id].p_surface;
    const std::vector<RobotWaypoint>& path = tasks_[current_test_id].result.robot_path;
    double max_distance = tasks_[current_test_id].robot.tool_radius;

    std::vector<SurfacePoint> surface_points;
    if(surface->getType() == GeometryType::TriangleMesh)
    {
        // Sample surface points
        const auto* mesh_data = static_cast<const TriangleMeshData*>(surface.get());

        if(!mesh_data->isValid())
        {
            tasks_[current_test_id].result = {};
            return;
        }

        surface_points = sampler_->sampleUniformly(*(mesh_data->getData()));
        // surface_points = sampler_->sampleUniformly(*mesh.getData());
    }
    else if(surface->getType() == GeometryType::PointCloud)
    {
        // We directly use the point cloud as the surface points
        std::cout << "We haven't implemented this branch" << std::endl;
        const auto* cloud_data = static_cast<const PointCloudData*>(surface.get());

        if(!cloud_data->isValid())
        {
            tasks_[current_test_id].result = {};
            return;
        }

        surface_points = sampler_->sampleUniformly(*(cloud_data->getData()));

            tasks_[current_test_id].result = {};
            return;
    }
    
    rclcpp::Time t1 = rclcpp::Clock().now();
    // Calculate coverage
    std::vector<bool> coverage_mask;
    if (use_cuda_) {
        coverage_mask = calculateCoverageCUDA(surface_points, path, 
                                            max_distance, M_PI);
    } else {
        coverage_mask = calculateCoverageCPU(surface_points, path,
                                           max_distance, M_PI);
    }
    rclcpp::Time t2 = rclcpp::Clock().now();
    std::cout << "Coverage evaluation time (CUDA): " << (t2 - t1).seconds() << "s" << std::endl;
    
    // Count covered points
    size_t covered_count = std::count(coverage_mask.begin(), 
                                     coverage_mask.end(), true);
    double coverage_ratio = static_cast<double>(covered_count) / 
                           surface_points.size();
    
    // tasks_[current_test_id].result = {path, coverage_ratio, coverage_mask, 
    //         surface_points.size(), covered_count, surface_points};
    tasks_[current_test_id].result.robot_path = path;
    tasks_[current_test_id].result.coverage_ratio = coverage_ratio;
    tasks_[current_test_id].result.coverage_mask = coverage_mask;
    tasks_[current_test_id].result.total_points = surface_points.size();
    tasks_[current_test_id].result.covered_points = covered_count;
    tasks_[current_test_id].result.surface_points = surface_points;
}

std::vector<bool> CoverageEvaluator::calculateCoverageCPU(
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

std::vector<bool> CoverageEvaluator::calculateCoverageCUDA(
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

void CoverageEvaluator::registerATest(int index, const std::string& robot_name, const std::string& scene_name, 
    const std::string& algorithm_name, const YAML::Node& config, 
    const std::unordered_map<std::string, std::shared_ptr<GeometryData> >& scenes)
    {
        Task new_task;
        new_task.task_id = index;
        new_task.robot.name = robot_name;

        bool robot_found = false;

        // 2. 遍历 "robots" 列表中的每一个机器人
        for (const auto& robot_node : config["robots"]) {
            // 3. 检查当前机器人的 "name" 是否匹配
            // robot_node["name"].as<std::string>() 会将YAML节点转换为C++字符串
            if (robot_node["name"] && robot_node["name"].as<std::string>() == robot_name) {
                // 4. 如果匹配，获取其 "radius"
                if (robot_node["radius"]) {
                    new_task.robot.tool_radius = robot_node["radius"].as<double>();
                    robot_found = true;
                } else {
                    // 找到了机器人，但它没有 radius 属性
                    throw std::runtime_error("Robot '" + robot_name + "' found, but it has no 'radius' property.");
                }
                break; // 找到后就退出循环
            }
        }

        // 5. 如果遍历完都没找到，则报错
        if (!robot_found) {
            throw std::runtime_error("Robot with name '" + robot_name + "' not found in the configuration.");
        }

        for(const auto& scene_node: config["scenes"])
        {
            if (scene_node["name"] && scene_node["name"].as<std::string>() == scene_name)
            {
                new_task.scene.name = scene_name;
                new_task.scene.frame_id = scene_node["frame_id"].as<std::string>();
            }
        }

        // Based on the scene name, we create a pointer to the surface
        new_task.p_surface = scenes.at(scene_name);

        new_task.algorithm.name = algorithm_name;

        tasks_.emplace_back(new_task);

    }
    
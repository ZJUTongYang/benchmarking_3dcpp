#include <benchmarking_3dcpp/eval/coverage_evaluator.hpp>
#include <benchmarking_3dcpp/eval/surface_sampler.hpp>
#include <algorithm>
#include <execution>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#ifdef USE_CUDA
#include <benchmarking_3dcpp/cuda/cuda_kernels.cuh>
#endif

CoverageEvaluator::CoverageEvaluator(bool use_cuda, double point_density) 
    : use_cuda_(use_cuda), sampler_(std::make_unique<SurfaceSampler>(point_density)) 
{
#ifdef USE_CUDA
    if (use_cuda_) {
        setupCUDA();
    }
#endif
}

CoverageEvaluator::~CoverageEvaluator() 
{
#ifdef USE_CUDA
    if (use_cuda_) {
        cleanupCUDA();
    }
#endif
}

int CoverageEvaluator::countContinuousCoverage(const std::vector<int>& coverage_indices) 
{
    if (coverage_indices.empty()) {
        return 0;
    }
    
    std::vector<int> sorted_indices = coverage_indices;
    std::sort(sorted_indices.begin(), sorted_indices.end());
    
    int continuous_count = 1;
    
    for (size_t i = 1; i < sorted_indices.size(); ++i) {
        if (sorted_indices[i] != sorted_indices[i-1] + 1) {
            continuous_count++;
        }
    }
    
    return continuous_count;
}

void CoverageEvaluator::eval(int current_test_id, 
    const std::vector<SurfacePoint>& surface_points)
{
    std::shared_ptr<GeometryData> surface = tasks_[current_test_id].p_surface;
    const std::vector<RobotWaypoint>& path = tasks_[current_test_id].result.robot_path;
    double max_distance = tasks_[current_test_id].robot.tool_radius;
    
    rclcpp::Time t1 = rclcpp::Clock().now();
    // Calculate coverage
    std::vector<std::vector<int>> coverage_indices;
#ifdef USE_CUDA
    std::cout << "We have CUDA" << std::endl;

    if (use_cuda_) {
        coverage_indices = calculateCoverageCUDA(surface_points, path, 
                                            max_distance);
    } else {
        coverage_indices = calculateCoverageCPU(surface_points, path,
                                           max_distance);
    }
#else
    std::cout << "We do not have CUDA" << std::endl;
    coverage_indices = calculateCoverageCPU(surface_points, path,
                                           max_distance);
#endif

    rclcpp::Time t2 = rclcpp::Clock().now();
    std::cout << "Coverage evaluation time: " << (t2 - t1).seconds() << "s" << std::endl;
    
    // Count covered points
    size_t covered_count = 0;
    std::vector<int> coverage_counts(surface_points.size(), 0);
    
    for (size_t i = 0; i < surface_points.size(); ++i) {
        coverage_counts[i] = coverage_indices[i].size();
        if (coverage_counts[i] > 0) {
            covered_count++;
        }
    }

    double coverage_ratio = static_cast<double>(covered_count) / 
                           surface_points.size();
    
    tasks_[current_test_id].result.point_covered_num.resize(coverage_indices.size());
    // We need to deduplicate the visiting of consecutive robot waypoints
    for(size_t i = 0; i < coverage_indices.size(); ++i)
    {
        tasks_[current_test_id].result.point_covered_num[i] = countContinuousCoverage(coverage_indices[i]);
    }

    tasks_[current_test_id].result.robot_path = path;
    tasks_[current_test_id].result.coverage_ratio = coverage_ratio;
    // tasks_[current_test_id].result.point_covered_by = coverage_indices;
    // tasks_[current_test_id].result.point_covered_num = coverage_counts;
    tasks_[current_test_id].result.total_points = surface_points.size();
    tasks_[current_test_id].result.covered_points = covered_count;

    // 可选：输出一些统计信息
    int max_coverage = 0;
    double avg_coverage = 0.0;
    for (const auto& count : tasks_[current_test_id].result.point_covered_num) {
        max_coverage = std::max(max_coverage, count);
        avg_coverage += count;
    }
    avg_coverage /= surface_points.size();
    
    std::cout << "Coverage statistics: " << std::endl;
    std::cout << "  Coverage ratio: " << coverage_ratio * 100 << "%" << std::endl;
    std::cout << "  Max coverage count: " << max_coverage << std::endl;
    std::cout << "  Average coverage count: " << avg_coverage << std::endl;
}


std::vector<std::vector<int> > CoverageEvaluator::calculateCoverageCPU(
    const std::vector<SurfacePoint>& surface_points,
    const std::vector<RobotWaypoint>& path,
    double max_distance) {
    
    std::vector<std::vector<int> > coverage_indices(surface_points.size());
    const double max_distance_sq = max_distance * max_distance;
    
    // Parallelize over surface points
    std::for_each(std::execution::par_unseq,
                 surface_points.begin(), surface_points.end(),
                 [&](const SurfacePoint& point) {
        size_t idx = &point - &surface_points[0];
        
        for(size_t wp_idx = 0; wp_idx < path.size(); ++wp_idx)
        {
            const auto& waypoint = path[wp_idx];
            double distance_sq = (point.position - waypoint.position).squaredNorm();
            if (distance_sq > max_distance_sq) continue;
            
            // // Check angle constraint
            // Eigen::Vector3d robot_approach = 
            //     waypoint.orientation * Eigen::Vector3d::UnitZ();
            // double cos_angle = point.normal.dot(-robot_approach);
            // double angle = std::acos(std::clamp(cos_angle, -1.0, 1.0));
            
            // if (angle <= max_angle_rad) {
            //     // coverage_mask[idx] = true;
            //     // break;
            coverage_indices[idx].emplace_back(static_cast<int>(wp_idx));
            // }
        }
    });
    
    return coverage_indices;
    // return coverage_mask;
}

#ifdef USE_CUDA
std::vector<std::vector<int>> CoverageEvaluator::calculateCoverageCUDA(
    const std::vector<SurfacePoint>& surface_points,
    const std::vector<RobotWaypoint>& path,
    double max_distance) {
    
    size_t num_points = surface_points.size();
    size_t num_waypoints = path.size();
    
    // 转换数据到CUDA格式
    std::vector<CudaSurfacePoint> cuda_points;
    cuda_points.reserve(num_points);
    for (const auto& point : surface_points) {
        cuda_points.push_back({
            static_cast<float>(point.position.x()),
            static_cast<float>(point.position.y()),
            static_cast<float>(point.position.z()),
            static_cast<float>(point.normal.x()),
            static_cast<float>(point.normal.y()),
            static_cast<float>(point.normal.z())
        });
    }
    
    std::vector<CudaWaypoint> cuda_waypoints;
    cuda_waypoints.reserve(num_waypoints);
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
    
    // 分配GPU内存
    CudaSurfacePoint* d_points = nullptr;
    CudaWaypoint* d_waypoints = nullptr;
    char* d_coverage_matrix = nullptr;
    int* d_coverage_counts = nullptr;
    
    cudaError_t err;
    bool success = true;
    
    // 分配点数据内存
    err = cudaMalloc(&d_points, num_points * sizeof(CudaSurfacePoint));
    if (err != cudaSuccess) {
        fprintf(stderr, "cudaMalloc failed for points: %s\n", cudaGetErrorString(err));
        success = false;
    }
    
    // 分配路径点数据内存
    if(success)
    {
        err = cudaMalloc(&d_waypoints, num_waypoints * sizeof(CudaWaypoint));
        if (err != cudaSuccess) {
            fprintf(stderr, "cudaMalloc failed for waypoints: %s\n", cudaGetErrorString(err));
            cudaFree(d_points);
            success = false;
        }
    }
    
    // 分配覆盖矩阵内存 (num_points × num_waypoints)
    if(success)
    {
        err = cudaMalloc(&d_coverage_matrix, num_points * num_waypoints * sizeof(char));
        if (err != cudaSuccess) {
            fprintf(stderr, "cudaMalloc failed for coverage matrix: %s\n", cudaGetErrorString(err));
            cudaFree(d_points);
            cudaFree(d_waypoints);
            success = false;
        }
    }
    
    // 分配覆盖计数内存
    err = cudaMalloc(&d_coverage_counts, num_points * sizeof(int));
    if (err != cudaSuccess) {
        fprintf(stderr, "cudaMalloc failed for coverage counts: %s\n", cudaGetErrorString(err));
        cudaFree(d_points);
        cudaFree(d_waypoints);
        cudaFree(d_coverage_matrix);
        return std::vector<std::vector<int>>(num_points);
    }
    

    // 复制数据到GPU
    if(success){
        err = cudaMemcpy(d_points, cuda_points.data(),
                        num_points * sizeof(CudaSurfacePoint),
                        cudaMemcpyHostToDevice);
        if (err != cudaSuccess) {
            fprintf(stderr, "cudaMemcpy failed for points: %s\n", cudaGetErrorString(err));
            success = false;
        }
    }
    
    if(success){
        err = cudaMemcpy(d_waypoints, cuda_waypoints.data(),
                        num_waypoints * sizeof(CudaWaypoint),
                        cudaMemcpyHostToDevice);
        if (err != cudaSuccess) {
            fprintf(stderr, "cudaMemcpy failed for waypoints: %s\n", cudaGetErrorString(err));
            success = false;
        }
    }

    std::vector<char> coverage_matrix(num_points * num_waypoints);
    if(success)
    {
        // 启动内核
        detailedCoverageKernelLauncher(
            d_points, num_points,
            d_waypoints, num_waypoints,
            static_cast<float>(max_distance),
            // static_cast<float>(max_angle_rad),
            d_coverage_matrix, d_coverage_counts);
        
        // 将覆盖矩阵复制回主机
        err = cudaMemcpy(coverage_matrix.data(), d_coverage_matrix,
                        num_points * num_waypoints * sizeof(char),
                        cudaMemcpyDeviceToHost);
        if (err != cudaSuccess) {
            fprintf(stderr, "cudaMemcpy failed for coverage matrix: %s\n", cudaGetErrorString(err));
            success = false;
        }
    }

    
    // 构建最终结果
    std::vector<std::vector<int>> coverage_indices(num_points);
    for (size_t point_idx = 0; point_idx < num_points; ++point_idx) {
        for (size_t wp_idx = 0; wp_idx < num_waypoints; ++wp_idx) {
            if (coverage_matrix[point_idx * num_waypoints + wp_idx] == 1) {
                coverage_indices[point_idx].push_back(static_cast<int>(wp_idx));
            }
        }
    }
    
    // 清理GPU内存
    if (d_points) cudaFree(d_points);
    if (d_waypoints) cudaFree(d_waypoints);
    if (d_coverage_matrix) cudaFree(d_coverage_matrix);
    if (d_coverage_counts) cudaFree(d_coverage_counts);
    
    return coverage_indices;
}
#endif

void CoverageEvaluator::registerATest(int index, const std::string& robot_name, const std::string& scene_name, 
    const std::string& algorithm_name, const YAML::Node& config, 
    const std::unordered_map<std::string, std::shared_ptr<Scene> >& scenes)
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
        new_task.p_surface = scenes.at(scene_name)->scene_object_;

        new_task.algorithm.name = algorithm_name;

        tasks_.emplace_back(new_task);
        std::cout << "We create task " << index << ", robot: " << new_task.robot.name << ", scene: " << new_task.scene.name  << ", algorithm: " << new_task.algorithm.name << std::endl;
    }
    
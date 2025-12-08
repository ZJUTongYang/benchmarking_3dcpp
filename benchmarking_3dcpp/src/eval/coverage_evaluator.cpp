#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <benchmarking_3dcpp/common_types.hpp>
#include <benchmarking_3dcpp/eval/coverage_evaluator.hpp>
#include <benchmarking_3dcpp/eval/surface_sampler.hpp>
#include <benchmarking_3dcpp/robot_model/line_lidar.hpp>
#include <benchmarking_3dcpp/robot_model/circular.hpp>
#include <execution>
#include <rclcpp/rclcpp.hpp>

#ifdef USE_CUDA
#include <benchmarking_3dcpp/cuda/cuda_kernels.cuh>
#endif

ToolType getToolTypeFromString(const std::string& tool_name) {
    if (tool_name == "circular") 
    {
        return CIRCULAR_TOOL;
    } 
    else if (tool_name == "line_lidar") 
    {
        return LINE_LIDAR_TOOL;
    } 
    else 
    {
        // 默认使用Circular
        // Cirular by default
        RCLCPP_WARN(rclcpp::get_logger("CoverageEvaluator"), 
                   "Unknown tool type: %s, using CIRCULAR_TOOL as default", tool_name.c_str());
        return CIRCULAR_TOOL;
    }
}

ToolParameters getToolParameters(std::shared_ptr<RobotModel> robot_model) 
{
    ToolParameters params;
    
    auto circular_tool = std::dynamic_pointer_cast<Circular>(robot_model);
    if (circular_tool) {
        params.param1 = static_cast<float>(circular_tool->getRadius());
        params.param2 = static_cast<float>(circular_tool->getDepth());
        return params;
    }
    
    auto line_lidar = std::dynamic_pointer_cast<LineLidar>(robot_model);
    if (line_lidar) 
    {
        params.param1 = static_cast<float>(line_lidar->getMaxDistance());
        params.param2 = static_cast<float>(line_lidar->getEpsilon());
        params.size_triple_array_param1 = static_cast<int>(line_lidar->getBeamNum());
        params.float_triple_array_param1 = new float[params.size_triple_array_param1 * 3];
        line_lidar->getBeamsTripleArray(params.float_triple_array_param1);

        return params;
    }
    
    return params;
}

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
    const std::shared_ptr<RobotModel>& p_robot_model,
    const std::vector<SurfacePoint>& surface_points)
{
    std::shared_ptr<GeometryData> surface = tasks_[current_test_id].p_surface;
    const std::vector<RobotWaypoint>& path = tasks_[current_test_id].result.robot_path;
    // double max_distance = tasks_[current_test_id].robot.tool_radius;
    
    rclcpp::Time t1 = rclcpp::Clock().now();
    // Calculate coverage
    std::vector<std::vector<int>> coverage_indices;
#ifdef USE_CUDA

    if (use_cuda_) 
    {
        std::cout << "We use CUDA" << std::endl;
        coverage_indices = calculateCoverageCUDA(surface_points, path, 
                                            p_robot_model);
    } else 
    {
        std::cout << "We use CPU" << std::endl;
        coverage_indices = calculateCoverageCPU(p_robot_model, surface_points, path);
    }
#else
    std::cout << "We use CPU" << std::endl;
    coverage_indices = calculateCoverageCPU(p_robot_model, surface_points, path);
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
    
    tasks_[current_test_id].coverage_indices = coverage_indices;

    tasks_[current_test_id].result.point_covered_num.resize(coverage_indices.size());
    // We need to deduplicate the visiting of consecutive robot waypoints
    for(size_t i = 0; i < coverage_indices.size(); ++i)
    {
        tasks_[current_test_id].result.point_covered_num[i] = countContinuousCoverage(coverage_indices[i]);
    }

    tasks_[current_test_id].result.robot_path = path;
    tasks_[current_test_id].result.coverage_ratio = coverage_ratio;
    tasks_[current_test_id].result.total_points = surface_points.size();
    tasks_[current_test_id].result.covered_points = covered_count;

    // Print some statistics
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
    const std::shared_ptr<RobotModel>& p_robot_model,
    const std::vector<SurfacePoint>& surface_points,
    const std::vector<RobotWaypoint>& path) {
    
    std::vector<std::vector<int> > coverage_indices(surface_points.size());
    
    // Parallelize over surface points
    std::for_each(std::execution::par_unseq,
                 surface_points.begin(), surface_points.end(),
                 [&](const SurfacePoint& point) {
        size_t idx = &point - &surface_points[0];
        
        for(size_t wp_idx = 0; wp_idx < path.size(); ++wp_idx)
        {
            const auto& waypoint = path[wp_idx];

            if(p_robot_model->isPointCovered(point, waypoint))
            {
                coverage_indices[idx].emplace_back(static_cast<int>(wp_idx));
            }
        }
    });
    
    return coverage_indices;
}

#ifdef USE_CUDA
std::vector<std::vector<int>> CoverageEvaluator::calculateCoverageCUDA(
    const std::vector<SurfacePoint>& surface_points,
    const std::vector<RobotWaypoint>& path,
    std::shared_ptr<RobotModel> the_tool) 
{
    ToolType tool_type = getToolTypeFromString(the_tool->getName());

    // Now this struct has array pointers, so we have to allocate memory for it
    ToolParameters h_params = getToolParameters(the_tool);

    size_t num_points = surface_points.size();
    size_t num_waypoints = path.size();
    
    // 转换数据到CUDA格式
    std::vector<CudaSurfacePoint> cuda_points;
    cuda_points.reserve(num_points);
    for (const auto& point : surface_points) 
    {
        CudaSurfacePoint temp;
        temp.position.x = static_cast<float>(point.position.x());
        temp.position.y = static_cast<float>(point.position.y());
        temp.position.z = static_cast<float>(point.position.z());
        temp.normal.x = static_cast<float>(point.normal.x());
        temp.normal.y = static_cast<float>(point.normal.y());
        temp.normal.z = static_cast<float>(point.normal.z());
        cuda_points.emplace_back(temp);
    }
    
    std::vector<CudaWaypoint> cuda_waypoints;
    cuda_waypoints.reserve(num_waypoints);
    for (const auto& wp : path) 
    {
        CudaWaypoint temp;
        temp.position.x = static_cast<float>(wp.position.x());
        temp.position.y = static_cast<float>(wp.position.y());
        temp.position.z = static_cast<float>(wp.position.z());
        temp.orientation.x = static_cast<float>(wp.orientation.x());
        temp.orientation.y = static_cast<float>(wp.orientation.y());
        temp.orientation.z = static_cast<float>(wp.orientation.z());
        temp.orientation.w = static_cast<float>(wp.orientation.w());
        temp.coverage_radius = static_cast<float>(wp.coverage_radius);
        cuda_waypoints.emplace_back(temp);
    }
    
    // Allocate GPU memory for Tool Parameters
    float* d_float_array = nullptr;
    if (h_params.size_triple_array_param1 > 0 && h_params.float_triple_array_param1 != nullptr) {
        size_t array_size = h_params.size_triple_array_param1 * 3 * sizeof(float);
        cudaError_t err = cudaMalloc(&d_float_array, array_size);
        if (err != cudaSuccess) {
            fprintf(stderr, "cudaMalloc failed for params float array: %s\n", cudaGetErrorString(err));
            return {};
        }
        err = cudaMemcpy(d_float_array, h_params.float_triple_array_param1, array_size, cudaMemcpyHostToDevice);
        if (err != cudaSuccess) {
            fprintf(stderr, "cudaMemcpy failed for params float array: %s\n", cudaGetErrorString(err));
            cudaFree(d_float_array);
            return {};
        }
    }

    CudaToolParameters d_params_struct;
    d_params_struct.param1 = h_params.param1;
    d_params_struct.param2 = h_params.param2;
    d_params_struct.param3 = h_params.param3;
    d_params_struct.size_triple_array_param1 = h_params.size_triple_array_param1;
    d_params_struct.float_triple_array_param1 = d_float_array; // 指向设备内存

    CudaToolParameters* d_params = nullptr;
    cudaError_t err = cudaMalloc(&d_params, sizeof(CudaToolParameters));
    if (err != cudaSuccess) {
        fprintf(stderr, "cudaMalloc failed for params struct: %s\n", cudaGetErrorString(err));
        if (d_float_array) cudaFree(d_float_array); // 清理已分配的数组内存
        return {};
    }
    err = cudaMemcpy(d_params, &d_params_struct, sizeof(CudaToolParameters), cudaMemcpyHostToDevice);
    if (err != cudaSuccess) {
        fprintf(stderr, "cudaMemcpy failed for params struct: %s\n", cudaGetErrorString(err));
        cudaFree(d_params);
        if (d_float_array) cudaFree(d_float_array); // 清理已分配的数组内存
        return {};
    }

    // 分配GPU内存
    CudaSurfacePoint* d_points = nullptr;
    CudaWaypoint* d_waypoints = nullptr;
    char* d_coverage_matrix = nullptr;
    
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
        CoverageKernelLauncher(
            d_points, num_points,
            d_waypoints, num_waypoints,
            tool_type, d_params,
            d_coverage_matrix);
            
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
    if (d_float_array) cudaFree(d_float_array); // 释放动态数组
    if (d_params) cudaFree(d_params);           // 释放参数结构体

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
                robot_found = true;
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
    
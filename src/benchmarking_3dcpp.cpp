#include <benchmarking_3dcpp/benchmarking_3dcpp.hpp>
#include <open3d/Open3D.h>
#include <filesystem>
#include <string>
#include <memory>
#include <iostream>
#include <benchmarking_3dcpp/eval/coverage_evaluator.hpp>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <benchmarking_3dcpp/alg/coverage_algorithm.hpp>
#include <benchmarking_3dcpp/alg/nuc.hpp>

Benchmarking3DCPP::Benchmarking3DCPP(): 
    Node("benchmarking_3dcpp_node")
{
    initialized_ = false;
    geometryLoader_ = std::make_unique<GeometryLoader>();

    this->declare_parameter("config_filename", "NOT_SET");
    config_filename_ = this->get_parameter("config_filename").as_string();
    if(config_filename_ == "NOT_SET")
    {
        RCLCPP_ERROR(this->get_logger(), "Config filename not set");
        return;
    }

    this->declare_parameter("max_angle", M_PI);

    // Subscribers
    path_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "robot_path", 10,
        std::bind(&Benchmarking3DCPP::pathCallback, this, std::placeholders::_1));
        
    // Publishers
    coverage_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "coverage_visualization", 10);

    path_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "path_visualization", 10
    );

    scene_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "scene_visualization", 10
    );
        
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&Benchmarking3DCPP::runSingleTest, this));
}

void Benchmarking3DCPP::initialize()
{
    YAML::Node config = YAML::LoadFile(config_filename_);

    test_is_running_= false; // A marker indicating that whether we should wait for the result of the "current task"
    curr_test_index_ = 0;

    // The benchmarking platform loads all scenes and only share pointers to the evaluator
    scenes_.clear();
    for(const auto& scene : config["scenes"])
    {
        std::string scene_name = scene["name"].as<std::string>();
        std::string scene_filename = scene["filename"].as<std::string>();
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("benchmarking_3dcpp");
        std::filesystem::path scene_path = std::filesystem::path(package_share_directory) / "scene" / scene_filename;
        std::cout << "scene_path: " << scene_path.string() << std::endl;
        std::shared_ptr<GeometryData> p_scene = loadGeometryFile(scene_path.string());
        if(p_scene == nullptr)
        {
            std::cout << "YT: error, we cannot skip a scene file, because they are ordered" << std::endl;
            return ;
        }
        scenes_[scene_name] = p_scene;
    }

    robots_.clear();
    for(const auto& robot : config["robots"])
    {
        std::string robot_name = robot["name"].as<std::string>();
        auto p_robot = std::make_shared<RobotModel>(robot_name);
        robots_[robot_name] = p_robot;
    }

    algorithms_.clear();
    for(const auto& algorithm : config["algorithms"])
    {
        std::string algorithm_name = algorithm["name"].as<std::string>();

        std::shared_ptr<CoverageAlgorithm> p_algorithm;
        if(algorithm_name == "Yang2023Template")
        {
            p_algorithm = std::make_shared<NUCAlgorithm>(this->shared_from_this());
        }
        else
        {
            std::cout << "YT: error, we do not support this algorithm: " << algorithm_name << std::endl;
            return ;
        }

        algorithms_[algorithm_name] = p_algorithm;
    }

    // Initialize coverage calculator
    // double point_density = this->get_parameter("point_density").as_double();
    bool use_cuda = config["use_cuda"].as<bool>();
    double point_density = config["point_density"].as<double>();
    benchmarker_ = std::make_unique<CoverageEvaluator>(use_cuda, point_density);

    int num_robots = config["robots"].size();
    int num_scenes = config["scenes"].size();
    int num_algorithms = config["algorithms"].size();
    
    scheduleAllTests(num_robots, num_scenes, num_algorithms, config);

    initialized_ = true;
}

void Benchmarking3DCPP::scheduleAllTests(int num_robots, int num_scenes, int num_algorithms, const YAML::Node& config)
{
    for(int i = 0; i < num_robots; i++)
    {
        for(int j = 0; j < num_scenes; j++)
        {
            for(int k = 0; k < num_algorithms; k++)
            {
                int task_id = i * num_scenes * num_algorithms + j * num_algorithms + k;

                std::string robot_name = config["robots"][i]["name"].as<std::string>();
                std::string scene_name = config["scenes"][j]["name"].as<std::string>();
                std::string algorithm_name = config["algorithms"][k]["name"].as<std::string>();
                benchmarker_->registerATest(task_id, robot_name, scene_name, algorithm_name, config, scenes_);
            }
        }
    }
}

void Benchmarking3DCPP::runSingleTest()
{
    if(!initialized_)
    {
        initialize();
        return ;
    }

    if(curr_test_index_ >= benchmarker_->getTaskNum())
    {
        // All tests have finished
        timer_->cancel();  
        return ;
    }

    // we perform the i-th test
    const RobotConfig& the_robot_config = benchmarker_->getTask(curr_test_index_).robot;
    const SceneConfig& the_scene_config = benchmarker_->getTask(curr_test_index_).scene;
    const std::shared_ptr<GeometryData>& p_the_surface = benchmarker_->getTask(curr_test_index_).p_surface;

    const AlgorithmConfig& the_algorithm_config = benchmarker_->getTask(curr_test_index_).algorithm;

    std::shared_ptr<CoverageAlgorithm> p_algorithm = algorithms_[the_algorithm_config.name];

    if(!test_is_running_)
    {
        p_algorithm->execute(p_the_surface);
        test_is_running_ = true;
        return;
    }
    else
    {
        // After we call "execute", the algorithm may run for a long time and terminated with a callback, so we are going to wait for the callback to finish

        if(p_algorithm->getSolution() == nullptr)
        {
            // We still need to wait a bit longer
            return ;
        }

        // We copy the result to the evaluator
        std::shared_ptr<CoverageResult> p_result = p_algorithm->getSolution();
        benchmarker_->setSolution(curr_test_index_, p_result);
        test_is_running_ = false;
        curr_test_index_++;
    }
}

void Benchmarking3DCPP::pathCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) 
{
    std::vector<RobotWaypoint> path;
    path.reserve(msg->poses.size());
    
    for (const auto& pose : msg->poses) {
        RobotWaypoint wp;
        wp.position = Eigen::Vector3d(
            pose.position.x, pose.position.y, pose.position.z);
        wp.orientation = Eigen::Quaterniond(
            pose.orientation.w, pose.orientation.x,
            pose.orientation.y, pose.orientation.z);
        wp.coverage_radius = 0.05; // Default radius
        wp.timestamp = this->now().seconds();
        path.push_back(wp);
    }
    
    // Calculate coverage    
    benchmarker_->calculateCoverage(curr_test_index_);
    
    RCLCPP_INFO(this->get_logger(), "Coverage: %.2f%% (%zu/%zu points)",
            benchmarker_->getTask(curr_test_index_).result.coverage_ratio * 100.0, 
            benchmarker_->getTask(curr_test_index_).result.covered_points, 
            benchmarker_->getTask(curr_test_index_).result.total_points);
            
    publishVisualization(benchmarker_->getTask(curr_test_index_));

    // We deal with the next test
    curr_test_index_++;
}

void Benchmarking3DCPP::publishVisualization(const Task& the_task) 
{
    const auto& result = the_task.result;

    visualization_msgs::msg::MarkerArray markers;
    
    // Create marker for covered points (green)
    visualization_msgs::msg::Marker covered_marker;
    covered_marker.header.frame_id = "world";
    covered_marker.header.stamp = this->now();
    covered_marker.ns = "coverage";
    covered_marker.id = 0;
    covered_marker.type = visualization_msgs::msg::Marker::POINTS;
    covered_marker.action = visualization_msgs::msg::Marker::ADD;
    covered_marker.pose.orientation.w = 1.0;
    covered_marker.scale.x = 0.005;
    covered_marker.scale.y = 0.005;
    covered_marker.color.g = 1.0;
    covered_marker.color.a = 1.0;
    
    // Create marker for uncovered points (red)
    visualization_msgs::msg::Marker uncovered_marker = covered_marker;
    uncovered_marker.id = 1;
    uncovered_marker.color.r = 1.0;
    uncovered_marker.color.g = 0.0;
    
    // TODO: Populate markers with actual point positions
    for(size_t i = 0; i < result.total_points; ++i)
    {
        geometry_msgs::msg::Point point;
        if(result.coverage_mask[i] == true)
        {
            point.x = result.surface_points[i].position[0];
            point.y = result.surface_points[i].position[1];
            point.z = result.surface_points[i].position[2];
            covered_marker.points.push_back(point);
        }
        else
        {
            point.x = result.surface_points[i].position[0];
            point.y = result.surface_points[i].position[1];
            point.z = result.surface_points[i].position[2];
            uncovered_marker.points.push_back(point);
        }

    }
    
    markers.markers.push_back(covered_marker);
    markers.markers.push_back(uncovered_marker);
    
    coverage_pub_->publish(markers);

    // Add robot path visualization if available
    if (!result.robot_path.empty()) {
        visualization_msgs::msg::Marker path_marker;
        path_marker.header.frame_id = "world";
        path_marker.header.stamp = this->now();
        path_marker.ns = "robot_path";
        path_marker.id = 3;
        path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        path_marker.action = visualization_msgs::msg::Marker::ADD;
        path_marker.pose.orientation.w = 1.0;
        path_marker.scale.x = 0.01;
        path_marker.color.b = 1.0;
        path_marker.color.g = 0.5;
        path_marker.color.a = 1.0;

        visualization_msgs::msg::Marker waypoint_marker;
        waypoint_marker.header.frame_id = "world";
        waypoint_marker.header.stamp = this->now();
        waypoint_marker.ns = "robot_waypoints";
        waypoint_marker.id = 4;
        waypoint_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        waypoint_marker.action = visualization_msgs::msg::Marker::ADD;
        waypoint_marker.pose.orientation.w = 1.0;
        waypoint_marker.scale.x = 0.02;
        waypoint_marker.scale.y = 0.02;
        waypoint_marker.scale.z = 0.02;
        waypoint_marker.color.r = 1.0;
        waypoint_marker.color.g = 1.0;
        waypoint_marker.color.a = 0.8;

        // for (const auto& waypoint : result.robot_path) {
        //     geometry_msgs::msg::Point path_point;
        //     path_point.x = waypoint.position.x();
        //     path_point.y = waypoint.position.y();
        //     path_point.z = waypoint.position.z();
            
        //     path_marker.points.push_back(path_point);
        //     waypoint_marker.points.push_back(path_point);
        // }
        for(size_t i = 0; i < result.robot_path.size()-1; ++i)
        {
            geometry_msgs::msg::Point path_point;
            path_point.x = result.robot_path[i].position.x();
            path_point.y = result.robot_path[i].position.y();
            path_point.z = result.robot_path[i].position.z();
            path_marker.points.push_back(path_point);

            path_point.x = result.robot_path[i+1].position.x();
            path_point.y = result.robot_path[i+1].position.y();
            path_point.z = result.robot_path[i+1].position.z();
            path_marker.points.push_back(path_point);
        }
        // We re-add the last point
        {
            geometry_msgs::msg::Point path_point;
            path_point.x = result.robot_path[result.robot_path.size()-1].position.x();
            path_point.y = result.robot_path[result.robot_path.size()-1].position.y();
            path_point.z = result.robot_path[result.robot_path.size()-1].position.z();
            path_marker.points.push_back(path_point);
        }


        // if (!path_marker.points.empty()) {
        //     markers.markers.push_back(path_marker);
        // }
        // if (!waypoint_marker.points.empty()) {
        //     markers.markers.push_back(waypoint_marker);
        // }

        path_pub_->publish(path_marker);
    }

    // We publish the triangular mesh
    // Add mesh wireframe visualization
    std::string scene_name = the_task.scene.name;
    const std::shared_ptr<GeometryData> p_the_scene = scenes_[scene_name];
    // If the scene is a mesh, we visualize it
    const auto* mesh_data = static_cast<const TriangleMeshData*>(p_the_scene.get());


    const auto& vertices = mesh_data->getData()->vertices_;
    const auto& triangles = mesh_data->getData()->triangles_;
    visualization_msgs::msg::Marker mesh_marker;
    for (const auto& triangle : triangles) {
        if (static_cast<size_t>(triangle[0]) < vertices.size() &&
            static_cast<size_t>(triangle[1]) < vertices.size() && 
            static_cast<size_t>(triangle[2]) < vertices.size()) {
            // Add three edges for each triangle
            geometry_msgs::msg::Point p1, p2, p3;
            p1.x = vertices[triangle[0]].x(); p1.y = vertices[triangle[0]].y(); p1.z = vertices[triangle[0]].z();
            p2.x = vertices[triangle[1]].x(); p2.y = vertices[triangle[1]].y(); p2.z = vertices[triangle[1]].z();
            p3.x = vertices[triangle[2]].x(); p3.y = vertices[triangle[2]].y(); p3.z = vertices[triangle[2]].z();
            
            // Edge 1-2
            mesh_marker.points.push_back(p1);
            mesh_marker.points.push_back(p2);
            // Edge 2-3
            mesh_marker.points.push_back(p2);
            mesh_marker.points.push_back(p3);
            // Edge 3-1
            mesh_marker.points.push_back(p3);
            mesh_marker.points.push_back(p1);
        }
    }
    mesh_marker.header.frame_id = "world";
    mesh_marker.header.stamp = this->now();
    mesh_marker.ns = "mesh_wireframe";
    mesh_marker.id = 2;
    mesh_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    mesh_marker.action = visualization_msgs::msg::Marker::ADD;
    mesh_marker.pose.orientation.w = 1.0;
    mesh_marker.scale.x = 0.002;  // Thin lines
    mesh_marker.color.r = 0.5;
    mesh_marker.color.g = 0.5;
    mesh_marker.color.b = 0.5;
    mesh_marker.color.a = 0.3;    // Semi-transparent
    scene_pub_->publish(mesh_marker);


}

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

    std::cout << "test 1" << std::endl;
    this->declare_parameter("config_filename", "NOT_SET");
    std::string config_filename_ = this->get_parameter("config_filename").as_string();
    if(config_filename_ == "NOT_SET")
    {
        RCLCPP_ERROR(this->get_logger(), "Config filename not set");
        return;
    }
    YAML::Node config = YAML::LoadFile(config_filename_);
    std::cout << "test 2" << std::endl;

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
    std::cout << "test 3" << std::endl;
        
    // Create client
    client_ = this->create_client<nuc_msgs::srv::GetNuc>("get_nuc");

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
    std::cout << "test 4" << std::endl;

    // p_the_scene_ = loadGeometryFile(scene_filename_);

    // if(p_the_scene_ == nullptr)
    //     return ;

    // Initialize coverage calculator
    // bool use_cuda = this->get_parameter("use_cuda").as_bool();
    // double point_density = this->get_parameter("point_density").as_double();
    bool use_cuda = config["use_cuda"].as<bool>();
    double point_density = config["point_density"].as<double>();
    benchmarker_ = std::make_unique<CoverageEvaluator>(use_cuda, point_density);
    std::cout << "test 5" << std::endl;

    int num_robots = config["robots"].size();
    int num_scenes = config["scenes"].size();
    int num_algorithms = config["algorithms"].size();
    scheduleAllTests(num_robots, num_scenes, num_algorithms, config);
    std::cout << "test 6" << std::endl;

    test_is_running_= false;
    curr_test_index_ = 0;

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&Benchmarking3DCPP::runSingleTest, this));
    std::cout << "test 7" << std::endl;
        
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
    if(!initialized_ || test_is_running_)
        return ;

    // we perform the i-th test
    const RobotConfig& the_robot_config = benchmarker_->getTask(curr_test_index_).robot;
    const SceneConfig& the_scene_config = benchmarker_->getTask(curr_test_index_).scene;
    const std::shared_ptr<GeometryData>& p_the_surface = benchmarker_->getTask(curr_test_index_).p_surface;

    const AlgorithmConfig& the_algorithm_config = benchmarker_->getTask(curr_test_index_).algorithm;

    std::shared_ptr<CoverageAlgorithm> p_algorithm;
    if(the_algorithm_config.name == "nuc")
    {
        p_algorithm = std::make_shared<NUCAlgorithm>(this->shared_from_this());
    }
    else
    {
        std::cout << "YT: error, we do not support this algorithm: " << the_algorithm_config.name << std::endl;
        return ;
    }
    benchmarker_->getTaskNonConst(curr_test_index_).result = p_algorithm->execute(p_the_surface);



// [this](){
        //     if(initialized_ && !algorithm_is_called_)
        //     {
        //         auto request = this->createNUCRequestFromMesh(*p_the_scene_, "world");
        //         // std::cout << "We check the scene: " << std::endl;
        //         // for(auto iter = request.mesh.triangles.begin(); iter != request.mesh.triangles.end(); ++iter)
        //         // {
        //         //     std::cout << iter->vertex_indices[0] << ", " << iter->vertex_indices[1] << ", " << iter->vertex_indices[2] << std::endl;
        //         // }
        //         // for(auto iter = request.mesh.vertices.begin(); iter != request.mesh.vertices.end(); ++iter)
        //         // {
        //         //     std::cout << iter->x << ", " << iter->y << ", " << iter->z << std::endl;
        //         // }
        //         auto request_ptr = std::make_shared<nuc_msgs::srv::GetNuc::Request>(request);
        //         auto result_future = this->client_->async_send_request(request_ptr, 
        //             std::bind(&Benchmarking3DCPP::nucResultCallback, this, std::placeholders::_1));
        //     }
        //     algorithm_is_called_ = true;
        // }
    // );
}

void Benchmarking3DCPP::nucResultCallback(rclcpp::Client<nuc_msgs::srv::GetNuc>::SharedFuture future)
{
    auto response = future.get();
    nav_msgs::msg::Path path = response->coverage;
    std::cout << "We got response from nuc_ros2 with size: " << path.poses.size() << std::endl;
    // for(auto iter = path.poses.begin(); iter != path.poses.end(); iter++)
    // {
    //     std::cout << iter->pose.position.x << ", " << iter->pose.position.y << std::endl;
    // }

    geometry_msgs::msg::PoseArray path_array;
    path_array.header = path.header;
    for(auto iter = path.poses.begin(); iter != path.poses.end(); iter++)
    {
        path_array.poses.push_back(iter->pose);
    }
    pathCallback(std::make_shared<geometry_msgs::msg::PoseArray>(path_array));

}


// nuc_msgs::srv::GetNuc::Request Benchmarking3DCPP::createNUCRequestFromMesh(
//     const open3d::geometry::TriangleMesh& mesh, 
//     const std::string& frame_id) {
    
//     nuc_msgs::srv::GetNuc::Request request;
    
//     // Set frame_id
//     request.frame_id = frame_id;
    
//     // Convert triangles
//     const auto& triangles = mesh.triangles_;
//     request.mesh.triangles.resize(triangles.size());
//     for (size_t i = 0; i < triangles.size(); ++i) {
//         request.mesh.triangles[i].vertex_indices[0] = triangles[i][0];
//         request.mesh.triangles[i].vertex_indices[1] = triangles[i][1];
//         request.mesh.triangles[i].vertex_indices[2] = triangles[i][2];
//     }
    
//     // Convert vertices
//     const auto& vertices = mesh.vertices_;
//     request.mesh.vertices.resize(vertices.size());
//     for (size_t i = 0; i < vertices.size(); ++i) {
//         request.mesh.vertices[i].x = vertices[i].x();
//         request.mesh.vertices[i].y = vertices[i].y();
//         request.mesh.vertices[i].z = vertices[i].z();
//     }
    
//     RCLCPP_DEBUG(this->get_logger(), "Created request with %zu vertices and %zu triangles",
//                 request.mesh.vertices.size(), request.mesh.triangles.size());
    
//     return request;
// }


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

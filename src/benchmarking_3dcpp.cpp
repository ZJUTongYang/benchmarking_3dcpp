#include <benchmarking_3dcpp/benchmarking_3dcpp.hpp>
#include <open3d/Open3D.h>
#include <filesystem>
#include <string>
#include <memory>
#include <iostream>
#include <benchmarking_3dcpp/coverage_calculator.hpp>

Benchmarking3DCPP::Benchmarking3DCPP(): 
    Node("benchmarking_3dcpp_node")
{
    initialized_ = false;
    // We set some parameters
    // wall_clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);

    this->declare_parameter("scene_filename", "NOT_SET");
    scene_filename_ = this->get_parameter("scene_filename").as_string();
    if(scene_filename_ == "NOT_SET")
    {
        RCLCPP_ERROR(this->get_logger(), "Scene filename not set");
        return;
    }

    this->declare_parameter("use_cuda", true);
    this->declare_parameter("point_density", 1000.0);
    this->declare_parameter("max_distance", 0.1);
    this->declare_parameter("max_angle", M_PI/4.0);

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
        
    // Create client
    client_ = this->create_client<nuc_msgs::srv::GetNuc>("get_nuc");


    // If the file is a stl file, we directly load it as a mesh
    std::string ext = getFileExtension(scene_filename_);

    if (ext == ".pcd") {
        std::cout << "This is a PCD file" << std::endl;
        the_scene_ = nullptr;
        // Load as point cloud
    } else if (ext == ".stl") 
    {
        // std::cout << "This is an STL file" << std::endl;
        the_scene_ = loadSTLFile(scene_filename_);
        // Load as mesh
    } else {
        std::cout << "Unknown file type: " << ext << std::endl;
        the_scene_ = nullptr;
    }

    if(the_scene_ == nullptr)
        return ;

    // Initialize coverage calculator
    bool use_cuda = this->get_parameter("use_cuda").as_bool();
    calculator_ = std::make_unique<CoverageCalculator>(use_cuda);

    algorithm_is_called_= false;

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        [this](){
            if(initialized_ && !algorithm_is_called_)
            {
                auto request = this->createNUCRequestFromMesh(*the_scene_, "world");
                std::cout << "We check the scene: " << std::endl;
                for(auto iter = request.mesh.triangles.begin(); iter != request.mesh.triangles.end(); ++iter)
                {
                    std::cout << iter->vertex_indices[0] << ", " << iter->vertex_indices[1] << ", " << iter->vertex_indices[2] << std::endl;
                }
                for(auto iter = request.mesh.vertices.begin(); iter != request.mesh.vertices.end(); ++iter)
                {
                    std::cout << iter->x << ", " << iter->y << ", " << iter->z << std::endl;
                }
                auto request_ptr = std::make_shared<nuc_msgs::srv::GetNuc::Request>(request);
                auto result_future = this->client_->async_send_request(request_ptr, 
                    std::bind(&Benchmarking3DCPP::nucResultCallback, this, std::placeholders::_1));
            }
            algorithm_is_called_ = true;
        }
    );
    initialized_ = true;
}

void Benchmarking3DCPP::nucResultCallback(rclcpp::Client<nuc_msgs::srv::GetNuc>::SharedFuture future)
{
    auto response = future.get();
    nav_msgs::msg::Path path = response->coverage;
    std::cout << "We got response from nuc_ros2 with size: " << path.poses.size() << std::endl;
    for(auto iter = path.poses.begin(); iter != path.poses.end(); iter++)
    {
        std::cout << iter->pose.position.x << ", " << iter->pose.position.y << std::endl;
    }

    geometry_msgs::msg::PoseArray path_array;
    path_array.header = path.header;
    for(auto iter = path.poses.begin(); iter != path.poses.end(); iter++)
    {
        path_array.poses.push_back(iter->pose);
    }
    pathCallback(std::make_shared<geometry_msgs::msg::PoseArray>(path_array));

}


void Benchmarking3DCPP::loadPath()
{

}

void Benchmarking3DCPP::loadScene()
{

}

void Benchmarking3DCPP::loadCoveringModel()
{
    
}


nuc_msgs::srv::GetNuc::Request Benchmarking3DCPP::createNUCRequestFromMesh(
    const open3d::geometry::TriangleMesh& mesh, 
    const std::string& frame_id) {
    
    nuc_msgs::srv::GetNuc::Request request;
    
    // Set frame_id
    request.frame_id = frame_id;
    
    // Convert triangles
    const auto& triangles = mesh.triangles_;
    request.mesh.triangles.resize(triangles.size());
    for (size_t i = 0; i < triangles.size(); ++i) {
        request.mesh.triangles[i].vertex_indices[0] = triangles[i][0];
        request.mesh.triangles[i].vertex_indices[1] = triangles[i][1];
        request.mesh.triangles[i].vertex_indices[2] = triangles[i][2];
    }
    
    // Convert vertices
    const auto& vertices = mesh.vertices_;
    request.mesh.vertices.resize(vertices.size());
    for (size_t i = 0; i < vertices.size(); ++i) {
        request.mesh.vertices[i].x = vertices[i].x();
        request.mesh.vertices[i].y = vertices[i].y();
        request.mesh.vertices[i].z = vertices[i].z();
    }
    
    RCLCPP_DEBUG(this->get_logger(), "Created request with %zu vertices and %zu triangles",
                request.mesh.vertices.size(), request.mesh.triangles.size());
    
    return request;
}


void Benchmarking3DCPP::pathCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
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
    double max_dist = this->get_parameter("max_distance").as_double();
    double max_angle = this->get_parameter("max_angle").as_double();
    
    auto result = calculator_->calculateCoverage(*the_scene_, path, max_dist, max_angle);
    
    RCLCPP_INFO(this->get_logger(), "Coverage: %.2f%% (%zu/%zu points)",
            result.coverage_ratio * 100.0, 
            result.covered_points, result.total_points);
            
    publishVisualization(result);
}

void Benchmarking3DCPP::publishVisualization(const CoverageResult& result) {
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
    const auto& vertices = the_scene_->vertices_;
    const auto& triangles = the_scene_->triangles_;
    visualization_msgs::msg::Marker mesh_marker;
    for (const auto& triangle : triangles) {
        if (triangle[0] < vertices.size() && triangle[1] < vertices.size() && triangle[2] < vertices.size()) {
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

std::string Benchmarking3DCPP::getFileExtension(const std::string& filename) {
    std::filesystem::path filePath(filename);
    return filePath.extension().string();
}

std::shared_ptr<open3d::geometry::TriangleMesh> Benchmarking3DCPP::loadSTLFile(const std::string& filename) {
    auto mesh = std::make_shared<open3d::geometry::TriangleMesh>();
    if(open3d::io::ReadTriangleMesh(filename, *mesh))
    {
        mesh->RemoveDuplicatedVertices();
        mesh->RemoveDuplicatedTriangles();
        mesh->RemoveDegenerateTriangles();
        mesh->RemoveUnreferencedVertices();
        if(!mesh->HasVertexNormals())
        {
            mesh->ComputeVertexNormals();
        }
        return mesh;
    }

    open3d::utility::LogError("Failed to read STL file: {}", filename);
    return nullptr;
}

std::shared_ptr<open3d::geometry::PointCloud> Benchmarking3DCPP::loadPCDFile(const std::string& filename) {
    auto cloud = std::make_shared<open3d::geometry::PointCloud>();
    if (open3d::io::ReadPointCloud(filename, *cloud)) {
        return cloud;
    }
    open3d::utility::LogError("Failed to read PCD file: {}", filename);
    return nullptr;
}
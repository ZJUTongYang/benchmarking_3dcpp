#include "surface_coverage/nuc_client.hpp"
#include "surface_coverage/coverage_calculator.hpp"
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

NUCClient::NUCClient() : Node("nuc_client_node") {
    // Create client
    client_ = this->create_client<nuc_msgs::srv::GetNuc>("get_nuc");
    
    // Create publishers for visualization
    path_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "coverage_path_visualization", 10);
    path_publisher_raw_ = this->create_publisher<nav_msgs::msg::Path>(
        "coverage_path", 10);
    
    RCLCPP_INFO(this->get_logger(), "NUC Client initialized");
}

bool NUCClient::requestCoveragePath(const std::string& mesh_file_path) {
    // Wait for service to be available
    if (!client_->wait_for_service(5s)) {
        RCLCPP_ERROR(this->get_logger(), "Service 'get_nuc' not available after waiting");
        return false;
    }
    
    // Load mesh
    auto mesh = loadMesh(mesh_file_path);
    if (!mesh) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load mesh: %s", mesh_file_path.c_str());
        return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "Mesh loaded successfully: %s", mesh_file_path.c_str());
    
    // Create request
    auto request = createRequestFromMesh(*mesh);
    
    // Send request
    auto future = client_->async_send_request(request);
    
    RCLCPP_INFO(this->get_logger(), "Sent coverage path request to NUC service");
    
    // Wait for result
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
        rclcpp::FutureReturnCode::SUCCESS) {
        
        auto response = future.get();
        RCLCPP_INFO(this->get_logger(), "Received coverage path with %zu waypoints", 
                   response->coverage.poses.size());
        
        // Publish visualization
        publishVisualization(response->coverage);
        
        // Also publish raw path
        path_publisher_raw_->publish(response->coverage);
        
        return true;
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to call service get_nuc");
        return false;
    }
}

std::shared_ptr<open3d::geometry::TriangleMesh> NUCClient::loadMesh(const std::string& file_path) {
    auto mesh = open3d::io::CreateMeshFromFile(file_path);
    if (!mesh) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load mesh from: %s", file_path.c_str());
        return nullptr;
    }
    
    // Ensure mesh has normals for proper processing
    if (!mesh->HasVertexNormals()) {
        mesh->ComputeVertexNormals();
    }
    
    RCLCPP_INFO(this->get_logger(), "Mesh loaded: %zu vertices, %zu triangles", 
               mesh->vertices_.size(), mesh->triangles_.size());
    
    return mesh;
}

nuc_msgs::srv::GetNuc::Request NUCClient::createRequestFromMesh(
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

void NUCClient::publishVisualization(const nav_msgs::msg::Path& path) {
    visualization_msgs::msg::MarkerArray markers;
    
    // Path line marker
    visualization_msgs::msg::Marker path_line;
    path_line.header = path.header;
    path_line.ns = "coverage_path";
    path_line.id = 0;
    path_line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    path_line.action = visualization_msgs::msg::Marker::ADD;
    path_line.pose.orientation.w = 1.0;
    path_line.scale.x = 0.01; // Line width
    path_line.color.r = 0.0;
    path_line.color.g = 1.0;
    path_line.color.b = 0.0;
    path_line.color.a = 1.0;
    
    // Waypoint markers
    visualization_msgs::msg::Marker waypoints;
    waypoints.header = path.header;
    waypoints.ns = "coverage_waypoints";
    waypoints.id = 1;
    waypoints.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    waypoints.action = visualization_msgs::msg::Marker::ADD;
    waypoints.pose.orientation.w = 1.0;
    waypoints.scale.x = 0.02;
    waypoints.scale.y = 0.02;
    waypoints.scale.z = 0.02;
    waypoints.color.r = 1.0;
    waypoints.color.g = 0.0;
    waypoints.color.b = 0.0;
    waypoints.color.a = 1.0;
    
    // Add points to markers
    for (const auto& pose_stamped : path.poses) {
        // Add to path line
        path_line.points.push_back(pose_stamped.pose.position);
        
        // Add to waypoints
        waypoints.points.push_back(pose_stamped.pose.position);
    }
    
    markers.markers.push_back(path_line);
    markers.markers.push_back(waypoints);
    
    path_publisher_->publish(markers);
    RCLCPP_INFO(this->get_logger(), "Published path visualization with %zu waypoints", 
               path.poses.size());
}
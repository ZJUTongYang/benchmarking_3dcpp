#pragma once
#include <rclcpp/rclcpp.hpp>
#include <nuc_msgs/srv/get_nuc.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <open3d/io/TriangleMeshIO.h>
#include <memory>
#include <string>

class NUCClient : public rclcpp::Node {
public:
    NUCClient();
    
    bool requestCoveragePath(const std::string& mesh_file_path);
    void publishVisualization(const nav_msgs::msg::Path& path);
    
private:
    rclcpp::Client<nuc_msgs::srv::GetNuc>::SharedPtr client_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr path_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_raw_;
    
    std::shared_ptr<open3d::geometry::TriangleMesh> loadMesh(const std::string& file_path);
    nuc_msgs::srv::GetNuc::Request createRequestFromMesh(
        const open3d::geometry::TriangleMesh& mesh, 
        const std::string& frame_id = "world");
};
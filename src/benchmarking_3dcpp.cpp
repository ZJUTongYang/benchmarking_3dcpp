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
    
    markers.markers.push_back(covered_marker);
    markers.markers.push_back(uncovered_marker);
    
    coverage_pub_->publish(markers);
}

std::string Benchmarking3DCPP::getFileExtension(const std::string& filename) {
    std::filesystem::path filePath(filename);
    return filePath.extension().string();
}

std::shared_ptr<open3d::geometry::TriangleMesh> Benchmarking3DCPP::loadSTLFile(const std::string& filename) {
    auto mesh = std::make_shared<open3d::geometry::TriangleMesh>();
    if(open3d::io::ReadTriangleMesh(filename, *mesh))
    {
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
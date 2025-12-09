#pragma once
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <open3d/io/TriangleMeshIO.h>
#include <memory>
#include <string>
#include <benchmarking_3dcpp/alg/coverage_algorithm.hpp>
#include <benchmarking_3dcpp/types.hpp>

#include <benchmarking_3dcpp_interfaces/srv/get_nuc.hpp>

class Yang2023Template : public CoverageAlgorithm 
{
public:
    explicit Yang2023Template(rclcpp::Node::SharedPtr node);

    std::string getName() const override { return "Yang2023Template";}

    std::string getDescription() const override { return "Input: a uniform mesh. Output: a coverage skeleton";}

    std::vector<GeometryType> getSupportedInputTypes() const override { return {GeometryType::TriangleMesh};}

    void execute(std::shared_ptr<GeometryData> input) override;
    
private:
    rclcpp::Node::SharedPtr node_;
    
    void resultCallback(rclcpp::Client<benchmarking_3dcpp_interfaces::srv::GetNuc>::SharedFuture future);

    rclcpp::Client<benchmarking_3dcpp_interfaces::srv::GetNuc>::SharedPtr client_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_raw_;
    
    std::shared_ptr<open3d::geometry::TriangleMesh> loadMesh(const std::string& file_path);
};
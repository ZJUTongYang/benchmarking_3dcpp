#pragma once

#include <rclcpp/rclcpp.hpp>
#include <open3d/Open3D.h>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <benchmarking_3dcpp/types.hpp>
#include <benchmarking_3dcpp/coverage_calculator.hpp>
#include <memory>

class Benchmarking3DCPP: public rclcpp::Node
{
public:
    Benchmarking3DCPP();

    void runBenchmarking();

private:
    void loadPath();
    void loadScene();
    void loadCoveringModel();

    // void uniformSampling();


    void pathCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
    void publishVisualization(const CoverageResult& result);

    std::shared_ptr<open3d::geometry::TriangleMesh> loadSTLFile(const std::string& filename);
    std::shared_ptr<open3d::geometry::PointCloud> loadPCDFile(const std::string& filename);
    std::string getFileExtension(const std::string& filename);

    std::shared_ptr<open3d::geometry::TriangleMesh> the_scene_;

    // For the scene
    std::string scene_filename_;

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr path_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr coverage_pub_;
    std::unique_ptr<CoverageCalculator> calculator_;


};
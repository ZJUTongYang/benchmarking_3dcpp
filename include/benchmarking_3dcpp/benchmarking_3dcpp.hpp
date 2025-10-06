#pragma once

#include <rclcpp/rclcpp.hpp>
#include <open3d/Open3D.h>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <benchmarking_3dcpp/types.hpp>
#include <benchmarking_3dcpp/coverage_calculator.hpp>
#include <memory>
#include <nuc_msgs/srv/get_nuc.hpp>

class Benchmarking3DCPP: public rclcpp::Node
{
public:
    Benchmarking3DCPP();

    void runBenchmarking();


    nuc_msgs::srv::GetNuc::Request createNUCRequestFromMesh(
        const open3d::geometry::TriangleMesh& mesh, 
        const std::string& frame_id);
    rclcpp::Client<nuc_msgs::srv::GetNuc>::SharedPtr client_;

private:
    void loadPath();
    void loadScene();
    void loadCoveringModel();

    // void uniformSampling();


    void pathCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
    void publishVisualization(const CoverageResult& result);

    void nucResultCallback(rclcpp::Client<nuc_msgs::srv::GetNuc>::SharedFuture future);


    std::shared_ptr<open3d::geometry::TriangleMesh> loadSTLFile(const std::string& filename);
    std::shared_ptr<open3d::geometry::PointCloud> loadPCDFile(const std::string& filename);
    std::string getFileExtension(const std::string& filename);

    std::shared_ptr<open3d::geometry::TriangleMesh> the_scene_;

    // For the scene
    std::string scene_filename_;

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr path_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr coverage_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr scene_pub_;
    std::unique_ptr<CoverageCalculator> calculator_;

    bool algorithm_is_called_;
    bool initialized_;

    // rclcpp::Clock::SharedPtr wall_clock_;
    rclcpp::TimerBase::SharedPtr timer_;


};
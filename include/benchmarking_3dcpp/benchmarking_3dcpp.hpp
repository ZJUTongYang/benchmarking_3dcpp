#pragma once

#include <rclcpp/rclcpp.hpp>
#include <open3d/Open3D.h>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <benchmarking_3dcpp/types.hpp>
#include <benchmarking_3dcpp/eval/coverage_evaluator.hpp>
#include <memory>
#include <nuc_msgs/srv/get_nuc.hpp>
#include <optional>
#include <benchmarking_3dcpp/input_types.hpp>
#include <yaml-cpp/yaml.h>

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
    void loadCoveringModel();

    void pathCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
    void publishVisualization(const Task& result);

    void nucResultCallback(rclcpp::Client<nuc_msgs::srv::GetNuc>::SharedFuture future);

    void scheduleAllTests(int num_robots, int num_scenes, int num_algorithms, const YAML::Node& config);
    void runSingleTest();

    std::shared_ptr<GeometryData> loadGeometryFile(const std::string& filename)
    {
        return geometryLoader_->load(filename);
    }

    // For the scene
    // std::unique_ptr<GeometryData> p_the_scene_;
    // std::vector<GeometryData> scenes_;
    // std::string scene_filename_;
    std::unordered_map<std::string, std::shared_ptr<GeometryData> > scenes_;

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr path_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr coverage_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr scene_pub_;


    std::unique_ptr<CoverageEvaluator> benchmarker_;

    bool test_is_running_;
    int curr_test_index_;
    bool initialized_;

    // rclcpp::Clock::SharedPtr wall_clock_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<GeometryLoader> geometryLoader_;


};
